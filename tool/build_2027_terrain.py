#!/usr/bin/env python3
"""从已核对的 CAD 网格生成 2027 地形候选分区；不替换旧两层投影掩码。"""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import struct

import numpy as np

from export_2027_map import ROOT, PROFILE, SOURCE_SHA256, digest, write_json


# Geometric categories, not rule-defined forbidden areas or traversability.
REGIONS = [
    ("unknown", "#303030"),
    ("ground_y_low", "#c4c8cc"),
    ("ground_y_high", "#90a9b8"),
    ("central_highland", "#54ba79"),
    ("trapezoid_highland", "#e5c14c"),
    ("commanding_structure", "#a37ac9"),
    ("road_structure", "#41b4b0"),
    ("resource_structure", "#e79345"),
    ("fortress_structure", "#d37aaa"),
    ("tunnel_boundary", "#4169b3"),
    ("fan_frame", "#a3683f"),
    ("other_structure", "#d45757"),
]
RAMP_MIN_DEG, RAMP_MAX_DEG = 4.0, 35.0  # Review heuristics, not official limits.


def component_region(component):
    name, path = component["name"], component["path"]
    if "战场大地面" in name:
        return 1
    if name == "中央高地_地面带丝印":
        return 3
    if name.startswith("梯形高地带丝印_"):
        return 4
    if name.startswith("制高点_"):
        return 5
    if any(part.startswith("公路带丝印_") for part in path):
        return 6
    if "资源区_带丝印" in path:
        return 7
    if name == "堡垒_新":
        return 8
    if name.startswith("中央高地_侧边隧道挡边"):
        return 9
    if "风机框" in name:
        return 10
    return 11


def node_transform(node):
    if "matrix" in node:
        return np.array(node["matrix"]).reshape((4, 4), order="F")
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    matrix = np.eye(4)
    matrix[:3, :3] = [
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ]
    matrix[:3, :3] *= np.array(node.get("scale", [1, 1, 1]))
    matrix[:3, 3] = node.get("translation", [0, 0, 0])
    return matrix


def geometry_key(points, indices):
    return (hashlib.sha256(points.astype("<f4", copy=False).tobytes()).digest(),
            hashlib.sha256(indices.astype("<u4", copy=False).tobytes()).digest())


def primitive_lookup(path, components):
    """Bind VTK actors by actual vertices/indices and world transforms, not order."""
    with path.open("rb") as stream:
        magic, version, _ = struct.unpack("<III", stream.read(12))
        if magic != 0x46546C67 or version != 2:
            raise ValueError("需要 glTF 2.0 GLB 缓存")
        count, kind = struct.unpack("<II", stream.read(8))
        if kind != 0x4E4F534A:
            raise ValueError("GLB JSON 块缺失")
        document = json.loads(stream.read(count))
        count, kind = struct.unpack("<II", stream.read(8))
        if kind != 0x004E4942:
            raise ValueError("GLB 二进制块缺失")
        binary = stream.read(count)

    def accessor(index):
        item = document["accessors"][index]
        view = document["bufferViews"][item["bufferView"]]
        dtype = np.dtype({5126: "<f4", 5125: "<u4", 5123: "<u2", 5121: "u1"}[item["componentType"]])
        width = {"SCALAR": 1, "VEC3": 3}[item["type"]]
        return np.ndarray((item["count"], width), dtype=dtype, buffer=binary,
                          offset=view.get("byteOffset", 0) + item.get("byteOffset", 0),
                          strides=(view.get("byteStride", dtype.itemsize * width), dtype.itemsize))

    lookup, leaves = defaultdict(list), []

    def visit(index, parent, path):
        node = document["nodes"][index]
        matrix = parent @ node_transform(node)
        path = path + [node.get("name", "")]
        if "mesh" in node:
            component_id = len(leaves) + 1
            leaves.append(path)
            for primitive in document["meshes"][node["mesh"]]["primitives"]:
                if primitive.get("mode", 4) != 4:
                    raise ValueError("此工具仅处理 CAD 三角网格")
                key = geometry_key(accessor(primitive["attributes"]["POSITION"]),
                                   accessor(primitive["indices"]))
                lookup[key].append((component_id, matrix))
        for child in node.get("children", []):
            visit(child, matrix, path)

    for root in document["scenes"][document.get("scene", 0)]["nodes"]:
        visit(root, np.eye(4), [])
    if leaves != [component["path"] for component in components]:
        raise ValueError("GLB 装配路径与已提交的 CAD 部件记录不同")
    return lookup


def render_components(glb_path, components, cad_manifest):
    import vtk
    from vtk.util.numpy_support import numpy_to_vtk, vtk_to_numpy

    lookup = primitive_lookup(glb_path, components)
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(2800, 1500)
    window.SetMultiSamples(0)
    importer = vtk.vtkGLTFImporter()
    importer.SetFileName(str(glb_path))
    importer.SetRenderWindow(window)
    importer.Read()
    renderer = window.GetRenderers().GetFirstRenderer()
    renderer.SetBackground(0, 0, 0)
    renderer.SetUseFXAA(False)
    actors = renderer.GetActors()
    actors.InitTraversal()
    matched = defaultdict(int)
    for _ in range(actors.GetNumberOfItems()):
        actor = actors.GetNextActor()
        mapper = actor.GetMapper()
        poly = mapper.GetInput()
        points = vtk_to_numpy(poly.GetPoints().GetData())
        indices = vtk_to_numpy(poly.GetPolys().GetConnectivityArray())
        matrix = np.array([[actor.GetMatrix().GetElement(i, j) for j in range(4)] for i in range(4)])
        matches = {index for index, expected in lookup[geometry_key(points, indices)]
                   if np.allclose(matrix, expected, rtol=0, atol=2e-6)}
        if len(matches) != 1:
            raise ValueError("VTK 几何无法唯一绑定到 CAD 部件")
        component_id = matches.pop()
        matched[component_id] += 1
        normals = vtk.vtkPolyDataNormals()
        normals.SetInputData(poly)
        normals.ComputePointNormalsOff()
        normals.ComputeCellNormalsOn()
        normals.ConsistencyOff()
        normals.SplittingOff()
        normals.Update()
        output = normals.GetOutput()
        vectors = vtk_to_numpy(output.GetCellData().GetNormals()) @ np.linalg.inv(matrix[:3, :3])
        lengths = np.linalg.norm(vectors, axis=1)
        valid = lengths > 1e-12
        slopes = np.full(len(vectors), 255, dtype=np.uint8)
        slopes[valid] = np.rint(np.degrees(np.arccos(np.clip(
            abs(vectors[valid, 1]) / lengths[valid], 0, 1))) * 2).astype(np.uint8)
        colors = np.empty((len(vectors), 3), dtype=np.uint8)
        colors[:, 0], colors[:, 1], colors[:, 2] = component_id, slopes, 255 - component_id
        output.GetCellData().SetScalars(numpy_to_vtk(colors, deep=True))
        mapper.SetInputData(output)
        mapper.SetScalarModeToUseCellData()
        mapper.SetColorModeToDirectScalars()
        mapper.ScalarVisibilityOn()
        actor.GetProperty().LightingOff()
        actor.GetProperty().SetInterpolationToFlat()
        actor.GetProperty().SetColor(1, 1, 1)
        actor.GetProperty().SetOpacity(1)
        actor.SetTexture(None)
    if len(matched) != len(components):
        raise ValueError("部分 CAD 部件未被绑定")
    xmin, xmax, ymin, ymax = cad_manifest["cad_crop_xy_mm"]
    bounds = renderer.ComputeVisiblePropBounds()
    focus = ((xmin + xmax) / 2000, (bounds[2] + bounds[3]) / 2, -(ymin + ymax) / 2000)
    camera = renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    camera.UseExplicitAspectRatioOn()
    camera.SetPosition(focus[0], bounds[3] + 50, focus[2])
    camera.SetFocalPoint(*focus)
    camera.SetViewUp(0, 0, 1)
    camera.SetParallelScale((ymax - ymin) / 2000)
    camera.SetExplicitAspectRatio((xmax - xmin) / (ymax - ymin))
    renderer.ResetCameraClippingRange()
    window.Render()
    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()
    capture.Update()
    pixels = vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(1500, 2800, 3)[::-1].copy()
    window.Finalize()
    ids = pixels[:, :, 0]
    valid = ids > 0
    if ids.max() > len(components) or np.any(ids[valid].astype(int) + pixels[:, :, 2][valid] != 255):
        raise ValueError("标签颜色被插值或色彩处理，拒绝导出掩码")
    if valid.mean() < 0.99:
        raise ValueError("CAD 地形覆盖率异常")
    angles = pixels[:, :, 1].astype(np.float32) / 2
    angles[~valid | (pixels[:, :, 1] == 255)] = np.nan
    return ids, angles, dict(matched), vtk.vtkVersion.GetVTKVersion()


def main():
    from PIL import Image
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import BoundaryNorm, ListedColormap
    from matplotlib.patches import Patch

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache", type=Path, default=ROOT / "tool/data/map2027")
    parser.add_argument("--cad", type=Path, default=ROOT / "vision/images-2027/cad-v1.0.0")
    parser.add_argument("--display", type=Path, default=ROOT / "vision/images-2027/display-v1.0.0")
    parser.add_argument("--output", type=Path, default=ROOT / "vision/images-2027/terrain-v1.0.0")
    args = parser.parse_args()
    cad = json.loads((args.cad / "manifest.json").read_text())
    geometry = json.loads((args.cache / "geometry.json").read_text())
    display = json.loads((args.display / "manifest.json").read_text())
    if (cad["profile"] != PROFILE or geometry["profile"] != PROFILE or
            geometry["source_sha256"] != SOURCE_SHA256 or cad["source_sha256"] != SOURCE_SHA256 or
            display["cad_manifest_sha256"] != digest(args.cad / "manifest.json")):
        raise ValueError("CAD 缓存、参考资产或显示图版本不一致")
    for path, expected in [(args.cad / "components.json", cad["files"]["components.json"]["sha256"]),
                           (args.cad / "top_surface.npz", cad["files"]["top_surface.npz"]["sha256"]),
                           (args.display / "topdown.png", display["files"]["topdown.png"]["sha256"])]:
        if digest(path) != expected:
            raise ValueError(f"输入资产校验值不符：{path}")
    components = json.loads((args.cad / "components.json").read_text())
    if components != geometry["components"]:
        raise ValueError("CAD 缓存部件记录与已提交版本不同")
    ids, slopes, matched, vtk_version = render_components(args.cache / "field.glb", components, cad)
    roles = np.array([0] + [component_region(component) for component in components], dtype=np.uint8)
    regions = roles[ids]
    # Split the inclined base at the CAD ridge, retaining the two ground planes.
    regions[(regions == 1) & (np.arange(1500)[:, None] < 750)] = 2
    known = (ids > 0) & np.isfinite(slopes)
    # An independent depth-gradient check catches wrong normal axes/transforms.
    with np.load(args.cad / "top_surface.npz", allow_pickle=False) as reference:
        height = reference["cad_z_mm"]
    coverage_agreement = float(np.mean((ids > 0) == np.isfinite(height)))
    same = ((ids[1:-1, 1:-1] == ids[:-2, 1:-1]) &
            (ids[1:-1, 1:-1] == ids[2:, 1:-1]) &
            (ids[1:-1, 1:-1] == ids[1:-1, :-2]) &
            (ids[1:-1, 1:-1] == ids[1:-1, 2:]))
    xmin, xmax, ymin, ymax = cad["cad_crop_xy_mm"]
    dx = (height[1:-1, 2:] - height[1:-1, :-2]) / (2 * (xmax - xmin) / 2800)
    dy = (height[2:, 1:-1] - height[:-2, 1:-1]) / (2 * (ymax - ymin) / 1500)
    gradient = np.degrees(np.arctan(np.hypot(dx, dy)))
    selected = (same & np.isfinite(gradient) & known[1:-1, 1:-1] &
                (regions[1:-1, 1:-1] >= 1) & (regions[1:-1, 1:-1] <= 8) &
                (slopes[1:-1, 1:-1] < RAMP_MAX_DEG))
    delta = np.abs(gradient[selected] - slopes[1:-1, 1:-1][selected])
    if coverage_agreement < 0.999 or len(delta) < 100000 or np.median(delta) > 0.5:
        raise ValueError("新掩码与已有 CAD 深度或坡度参考不一致")
    verification = {"cad_depth_coverage_agreement": coverage_agreement,
                    "normal_gradient_check_pixels": len(delta),
                    "normal_gradient_median_delta_deg": float(np.median(delta)),
                    "normal_gradient_p95_delta_deg": float(np.percentile(delta, 95))}
    ramp = known & (regions >= 3) & (regions <= 8) & (slopes > RAMP_MIN_DEG) & (slopes < RAMP_MAX_DEG)
    review = ~known | (regions >= 9) | (slopes >= RAMP_MAX_DEG)
    args.output.mkdir(parents=True, exist_ok=True)
    files = {}
    for prefix, values in [("regions", regions), ("ramp_candidates", ramp.astype(np.uint8) * 255),
                            ("structure_review", review.astype(np.uint8) * 255)]:
        for view, rotated in [("topdown", values), ("red_bottom", np.rot90(values)),
                              ("blue_bottom", np.rot90(values, -1))]:
            path = args.output / f"{prefix}_{view}.png"
            Image.fromarray(rotated).save(path)
            files[path.name] = {"sha256": digest(path), "size_px": list(rotated.shape[::-1])}
    np.savez_compressed(args.output / "terrain.npz", region_ids=regions, component_ids=ids,
                        slope_deg=slopes, ramp_candidates=ramp, structure_review=review,
                        cad_to_field_cm=np.array(cad["cad_to_field_cm"]),
                        field_cm_to_cad_mm=np.array(cad["field_cm_to_cad_mm"]))
    files["terrain.npz"] = {"sha256": digest(args.output / "terrain.npz")}
    component_roles = [{"component_id": index + 1,
                        "region_ids": [1, 2] if roles[index + 1] == 1 else [int(roles[index + 1])],
                        "path": item["path"], "primitive_count": matched[index + 1]}
                       for index, item in enumerate(components)]
    write_json(args.output / "component_roles.json", component_roles)
    files["component_roles.json"] = {"sha256": digest(args.output / "component_roles.json")}
    background = np.asarray(Image.open(args.display / "topdown.png"))
    colors = ListedColormap([color for _, color in REGIONS])
    norm = BoundaryNorm(np.arange(-0.5, len(REGIONS) + 0.5), len(REGIONS))
    figure, axis = plt.subplots(figsize=(16, 10))
    figure.subplots_adjust(left=0.06, right=0.98, top=0.93, bottom=0.18)
    axis.imshow(background, extent=(0, 28, 0, 15))
    alpha = np.where(regions <= 2, 0.10, 0.46)
    axis.imshow(regions, cmap=colors, norm=norm, alpha=alpha, extent=(0, 28, 0, 15))
    axis.set(xlabel="Field X (m)", ylabel="Field Y (m)",
             title="RMUC 2027 CAD terrain regions - candidates for projection review")
    axis.axhline(7.5, color="white", linestyle="--", linewidth=0.7)
    figure.legend(handles=[Patch(facecolor=color, label=f"{index}: {name}")
                           for index, (name, color) in enumerate(REGIONS)],
                  loc="lower center", ncol=4, frameon=False)
    figure.savefig(args.output / "regions_preview.png", dpi=150)
    plt.close(figure)
    figure, axis = plt.subplots(figsize=(14, 8), layout="constrained")
    axis.imshow(background, extent=(0, 28, 0, 15))
    overlay = np.zeros((*regions.shape, 4), dtype=float)
    overlay[review] = (0.85, 0.15, 0.15, 0.55)
    overlay[ramp] = (1, 0.65, 0.05, 0.68)
    axis.imshow(overlay, extent=(0, 28, 0, 15))
    axis.set(xlabel="Field X (m)", ylabel="Field Y (m)",
             title="Orange: 4-35 degree surface candidates; red: structures/steep/unknown")
    figure.savefig(args.output / "surface_review.png", dpi=150)
    plt.close(figure)
    for name in ("regions_preview.png", "surface_review.png"):
        files[name] = {"sha256": digest(args.output / name)}
    ground_planes = []
    for region_id, angle in zip((2, 1), cad["ground_slope_deg"]):
        slope = math.tan(math.radians(angle)) * (1 if region_id == 2 else -1)
        ground_planes.append({"region_id": region_id,
                              "z_from_cad_xy_mm": [0, slope, cad["ground_ridge_cad_z_mm"] -
                                                    slope * cad["ground_center_y_mm"]]})
    write_json(args.output / "manifest.json", {
        "schema": "ysu.radar.terrain_candidates.v1", "profile": "rmuc2027-v1.0.0-terrain1",
        "source_step_sha256": SOURCE_SHA256, "source_glb_sha256": digest(args.cache / "field.glb"),
        "cad_manifest_sha256": digest(args.cad / "manifest.json"),
        "display_manifest_sha256": digest(args.display / "manifest.json"),
        "vtk_version": vtk_version, "field_size_cm": cad["field_size_cm"],
        "image_coordinates": cad["image_coordinates"], "source_component_count": len(components),
        "source_primitive_count": sum(matched.values()), "valid_fraction": float(known.mean()),
        "slope_quantization_deg": 0.5,
        "encoding": {"region_png": "uint8 label IDs, not RGB palette colours",
                     "binary_png": "uint8 0 or 255", "component_ids": "0 invalid; 1-based CAD component index",
                     "slope_deg": "float32 degrees; NaN where unavailable"},
        "ground_plane_formula": "CAD Z_mm = a * X_mm + b * Y_mm + c",
        "ground_planes": ground_planes, "verification": verification,
        "ramp_review_thresholds_deg": [RAMP_MIN_DEG, RAMP_MAX_DEG],
        "regions": [{"id": index, "name": name, "color": color,
                     "pixels": int(np.sum(regions == index))}
                    for index, (name, color) in enumerate(REGIONS)],
        "ramp_candidate_pixels": int(ramp.sum()), "structure_review_pixels": int(review.sum()),
        "physical_validation": False, "compatible_with_legacy_two_layer_mask": False,
        "limitations": ["topmost geometric surface only; roofs do not imply traversable ground",
                        "tunnel boundaries are not inferred tunnel passages",
                        "camera-specific occlusion, legal forbidden areas and blind predictions are not defined",
                        "slope thresholds are project review heuristics, not official rules"],
        "files": files,
    })
    print(f"已绑定 {len(components)} 个部件、{sum(matched.values())} 个图元，"
          f"地形有效覆盖 {known.mean():.6%}，坡面候选 {ramp.sum()} 像素", flush=True)


if __name__ == "__main__":
    main()
