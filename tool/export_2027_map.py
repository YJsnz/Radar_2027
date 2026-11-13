#!/usr/bin/env python3
"""从已核对版本的 RMUC2027 STEP 导出正交参考地图；不修改比赛配置。"""
import argparse
import ctypes
import gc
import hashlib
import json
import math
from pathlib import Path
import time


ROOT = Path(__file__).resolve().parents[1]
SOURCE_SHA256 = "a5ad3ede3b7674fc98f2e73b71717216383cb2b4b5dcdbe900c2038918bea8e0"
PROFILE = "rmuc2027-v1.0.0-cad1"
SOURCE_URL = "https://bbs.robomaster.com/article/1970956?source=9"


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")


def mesh(args):
    # Rendering is a separate process: STEP's parsing graph must be released
    # before meshing, and the CAD document must be released before VTK import.
    import OCP
    from OCP.Bnd import Bnd_Box
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.GProp import GProp_GProps
    from OCP.Message import Message_ProgressRange
    from OCP.RWGltf import RWGltf_CafWriter
    from OCP.RWMesh import RWMesh_CoordinateSystem_Yup, RWMesh_CoordinateSystem_Zup
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.TCollection import TCollection_AsciiString, TCollection_ExtendedString
    from OCP.TColStd import TColStd_IndexedDataMapOfStringString
    from OCP.TDataStd import TDataStd_Name
    from OCP.TDF import TDF_Label, TDF_LabelSequence
    from OCP.TDocStd import TDocStd_Document
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopExp import TopExp_Explorer
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopoDS import TopoDS
    from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ShapeTool
    from OCP.IFSelect import IFSelect_RetDone

    if digest(args.step) != SOURCE_SHA256:
        raise ValueError("STEP 校验值与已核对的 2027 V1.0.0 不同，请先重新审查来源和场地定义")
    args.cache.mkdir(parents=True, exist_ok=True)
    started = time.monotonic()

    def log(message):
        print(f"[{time.monotonic() - started:.1f}s] {message}", flush=True)

    document = TDocStd_Document(TCollection_ExtendedString(PROFILE))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    reader.SetColorMode(True)
    reader.SetLayerMode(False)
    log("读取 STEP")
    if reader.ReadFile(str(args.step)) != IFSelect_RetDone:
        raise RuntimeError("STEP 读取失败")
    log("转换装配与颜色")
    if not reader.Transfer(document):
        raise RuntimeError("STEP 装配转换失败")
    del reader
    gc.collect()
    if hasattr(ctypes.CDLL(None), "malloc_trim"):
        ctypes.CDLL(None).malloc_trim(0)
    log("已释放 STEP 解析数据")
    tool = XCAFDoc_DocumentTool.ShapeTool_s(document.Main())
    roots = TDF_LabelSequence()
    tool.GetFreeShapes(roots)
    if roots.Length() != 1:
        raise ValueError("预期一个顶层装配")
    components, floor_faces = [], []

    def name(label):
        attribute = TDataStd_Name()
        return (attribute.Get().ToExtString()
                if label.FindAttribute(TDataStd_Name.GetID_s(), attribute) else "")

    def bounds(shape):
        box = Bnd_Box()
        BRepBndLib.AddOptimal_s(shape, box)
        return list(box.Get())

    def visit(label, parent_location, parent_path):
        location = parent_location.Multiplied(XCAFDoc_ShapeTool.GetLocation_s(label))
        target = TDF_Label()
        if XCAFDoc_ShapeTool.IsReference_s(label):
            if not XCAFDoc_ShapeTool.GetReferredShape_s(label, target):
                raise RuntimeError("装配引用无法解析")
        else:
            target = label
        instance_name = name(label) or name(target)
        path = parent_path + [instance_name]
        if XCAFDoc_ShapeTool.IsAssembly_s(target):
            children = TDF_LabelSequence()
            XCAFDoc_ShapeTool.GetComponents_s(target, children)
            for index in range(1, children.Length() + 1):
                visit(children.Value(index), location, path)
            return
        shape = XCAFDoc_ShapeTool.GetShape_s(target).Moved(location)
        components.append({
            "name": instance_name, "definition_name": name(target), "path": path,
            "bbox_mm": bounds(shape),
            "transform": [[location.Transformation().Value(i, j)
                           for j in range(1, 5)] for i in range(1, 4)],
        })
        if "战场大地面" in instance_name:
            explorer = TopExp_Explorer(shape, TopAbs_FACE)
            while explorer.More():
                face = TopoDS.Face_s(explorer.Current())
                properties = GProp_GProps()
                BRepGProp.SurfaceProperties_s(face, properties)
                if properties.Mass() > 1e6:
                    center = properties.CentreOfMass()
                    floor_faces.append({
                        "area_mm2": properties.Mass(), "bbox_mm": bounds(face),
                        "type": str(BRepAdaptor_Surface(face).GetType()),
                        "center_mm": [center.X(), center.Y(), center.Z()],
                    })
                explorer.Next()

    visit(roots.Value(1), TopLoc_Location(), [])
    floor_faces.sort(key=lambda face: -face["area_mm2"])
    if len(components) != 81 or len(floor_faces) < 3:
        raise ValueError("装配实例或地面面片与已核对版本不符")
    write_json(args.cache / "geometry.json", {
        "profile": PROFILE, "source_sha256": SOURCE_SHA256,
        "source_url": SOURCE_URL, "source_length_unit": "mm",
        "ocp_version": OCP.__version__, "linear_deflection_mm": 5.0,
        "angular_deflection_rad": 0.25, "components": components,
        "floor_faces": floor_faces,
        "gltf_coordinates": "metres, (X,Y,Z)_gltf = (X,Z,-Y)_cad / 1000",
    })
    shape = XCAFDoc_ShapeTool.GetShape_s(roots.Value(1))
    log(f"网格化 {len(components)} 个叶子实例")
    BRepMesh_IncrementalMesh(shape, 5.0, False, 0.25, True)
    writer = RWGltf_CafWriter(TCollection_AsciiString(str(args.cache / "field.glb")), True)
    # Otherwise every STEP face becomes a separate glTF primitive/VTK actor.
    writer.SetMergeFaces(True)
    converter = writer.ChangeCoordinateSystemConverter()
    converter.SetInputLengthUnit(0.001)
    converter.SetOutputLengthUnit(1.0)
    converter.SetInputCoordinateSystem(RWMesh_CoordinateSystem_Zup)
    converter.SetOutputCoordinateSystem(RWMesh_CoordinateSystem_Yup)
    if not writer.Perform(document, TColStd_IndexedDataMapOfStringString(), Message_ProgressRange()):
        raise RuntimeError("glTF 导出失败")
    log("CAD 网格缓存已保存")


def field_profile(geometry):
    import numpy as np

    floor = next(row for row in geometry["components"] if "战场大地面" in row["name"])
    box = floor["bbox_mm"]
    # Two large inclined top faces follow the larger horizontal bottom face.
    slopes = []
    for face in geometry["floor_faces"][1:3]:
        bounds = face["bbox_mm"]
        if "Plane" not in face["type"]:
            raise ValueError("地面应为平面面片")
        slopes.append(abs((bounds[5] - bounds[2]) / (bounds[4] - bounds[1])))
    angles = [math.atan(slope) for slope in slopes]
    if any(abs(math.degrees(angle) - 1.5) > 0.001 for angle in angles):
        raise ValueError("地面坡度与已核对的 1.5° 不符")
    middle_y = (box[1] + box[4]) / 2
    half_width = 7500 * math.cos(sum(angles) / 2)
    xmin, xmax, ymin, ymax = box[0], box[3], middle_y - half_width, middle_y + half_width
    if abs((xmax - xmin) - 28000) > 1:
        raise ValueError("地面长边与 28000 mm 不符")
    # The handbook's 15000 mm is measured along the two inclined ground planes.
    # This rectification is explicit; it is not an unrecorded image stretch.
    sx, sy = 2800 / (xmax - xmin), 1500 / (ymax - ymin)
    transform = np.array([[-sx, 0, sx * xmax],
                          [0, -sy, sy * ymax], [0, 0, 1]], dtype=float)
    return {"cad_crop_xy_mm": [xmin, xmax, ymin, ymax],
            "ground_slope_deg": [math.degrees(angle) for angle in angles],
            "ground_center_y_mm": middle_y,
            "ground_ridge_cad_z_mm": max(face["bbox_mm"][5] for face in geometry["floor_faces"][1:3]),
            "field_size_cm": [2800, 1500],
            "cad_to_field_cm": transform.tolist(),
            "field_cm_to_cad_mm": np.linalg.inv(transform).tolist(),
            "transform_convention": "3x3 homogeneous CAD XY mm to field XY cm",
            "height_convention": "separate CAD absolute Z in mm; not a referee Z coordinate",
            "boundary_basis": "CAD floor X edges and centreline; handbook 15000 mm along 1.5-degree ground",
            "origin_description": "red supply-side corner; X towards blue, Y towards red upper corner",
            "physical_validation": False}


def render(args):
    import numpy as np
    from PIL import Image
    import vtk
    from vtk.util.numpy_support import vtk_to_numpy

    geometry = json.loads((args.cache / "geometry.json").read_text())
    if geometry["source_sha256"] != SOURCE_SHA256 or geometry["profile"] != PROFILE:
        raise ValueError("CAD 缓存版本不匹配")
    profile = field_profile(geometry)
    xmin, xmax, ymin, ymax = profile["cad_crop_xy_mm"]
    args.output.mkdir(parents=True, exist_ok=True)
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetMultiSamples(0)
    importer = vtk.vtkGLTFImporter()
    importer.SetFileName(str(args.cache / "field.glb"))
    importer.SetRenderWindow(window)
    importer.Read()
    renderer = window.GetRenderers().GetFirstRenderer()
    if renderer is None:
        raise RuntimeError("glTF 未产生可绘制场景")
    renderer.SetBackground(0.92, 0.92, 0.92)
    actors = renderer.GetActors()
    actors.InitTraversal()
    vertex_min, vertex_max = np.full(3, np.inf), np.full(3, -np.inf)
    for _ in range(actors.GetNumberOfItems()):
        actor = actors.GetNextActor()
        actor.GetProperty().LightingOff()
        vertices = vtk_to_numpy(actor.GetMapper().GetInput().GetPoints().GetData())
        transform = np.array([[actor.GetMatrix().GetElement(i, j)
                               for j in range(4)] for i in range(4)])
        world = vertices @ transform[:3, :3].T + transform[:3, 3]
        vertex_min = np.minimum(vertex_min, world.min(axis=0))
        vertex_max = np.maximum(vertex_max, world.max(axis=0))
    if not actors.GetNumberOfItems():
        raise RuntimeError("glTF 场景为空")
    bounds = renderer.ComputeVisiblePropBounds()
    boxes = np.array([row["bbox_mm"] for row in geometry["components"]])
    cad_min, cad_max = boxes[:, :3].min(axis=0), boxes[:, 3:].max(axis=0)
    expected_bounds = np.array([cad_min[0], cad_max[0], cad_min[2], cad_max[2],
                                -cad_max[1], -cad_min[1]]) / 1000
    # Actor bounding boxes rotate the local AABB and can overestimate the real
    # geometry. Compare transformed vertices, not those conservative boxes.
    vertex_bounds = np.column_stack((vertex_min, vertex_max)).reshape(-1)
    bounds_error_mm = float(np.max(np.abs(vertex_bounds - expected_bounds)) * 1000)
    if bounds_error_mm > geometry["linear_deflection_mm"] + 1:
        raise ValueError(f"CAD 与 glTF 装配外框不一致：{bounds_error_mm}mm")
    camera = renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    camera.UseExplicitAspectRatioOn()
    eye_y = bounds[3] + 50.0
    focus = ((xmin + xmax) / 2000, (bounds[2] + bounds[3]) / 2, -(ymin + ymax) / 2000)
    camera.SetPosition(focus[0], eye_y, focus[2])
    camera.SetFocalPoint(*focus)
    matrix = np.array(profile["cad_to_field_cm"])
    inverse = np.linalg.inv(matrix)
    results = {}
    views = [
        ("topdown.png", 2800, 1500, (0, 0, 1), (ymax - ymin) / 2000, (xmax - xmin) / (ymax - ymin)),
        ("red_bottom.png", 1500, 2800, (-1, 0, 0), (xmax - xmin) / 2000, (ymax - ymin) / (xmax - xmin)),
        ("blue_bottom.png", 1500, 2800, (1, 0, 0), (xmax - xmin) / 2000, (ymax - ymin) / (xmax - xmin)),
    ]
    height = None
    for filename, width, height_px, up, scale, aspect in views:
        window.SetSize(width, height_px)
        camera.SetViewUp(*up)
        camera.SetParallelScale(scale)
        camera.SetExplicitAspectRatio(aspect)
        renderer.ResetCameraClippingRange()
        window.Render()
        error = 0.0
        for x in (0, 1400, 2800):
            for y in (0, 750, 1500):
                cad = inverse @ [x, y, 1]
                renderer.SetWorldPoint(cad[0] / 1000, focus[1], -cad[1] / 1000, 1)
                renderer.WorldToDisplay()
                pixel = renderer.GetDisplayPoint()
                expected = ((x, 1500 - y) if filename == "topdown.png" else
                            (1500 - y, 2800 - x) if filename == "red_bottom.png" else (y, x))
                error = max(error, abs(pixel[0] - expected[0]), abs(height_px - pixel[1] - expected[1]))
        if error > 0.01:
            raise ValueError(f"{filename} 坐标投影误差异常：{error}px")
        capture = vtk.vtkWindowToImageFilter()
        capture.SetInput(window)
        capture.SetInputBufferTypeToRGB()
        capture.ReadFrontBufferOff()
        capture.Update()
        pixels = vtk_to_numpy(capture.GetOutput().GetPointData().GetScalars()).reshape(height_px, width, 3)[::-1]
        Image.fromarray(pixels).save(args.output / filename)
        results[filename] = {"size_px": [width, height_px], "sha256": digest(args.output / filename),
                             "max_projection_error_px": error}
        if filename == "topdown.png":
            depth = vtk.vtkWindowToImageFilter()
            depth.SetInput(window)
            depth.SetInputBufferTypeToZBuffer()
            depth.ReadFrontBufferOff()
            depth.Update()
            z = vtk_to_numpy(depth.GetOutput().GetPointData().GetScalars()).reshape(height_px, width)[::-1]
            near, far = camera.GetClippingRange()
            height = ((eye_y - near - z * (far - near)) * 1000).astype(np.float32)
            height[z >= 1] = np.nan
        print(f"已导出 {filename}，坐标检查最大误差 {error:.6g}px", flush=True)
    window.Finalize()
    valid_fraction = float(np.isfinite(height).mean())
    if valid_fraction < 0.99:
        raise ValueError(f"有效场地表面覆盖率异常：{valid_fraction}")
    # Image rows increase towards CAD +Y. The handbook gives the ground width
    # along the slope; the height reference is the two measured CAD planes.
    cad_y = ymin + (np.arange(1500) + 0.5) / 1500 * (ymax - ymin)
    ground_z = (profile["ground_ridge_cad_z_mm"] -
                abs(cad_y - profile["ground_center_y_mm"]) *
                math.tan(math.radians(sum(profile["ground_slope_deg"]) / 2)))
    above_ground = (height - ground_z[:, None]).astype(np.float32)
    np.savez_compressed(args.output / "top_surface.npz", cad_z_mm=height,
                        above_ground_mm=above_ground,
                        cad_to_field_cm=matrix, field_cm_to_cad_mm=inverse)
    results["top_surface.npz"] = {"sha256": digest(args.output / "top_surface.npz"),
                                  "valid_fraction": valid_fraction,
                                  "meaning": "topmost CAD surface; roofs are not robot traversable ground"}
    write_json(args.output / "components.json", geometry["components"])
    results["components.json"] = {"sha256": digest(args.output / "components.json"), "leaf_instances": 81}
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    figure, axis = plt.subplots(figsize=(14, 8), layout="constrained")
    plot = axis.imshow(np.ma.masked_invalid(above_ground), extent=(0, 28, 0, 15),
                       origin="upper", vmin=0, vmax=1200, cmap="terrain")
    axis.set(xlabel="Field X (m)", ylabel="Field Y (m)",
             title="RMUC 2027 CAD top surface above inclined ground (roofs included)")
    axis.set_xticks(np.arange(0, 29, 2))
    axis.set_yticks(np.arange(0, 16, 1))
    axis.grid(alpha=0.25)
    figure.colorbar(plot, ax=axis, label="Height (mm), colour scale capped at 1200")
    figure.savefig(args.output / "height_preview.png", dpi=160)
    plt.close(figure)
    results["height_preview.png"] = {"sha256": digest(args.output / "height_preview.png")}
    write_json(args.output / "manifest.json", {
        "schema": "ysu.radar.map_asset.v1", "profile": PROFILE,
        "source_url": SOURCE_URL, "source_sha256": SOURCE_SHA256,
        "source_filename": "UTF-8__RMUC2027_V1.0.0.stp",
        "ocp_version": geometry["ocp_version"], "vtk_version": vtk.vtkVersion.GetVTKVersion(),
        "linear_deflection_mm": geometry["linear_deflection_mm"],
        "cad_gltf_bounds_max_error_mm": bounds_error_mm,
        **profile, "pixel_convention": "edge coordinates; pixel centres are index + 0.5",
        "image_coordinates": {"topdown.png": ["X_cm", "1500 - Y_cm"],
                              "red_bottom.png": ["1500 - Y_cm", "2800 - X_cm"],
                              "blue_bottom.png": ["Y_cm", "X_cm"]},
        "limitations": ["reference assets, not camera calibration",
                        "no legacy two-layer projection mask is generated",
                        "CAD colours are retained; standalone curves/textures may not appear"],
        "files": results,
    })


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("mesh", "render"))
    parser.add_argument("--step", type=Path, default=ROOT / "docs/UTF-8__RMUC2027_V1.0.0.stp")
    parser.add_argument("--cache", type=Path, default=ROOT / "tool/data/map2027")
    parser.add_argument("--output", type=Path, default=ROOT / "vision/images-2027/cad-v1.0.0")
    args = parser.parse_args()
    (mesh if args.operation == "mesh" else render)(args)


if __name__ == "__main__":
    main()
