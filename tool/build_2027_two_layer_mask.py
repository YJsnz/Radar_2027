#!/usr/bin/env python3
"""制作与 mask26 选层方式一致的黑绿 2027 掩码，不修改比赛配置。"""
import json

import cv2
import numpy as np
from PIL import Image

from export_2027_map import ROOT, SOURCE_SHA256, digest, write_json


def main():
    terrain = ROOT / "vision/images-2027/terrain-v1.0.0"
    cad = ROOT / "vision/images-2027/cad-v1.0.0"
    display = ROOT / "vision/images-2027/display-v1.0.0"
    output = ROOT / "vision/images-2027/two-layer-v1.0.0"
    terrain_manifest = json.loads((terrain / "manifest.json").read_text())
    display_manifest = json.loads((display / "manifest.json").read_text())
    # Bind this hand-reviewed selection to its source geometry and display map.
    if (terrain_manifest["source_step_sha256"] != SOURCE_SHA256
            or terrain_manifest["profile"] != "rmuc2027-v1.0.0-terrain1"
            or terrain_manifest["display_manifest_sha256"] != digest(display / "manifest.json")):
        raise ValueError("地形来源版本不一致")
    labels_path = terrain / "regions_topdown.png"
    if digest(labels_path) != terrain_manifest["files"][labels_path.name]["sha256"]:
        raise ValueError("区域标签校验失败")
    for name in ("topdown.png", "red_bottom.png", "blue_bottom.png"):
        if digest(display / name) != display_manifest["files"][name]["sha256"]:
            raise ValueError("地胶图校验失败")
    labels = np.asarray(Image.open(labels_path))
    if labels.shape != (1500, 2800):
        raise ValueError("区域标签尺寸不一致")
    cad_manifest = json.loads((cad / "manifest.json").read_text())
    if digest(cad / "manifest.json") != terrain_manifest["cad_manifest_sha256"]:
        raise ValueError("CAD 高度来源版本不一致")
    for folder, name, manifest in ((cad, "top_surface.npz", cad_manifest),
                                   (terrain, "terrain.npz", terrain_manifest)):
        if digest(folder / name) != manifest["files"][name]["sha256"]:
            raise ValueError("高度或坡度资产校验失败")
    with np.load(cad / "top_surface.npz") as data:
        above_ground = data["above_ground_mm"]
    with np.load(terrain / "terrain.npz") as data:
        slopes = data["slope_deg"]

    # Use black for base ground and green for elevated terrain footprints.
    # The 2027 road platform is about 200 mm above ground: the legacy
    # "ground/road" comment does not describe this season's road geometry.
    # Fill enclosed object holes so a base/flag standing on highland cannot
    # accidentally select the ground homography. Keep the external boundaries.
    # Region 9 contains the central highland's two stair/edge bands. Merge
    # their top-view footprints with the highland, as in the mask26 treatment.
    highland = np.isin(labels, (3, 4, 5, 6, 9)).astype(np.uint8) * 255
    contours, _ = cv2.findContours(highland, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    filled = np.zeros_like(highland)
    cv2.drawContours(filled, contours, -1, 255, cv2.FILLED)
    # Enclosed flag/base footprints may stay high, but exposed tunnel-mouth
    # ground must select M_ground even when enclosed by the terrain contours.
    exposed_ground = np.isin(labels, (1, 2))
    # The floor assembly also contains two thin strips on the road ramps.
    # Protect those using surface height/normals instead of the part name.
    # These tolerances are project heuristics, not official height limits.
    raised_ramp = exposed_ground & (above_ground > 5.0) & (slopes > 4.0) & (slopes < 35.0)
    protected_ramp_pixels = int(np.count_nonzero((filled != 0) & raised_ramp))
    exposed_ground &= ~raised_ramp
    reopened_ground_pixels = int(np.count_nonzero((filled != 0) & exposed_ground))
    unknown_pixels_cleared = int(np.count_nonzero((filled != 0) & (labels == 0)))
    filled[exposed_ground | (labels == 0)] = 0
    mask = np.zeros((*labels.shape, 3), dtype=np.uint8)
    mask[..., 1] = filled  # RGB and OpenCV BGR share the same green encoding.
    masks = {
        "map_mask27_topdown.png": mask,
        "map_mask27_red.png": np.rot90(mask, 1),
        "map_mask27.png": np.rot90(mask, -1),  # Like make_mask.py's blue.jpg.
    }
    output.mkdir(parents=True, exist_ok=True)
    for name, pixels in masks.items():
        Image.fromarray(pixels).save(output / name)

    # The overlay is only for human review; the consumer must load the plain mask.
    blue = np.asarray(Image.open(display / "blue_bottom.png").convert("RGB"))
    selected = masks["map_mask27.png"][..., 1] != 0
    overlay = blue.copy()
    overlay[selected] = np.rint(blue[selected] * .55 + [0, 255 * .45, 0]).astype(np.uint8)
    Image.fromarray(overlay).save(output / "overlay_blue.png")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(10, 7), layout="constrained")
    old_mask = ROOT / "vision/images-2026/map_mask26.jpg"
    for ax, pixels, title in zip(axes,
            [Image.open(old_mask), masks["map_mask27.png"], overlay],
            ["2026 mask26", "2027 mask27", "2027 on textured map"]):
        ax.imshow(pixels, interpolation="nearest")
        ax.set_title(title)
        ax.axis("off")
    fig.suptitle("Black: M_ground (base ground)    Green: M_height_r (elevated terrain)")
    fig.savefig(output / "comparison.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(10, 5), layout="constrained")
    for row, (start, end, side) in enumerate(((850, 1125, "Upper"), (1675, 1950, "Lower"))):
        for col, (pixels, title) in enumerate(((blue, "map"), (overlay, "high layer"))):
            axes[row, col].imshow(pixels[start:end, 350:1150], interpolation="nearest")
            axes[row, col].set_title(f"{side} stair band - {title}")
            axes[row, col].axis("off")
    fig.savefig(output / "stairs_detail.png", dpi=150)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(8, 6), layout="constrained")
    for row, (start, end, side) in enumerate(((70, 300, "Left"), (1200, 1430, "Right"))):
        for col, (pixels, title) in enumerate(((blue, "map"), (overlay, "layer overlay"))):
            axes[row, col].imshow(pixels[1250:1525, start:end], interpolation="nearest")
            axes[row, col].set_title(f"{side} tunnel mouth - {title}")
            axes[row, col].axis("off")
    fig.savefig(output / "tunnel_detail.png", dpi=150)
    plt.close(fig)

    files = {name: {"sha256": digest(output / name),
                    "size_px": list(Image.open(output / name).size)}
             for name in [*masks, "overlay_blue.png", "comparison.png", "stairs_detail.png", "tunnel_detail.png"]}
    write_json(output / "manifest.json", {
        "schema": "ysu.radar.two_layer_mask.v1",
        "profile": "rmuc2027-v1.0.0-two-layer5",
        "source_sha256": SOURCE_SHA256,
        "terrain_manifest_sha256": digest(terrain / "manifest.json"),
        "display_manifest_sha256": digest(display / "manifest.json"),
        "mask26_reference_sha256": digest(old_mask),
        "encoding_rgb": {"M_ground": [0, 0, 0], "M_height_r": [0, 255, 0]},
        "highland_region_ids": [3, 4, 5, 6, 9],
        "highland_regions": ["central_highland", "trapezoid_highland", "commanding_structure", "road_structure", "tunnel_boundary"],
        "road_assignment_basis": "CAD road deck about 200 mm above local base ground; include ramps in second-layer footprint",
        "stair_assignment_basis": "merge both central highland stair/edge bands (region 9) into the second-layer footprint, following mask26",
        "boundary_policy": "union of visible highland contours; enclosed object holes filled; clear exposed base ground and unknown, preserving raised ramp strips by height/normals",
        "other_regions": "base ground black except confirmed raised ramp strips inside terrain contours; unknown always black",
        "height_source_sha256": digest(cad / "top_surface.npz"),
        "slope_source_sha256": digest(terrain / "terrain.npz"),
        "ramp_protection_heuristics": {"min_above_ground_mm": 5.0, "slope_range_deg_exclusive": [4.0, 35.0]},
        "protected_ramp_pixels": protected_ramp_pixels,
        "reopened_ground_pixels": reopened_ground_pixels,
        "unknown_pixels_cleared": unknown_pixels_cleared,
        "orientations": {"map_mask27.png": "blue_bottom",
                         "map_mask27_red.png": "red_bottom",
                         "map_mask27_topdown.png": "topdown"},
        "highland_pixels": int(np.count_nonzero(filled)),
        "enclosed_pixels_filled": int(np.count_nonzero(filled) - np.count_nonzero(highland)),
        "format_compatible_with_legacy_two_layer_mask": True,
        "physical_validation": False,
        "limitations": ["Two homographies approximate different elevations and ramps.",
                        "Stair/edge top-view footprints do not resolve lower tunnel passages.",
                        "Requires 2027 camera-to-map calibration; not 2026 matrices.",
                        "This is a layer selector, not an occlusion or forbidden-area mask.",
                        "Default configuration is not switched."],
        "files": files,
    })
    print(f"已生成黑绿两层掩码：{output}")
    print(f"高地像素：{np.count_nonzero(filled)}；填补内部结构孔洞："
          f"{np.count_nonzero(filled) - np.count_nonzero(highland)}")


if __name__ == "__main__":
    main()
