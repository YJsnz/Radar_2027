#!/usr/bin/env python3
"""用官方俯视图的原始地胶外观生成与 CAD 参考坐标对齐的 SVG/PNG 显示资产。"""
import argparse
import base64
import json
import os
from pathlib import Path

import cv2
import numpy as np

from export_2027_map import ROOT, SOURCE_SHA256, digest, write_json


REFERENCE_SHA256 = "941a9d71368f7650d2b70ee6f46539ffed63adf43b3b428e60de22969e2b3ce9"


def register(source, target):
    cv2.setRNGSeed(0)
    reduced = cv2.resize(target, (1400, 750), interpolation=cv2.INTER_AREA)
    sift = cv2.SIFT_create(nfeatures=12000, contrastThreshold=0.02)
    source_keys, source_features = sift.detectAndCompute(cv2.cvtColor(source, cv2.COLOR_BGR2GRAY), None)
    target_keys, target_features = sift.detectAndCompute(cv2.cvtColor(reduced, cv2.COLOR_BGR2GRAY), None)
    pairs = cv2.BFMatcher().knnMatch(source_features, target_features, k=2)
    matches = [first for first, second in pairs if first.distance < 0.7 * second.distance]
    a = np.float32([source_keys[match.queryIdx].pt for match in matches])
    b = np.float32([target_keys[match.trainIdx].pt for match in matches]) * 2
    if len(matches) < 16:
        raise ValueError("缺少足够的跨图匹配点")
    affine, mask = cv2.estimateAffine2D(a, b, method=cv2.RANSAC, ransacReprojThreshold=5,
                                      maxIters=20000, confidence=0.999, refineIters=20)
    if affine is None:
        raise ValueError("官方俯视图无法与 CAD 参考图配准")
    keep = mask.ravel().astype(bool)
    a, b = a[keep], b[keep]
    if len(a) < 16 or np.any(np.ptp(a, axis=0) < np.array([source.shape[1], source.shape[0]]) * 0.65):
        raise ValueError("配准点数量或空间覆盖不足")
    if np.linalg.det(affine[:, :2]) <= 0:
        raise ValueError("配准产生镜像，拒绝导出")
    homogeneous = np.column_stack((a, np.ones(len(a))))
    predicted = homogeneous @ affine.T
    errors = np.linalg.norm(predicted - b, axis=1)
    cross_validation = []
    for index in range(len(a)):
        use = np.arange(len(a)) != index
        estimate = np.linalg.lstsq(homogeneous[use], b[use], rcond=None)[0]
        cross_validation.append(float(np.linalg.norm(homogeneous[index] @ estimate - b[index])))
    quadrant = (b[:, 0] > 1400).astype(int) + 2 * (b[:, 1] > 750)
    quadrant_counts = [int(np.sum(quadrant == index)) for index in range(4)]
    if min(quadrant_counts) == 0 or max(cross_validation) > 8:
        raise ValueError("配准空间分布或留一检查未通过")
    transform = np.vstack((affine, [0, 0, 1]))
    return transform, {
        "method": "SIFT descriptor ratio 0.7; affine RANSAC threshold 5 CAD pixels",
        "opencv_version": cv2.__version__, "descriptor_matches": len(matches),
        "inlier_count": len(a), "median_residual_cad_px": float(np.median(errors)),
        "quadrant_counts": quadrant_counts,
        "max_residual_cad_px": float(errors.max()),
        "leave_one_out_median_cad_px": float(np.median(cross_validation)),
        "leave_one_out_max_cad_px": max(cross_validation),
        "source_pixels": a.tolist(), "cad_pixels": b.tolist(),
        "predicted_cad_pixels": predicted.tolist(),
        "source_to_cad_image": transform.tolist(),
        "interpretation": "display registration residuals; not physical position accuracy",
    }


def svg_document(payload, transform, width, height, overlay=""):
    a, b, c = transform[0]
    d, e, f = transform[1]
    return (f'<svg xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" '
            f'width="{width}" height="{height}" viewBox="0 0 {width} {height}">'
            '<title>RMUC 2027 official appearance, registered to CAD reference</title>'
            '<desc>Original PDF image embedded without retouching. Display registration only.</desc>'
            f'<image width="981" height="559" transform="matrix({a} {d} {b} {e} {c} {f})" '
            f'xlink:href="data:image/jpeg;base64,{payload}"/>{overlay}</svg>')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, default=ROOT / "vision/images-2027/rmuc2027-v1.0.0-overhead.jpg")
    parser.add_argument("--cad", type=Path, default=ROOT / "vision/images-2027/cad-v1.0.0")
    parser.add_argument("--output", type=Path, default=ROOT / "vision/images-2027/display-v1.0.0")
    args = parser.parse_args()
    cad_manifest = json.loads((args.cad / "manifest.json").read_text())
    if digest(args.reference) != REFERENCE_SHA256 or cad_manifest["source_sha256"] != SOURCE_SHA256:
        raise ValueError("输入地图或 STEP 版本与已核对版本不同")
    if digest(args.cad / "topdown.png") != cad_manifest["files"]["topdown.png"]["sha256"]:
        raise ValueError("CAD 参考底图校验值不匹配")
    source = cv2.imread(str(args.reference))
    target = cv2.imread(str(args.cad / "topdown.png"))
    if source is None or target is None or source.shape[:2] != (559, 981) or target.shape[:2] != (1500, 2800):
        raise ValueError("输入图片尺寸不匹配")
    transform, registration = register(source, target)
    corners = np.array([[0, 0, 1], [2800, 0, 1], [0, 1500, 1], [2800, 1500, 1]])
    source_corners = (np.linalg.inv(transform) @ corners.T).T[:, :2]
    if np.any(source_corners < 0) or np.any(source_corners >= [981, 559]):
        raise ValueError("场地裁切超出原图，拒绝填充缺失内容")
    registration["field_corners_in_source_pixels"] = source_corners.tolist()
    payload = base64.b64encode(args.reference.read_bytes()).decode("ascii")
    # SVG retains the untouched official raster and an explicit affine matrix.
    # Qt renders the native SVG; no generated texture or geometry is introduced.
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    from PyQt5.QtCore import QByteArray, PYQT_VERSION_STR, qVersion
    from PyQt5.QtGui import QImage, QPainter
    from PyQt5.QtSvg import QSvgRenderer
    from PyQt5.QtWidgets import QApplication

    application = QApplication.instance() or QApplication([])
    args.output.mkdir(parents=True, exist_ok=True)
    files = {}
    views = [
        ("topdown", np.eye(3), 2800, 1500),
        ("red_bottom", np.array([[0, 1, 0], [-1, 0, 2800], [0, 0, 1]]), 1500, 2800),
        ("blue_bottom", np.array([[0, -1, 1500], [1, 0, 0], [0, 0, 1]]), 1500, 2800),
    ]
    for name, orientation, width, height in views:
        text = svg_document(payload, orientation @ transform, width, height)
        svg_path = args.output / f"{name}.svg"
        svg_path.write_text(text)
        renderer = QSvgRenderer(QByteArray(text.encode()))
        if not renderer.isValid():
            raise ValueError(f"{name} SVG 无效")
        image = QImage(width, height, QImage.Format_RGB32)
        image.fill(0xffffffff)
        painter = QPainter(image)
        painter.setRenderHint(QPainter.SmoothPixmapTransform, True)
        renderer.render(painter)
        painter.end()
        png_path = svg_path.with_suffix(".png")
        if not image.save(str(png_path)):
            raise RuntimeError(f"无法保存 {png_path}")
        for path in (svg_path, png_path):
            files[path.name] = {"sha256": digest(path), "size_px": [width, height]}
    overlay = []
    for predicted, cad in zip(registration["predicted_cad_pixels"], registration["cad_pixels"]):
        x, y = predicted
        u, v = cad
        overlay.append(f'<line x1="{x}" y1="{y}" x2="{u}" y2="{v}" stroke="yellow" stroke-width="2"/>')
        overlay.append(f'<circle cx="{x}" cy="{y}" r="9" fill="none" stroke="lime" stroke-width="2"/>')
        overlay.append(f'<circle cx="{u}" cy="{v}" r="4" fill="red"/>')
    check = args.output / "alignment_check.svg"
    check.write_text(svg_document(payload, transform, 2800, 1500, "".join(overlay)))
    files[check.name] = {"sha256": digest(check)}
    write_json(args.output / "registration.json", registration)
    files["registration.json"] = {"sha256": digest(args.output / "registration.json")}
    write_json(args.output / "manifest.json", {
        "schema": "ysu.radar.display_map.v1", "profile": "rmuc2027-v1.0.0-display1",
        "appearance_source": "rules V1.0.0 PDF page 30, printed page 29, figure 4-1",
        "appearance_sha256": REFERENCE_SHA256, "source_resolution_px": [981, 559],
        "svg_rasterizer": "Qt QSvgRenderer", "qt_version": qVersion(),
        "pyqt_version": PYQT_VERSION_STR,
        "cad_manifest_sha256": digest(args.cad / "manifest.json"),
        "cad_profile": cad_manifest["profile"], "source_step_sha256": SOURCE_SHA256,
        "field_size_cm": cad_manifest["field_size_cm"],
        "image_coordinates": cad_manifest["image_coordinates"],
        "source_to_cad_image": transform.tolist(), "physical_validation": False,
        "limitations": ["official appearance is upsampled, not higher-resolution source detail",
                        "display registration is approximate; CAD geometry remains the coordinate reference",
                        "camera extrinsics and terrain projection masks must be recalibrated"],
        "files": files,
    })
    print(f"已保留官方地胶，匹配内点 {registration['inlier_count']}，"
          f"配准残差中位数 {registration['median_residual_cad_px']:.3f} CAD像素")
    # Keep the QApplication alive until every QImage/SVG render has completed.
    application.processEvents()


if __name__ == "__main__":
    main()
