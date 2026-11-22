"""从普通棋盘格照片求解海康相机内参；仅在已知角点规格时运行。"""
import argparse
import csv
from pathlib import Path

import cv2
import numpy as np
import yaml


def calibrate(files, columns, rows, square_m, output, outlier_px=None, rotate_180=False):
    if square_m is not None and square_m <= 0:
        raise ValueError("方格尺寸无效")
    if columns < 2 or rows < 2:
        raise ValueError("角点数量与方格尺寸无效")
    physical_square_m = square_m if square_m is not None else 1.0
    object_grid = np.zeros((columns * rows, 3), np.float32)
    object_grid[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * physical_square_m
    accepted, rejected, resolution = [], [], None
    for file in files:
        image = cv2.imread(str(file))
        if image is None:
            rejected.append((str(file), "读取失败"))
            continue
        if rotate_180:
            image = cv2.rotate(image, cv2.ROTATE_180)
        size = (image.shape[1], image.shape[0])
        if resolution is None:
            resolution = size
        if size != resolution:
            rejected.append((str(file), "分辨率不一致"))
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        enhanced = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        found, corners = cv2.findChessboardCornersSB(enhanced, (columns, rows),
                                                       flags=cv2.CALIB_CB_EXHAUSTIVE)
        if found:
            accepted.append((file, corners.astype(np.float32)))
        else:
            rejected.append((str(file), "角点未检出"))
    if len(accepted) < 5:
        raise ValueError(f"仅检出 {len(accepted)} 张棋盘格图像；至少需要 5 张")

    def solve(samples):
        rms, k, d, rvecs, tvecs = cv2.calibrateCamera(
            [object_grid] * len(samples), [x[1] for x in samples], resolution, None, None)
        errors = []
        for (_, corners), rv, tv in zip(samples, rvecs, tvecs):
            projected, _ = cv2.projectPoints(object_grid, rv, tv, k, d)
            errors.append(float(np.sqrt(np.mean(np.sum((projected-corners)**2, axis=2)))))
        return rms, k, d, errors

    initial_rms, _, _, initial_errors = solve(accepted)
    removed = []
    if outlier_px is not None:
        kept = []
        for sample, error in zip(accepted, initial_errors):
            if error > outlier_px:
                removed.append((str(sample[0]), error))
            else:
                kept.append(sample)
        if len(kept) < 5:
            raise ValueError("剔除异常图像后不足 5 张")
        accepted = kept
    rms, k, d, errors = solve(accepted)
    out = Path(output)
    out.mkdir(parents=True, exist_ok=True)
    payload = {"image_width": resolution[0], "image_height": resolution[1],
               "camera_model": "plumb_bob", "distortion_model": "plumb_bob",
               "checkerboard_inner_corners": [columns, rows], "square_size_m": square_m, "square_size_measured": square_m is not None,
               "K": k.tolist(), "D": d.flatten().tolist(), "rms_px": float(rms)}
    (out / "camera_intrinsics.yaml").write_text(yaml.safe_dump(payload, allow_unicode=True, sort_keys=False))
    with (out / "camera_reprojection_errors.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["image", "rmse_px"])
        writer.writerows((str(file), error) for (file, _), error in zip(accepted, errors))
    report = f"""# 相机内参标定报告

- 图像分辨率：{resolution[0]} × {resolution[1]}
- 棋盘格内部角点：{columns} × {rows}；方格边长：{square_m if square_m is not None else "未测量（内参求解采用归一化单位；板位姿不具备米制意义）"}
- 输入图像：{len(files)}；成功参与标定：{len(accepted)}
- 初次 RMS：{initial_rms:.4f} px；最终 RMS：{rms:.4f} px
- K：`{k.tolist()}`
- D：`{d.flatten().tolist()}`
- 未检出或读取失败：{len(rejected)}；按阈值剔除：{len(removed)}

## 异常图像

"""
    report += "\n".join(f"- {name}: {reason}" for name, reason in rejected)
    report += "\n" + "\n".join(f"- {name}: 初次误差 {error:.4f} px" for name, error in removed) + "\n"
    (out / "camera_calibration_report.md").write_text(report)
    return payload


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="棋盘格相机内参标定")
    parser.add_argument("image_dir")
    parser.add_argument("--columns", type=int, required=True, help="每行内部角点数")
    parser.add_argument("--rows", type=int, required=True, help="每列内部角点数")
    parser.add_argument("--square-m", type=float, help="实测方格边长（米）；仅求内参可省略")
    parser.add_argument("--output", required=True,
                        help="本台相机的新标定结果目录；两台相机使用不同目录")
    parser.add_argument("--outlier-px", type=float)
    parser.add_argument("--rotate-180", action=argparse.BooleanOptionalAction,
                        default=False,
                        help="离线读取照片时默认不旋转；仅处理倒装相机原始照片时启用")
    args = parser.parse_args()
    files = sorted(p for p in Path(args.image_dir).iterdir() if p.suffix.lower() in (".png", ".jpg", ".jpeg"))
    print(calibrate(files, args.columns, args.rows, args.square_m,
                    args.output, args.outlier_px, args.rotate_180))
