#!/usr/bin/env python3
"""Calibrate a pinhole camera from chessboard images and write a complete ROS YAML."""
import argparse
import glob
from pathlib import Path
import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', required=True, help='Quoted image glob')
    parser.add_argument('--cols', type=int, required=True, help='Inner corners per row')
    parser.add_argument('--rows', type=int, required=True, help='Inner corners per column')
    parser.add_argument('--square-m', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.cols < 3 or args.rows < 3 or not np.isfinite(args.square_m) or args.square_m <= 0:
        parser.error('Invalid chessboard dimensions')
    object_grid = np.zeros((args.rows * args.cols, 3), np.float32)
    object_grid[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_m
    objects, pixels, accepted = [], [], []
    image_size = None
    for path in sorted(glob.glob(args.images)):
        gray = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        size = (gray.shape[1], gray.shape[0])
        if image_size is not None and size != image_size:
            parser.error('All calibration images must have the same resolution')
        image_size = size
        found, corners = cv2.findChessboardCornersSB(gray, (args.cols, args.rows))
        if found:
            objects.append(object_grid.copy())
            pixels.append(corners)
            accepted.append(path)
    if len(objects) < 6:
        parser.error(f'Only {len(objects)} usable views; need at least 6 with varied orientations')
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(objects, pixels, image_size, None, None)
    if not np.isfinite(rms) or not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
        raise RuntimeError('Calibration produced nonfinite values')
    template = Path(__file__).resolve().parents[1] / 'src/rm_aa_bringup/config/perception.yaml'
    config = yaml.safe_load(template.read_text(encoding='utf-8'))
    params = config['perception']['ros__parameters']
    params['calibration'] = {'calibrated': True, 'width': image_size[0], 'height': image_size[1],
                             'k': matrix.reshape(-1).tolist(), 'd': distortion.reshape(-1).tolist()}
    params['source']['width'], params['source']['height'] = image_size
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(config, sort_keys=False), encoding='utf-8')
    print(f'Calibration RMS: {rms:.4f} px; {len(objects)} views; saved {args.output}')
    for path, object_points, image_points, rotation, translation in zip(accepted, objects, pixels, rotations, translations):
        projected, _ = cv2.projectPoints(object_points, rotation, translation, matrix, distortion)
        error = np.sqrt(np.mean(np.sum((projected - image_points) ** 2, axis=2)))
        print(f'{error:.4f} px  {path}')


if __name__ == '__main__':
    main()
