#!/usr/bin/env python3
"""把训练好的 .pt 转成 vision/src/Classifier.cpp 可直接加载的 .onnx。

输入 (1, 1, 28, 20)，输出 (1, 9) logits，与 train_cnn.py / 现有接口一致。
支持 torch.save(model) 保存的整模型，也支持 torch.save(model.state_dict()) 保存的权重。

在仓库根目录运行：
  python3 models/pt2onnx.py
  python3 models/pt2onnx.py --pt models/cnn_fc.pt --onnx models/cnn_fc.onnx
"""
import argparse

import cv2
import numpy as np
import torch

from train_cnn import H, W, Net


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pt", default="models/cnn_fc.pt")
    ap.add_argument("--onnx", default="models/cnn_fc.onnx")
    ap.add_argument("--classes", type=int, default=9)
    ap.add_argument("--opset", type=int, default=11)
    args = ap.parse_args()

    obj = torch.load(args.pt, map_location="cpu")
    if isinstance(obj, torch.nn.Module):
        model = obj
    else:
        model = Net(args.classes)
        model.load_state_dict(obj)
    model.eval()

    torch.onnx.export(model, torch.zeros(1, 1, H, W), args.onnx,
                      input_names=["input"], output_names=["logits"], opset_version=args.opset)

    # 兼容性自检：用 OpenCV DNN 加载并前向一次，确认输出是 9 维。
    net = cv2.dnn.readNetFromONNX(args.onnx)
    net.setInput(np.zeros((1, 1, H, W), np.float32))
    print(f"{args.pt} -> {args.onnx}  (cv2 output {net.forward().shape})")


if __name__ == "__main__":
    main()
