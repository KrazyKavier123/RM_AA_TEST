#!/usr/bin/env python3
"""训练装甲板数字分类网络：CNN 提特征 + 全连接分类头。

输出与 vision/src/Classifier.cpp 的接口兼容：
  - 输入 (1, 1, 28, 20)：extract() 得到的 28x20 单通道 OTSU 二值图，/255 到 {0,1}
  - 输出 (1, 9) logits：softmax 在 C++ 里做，模型末端不要加 softmax
  - 类别顺序与 models/label.txt 一致

数据集：DATA_DIR 下每个类别一个子文件夹（图片是已裁好的装甲板数字区域），
类别顺序取文件夹名排序；现有数据集用 1..9 前缀保证与 label.txt 的顺序一致。
本脚本只处理这条真实数据链路，不生成任何合成样本。

在仓库根目录运行：python3 models/train_cnn.py
"""
import os

import cv2
import torch
import torch.nn as nn

# ===================== 可调参数 =====================
DATA_DIR = "models/TEST_CNN/datasets"  # 数据集：每个类别一个子文件夹
LABEL_FILE = "models/label.txt"        # 类别名，顺序需与文件夹排序一致
OUT_PT = "models/cnn_fc.pt"        # 训练权重
OUT_ONNX = "models/cnn_fc.onnx"    # 给 C++ 加载的模型
EPOCHS = 50
BATCH = 64
LR = 1e-3
W, H = 20, 28                      # 与 Classifier::extract() 的 20x28 一致
OPSET = 11                         # ONNX opset


class PatchSet(torch.utils.data.Dataset):
    """按类别文件夹读取图片，缩放并 OTSU 二值化后返回 (1, H, W)。"""

    def __init__(self, root):
        self.classes = sorted(d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d)))
        self.samples = []
        for idx, name in enumerate(self.classes):
            folder = os.path.join(root, name)
            for f in os.listdir(folder):
                if f.lower().endswith((".png", ".jpg", ".jpeg", ".bmp")):
                    self.samples.append((os.path.join(folder, f), idx))

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, i):
        path, idx = self.samples[i]
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        img = cv2.resize(img, (W, H))
        _, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
        x = torch.from_numpy(img).float().div_(255).unsqueeze(0)  # (1, H, W)
        return x, idx


class Net(nn.Module):
    """CNN + 全连接分类头。输入 (N,1,28,20)，输出 (N,classes) logits。"""

    def __init__(self, classes):
        super().__init__()
        self.cnn = nn.Sequential(
            nn.Conv2d(1, 16, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),   # 16x14x10
            nn.Conv2d(16, 32, 3, padding=1), nn.ReLU(), nn.MaxPool2d(2),  # 32x7x5
        )
        self.fc = nn.Sequential(
            nn.Flatten(),
            nn.Linear(32 * (H // 4) * (W // 4), 84), nn.ReLU(),
            nn.Linear(84, classes),
        )

    def forward(self, x):
        return self.fc(self.cnn(x))


def main():
    with open(LABEL_FILE) as f:
        labels = [s.strip() for s in f if s.strip()]

    data = PatchSet(DATA_DIR)
    print("folders:", data.classes)
    print("labels :", labels)

    loader = torch.utils.data.DataLoader(data, batch_size=BATCH, shuffle=True)
    model = Net(len(data.classes))
    opt = torch.optim.Adam(model.parameters(), lr=LR)
    loss_fn = nn.CrossEntropyLoss()

    for epoch in range(EPOCHS):
        model.train()
        loss_sum = correct = total = 0
        for x, y in loader:
            opt.zero_grad()
            out = model(x)
            loss = loss_fn(out, y)
            loss.backward()
            opt.step()
            loss_sum += loss.item() * y.numel()
            correct += (out.argmax(1) == y).sum().item()
            total += y.numel()
        print(f"epoch {epoch + 1:3d}  loss {loss_sum / max(total, 1):.4f}  acc {correct / max(total, 1):.3f}")

    model.eval()
    torch.save(model.state_dict(), OUT_PT)
    torch.onnx.export(model, torch.zeros(1, 1, H, W), OUT_ONNX,
                      input_names=["input"], output_names=["logits"], opset_version=OPSET)
    print(f"saved {OUT_PT} and {OUT_ONNX}")


if __name__ == "__main__":
    main()
