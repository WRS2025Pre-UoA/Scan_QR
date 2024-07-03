# Scan_QR

## 環境

### 単体
- Python3
- Open-CV
- pyzbar

### ROS2
- Python3
- Open-CV
- cv_bridge
- pyzbar

### 環境構築
python3やpipのインストールは省く

OpenCVのインストール
```bash
pip install opencv-python
```

pyzbarのインストール(mac)
```bash
brew install zbar
pip3 install pyzbar
```

pyzbarのインストール(Ubuntu)
```bash
sudo apt install libzbar0
pip3 install pyzbar
```

## 初期設定
### 単体
単体で実行したい場合は特に制限がなく、好きな場所にリポジトリをクローンして良い

### ROS2
ワークスペースの`src`ディレクトリ内にリポジトリをクローンする
```bash
cd src/
git clone git@github.com:WRS2025Pre-UoA/Scan_QR.git
```

## 実行方法

### 単体
```bash
python3 qr_reader.py
```

### ROS2
ワークスペースのルートディレクトリでビルド(`colcon build`)をして、以下のコマンドで実行する

```bash
ros2 run qr_scan listener
```