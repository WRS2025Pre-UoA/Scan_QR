import cv2

# 画像ファイルのパス
image_path = '####.png'

# 画像を読み込む
image = cv2.imread(image_path)

# グレースケールに変換
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# QRコードリーダーを作成
qr_code_detector = cv2.QRCodeDetector()

# QRコードを検出
data, bbox, _ = qr_code_detector.detectAndDecode(gray)

# QRコードが検出された場合
if bbox is not None:
    # QRコードの内容を出力
    print(f'QRコードの内容: {data}')
else:
    print('QRコードが見つかりませんでした')
