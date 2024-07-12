import os
import cv2
from pyzbar import pyzbar

OPENCV = "opencv"
ZBAR = "zbar"

RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
RESET_COLOR = '\033[0m'


def scan_qr(image, library_type=OPENCV):

    if library_type == OPENCV:
        # グレースケールに変換
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        # QRコードリーダーを作成
        qr_code_detector = cv2.QRCodeDetectorAruco()

        # QRコードを検出
        data, bbox, _ = qr_code_detector.detectAndDecode(gray)

        # QRコードが検出された場合
        if bbox is not None:
            # QRコードの内容を出力
            print(f'{GREEN if data!="" else YELLOW}QRコードの内容: {data}{RESET_COLOR}')

            box = bbox[0]
            if len(box) == 4 and all(len(b) == 2 for b in box):
                for i in range(len(box)):
                    # bboxの点を整数に変換
                    point1 = tuple(map(int, box[i]))
                    point2 = tuple(map(int, box[(i + 1) % len(box)]))

                    # 線を引く
                    cv2.line(image, point1, point2, (0, 255, 0), 2)

                    # QRコードの内容を画像に描画
                    cv2.putText(image, data, (box[0][0].astype(int), box[0][1].astype(
                        int) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            print(f'{RED}QRコードが読み取れませんでした{RESET_COLOR}')
        return image
    elif library_type == ZBAR:
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        value = pyzbar.decode(gray, symbols=[pyzbar.ZBarSymbol.QRCODE])

        if value:
            data = ""
            for qrcode in value:
                data += qrcode.data.decode('utf-8')+","

                # QRコード座標取得
                x, y, w, h = qrcode.rect

                # QRコードデータ
                cv2.putText(image, data, (x, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # バウンディングボックス
                cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # QRコードの内容を出力
            print(f'{GREEN}QRコードの内容: {data}{RESET_COLOR}')

        else:
            print(f'{RED}QRコードが読み取れませんでした{RESET_COLOR}')
        return image
    else:
        print(f'ライブラリ{library_type}は存在しません')


if __name__ == "__main__":
    # 画像ファイルのパス
    image_dir = "QR_image"
    image_dir_list = [os.path.join(image_dir, file)
                      for file in os.listdir(image_dir)]

    # scanの設定
    lib_type = OPENCV

    print(image_dir+"/")
    for image_path in [file for file in image_dir_list if os.path.isfile(file)]:
        image = cv2.imread(image_path)
        print("  "+image_path[len(image_dir)+1:]+":\t", end="")
        scan_qr(image, lib_type)
    print()

    for dir in [dir for dir in image_dir_list if os.path.isdir(dir)]:
        print(dir+"/")
        for file_name in os.listdir(dir):
            image_path = os.path.join(dir, file_name)
            if os.path.isfile(image_path) and file_name.endswith((".png", ".jpg", ".jpeg")):
                image = cv2.imread(image_path)
                print("  "+file_name+":\t", end="")
                scan_qr(image, lib_type)
        print()
