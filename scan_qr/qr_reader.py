import os
import cv2
from pyzbar import pyzbar

OPENCV = "opencv"
ZBAR = "zbar"

RED = '\033[31m'
GREEN = '\033[32m'
YELLOW = '\033[33m'
RESET_COLOR = '\033[0m'


def image_resize(image, width=1280):
    h, w = image.shape[:2]
    height = round(h * (width / w))
    image = cv2.resize(image, (width, height), interpolation=cv2.INTER_LINEAR)
    return image


def convert_image(origin, thresh=51):
    gray = cv2.cvtColor(origin, cv2.COLOR_BGR2GRAY)
    img = cv2.adaptiveThreshold(
        gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, thresh, 2)

    return img


def scan_qr(image, library_type=OPENCV, original=[]):
    if original == []:
        original = image.copy()

    if library_type == OPENCV:

        # QRコードリーダーを作成
        qr_code_detector = cv2.QRCodeDetectorAruco()

        # QRコードを検出
        data, bbox, _ = qr_code_detector.detectAndDecode(image)

        # QRコードが検出された場合
        if bbox is not None:

            box = bbox[0]
            if len(box) == 4 and all(len(b) == 2 for b in box):
                for i in range(len(box)):
                    # bboxの点を整数に変換
                    point1 = tuple(map(int, box[i]))
                    point2 = tuple(map(int, box[(i + 1) % len(box)]))

                    # 線を引く
                    cv2.line(original, point1, point2, (0, 255, 0), 2)

                    # QRコードの内容を画像に描画
                    cv2.putText(original, data, (box[0][0].astype(int), box[0][1].astype(
                        int) - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            return original, [data]
        else:
            return original, []

    elif library_type == ZBAR:
        value = pyzbar.decode(image, symbols=[pyzbar.ZBarSymbol.QRCODE])

        if value:
            data = []
            for qrcode in value:
                v = qrcode.data.decode('utf-8')
                data.append(v)

                # QRコード座標取得
                x, y, w, h = qrcode.rect

                # QRコードデータ
                cv2.putText(original, v, (x, y - 6),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                # バウンディングボックス
                cv2.rectangle(original, (x, y), (x + w, y + h), (0, 255, 0), 2)

            return original, data
        else:
            return original, []
    else:
        print(f'ライブラリ{library_type}は存在しません')
        return original, []


if __name__ == "__main__":
    # 画像ファイルのパス
    image_dir = "../QR_image"
    image_dir_list = [os.path.join(image_dir, file)
                      for file in os.listdir(image_dir)]

    # scanの設定
    lib_type = ZBAR

    print(image_dir+"/")
    for image_path in [file for file in image_dir_list if os.path.isfile(file)]:
        image = cv2.imread(image_path)
        image = image_resize(image, width=1280)
        image = convert_image(image, 51)
        print("  "+image_path[len(image_dir)+1:]+":\t", end="")
        _, v = scan_qr(image, lib_type)
        if v == []:
            print(f'{RED}読み取れませんでした{RESET_COLOR}')
        elif v[0] == "":
            print(f'{YELLOW}内容がわかりませんでした{RESET_COLOR}')
        else:
            text = ', '.join(v)
            print(f'{GREEN}内容:{text}{RESET_COLOR}')
    print()

    for dir in [dir for dir in image_dir_list if os.path.isdir(dir)]:
        print(dir+"/")
        for file_name in os.listdir(dir):
            image_path = os.path.join(dir, file_name)
            if os.path.isfile(image_path) and file_name.endswith((".png", ".jpg", ".jpeg")):
                image = cv2.imread(image_path)
                image = image_resize(image, width=1280)
                image = convert_image(image, 51)
                print("  "+file_name+":\t", end="")
                _, v = scan_qr(image, lib_type)
                if v == []:
                    print(f'{RED}読み取れませんでした{RESET_COLOR}')
                elif v[0] == "":
                    print(f'{YELLOW}内容がわかりませんでした{RESET_COLOR}')
                else:
                    text = ', '.join(v)
                    print(f'{GREEN}内容:{text}{RESET_COLOR}')

        print()
