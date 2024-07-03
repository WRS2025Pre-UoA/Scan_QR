import cv2
from pyzbar.pyzbar import decode, ZBarSymbol

OPENCV="opencv"
ZBAR="zbar"

def scan_qr(image, library_type=OPENCV):

    if library_type==OPENCV:
        # グレースケールに変換
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # QRコードリーダーを作成
        qr_code_detector = cv2.QRCodeDetectorAruco()

        # QRコードを検出
        data, bbox, _ = qr_code_detector.detectAndDecode(gray)

        # QRコードが検出された場合
        if bbox is not None:
            # QRコードの内容を出力
            print(f'QRコードの内容: {data}')

            box=bbox[0]
            if len(box) == 4 and all(len(b) == 2 for b in box):
                for i in range(len(box)):
                    # bboxの点を整数に変換
                    point1 = tuple(map(int,box[i]))
                    point2 = tuple(map(int, box[(i + 1) % len(box)]))

                    # 線を引く
                    cv2.line(image, point1, point2, (0, 255, 0), 2)

                    # QRコードの内容を画像に描画
                    cv2.putText(image, data, (box[0][0].astype(int), box[0][1].astype(int) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        else:
            print('QRコードが読み取れませんでした')
        return image
    elif library_type==ZBAR:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        value = decode(gray, symbols=[ZBarSymbol.QRCODE])

        if value:
            qrcode=value[0]

            # QRコードの内容を出力
            data = qrcode.data.decode('utf-8')
            print(f'QRコードの内容: {data}')

            # QRコード座標取得
            x, y, w, h = qrcode.rect

            # QRコードデータ
            cv2.putText(image, data, (x, y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # バウンディングボックス
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
        return image
    else:
        print(f'ライブラリ{library_type}は存在しません')

if __name__=="__main__":
    # 画像ファイルのパス
    image_path = '####.png'

    # 画像を読み込む
    image = cv2.imread(image_path)

    scan_qr(image)