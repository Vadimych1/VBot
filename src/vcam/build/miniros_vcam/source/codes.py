import cv2
from pyzbar import pyzbar
from pyzbar.pyzbar import ZBarSymbol

class Detector:
    @staticmethod
    def detect(frame: cv2.Mat):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        barcodes = pyzbar.decode(
            gray,
            symbols=[ZBarSymbol.QRCODE, ZBarSymbol.CODE128, ZBarSymbol.EAN13]
        )

        return list(map(lambda x: x.data, barcodes))
    

if __name__ == "__main__":  
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            break

        data = Detector.detect(frame=frame)

        print(data)

    cap.release()