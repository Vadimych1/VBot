from miniros import AsyncROSClient
from miniros import decorators
from miniros import datatypes
from miniros import utils
from miniros_vcam.source.codes import Detector
import cv2 as cv
import asyncio


class VCamClient(AsyncROSClient):
    def __init__(self, ip = "localhost", port = 3000):
        super().__init__("vcam", ip, port)
        self.last_frame = None

    async def on_detect(self, data: int, node: str):
        barcodes = Detector.detect(self.last_frame)
        to_send = {
            "l": len(barcodes),
            **{i: k for i, k in zip(range(len(barcodes)), barcodes)}
        }

        self.anon(
            node,
            "barcodes",
            datatypes.Dict.encode(to_send)
        )


async def main():
    client = VCamClient()
    cap = cv.VideoCapture(0)

    async def run():
        await client.wait()

        camera_topic = await client.topic("camera", datatypes.OpenCVImage)

        ticker = utils.Ticker(10)
        while cap.isOpened():
            await ticker.tick_async()

            ret, frame = cap.read()

            if not ret:
                break

            await camera_topic.post(frame)
            client.last_frame = frame

    asyncio.gather(
        client.run(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
