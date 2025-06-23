from miniros import AsyncROSClient
from miniros.util.decorators import decorators
from miniros_vlidar.source.datatypes import LidarData
import pyrplidar
import asyncio


class VLidarClient(AsyncROSClient):
    def __init__(self, ip = "localhost", port = 3000):
        super().__init__("vlidar", ip, port)

        self.lidar = pyrplidar.PyRPlidar()
        self.lidar.connect()


async def main():
    client = VLidarClient()

    async def run():
        await client.wait()

        ldr_topic = await client.topic("lidar", LidarData)

        distances = []
        angles = []

        for scan in client.lidar.start_scan():
            distances.append(scan.distance)
            angles.append(scan.angle)

            if len(distances) >= 360:
                await ldr_topic.post(
                    LidarData(
                        distances,
                        angles,
                    )
                )

                distances.clear()
                angles.clear()

    await asyncio.gather(
        client.run(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
