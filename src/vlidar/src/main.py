from miniros import AsyncROSClient
from miniros.util.decorators import decorators
from miniros_vlidar.source.datatypes import LidarData
import adafruit_rplidar as pyrplidar
import asyncio


class VLidarClient(AsyncROSClient):
    def __init__(self, ip = "localhost", port = 3000):
        super().__init__("vlidar", ip, port)

        self.lidar = pyrplidar.RPLidar(None, "/dev/ttyUSB0", baudrate=115200)
        self.lidar.connect()
        
        self.lidar.stop_motor()


async def main():
    client = VLidarClient()

    async def run():
        await client.wait()

        ldr_topic = await client.topic("lidar", LidarData)

        client.lidar.start_motor()

        while True:
            try:
                for scan in client.lidar.iter_scans():
                    quality, angles, distances = zip(*scan)

                    print(angles, distances)

                    await ldr_topic.post(
                        LidarData(
                            distances,
                            angles,
                        )
                    )

            except pyrplidar.RPLidarException as e:
                print(f"Lidar exception: {e}. Reconnecting...")
                
                client.lidar.stop()
                client.lidar.disconnect()

                await asyncio.sleep(0.4)
                
                client.lidar = pyrplidar.RPLidar(None, "/dev/ttyUSB0")
                
            except Exception as e:
                print(f"Unexpected error: {e}")
                
                await asyncio.sleep(0.2)
                
                
    await asyncio.gather(
        client.run(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
