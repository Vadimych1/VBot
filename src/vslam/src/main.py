from miniros import AsyncROSClient
from miniros.util.decorators import decorators
from miniros.util.datatypes import Vector
from miniros.util.util import Ticker
from miniros_vslam.source.datatypes import SLAMMap, SLAMPosition, SLAMAnonSave, SLAMAnonLoad
import miniros_breezyslam.algorithms as algos
import miniros_breezyslam.sensors as sensors
import miniros_vlidar.source.datatypes as vlidar_datatypes
import asyncio


MAP_SIZE_PX = 4000
MAP_SIZE_MET = 40


class VSLAMClient(AsyncROSClient):
    def __init__(self, ip = "localhost", port = 3000):
        super().__init__("vslam", ip, port)

        self.slam = algos.RMHC_SLAM(
            sensors.RPLidarA1(),
            MAP_SIZE_PX,
            MAP_SIZE_MET
        )

        self.map = bytearray(MAP_SIZE_PX ** 2)
        self.pos = (0, 0, 0)


    @decorators.aparsedata(SLAMAnonSave)
    async def on_save(self, data: int, field: str):
        self.slam.getmap(self.map)
        with open(f"maps/{data}.map", "wb") as f:
            f.write(self.map)


    @decorators.aparsedata(SLAMAnonLoad)
    async def on_read(self, data: int, field: str):
        try:
            with open(f"maps/{data}.map", "rb") as f:
                data = f.read()
                if len(data) != MAP_SIZE_PX ** 2: return
                self.map = data
                self.slam.setmap(data)
        except: pass


    @decorators.aparsedata(vlidar_datatypes.LidarData)
    async def on_vlidar_lidar(self, data: vlidar_datatypes.LidarData):
        dist, ang = data.distances, data.angles
        self.slam.update(
            dist,
            scan_angles_degrees=ang,
        )

        self.slam.getmap(self.map)
        self.pos = self.slam.getpos()


async def main():
    client = VSLAMClient()

    async def run():
        await client.wait()

        map_topic = await client.topic("map", SLAMMap)
        pos_topic = await client.topic("pos", SLAMPosition)

        ticker = Ticker(2)

        while True:
            await ticker.tick_async()

            await map_topic.post(
                SLAMMap(client.map)
            )

            x, y, theta = client.pos
            await pos_topic.post(
                SLAMPosition(
                    Vector(x, 0, y),
                    Vector(0, theta, 0)
                )
            )
    
    await asyncio.gather(
        client.run(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
