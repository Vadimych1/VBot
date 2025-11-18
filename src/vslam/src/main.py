from miniros import AsyncROSClient, datatypes
from miniros.util.decorators import aparsedata
from miniros.util.util import Ticker
from miniros_vslam.source.datatypes import (
    SLAMMap,
    SLAMAnonSave,
    SLAMAnonLoad,
    SLAMPosition,
)
import miniros_breezyslam.algorithms as algos
import miniros_breezyslam.sensors as sensors
import asyncio

# import matplotlib.pyplot as plt
# plt.ion()

MAP_SIZE_PX = 1000
MAP_SIZE_MET = 25

# fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
# ax.set_rmax(3000)
# ax.grid(True)

# line, = ax.plot([], [], "b-")


class VSLAMClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000):
        super().__init__("vslam", ip, port)

        self.slam = algos.RMHC_SLAM(
            sensors.RPLidarA1(),
            MAP_SIZE_PX,
            MAP_SIZE_MET,
            hole_width_mm=130,  # TODO: replace with better hole_width
        )

        self.map = bytearray(MAP_SIZE_PX**2)
        self.pos = (0, 0, 0)

    @aparsedata(SLAMAnonSave)
    async def on_save(self, data: int, _: str):
        return

        self.slam.getmap(self.map)
        with open(f"maps/{data}.map", "wb") as f:
            f.write(self.map)

    @aparsedata(SLAMAnonLoad)
    async def on_read(self, data: int, _: str):
        return
    
        try:
            with open(f"maps/{data}.map", "rb") as f:
                data = f.read()
                if len(data) != MAP_SIZE_PX**2:
                    return
                self.map = data
                self.slam.setmap(data)
        except:
            pass

    @aparsedata(datatypes.LidarDatatype)
    async def on_vlidar_lidar(self, data: datatypes.LidarDatatype):
        dist, ang = data.distances, data.angles

        self.slam.update(scans_mm=dist, scan_angles_degrees=ang)

        self.slam.getmap(self.map)
        self.pos = self.slam.getpos()

    # NOTE: simulator support
    @aparsedata(datatypes.LidarDatatype)
    async def on_vsimulator_lidar(self, data: datatypes.LidarDatatype):
        dist, ang = list(data.distances), list(data.angles)
        
        # with open("r.txt", "a+") as f:
        #     f.write(",".join(map(str, [float(x) for x in dist])) + "\n\n")

        if len(dist) != len(ang):
            return "Got invalid data"

        # # visualisation purposes
        # if len(dist) >= 0 and len(ang) >= 0:
        #     ax.set_ylim(0, max(10, max(dist)))
        #     ax.scatter(ang, dist, c="red", s=30)

        #     fig.canvas.draw()
        #     fig.canvas.flush_events()

        #     plt.pause(0.01)

        self.slam.update(scans_mm=dist, scan_angles_degrees=ang)

        self.slam.getmap(self.map)

        self.pos = self.slam.getpos()
        
        # with open("map.bin", "wb") as f: f.write(self.map)

    @aparsedata(SLAMMap)
    async def on_setmap(self, data: SLAMMap, node: str):
        # TODO: make better
        
        return
    
        self.slam.setmap(data.data)


async def main():
    client = VSLAMClient()

    async def run():
        await client.wait()

        map_topic = await client.topic("map", SLAMMap)
        pos_topic = await client.topic("pos", SLAMPosition)

        ticker = Ticker(2)

        while True:
            await ticker.tick_async()
            await map_topic.post(SLAMMap(client.map))

            x, y, theta = client.pos
            await pos_topic.post(
                SLAMPosition(datatypes.Vector(x, 0, y), datatypes.Vector(0, theta, 0))
            )

    await asyncio.gather(
        client.run(),
        run(),
    )


if __name__ == "__main__":
    asyncio.run(main())
