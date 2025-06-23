from miniros import AsyncROSClient
from miniros.util.datatypes import Vector
from miniros.util.decorators import decorators
from miniros_vslam.source.datatypes import SLAMPosition
import asyncio

class VMovementClient(AsyncROSClient):
    def __init__(self, ip = "localhost", port = 3000):
        super().__init__("vmovement", ip, port)

        self.s_pos = (0, 0)
        self.s_rot = 0

        self.m_target = None
        self.m_moving = False

    @decorators.aparsedata(Vector, 1)
    async def on_moveto(self, data: Vector, node: str):
        if data.y == 1:
            self.m_moving = False
            self.m_target = None
            
            # TODO: Stop

        else:
            self.m_target = (data.x, data.z)
            self.m_moving = True

    @decorators.aparsedata(SLAMPosition, 1)
    async def on_vslam_pos(self, data: SLAMPosition):
        self.s_pos = (data.pos.x, data.pos.z)
        self.s_rot = data.ang.y

    @staticmethod
    def _calculate_motors_speed(go_from: tuple[float, float], go_to: tuple[float, float]):
        # TODO: algos
        ...


async def main():
    client = VMovementClient()
    await client.run()


if __name__ == "__main__":
    asyncio.run(main())