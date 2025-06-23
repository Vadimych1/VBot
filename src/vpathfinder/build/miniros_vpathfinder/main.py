from miniros import AsyncROSClient
from miniros.util.decorators import decorators
from miniros.util.datatypes import Vector, Int
from miniros.util.util import Ticker
from miniros_vslam.source.datatypes import SLAMMap, SLAMPosition
import miniros_vpathfinder.source.algorithms as algos
import asyncio
import math
import numpy as np


class VPathfinderClient(AsyncROSClient):
    def __init__(self, ip = "localhost", port = 3000):
        super().__init__("vpathfinder", ip, port)

        self.s_map = None
        self.s_pos = None
        self.s_ang = None

        self.p_end = None
        self.p_alive = False
        self.p_path = None

        self.p_path_built = False
        self.p_path_built_ticks = 0

        self.m_now_moving = None

    @decorators.aparsedata(Vector)
    async def on_end(self, data: Vector, node: str):
        if not self.p_alive:
            self.p_end = (data.x, data.z)
            self.p_alive = True

            self.p_path_built = False
            self.p_path_built_ticks = 0

    @decorators.aparsedata(SLAMMap)
    async def on_vslam_map(self, data: SLAMMap):
        self.s_map = data.to_numpy(int(math.sqrt(len(data.data))))

    @decorators.aparsedata(SLAMPosition)
    async def on_vslam_pos(self, data: SLAMPosition):
        self.s_pos = data.pos_to_numpy()
        self.s_ang = data.rot_to_numpy()

    async def on_moved(self, data: int, node: str):
        if self.p_path is not None and self.p_path_built:
            cur_pos = self.m_now_moving

            prev_ind = np.argmin([algos.astar_heuristic(cur_pos, p) for p in self.p_path])
            next_ind = max(prev_ind + 1, len(self.p_path) - 1)

            next_pos = self.p_path

            await self.anon(node, "moveto", Vector.encode(
                Vector(next_pos[0], 0 + next_ind == prev_ind, next_pos[1])  
            ))

    def build_path(self):
        global_grid = algos.prepare_map(self.s_map)
        path = algos.astar(global_grid, (int(self.s_pos[0]), int(self.s_pos[2])), self.p_end)
        simplified = algos.simplify_path(path, global_grid)
        smoothed = algos.smooth_path(simplified, global_grid)

        self.p_path = smoothed # TODO: ADD ADAPTIVE MAX_LOOKAHEAD
        self.p_path_built = True
        self.p_path_built_ticks = 0

    def update_path(self):
        dilated = algos.prepare_map(self.s_map)

        path = algos.local_update_path(self.p_path, (int(self.s_pos[0]), int(self.s_pos[2])), dilated)
        simplified = algos.simplify_path(path, dilated)
        smoothed = algos.smooth_path(simplified, dilated, iterations=20)

        self.p_path = smoothed # TODO: ADD ADAPTIVE MAX_LOOKAHEAD


async def main():
    client = VPathfinderClient()
    ticker = Ticker(2)

    async def run():
        await client.wait()

        while True:
            await ticker.tick_async()

            if client.p_alive and client.p_path_built:
                if client.p_path_built_ticks >= 40: # fully reconstruct path every 20 sec
                    client.build_path()

                else:
                    client.update_path()
                    client.p_path_built_ticks += 1

            elif client.p_alive:
                client.build_path()

    await asyncio.gather(
        client.run(),
        run()
    )


if __name__ == "__main__":
    asyncio.run(main())
