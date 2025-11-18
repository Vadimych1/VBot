from miniros import AsyncROSClient, aparsedata, datatypes
from miniros_vslam.source.datatypes import SLAMPosition
from aioconsole import ainput
import asyncio


class ControlClient(AsyncROSClient):
    def __init__(self, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("vctrl", ip, port, _parse_handlers)
        self.pos: datatypes.Vector = datatypes.Vector(12500, 0, 12500)

    @aparsedata(SLAMPosition)
    async def on_vslam_pos(self, data: SLAMPosition):
        self.pos = data.pos


async def main():
    client = ControlClient()

    async def run():
        await client.wait()
        
        while True:
            cmd: str = await ainput("> ")
            tokens: list[str] = cmd.strip().split(" ")

            if len(tokens) <= 0:
                continue

            match tokens[0]:
                case "goby":
                    dx, dy = map(float, tokens[1:])
                    await client.anon(
                        "vsimulator", "moveto", datatypes.Vector.encode(datatypes.Vector(dx, 0, dy) + client.pos)
                    )

                case "goto":
                    x, y = map(float, tokens[1:])
                    await client.anon("vsimulator", "moveto", datatypes.Vector.encode(datatypes.Vector(x, 0, y)))

                case _:
                    ...

    await asyncio.gather(client.run(), run())


asyncio.run(main())
