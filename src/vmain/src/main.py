from miniros import AsyncROSClient, datatypes, utils, aparsedata
from miniros_ssmain.source.datatypes import RobotTask
import asyncio
import argparse
import json


class VMainClient(AsyncROSClient):
    def __init__(self, robot_name: str, ip="localhost", port=3000, _parse_handlers=True):
        super().__init__("vmain", ip, port, _parse_handlers)
        
        self.robot_name = robot_name
        self.movedone = False
        
        self.task: RobotTask | None = None
        self.other_positions: dict[str, datatypes.Vector] = {}
        self.barcodes: dict[int, str] | None = None
        
    @aparsedata(RobotTask)
    async def on_task(self, data: RobotTask, node):
        if data is None: return
        if data.target != self.robot_name: return
        
        self.task = data
    
    @aparsedata(datatypes.Dict)
    async def on_otherpos(self, data, node):
        self.other_positions = data
        
    @aparsedata(datatypes.Dict)
    async def on_barcodes(self, data, node):
        self.barcodes = data
        
    async def on_movementdone(self, data, node):
        self.movedone = True
        
    async def wait_for_move(self):
        while not self.movedone:
            await asyncio.sleep(0.1)


async def main(robot_name: str):
    client = VMainClient(robot_name)
    ticker = utils.Ticker(0.5) # every 2 seconds

    async def run():
        await client.wait()
        
        taskdone_topic = await client.topic("taskdone", datatypes.Bytes)
        
        while True:
            await ticker.tick_async()
            
            if client.task is not None:
                # notify ssmain that task is received
                taskdone_topic.post(b"\x00")
                
                n = 0
                for task_item in client.task.items:
                    start_x, start_y, end_x, end_y, destination_x, destination_y = \
                        task_item.sx, task_item.sy, task_item.ex, task_item.ey, client.task.dx, client.task.dy
 
                    # move to start of shelf
                    await client.anon("vpathfinder", "settarget", datatypes.Vector.encode(datatypes.Vector(start_x, 0, start_y)))
                    await client.wait_for_move()

                    # move to end of shelf                    
                    await client.anon("vpathfinder", "settarget", datatypes.Vector.encode(datatypes.Vector(end_x, 0, end_y)))
                    
                    # while moving send anon to vcam:detect, wait for answer to vmain:barcodes
                    while client.barcodes is None or task_item.name not in client.barcodes.values():
                        await client.anon("vcam", "detect", b'k')
                        asyncio.sleep(0.1)
                        
                    # stop
                    await client.anon("vpathfinder", "settarget", datatypes.Vector.encode(datatypes.Vector(0, 1, 0)))
                    
                    # move to dest
                    await client.anon("vpathfinder", "settarget", datatypes.Vector.encode(datatypes.Vector(destination_x, 0, destination_y)))
                    await client.wait_for_move()
    
                    # notify ssmain that part of task is done
                    n += 1
                    taskdone_topic.post(bytes([n]))
    
                client.task = None
 
                # end task
                await taskdone_topic.post(b'\xff')

    await asyncio.gather(
        client.run(),
        run()
    )
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser("vmain")
    parser.add_argument("ss_config_path", type=str)
    args = parser.parse_args()
    
    cfg_path = args.ss_config_path
    with open(cfg_path) as f:
        cfg = json.load(f)
    
    robot_name = cfg["robot_name"]
    
    asyncio.run(main(robot_name))
