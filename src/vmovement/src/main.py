from miniros import AsyncROSClient
from miniros.util.datatypes import Vector
from miniros.util.decorators import decorators
from miniros_vslam.source.datatypes import SLAMPosition
from miniros_vmovement.source.control import TrackedRobotIK, MotorController, SerialReader
import asyncio
import numpy as np
import platform
import serial_asyncio as serial


DRIVE_WHEEL_RADIUS = 0.02 # m
TRACKS_SPACE = 0.14 # m
MAX_RPM = 330 # rpm
MAX_LINEAR_SPEED = 2 # m/s
MAX_ANGULAR_SPEED = 25.4 # rad/s


class VMovementClient(AsyncROSClient):
    def __init__(self, controller: MotorController, ip = "localhost", port = 3000):
        super().__init__("vmovement", ip, port)

        self.controller = controller

        self.s_pos = (0, 0)
        self.s_rot = 0

        self.m_target = None
        self.m_moving = False


    @decorators.aparsedata(Vector, 1)
    async def on_moveto(self, data: Vector, node: str):
        if data.y == 1:
            self.m_moving = False
            self.m_target = None
            
            self.controller.stop()
            
        else:
            self.m_target = (data.x, data.z)
            self.m_moving = True

            self.controller.set_position(*self.s_pos, self.s_rot) # TODO: check is s_rot degrees or radians
    
            await self.controller.move_to(
                *self.m_target, 
                high_precision = np.linalg.norm([self.s_pos[0] - self.m_target[0], self.s_pos[1] - self.m_target[1]]) < 0.5
            )
                

    @decorators.aparsedata(SLAMPosition, 1)
    async def on_vslam_pos(self, data: SLAMPosition):
        self.s_pos = (data.pos.x, data.pos.z)
        self.s_rot = data.ang.y


async def main():
    kinematics = TrackedRobotIK(
        TRACKS_SPACE,
        MAX_RPM,
        DRIVE_WHEEL_RADIUS
    )
    
    loop = asyncio.get_event_loop()
    serial_port = "COM6" if platform.system() == "Windows" else "/ttyUSBmotor"
    transport, protocol = await serial.create_serial_connection(loop, lambda: SerialReader(controller), serial_port, baudrate=9600)

    controller = MotorController(kinematics, transport, MAX_RPM, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)
    
    await asyncio.sleep(3.5) # wait for initialization of arduino
    
    client = VMovementClient(controller)

    await client.run()

async def test():
    kinematics = TrackedRobotIK(
        TRACKS_SPACE,
        MAX_RPM,
        DRIVE_WHEEL_RADIUS
    )
    
    loop = asyncio.get_event_loop()
    serial_port = "COM6" if platform.system() == "Windows" else "/ttyUSBmotor"

    controller = MotorController(kinematics, None, MAX_RPM, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)    
    transport, protocol = await serial.create_serial_connection(loop, lambda: SerialReader(controller), serial_port, baudrate=9600)
    controller.ser = transport

    await asyncio.sleep(3.5)
    
    controller.set_lcd(False)
    controller.set_speed(160, 160)
    controller.send()

if __name__ == "__main__":
    asyncio.run(test())
