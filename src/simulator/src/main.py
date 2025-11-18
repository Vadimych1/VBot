from miniros.simulator.simulator import Robot, JointGroup, load_urdf, LidarSensor, mainloop
from miniros import datatypes, aparsedata
from miniros_vslam.source.datatypes import SLAMPosition
from miniros_vmovement.source.control import TrackedRobotIK
import numpy as np
import asyncio

class MyRobot(Robot):
    def __init__(self, urdf: str, controller: "MotorController", startPos = [0, 0, 1], startOrientation = [0, 0, 0]):
        super().__init__(urdf, startPos, startOrientation, "vsimulator")
        
        self.s_pos = (0, 0)
        self.s_rot = 0
        
        self.controller = controller
        
    @aparsedata(datatypes.Vector)
    async def on_moveto(self, data: datatypes.Vector, _: str):
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
                t = 3,
                high_precision = np.linalg.norm([self.s_pos[0] - self.m_target[0], self.s_pos[1] - self.m_target[1]]) < 0.5
            )
            
    @aparsedata(SLAMPosition)
    async def on_vslam_pos(self, data: SLAMPosition):
        self.s_pos = (data.pos.x, data.pos.z)
        self.s_rot = data.ang.y

class MotorController:
    def __init__(self, kinematics: TrackedRobotIK, left_track: JointGroup, right_track: JointGroup, max_rpm: int = 300, max_lin_speed=2, max_ang_speed=25.4):
        self.max_rpm = max_rpm
        
        self.left = 0
        self.left_sign = 0
        
        self.right = 0
        self.right_sign = 0
        
        self.x = 0
        self.y = 0
        self.w = 0
        
        self.max_lin_speed = max_lin_speed
        self.max_ang_speed = max_ang_speed
        
        self.ik = kinematics

        self.left_track = left_track
        self.right_track = right_track
        
        self.acc_time = 0.3
        self.acc = 0
        
    def set_position(self, x, y, w):
        """
        Set position of robot
        
        :param x: x position of robot, m
        :param y: y posotion of robot, m
        :param w: rotation of robot on z axis, rad
        """
        
        self.x = x
        self.y = y
        self.w = w


    async def move_to(self, x, y, t = 3, high_precision = False):
        """
        Move robot to specific coordinates
        
        Make sure you called `set_position` with actual data before using
        
        Time for rotation and movement will be split in half
        
        Returns tuple of was movement successful and what time was taken to move
        
        :param x: target x position, m
        :param y: target y position, m
        :param t: wanted time to move, seconds
        """
        
        print("moving to", x, y, t)
        
        delta_x = (x - self.x) / 10 # TODO: add this to vmovement package (conversion from mm)
        delta_y = (y - self.y) / 10 # TODO: add this to vmovement package (conversion from mm)
        
        print(delta_x, delta_y)
        
        angle = np.arctan2(delta_y, delta_x) - (np.pi if delta_x < 0 else 0)
        distance = np.linalg.norm([delta_x, delta_y])
        
        print(angle, distance, t)
        
        result, rt = await self.rotate_to(angle, t=t/3*2) # rotate to (dx; dy) vector direction
        await asyncio.sleep(0.1) # delay to stabilize

        if not result:
            return False, 0

        t = min(t / 3, t - rt)
        
        print(t, rt)
        
        speed = max(self.max_lin_speed / 4, min(self.max_lin_speed / 6 * 5, distance / t))
        t = abs(distance / speed)
                
        # if t < (at := (self.acc_time / 100)) and not high_precision:
        #     speed /= (at / t)
        #     t = at
        #     if speed < self.max_lin_speed / 4:
        #         return False, 0
            
        l, r, scale = self.ik.compute_wheel_speeds(speed, 0)
        t /= scale
            
        # acc_stored_val = self.acc
        # if high_precision:
        #     self.acc = 0
            
        print("Moving:", l, r, t)
            
        self.set_speed(l, r)
        # await self.send()
        await asyncio.sleep(t)

        self.stop()
        # self.acc = acc_stored_val
        # await self.send()
        
        return True, rt + t + 0.1


    async def rotate_to(self, w, t, high_precision = False):
        print("rotating to", w / np.pi * 180, t)
        
        """
        Rotate robot to specific angle
        
        Make sure you called `set_position` with actual data before using
        
        Returns tuple of was movement successful and what time was taken to move
        
        :param w: target rotation on z axis, rad
        :param t: wanted time of movement, seconds
        """
        
        delta_w = w - self.w
        
        # speed for moving by `time` seconds (if in range of [1/4 of max speed; 2/3 of max speed])
        w_speed = max(self.max_ang_speed / 4, min(self.max_ang_speed / 3 * 2, abs(delta_w) / t))    
        t = abs(delta_w / w_speed)
        
        # acc_time is a minimum time of controller to respond to next command
        # if t < (at := (self.acc_time / 100)) and not high_precision:
        #     w_speed /= (at / t)
        #     t = at
        #     if w_speed < self.max_ang_speed / 4:
        #         return False, 0
            
        l, r, scale = self.ik.compute_wheel_speeds(0, w_speed)
        t /= scale
        
        print("Rotating:", l, r, t)
        
        # acc_stored_val = self.acc
        # if high_precision:
        #     self.acc = 0
        
        self.set_speed(l, r)
        # await self.send()
        await asyncio.sleep(t)
        
        self.stop()
        # self.acc = acc_stored_val
        # await self.send()
        
        return True, t
        

    def set_speed(self, left: int, right: int):
        """
        Change speed of motors
        
        left, right - motor speeds in RPM
        
        negative values mean that motor is spinning in opposite direction
        """
        
        self.left_track.set_speed(left)
        self.right_track.set_speed(right)
        

    def stop(self):
        """
        Stop motors
        """
        
        self.set_speed(0, 0)
    
    
    def set_accelerate(self, val: bool):
        """
        Disable/enable motor easing acceleration
        """
        
        self.acc = 1 if val else 0

plane = load_urdf("plane.urdf")
shelf_1 = load_urdf("data/shelf.urdf", [0, 8, 1], [0, 0, np.pi])
shelf_2_1 = load_urdf("data/shelf_small.urdf", [-20 / 3, 2, 1], [0, 0, np.pi])
shelf_2_2 = load_urdf("data/shelf_small.urdf", [0, 2, 1], [0, 0, np.pi])
shelf_2_3 = load_urdf("data/shelf_small.urdf", [20 / 3, 2, 1], [0, 0, np.pi])
shelf_3 = load_urdf("data/shelf.urdf", [0, -4, 1], [0, 0, np.pi])
room = load_urdf("data/room.urdf", [0, 0, 1])

DRIVE_WHEEL_RADIUS = 0.02 * 10# m
TRACKS_SPACE = 0.14 * 10 # m
MAX_RPM = 330 # rpm
MAX_LINEAR_SPEED = 2 * 10 # m/s
MAX_ANGULAR_SPEED = 25.4 # rad/s

kinematics = TrackedRobotIK(
    TRACKS_SPACE,
    MAX_RPM,
    DRIVE_WHEEL_RADIUS
)

robot = MyRobot("data/robot.urdf", None, startPos=[0, -8, 0])

left_track = robot.create_joint_group([f"lwheel{i+1}_joint" for i in range(4)])
right_track = robot.create_joint_group([f"rwheel{i+1}_joint" for i in range(4)])

controller = MotorController(kinematics, left_track, right_track, MAX_RPM, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)
robot.controller = controller

lidar: LidarSensor = robot.create_sensor("lidar", "lidar", LidarSensor, xyz=[0, 0, 1])
lidar.set_attribute("measures_per_scan", 360)

if __name__ == "__main__":
    mainloop()
    
    async def main():
        async def run():
            await robot.wait()
            
            await lidar.initialize()
        
            while True:
                await asyncio.sleep(1)
                await lidar.measure_and_post()
        
        await asyncio.gather(
            robot.run(),
            run()
        )
    
    asyncio.run(main())
