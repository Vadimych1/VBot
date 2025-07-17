import serial
import numpy as np
import platform
import time

class IK:
    def compute_wheel_speeds(self, v, w):
        """
        Compute motors speed in RPM
        
        :param v: linear speed, m/s
        :param w: angular speed, rad/s
        """
        
        raise NotImplementedError()
    
class TrackedRobotIK(IK):
    def __init__(self, track_width, max_speed, wheel_r):
        """
        Tracked robot inverse kinematics
        
        :param track_width: distance between tracks, m
        :param max_speed: max rotation speed of track, RPM
        :param wheel_r: radius of wheel, m
        """
        
        self.track_width = track_width
        self.max_speed = max_speed
        self.wheel_r = wheel_r
        
    def compute_wheel_speeds(self, v, w):
        """
        Compute motors speed in RPM
        
        :param v: linear speed, m/s
        :param w: angular speed, rad/s
        """
        
        v_left = v - 0.5 * w * self.track_width
        v_right = v + 0.5 * w * self.track_width
        
        v_left = v_left * np.pi / self.wheel_r
        v_right = v_right * np.pi / self.wheel_r
        
        scale = max(
            abs(v_left) / self.max_speed,
            abs(v_right) / self.max_speed,
            1.0
        )
        
        v_left /= scale
        v_right /= scale

        return v_left, v_right, scale


class MotorController:
    def __init__(self, kinematics: IK, max_rpm: int = 300, max_lin_speed=2, max_ang_speed=25.4, port=("COM6" if platform.system() == "Windows" else "/ttyUSBmotor")):
        self.ser = serial.Serial(port, 9600)
        self.max_rpm = max_rpm
        
        self.left = 0
        self.left_sign = 0
        
        self.right = 0
        self.right_sign = 0
        
        self.lcd = 1
        self.acc = 1
        self.acc_time = 60
        
        self.x = 0
        self.y = 0
        self.w = 0
        
        self.max_lin_speed = max_lin_speed
        self.max_ang_speed = max_ang_speed
        
        self.ik = kinematics
        
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

    def move_to(self, x, y, t = 3, high_precision = False):
        """
        Move robot to specific coordinates
        
        Make sure you called `set_position` with actual data before using
        
        Time for rotation and movement will be split with 1:2 ratio
        
        Returns tuple of was movement successful and what time was taken to move
        
        :param x: target x position, m
        :param y: target y position, m
        :param t: wanted time to move, seconds
        """
        
        delta_x = x - self.x
        delta_y = y - self.y
        
        angle = np.atan2(delta_y, delta_x) - (np.pi if delta_x < 0 else 0)
        distance = np.linalg.norm([delta_x, delta_y])
        
        result, rt = self.rotate_to(angle, t=t/3, high_precision=high_precision) # rotate to (dx; dy) vector direction
        time.sleep(0.1) # delay to stabilize

        if not result:
            return False, 0

        t = t - rt
        speed = max(self.max_lin_speed / 4, min(self.max_lin_speed / 6 * 5, distance / t))
        t = abs(distance / speed)
                
        if t < (at := (self.acc_time / 100)) and not high_precision:
            speed /= (at / t)
            t = at
            if speed < self.max_lin_speed / 4:
                return False, 0
            
        l, r, scale = self.ik.compute_wheel_speeds(speed, 0)
            
        acc_stored_val = self.acc
        if high_precision:
            self.acc = 0
            
        self.set_speed(l, r)
        self.send()
        time.sleep(t)

        self.set_speed(0, 0)
        self.acc = acc_stored_val
        self.send()
        
        return True, rt + t + 0.1

    def rotate_to(self, w, t = 1.5, high_precision = False):
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
        if t < (at := (self.acc_time / 100)) and not high_precision:
            w_speed /= (at / t)
            t = at
            if w_speed < self.max_ang_speed / 4:
                return False, 0
            
        l, r, scale = self.ik.compute_wheel_speeds(0, w_speed)
        
        acc_stored_val = self.acc
        if high_precision:
            self.acc = 0
        
        self.set_speed(l, r)
        self.send()
        time.sleep(t)
        
        self.set_speed(0, 0) # does not take time to accelerate
        self.acc = acc_stored_val
        self.send()
        
        return True, t
        
    def set_speed(self, left: int, right: int):
        """
        Change speed of motors
        
        left, right - motor speeds in RPM
        
        negative values mean that motor is spinning in opposite direction
        """
        
        left = int(left / self.max_rpm * 255)
        right = int(right / self.max_rpm * 255)

        lsign = int((left / abs(left) + 1) / 2) if left != 0 else 1
        left = min(255, max(0, abs(left)))
        
        rsign = int((right / abs(right) + 1) / 2) if right != 0 else 1
        right = min(255, max(0, abs(right)))
        
        self.right = right
        self.right_sign = rsign
        
        self.left = left
        self.left_sign = lsign
        
    def stop(self):
        """
        Stop motors
        
        equal to `controller.change_speeds(0, 0)`
        """
        
        self.change_speeds(0, 0)
    
    def set_lcd(self, val: bool):
        """
        Disable/enable LCD lights
        """
        
        self.lcd = 1 if val else 0
        
    def set_accelerate(self, val: bool):
        """
        Disable/enable motor easing acceleration
        """
        
        self.acc = 1 if val else 0
        
    def set_accelerate_speed(self, val: int):
        """
        Set acceleration time (100-2500, ms)
        """
        
        self.acc_time = int(val / 10)
        
    def send(self):
        """
        Send data to Arduino controller
        """
        
        self.ser.write(bytes([
            self.lcd, self.acc, self.right, self.right_sign, self.left, self.left_sign, self.acc_time
        ]))

DRIVE_WHEEL_RADIUS = 0.02 # m
TRACKS_SPACE = 0.14 # m
MAX_RPM = 330 # rpm
MAX_LINEAR_SPEED = 2 # m/s
MAX_ANGULAR_SPEED = 25.4 # rad/s

kinematics = TrackedRobotIK(
    TRACKS_SPACE,
    MAX_RPM,
    DRIVE_WHEEL_RADIUS
)
controller = MotorController(kinematics, MAX_RPM, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED)

time.sleep(4) # init of arduino

controller.move_to(-10, -10)
