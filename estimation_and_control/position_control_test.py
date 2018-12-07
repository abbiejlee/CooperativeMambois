"""
    position_control_test.py
    Marcus Abate | 16.30
    12/6/18

    Test of the PositionController class and its ability to track a
    reference trajectory using only the sensor readings off the mambo,
    and no state estimation.
"""

from Drone import Drone
from PositionController import MamboPositionController

class PosCtrlDrone(Drone):
    def __init__(self, test_flying, mambo_addr, use_wifi, use_vision):
        super().__init__(test_flying, mambo_addr, use_wifi, use_vision)
        self.controller = MamboPositionController()
        self.current_state = []
        self.desired_waypoint = [1, 0, 1]
        self.desired_state = self.desired_waypoint + [0, 0, 0]
        self.eps = 0.08

    def sensor_cb(self, args):
        self.current_state = [self.mambo.sensors.sensor_dict['DronePosition_posx'],
                              self.mambo.sensors.sensor_dict['DronePosition_posy'],
                              self.mambo.sensors.sensor_dict['DronePosition_posz'],
                              self.mambo.sensors.speed_x,
                              self.mambo.sensors.speed_y,
                              self.mambo.sensors.speed_z]
        self.controller.set_current_state(self.current_state)

    def vision_cb(self, args):
        pass

    def flight_func(self, mamboVision, args):
        """
        Takeoff, fly to (1, 0, 1) with units (m), land.
        """
        if self.test_flying:
            print('taking off')
            self.mambo.safe_takeoff(5)

            if self.mambo.sensors.flying_state != 'emergency':

                print('sensor calib:')
                while self.mambo.sensors.speed_ts == 0:
                    continue

                self.controller.set_desired_state(self.desired_state)
                print('flying to position ', self.desired_waypoint)
                while ( (self.current_state[0] - self.desired_state[0])**2 +
                        (self.current_state[1] - self.desired_state[1])**2 +
                        (self.current_state[2] - self.desired_state[2])**2 )**0.5 > self.esp:
                    cmd = self.controller.calculate_cmd_input()
                    self.mambo.fly_direct(roll=cmd[0],
                                            pitch=cmd[1],
                                            yaw=cmd[2],
                                            vertical_movement=cmd[3],
                                            duration=None)
                    print('cmd:',cmd)
            print('landing')
            self.mambo.safe_land(5)

        else:
            print('not flying')

test_flying = True
mambo_addr = "e0:14:ad:f6:3d:fc"
use_wifi = True
use_vision = False

if __name__ == "__main__":
    posCtrlTest = PosCtrlDrone(test_flying, mambo_addr, use_wifi, use_vision)
    posCtrlTest.execute()
