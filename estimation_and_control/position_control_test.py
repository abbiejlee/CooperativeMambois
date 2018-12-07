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
        self.current_state = [] # meters
        self.desired_state = [1, 0, 1] # meters
        self.eps = 0.08
        self.start_measure = False

    def sensor_cb(self, args):
        if self.start_measure:
            self.current_state = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                  self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                  self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]
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
                self.start_measure = True

                print('getting first state')
                while self.current_state == []:
                    continue

                self.controller.set_desired_state(self.desired_state)
                print('flying to position ', self.desired_state)
                while ( (self.current_state[0] - self.desired_state[0])**2 +
                        (self.current_state[1] - self.desired_state[1])**2 +
                        (self.current_state[2] - self.desired_state[2])**2 )**0.5 > self.eps:
                    cmd = self.controller.calculate_cmd_input()
                    print('current state:',self.current_state)
                    print('cmd:          ',cmd)
                    self.mambo.fly_direct(roll=cmd[1],
                                            pitch=cmd[0],
                                            yaw=cmd[2],
                                            vertical_movement=cmd[3],
                                            duration=None)
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
