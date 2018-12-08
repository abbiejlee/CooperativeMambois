"""
    shooter_script.py
    Marcus Abate | 16.30
    12/7/18

    Shooter drone flies to position marked in the shoot_here.txt file
    and fires gun.
"""

from Drone import Drone
from PositionController import MamboPositionController
from KalmanFilter import MamboKalman
from ColorSegmentation import cd_color_segmentation
import csv

filename = 'shoot_here.csv'

class ShooterDrone(Drone):
    """
    Handles the shooter drone's flight and callbacks.
    """

    def __init__(self, mambo_addr, firing_position):
        super().__init__(True, mambo_addr, False, False) # BLE enabled drone
        self.controller = MamboPositionController()
        self.kalmanfilter = MamboKalman([0,0,0],[0,0,0])
        self.current_vels = []
        self.current_state = []
        self.desired_state = firing_position
        self.eps = 0.08
        self.max_alt = 3 # maximum altitude (m)
        self.max_dist = 10 # maximum distance from target accepted (m)
        self.start_measure = False

    def sensor_cb(self, args):
        """
        Throw sensor readings into the state estimator and then save the
        current state. Then update the controller with this current state.
        """
        # print('callback')
        if self.start_measure:
            current_measurement = [self.mambo.sensors.sensors_dict['DronePosition_posx']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posy']/100,
                                        self.mambo.sensors.sensors_dict['DronePosition_posz']/-100]
            self.current_vels = [self.mambo.sensors.speed_x,
                                 self.mambo.sensors.speed_y,
                                 self.mambo.sensors.speed_z]
            self.current_state = self.kalmanfilter.get_state_estimate(current_measurement,
                                                                        self.current_vels)
            self.controller.set_current_state(self.current_state)

    def vision_cb(self, args):
        pass # no need; it's a BLE drone with no camera

    def go_to_firing_pos(self):
        """
        Use position controller to execute flight command to go to a desired
        xyz position from origin (defined at takeoff).

        Returns True if position reached within tolerance self.eps
        Returns False if maximum altitude is reached/exceeded or distance gets
            too large.
        """
        self.controller.set_desired_state(self.desired_state)
        print('flying to position ', self.desired_state)

        dist = ((self.current_state[0] - self.desired_state[0])**2 +
                (self.current_state[1] - self.desired_state[1])**2 +
                (self.current_state[2] - self.desired_state[2])**2 )**0.5

        while dist > self.eps:
            cmd = self.controller.calculate_cmd_input()

            self.mambo.fly_direct(roll=cmd[1],
                                    pitch=cmd[0],
                                    yaw=cmd[2],
                                    vertical_movement=cmd[3],
                                    duration=0.1)

            dist = ((self.current_state[0] - self.desired_state[0])**2 +
                    (self.current_state[1] - self.desired_state[1])**2 +
                    (self.current_state[2] - self.desired_state[2])**2 )**0.5

            if self.current_state[2] >= self.max_alt or dist >= self.max_dist:
                return False
        return True

    def fly_away(self):
        """
        Drone flies back and right facing the target for a safe landing.
        """
        self.mambo.fly_direct(roll=-30, pitch=-30, yaw=0, vertical_movement=0, duration=3)

    def flight_func(self, mamboVision, args):
        """
        Take off, fly to the firing position, fire gun. Fly away in xy and land.

        Can choose between doing the trajectory using PositonController and
        KalmanFilter, or just as a dumb "fly up" trajectory.
        """
        print('taking off')
        self.mambo.safe_takeoff(5)

        if self.mambo.sensors.flying_state != 'emergency':

            print('sensor calib:')
            while self.mambo.sensors.speed_ts == 0:
                continue
            self.start_measure = True
            print(self.start_measure)

            print('getting first state')
            while self.current_state == []:
                self.mambo.smart_sleep(1)

            self.go_to_firing_pos()

            shots_fired = self.mambo.fire_gun()
            if not shots_fired:
                print('failed to shoot')
            self.mambo.smart_sleep(2)

        # self.fly_away()
        print('landing')
        self.mambo.safe_land(5)

shoot_addr = "e0:14:ed:d2:3d:d1"

if __name__ == "__main__":

    firing_pos = []

    with open(filename, newline='') as file:
        reader = csv.reader(file)
        for row in reader:
            firing_pos = [float(i) for i in row]

    shootDrone = ShooterDrone(shoot_addr, firing_pos)
    shootDrone.execute()
