"""
    displacement_calculation_test.py
    Marcus Abate | 16.30
    12/1/18

    Simple test of the PositionController class's ability to determine total
    displacement from original starting point based on velocity feedback.
    This test will help to determine whether state updates are happening fast
    enough and whether the sensors have low enough drift to accurately track
    position from the starting point.
"""

from pyparrot.Minidrone import Mambo
from pyparrot.DroneVisionGUI import DroneVisionGUI
import time
import math

class DisplacementCalculationTest:
    def __init__(self, testFlying, mamboAddr, use_wifi, use_vision):
        self.testFlying = testFlying
        self.mamboAddr = mamboAddr
        self.use_wifi = use_wifi
        self.use_vision = use_vision
        self.mambo = Mambo(self.mamboAddr, self.use_wifi)
        self.mambo.set_user_sensor_callback(self.sensor_avg_cb, args=None)
        if self.use_vision:
            self.mamboVision = DroneVisionGUI(self.mambo, is_bebop=False, buffer_size=200,
                                     user_code_to_run=self.mambo_fly_function, user_args=None)
            self.mamboVision.set_user_callback_function(self.vision_cb, user_callback_args=None)
        self.current_xyz_pos = [0, 0, 0]
        self.current_xyz_vel = [0, 0, 0]
        # self.current_state = self.current_xyz_pos + self.current_xyz_vel
        # self.time_of_last_update = time.perf_counter()
        self.time_of_last_update = self.mambo.sensors.speed_ts
        self.dt_since_last_update = 0
        self.vels = [] # for calculating running average

    def vision_cb(self, args):
        """
        Any optional vision frame handling can be done here if desired.
        Currently not implemented.
        """
        pass

    def sensor_cb(self, args):
        """
        Called whenever a drone sensor is updated.
        Sends sensor readings to the position controller to get displacement.
        """
        self.current_xyz_vel = [self.mambo.sensors.speed_x,
                                self.mambo.sensors.speed_y,
                                self.mambo.sensors.speed_z]
        # self.dt_since_last_update = time.perf_counter() - self.time_of_last_update
        # self.time_of_last_update = time.perf_counter()
        self.dt_since_last_update = self.mambo.sensors.speed_ts - self.time_of_last_update
        self.time_of_last_update = self.mambo.sensors.speed_ts
        for i in range(3):
            self.current_xyz_pos[i] += self.current_xyz_vel[i]*self.dt_since_last_update

        # print("\nPosition Estimate:")
        # print("\t" + str(self.current_xyz_pos))
        print("Euclidean XY Plane Distance: " + str(self.calc_xy_dist()))
        print("speed_ts: " + str(self.mambo.sensors.speed_ts))
        # self.current_state = self.current_xyz_pos + self.current_xyz_vel

    def sensor_avg_cb(self, args):
        """
        Same as sensor_cb but uses a 2 second (4 sample) running average for
        velocity rather than the sensor output.
        """
        if len(self.vels) == 4:
            del self.vels[0]

        self.vels.append([self.mambo.sensors.speed_x,
                          self.mambo.sensors.speed_y,
                          self.mambo.sensors.speed_z])

        avg_vels = [sum(a)/len(a) for a in zip(*self.vels)]

        self.dt_since_last_update = time.perf_counter() - self.time_of_last_update
        self.time_of_last_update = time.perf_counter()
        for i in range(3):
            self.current_xyz_pos[i] += avg_vels[i]*self.dt_since_last_update

        # print("\nPosition Estimate:")
        # print("\t" + str(self.current_xyz_pos))
        print("Euclidean XY Plane Distance: " + str(self.calc_xy_dist()))
        # self.current_state = self.current_xyz_pos + self.current_xyz_vel

    def calc_xy_dist(self):
        dist = 0
        dist += self.current_xyz_pos[0]**2
        dist += self.current_xyz_pos[1]**2
        dist = math.sqrt(dist)
        return dist

    def mambo_fly_function(self, mamboVision, args):
        """
        self.mambo takes off, hovers, flies forward, hovers, lands.
        mamboVision and args are only supplied because DroneVisionGUI requires
        them; all references to mamboVision are done using the class
        member self.mamboVision.
        """
        if testFlying:
            print('taking off')
            self.mambo.safe_takeoff(5)

            if self.mambo.sensors.flying_state != 'emergency':

                print("flying directly forward")
                self.mambo.fly_direct(roll=0, pitch=10, yaw=0, vertical_movement=0, duration=6)
                self.mambo.smart_sleep(3)

            print("landing")
            self.mambo.safe_land(5)
        else:
            print("Sleeeping for 15 seconds - move the self.mambo around")
            self.mambo.smart_sleep(5)

        if self.use_vision:
            print("ending vision")
            self.mamboVision.close_video()
        self.mambo.smart_sleep(5)
        print("disconnecting")
        self.mambo.disconnect()

        print("\n\nFinal Position Estimate:")
        print("\t" + str(self.current_xyz_pos))

    def run_test(self):
        print("trying to connect to self.mambo now")
        success = self.mambo.connect(num_retries=3)
        print("connected: %s" % success)

        if (success):
            # get the state information
            print("sleeping")
            self.mambo.smart_sleep(1)
            self.mambo.ask_for_state_update()
            self.mambo.smart_sleep(1)

            if self.use_vision:
                print("starting vision")
                self.mamboVision.open_video()
            else:
                self.mambo_fly_function(None, None)

testFlying = True # set this to true if you want to fly
mamboAddr = "e0:14:ad:f6:3d:fc" # BLE address
use_wifi = True # set to true if using wifi instead of BLE
use_vision = True # set to true if you want to turn on vision

if __name__ == "__main__":
    displacementCalculationTest = DisplacementCalculationTest(testFlying, mamboAddr, use_wifi, use_vision)
    displacementCalculationTest.run_test()
