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

from Drone import Drone
import time

class DisplacementCalculationTest(Drone):
    def __init__(self, test_flying, mambo_addr, use_wifi, use_vision):
        super().__init__(test_flying, mambo_addr, use_wifi, use_vision)
        self.current_xyz_pos = [0, 0, 0]
        self.current_xyz_vel = [0, 0, 0]
        self.current_state = self.current_xyz_pos + self.current_xyz_vel
        self.start_measure = False

        self.mambo.set_user_sensor_callback(self.sensor_cb, args=None)
        self.time_of_last_update = time.perf_counter()
        self.vels = [] # for calculating running average
        self.dt_since_last_update = 0

    def sensor_cb(self, args):
        """
        Same as sensor_cb but uses a 2 second (4 sample) running average for
        velocity rather than the sensor output.
        """
        if self.start_measure:
            if len(self.vels) == 10:
                del self.vels[0]

            self.vels.append([self.mambo.sensors.speed_x,
                              self.mambo.sensors.speed_y,
                              self.mambo.sensors.speed_z])

            avg_vels = [sum(a)/len(a) for a in zip(*self.vels)]

            self.dt_since_last_update = time.perf_counter() - self.time_of_last_update
            self.time_of_last_update = time.perf_counter()
            for i in range(3):
                self.current_xyz_pos[i] += avg_vels[i]*self.dt_since_last_update*12
            # self.current_state = self.current_xyz_pos + self.current_xyz_vel

            # print("\nPosition Estimate:")
            # print("\t" + str(self.current_xyz_pos))
            # print('dt:',self.dt_since_last_update)
            print("Euclidean XY Plane Distance: " + str(self.calc_xy_dist()))
            print("POSX: ", self.mambo.sensors.sensors_dict['DronePosition_posx'])
            print("POSY: ", self.mambo.sensors.sensors_dict['DronePosition_posy'])
            print("POSZ: ", self.mambo.sensors.sensors_dict['DronePosition_posz'])
            print("New Dist:", (self.mambo.sensors.sensors_dict['DronePosition_posx']**2 +
                                self.mambo.sensors.sensors_dict['DronePosition_posy']**2)**0.5)
            # for i,v in self.mambo.sensors.sensors_dict.items():
                # print(i,v)
            print('\n')

    def calc_xy_dist(self):
        dist = 0
        dist += self.current_xyz_pos[0]**2
        dist += self.current_xyz_pos[1]**2
        dist = dist**0.5
        return dist

    def flight_func(self, mamboVision, args):
        """
        self.mambo takes off, hovers, flies forward, hovers, lands.
        mamboVision and args are only supplied because DroneVisionGUI requires
        them; all references to mamboVision are done using the class
        member self.mamboVision.
        """
        if self.test_flying:
            print('taking off')
            self.mambo.safe_takeoff(5)

            if self.mambo.sensors.flying_state != 'emergency':

                print("\n\n\nsensor calib\n\n\n")
                while self.mambo.sensors.speed_ts == 0:
                    continue
                self.start_measure = True

                print("\n\n\nflying directly forward\n\n\n")
                self.mambo.fly_direct(roll=0, pitch=10, yaw=0, vertical_movement=0,
                                        duration=6)
                self.mambo.smart_sleep(3)

            print("\n\n\nlanding\n\n\n")
            self.mambo.safe_land(5)
        else:
            print("\n\n\nsensor calib\n\n\n")
            while self.mambo.sensors.speed_ts == 0:
                continue
            self.start_measure = True
            print('done calibrating')
            self.mambo.smart_sleep(20)
            # print("Sleeeping for 15 seconds - move the self.mambo around")
            # self.mambo.smart_sleep(5)

        print("\n\nFinal Position Estimate:")
        print("\t" + str(self.current_xyz_pos))



test_flying = True # set this to true if you want to fly
mambo_addr = "e0:14:ad:f6:3d:fc" # BLE address
use_wifi = True # set to true if using wifi instead of BLE
use_vision = True # set to true if you want to turn on vision

if __name__ == "__main__":
    displacementCalculationTest = DisplacementCalculationTest(test_flying,
                                                            mambo_addr,
                                                            use_wifi,
                                                            use_vision)
    displacementCalculationTest.execute()
