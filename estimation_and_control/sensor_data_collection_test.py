"""
    sensor_data_collection_test.py
    Marcus Abate | 16.30
    12/2/18

    Test that stores sensor data readouts to a file for analysis later. The
    analysis will be done to characterize the noise in the sensors for use with
    the Kalman filter implementation.
"""

from Drone import Drone

filename = "data_collecton.txt"
write_mode = "a" # set to "a" to append. set to "w" to overwrite.

class DroneDataCollectionTest(Drone):
    def __init__(self, test_flying, mambo_addr, use_wifi, use_vision):
        super().__init__(test_flying, mambo_addr, use_wifi, use_vision)
        self.start_measure = False
        self.pos_x = []
        self.pos_y = []
        self.pos_z = []
        self.vel_x = []
        self.vel_y = []
        self.vel_z = []

    def sensor_cb(self, args):
        if self.start_measure:
            # print('appending data')
            self.pos_x.append(self.mambo.sensors.sensors_dict['DronePosition_posx'])
            self.pos_y.append(self.mambo.sensors.sensors_dict['DronePosition_posy'])
            self.pos_z.append(self.mambo.sensors.sensors_dict['DronePosition_posz'])
            self.vel_x.append(self.mambo.sensors.speed_x)
            self.vel_y.append(self.mambo.sensors.speed_y)
            self.vel_z.append(self.mambo.sensors.speed_z)

    def flight_func(self, mamboVision, args):
        """
        takeoff, hover, fly forward for 5 seconds, fly left for 5 seconds,
        fly right for 5 seconds, fly back for 5 seconds.
        """
        if self.test_flying:
            print('taking off')
            self.mambo.safe_takeoff(5)

            if self.mambo.sensors.flying_state != 'emergency':

                print('sensor calibrating')
                while self.mambo.sensors.speed_ts == 0:
                    continue
                self.start_measure = True

                print('flying forward')
                self.mambo.fly_direct(roll=0, pitch=10, yaw=0,
                                        vertical_movement=0, duration=5)
                print('flying left')
                self.mambo.fly_direct(roll=-10, pitch=0, yaw=0,
                                        vertical_movement=0, duration=5)

                print('flying right')
                self.mambo.fly_direct(roll=--10, pitch=0, yaw=0,
                                        vertical_movement=0, duration=5)

                print('flying backward')
                self.mambo.fly_direct(roll=0, pitch=-10, yaw=0,
                                        vertical_movement=0, duration=5)

                self.mambo.smart_sleep(3)

            print('landing')
            self.mambo.safe_land(5)

        else:
            print ('not test_flying')

        print('writing to file...')
        f = open(filename, write_mode)
        f.write('\n\nNew Info:\n')
        f.write('pos_x: ' + str(self.pos_x))
        f.write('pos_y: ' + str(self.pos_y))
        f.write('pos_z: ' + str(self.pos_z))
        f.write('vel_x: ' + str(self.vel_x))
        f.write('vel_y: ' + str(self.vel_y))
        f.write('vel_z: ' + str(self.vel_z))

test_flying = True # set this to true if you want to fly
mambo_addr = "e0:14:ad:f6:3d:fc" # BLE address
use_wifi = True # set to true if using wifi instead of BLE
use_vision = False # set to true if you want to turn on vision

if __name__ == "__main__":
    drone_test = DroneDataCollectionTest(test_flying, mambo_addr, use_wifi,
                                            use_vision)
    drone_test.execute()
