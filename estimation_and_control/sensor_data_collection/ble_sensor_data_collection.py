"""
    ble_sensor_data_collection.py
    Marcus Abate | 16.30
    12/3/18

    Test the data being sent over BLE from the mambo drone. The purpose of this
    is to see whether any quaternion/rpy information is sent so as to be
    useful with state estimation. This data is all definitely sent over
    WiFi, but unclear whether it's sent over ble.
"""

from Drone import Drone
import time

filename = 'ble_data.txt'
write_mode = 'a'

class DroneDataCollectionTest(Drone):
    def __init__(self, test_flying, mambo_addr, use_wifi, use_vision):
        super().__init__(test_flying, mambo_addr, use_wifi, use_vision)
        self.start_measure = False
        self.f = open(filename, write_mode)
        self.pos_last_update = time.perf_counter()
        self.vel_last_update = time.perf_counter()
        self.vel_ts_last_update = 0
        self.vel_ts_dt = 0
        self.vel_dt = 0
        self.pos_dt = 0
        self.data = []

    def sensor_cb(self, args):
        """
        Prints out data sent by the mambo.
        """
        if self.start_measure:
            self.vel_dt = time.perf_counter() - self.vel_last_update
            self.pos_dt = time.perf_counter() - self.pos_last_update
            self.vel_ts_dt = self.mambo.sensors.speed_ts - self.vel_ts_last_update
            self.vel_ts_last_update = self.mambo.sensors.speed_ts
            self.vel_last_update = time.perf_counter()
            self.pos_last_update = time.perf_counter()

            sensor_readout = "\n\nmambo states:"
            sensor_readout += "\n\tflying_state: " + str(self.mambo.sensors.flying_state)
            sensor_readout += "\n\tbattery :" + str(self.mambo.sensors.battery)
            sensor_readout += "\n\tspeed (x, y, z) :" + str([self.mambo.sensors.speed_x,
                                                                self.mambo.sensors.speed_y,
                                                                self.mambo.sensors.speed_z])
            sensor_readout += "\n\taltitude: " + str(self.mambo.sensors.altitude)
            sensor_readout += "\n\tquat_w: " + str(self.mambo.sensors.quaternion_w)
            sensor_readout += "\n\tquat_x: " + str(self.mambo.sensors.quaternion_x)
            sensor_readout += "\n\tquat_y: " + str(self.mambo.sensors.quaternion_y)
            sensor_readout += "\n\tquat_z: " + str(self.mambo.sensors.quaternion_z)

            sensor_readout += "\n\tsensor_dict: "
            for key, value in self.mambo.sensors.sensors_dict.items():
                sensor_readout += "\n\t\t" + str(key) + ": " + str(value)

            sensor_readout += "\npos_dt: " + str(self.pos_dt)
            sensor_readout += "\nvel_dt: " + str(self.vel_dt)
            sensor_readout += "\nvel_ts_dt: " + str(self.vel_ts_dt)

            self.data.append(sensor_readout)

    def flight_func(self, mamboVision, args):
        """
        Takeoff, fly forward, yaw, land.
        """
        if self.test_flying:
            print('taking off')
            self.mambo.safe_takeoff(5)

            if self.mambo.sensors.flying_state != 'emergency':

                print('sensor calib')
                while self.mambo.sensors.speed_ts == 0:
                    continue
                self.start_measure = True

                print('flying directly forward')
                self.mambo.fly_direct(roll=0, pitch=10, yaw=0,
                                        vertical_movement=0, duration = 6)
                self.mambo.smart_sleep(1)

                print('yawing')
                self.mambo.fly_direct(roll=0, pitch=10, yaw=30,
                                        vertical_movement=0, duration = 5)
                self.mambo.smart_sleep(1)

            print('landing')
            self.mambo.safe_land(5)

        else:
            print('not flying')

        self.start_measure = False
        print('writing to file')
        for sensor_readout in self.data:
            self.f.write(sensor_readout)

test_flying = True
mambo_addr = "e0:14:ed:d2:3d:d1"
use_wifi = False
use_vision = False


if __name__ == '__main__':
    drone_test = DroneDataCollectionTest(test_flying, mambo_addr, use_wifi, use_vision)
    drone_test.execute()
