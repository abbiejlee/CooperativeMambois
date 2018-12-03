"""
    detection_drone_test.py
    Marcus Abate | 16.30
    12/2/18

    This script tests usage of the generalized DetectionDrone class so that
    testing and iteration can happen faster.
"""

# from DetectionDrone import DetectionDrone
# import cv2
#
# def flight_func(self, mamboVision, args):
#     """
#     mambo takes off, flies forward, flies back and lands.
#     """
#     if self.testFlying:
#         print('taking off')
#         self.mambo.safe_takeoff(5)
#
#         if self.mambo.sensors.flying_state != 'emergency':
#
#             # this part is required to prevent faulty readings from contributing to position estimate
#             print("sensor calib")
#             while self.mambo.sensors.speed_ts == 0:
#                 continue
#             self.start_measure = True
#
#             print("flying directly forward")
#             self.mambo.fly_direct(roll=0, pitch=20, yaw=0, vertical_movement=0,
#                                     duration=4)
#             self.mambo.smart_sleep(3)
#
#             print("flying directly backward")
#             self.mambo.fly_direct(roll=0, pitch=-20, yaw=0, vertical_movement=0,
#                                     duration=4)
#
#         print("\n\n\nlanding\n\n\n")
#         self.mambo.safe_land(5)
#
#     else:
#         print("testFlying is False, no takeoff.")
#
# def sensor_cb(self, args):
#     """
#     print speeds.
#     """
#     print([self.mambo.sensors.speed_x,
#             self.mambo.sensors.speed_y,
#             self.mambo.sensors.speed_z])
#
# def vision_cb(self, args):
#     """
#     save latest image.
#     """
#     img = self.mamboVision.get_latest_valid_picture()
#
#     if img is not None:
#         cv2.imwrite('last_capture.png', ing)
#     else:
#         print('image is none')
#
#
# testFlying = True
# mamboAddr = "e0:14:ed:d2:3d:d1"
# use_wifi = False
# use_vision = False
#
# if __name__ == "__main__":
#     detection_drone = DetectionDrone(testFlying, mamboAddr, flight_func,
#                                         use_vision, vision_cb, sensor_cb)
#     detection_drone.execute()

from DetectionDrone import DetectionDrone
import cv2

class DetectionDroneTest(DetectionDrone):
    def flight_func(self, mamboVision, args):
        """
        mambo takes off, flies forward, flies back and lands.
        """
        if self.test_flying:
            print('taking off')
            self.mambo.safe_takeoff(5)

            if self.mambo.sensors.flying_state != 'emergency':

                # this part is required to prevent faulty readings from contributing to position estimate
                print("sensor calib")
                while self.mambo.sensors.speed_ts == 0:
                    continue
                self.start_measure = True

                print("flying directly forward")
                self.mambo.fly_direct(roll=0, pitch=20, yaw=0, vertical_movement=0,
                                        duration=4)
                self.mambo.smart_sleep(3)

                print("flying directly backward")
                self.mambo.fly_direct(roll=0, pitch=-20, yaw=0, vertical_movement=0,
                                        duration=4)

            print("\n\n\nlanding\n\n\n")
            self.mambo.safe_land(5)

        else:
            print("testFlying is False, no takeoff.")

    def sensor_cb(self, args):
        """
        print speeds.
        """
        print([self.mambo.sensors.speed_x,
                self.mambo.sensors.speed_y,
                self.mambo.sensors.speed_z])

    def vision_cb(self, args):
        """
        save latest image.
        """
        img = self.mamboVision.get_latest_valid_picture()

        if img is not None:
            cv2.imwrite('last_capture.png', ing)
        else:
            print('image is none')


test_flying = True
mambo_addr = "e0:14:ed:d2:3d:d1"
use_wifi = False
use_vision = False

if __name__ == "__main__":
    detection_drone = DetectionDroneTest(test_flying, mambo_addr, use_wifi, use_vision)
    detection_drone.execute()
