"""
    drone_color_seg_test.py
    Marcus Abate | 16.30

    Parrot self.mambo takes off, yaws slowly until it recognizes the color orange.
    Does a flip and then lands.
"""

from pyparrot.Minidrone import Mambo
from pyparrot.DroneVisionGUI import DroneVisionGUI
from ColorSegmentation import cd_color_segmentation
import cv2

class DroneColorSegTest:
    def __init__(self, testFlying, mamboAddr, use_wifi):
        self.bb = [0, 0, 0, 0]
        self.bb_threshold = 4000
        self.bb_trigger = False

        self.testFlying = testFlying
        self.mamboAddr = mamboAddr
        self.use_wifi = use_wifi
        self.mambo = Mambo(self.mamboAddr, self.use_wifi)
        self.mamboVision = DroneVisionGUI(self.mambo, is_bebop=False, buffer_size=200,
                                     user_code_to_run=self.mambo_fly_function, user_args=None)

    def color_segmentation(self, args):
        img = self.mamboVision.get_latest_valid_picture()

        if img is not None:
            [((x1, y1), (x2, y2)), ln_color] = cd_color_segmentation(img)
            self.bb = [x1, y1, x2, y2]

            bb_size = self.get_bb_size()
            print('bb_size:',bb_size)
            cv2.imwrite('test_file.png', img)
            if bb_size >= self.bb_threshold:
                print('orange detected')
                self.bb_trigger = True
            # else:
                # self.bb_trigger = False
        else:
            print('image is None')

    def get_bb_size(self):
        ''' returns area of self.bb (bounding box) '''
        return (self.bb[2] - self.bb[0])*(self.bb[3] - self.bb[1])

    def mambo_fly_function(self, mamboVision, args):
        """
        self.mambo takes off and yaws slowly in a circle until the vision processing
        detects a large patch of orange. It then performs a flip and lands.
        """

        if (testFlying):
            print("taking off!")
            self.mambo.safe_takeoff(5)

            if (self.mambo.sensors.flying_state != "emergency"):
                print("flying state is %s" % self.mambo.sensors.flying_state)
                print("Flying direct: going up")
                self.mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=15, duration=2)

                print("flying state is %s" % self.mambo.sensors.flying_state)
                print("yawing slowly")
                for deg in range(36):
                    self.mambo.turn_degrees(10)
                    if self.bb_trigger:
                        break
                    self.mambo.smart_sleep(1)

                print("flying state is %s" % self.mambo.sensors.flying_state)
                print("flip left")
                success = self.mambo.flip(direction="left")
                print("self.mambo flip result %s" % success)
                self.mambo.smart_sleep(2)

            print("landing")
            print("flying state is %s" % self.mambo.sensors.flying_state)
            self.mambo.safe_land(5)
        else:
            print("Sleeeping for 15 seconds - move the self.mambo around")
            self.mambo.smart_sleep(15)

        # done doing vision demo
        print("Ending the sleep and vision")
        self.mamboVision.close_video()

        self.mambo.smart_sleep(5)

        print("disconnecting")
        self.mambo.disconnect()

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

            print("Preparing to open vision")
            self.mamboVision.set_user_callback_function(self.color_segmentation, user_callback_args=None)
            self.mamboVision.open_video()

# set this to true if you want to fly
testFlying = True
# BLE address:
mamboAddr = "e0:14:d0:63:3d:d1"
use_wifi = True

if __name__ == "__main__":
    droneColorSegTest = DroneColorSegTest(testFlying, mamboAddr, use_wifi)
    droneColorSegTest.run_test()
