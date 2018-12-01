"""
    state_estimation_test.py
    Marcus Abate | 16.30

    Test to grab state estimates off the mambo onboard state estimator via
    pyparrot. The purpose of this test is to determine the best way to
    get the current state of the drone.
    This test also has the drone take off, fly forward, and then return to
    its original start location. It can be used to determine how accurate state
    estimation is and how bad the drift is WITHOUT a custom position controller.
    This also runs the forward facing camera, so that tests are more
    realistic. The vision frames arent processed.
"""

from pyparrot.Minidrone import Mambo
from pyparrot.DroneVisionGUI import DroneVisionGUI

class DroneStateEstimationTest:
    def __init__(self, testFlying, mamboAddr, use_wifi, use_vision):
        self.testFlying = testFlying
        self.mamboAddr = mamboAddr
        self.use_wifi = use_wifi
        self.use_vision = use_vision
        self.mambo = Mambo(self.mamboAddr, self.use_wifi)
        self.mambo.set_user_sensor_callback(self.sensor_cb, args=None)
        if self.use_vision:
            self.mamboVision = DroneVisionGUI(self.mambo, is_bebop=False, buffer_size=200,
                                     user_code_to_run=self.mambo_fly_function, user_args=None)

    def vision_cb(self, args):
        """
        Any optional vision frame handling can be done here if desired.
        Currently not implemented.
        """
        pass

    def sensor_cb(self, args):
        """
        Called whenever a drone sensor is updated.
        Prints the state to terminal.
        """
        # self.mambo.ask_for_state_update()
        sensor_readout = "\nmambo states:"
        sensor_readout += "\n\tflying_state: " + str(self.mambo.sensors.flying_state)
        sensor_readout += "\n\tbattery :" + str(self.mambo.sensors.battery)
        sensor_readout += "\n\tspeed (x, y, z) :" + str([self.mambo.sensors.speed_x,
                                                            self.mambo.sensors.speed_y,
                                                            self.mambo.sensors.speed_z])
        sensor_readout += "\n\taltitude: " + str(self.mambo.sensors.altitude)
        print(sensor_readout)


    def mambo_fly_function(self, mamboVision, args):
        """
        self.mambo takes off, hovers, flies forward, hovers, flies back, lands.
        self.mambo will offload its current state as fast as possible to be
        displayed at terminal and stored offboard.
        mamboVision and args are only supplied because DroneVisionGUI requires
        them; all references to mamboVision are done using the class
        member self.mamboVision.
        """
        if testFlying:
            print('taking off')
            self.mambo.safe_takeoff(5)

            if self.mambo.sensors.flying_state != 'emergency':
                print('flying directly up')
                self.mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=15, duration=4)
                self.mambo.smart_sleep(5)

                # print("flying directly forward")
                # self.mambo.fly_direct(roll=0, pitch=25, yaw=0, vertical_movement=0, duration=3)
                # self.mambo.smart_sleep(3)
                #
                # print("flying directly backward")
                # self.mambo.fly_direct(roll=0, pitch=-25, yaw=0, vertical_movement=0, duration=3)
                # self.mambo.smart_sleep(3)
                # self.mambo.smart_sleep(5) # commented out above and using this b/c I'm flying in my room rn

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
                self.mamboVision.set_user_callback_function(self.vision_cb, user_callback_args=None)
                self.mamboVision.open_video()
            else:
                self.mambo_fly_function(None, None)

testFlying = True # set this to true if you want to fly
mamboAddr = "e0:14:ad:f6:3d:fc" # BLE address
use_wifi = True # set to true if using wifi instead of BLE
use_vision = True # set to true if you want to turn on vision

if __name__ == "__main__":
    droneStateEstimationTest = DroneStateEstimationTest(testFlying, mamboAddr, use_wifi, use_vision)
    droneStateEstimationTest.run_test()
