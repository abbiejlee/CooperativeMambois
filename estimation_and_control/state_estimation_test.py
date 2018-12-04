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

from Drone import Drone

class DroneStateEstimationTest(Drone):
    def sensor_cb(self, args):
        """
        Called whenever a drone sensor is updated.
        Prints the state to terminal.
        """
        sensor_readout = "\nmambo states:"
        sensor_readout += "\n\tflying_state: " + str(self.mambo.sensors.flying_state)
        sensor_readout += "\n\tbattery :" + str(self.mambo.sensors.battery)
        sensor_readout += "\n\tspeed (x, y, z) :" + str([self.mambo.sensors.speed_x,
                                                            self.mambo.sensors.speed_y,
                                                            self.mambo.sensors.speed_z])
        sensor_readout += "\n\taltitude: " + str(self.mambo.sensors.altitude)
        print(sensor_readout)


    def flight_func(self, mamboVision, args):
        """
        self.mambo takes off, hovers, flies forward, hovers, flies back, lands.
        self.mambo will offload its current state as fast as possible to be
        displayed at terminal and stored offboard.
        mamboVision and args are only supplied because DroneVisionGUI requires
        them; all references to mamboVision are done using the class
        member self.mamboVision.
        """
        if self.test_flying:
            print('taking off')
            self.mambo.safe_takeoff(5)

            if self.mambo.sensors.flying_state != 'emergency':
                print('flying directly up')
                self.mambo.fly_direct(roll=0, pitch=0, yaw=0, vertical_movement=15, duration=4)
                self.mambo.smart_sleep(5)

                print("flying directly forward")
                self.mambo.fly_direct(roll=0, pitch=25, yaw=0, vertical_movement=0, duration=3)
                self.mambo.smart_sleep(3)

                print("flying directly backward")
                self.mambo.fly_direct(roll=0, pitch=-25, yaw=0, vertical_movement=0, duration=3)
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

test_flying = True # set this to true if you want to fly
mambo_addr = "e0:14:a7:ed:3d:fc" # BLE address
use_wifi = False # set to true if using wifi instead of BLE
use_vision = False # set to true if you want to turn on vision

if __name__ == "__main__":
    droneStateEstimationTest = DroneStateEstimationTest(test_flying, mambo_addr, use_wifi, use_vision)
    droneStateEstimationTest.execute()
