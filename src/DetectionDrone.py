"""
    DetectionDrone.py
    Marcus Abate | 16.30
    12/1/18

    This class encapsulates many of the standard methods and objects used to
    run tests and scripts on the Parrot Mambo drone with vision. WiFi is assumed
    for this implementation.
"""

from pyparrot.Minidrone import Mambo
from pyparrot.DroneVisionGUI import DroneVisionGUI

class DetectionDrone:
    def __init__(self, testFlying, mamboAddr, flight_func, use_vision=True, vision_cb=None, sensor_cb=None):
        """
        testFlying:     bool : run flight code if True
        mamboAddr:      str  : BLE address of drone
        flight_func:    func : code to be run during drone flight
        use_vision:     bool : turn on DroneVisionGUI module if True
        vision_cb:      func : callback function for vision frames
        sensor_cb:      func : callback function for sensor updates
        """
        self.testFlying = testFlying
        self.use_vision = use_vision
        self.mamboAddr = mamboAddr
        self.flight_func = flight_func
        self.vision_cb = vision_cb
        self.sensor_cb = sensor_cb
        self.mambo = Mambo(self.mamboAddr, use_wifi=True)
        if self.sensor_cb is not None:
            self.mambo.set_user_sensor_callback(self.sensor_cb, args=None)
        if self.useVision:
            self.mamboVision = DroneVisionGUI(self.mambo, is_bebop=False, buffer_size=200,
                                     user_code_to_run=self.flight_func, user_args=None)
        if self.vision_cb is not None:
            self.mamboVision.set_user_callback_function(self.vision_cb, user_callback_args=None)

    def execute(self):
        """
        Connects to drone and executes relevant functions passed in through the
        constructor.
        """
        print("trying to connect to self.mambo now")
        success = self.mambo.connect(num_retries=3)
        print("connected: %s" % success)

        if (success):
            self.mambo.smart_sleep(1)
            self.mambo.ask_for_state_update()
            self.mambo.smart_sleep(1)

            if self.use_vision:
                print("starting vision")
                self.mamboVision.open_video()
            else:
                self.mambo_fly_function(self.mamboVision, None)
