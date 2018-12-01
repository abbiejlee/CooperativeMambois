"""
    position_controller.py
    Marcus Abate | 16.30
    12/1/18

    Calculates control input required to converge to a desired xyz position
    based on current state and drone dynamics. This is an LQR model.
    Control inputs are roll, pitch, yaw commands for the pyparrot interface.

"""

class PositionController:
    def __init__(self):
        self.desired_state = []
        self.current_state = []
        self.current_rpy = []
        self.current_rpy_cmd = []
        self.current_vertical_mvmt_cmd = 0

    def current_state_cb(self, current_state):
        """
        Called at every state update in a main loop.

            current_state: holds the current state of the drone.
        """
        self.current_state = current_state

    def desired_state_cb(self, desired_state):
        """
        Called when the desired waypoint is updated in a main loop.

            desired_state: holds the desired x-y-z position of the drone as a
                3-element list.
        """
        self.desired_state = desired_state

    def calculate_rpy(self):
        """
        Determine roll and pitch to converge to desired xy-positon. Yaw is not
        used for control input.
        RPY is calculated in degrees here.
        """
        raise NotImplementedError

    def calculate_rpy_cmd(self):
        """
        Determine the roll and pitch commands to be sent to the drone. This
        essentially requires normalizing to the limits of the pyparrot
        command protocol.
        RPY commands range from -100 to 100 and are 'speed' commands.
        """
        raise NotImplementedError

    def calculate_vertical_mvmt_cmd(self):
        """
        Determine the vertical_mvmt (thrust beyond that required to hover)
        required to converge on desired z-position.
        The command is normalized for pyparrot input.
        vertical_mvmt commands range from -100 to 100 and are 'speed' commands.
        """
        raise NotImplementedError
