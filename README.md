# CooperativeMambois

Marcus Abate, Abbie Lee, Kimberly Jung, Travis Hank, Bradley Jomard, Cecilia McCormick, Shakti Shaligram, Tingxiao Sun

MIT 16.30 Final Project | Fall 2018

[![Demo Video Here](http://img.youtube.com/vi/2mRfMGzRBg0/0.jpg)](http://www.youtube.com/watch?v=2mRfMGzRBg0)

[Demo Video Here](https://www.youtube.com/watch?v=2mRfMGzRBg0&feature=youtu.be)

This project works with two [Parrot Mambo Drones](https://www.parrot.com/us/drones/parrot-mambo-fpv) to detect targets in a room and then shoot them.

The challenge with the Mambo platform is that the drone can have either a forward-facing camera or a gun mounted. It cannot have both. For this reason, one drone (the Detection Drone) is tasked with finding the target using the forward-facing camera. The other drone (the Shooter Drone) then flies to the target location gathered by the Detection Drone and then shoots the target.

The targets are orange and identified using color segmentation with [OpenCV](https://opencv.org/) for python. The two drones are controlled via [pyparrot](https://github.com/amymcgovern/pyparrot).

## Installation

This package requires [pyparrot](https://github.com/amymcgovern/pyparrot) to control the drones. Installation can be found [in the pyparrot docs](https://pyparrot.readthedocs.io/en/latest/index.html). The package is based on python3, so make sure to ``pip3 install`` every package instead of ``pip install``.

The detection drone works via WiFi, however the shooter drone requires a Bluetooth connection. Currently, this feature is only supported on Linux systems.

The detection drone handles color segmentation with OpenCv. To intall:
```
pip3 install opencv-python
pip3 install imutils
```
[pyparrot](https://github.com/amymcgovern/pyparrot) has example code that can be run to test your installation before using test scripts in this package. In particular, try ``/examples/demoMamboVisionGui.py``.

## Module Descriptions

Under ``/Modules`` there are several important files for use with the package.

* ``Drone.py`` encapsulates many of the common methods required to run test scripts with pyparrot.
* ``KalmanFilter.py`` is a generalized and Mambo-specific implementation of Kalman Filter, to be run at every sensor callback for state estimation.
* ``PositionController.py`` is a generalized and Mambo-specific implementation of LQR, to get commands for the drone for trajectory tracking.
* ``ColorSegmentation.py`` has tools for vision handling for the package.

This repo doesn't install to your python ``/dist-packages`` or ``/site-packages`` folder, so to use any of these modules you will have to copy them to the directory of your script.

## Demo

A partial demo is available under ``/src``. The ``detection_script.py`` script can be run to fly the detection drone. It flies vertically until it detects the orange target, and then saves the xyz coordinates of its position upon target detection to ``shoot_here.csv`` as a 3-element list. The drone then flies away and lands safely.
``shooter_script.py`` can then be run to fly the shooter drone equipped with the gun. It navigates to the location defined in ``shoot_here.csv`` by the detection drone and then shoots the target.

The full demo will involve the detection drone taking off and searching the room in all three axes (instead of being constrained to the z-axis only) to find the target. Currently, we are limited in the state estimation side as the Parrot Mambo does not provide access to the requisite sensor readings such as accelerometer and gyroscope readings. The current demo uses a "position sensor" with little real documentation in the Mambo SDK.

## Other Code

* ``/legacy`` contains code for previous demos as well as initial project tests.
* ``/search`` contains an initial pass at path planning for the full demo.
* ``/perception`` contains code for handling vision frames for the full demo.
* ``/estimation_and_control`` contains code for testing sensors, estimation processes, and control methods.
