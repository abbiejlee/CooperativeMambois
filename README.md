# CooperativeMambois

*UNDER CONSTRUCTION*

## DEVEL NOTES

* Always remember to develop on your own test branch before merging to master. Talk to Marcus if you have questions about this.
* ``__init__.py`` is added to every subdirectory so that modules defined in those directories can inherit some of the same functionality. Leave that in there for now.

``src/Drone.py`` is a useful tool for quick iteration and testing. You can use this by copying it into the directory you are working in and then calling it as an import in your script:

```
from Drone import Drone
```

at the top of your script. Then, you define a class for your test script (named TestDrone, for instance) that *inherits* the Drone class:

```
class TestDrone(Drone):
   # your code here
```

You must then redefine the following functions within your class:

```
def flight_func():
    # your code here

def vision_cb():
    # your code here

def sensor_cb():
    # your code here
```

Your main method should almost always look like this:

```
if __name__ == "__main__":
    droneTest = DroneTest(args)
    droneTest.execute()
```

Note that you only have to redefine the two callbacks if you're planning on using them. ``flight_func`` must be redefined always. See the comments in ``Drone.py`` to get a better idea of how this works, or see ``src/detection_drone_test.py`` for a working example.
