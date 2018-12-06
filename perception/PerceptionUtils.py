"""
  PerceptionUtils.py
  Abbie Lee | 16.30

  Tools for processing camera inputs and interfacing with the controls pipeline.
"""
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

def target_dist(bb_size, plot = False):
    """
    Given a bounding box size, returns estimated distance to target. Measurements
    in inches from target based on tests with a 24 x 18 in rectangle.

    bb_size (int): number of pixels in the bounding box

    Returns: a float representing estimated distance from target using a
             fitted to the bb_size -> distance data
    """

    meas_dist = np.array([i for i in range(10, 111, 10)])
    meas_bb = np.array([1272789, 613795, 304475, 206437, 123487, 75603, 63869,
                        49215, 38115, 33741, 28665])

    # TODO(abbielee): find form of function
    def power(x, a, b):
        """
        Define form of function to fit to data (power function)
        f = ax^b
        """
        return a*x**b


    params = curve_fit(power, meas_bb, meas_dist)
    a, b = params[0]

    # plot fit
    if plot:
        xp = np.linspace(20000, 2000000, 1000)
        _ = plt.plot(meas_bb, meas_dist, '.', xp, power(xp, a, b), '--')
        plt.title("Distance from Target vs. Bounding Box Size")
        plt.xlabel("Bounding Box Size (pixels)")
        plt.ylabel("Distance from Target (in)")
        plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
        plt.show()

    return p(bb_size)
