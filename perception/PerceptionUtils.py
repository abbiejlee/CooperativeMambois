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
