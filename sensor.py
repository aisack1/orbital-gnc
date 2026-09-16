# ----- SENSOR CLASS -----
# PARAMS: Takes in the scale and size of values.
# FUNCTIONS:
#   - Measure: Uses the scale and size to add noise to the true values inputted.

import numpy as np
from numpy import random

class Sensor:
    def __init__(self, scale, size):
        self.scale = scale
        self.size = size

    def measure(self, true_value):
        noise = random.normal(loc=0, scale=self.scale, size=self.size)
        sensor_value = true_value + noise

        return sensor_value