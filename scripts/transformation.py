#!/usr/bin/env python3
import numpy as np
import math

def angle_with_z(m):
    m = np.array(m)
    r = np.math.sqrt(np.sum(m*m))
    angle = math.degrees(math.acos(m[2]/r))
    return angle 