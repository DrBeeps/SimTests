import numpy as np
import matplotlib.pyplot as plt
import math

F = 15 # thrust avg
r = 0.033# radius

theta = 0 # angle between force and moment arm


torque = r * F * math.sine(theta)


