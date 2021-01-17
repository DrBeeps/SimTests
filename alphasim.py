import numpy as np
import math
import matplotlib.pyplot as plt
import time

# GLOBAL PARAMS #
g = -9.8 # m/s^2

# ROCKET  PARAMS #
mass = 0.6095
mmoi = 0.09452292736
avgThrust = 15

xAccel = 0
YAccel = 0
ZAccel = 0

yaw = 0 # Z
pitch = 0 # Y
roll = 0 # X

# TVC PARAMS #
tvcMax = 5
tvcDegS = 12.5 # deg/s
tvcCOM = 0.4

tvcZPIDOut = 0
tvcYPIDOut = 0

tvcZOut = 0
tvcYOut = 0

# PID PARAMS #
Kp = 0.12
ki = 0.01
kD = 0.08

# MOTOR THRUST ARRAY #
F15 = np.array([12.50, 25.00, 16.50, 15.25, 15.00, 14.50, 14.50, 14.25, 14.00, 13.00, 13.00])

plt.plot(F15)
plt.ylabel("F15 Motor Thrust Curve")
plt.show()

def PID(self, Input, Setpoint):
    self.Error - self.Setpoint - Input
    self.pError = self.Error
    self.iError = self.Error * self.dt
    self.dError = (self.Error - self.previousError) / self.dt

    PIDOut = (self.pError * Kp) + (self.iError * Ki) + (self.dError * Kd)
    return PIDOut


def ChangeSetpoint(self, target, timeStep):
    PIDError = target - self.Setpoint
    PIDSpeed = self.SetpointRate * timeStep
    PIDError = clamp(PIDError, -PIDSPeed, PIDSpeed)
    self.Setpoint += PIDError


def reset(self):
    self.pError = 0.0
    self.iError = 0.0
    self.dError = 0.0
    self.previousError = 0.0
    self.Error = 0.0


class Simulation(object):
    def __init__(self):

    def loop(self):
        
class Rocket(object):
    def __init__(self):
        global Rocket
        
    

# TVC Mount Angle
tvcZPIDOut = 


# tvcXPIDOutput = tvcXPID.update(rocketOrientationX);
# tvcX.update(asin(tvcXPIDOutput / (measuredForce / whateverTheRadiusOfYourMountIs)));


# Torque
# Angular Accel
# Angular Velocity
# Angle

# Force
# Linear Acceleration
# Velocity
# Position