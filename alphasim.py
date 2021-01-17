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

radius = 0.033 # meters

xAccel = 0
YAccel = 0
ZAccel = 0

yaw = 0 # Z
pitch = 0 # Y
roll = 0 # X

oriZ = 12
oriY = 0
oriX = 0

# TVC PARAMS #
tvcMax = 5
tvcDegS = 12.5 # deg/s
tvcCOM = 0.4

tvcZPIDOut = 0
tvcYPIDOut = 0

tvcZOut = 0
tvcYOut = 0

# PID PARAMS #
Setpoint = 0
Kp = 0.12
Ki = 0.08
Kd = 0.115

# MOTOR THRUST ARRAY #
F15 = np.array([12.50, 25.00, 16.50, 15.25, 15.00, 14.50, 14.50, 14.25, 14.00, 13.00, 13.00])
F = 15 # N

class PID(object):
    def __init__(self, _Setpoint):
        self._Setpoint = _Setpoint
        self.Error = 0
        self.pError = 0
        self.iError = 0
        self.dError = 0
        self.previousError = 0
        self.PIDOut = 0
        # Time
        self.dt = 0.065

    def compute(self, Input):
        self.Input = Input

        self.Error = self._Setpoint - Input 
        self.pError = self.Error
        self.iError = self.Error * self.dt
        self.dError = (self.Error - self.previousError) / self.dt

        PIDOut = (self.pError * Kp) + (self.iError * Ki) + (self.dError * Kd)
        return PIDOut


    def ChangeSetpoint(self, target, timeStep):
        PIDError = target - self._Setpoint
        PIDSpeed = self.SetpointRate * timeStep
        PIDError = clamp(PIDError, -PIDSPeed, PIDSpeed)
        self._Setpoint += PIDError


    def reset(self):
        self.pError = 0.0
        self.iError = 0.0
        self.dError = 0.0
        self.previousError = 0.0
        self.Error = 0.0


class Simulation(object):
    def __init__(self):
        pass
    def loop(self):
        pass
        
class Rocket(object):
    def __init__(self, pX, pY, pZ, vX, vY, vZ, fX, fY, fZ):
        pass
    
    def findPos(self, Force, posX, posY, posZ):

        
tvcZ = PID(Setpoint)
tvcY = PID(Setpoint)

# TVC Mount Angle
tvcZPIDOut = tvcZ.compute(oriZ)

# tvcXPIDOutput = tvcXPID.update(rocketOrientationX);
# tvcX.update(asin(tvcXPIDOutput / (measuredForce / whateverTheRadiusOfYourMountIs)));


# Torque
tvcZTorque = math.asin(tvcZPIDOut / (F / radius))
print(str(tvcZTorque) + " Nm")

# Angular Accel
# Angular Velocity
# Angle
# Force
# Linear Acceleration
# Velocity
# Position
