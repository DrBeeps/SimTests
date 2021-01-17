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

gravity = [-9.8, 0.0, 0.0]

yaw = 0 # Z
pitch = 0 # Y
roll = 0 # X

oriZ = 12
oriY = 0
oriX = 0

# TVC PARAMS #
tvcMax = 5
tvcDegS = 12.5 # deg/s
tvcMomentArm = 0.4 # COM to TVC dist

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
        self.Error = target - self._Setpoint
        PIDSpeed = self.SetpointRate * timeStep
        self.Error = clamp(self.Error, -PIDSPeed, PIDSpeed)
        self._Setpoint += self.Error


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
    def __init__(self, pX, pY, pZ, vX, vY, vZ, fX, fY, fZ, _mass, tvcZAngle, tvcYAngle, _tvcMomentArm):
        self.pX = pX
        self.pY = pY
        self.pZ = pZ
        self.vX = vX
        self.vY = vY
        self.vZ = vZ
        self.fX = fX
        self.fY = fY
        self.fZ = fZ
        self._mass = _mass
        self.tvcZAngle = tvcZAngle
        self.tvcYAngle = tvcYAngle
    
    def calcForce(self):
        self.tvcZAngle *= math.pi / 180 # degrees to radians
        self.tvcYAngle *= math.pi / 180

        self.tvcZAngle = math.sin(self.tvcZAngle) * F
        self.tvcYAngle = math.cos(self.tvcYAngle) * F
        return tvcZAngle * 180 / math.pi

    def updatePhysics(self, dt):
        self.dt = dt

        self.fX += self.fZ * gravity[0]
        self.fY += self.fY * gravity[1]
        self.fZ += self.fX * gravity[2]

        # apply motor thrust to force vector here?

        self.vX += self.fX / self._mass * self.dt
        self.vY += self.fY / self._mass * self.dt
        self.vZ += self.fZ / self._mass * self.dt

        self.pX += self.vX * self.dt
        self.pY += self.vY * self.dt
        self.pZ += self.vZ * self.dt
        
    def getPX(self):
        return self.pX
    def getPY(self):
        return self.pY
    def getPZ(self):
        return self.pZ


        
tvcZ = PID(Setpoint)
tvcY = PID(Setpoint)

# TVC Mount Angle
tvcZPIDOut = tvcZ.compute(oriZ)
tvcYPIDOut = tvcY.compute(oriY)

# Rocket Physics
testRocket = Rocket(0, 0, 0, 0, 0, 0, 0, 0, 1, mass, tvcZPIDOut, tvcYPIDOut, tvcMomentArm)

testRocket.updatePhysics(0.065)
# print(str(testRocket.getPX()) + " " + str(testRocket.getPY()) + " " + str(testRocket.getPZ()))
print(testRocket.calcForce())

# tvcXPIDOutput = tvcXPID.update(rocketOrientationX);
# tvcX.update(asin(tvcXPIDOutput / (measuredForce / whateverTheRadiusOfYourMountIs)));


# Torque
tvcZTorque = math.asin(tvcZPIDOut / (F / radius))
# print(str(tvcZTorque) + " Nm")



# Angular Accel


# Angular Velocity
# Angle
# Force
# Linear Acceleration
# Velocity
# Position
