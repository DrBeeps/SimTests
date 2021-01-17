import numpy as np
import matplotlib
import time

timer = 0
setpoint = 100
sim_time = 10
timestep = 0.065
initialX = 0
initialY = -100
mass = 1 # kg
max_thrust = 15 # N
g = -9.8
Vi = 0 # initial velocity
Yi = 0 # initial height


def Simulation(object):
    def __init__(self):
        self.TestRocket = Rocket()
        self.sim = True
        self.timer = 0
        self.poses = np.array([])
		self.times = np.array([])
		self.thrust = np.array([])

    def loop(self):

        while(self.sim):
            thrust = 10
            self.TestRocket.setDdy(thrust)
            self.TestRocket.setDy()
            self.TestRocket.setY()

            time.sleep(timestep)
            self.timer += 1
            
            if self.timer > sim_time:
                self.sim = False
            elif self.TestRocket.getY() > 800:
                self.sim = False
            elif self.TestRocket.getY() < -800:
                self.sim = False
    

class Rocket(object):
    def __init__(self):
        global Rocket
        self.ddy = 0
        self.dy = Vi
        self.y = Yi

    def setDdy(self, thrust):
        self.ddy = g + thrust / mass

    def getDdy(self):
        return self.ddy

    def setDy(self):
        self.dy += self.ddy

    def getDy(self):
        return self.dy

    def setY(self):
        rocket.sety(self.y + self.dy)

    def getY(self):
        return self.y


def main():
    sim = Simulation()
    sim.loop()

main()
