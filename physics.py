import numpy as np
import matplotlib.pyplot as plt

# Angular Velocity
time = np.random.uniform(low=0, high=1, size=(50))
imuAngularVelocity = np.random.uniform(low=0, high=1, size=(50))

integratedImuAngularVelocity = time * imuAngularVelocity

plt.plot(integratedImuAngularVelocity)
plt.ylabel('IMU Angular Velocity (Z)')
plt.show()

def globalAccel(self):
    