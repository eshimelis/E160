# UKF Testing

from E160_environment import *
from E160_state import E160_state 
import numpy as np
from E160_UKF import *
from E160_UKF import E160_UKF as ukf


environment = E160_environment()
robotRadius = 0.14149/2
robotWidth = 2*robotRadius
wheelRadius = 0.06955/2
encoderResolution = 1440
spaceDim = 3

# create two-dimensional mean vector in numpy
mean = E160_state(0, 0, 0)
covariance = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.2]])
# covariance = np.array([[1, 1, 1], [1, 1, 1], [1, 1, 1]])

ukf = ukf(environment, spaceDim, robotWidth, wheelRadius, encoderResolution)

print(ukf.GenerateSigmaPoints(mean, covariance))