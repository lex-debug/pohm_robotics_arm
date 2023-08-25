import numpy as np
import matplotlib.pyplot as plt
import math
import itertools

DEG60 = math.pi / 3
DEG90 = math.pi / 2
DEG120 = math.pi / 3 * 2
numOfPoints = 10

# Joint 1 is a revolute joint
# Angle is measured in radian
j1Theta = np.linspace(1.0624, 1.9876, num=numOfPoints)
j1Theta_prime = j1Theta - DEG60 # This is referred to the ground

# Joint 2 is a prismatic joint
# The linear extension is measured in mm
j2X = np.linspace(2190.05, 3690.05, num=numOfPoints)

# Joint 4 is a revolute joint
j3Theta = np.linspace(0.7798, 2.9671, num=numOfPoints)
alphaTheta = DEG90 - j1Theta_prime
betaTheta = j3Theta - alphaTheta
j3Theta_prime = DEG90 - betaTheta # as mentioned earlier, the theta_prime is the angle with the ground as reference 

# The link itself is a motor, namely SE3C, however, it is always 90 degree to the link 3
se3c_theta = DEG90 - j3Theta_prime

# Joint 5 is a prismatic joint
# As this link is always parallel to the link 3, the angle will be the same
j5X = np.linspace(652.94, 902.94, num=numOfPoints)
j5Theta_prime = j3Theta_prime

# length of each link (mm)
l0 = 1096.077
l1 = j2X
l2 = 950
l3 = 233.5
l4 = j5X

c1 = l0 * np.cos(DEG120)
c2 = l1[:, np.newaxis] * np.cos(j1Theta_prime)
c3 = l2 * np.cos(j3Theta_prime)
c4 = l3 * np.cos(se3c_theta)
c5 = l4[:, np.newaxis] * np.cos(j5Theta_prime)

s1 = l0 * np.sin(DEG120)
s2 = l1[:, np.newaxis] * np.sin(j1Theta_prime)
s3 = l2 * np.sin(j3Theta_prime)
s4 = l3 * np.sin(se3c_theta)
s5 = l4[:, np.newaxis] * np.sin(j5Theta_prime)

x = c1 + c2
x = x.reshape(-1)[:, np.newaxis] + c3
x = x.reshape(-1)[:, np.newaxis] + c4
x = x.reshape(-1)[:, np.newaxis] + c5.reshape(-1)
x = x.reshape(-1)
y = s1 + s2
y = y.reshape(-1)[:, np.newaxis] + s3
y = y.reshape(-1)[:, np.newaxis] + s4
y = y.reshape(-1)[:, np.newaxis] + s5.reshape(-1)
y = y.reshape(-1)

print(x)
print(y)
print(len(x))
print(len(y))
i = 0
j1Theta_prime_data = []
j2X_data = []
j3Theta_prime_data = []
se3c_theta_data = []
j5X_data = []
j5Theta_prime_data = []
theta_x_combinations = itertools.product(j1Theta_prime, j2X, j3Theta_prime, se3c_theta, j5X, j5Theta_prime)
for combination in theta_x_combinations:
    j1Theta_prime_data.append(combination[0])
    j2X_data.append(combination[1])
    j3Theta_prime_data.append(combination[2])
    se3c_theta_data.append(combination[3])
    j5X_data.append(combination[4])
    j5Theta_prime_data.append(combination[5])
    i = i + 1

print(j1Theta_prime_data)
print(j2X_data)
print(j3Theta_prime_data)
print(se3c_theta_data)
print(j5X_data)
print(j5Theta_prime_data)

# coordinates = []
# for i, j in zip(x, y):
#     coordinates.append((i, j))

# print(coordinates)
# plt.scatter(x, y)
# plt.show()