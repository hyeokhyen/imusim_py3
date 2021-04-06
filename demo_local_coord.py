from imusim.platforms.imus import IdealIMU
from imusim.io.bvh import BVHLoader
from imusim.simulation.base import Simulation
from imusim.behaviours.imu import BasicIMUBehaviour
from imusim.trajectories.rigid_body import SplinedBodyModel

import numpy as np

file_bvh = './141_01.bvh'
CM_TO_M_CONVERSION = 0.01

with open(file_bvh, 'r') as bvhFile:
	loader = BVHLoader(bvhFile, CM_TO_M_CONVERSION)
	loader._readHeader()
	loader._readMotionData()
	model = loader.model

splinedModel = SplinedBodyModel(model)
startTime = splinedModel.startTime
endTime = splinedModel.endTime

samplingPeriod = (endTime - startTime)/loader.frameCount

sim = Simulation()
sim.time = startTime

imu = IdealIMU()
# print(imu.accelerometer._positionOffset)
# print(imu.accelerometer._rotationOffset)
# print(imu.gyroscope._positionOffset)
# print(imu.gyroscope._rotationOffset)
# assert False

imu.simulation = sim
imu.trajectory = splinedModel.getJoint('Hips')
# print (imu.trajectory)

BasicIMUBehaviour(imu, samplingPeriod)

sim.run(endTime)
# print(imu.accelerometer._positionOffset)
# print(imu.accelerometer._rotationOffset)
# print(imu.gyroscope._positionOffset)
# print(imu.gyroscope._rotationOffset)

timestamps = imu.accelerometer.rawMeasurements.timestamps
print (timestamps.shape)

print ('trajectory')
acc_position = np.empty((timestamps.shape[0], 3))
acc_rotation = np.empty((timestamps.shape[0], 3))
gyro_position = np.empty((timestamps.shape[0], 3))
gyro_rotation = np.empty((timestamps.shape[0], 3))
for i, t in enumerate(timestamps):
	# position
	acc_pose = imu.accelerometer.trajectory.position(t)
	acc_position[i] = acc_pose.reshape((-1,))

	gyro_pose = imu.gyroscope.trajectory.position(t)
	gyro_position[i] = gyro_pose.reshape((-1,))   
	# print (acc_pose-gyro_pose)

	# rotation
	acc_quat = imu.accelerometer.trajectory.rotation(t)
	acc_rot.to
	print (acc_quat)
	acc_rotation[i] = acc_quat.reshape((-1,))

	gyro_quat = imu.gyroscope.trajectory.rotation(t)
	gyro_rotation[i] = gyro_quat.reshape((-1,))   
	print (acc_rot-gyro_quat)

# assert np.array_equal(acc_position, gyro_position)

# print (acc_trajectory)
# print (acc_trajectory.shape)
# print (gyro_trajectory)
# print (gyro_trajectory.shape)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.axes3d as p3

fig = plt.figure()
ax = p3.Axes3D(fig)

