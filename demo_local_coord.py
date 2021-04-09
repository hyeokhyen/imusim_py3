'''
Global coordinate is identity(3)
The person is flipped in z axis
Need to implement IdealIMUoffset -> demo_offset.py
'''

from imusim.platforms.imus import IdealIMU
from imusim.io.bvh import BVHLoader
from imusim.simulation.base import Simulation
from imusim.behaviours.imu import BasicIMUBehaviour
from imusim.trajectories.rigid_body import SplinedBodyModel
from pprint import pprint
import numpy as np

file_bvh = './141_01.bvh'
CM_TO_M_CONVERSION = 0.01

with open(file_bvh, 'r') as bvhFile:
	loader = BVHLoader(bvhFile, CM_TO_M_CONVERSION)
	loader._readHeader()
	loader._readMotionData()
	model = loader.model

list_joint = []
list_bone = {}
list_position = {}
# for p in model.points:
# 	print (p.name)
# assert False

for p in model.joints:
	print (p.name)
	list_joint.append(p.name)
	list_bone[p.name] = []

	for child in p.children:
		list_bone[p.name].append(child.name)
		print (child.name)
	print ('-----------')
pprint (list_bone)
# assert False

splinedModel = SplinedBodyModel(model)
startTime = splinedModel.startTime
endTime = splinedModel.endTime

print (model.positionKeyFrames.timestamps)

joint = splinedModel.getJoint('Hips')
print (joint.position(startTime))
assert False

samplingPeriod = (endTime - startTime)/loader.frameCount

sim = Simulation()
sim.time = startTime

imu = IdealIMU()

rotationOffset = imu.accelerometer._rotationOffset
# print (rotationOffset.toMatrix())
# assert False
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
# print (timestamps.shape)

acc = np.empty((timestamps.shape[0], 3))
acc_volt = np.empty((timestamps.shape[0], 3))
acc_rotFrame = np.empty((timestamps.shape[0], 3))
acc_position = np.empty((timestamps.shape[0], 3))
acc_rotation = np.empty((timestamps.shape[0], 3, 3))

gyro = np.empty((timestamps.shape[0], 3))
gyro_position = np.empty((timestamps.shape[0], 3))
gyro_rotation = np.empty((timestamps.shape[0], 3, 3))

gravity_pos = imu.accelerometer.platform.simulation.environment.gravitationalField
gravity = np.empty((timestamps.shape[0], 3))

for i, t in enumerate(timestamps):
	# gravity
	g = gravity_pos(imu.accelerometer.trajectory.position(t), t)
	# print (g)
	gravity[i] = g.reshape((-1,))
	
	# acceleration
	_acc = imu.accelerometer.trajectory.acceleration(t)
	# print (_acc)
	# assert False
	acc[i] = _acc.reshape((-1,))

	# acceleration - gravity -> rotateFrame
	a = imu.accelerometer.trajectory.rotation(t).rotateFrame(_acc - g)
	acc_rotFrame[i] = a.reshape((-1,))

	# voltage
	volt = imu.accelerometer.voltages(t)
	acc_volt[i] = volt.reshape((-1,))
	# print (volt)
	# print (_acc)
	# print ('----------')
	# assert False

	# position
	acc_pose = imu.accelerometer.trajectory.position(t)
	acc_position[i] = acc_pose.reshape((-1,))

	gyro_pose = imu.gyroscope.trajectory.position(t)
	gyro_position[i] = gyro_pose.reshape((-1,))   
	# print (acc_pose-gyro_pose)

	# rotation
	model_rot = splinedModel.getJoint('Hips').rotation(t).toMatrix()
	# print (model_rot)	
	
	acc_quat = imu.accelerometer.trajectory.rotation(t)
	acc_rot = acc_quat.toMatrix()
	acc_rotation[i] = acc_rot

	_gyro = imu.accelerometer.trajectory.rotationalVelocity(t)
	gyro[i] = _gyro.reshape((-1,))

	gyro_quat = imu.gyroscope.trajectory.rotation(t)
	gyro_rot = gyro_quat.toMatrix()
	gyro_rotation[i] = gyro_rot   
	# print (acc_rot-model_rot)
	# print (acc_rot-gyro_rot)
	
	# print ('-------------')

# assert np.array_equal(acc_position, gyro_position)

# print (acc_trajectory)
# print (acc_trajectory.shape)
# print (gyro_trajectory)
# print (gyro_trajectory.shape)

# print (gravity)
if 1:
	acc_raw = imu.accelerometer.rawMeasurements.values.T
	print (acc_raw.shape)
	for i in range(acc.shape[0]-4):
		# print (acc[i+4])
		# print (acc_rotFrame[i+4])
		print (acc_volt[i+4])
		print (acc_raw[i])
		print ('---------------------')

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import mpl_toolkits.mplot3d.axes3d as p3

fig = plt.figure()
ax = fig.add_subplot(311)
ax.plot(acc_rotFrame[4:,0], 'b')
ax.plot(acc_raw[:-4,0], 'r')

ax = fig.add_subplot(312)
ax.plot(acc_rotFrame[4:,1], 'b')
ax.plot(acc_raw[:-4,1], 'r')

ax = fig.add_subplot(313)
ax.plot(acc_rotFrame[4:,2], 'b')
ax.plot(acc_raw[:-4,2], 'r')

fig.tight_layout()

file_fig = '/home/ubuntu/dataset/analysis/imusim_py3/acc.png'
fig.savefig(file_fig)
print ('save in ...', file_fig)


