'''
Global coordinate is identity(3)
The person is flipped in z axis
Offset for position and rotation done!
'''

from imusim.platforms.imus import IdealIMU, IdealIMUoffset
from imusim.io.bvh import BVHLoader
from imusim.simulation.base import Simulation
from imusim.behaviours.imu import BasicIMUBehaviour
from imusim.trajectories.rigid_body import SplinedBodyModel
from imusim.maths.quaternions import Quaternion
from imusim.maths.vectors import vector

from scipy.spatial.transform import Rotation as R
import numpy as np
import pickle as cp
from pprint import pprint

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

file_bvh = './141_01.bvh'
CMU_CONVERSION = (1.0/0.45)*2.54/100.0

with open(file_bvh, 'r') as bvhFile:
	loader = BVHLoader(bvhFile, CMU_CONVERSION)
	loader._readHeader()
	loader._readMotionData()
	model = loader.model

list_joint = []
list_bone = {}
for p in model.joints:
	# print (p.name)
	list_joint.append(p.name)
	list_bone[p.name] = []

	for child in p.children:
		list_bone[p.name].append(child.name)
	# 	print (child.name)
	# print ('-----------')
# pprint (list_bone)

file_info = '/home/ubuntu/dataset/analysis/imusim_py3/141_01_bone.p'
cp.dump(list_bone, open(file_info, 'wb'))
print ('save in ...', file_info)

# sampled model
timestamps = model.positionKeyFrames.timestamps
print (timestamps.shape)

list_info = {}
for joint_name in list_joint:
	joint = model.getJoint(joint_name)
	joint_loc = np.empty((timestamps.shape[0], 3))
	position = np.empty((timestamps.shape[0], 3))
	rotation = np.empty((timestamps.shape[0], 3, 3))
	for i, t in enumerate(timestamps):
		pos = joint.position(t)
		rot = joint.rotation(t).toMatrix()
		# print (pos)
		# print (rot)
		# assert False

		joint_loc[i] = pos.reshape((-1,))
		position[i] = pos.reshape((-1,))
		rotation[i] = rot

	list_info[joint_name] = {
		'joint_loc': joint_loc,
		'position': position,
		'rotation': rotation}

file_info = '/home/ubuntu/dataset/analysis/imusim_py3/141_01_info_sampled.p'
cp.dump(list_info, open(file_info, 'wb'))
print ('save in ...', file_info)

# assert False

splinedModel = SplinedBodyModel(model)
startTime = splinedModel.startTime
endTime = splinedModel.endTime

samplingPeriod = (endTime - startTime)/loader.frameCount

sim = Simulation()
sim.time = startTime

dict_imu = {}

is_offset_rot = False
is_offset_pos = True
# list_joint = ['RightHand']

for joint_name in list_joint:

	offset_quat = Quaternion(1,0,0,0)
	if is_offset_rot:
		offset_euler = R.random().as_euler('zxy', degrees=True)
		offset_quat = Quaternion.fromEuler(tuple(offset_euler), order='zxy')
		print (f'{joint_name}| offset rotation ...')
		print (offset_euler)
		print (offset_quat)

	offset_vec = vector(0,0,0)
	if is_offset_pos:
		offset_np = np.random.uniform(-0.5, 0.5, size=(3,))
		offset_vec = vector(offset_np[0], offset_np[1], offset_np[2])
		# offset_vec = offset_np
		print (f'{joint_name} | offset position ...')
		print (offset_vec)

	imu = IdealIMUoffset(positionOffset=offset_vec,
									rotationOffset=offset_quat)
	imu.simulation = sim
	# imu.trajectory = model.getJoint(joint_name)
	imu.trajectory = splinedModel.getJoint(joint_name)

	BasicIMUBehaviour(imu, samplingPeriod)
	dict_imu[joint_name] = imu

sim.run(endTime)
# assert False

# print(dict_imu['RightHand'].accelerometer._positionOffset)
# print(dict_imu['RightHand'].accelerometer._rotationOffset)
# print(dict_imu['RightHand'].gyroscope._positionOffset)
# print(dict_imu['RightHand'].gyroscope._rotationOffset)
# assert False

list_info = {}
for joint_name in dict_imu:
	joint = splinedModel.getJoint(joint_name)
	
	imu = dict_imu[joint_name]

	timestamps = imu.accelerometer.rawMeasurements.timestamps
	joint_loc = np.empty((timestamps.shape[0], 3))
	position = np.empty((timestamps.shape[0], 3))
	rotation = np.empty((timestamps.shape[0], 3, 3))
	for i, t in enumerate(timestamps):
		j_pos = joint.position(t)
		joint_loc[i] = j_pos.reshape((-1,))

		# if is_offset_rot:
		# 	imu.accelerometer.trajectory.rotationOffset = imu.accelerometer._rotationOffset
		# 	imu.gyroscope.trajectory.rotationOffset = imu.gyroscope._rotationOffset
		
		# if is_offset_pos:
		# 	imu.accelerometer.trajectory.positionOffset = imu.accelerometer._positionOffset
		# 	imu.gyroscope.trajectory.positionOffset = imu.gyroscope._positionOffset

		pose = imu.accelerometer.trajectory.position(t)
		position[i] = pose.reshape((-1,))

		rot = imu.accelerometer.trajectory.rotation(t).toMatrix()
		rotation[i] = rot
	
	list_info[joint_name] = {
		'joint_loc': joint_loc,
		'position': position,
		'rotation': rotation}

file_info = '/home/ubuntu/dataset/analysis/imusim_py3/141_01_info'
if is_offset_rot:
	file_info += '_offset_rot'
if is_offset_pos:
	file_info += '_offset_pos'
file_info += '.p'

cp.dump(list_info, open(file_info, 'wb'))
print ('save in ...', file_info)

joint_name = 'RightHand'
imu = dict_imu[joint_name]
acc_raw = imu.accelerometer.rawMeasurements.values.T

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(acc_raw[:-4,0], 'r')
ax.plot(acc_raw[:-4,1], 'g')
ax.plot(acc_raw[:-4,2], 'b')
fig.tight_layout()

file_fig = '/home/ubuntu/dataset/analysis/imusim_py3/141_01_acc'
if is_offset_rot:
	file_fig += '_offset_rot'
if is_offset_pos:
	file_fig += '_offset_pos'
file_fig += '.png'
fig.savefig(file_fig)
print ('save in ...', file_fig)
