# Author: Tom Wallenstein
# documentation can be found in Tom's github
import numpy as np

class Object3D():
	def rotateXinWorldSpace(self, vector, theta):
		c, s = np.cos(theta), np.sin(theta)
		R = np.array((	(1, 0, 0),
						(0, c, -s),
						(0, s, c)))
		return np.matmul(R, vector)

	def rotateYinWorldSpace(self, vector, theta):
		c = np.cos(theta)
		s = np.sin(theta)
		R = np.array(((c, 0, s),
					(0, 1, 0),
					(-s, 0, c)))
		return np.matmul(R, vector)

	def rotateZinWorldSpace(self, vector, theta):
		c, s = np.cos(theta), np.sin(theta)
		R = np.array((	(c, -s, 0),
						(s, c, 0),
						(0, 0, 1)))
		return np.matmul(R, vector)

	def translateToPosition(self, v1, v2):
		return np.add(v1, v2)

class Vector(Object3D):
	def __init__(self, point1, point2):
		self.coords = np.substract(point2, point1)

	def __init__(self, coords):
		self.coords = np.array(coords)

	def getXY(self):
		return np.array([self.coords[0], self.coords[1]])
	def getXZ(self):
		return np.array([self.coords[0], self.coords[2]])
	def getYZ(self):
		return np.array([self.coords[1], self.coords[2]])

	# returns a vector rotated by specified Euler angles
	def rotate_vector(self, rot_x, rot_y, rot_z):
		self.coords = self.rotateXinWorldSpace(self.coords, rot_x)
		self.coords = self.rotateYinWorldSpace(self.coords, rot_y)
		self.coords = self.rotateZinWorldSpace(self.coords, rot_z)

class Jibo(Object3D):
	def __init__(self, position):
		self.position = np.array(position)

class Camera(Object3D):
	def __init__(self, name, position, rot_y, rot_z):
		self.name = name
		self.position = np.array(position)
		self.rot_y = np.radians(rot_y)
		self.rot_z = np.radians(rot_z)


class Face(Object3D):
	def __init__(self, camera, position, rot_x, rot_y, rot_z, gazeAngle=None):
		self.camera = camera
		# TO-DO: check if this is right
		# 1.) check the assignment of coordinates axis
		# 2.) OpenFace says values of rotation are in left-handed positive sign
		# (e.g. clockwise rotation is positive). Therefore, we need to negate
		# values since in our coordinate system  positive rotation is ccw.
		self.position = np.array([position[2], -position[0], -position[1]])

		self.rot_x = -rot_z
		self.rot_y = -rot_x
		self.rot_z = -rot_y
		self.pose_direction = np.array([1.0, 0.0, 0.0])
		if gazeAngle is not None:
			self.gazeAngle = np.array(gazeAngle)
		else:
			self.gazeAngle = None

	def convertFaceFromLocalToWorldSpace(self):          
		self.position = self.rotateYinWorldSpace(self.position, self.camera.rot_y)
		self.position = self.rotateZinWorldSpace(self.position, self.camera.rot_z)
		self.position = self.translateToPosition(self.position, self.camera.position)
