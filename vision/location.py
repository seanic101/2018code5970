
class Location(object):
	#def encode(self, obj):
	#	return [obj.degrees, obj.azim, obj.distance]

	def __init__(self):
		#print("init")
		self.reset()

	def reset(self):
		# neg = left, pos = right
		self.degrees = 0
		# 0 = horizontal, neg = down, pos = up
		self.azim = 0
		# -1 = unset, otherwise positive
		self.distance = -1.0

	def set_degrees(self, value):
		self.degrees = value
	def set_azim(self, value):
		self.azim = value
	def set_distance(self, value):
		self.distance = value

	def degrees(self):	
		return self.degrees
	def azim(self):
		return self.azim
	def distance(self):
		return self.distance
