import json

class Location:
	# neg = left, pos = right
	DEGREES_DEF  = 0
	# 0 = horizontal, neg = down, pos = up
	AZIM_DEF     = 0
	# -1 = unset, otherwise positive
	DISTANCE_DEF = -1

	def __init__(self, degrees=DEGREES_DEF, azim=AZIM_DEF, distance=DISTANCE_DEF):
		#print("init")
		self.reset(degrees, azim, distance)

	def toJSON(self):
		return json.dumps(self, default=lambda o: o.__dict__, 
			sort_keys=True, indent=4)

	def reset(self, degrees=DEGREES_DEF, azim=AZIM_DEF, distance=DISTANCE_DEF):
		__set_degrees(degrees)
		__set_azim(azim)
		__set_distance(distance)

	def __set_degrees(self, value):
		self.__degrees = value
	def __set_azim(self, value):
		self.__azim = value
	def __set_distance(self, value):
		self.__distance = value

	def __get_degrees(self):	
		return self.__degrees
	def __get_azim(self):
		return self.__azim
	def __get_distance(self):
		return self.__distance

    	degrees = property(__get_degrees, __set_degrees)
    	azim = property(__get_azim, __set_azim)
    	distance = property(__get_distance, __set_distance)

