def set(capture, prop_name, prop, new_value):
	old = capture.get(prop)
	capture.set(prop, new_value)
	dbg = (prop_name + "\n\told: " + str(old) + "\n\tnew: " +
		str(new_value))
	return old, dbg

cap = cv2.VideoCapture(0)

_, dbg = (set(cap, "contrast",
	cv2.cv.CV_CAP_PROP_CONTRAST, 0.01))
print dbg
_, dbg = (set(cap, "brightness",
	cv2.cv.CV_CAP_PROP_BRIGHTNESS, 0.1))
print dbg
_, dbg = (set(cap, "saturation",
	cv2.cv.CV_CAP_PROP_SATURATION, 0.5))
print dbg

