# vim: sm ai tabstop=4 shiftwidth=4 softtabstop=4
import cv2
import sys

class Camera:
	def __init__(self, cam_index):
		self.cam_index = cam_index
		self.cam = cv2.VideoCapture(self.cam_index)
		if not self.cam.isOpened():
			sys.stderr.write("No camera could be opened for capture\n")
			exit(1)
		self.debug = False

	def set_debug(self, on_or_off):
		self.debug = on_or_off

	def setup(self):
		# Here are the numeric values of the properties that can be set on a
		# VideoCapture:
		# 0. CV_CAP_PROP_POS_MSEC Current position of the video file in milliseconds.
		# 1. CV_CAP_PROP_POS_FRAMES 0-based index of the frame to be decoded/captured
		# next.
		# 2. CV_CAP_PROP_POS_AVI_RATIO Relative position of the video file
		# 3. CV_CAP_PROP_FRAME_WIDTH Width of the frames in the video stream.
		# 4. CV_CAP_PROP_FRAME_HEIGHT Height of the frames in the video stream.
		# 5. CV_CAP_PROP_FPS Frame rate.
		# 6. CV_CAP_PROP_FOURCC 4-character code of codec.
		# 7. CV_CAP_PROP_FRAME_COUNT Number of frames in the video file.
		# 8. CV_CAP_PROP_FORMAT Format of the Mat objects returned by retrieve() .
		# 9. CV_CAP_PROP_MODE Backend-specific value indicating the current capture
		# mode.
		# 10. CV_CAP_PROP_BRIGHTNESS Brightness of the image (only for cameras).
		# 11. CV_CAP_PROP_CONTRAST Contrast of the image (only for cameras).
		# 12. CV_CAP_PROP_SATURATION Saturation of the image (only for cameras).
		# 13. *** CV_CAP_PROP_HUE Hue of the image (only for cameras).
		# 14. *** CV_CAP_PROP_GAIN Gain of the image (only for cameras).
		# 15. *** CV_CAP_PROP_EXPOSURE Exposure (only for cameras).
		# 16. CV_CAP_PROP_CONVERT_RGB Boolean flags indicating whether images should
		# be converted to RGB.
		# 17. CV_CAP_PROP_WHITE_BALANCE Currently unsupported
		# 18. CV_CAP_PROP_RECTIFICATION Rectification flag for stereo cameras (note:
		# only supported by DC1394 v 2.x backend currently)
		# *** not supported by MS camera

		contrast = self.cam.get(11) # (11) or (cv2.cv.CV_CAP_PROP_CONTRAST)
		# default was .433, lower contrast is better
		new_con = self.cam.set(11, 0.1)
		if self.debug:
			print("old contrast " + (str(contrast)))
			print("new contrast " + (str(new_con))) #-trast

	def read(self):
		img = self.cam.read()
		return img
	
	def shutdown(self):
		cv2.destroyAllWindows()
		self.cam.release()


