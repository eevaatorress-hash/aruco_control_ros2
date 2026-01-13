#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

from cv_bridge import CvBridge
import cv2


class ArucoControlNode(Node):
	def __init__(self):
		super().__init__('aruco_control_node')

		# Params
		self.declare_parameter('image_topic', '/image')
		self.declare_parameter('camera_info_topic', '/camera_info')
		self.declare_parameter('cmd_vel_topic', '/cmd_vel')
		self.declare_parameter('annotated_topic', '/aruco/image_annotated')
		self.declare_parameter('pose_topic', '/aruco/pose')
		self.declare_parameter('marker_id_topic', '/aruco/id')

		self.declare_parameter('marker_id', -1)
		self.declare_parameter('marker_length', 0.05) # 5cm
		self.declare_parameter('aruco_dict', 'DICT_4X4_50')

		self.image_topic = self.get_parameter('image_topic').value
		self.camera_info_topic = self.get_parameter('camera_info_topic').value
		self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
		self.annotated_topic = self.get_parameter('annotated_topic').value
		self.pose_topic = self.get_parameter('pose_topic').value
		self.marker_id_topic = self.get_parameter('marker_id_topic').value

		self.target_marker_id = int(self.get_parameter('marker_id').value)
		self.marker_length = self.get_parameter('marker_length').value
		self.aruco_dict_name = self.get_parameter('aruco_dict').value

		self.declare_parameter('deadband_px', 25)
		self.declare_parameter('deadband_yaw_px', 35)

		self. declare_parameter('forward_speed', 0.08)
		self.declare_parameter('backward_speed', -0.06)
		self.declare_parameter('yaw_speed', 0.35)

		self.declare_parameter('use_yaw', True)
		self.declare_parameter('yaw_kp', 0.002)
		self.declare_parameter('max_yaw', 0.6)

		self.declare_parameter('draw_debug', True)
		self.declare_parameter('log_every_n', 30)
		self.declare_parameter('flip_image', False)

		self.deadband_px = int(self.get_parameter('deadband_px').value)
		self.deadband_yaw_px = int(self.get_parameter('deadband_yaw_px').value)

		self.forward_speed = float(self.get_parameter('forward_speed').value)
		self.backward_speed = float(self.get_parameter('backward_speed').value)
		self.yaw_speed = float(self.get_parameter('yaw_speed').value)

		self.draw_debug = bool(self.get_parameter('draw_debug').value)
		self.log_every_n = int(self.get_parameter('log_every_n').value)
		self.flip_image = bool(self.get_parameter('flip_image').value)

		# CV
		self.bridge = CvBridge()
		self.frame_count = 0

		# Camera calibration (filled when we receive Camerainfo)
		self.camera_matrix = None
		self.dist_coeffs = None

		# ArUco setup
		self.aruco_dict = self._get_aruco_dictionary(self.aruco_dict_name)
		self.aruco_params = self._make_detector_params()
		self.aruco_detector = self._make_detector(self.aruco_dict, self.aruco_params)

		# Subs/Pubs
		self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
		self.annot_pub = self.create_publisher(Image, self.annotated_topic, 10)
		self.id_pub = self.create_publisher(Int32, self.marker_id_topic, 10)

		self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
		self.create_subscription(Image, self.image_topic, self.on_image, 10)

		self.get_logger().info(f"Listening image: {self.image_topic}")
		self.get_logger().info(f"Listening camera_info: {self.camera_info_topic}")
		self.get_logger().info(f"Marker length: {self.marker_length} m, dict: {self.aruco_dict_name}")

	def _get_aruco_dictionary(self, name: str):
		if not hasattr(cv2.aruco, name):
			self_logger().warn(f"Unkwon aruco_dict '{name}', using DICT_4X4_50")
			name = "DICT_4X4_50"

		dict_id = getattr(cv2.aruco,name)
		return cv2.aruco.getPredefinedDictionary(dict_id)

	def _make_detector_params(self):
		if hasattr(cv2.aruco, "DetectorParameters_create"):
			return cv2.aruco.DetectorParameters_create()
		elif hasattr(cv2.aruco, "DetectorParameters"):
			return cv2.aruco.DetectorParameters()
		else:
			return None

	def _make_detector(self, aruco_dict, params):
		if hasattr(cv2.aruco, "ArucoDetector"):
			return cv2.aruco.ArucoDetector(aruco_dict, params)
		return None

	def on_camera_info(self, msg: CameraInfo):
		k = np.array(msg.k, dtype=np.float64).reshape(3, 3)
		d = np.array(msg.d, dtype=np.float64).reshape(-1, 1) if len(msg.d) > 0 else np.zeros((5,1), dtype=np.float64)
		self.camera_matrix = k
		self.dist_coeffs = d

	def on_image(self, msg: Image):
		# convert to OpenCV
		try:
			frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		except TypeError:
			frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

		if self.flip_image:
			frame = cv2.flip(frame, 1)

		h, w = frame.shape[:2]
		gray = cv2.cvtColor(frame, cv2.	COLOR_BGR2GRAY)

		corners, ids, rejected = self._detect_markers(gray)

		if ids is None or len(ids) == 0:
			self._publish_stop()
			if self.draw_debug:
				self._publish_annotated(frame, msg)
			return
		ids = ids.flatten().astype(int)

		idx = self._select_marker_index(ids)
		if idx is None:
			self._publish_stop()
			if self.draw_debug:
				self._publish_annotated(frame, msg)
			return

		marker_id = int(ids[idx])
		self.id_pub.publish(Int32(data=marker_id))

		c = corners[idx][0]
		cx = int(np.mean(c[:, 0]))
		cy = int(np.mean(c[:, 1]))

		if self.draw_debug:
			cv2.aruco.drawDetectedMarkers(frame, [corners[idx]], np.array([[marker_id]]))
			cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
			cv2.line(frame, (w // 2, 0), (w // 2, h), (255, 0, 0), 2)
			cv2.line(frame, (0, h // 2), (w, h // 2), (255, 0, 0), 2)

		err_x = cx - (w //2)
		tw = Twist()

		if abs(err_x) <= self.deadband_yaw_px:
			tw.angular.z = 0.0
		else:
			tw.angular.z = -self.yaw_speed if err_x > 0 else self.yaw_speed

		center_y = h // 2
		err = cy - center_y

		if abs(err) <= self.deadband_px:
			tw.linear.x = 0.0
		else:
			tw.linear.x = self.forward_speed if err < 0 else self.backward_speed

		self.cmd_pub.publish(tw)

		if self.frame_count % self.log_every_n == 0:
			self.get_logger().info(f"id={marker_id} cx={cx} cy={cy} err_x={err_x} err_y={err} cmd(v={tw.linear.x:.2f}, w={tw.angular.z:.2f})")

		if self.draw_debug:
			self._publish_annotated(frame, msg)

	def _detect_markers(self, gray):
		if self.aruco_detector is not None:
			corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
		else:
			corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
		return corners, ids, rejected

	def _select_marker_index(self, ids: np.ndarray):
		if self.target_marker_id == -1:
			return 0
		matches = np.where(ids == self.target_marker_id)[0]
		if len(matches) == 0:
			return None
		return int(matches[0])

	def _publish_stop(self):
		tw = Twist()
		tw.linear.x = 0.0
		tw.angular.z = 0.0
		self.cmd_pub.publish(tw)

	def _publish_annotated(self, fram_bgr, original_msg: Image):
		try:
			out = self.bridge.cv2_to_imgmsg(fram_bgr, encodings='bgr8')
		except TypeError:
			out = self.bridge.cv2_to_imgmsg(fram_bgr, 'bgr8')
		out.header = original_msg.header
		self.annot_pub.publish(out)

def main (args=None):
	rclpy.init(args=args)
	node= ArucoControlNode()
	try:
		rclpy.spin(node)
	except KeyboardInterrupt:
		pass
	node.detroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()
