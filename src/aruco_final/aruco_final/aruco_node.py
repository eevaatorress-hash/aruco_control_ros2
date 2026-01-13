#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge

import cv2
import numpy as np

from tf2_ros import TransformBroadcaster


class ArucoNode(Node):
	def __init__(self):
		super().__init__('aruco_node')

		self.declare_parameter('image_topic','/image')
		self.declare_parameter('marker_id',0)
		self.declare_parameter('marker_size_m',0.05) # 5 cm
		self.declare_parameter('camera_frame','camera_frame')
		self.declare_parameter('marker_frame','aruco_marker')

		self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
		self.marker_id = self.get_parameter('marker_id').get_parameter_value().integer_value
		self.marker_size = float(self.get_parameter('marker_size_m').value)
		self.camera_frame = self.get_parameter('camera_frame').value
		self.marker_frame = self.get_parameter('marker_frame').value

		self.bridge = CvBridge()
		self.sub = self.create_subscription(Image, self.image_topic, self.cb, 10)

		self.tf_broadcaster = TransformBroadcaster(self)

		# ArUco
		self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
		self.aruco_params = cv2.aruco.DetectorParameters()
		self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

		# Camera
		self.fx = 600.0
		self.fy = 600.0
		self.cx = 320.0
		self.cy = 240.0
		self.K = np.array([[self.fx, 0, self.cx],
			       	[0, self.fy, self.cy],
				[0, 0, 1]], dtype=np.float64)
		self.D = np.zeros((5,1),dtype=np.float64)

		self.get_logger().info(f"Listening to images: {self.image_topic}")
	
	def cb(self, msg: Image):
		frame = self.bridge.imgmsg_to_cv2(msg, desired_encodings= 'bgr8')
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		corners, ids, _ = self.detector.detectMarkers(gray)

		if ids is None:
			return
		
		ids = ids.flatten()
		for i, mid in enumerate(ids):
			if int(mid) != int(self.marker_id):
				continue

			c = corners[i]

			# Position estimation
			rvecs, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(c, self.marker_size, self.K, self.D)
			rvec = rvec[0][0]
			tvec = tvecs[0][0]

			#Draw
			cv2.aruco.drawDetectedMarkers(fram,[c])
			cv2.drawFrameAxes(fram, self.K, self.D, rvec, tvec, self.marker_size * 0.5)

			# Publish TF camera -> marker
			T = TransformStamped()
			T.header.stamp = self.get_clock().now().to_msg()
			T.header.frame_id = self.camera_frame

			T.transform.translation.x = float(tvec[0])
			T.transform.translation.y = float(tvec[1])
			T.transform.translation.z = float(tvec[2])

			# rvec -> quaternion
			R, _ = cv2.Rodrigues(rvec)
			qw, qx, qy, qz = self.rot_to_quat(R)
			T.transform.rotation.x = float(qx)
			T.transform.rotation.y = float(qy)
			T.transform.rotation.z = float(qz)
			T.transform.rotation.w = float(qw)

			self.tf_broadcaster.sendTransform(T)
			self.get_logger().info(f"ArUco {mid} t=[{tvec[0]:.3f}, {tvec[1]:.3f}, tvec[2]:.3f}]")

			# Ventana debug
			try:
				cv2.imshow("aruco_debug", frame)
				cv2.waitKey(1)
			except Exception:
				pass

	@staticmethod
	def rot_to_quat(R):
		# Convierte matriz 3x3 a quaternion (w,x,y,x)
		tr = R[0, 0] + R[1, 1] + R[2, 2]
		if tr > 0:
			S = np.sqrt(tr + 1.0) * 2
			qw = 0.25 * S
			qx = (R[2, 1] - R[1, 2]) / S
			qy = (R[0, 2] - R[2, 0]) / S
			qz = (R[1, 0] - R[0, 1]) / S
		elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
			S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
			qw = (R[2, 1] - R[1, 2]) / S
			qx = 0.25 * S
			qy = (R[0, 1] + R[1, 0]) / S
			qz = (R[0, 2] + R[2, 0]) / S
		elif R[1, 1] > R[2, 2]:
			S = np.sqrt(1.0 + R[1,1] - R[0, 0] - R[2, 2]) * 2
			qw = (R[0, 2] - R[2, 0]) / S
			qx = (R[0, 1] + R[1, 0]) / S
			qy = 0.25 * S
			qz = (R[1, 2] + R[2, 1]) / S
		else:
			S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
			qw = (R[1, 0] - R[0, 1]) / S
			qx = (R[0, 2] + R[2, 0]) / S
			qy = (R[1, 2] + R[2, 1]) / S
			qz = 0.25 * S
		return qw, qx, qy, qz


def main():
	rclpy.init()
	node = ArucoNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
