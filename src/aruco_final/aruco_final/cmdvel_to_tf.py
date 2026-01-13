import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

def yaw_to_quat(yaw):
	# quaternion from yaw only
	qz = math.sin(yaw * 0.5)
	qw = math.cos(yaw * 0.5)
	return (0.0, 0.0, qz, qw)

class CmdVelToTF(Node):
	def __init__(self):
		super().__init__('cmdvel_to_tf')

		self.declare_parameter('cmd_topic', '/cmd_vel')
		self.declare_parameter('odom_topic', '/odom')
		self.declare_parameter('odom_frame', 'odom')
		self.declare_parameter('base_frame', 'base_link')
		self.declare_parameter('rate', 30.0)

		self.cmd_topic = self.get_parameter('cmd_topic').value
		self.odom_topic = self.get_parameter('odom_topic').value
		self.odom_frame = self.get_parameter('odom_frame').value
		self.base_frame = self.get_parameter('base_frame').value
		rate = float(self.get_parameter('rate').value)

		self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
		self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
		self.tf_broadcaster = TransformBroadcaster(self)

		self.vx = 0.0
		self.wz = 0.0

		self.x = 0.0
		self.y = 0.0
		self.yaw = 0.0

		self.last_time = self.get_clock().now()
		self.timer = self.create_timer(1.0 / rate, self.on_timer)

		self.get_logger().info(f"Listening cmd_vel: {self.cmd_topic}")
		self.get_logger().info(f"Publishing odom: {self.odom_topic} and TF {self.odom_frame}->{self.base_frame}")

	def on_cmd(self, msg: Twist):
		self.vx = float(msg.linear.x)
		self.wz = float(msg.angular.z)

	def on_timer(self):
		now = self.get_clock().now()
		dt = (now - self.last_time).nanoseconds * 1e-9
		self.last_time = now
		if dt <= 0.0:
			return
		self.last_time = now

		# integrate
		self.yaw += self.wz * dt
		self.x += self.vx * math.cos(self.yaw)*dt
		self.y += self.vx * math.sin(self.yaw)*dt

		qx, qy, qz, qw = yaw_to_quat(self.yaw)

		# TF: odom -> base_link
		t = TransformStamped()
		t.header.stamp = now.to_msg()
		t.header.frame_id = self.odom_frame
		t.child_frame_id = self.base_frame
		t.transform.translation.x = self.x
		t.transform.translation.y = self.y
		t.transform.translation.z = 0.0

		qz = math.sin(self.yaw * 0.5)
		qw = math.cos(self.yaw * 0.5)
		t.transform.rotation.x = qx
		t.transform.rotation.y = qy
		t.transform.rotation.z = qz
		t.transform.rotation.w = qw

		self.tf_broadcaster.sendTransform(t)

		# Odometry
		odom = Odometry()
		odom.header.stamp = t.header.stamp
		odom.header.frame_id = self.odom_frame
		odom.child_frame_id = self.base_frame
		odom.pose.pose.position.x = self.x
		odom.pose.pose.position.y = self.y
		odom.pose.pose.position.z = 0.0
		odom.pose.pose.orientation.z = qz
		odom.pose.pose.orientation.w = qw
		odom.twist.twist.linear.x = self.vx
		odom.twist.twist.angular.z = self.wz

		self.odom_pub.publish(odom)

def main():
	rclpy.init()
	node = CmdVelToTF()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
