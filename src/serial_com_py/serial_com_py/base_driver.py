#!/usr/bin/env python3

import math
import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros


class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver')

        # =========================
        # PARÂMETROS DO ROBÔ
        # =========================
        self.wheel_base = 0.250
        self.wheel_diameter = 0.165
        self.wheel_radius = self.wheel_diameter / 2.0
        self.pulses_per_rev = 51.8 # esse trem tem que ser o mesmo que o do arduino
        self.meters_per_pulse = (
            math.pi * self.wheel_diameter
        ) / self.pulses_per_rev

        # =========================
        # SERIAL
        # =========================
        self.serial_port = '/dev/ttyACM0'
        self.baud_rate = 115200
        self.serial_buffer = ""

        try:
            self.ser = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0
            )
            self.get_logger().info(
                f'Conectado em {self.serial_port}'
            )
        except Exception as e:
            self.get_logger().error(
                f'Erro serial: {e}'
            )
            self.ser = None

        # =========================
        # ESTADO ODOM
        # =========================
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_left_pulses = None
        self.last_right_pulses = None

        # =========================
        # ROS
        # =========================
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(
            0.02,
            self.read_serial
        )

    # =====================================
    # CMD_VEL -> Arduino
    # =====================================
    def cmd_callback(self, msg):

        MAX_LINEAR = 0.5
        MAX_ANGULAR = 1.0

        v = max(
            min(msg.linear.x, MAX_LINEAR),
            -MAX_LINEAR
        )

        w = max(
            min(msg.angular.z, MAX_ANGULAR),
            -MAX_ANGULAR
        )

        # Cinemática diferencial padrão
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        #esquerda primeiro, depois direita, na duvida inverte esse trem
        cmd = f"CMD:{v_left:.3f};{v_right:.3f}\n"

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode())
            except Exception as e:
                self.get_logger().warn(
                    f'Erro enviando serial: {e}'
                )

    # =====================================
    # QUATERNION
    # =====================================
    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    # =====================================
    # SERIAL -> ODOM
    # =====================================
    def read_serial(self):

        if self.ser is None:
            return

        try:
            n = self.ser.in_waiting

            if n <= 0:
                return

            data = self.ser.read(
                n
            ).decode(
                'utf-8',
                errors='ignore'
            )

            self.serial_buffer += data

            if '\n' not in self.serial_buffer:
                return

            lines = self.serial_buffer.split('\n')
            self.serial_buffer = lines[-1]

            valid_line = None

            for line in lines[:-1]:
                line = line.strip()

                if line.startswith("ODO:"):
                    valid_line = line

            if valid_line is None:
                return

            payload = valid_line.replace(
                "ODO:",
                ""
            ).split(";")

            if len(payload) != 3:
                return

            left_p = int(payload[0])
            right_p = int(payload[1])
            dt_ms = int(payload[2])

            if dt_ms <= 0:
                return

            dt = dt_ms / 1000.0

            # primeira leitura
            if self.last_left_pulses is None:
                self.last_left_pulses = left_p
                self.last_right_pulses = right_p
                return

            # ==========================
            # DELTAS
            # ==========================
            d_left = (
                left_p - self.last_left_pulses
            )

            d_right = (
                right_p - self.last_right_pulses
            )

            self.last_left_pulses = left_p
            self.last_right_pulses = right_p

            # Se a roda esquerda estiver invertida,
            # descomenta esse trem aqui:
            # d_left = -d_left

            dist_left = (
                d_left *
                self.meters_per_pulse
            )

            dist_right = (
                d_right *
                self.meters_per_pulse
            )

            v_left = dist_left / dt
            v_right = dist_right / dt

            # ==========================
            # ODOM PADRÃO
            # ==========================
            v = (
                v_right + v_left
            ) / 2.0

            w = (
                v_right - v_left
            ) / self.wheel_base

            self.x += (
                v *
                math.cos(self.th) *
                dt
            )

            self.y += (
                v *
                math.sin(self.th) *
                dt
            )

            self.th += w * dt

            now = self.get_clock().now()

            # ==========================
            # ODOM MSG
            # ==========================
            odom = Odometry()

            odom.header.stamp = now.to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y

            qz, qw = self.yaw_to_quaternion(
                self.th
            )

            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w

            self.odom_pub.publish(odom)

            # ==========================
            # TF
            # ==========================
            tf = TransformStamped()

            tf.header.stamp = now.to_msg()
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_footprint"

            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y

            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf)

            # ==========================
            # JOINT STATES
            # ==========================
            js = JointState()

            js.header.stamp = now.to_msg()

            js.name = [
                'left_wheel_joint',
                'right_wheel_joint'
            ]

            js.position = [
                left_p * self.meters_per_pulse
                / self.wheel_radius,

                right_p * self.meters_per_pulse
                / self.wheel_radius
            ]

            js.velocity = [
                v_left / self.wheel_radius,
                v_right / self.wheel_radius
            ]

            self.joint_pub.publish(js)

        except Exception as e:
            self.get_logger().warn(
                f'Erro serial: {e}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()