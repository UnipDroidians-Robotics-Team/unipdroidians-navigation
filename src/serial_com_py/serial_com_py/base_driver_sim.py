#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import tf2_ros


# ============================================
# SERIAL FALSA (SIMULA O ARDUINO)
# ============================================
class FakeSerial:
    def __init__(self):
        self.left_cmd = 0.0
        self.right_cmd = 0.0

        # acumuladores reais
        self.left_pulses = 0.0
        self.right_pulses = 0.0

        self.last_time = time.time()

    @property
    def in_waiting(self):
        return 1

    @property
    def is_open(self):
        return True

    def write(self, data):
        """
        Recebe:
        CMD:v_left;v_right
        """
        try:
            txt = data.decode().strip()

            if txt.startswith("CMD:"):
                vals = txt.replace(
                    "CMD:",
                    ""
                ).split(";")

                self.left_cmd = float(vals[0])
                self.right_cmd = float(vals[1])

                print(
                    f"CMD recebido: "
                    f"L={self.left_cmd:.3f} "
                    f"R={self.right_cmd:.3f}"
                )

        except Exception:
            pass

    def read(self, n):
        """
        Simula:
        ODO:left;right;dt_ms
        """

        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        dt_ms = int(dt * 1000)

        if dt_ms <= 0:
            dt_ms = 20

        pulses_per_meter = (
            51.8 /
            (math.pi * 0.165)
        )

        # acumula em FLOAT
        self.left_pulses += (
            self.left_cmd *
            dt *
            pulses_per_meter
        )

        self.right_pulses += (
            self.right_cmd *
            dt *
            pulses_per_meter
        )

        # só converte aqui
        line = (
            f"ODO:"
            f"{int(self.left_pulses)};"
            f"{int(self.right_pulses)};"
            f"{dt_ms}\n"
        )

        return line.encode()


# ============================================
# BASE DRIVER
# ============================================
class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver_sim')

        # ---------------------------
        # parâmetros do robô
        # ---------------------------
        self.wheel_base = 0.250
        self.wheel_diameter = 0.165
        self.wheel_radius = self.wheel_diameter / 2.0

        self.pulses_per_rev = 51.8

        self.meters_per_pulse = (
            math.pi * self.wheel_diameter
        ) / self.pulses_per_rev

        # ---------------------------
        # usar serial fake
        # ---------------------------
        self.ser = FakeSerial()

        self.get_logger().info(
            "Rodando em modo SIMULADO"
        )

        # ---------------------------
        # estado da odometria
        # ---------------------------
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.last_left = None
        self.last_right = None

        # ---------------------------
        # ROS interfaces
        # ---------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            'cmd_vel_safe',
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

        self.tf_broadcaster = (
            tf2_ros.TransformBroadcaster(self)
        )

        self.serial_buffer = ""

        self.timer = self.create_timer(
            0.02,
            self.read_serial
        )

    # ====================================
    # recebe cmd_vel
    # ====================================
    def cmd_callback(self, msg):
        max_linear = 0.5
        max_angular = 1.0

        v = max(
            min(msg.linear.x, max_linear),
            -max_linear
        )

        w = max(
            min(msg.angular.z, max_angular),
            -max_angular
        )

        # diferencial padrão
        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        cmd = (
            f"CMD:"
            f"{v_left:.3f};"
            f"{v_right:.3f}\n"
        )

        self.ser.write(cmd.encode())

    # ====================================
    # quaternion
    # ====================================
    def yaw_to_quaternion(self, yaw):
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        return qz, qw

    # ====================================
    # lê serial simulada
    # ====================================
    def read_serial(self):
        try:
            data = self.ser.read(1).decode()

            self.serial_buffer += data

            if "\n" not in self.serial_buffer:
                return

            lines = self.serial_buffer.split("\n")
            self.serial_buffer = lines[-1]

            line = lines[-2].strip()

            if not line.startswith("ODO:"):
                return

            vals = line.replace(
                "ODO:",
                ""
            ).split(";")

            left_p = int(vals[0])
            right_p = int(vals[1])
            dt_ms = int(vals[2])

            dt = dt_ms / 1000.0

            if self.last_left is None:
                self.last_left = left_p
                self.last_right = right_p
                return

            # deltas
            d_left = left_p - self.last_left
            d_right = right_p - self.last_right

            self.last_left = left_p
            self.last_right = right_p

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

            # odometria diferencial
            v = (
                v_left + v_right
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

            qz, qw = self.yaw_to_quaternion(
                self.th
            )

            # -------------------
            # ODOM
            # -------------------
            odom = Odometry()

            odom.header.stamp = now.to_msg()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"

            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y

            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw

            odom.twist.twist.linear.x = v
            odom.twist.twist.angular.z = w

            self.odom_pub.publish(odom)

            # -------------------
            # TF
            # -------------------
            tf = TransformStamped()

            tf.header.stamp = now.to_msg()
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_footprint"

            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y

            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(tf)

            # -------------------
            # joint states
            # -------------------
            js = JointState()

            js.header.stamp = now.to_msg()

            js.name = [
                "left_wheel_joint",
                "right_wheel_joint"
            ]

            js.position = [
                left_p
                * self.meters_per_pulse
                / self.wheel_radius,

                right_p
                * self.meters_per_pulse
                / self.wheel_radius
            ]

            js.velocity = [
                v_left / self.wheel_radius,
                v_right / self.wheel_radius
            ]

            self.joint_pub.publish(js)

        except Exception as e:
            self.get_logger().warn(
                f"Erro: {e}"
            )


# ============================================
# MAIN
# ============================================
def main(args=None):
    rclpy.init(args=args)

    node = BaseDriver()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()