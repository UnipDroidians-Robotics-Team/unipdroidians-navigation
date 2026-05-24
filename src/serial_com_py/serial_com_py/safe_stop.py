#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SafetyStop(Node):

    def __init__(self):
        super().__init__('safety_stop')

        self.stop_distance = 0.30 #distancia min em metros

        self.obstacle_detected = False
        self.last_cmd = Twist()

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        #esse trem pega o cmd_vel do teleop e publica um cmd_vel_safe
        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.safe_pub = self.create_publisher(
            Twist,
            '/cmd_vel_safe',
            10
        )

        self.get_logger().info(
            'Safety stop iniciado'
        )

    def scan_callback(self, msg):
        self.obstacle_detected = False
        angle = msg.angle_min
        for r in msg.ranges:
            # ignora leitura inválida
            if not math.isfinite(r):
                angle += msg.angle_increment
                continue
            # radianos -> graus
            deg = math.degrees(angle)
            # verifica a frente do robo, aproximadamente entre -35° e +35°
            if deg >= 120.0 or deg <= -120.0:
                if r < self.stop_distance:
                    self.obstacle_detected = True
                    return
            angle += msg.angle_increment

    def cmd_callback(self, msg):
        out = Twist()

        if self.obstacle_detected:
            # para o robo
            out.linear.x = 0.0
            out.angular.z = 0.0

            self.get_logger().warn(
                'OBSTÁCULO! PARANDO.'
            )

        else:
            out = msg

        self.safe_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()