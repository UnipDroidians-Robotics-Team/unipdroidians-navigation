#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from action_msgs.srv import CancelGoal


class DistanceWatchdog(Node):

    def __init__(self):
        super().__init__('distance_watchdog')

        self.declare_parameter('fator_desvio', 1.5)
        self.declare_parameter('margem_metros', 0.3)

        self.fator  = self.get_parameter('fator_desvio').value
        self.margem = self.get_parameter('margem_metros').value

        self.limite     = None
        self.percorrido = 0.0
        self.ultima_pos = None
        self.ativo      = False
        self.stop_ticks = 0

        self.create_subscription(PoseStamped, '/goal_pose', self.cb_goal, 10)
        self.create_subscription(Path,        '/plan',      self.cb_plan, 10)
        self.create_subscription(Odometry,    '/odom',      self.cb_odom, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cancel_cli = self.create_client(
            CancelGoal, '/navigate_to_pose/_action/cancel_goal'
        )

        self.create_timer(0.1, self.timer_cb)

        self.get_logger().info(
            f'[watchdog] iniciado | fator={self.fator} | margem={self.margem}m'
        )

    def cb_goal(self, _msg):
        self.limite     = None
        self.percorrido = 0.0
        self.ultima_pos = None
        self.ativo      = False
        self.get_logger().info('[watchdog] novo goal — acumulador resetado')

    def cb_plan(self, msg: Path):
        if len(msg.poses) < 2:
            return
        total = sum(
            math.hypot(
                msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x,
                msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y,
            )
            for i in range(1, len(msg.poses))
        )
        self.limite     = total * self.fator + self.margem
        self.percorrido = 0.0
        self.ultima_pos = None
        self.ativo      = True
        self.get_logger().info(
            f'[watchdog] plano={total:.2f}m | limite={self.limite:.2f}m'
        )

    def cb_odom(self, msg: Odometry):
        if not self.ativo:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.ultima_pos is not None:
            self.percorrido += math.hypot(
                x - self.ultima_pos[0],
                y - self.ultima_pos[1]
            )
        self.ultima_pos = (x, y)
        if self.limite is not None and self.percorrido > self.limite:
            self.get_logger().warn(
                f'[watchdog] LIMITE! percorrido={self.percorrido:.2f}m '
                f'> limite={self.limite:.2f}m → PARANDO'
            )
            self.acionar_parada()

    def acionar_parada(self):
        self.ativo      = False
        self.stop_ticks = 30
        if self.cancel_cli.service_is_ready():
            self.cancel_cli.call_async(CancelGoal.Request())

    def timer_cb(self):
        if self.stop_ticks > 0:
            self.cmd_pub.publish(Twist())
            self.stop_ticks -= 1


def main(args=None):
    rclpy.init(args=args)
    node = DistanceWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
