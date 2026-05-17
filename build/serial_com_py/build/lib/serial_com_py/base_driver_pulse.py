#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from sensor_msgs.msg import JointState
import serial
import tf2_ros
import math

class BaseDriver(Node):
    def __init__(self):
        super().__init__('base_driver_pulse')

        self.largura_eixo         = 0.250
        self.diametro_roda        = 0.165
        self.raio_roda            = self.diametro_roda / 2.0
        self.pulsos_por_revolucao = 45.0
        self.circunferencia       = self.diametro_roda * math.pi
        self.metros_por_pulso     = self.circunferencia / self.pulsos_por_revolucao

        self.serial_port   = '/dev/ttyACM0'
        self.baud_rate     = 115200
        self.serial_buffer = ""

        self.last_pulsos_esq = 0
        self.last_pulsos_dir = 0
        self.first_read = True

        self.x  = 0.0
        self.y  = 0.0
        self.th = 0.0

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0)
            self.get_logger().info(f"UDH1 Base: Conectado ao Arduino em {self.serial_port}")
        except Exception as e:
            self.get_logger().error(f"Erro na Serial. Detalhes: {e}")
            self.ser = None

        self.cmd_sub        = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.odom_pub       = self.create_publisher(Odometry, 'odom', 10)
        self.joint_pub      = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.02, self.leitura_serial)  # 50 Hz

    def cmd_callback(self, msg):
        MAX_VEL_LINEAR  = 0.5
        MAX_VEL_ANGULAR = 1.0

        v_linear  = max(min(msg.linear.x,  MAX_VEL_LINEAR),  -MAX_VEL_LINEAR)
        v_angular = max(min(msg.angular.z, MAX_VEL_ANGULAR), -MAX_VEL_ANGULAR)

        v_left  = v_linear - (v_angular * self.largura_eixo / 2.0)
        v_right = v_linear + (v_angular * self.largura_eixo / 2.0)

        if self.ser and self.ser.is_open:
            comando = f"CMD:{v_left:.3f};{v_right:.3f}\n"
            try:
                self.ser.write(comando.encode('utf-8'))
            except Exception as e:
                self.get_logger().warn(f"Falha ao enviar: {e}")

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
        qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
        qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        return qx, qy, qz, qw

    def leitura_serial(self):
        if self.ser is None or not self.ser.is_open:
            return

        try:
            bytes_waiting = self.ser.in_waiting
            if bytes_waiting > 0:
                novo_dado = self.ser.read(bytes_waiting).decode('utf-8', errors='ignore')
                self.serial_buffer += novo_dado

                if '\n' in self.serial_buffer:
                    linhas = self.serial_buffer.split('\n')
                    self.serial_buffer = linhas[-1]
                    ultimo_dado_valido = None

                    for linha in linhas[:-1]:
                        linha_limpa = linha.strip()
                        if linha_limpa.startswith("ODO:"):
                            ultimo_dado_valido = linha_limpa

                    if ultimo_dado_valido:
                        data = ultimo_dado_valido.replace("ODO:", "").split(";")

                        if len(data) == 3:
                            p_esq = int(data[0])
                            p_dir = int(data[1])
                            dt_ms = int(data[2])

                            if dt_ms <= 0:
                                return
                            dt_sec = dt_ms / 1000.0

                            if self.first_read:
                                self.last_pulsos_esq = p_esq
                                self.last_pulsos_dir = p_dir
                                self.first_read = False
                                return

                            # Encoder esquerdo conta direção oposta ao direito
                            dp_esq = -(p_esq - self.last_pulsos_esq)
                            dp_dir =  (p_dir - self.last_pulsos_dir)

                            self.last_pulsos_esq = p_esq
                            self.last_pulsos_dir = p_dir

                            dist_esq_delta = dp_esq * self.metros_por_pulso
                            dist_dir_delta = dp_dir * self.metros_por_pulso

                            v_esq = dist_esq_delta / dt_sec
                            v_dir = dist_dir_delta / dt_sec

                            # FIX: negação global corrige RViz invertido
                            # (frente/trás/esq/dir na vida real estavam certos,
                            #  mas o robô andava ao contrário no RViz)
                            v_linear  = -((v_dir + v_esq) / 2.0)
                            v_angular = -((v_dir - v_esq) / self.largura_eixo)

                            current_time = self.get_clock().now()
                            self.x  += v_linear * math.cos(self.th) * dt_sec
                            self.y  += v_linear * math.sin(self.th) * dt_sec
                            self.th += v_angular * dt_sec

                            odom = Odometry()
                            odom.header.stamp    = current_time.to_msg()
                            odom.header.frame_id = "odom"
                            odom.child_frame_id  = "base_footprint"
                            odom.pose.pose.position.x = self.x
                            odom.pose.pose.position.y = self.y
                            qx, qy, qz, qw = self.quaternion_from_euler(0, 0, self.th)
                            odom.pose.pose.orientation.x = qx
                            odom.pose.pose.orientation.y = qy
                            odom.pose.pose.orientation.z = qz
                            odom.pose.pose.orientation.w = qw
                            odom.twist.twist.linear.x  = v_linear
                            odom.twist.twist.angular.z = v_angular
                            self.odom_pub.publish(odom)

                            t = TransformStamped()
                            t.header.stamp    = current_time.to_msg()
                            t.header.frame_id = "odom"
                            t.child_frame_id  = "base_footprint"
                            t.transform.translation.x = self.x
                            t.transform.translation.y = self.y
                            t.transform.rotation      = odom.pose.pose.orientation
                            self.tf_broadcaster.sendTransform(t)

                            dist_esq_total = -(p_esq) * self.metros_por_pulso
                            dist_dir_total =  (p_dir) * self.metros_por_pulso

                            js = JointState()
                            js.header.stamp = current_time.to_msg()
                            js.name         = ['left_wheel_joint', 'right_wheel_joint']
                            js.position     = [dist_esq_total / self.raio_roda,
                                               dist_dir_total / self.raio_roda]
                            js.velocity     = [v_esq / self.raio_roda,
                                               v_dir / self.raio_roda]
                            self.joint_pub.publish(js)

        except Exception as e:
            self.get_logger().warn(f"Ignorando pacote corrompido: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
