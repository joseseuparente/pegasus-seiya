#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode


class LocalPositionControl(Node):
    def __init__(self):
        super().__init__('local_position_control')

        # Parâmetros
        self.waypoints_ = [
            [5.0, 0.0, 2.0],
            [0.0, 5.0, 2.0],
            [-5.0, 0.0, 2.0],
            [0.0, -5.0, 2.0]
        ]
        self.current_wp_index = 0
        self.tolerance = 0.20  # 20 cm de margem
        self.is_landing = False
        self.is_taking_off = True
        self.initial_z = None
        self.target_takeoff_alt = 2.0
        self.wp_reached_time = None

        # Clients
        self.arming_client_ = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client_ = self.create_client(SetMode, '/mavros/set_mode')
        self.land_client_ = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.takeoff_client_ = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        self.get_logger().info('Esperando serviços do MAVROS...')
        self.wait_for_services()
        self.get_logger().info('MAVROS pronto!')

        # Publishers
        self.target_coords_publisher_ = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )
        self.publisher_timer_ = self.create_timer(0.05, self.publish_target_coords)

        # Subscribers
        self.pose_subscriber_ = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_sensor_data
        )

    def wait_for_services(self):
        services_list = [
            self.arming_client_,
            self.set_mode_client_,
            self.land_client_,
            self.takeoff_client_
        ]
        for service in services_list:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Esperando serviços...')

    def set_mode(self, mode: str) -> bool:
        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().mode_sent:
            self.get_logger().info(f'Modo de voo setado para: {mode}')
            return True
        self.get_logger().error(f'Falha ao setar modo de voo para: {mode}')
        return False

    def arm(self) -> bool:
        request = CommandBool.Request()
        request.value = True

        future = self.arming_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info('Sucesso ao armar!')
            return True
        self.get_logger().warn('Falha ao armar (verifique pre-arm / EKF).')
        return False

    def takeoff(self, altitude: float) -> bool:
        self.target_takeoff_alt = altitude
        request = CommandTOL.Request()
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = altitude

        future = self.takeoff_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Takeoff acionado, altura alvo: {altitude}m')
            return True
        self.get_logger().error('Falha ao acionar Takeoff.')
        return False

    def land_async(self):
        request = CommandTOL.Request()
        request.altitude = 0.0
        self.land_client_.call_async(request)

    def publish_target_coords(self):
        if self.is_taking_off or self.is_landing:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        idx = min(self.current_wp_index, len(self.waypoints_) - 1)
        current_target = self.waypoints_[idx]

        msg.pose.position.x = float(current_target[0])
        msg.pose.position.y = float(current_target[1])
        msg.pose.position.z = float(current_target[2])
        msg.pose.orientation.w = 1.0

        self.target_coords_publisher_.publish(msg)

    def pose_callback(self, msg: PoseStamped):
        if self.is_landing or self.current_wp_index >= len(self.waypoints_):
            return

        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        current_z = msg.pose.position.z

        if self.initial_z is None:
            self.initial_z = current_z

        # Verifica conclusão do takeoff com margem
        if self.is_taking_off:
            rel_z = current_z - self.initial_z
            if rel_z >= (self.target_takeoff_alt * 0.9):
                self.get_logger().info(f"Decolagem concluída (z={rel_z:.2f}m)! Assumindo navegação local.")
                self.is_taking_off = False
            return

        target = self.waypoints_[self.current_wp_index]
        distance = math.sqrt(
            (current_x - target[0]) ** 2 +
            (current_y - target[1]) ** 2 +
            (current_z - target[2]) ** 2
        )

        # Verifica chegada no waypoint
        if distance < self.tolerance:
            now = self.get_clock().now()
            if self.wp_reached_time is None:
                self.wp_reached_time = now
                self.get_logger().info(
                    f"Waypoint {self.current_wp_index} alcançado! (Erro: {distance:.2f}m). Aguardando 1s..."
                )
            elif (now - self.wp_reached_time).nanoseconds >= 1e9:
                self.wp_reached_time = None
                self.current_wp_index += 1

                if self.current_wp_index < len(self.waypoints_):
                    next_pt = self.waypoints_[self.current_wp_index]
                    self.get_logger().info(
                        f"Navegando para próximo alvo: X={next_pt[0]}, Y={next_pt[1]}, Z={next_pt[2]}"
                    )
                else:
                    self.get_logger().info("Missão concluída. Iniciando pouso...")
                    self.is_landing = True
                    self.land_async()
        else:
            self.wp_reached_time = None


def main(args=None):
    rclpy.init(args=args)
    node = LocalPositionControl()

    node.get_logger().info('=== Missão Teste: Navegação Local ===')

    if node.set_mode("GUIDED"):
        time.sleep(1.5)
        if node.arm():
            time.sleep(1.5)
            node.takeoff(1.9)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Execução interrompida pelo usuário.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()