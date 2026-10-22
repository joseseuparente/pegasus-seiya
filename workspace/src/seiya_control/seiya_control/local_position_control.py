#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

class LocalPositionControl(Node):
    def __init__(self):
        super().__init__('local_position_control')

        # Parâmetros
        self.waypoints_ = [
            [5.0, 0.0, 2.0],  # Movimento X
            [0.0, 5.0, 2.0],   # Movimento Y
            [-5.0, 0.0, 2.0],
            [0.0, -5.0, 2.0]
        ]
        self.current_wp_index = 0  # Começa apontando para a decolagem
        self.tolerance = 0.05 # Margem de aceitação (m)
        self.is_landing = False #  Flag para pouso
        self.is_taking_off = True # Flag para Takeoff

        # Clients
        self.arming_client_ = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client_ = self.create_client(SetMode, '/mavros/set_mode')
        self.land_client_ = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.takeoff_client_ = self.create_client(CommandTOL, '/mavros/cmd/takeoff')

        # Espera carregar serviços
        self.get_logger().info('Esperando serviços do MAVROS...')
        self.wait_for_services()
        self.get_logger().info('MAVROS pronto!')

        # Publishers
        self.target_coords_publisher_= self.create_publisher(PoseStamped, 'mavros/setpoint_position/local',10)
        self.publisher_timer_ = self.create_timer(0.05, self.publish_target_coords)

        # Subscribers
        self.pose_subscriber_ = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile_sensor_data)
    
    def wait_for_services(self):
        """Espera serviços estarem disponíveis"""
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
        """
        Define o modo de Voo.
        
        STABILIZED, GUIDED, RTL, LAND
        """
        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f'Modo de voo setado para: {mode}\n')
                return True
            else:
                self.get_logger().info(f'Falha ao setar modo de voo para: {mode}')
                return False
        else:
            self.get_logger().error('Falha no serviço de modo de voo.')
            return False
    
    def arm(self) -> bool:
        """Arma o drone"""
        request = CommandBool.Request()
        request.value = True

        future = self.arming_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Sucesso ao armar!\n')
                return True
            else:
                self.get_logger().warn('Falha ao armar.')
                return False
        else:
            self.get_logger().error('Falha no serviço de armar drone')
            return False
    
    def takeoff(self, altitude: float) -> bool:
        """
        Takeoff para altura específica

        Parâmetros:
            - Altitude em metros (float)
        """
        request = CommandTOL.Request()
        request.latitude = 0.0 # Posição atual
        request.longitude = 0.0
        request.altitude = altitude

        future = self.takeoff_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info(f'Takeoff acionado, altura: {altitude}m')
                return True
            else:
                self.get_logger().info(f'Falha ao acionar Takeoff.')
                return False
        else:
            self.get_logger().error('Falha no serviço de Takeoff.')
            return False
    
    def land_async(self) -> bool:
        """Versão assíncrona do pouso para segurança"""
        request = CommandTOL.Request()
        request.altitude = 0.0

        future = self.land_client_.call_async(request)

    def publish_target_coords(self) -> bool:
        """Realiza a transmissão das coordenadas (waypoints) a 20hz."""
        # Apenas inicia transmissão após drone estar voando
        if self.is_taking_off:
            return
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # Controle para evitar acesso fora de alcance do vetor de waypoints
        if self.current_wp_index < len(self.waypoints_):
            current_target = self.waypoints_[self.current_wp_index]
        else:
            current_target = self.waypoints_[-1]

        msg.pose.position.x = float(current_target[0])
        msg.pose.position.y = float(current_target[1])
        msg.pose.position.z = float(current_target[2])
        msg.pose.orientation.w = 1.0 # Preserva a yaw

        self.target_coords_publisher_.publish(msg)
    
    def pose_callback(self, msg: PoseStamped):
        """
        Callback de telemetria do MTF01
        """
        # Verifica se está pousando
        if self.is_landing or self.current_wp_index >= len(self.waypoints_):
            return

        current_x = msg.pose.position.x
        current_y = msg.pose.position.y
        current_z = msg.pose.position.z

        target = self.waypoints_[self.current_wp_index]

        # Cálcula o módulo da distância 3D
        distance = math.sqrt(
            (current_x - target[0]) ** 2 +
            (current_y - target[1]) ** 2 +
            (current_z - target[2]) ** 2
        )

        if self.is_taking_off:
            if current_z >= 1.95:  # Tolerância para considerar que chegou nos 2m de altura
                self.get_logger().info("Decolagem concluída! Assumindo navegação local")
                self.is_taking_off = False
            return

        # Verifica se a distância está no raio da tolerância
        if distance < self.tolerance:
            self.get_logger().info(f"Waypoint {self.current_wp_index} alcançado! (Distância do alvo: {distance:.2f}m)")

            self.current_wp_index += 1

            if self.current_wp_index < len(self.waypoints_):
                next_point = self.waypoints_[self.current_wp_index]
                self.get_logger().info(f"Navegando para o próximo alvo: X={next_point[0]}, Y={next_point[1]}, Z={next_point[2]}")
                time.sleep(1)
            else:
                self.get_logger().info("Missão concluída. Iniciando procedimento de pouso...")
                self.is_landing = True
                self.land_async()

def main(args=None):
    rclpy.init(args=args)
    
    node = LocalPositionControl()

    node.get_logger().info('=== Missão Teste: Voo Utilizando Telemetria do MTF01 ===\n')

    # Procedimento inicial, caso esteja tudo OK
    if node.set_mode("GUIDED"):
        if node.arm():
            node.takeoff(2.0)
    
    # Loop principal
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Execução interrompida pelo usuário.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()