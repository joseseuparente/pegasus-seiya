#!/usr/bin/env python3
# LEIA COM ATENÇÃO:
#   É esperado que o drone tenha passado por todas as verificações de pre-arm 
#   (Calibração, Bateria, GPS...) antes de executar o script.

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

# Serviços utilizados
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, SetMode

class FlightControl(Node):
    def __init__(self):
        super().__init__('flight_control')

        # Parâmetros
        self.declare_parameter("altitude", 5.0)
        self.altitude_ = self.get_parameter("altitude").value

        # Clients
        self.arming_client_ = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client_ = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client_ = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client_ = self.create_client(CommandTOL, '/mavros/cmd/land')
        self.set_home_client_ = self.create_client(CommandHome, '/mavros/cmd/set_home')

        # Espera carregar serviços
        self.get_logger().info('Esperando serviços do MAVROS...')
        self.wait_for_services()
        self.get_logger().info('MAVROS pronto!')
    
    def wait_for_services(self):
        """Espera serviços estarem disponíveis"""
        services_list = [
            self.arming_client_,
            self.set_mode_client_,
            self.takeoff_client_,
            self.land_client_,
            self.set_home_client_,
        ]

        for service in services_list:
            while not service.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('Esperando serviços...')

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
    
    def disarm(self) -> bool:
        """Disarma o drone"""
        request = CommandBool.Request()
        request.value = False

        future = self.arming_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Sucesso no disarme!\n')
                return True
            else:
                self.get_logger().warn('Falha no disarme.')
                return False
        else:
            self.get_logger().error('Falha no serviço de disarme do drone')
            return False           

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

    def takeoff(self, altitude: float) -> bool:
        """
        Takeoff para altura específica

        Parâmetros:
            - Altitude em metros (float)
        """
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
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

    def land(self) -> bool:
        """Pousa o drone"""
        request = CommandTOL.Request()
        request.min_pitch = 0.0
        request.yaw = 0.0
        request.latitude = 0.0
        request.longitude = 0.0
        request.altitude = 0.0

        future = self.land_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Pouso acionado!\n')
                return True
            else:
                self.get_logger().info('Falha ao acionar pouso.')
                return False
        else:
            self.get_logger().error('Falha no serviço de pouso.')
            return False

    def set_home_current(self) -> bool:
        """Define a home como local atual"""
        request = CommandHome.Request()
        request.current_gps = True

        future = self.set_home_client_.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().success:
                self.get_logger().info('Home setada como local atual.\n')
                return True
            else:
                self.get_logger().info('Falha ao setar home.')
                return False
        else:
            self.get_logger().error('Falha no serviço de home.')
            return False


def main(args=None):
    rclpy.init(args=args)
    flight_control = None
    try:
        flight_control = FlightControl()

        flight_control.get_logger().info('=== Missão Teste: Sequência de Voo Básica ===\n')

        import time
        time.sleep(2)

        # Define a Home
        flight_control.get_logger().info('Definindo a home para localização atual...')
        if not flight_control.set_home_current():
            flight_control.get_logger().warn('Falha ao definir home, continuando de qualquer forma...')
        time.sleep(2)

        # Seta o drone para GUIDED
        flight_control.get_logger().info('Definindo o modo de voo para GUIDED...')
        if not flight_control.set_mode('GUIDED'):
            flight_control.get_logger().error('Falha ao definir para GUIDED. Encerrando execução...')
            return
        time.sleep(2)

        # Armando o drone
        flight_control.get_logger().info('Armando drone...')
        if not flight_control.arm():
            flight_control.get_logger().error('Falha ao armar. Encerrando execução...')
            return
        time.sleep(2)

        # Takeoff para a altitude definida
        flight_control.get_logger().info(f'Takeoff inciado para {flight_control.altitude_} metros...')
        if not flight_control.takeoff(flight_control.altitude_):
            flight_control.get_logger().error('Falha no Takeoff. Encerrando execução...')
            return
        flight_control.get_logger().info('Iniciando Takeoff...\n')
        time.sleep(15) # Espera o takeoff

        # Pouso
        flight_control.get_logger().info('Iniciando pouso...')
        if not flight_control.land():
            flight_control.get_logger().error('Falha no pouso. Encerrando execução...')
            return
        time.sleep(2)

        flight_control.get_logger().info('=== Missão Concluída ===')

    except KeyboardInterrupt:
        flight_control.get_logger().info('Voo interrompido pelo usuário')
    except Exception as e:
        flight_control.get_logger().error(f'Ocorreu um erro: {e}')
    finally:
        flight_control.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
