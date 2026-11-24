#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from gpiozero import DigitalOutputDevice, Device
from gpiozero.pins.mock import MockFactory

class GpioControllerNode(Node):
    def __init__(self):
        super().__init__('gpio_controller')

        # Parâmetros editáveis
        self.declare_parameter('pin', 17)               # Numeração do Pino
        self.declare_parameter('active_high', True)     # Ajustar de acordo com o modo de ativação do Relé
        self.declare_parameter('initial_value', False)

        pin_number = self.get_parameter('pin').get_parameter_value().integer_value
        active_high = self.get_parameter('active_high').get_parameter_value().bool_value
        initial_value = self.get_parameter('initial_value').get_parameter_value().bool_value

        # Utiliza pino virtual caso não esteja rodando no Raspberry Pi
        if os.environ.get('GPIOZERO_PIN_FACTORY') == 'mock' or not os.path.exists('/dev/gpiomem'):
            Device.pin_factory = MockFactory()
            self.get_logger().info('Hardware GPIO não detectado. Usando GPIOZERO MockFactory (simulação).')

        # Inicializa pino digital
        self.output_device = DigitalOutputDevice(
            pin=pin_number,
            active_high=active_high,
            initial_value=initial_value,
        )
        
        # Cria o serviço ON/OFF
        self.srv = self.create_service(
            SetBool,
            '~/set_gpio',
            self.set_gpio_callback
        )

        self.get_logger().info(
            f'Node GPIO inciado no pino {pin_number} | Estado inicial: {self.output_device.value}'
        )

        # Função para alterar estado do GPIO
        # Entrada: Request e Response da interface SetBool padrão do ROS2
        # Saída: Response gerada após a alteração do estado

    def set_gpio_callback(self, request: SetBool.Request, response: SetBool.Response):
        if request.data:
            self.output_device.on()
            estado_str = 'ALTO (HIGH / 1)'
        else:
            self.output_device.off()
            estado_str = 'BAIXO (LOW / 0)'

        pino_atual = self.output_device.pin.state
        self.get_logger().info(f'Pino configurado para {estado_str} | {pino_atual}')

        response.success = True
        response.message = f'GPIO {self.output_device.pin.number} definido como {estado_str}'

        return response

    def destroy_node(self):
        self.output_device.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GpioControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
