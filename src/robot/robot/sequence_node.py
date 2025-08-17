import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from robot.sequences import saludo, movimiento_par # Importar secuencias externas

class SequenceNode(Node):
    def __init__(self):
        super().__init__('sequence_node')

        # Publisher para enviar comandos al servo_node
        self.pub = self.create_publisher(Float32MultiArray, 'servo_commands', 10)

        # Subscriber para recibir el nombre de la secuencia
        self.sub = self.create_subscription(String, 'sequence_command', self.command_callback, 10)

        # Variables internas
        self.sequence = []
        self.step = 0
        self.timer_period = 0.05
        self.timer = None

        self.get_logger().info("SequenceNode listo y esperando comandos de secuencia")

    def command_callback(self, msg: String):
        name = msg.data.lower()
        self.get_logger().info(f"Recibido comando de secuencia: {name}")

        # Seleccionar la secuencia y su velocidad
        if name == 'saludo':
            self.sequence, self.timer_period = saludo()
        elif name == 'movimiento_par':
            self.sequence, self.timer_period = movimiento_par()
        else:
            self.get_logger().warn(f"Secuencia desconocida: {name}")
            return

        # Reiniciar paso y crear timer para ejecutar la secuencia
        self.step = 0
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        if self.step < len(self.sequence):
            msg = Float32MultiArray()
            msg.data = self.sequence[self.step]
            self.pub.publish(msg)
            self.step += 1
        else:
            self.get_logger().info("Secuencia completada")
            if self.timer is not None:
                self.timer.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = SequenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
