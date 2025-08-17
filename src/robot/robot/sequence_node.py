import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import time

class SequenceNode(Node):
    def __init__(self):
        super().__init__('sequence_node')
        # Publisher para enviar comandos al servo_node
        self.pub = self.create_publisher(Float32MultiArray, 'servo_commands', 10)
        # Subscriber para recibir el nombre de la secuencia
        self.sub = self.create_subscription(String, 'sequence_command', self.command_callback, 10)

        self.sequence = []
        self.step = 0
        self.timer_period = 0.05
        self.timer = None

        self.get_logger().info("SequenceNode listo y esperando comandos de secuencia")

    # Callback al recibir el nombre de la secuencia
    def command_callback(self, msg: String):
        name = msg.data.lower()
        self.get_logger().info(f"Recibido comando de secuencia: {name}")

        # Generar la secuencia correspondiente
        if name == 'saludo':
            self.sequence = self.generate_saludo_sequence()
        elif name == 'levantar_brazo':
            self.sequence = self.generate_levantar_brazo_sequence()
        else:
            self.get_logger().warn(f"Secuencia desconocida: {name}")
            return

        # Reiniciar paso y timer para ejecutar la secuencia
        self.step = 0
        if self.timer is not None:
            self.timer.cancel()
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    # Timer para publicar pasos de la secuencia
    def timer_callback(self):
        if self.step < len(self.sequence):
            msg = Float32MultiArray()
            msg.data = self.sequence[self.step]
            self.pub.publish(msg)
            self.step += 1
        else:
            self.get_logger().info("Secuencia completada")
            self.timer.cancel()

    # --- Secuencias predefinidas ---
    def generate_saludo_sequence(self):
        seq = []
        for a in range(180, -1, -5):
            seq.append([0, a])
        for _ in range(2):
            for a in range(0, 31, 3):
                seq.append([a, 0])
            for a in range(30, -1, -3):
                seq.append([a, 0])
        for a in range(0, 181, 5):
            seq.append([0, a])
        return seq

    def generate_levantar_brazo_sequence(self):
        seq = []
        for a in range(0, 181, 5):
            seq.append([0, a])
        return seq

def main(args=None):
    rclpy.init(args=args)
    node = SequenceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
