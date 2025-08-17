import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import board
from adafruit_pca9685 import PCA9685

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        # Inicializar PCA9685
        i2c = board.I2C()
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # servos típicos: 50 Hz

        # Suscriptor a comandos
        self.subscription = self.create_subscription(
            Float32MultiArray,   # formato: [angulo_servo1, angulo_servo2, ...]
            'servo_commands',
            self.listener_callback,
            10
        )

    def angle_to_pwm(self, angle):
        # conversión simple 0°–180° → pulso 500–2500us aprox
        pulse = 500 + (angle / 180.0) * 2000
        return int(pulse * 4096 / 20000)  # convertir a valor PCA9685 (20ms periodo)

    def listener_callback(self, msg):
        for i, angle in enumerate(msg.data):
            pwm_value = self.angle_to_pwm(angle)
            self.pca.channels[i].duty_cycle = pwm_value

def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
