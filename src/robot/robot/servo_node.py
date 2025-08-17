import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import board
import busio
from adafruit_pca9685 import PCA9685
import time


class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')

        # Inicializar I2C y PCA9685
        i2c = busio.I2C(board.SCL, board.SDA)
        self.pca = PCA9685(i2c)
        self.pca.frequency = 50  # 50 Hz -> estándar para servos

        # Diccionario con posiciones iniciales {canal: ángulo}
        self.posiciones_iniciales = {
            0: 0,
            2: 180,
            4: 0,
            6: 0,
            8: 150
        }

        # Configurar servos en posiciones iniciales
        self.mover_posiciones_iniciales()

        # Suscriptor al tópico "servo_commands"
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'servo_commands',
            self.listener_callback,
            10
        )

        self.get_logger().info("ServoNode listo. Esperando comandos en 'servo_commands'")

    # --- Conversión ángulo → duty_cycle ---
    def angle_to_duty_cycle(self, angle, min_us=500, max_us=2500):
        """
        Convierte un ángulo (0-180) en duty_cycle (0-65535) para PCA9685.
        min_us y max_us dependen del servo.
        """
        us_per_tick = 1000000 / (self.pca.frequency * 65536)  # duración de un tick en us
        pulse_us = min_us + (angle / 180.0) * (max_us - min_us)
        duty_cycle = int(pulse_us / us_per_tick)
        return duty_cycle

    def mover_posiciones_iniciales(self):
        for canal, angulo in self.posiciones_iniciales.items():
            duty = self.angle_to_duty_cycle(angulo)
            self.pca.channels[canal].duty_cycle = duty
            self.get_logger().info(f"Canal {canal}: inicializado a {angulo}°")
            time.sleep(0.3)

    def listener_callback(self, msg):
        """
        Espera un Float32MultiArray donde cada índice corresponde
        al servo en self.posiciones_iniciales.keys().
        """
        canales = list(self.posiciones_iniciales.keys())

        for i, angle in enumerate(msg.data):
            if i < len(canales):
                canal = canales[i]
                pwm_value = self.angle_to_duty_cycle(angle)
                self.pca.channels[canal].duty_cycle = pwm_value
                self.get_logger().info(f"Servo canal {canal}: {angle:.1f}° -> {pwm_value}")


def main(args=None):
    rclpy.init(args=args)
    node = ServoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
