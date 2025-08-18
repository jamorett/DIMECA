import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

def run_sequence():
    # Inicializar I2C y PCA9685
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Conversión de ángulos a duty_cycle
    def angle_to_pwm(angle):
        min_pulse = 500
        max_pulse = 2500
        pulse = min_pulse + (max_pulse - min_pulse) * (angle / 180)
        duty_cycle = int(pulse / 1000000 * pca.frequency * 65535)
        return duty_cycle

    # Movimiento suave de un servo
    def move_servo(channel, angle):
        pca.channels[channel].duty_cycle = angle_to_pwm(angle)

    # Movimiento simultáneo de 2 servos
    def move_servos_smooth(ch1, start1, end1, ch2, start2, end2, steps=2, delay=0.02):
        step1 = 1 if end1 > start1 else -1
        step2 = 1 if end2 > start2 else -1
        for a1, a2 in zip(range(start1, end1 + step1, steps * step1),
                          range(start2, end2 + step2, steps * step2)):
            move_servo(ch1, a1)
            move_servo(ch2, a2)
            time.sleep(delay)

    # --- Secuencia ---
    print("Parte 1: Movimiento inicial simultáneo")
    move_servos_smooth(2, 90, 50, 6, 90, 130)  # ambos a la vez

    print("Parte 2: Movimientos en pares pero primero solo channel 2")
    # 1) Primer movimiento solo de channel 2
    move_servo(2, 110)
    time.sleep(0.3)

    # 2) Movimientos en pares (2 y 6) más suaves
    start_time = time.time()
    while time.time() - start_time < 3:
        move_servos_smooth(2, 110, 50, 6, 130, 70, steps=4, delay=0.025)
        move_servos_smooth(2, 50, 110, 6, 70, 130, steps=4, delay=0.025)

    print("Parte 3: Retorno simultáneo")
    move_servos_smooth(2, 110, 180, 6, 130, 0)

    # Liberar PCA9685 al final
    pca.deinit()

# Para poder ejecutar directamente o importar sin correr la secuencia
if __name__ == "__main__":
    run_sequence()
import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

def run_sequence():
    # Inicializar I2C y PCA9685
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Conversión de ángulos a duty_cycle
    def angle_to_pwm(angle):
        min_pulse = 500
        max_pulse = 2500
        pulse = min_pulse + (max_pulse - min_pulse) * (angle / 180)
        duty_cycle = int(pulse / 1000000 * pca.frequency * 65535)
        return duty_cycle

    # Movimiento suave de un servo
    def move_servo(channel, angle):
        pca.channels[channel].duty_cycle = angle_to_pwm(angle)

    # Movimiento simultáneo de 2 servos
    def move_servos_smooth(ch1, start1, end1, ch2, start2, end2, steps=2, delay=0.02):
        step1 = 1 if end1 > start1 else -1
        step2 = 1 if end2 > start2 else -1
        for a1, a2 in zip(range(start1, end1 + step1, steps * step1),
                          range(start2, end2 + step2, steps * step2)):
            move_servo(ch1, a1)
            move_servo(ch2, a2)
            time.sleep(delay)

    # --- Secuencia ---
    print("Parte 1: Movimiento inicial simultáneo")
    move_servos_smooth(2, 90, 50, 6, 90, 130)  # ambos a la vez

    print("Parte 2: Movimientos en pares pero primero solo channel 2")
    # 1) Primer movimiento solo de channel 2
    move_servo(2, 110)
    time.sleep(0.3)

    # 2) Movimientos en pares (2 y 6) más suaves
    start_time = time.time()
    while time.time() - start_time < 3:
        move_servos_smooth(2, 110, 50, 6, 130, 70, steps=4, delay=0.025)
        move_servos_smooth(2, 50, 110, 6, 70, 130, steps=4, delay=0.025)

    print("Parte 3: Retorno simultáneo")
    move_servos_smooth(2, 110, 180, 6, 130, 0)

    # Liberar PCA9685 al final
    pca.deinit()

# Para poder ejecutar directamente o importar sin correr la secuencia
if __name__ == "__main__":
    run_sequence()
