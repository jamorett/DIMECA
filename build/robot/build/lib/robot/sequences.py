import time
from board import SCL, SDA
import busio
from adafruit_pca9685 import PCA9685

def home():
    """
    Inicializa el PCA9685 y envía los servos a la posición inicial (home) del robot.
    """
    seq = []

    # Inicializar I2C y PCA9685
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Conversión de ángulo a duty_cycle
    def angle_to_duty_cycle(angle, min_us=500, max_us=2500):
        us_per_tick = 1000000 / (pca.frequency * 65536)
        pulse_us = min_us + (angle / 180.0) * (max_us - min_us)
        duty_cycle = int(pulse_us / us_per_tick)
        return duty_cycle

    # Posiciones iniciales
    posiciones_iniciales = {
        0: 0,
        2: 180,
        4: 180,
        6: 0,
        8: 150
    }

    # Aplicamos posiciones
    for canal, angulo in posiciones_iniciales.items():
        duty = angle_to_duty_cycle(angulo)
        print(f"Canal {canal}: {angulo}° -> duty {duty}")
        pca.channels[canal].duty_cycle = duty
        time.sleep(0.5)

    print("Servos movidos a la posición inicial")
    pca.deinit()

    speed = 0.05
    return seq, speed

def choca_5():
    seq=[]
    """
    Secuencia de choque de los 5 con ambos brazos:
    - Channel 2 y 6 se mueven simultáneamente
    """
    # Inicializar I2C y PCA9685
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Función para convertir ángulo a duty_cycle
    def angle_to_duty_cycle(angle, min_us=500, max_us=2500):
        us_per_tick = 1000000 / (pca.frequency * 65536)
        pulse_us = min_us + (angle / 180.0) * (max_us - min_us)
        return int(pulse_us / us_per_tick)

    # Paso 1: channel 2 -> 15°, channel 6 -> 175°
    pca.channels[2].duty_cycle = angle_to_duty_cycle(15)
    pca.channels[6].duty_cycle = angle_to_duty_cycle(175)
    time.sleep(2)

    # Paso 2: channel 2 -> 40°, channel 6 -> 150°
    pca.channels[2].duty_cycle = angle_to_duty_cycle(40)
    pca.channels[6].duty_cycle = angle_to_duty_cycle(150)
    time.sleep(1.5)

    # Paso 3: channel 2 -> 180°, channel 6 -> 0°
    pca.channels[2].duty_cycle = angle_to_duty_cycle(180)
    pca.channels[6].duty_cycle = angle_to_duty_cycle(0)

    # Liberar PCA9685 al final
    pca.deinit()

    print("✅ Secuencia de choque completada")
    speed=0.05
    return seq, speed

def saludo():
    """
    Secuencia de saludo.
    Inicializa su propio PCA9685 de manera independiente.
    """
    seq = []

    # Inicializar I2C y PCA9685
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Conversión de ángulo a duty_cycle
    def angle_to_duty_cycle(angle, min_us=500, max_us=2500):
        us_per_tick = 1000000 / (pca.frequency * 65536)
        pulse_us = min_us + (angle / 180.0) * (max_us - min_us)
        duty_cycle = int(pulse_us / us_per_tick)
        return duty_cycle

    # Construimos la secuencia de saludo
    for a in range(180, -1, -5):
        seq.append([0, a])
    for _ in range(2):
        for a in range(0, 31, 3):
            seq.append([a, 0])
        for a in range(30, -1, -3):
            seq.append([a, 0])
    for a in range(0, 181, 5):
        seq.append([0, a])

    # Ejecutamos la secuencia suavemente
    for pos in seq:
        for ch, ang in enumerate(pos):
            if ang != 0:
                pca.channels[ch].duty_cycle = angle_to_duty_cycle(ang)
        time.sleep(0.05)

    pca.deinit()
    speed = 0.05
    return seq, speed

def movimiento_par():
    """
    Secuencia de movimientos simultáneos de servos 2 y 6.
    Inicializa su propio PCA9685 de manera independiente.
    """
    seq = []

    # Inicializar I2C y PCA9685
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Conversión de ángulo a duty_cycle
    def angle_to_duty_cycle(angle, min_us=500, max_us=2500):
        us_per_tick = 1000000 / (pca.frequency * 65536)
        pulse_us = min_us + (angle / 180.0) * (max_us - min_us)
        duty_cycle = int(pulse_us / us_per_tick)
        return duty_cycle

    # Función para mover dos servos suavemente
    def move_servos_smooth(ch1, start1, end1, ch2, start2, end2, steps=2, delay=0.02):
        step1 = 1 if end1 > start1 else -1
        step2 = 1 if end2 > start2 else -1
        for a1, a2 in zip(range(start1, end1 + step1, steps * step1),
                          range(start2, end2 + step2, steps * step2)):
            pca.channels[ch1].duty_cycle = angle_to_duty_cycle(a1)
            pca.channels[ch2].duty_cycle = angle_to_duty_cycle(a2)
            time.sleep(delay)

    # Parte 1: Movimiento inicial simultáneo
    move_servos_smooth(2, 90, 50, 6, 90, 130)

    # Parte 2: Movimientos en pares
    move_servos_smooth(2, 110, 50, 6, 130, 70)
    move_servos_smooth(2, 50, 110, 6, 70, 130)

    # Parte 3: Retorno simultáneo
    move_servos_smooth(2, 110, 180, 6, 130, 0)

    pca.deinit()
    speed = 0.05
    return seq, speed


def run_sequence():
    """
    Secuencia completa de prueba de movimientos.
    Inicializa su propio PCA9685 de manera independiente.
    """
    seq = []

    # Inicializar I2C y PCA9685
    i2c = busio.I2C(SCL, SDA)
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Conversión de ángulo a duty_cycle
    def angle_to_pwm(angle):
        min_pulse = 500
        max_pulse = 2500
        pulse = min_pulse + (max_pulse - min_pulse) * (angle / 180)
        duty_cycle = int(pulse / 1000000 * pca.frequency * 65535)
        return duty_cycle

    # Movimiento simultáneo de 2 servos
    def move_servos_smooth(ch1, start1, end1, ch2, start2, end2, steps=2, delay=0.02):
        step1 = 1 if end1 > start1 else -1
        step2 = 1 if end2 > start2 else -1
        for a1, a2 in zip(range(start1, end1 + step1, steps * step1),
                          range(start2, end2 + step2, steps * step2)):
            pca.channels[ch1].duty_cycle = angle_to_pwm(a1)
            pca.channels[ch2].duty_cycle = angle_to_pwm(a2)
            time.sleep(delay)

    # --- Secuencia ---
    print("Parte 1: Movimiento inicial simultáneo")
    move_servos_smooth(2, 90, 50, 6, 90, 130)

    print("Parte 2: Movimientos en pares pero primero solo channel 2")
    # 1) Primer movimiento solo de channel 2
    pca.channels[2].duty_cycle = angle_to_pwm(110)
    time.sleep(0.3)

    # 2) Movimientos en pares (2 y 6) más suaves
    start_time = time.time()
    while time.time() - start_time < 3:
        move_servos_smooth(2, 110, 50, 6, 130, 70, steps=4, delay=0.025)
        move_servos_smooth(2, 50, 110, 6, 70, 130, steps=4, delay=0.025)

    print("Parte 3: Retorno simultáneo")
    move_servos_smooth(2, 110, 180, 6, 130, 0)

    pca.deinit()
    speed = 0.05
    return seq, speed
