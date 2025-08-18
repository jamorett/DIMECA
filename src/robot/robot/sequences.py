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
        4: 160,
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

def abrazo():
    seq=[]
    """
    Secuencia de abrazo:
    - Mueve simultáneamente los servos según la descripción del usuario
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

    # Paso 1: channel 0 -> 90°, channel 2 -> 90°, channel 6 -> 90°, channel 8 -> 60°
    pca.channels[0].duty_cycle = angle_to_duty_cycle(90)
    pca.channels[2].duty_cycle = angle_to_duty_cycle(90)
    pca.channels[6].duty_cycle = angle_to_duty_cycle(90)
    pca.channels[8].duty_cycle = angle_to_duty_cycle(60)
    time.sleep(1.5)

    # Paso 2: channel 0 -> 0°, channel 8 -> 150°
    pca.channels[0].duty_cycle = angle_to_duty_cycle(0)
    pca.channels[8].duty_cycle = angle_to_duty_cycle(150)
    time.sleep(2)

    # Paso 3: channel 2 -> 180°, channel 6 -> 0°
    pca.channels[2].duty_cycle = angle_to_duty_cycle(180)
    pca.channels[6].duty_cycle = angle_to_duty_cycle(0)

    # Liberar PCA9685 al final
    pca.deinit()

    print("Secuencia de abrazo completada")

    speed=0.10
    return seq, speed

def saludo():
    seq=[]
    """
    Secuencia de saludo:
    - Channel 2 baja y sube el brazo
    - Channel 0 mueve el codo para saludar
    """
    # Inicializar PCA9685
    i2c = board.I2C()
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Conversión de ángulo a duty_cycle
    def set_servo_angle(channel, angle):
        pulse_length = 500 + (angle / 180) * 2000  # 500–2500 µs
        pwm_value = int((pulse_length / 20000) * 0xFFFF)
        pwm_value = max(0, min(0xFFFF, pwm_value))  # limitar rango
        pca.channels[channel].duty_cycle = pwm_value

    # Movimiento suave
    def move_servo_smooth(channel, start, end, steps=40, delay=0.02):
        for step in range(steps + 1):
            angle = start + (end - start) * (step / steps)
            set_servo_angle(channel, angle)
            time.sleep(delay)

    # ================== SECUENCIA: SALUDO ==================
    # 1) Channel 2 baja el brazo (180 → 0)
    print("Bajando brazo (Channel 2)")
    move_servo_smooth(2, 180, 0)

    # 2) Channel 0 hace movimiento de codo (0 → 30 → 0 → 30 → 0)
    print("Moviendo codo (Channel 0) para saludar")
    for _ in range(2):  # dos saludos
        move_servo_smooth(0, 0, 30, steps=20, delay=0.01)
        move_servo_smooth(0, 30, 0, steps=20, delay=0.01)

    # 3) Channel 2 sube el brazo (0 → 180)
    print("Regresando brazo a reposo (Channel 2)")
    move_servo_smooth(2, 0, 180)

    # ================== FIN ==================
    time.sleep(1)
    pca.deinit()
    print("Secuencia de saludo completada")
    speed=0.05
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

    print("Secuencia de choqua los 5 completada")
    speed=0.05
    return seq, speed

def baile():
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

import time
import board
from adafruit_pca9685 import PCA9685

def brazos_arriba():
    seq=[]
    """
    Secuencia de brazos arriba:
    - Mueve varios servos en simultáneo a ángulos intermedios y luego regresa a la posición inicial
    """
    # Inicializar PCA9685
    i2c = board.I2C()
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Conversión de ángulo a duty_cycle
    def set_servo_angle(channel, angle):
        pulse_length = 500 + (angle / 180) * 2000  # 500–2500 µs
        pwm_value = int((pulse_length / 20000) * 0xFFFF)
        pwm_value = max(0, min(0xFFFF, pwm_value))  # limitar rango
        pca.channels[channel].duty_cycle = pwm_value

    # Movimiento suave de varios servos simultáneamente
    def move_servos_smooth(channels, start_angles, end_angles, steps=50, delay=0.02):
        for step in range(steps + 1):
            for i, ch in enumerate(channels):
                angle = start_angles[i] + (end_angles[i] - start_angles[i]) * (step / steps)
                set_servo_angle(ch, angle)
            time.sleep(delay)

    # ================== SECUENCIA ==================
    channels = [0, 2, 6, 8]

    start_angles = [0,   180, 0, 150]   # posiciones iniciales
    mid_angles   = [30,   0, 180, 120]  # posiciones intermedias
    end_angles   = [0,   180, 0, 150]   # retorno a posición inicial

    print("Fase 1: moviendo servos en simultáneo")
    move_servos_smooth(channels, start_angles, mid_angles)

    time.sleep(1)  # pausa de 1 segundo

    print("Fase 2: regresando servos en simultáneo")
    move_servos_smooth(channels, mid_angles, end_angles)

    # ================== FIN ==================
    time.sleep(1)
    pca.deinit()
    print("Secuencia de brazos arriba completada")
    speed=0.05
    return seq, speed

import time
import board
from adafruit_pca9685 import PCA9685

def wakeup():
    seq=[]
    """
    Secuencia de estiramiento (wakeup):
    - Mueve varios servos en pasos simultáneos con pausas definidas
    """
    # Inicializar PCA9685
    i2c = board.I2C()
    pca = PCA9685(i2c)
    pca.frequency = 50

    # Conversión de ángulo a duty_cycle
    def set_servo_angle(channel, angle):
        pulse_length = 500 + (angle / 180) * 2000  # 500–2500 µs
        pwm_value = int((pulse_length / 20000) * 0xFFFF)
        pwm_value = max(0, min(0xFFFF, pwm_value))
        pca.channels[channel].duty_cycle = pwm_value

    # Movimiento suave de varios servos simultáneamente
    def move_servos_smooth(channels, start_angles, end_angles, steps=50, delay=0.02):
        for step in range(steps + 1):
            for i, ch in enumerate(channels):
                angle = start_angles[i] + (end_angles[i] - start_angles[i]) * (step / steps)
                set_servo_angle(ch, angle)
            time.sleep(delay)

    # ================== SECUENCIA ==================
    # Paso 1
    move_servos_smooth([2, 4, 6], [180, 160, 0], [0, 135, 180])
    time.sleep(3.25)

    # Paso 2
    move_servos_smooth([0, 8], [0, 150], [90, 60])
    time.sleep(1.5)

    # Paso 3
    move_servos_smooth([2, 4, 6], [0, 135, 180], [90, 160, 90])
    time.sleep(1.5)

    # Paso 4
    move_servos_smooth([0, 8], [90, 60], [0, 150])
    time.sleep(1)

    # Paso 5
    move_servos_smooth([2, 6], [90, 90], [180, 0])

    # ================== FIN ==================
    time.sleep(1)
    pca.deinit()
    print("Secuencia wakeup completada")
    speed=0.05
    return seq, speed
