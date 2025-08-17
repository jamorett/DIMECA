# sequences.py

def saludo():
    """Secuencia de saludo ya existente"""
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
    
    speed = 0.05  # 50 ms por paso
    return seq, speed


def movimiento_par():
    """Secuencia basada en tu segundo script: movimientos simultáneos de servos 2 y 6"""
    seq = []

    # Parte 1: Movimiento inicial simultáneo
    for a2, a6 in zip(range(90, 49, -2), range(90, 131, 2)):
        seq.append([0, a2, 0, 0, 0, 0, a6])  # solo channels 2 y 6 cambian

    # Parte 2: Movimientos en pares con bucle simulado
    # 1) Primer movimiento solo channel 2
    seq.append([0, 110, 0, 0, 0, 0, 130])  # channel 2 y 6
    # 2) Movimientos en pares más suaves (simulación de bucle)
    for _ in range(30):  # aproximadamente 3s con 0.025s por paso
        # hacia abajo
        for a2, a6 in zip(range(110, 49, -4), range(130, 69, -4)):
            seq.append([0, a2, 0, 0, 0, 0, a6])
        # hacia arriba
        for a2, a6 in zip(range(50, 111, 4), range(70, 131, 4)):
            seq.append([0, a2, 0, 0, 0, 0, a6])

    # Parte 3: Retorno simultáneo
    for a2, a6 in zip(range(110, 181, 5), range(130, -1, -5)):
        seq.append([0, a2, 0, 0, 0, 0, a6])

    speed = 0.05  # 50ms
    return seq, speed
