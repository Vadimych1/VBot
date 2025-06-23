import math
import numpy as np
from numpy import cos, sin

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def inverse_kinematics(x, y, z, a, b):
    """
    Вычисляет углы для робота с двумя звеньями и тремя степенями свободы
    :param x, y, z: Координаты целевой точки
    :param a: Длина первого звена
    :param b: Длина второго звена
    :return: Список возможных решений [(theta1, theta2, theta3)], либо пустой список если точка недостижима
    """
    # Рассчитываем азимутальный угол (горизонтальное вращение)
    theta1 = math.atan2(y, x)
    
    # Радиальное расстояние в горизонтальной плоскости
    r = math.sqrt(x**2 + y**2)
    
    # Проверка достижимости точки
    D = math.sqrt(r**2 + z**2)
    if D > a + b or D < abs(a - b):
        return []  # Точка вне рабочей зоны
    
    # Вычисляем угол для третьего сустава (между звеньями)
    cos_theta3 = (r**2 + z**2 - a**2 - b**2) / (2 * a * b)
    cos_theta3 = max(min(cos_theta3, 1), -1)  # Ограничиваем значение [-1, 1]
    
    solutions = []
    for sign in [1, -1]:  # Два решения: "локоть вверх" и "локоть вниз"
        theta3 = sign * math.acos(cos_theta3)
        
        # Вычисляем коэффициенты для системы уравнений
        A = a + b * math.cos(theta3)
        B = b * math.sin(theta3)
        denom = A**2 + B**2
        
        # Решаем систему уравнений для theta2
        cos_theta2 = (A * r + B * z) / denom
        sin_theta2 = (A * z - B * r) / denom
        
        # Нормализуем углы
        theta2 = math.atan2(sin_theta2, cos_theta2)
        
        solutions.append((theta1, theta2, theta3))
    
    return solutions

# Пример использования
if __name__ == "__main__":
    # Параметры робота
    a = 2.0  # Длина первого звена
    b = 1.5  # Длина второго звена
    
    # Целевая точка
    target = (1.0, 1.0, 1.0)
    
    # Рассчитываем углы
    solutions = inverse_kinematics(target[0], target[1], target[2], a, b)
    
    if solutions:
        print(f"Для точки {target} найдены решения:")
        for i, (theta1, theta2, theta3) in enumerate(solutions):
            print(f"Решение {i+1}:")
            print(f"  theta1 (азимут): {math.degrees(theta1):.2f}°")
            print(f"  theta2 (наклон 1 звена): {math.degrees(theta2):.2f}°")
            print(f"  theta3 (угол между звеньями): {math.degrees(theta3):.2f}°")
            print()

        C, A, B = solutions[0]

        B = A + B - np.pi

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        x_0, y_0, z_0 = 0, 0, 0
        x_1, y_1, z_1 = x_0 + a * cos(A) * sin(C), y_0 + a * cos(A) * cos(C), z_0 + a * sin(A)
        x_2, y_2, z_2 = x_1 - b * cos(B) * sin(C), y_1 - b * cos(B) * cos(C), z_1 + b * sin(C)

        ax.scatter(
            [x_0, x_1, x_2, *[(x_0 * n + x_1 * (30 - n)) / 30 for n in range(1, 31)], *[(x_1 * n + x_2 * (30 - n)) / 30 for n in range(1, 31)]] + [target[0]],
            [y_0, y_1, y_2, *[(y_0 * n + y_1 * (30 - n)) / 30 for n in range(1, 31)], *[(y_1 * n + y_2 * (30 - n)) / 30 for n in range(1, 31)]] + [target[1]],
            [z_0, z_1, z_2, *[(z_0 * n + z_1 * (30 - n)) / 30 for n in range(1, 31)], *[(z_1 * n + z_2 * (30 - n)) / 30 for n in range(1, 31)]] + [target[2]],
            # s=[255, 255, 255] + ([255] * 60),
            c=[[1, 1, 0]] * 63 + [[1, 0, 1]]
        )

        plt.show()

    else:
        print("Точка недостижима")