# Proyecto de Control de Orientación con TurtleBot3 y ESP32

Este proyecto se encarga de recibir la orientación de un TurtleBot3 utilizando una IMU, y luego enviar el ángulo de orientación (theta) a un ESP32 mediante una conexión TCP. El ESP32 recibe este valor y puede utilizarlo para realizar acciones, como controlar un dispositivo físico (LED, motor, etc.), aunque en este caso solo se muestra el funcionamiento de la conexión.

![DemostracionRobot-ESP-matlab](https://github.com/user-attachments/assets/6b98e940-6ad1-4ac2-801e-0db319bedeb7)

## Requisitos

- **TurtleBot3** con IMU configurada y disponible en el tópico `/imu`.
- **ESP32** con soporte Wi-Fi.
- **MATLAB** con soporte para ROS 2 instalado.
- Conexión Wi-Fi compartida entre el TurtleBot3 y el ESP32.

## Descripción del Flujo

1. **MATLAB** obtiene la orientación del TurtleBot3 a través de su IMU.
2. El valor del ángulo de orientación (theta) es convertido a grados.
3. MATLAB establece una conexión TCP con el ESP32 y envía el valor de theta.
4. **ESP32** actúa como servidor TCP, recibe el valor de theta y puede utilizarlo para acciones de control físico.

En general, este archivo `README.md` describe el propósito del proyecto, los pasos necesarios para configurarlo, y cómo funciona la comunicación entre el TurtleBot3 y el ESP32 mediante MATLAB y TCP.

A continuación se muestra el video completo:
https://github.com/user-attachments/assets/dba6834e-6b30-4563-8294-68a76a86949d
