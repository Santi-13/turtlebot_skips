# Control de Motor DC usando PID y Super-Twisting con ESP32

Este proyecto implementa dos tipos de control para un motor DC con un encoder, utilizando una ESP32 como controlador. Se utilizó un puente H para controlar la velocidad del motor.

El objetivo es demostrar el uso de dos técnicas de control para regular la velocidad del motor:

- **Control PID** (Proporcional-Integral-Derivativo)
- **Control Super-Twisting** (un tipo de controlador de modo deslizante)

Cada controlador se implementó en un código separado para facilitar su comprensión y pruebas.

## Componentes

- **ESP32**
- **Motor DC con encoder** (150:1 Micro Metal Gearmotor HP 6V with Extended Motor Shaft - [Pololu 2386](https://www.pololu.com/product/2386))
- **Puente H** (para control de la dirección del motor)
- **Encoder** (retroalimentación del motor)

### Diagrama de conexión

- **Puente H**: Controla la velocidad del motor.
  - Pin **IN1** del puente H: conectado al pin 25 de la ESP32.
  - Pin **IN2** del puente H: conectado al pin 35 de la ESP32.
  - Pin **PWM** del puente H: conectado al pin 27 de la ESP32.
  
- **Encoder**: Proporciona la retroalimentación de la velocidad del motor.
  - Pin **Encoder**: conectado al pin 39 de la ESP32.
  
## Control PID

El controlador PID ajusta la velocidad del motor comparando la velocidad deseada (velocidad de referencia) con la velocidad real medida por el encoder, generando una señal de control que se envía al motor.

### Parámetros del Control PID

- **Kp (Proporcional)**: 1.0
- **Ki (Integral)**: 0.1
- **Kd (Derivativo)**: 0.01

El código PID utiliza las interrupciones del encoder para medir la velocidad del motor en pulsos por segundo (PPS) y ajustar la señal PWM en consecuencia.

### Código PID

Esto se puede encontrar en [`/src/simple_commands/simple_commands/motorDC/encoder/pid_encoder`](pid_encoder).

### Demostración del control PID

https://github.com/user-attachments/assets/3af94d67-de60-431b-80bf-8f4485e4f505

## Control Super-Twisting

El controlador Super-Twisting es una técnica robusta de control no lineal que ajusta la velocidad del motor mediante la integral del signo del error y la raíz cuadrada del error, proporcionando una respuesta más robusta frente a perturbaciones.

### Parámetros del Control Super-Twisting

- **k_ps (Proporcional ajustado)**: 3.0
- **k_is (Integral ajustado)**: 28.0

El controlador Super-Twisting es útil en escenarios donde el sistema puede estar sujeto a perturbaciones o incertidumbre.

### Código Super-Twisting

Esto se puede encontrar en [`/src/simple_commands/simple_commands/motorDC/encoder/supertwisting_encoder`](supertwisting_encoder).

### Demostración del control Super-Twisting

https://github.com/user-attachments/assets/254ae08d-baa2-45bc-8722-b9ab318eb921


## Instrucciones de Uso

1. Conecta el hardware según el diagrama.
2. Carga el código correspondiente en la ESP32 (PID o Super-Twisting).
3. Abre el monitor serial para visualizar las velocidades del motor y las señales de control.
4. Realiza ajustes en los parámetros del controlador según sea necesario para obtener el comportamiento deseado.



