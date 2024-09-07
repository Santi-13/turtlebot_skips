# Proyecto AD2023: Sistema Ciberfísico Móvil Colaborativo e Intercomunicado

## Descripción
Este proyecto se enfoca en el desarrollo de un sistema robótico móvil y colaborativo, telecontrolado, que integre tecnologías de robótica y comunicaciones para resolver problemas de navegación industrial. Se trabajará en colaboración con un socio formador, Intel Labs, atendiendo los requerimientos de normativas nacionales e internacionales.

El sistema debe tener:
- Al menos tres grados de libertad.
- Un sistema de control de motores y sensado de velocidad/posición.
- Comunicación inalámbrica para el control y monitoreo desde un sistema remoto.
- Resguardo de información y monitoreo de variables del sistema.

El sistema será evaluado mediante simulaciones y pruebas físicas, integrando diversas tecnologías y enfoques de control.

## Objetivos
- Desarrollar un sistema robótico móvil telecontrolado.
- Implementar comunicación inalámbrica para control y monitoreo.
- Diseñar e implementar algoritmos de control para navegación y colaboración entre robots.

## Requerimientos
1. **Comunicación**: El robot debe conectarse a una computadora remota mediante MATLAB y enviar/recibir datos vía Wi-Fi.
2. **Lectura de sensores**: Utilizar una ESP32 para leer un sensor IMU de 9 DoF.
3. **Control**: Implementar algoritmos de control en MATLAB para generar comandos de velocidad lineal y angular.
4. **Colaboración entre robots**: Usar una cámara web para obtener las posiciones iniciales de los robots y coordinar sus movimientos.

5. **Control embebido**: Controlar un motor de DC mediante un microcontrolador ESP32 y visualizar datos en MATLAB.
6. **Procesamiento de imágenes**: Integrar una cámara para leer imágenes desde MATLAB.

### Etapa 1: Investigación y Diseño
- Investigar el estado del arte en la automatización robótica.
- Diseñar la arquitectura de control y automatización del sistema.
- Proponer el diseño y la estrategia de integración de los componentes.

### Etapa 2: Implementación de Algoritmos de Control
- Simular algoritmos de control en MATLAB para el seguimiento de trayectorias.
- Implementar algoritmos colaborativos para varios robots.
- Realizar pruebas de control embebido para un motor de CD con la ESP32.

### Etapa 3: Integración Tecnológica y Construcción
- Integrar algoritmos de seguimiento de trayectoria utilizando comunicación inalámbrica.
- Demostrar la coordinación de múltiples robots en tiempo real.
- Presentar el funcionamiento final del sistema.

## Entregables
1. **Semana 1**: Diseño detallado de la solución del reto (reporte en formato IEEE). Esto se puede encontrar en [`/src/documentation/`](src/documentation/).
2. **Semana 4**: Simulación de algoritmos de control en MATLAB y control de motores. Esto se puede encontrar en [`/src/simple_commands/simple_commands/navigation/`](src/simple_commands/simple_commands/navigation/).
3. **Semana 5**: Enlace de comunicación Wi-Fi entre la PC, ESP32 y robot, y algoritmo de coordinación entre robots. Esto se puede encontrar en [`/src/simple_commands/simple_commands/esp32/`](src/simple_commands/simple_commands/esp32/).
4. **Semana 6**: Presentación final del sistema robótico integrado y documentación técnica completa.

## Requisitos Técnicos
- MATLAB y CoppeliaSim para simulación y control.
- ESP32 para la lectura de sensores y control embebido.
- Turtlebot3 como robot móvil de referencia.

## Recursos
- [Turtlebot3 Burger](https://www.robotis.us/turtlebot-3-burger-us/)
- [Microcontrolador ESP32](https://www.amazon.com.mx/desarrollo-Procesador-microcontrolador-Bluetooth-integrado/dp/B07RY9MVCV)
- Cámara web para el procesamiento de imágenes
