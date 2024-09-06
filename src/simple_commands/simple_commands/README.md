# Simple Commands

Este directorio contiene los comandos básicos y scripts desarrollados durante el proyecto para la simulación, control, y comunicación de los robots móviles, así como la integración con el ESP32. Cada subdirectorio aborda un aspecto específico del sistema robótico que fue implementado durante distintas etapas del proyecto.

## Estructura de carpetas

- **`navigation/`**: Scripts relacionados con la navegación de los robots, incluyendo la simulación de algoritmos de control en MATLAB y el control de motores.
- **`esp32/`**: Código relacionado con la comunicación Wi-Fi entre la PC, el ESP32, y el robot, además del algoritmo de coordinación entre robots.

- **`sensors/`**: Código relacionado con la adquisición y procesamiento de datos de sensores. Principalmente incluye el código para el funcionamiento de la cámara, como la captura de imágenes en tiempo real y la detección de objetos de colores específicos. Los scripts en esta carpeta permiten visualizar la posición de los objetos detectados y calcular coordenadas relativas utilizando una cámara web. Además se incluye la implementación del filtro de Kalman.