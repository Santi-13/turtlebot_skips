# Sensors

Este directorio contiene scripts relacionados con la adquisición de datos de sensores y el procesamiento de imágenes en tiempo real utilizando MATLAB. Los scripts permiten capturar información visual a través de una cámara, detectar colores específicos y aplicar filtros de Kalman para mejorar la precisión de las mediciones.

## Video demostrativo

![Lectura_camara_interna_pres-ezgif com-optimize (1)](https://github.com/user-attachments/assets/6d210cb6-051c-40ef-aefb-b51ddec8bd9b)

## Estructura de archivos

- **`video_cel_v6.m`**: Script que utiliza una cámara web para capturar imágenes en tiempo real y detectar objetos basados en colores específicos (azul y amarillo). El script genera una cuadrícula en la imagen y calcula las coordenadas (X, Y) en función de los objetos detectados, permitiendo visualizar la posición relativa de los mismos. También dibuja líneas entre los objetos amarillos y calcula el punto medio entre ellos. Posterirormente se cambio para que identifique ArUcos para obtener sus cordenadas.
  
- **`kalman_filter.m`**: Implementación de un filtro de Kalman que se puede utilizar para mejorar las mediciones de posición obtenidas de los datos de los sensores, reduciendo el ruido y optimizando la precisión de los cálculos.

## Descripción del archivo `video_cel_v6.m`

Este script realiza las siguientes funciones principales:

1. **Inicialización de la cámara**: Se conecta a una cámara web y captura imágenes en tiempo real.
   
2. **Detección de objetos**: Convierte la imagen capturada del espacio RGB al espacio HSV y aplica máscaras para detectar regiones de color azul y amarillo.
   
3. **Procesamiento de imagen**: Utiliza operaciones morfológicas para limpiar las imágenes de ruido y mejorar la detección de los objetos basados en el color.

4. **Visualización**:
   - Dibuja un rectángulo alrededor de los objetos detectados (tanto azules como amarillos).
   - Dibuja una cuadrícula sobre la imagen para mostrar las coordenadas X e Y relativas a los objetos detectados.
   - Calcula y muestra las coordenadas del punto medio entre los objetos amarillos y la distancia entre el punto azul y este punto medio.
   
5. **Cálculo de coordenadas**: Convierte las coordenadas en píxeles a metros utilizando una función de conversión basada en el tamaño del objeto detectado.

## Uso del script

1. Asegúrate de que tu cámara está conectada y configurada correctamente en MATLAB.
2. Ejecuta el script `video_cel_v6.m` para iniciar la captura de video en tiempo real y la detección de objetos.
3. El sistema calculará automáticamente las coordenadas de los objetos detectados y las mostrará en la imagen.

## Función `pix2m`

Este script incluye la función `pix2m`, que convierte las coordenadas en píxeles a metros basándose en la distancia estimada del objeto. Esta función se utiliza para obtener una representación precisa de las coordenadas (X, Y) en el espacio físico.

## Próximos pasos

- **Kalman Filter**: Se puede mejorar la precisión del seguimiento de los objetos aplicando el filtro de Kalman, disponible en `kalman_filter.m`. Este filtro suavizará las coordenadas calculadas para reducir el ruido y mejorar la predicción de la trayectoria de los objetos.
