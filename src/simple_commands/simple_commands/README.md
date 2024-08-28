# Proyecto de Control de Trayectoria y Despliegue de Imágenes

Este proyecto en MATLAB se divide en dos partes principales: el control de trayectoria de un robot y el despliegue de imágenes capturadas por una cámara conectada a un entorno ROS 2.

## Parte 1: Control de Trayectoria

### Descripción
La parte de control de trayectoria de este script se encarga de mover un robot hacia una posición objetivo `(xd, yd)` en el plano 2D, utilizando un controlador basado en la cinemática de un robot diferencial. El script calcula la velocidad lineal y angular necesarias para dirigir al robot desde su posición actual `(x, y, theta)` hacia la meta, asegurando que se mantenga en curso mientras minimiza el error de orientación.

### Componentes Principales
- **Inicialización del Nodo y Publicador ROS 2**:
    - `movementNode = ros2node('/movement_publisher_node');`: Crea un nodo ROS 2 para manejar las comunicaciones del movimiento.
    - `twistPub = ros2publisher(movementNode,'/cmd_vel','geometry_msgs/Twist');`: Configura un publicador para enviar comandos de velocidad al robot.
    - `twistPubMsg = ros2message(twistPub);`: Crea un mensaje de tipo Twist para enviar las velocidades lineales y angulares.

- **Parámetros de Control**:
    - `tf`, `dt`: Tiempo total de simulación y paso de tiempo.
    - `kpd`, `kpr`: Ganancias proporcionales para el control de distancia y orientación, respectivamente.
    - `vMax`, `wMax`: Velocidades máxima lineal y angular permitidas.

- **Bucle de Control**:
    - El bucle `while` se ejecuta durante `tf` segundos y calcula en cada iteración las velocidades `v` y `w` basadas en la distancia al objetivo y el error de orientación.
    - **Lógica de Control**:
        - Calcula la orientación deseada `thetad` hacia la meta y la compara con la orientación actual `theta`.
        - Ajusta la velocidad lineal `v` y la velocidad angular `w` para minimizar el error.
    - **Actualización de Estado**:
        - Se actualizan las posiciones `x` y `y`, así como la orientación `theta`, en cada paso de tiempo.
    - **Publicación de Comandos de Movimiento**:
        - Los valores calculados de `v` y `w` se envían al robot a través del mensaje `twistPubMsg`.

- **Visualización**:
    - El progreso del robot se visualiza utilizando `scatter` para mostrar la posición actual y `plot` para mostrar la orientación.

## Parte 2: Despliegue de Imágenes Capturadas de una Cámara

### Descripción
Esta parte del script maneja la recepción y visualización en tiempo real de imágenes capturadas por una cámara conectada a ROS 2. Las imágenes se reciben del tópico `/image_raw` y se muestran en una ventana gráfica de MATLAB.

## Pasos previos para que funcione el código

Antes de ejecutar el código en MATLAB, es necesario realizar la siguiente configuración en la máquina virtual con Ubuntu:

1. **Instalar el paquete para capturar imágenes**: Este paquete proporciona los nodos necesarios para capturar imágenes a partir de una cámara compatible con v4l2.
   ```bash
   sudo apt install ros-humble-v4l2-camera
2. **Verificar la conexión de la cámara**: Utiliza el siguiente comando para verificar que la cámara esté conectada correctamente.
   ```bash
   v4l2-ctl --list-devices
3. **Instalar RViz2**: RViz2 es un puerto de RViz para ROS 2, que proporciona una interfaz gráfica para visualizar el robot, datos de sensores, mapas, y más.
    ```bash
    sudo apt install ros-humble-rviz2
3. **EXTRA**: En caso de que se desee probar la conexión de la cámara de manera externa a MATLAB es necesario instalar rqt_image_view, la cual es herramienta permite visualizar imágenes de los tópicos de ROS 2 fuera de MATLAB.
    ```bash
    sudo apt install ros-humble-rqt-image-view
### Componentes Principales
- **Suscripción al Tópico de Imágenes**:
    - `imageSub = ros2subscriber(movementNode,'/image_raw', 'sensor_msgs/Image');`: Crea un suscriptor para recibir mensajes de tipo `sensor_msgs/Image` desde el tópico `/image_raw`.
    - `imageSubMsg = ros2message(imageSub);`: Inicializa el mensaje para el suscriptor.

- **Recepción y Visualización de Imágenes**:
    - Dentro del bucle de control, se utiliza `receive(imageSub)` para obtener la última imagen capturada.
    - `a = rosReadImage(imgData);`: Convierte los datos del mensaje de imagen en un formato que MATLAB pueda manejar.
    - `imshow(a);`: Muestra la imagen recibida en tiempo real en una ventana gráfica.

### Resultado
Durante la simulación, el script muestra simultáneamente la trayectoria del robot en el plano 2D y las imágenes capturadas por la cámara, lo que permite observar el entorno del robot mientras se mueve hacia su objetivo.
