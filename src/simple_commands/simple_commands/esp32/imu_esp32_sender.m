clear all; close all; clc;

imuPublisherNode = ros2node('/imu_esp32_sender');

% Suscriptores
imuSub = ros2subscriber(imuPublisherNode,'/imu','sensor_msgs/Imu');
imuSubMsg = ros2message(imuSub);

% Cliente TCP
% Dirección IP y puerto del servidor
serverIP1 = '10.42.0.130';
serverPort = 80;

% Pausa por un momento para permitir que el mensaje sea procesado
pause(0.1);

imuSubMsg = receive(imuSub, 3);

% Obtener la orientación en cuaterniones del mensaje de la IMU
quat = imuSubMsg.orientation;
angles = quat2eul([quat.w quat.x quat.y quat.z]);

% Convertir a ángulos de Euler
theta = rad2deg(angles(1));  % El ángulo de yaw (rotación alrededor del eje Z)
disp(['Orientation (theta): ', num2str(theta)]);

% Conectar al servidor TCP
t = tcpclient(serverIP1, serverPort);

% Enviar el ángulo de orientación al servidor
writeline(t, num2str(theta));