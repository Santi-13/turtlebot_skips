clear all; close all; clc;

movementNode = ros2node('/movement_publisher_node');

% Suscriptor
imageSub = ros2subscriber(movementNode,'/image_raw', 'sensor_msgs/Image');

% Tiempo
t = 0;
tf = 10;

tic
while t < tf
   dt = toc;
   tic
   t = t + dt;

   figure(2)
   [imgData, imgStatus, imgStatusText] = receive(imageSub);
   a = rosReadImage(imgData);
   imshow(a);
   drawnow;
end
