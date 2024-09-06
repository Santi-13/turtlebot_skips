% Comando Inicializar Camara del Turtlebot:
% ros2 run v4l2_camera v4l2_camera_node --ros-args -p image_size:="[320,240]" -p framerate:=30 --ros-args --param reliability:=best_effort --param history:=keep_last --param depth:=10

movementNode = ros2node('/movement_publisher_node');

twistPub = ros2publisher(movementNode,'/tb3_0/cmd_vel','geometry_msgs/Twist');
twistPubMsg = ros2message('geometry_msgs/Twist');  % Direct initialization

%jointStateSub = ros2subscriber(movementNode, '/joint_states', 'sensor_msgs/JointState', @readJointStates);
global v_r
global w_r

imageSub = ros2subscriber(movementNode,'/image_raw', 'sensor_msgs/Image');
imageSubMsg = ros2message(imageSub);

function readJointStates(msg)
    global v_r
    global w_r
    v_r = (msg.velocity(1) + msg.velocity(2)) / 2;
    w_r = (msg.velocity(1) - msg.velocity(2)) / 2;
end

% Estado del robot 
x = 0;
y = 0;
theta = 0;

% Tiempos
tf = 20;
t = 0;
dt = 0.01;

% Ganancias
kpd = 1;
kpr = 1;

% Posición objetivo
xd = 0.5;
yd = 0.5;

w = 0;
v = 0;

% Constantes
vMax = 0.18;
wMax = 0.4;

tic

while t < tf
   thetad = atan2((yd - y), (xd - x));

   d = sqrt((x - xd)^2 + (y - yd)^2);
   thetae = theta - thetad;

   %% Condicional para tomar rotación más corta
   if thetae > pi
       thetae = thetae - 2 * pi;
   elseif thetae <= -pi
       thetae = thetae + 2 * pi;
   end

   %% Condicional para reversa
   if thetae > pi / 2
       thetae = thetae - pi;
       d = -d;
   elseif thetae < -pi / 2
       thetae = thetae + pi;
       d = -d;
   end

   %% Condicional para avanzar cuando este orientado
   if thetae < pi / 32 && thetae > -pi / 32
        v = kpd * d + kpd * (v - v_r);
        v = vMax * tanh(v / vMax);
   else
       v = 0;
   end

   w = -kpr * thetae + kpr * (w - w_r);
   w = wMax * tanh(w / wMax);
   %% Area Objetivo
   if d < 0.02 
       v = 0;
       w = 0;
   end

   %% Calculo de derivadas
   x_dot = v * cos(theta);
   y_dot = v * sin(theta);
   theta_dot = w;

   %% Publicación de velocidades al robot
   twistPubMsg.linear.x = double(v);
   twistPubMsg.linear.y = 0.0;
   twistPubMsg.linear.z = 0.0;  % Explicitly set to zero
   twistPubMsg.angular.x = 0.0;  % Explicitly set to zero
   twistPubMsg.angular.y = 0.0;  % Explicitly set to zero
   twistPubMsg.angular.z = double(theta_dot);

   disp(class(twistPubMsg.linear.x));  % Should output 'double'
   disp(class(twistPubMsg.linear.y));  % Should output 'double'
   disp(class(twistPubMsg.linear.z));  % Should output 'double'
   disp(class(twistPubMsg.angular.x)); % Should output 'double'
   disp(class(twistPubMsg.angular.y)); % Should output 'double'
   disp(class(twistPubMsg.angular.z)); % Should output 'double'

   send(twistPub, twistPubMsg);

   x = x + x_dot * dt;
   y = y + y_dot * dt;
   theta = theta + theta_dot * dt;

   dt = toc;
   tic
   t = t + dt;

   figure(1)
   scatter(x, y, 'color', 'blue', 'LineWidth', 2);
   hold on
   plot([x, x + 0.5 * cos(theta)], [y, y + 0.5 * sin(theta)], 'color', 'red', 'LineWidth', 2);
   hold off
   grid on
   axis([-1, 1, -1, 1]);
end
