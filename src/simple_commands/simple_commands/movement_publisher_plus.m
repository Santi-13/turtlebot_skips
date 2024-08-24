
movementNode = ros2node('/movement_publisher_node');

twistPub = ros2publisher(movementNode,'/cmd_vel','geometry_msgs/Twist');
twistPubMsg = ros2message(twistPub);

jointStateSub = ros2subscriber(movementNode, '/joint_states');
jointStateSubMsg = ros2message(jointStateSub);
[jointStatesData, status, statusText] = receive(jointStateSub);

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
xd = 0.2;
yd = 0.2;

% Constantes
vMax = 0.18;
wMax = 0.4;

while t < tf
   thetad = atan2((yd-y), (xd- x));

   d = sqrt( (x-xd)^2 + (y-yd)^2 );
   thetae = theta - thetad;

   if thetae > pi
       thetae = thetae - 2*pi;
   elseif thetae <= -pi
       thetae = thetae + 2*pi;
   end

   if thetae < pi/32 && thetae > -pi/32
        v = kpd * d;
        v = vMax * tanh(v / vMax);
   else
       v = 0;
   end

   w = -kpr * thetae;
   w = wMax * tanh(w / wMax);

   if d < 0.02 
       v = 0;
       w = 0;
       
   end

   x_dot = v * cos(theta);
   y_dot = v * sin(theta);
   theta_dot = w;

   twistPubMsg.linear.x = v;
   twistPubMsg.linear.y = 0.0;
   twistPubMsg.angular.z = theta_dot;

   % twistPubMsg.linear.x = 0;
   % twistPubMsg.linear.y = 0;
   % twistPubMsg.angular.z = 0;

   x = x + x_dot * dt;
   y = y + y_dot * dt;
   theta = theta + theta_dot*dt;

   send(twistPub,twistPubMsg);

   t = t + dt;

   figure(1)
   scatter(x,y,'color','blue','LineWidth',2);
   hold on
   plot([x,x+0.5*cos(theta)], [y,y+0.5*sin(theta)], 'color','red','LineWidth',2);
   hold off
   grid on
   axis([-1,1,-1,1]);
end




