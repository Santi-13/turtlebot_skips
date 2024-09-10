close all; clear all; clc;

% Initialize ROS 2 node and publishers
movementNode = ros2node('/movement_publisher_node');
twistPub1 = ros2publisher(movementNode,'/tb3_0/cmd_vel','geometry_msgs/Twist');
twistMsg1 = ros2message('geometry_msgs/Twist');
twistPub2 = ros2publisher(movementNode,'/cmd_vel','geometry_msgs/Twist');
twistMsg2 = ros2message('geometry_msgs/Twist');

% Subscriber
arucoSub = ros2subscriber(movementNode, '/aruco_coordinates', 'std_msgs/Float64MultiArray', @arucoCallback);
arucoMsg = ros2message(arucoSub);

%% Tiempo
tf = 50;
t = 0;
dt = 0.01;

%% Estados Iniciales de Robots
% Robot 1
% p1 = [rand()*4-2; rand()*4-2];
p1 = [0; 0.1];
theta1 = 0;

% Robot 2
% p2 = [rand()*4-2; rand()*4-2];kpd
p2 = [0; -0.1];
theta2 = 0;

% Robot Lider (Virtual)
pl = [0; 0];
thetal = 0;

%% Objetivos Lider (Robot Virtual)
x_path = [-0.4,     0.0,    -0.4     ];
y_path = [0.0,      0.0,    -0.4     ];
counter = 1;    % Contador para elegir que punto objetivo hay que seguir

% Posiciones deseadas
pd1 = [0; 0];
pd2 = [0; 0];
pdl = [0; 0];

%% Ganancias
kpd = 2;
kpr = 4;

%% Variables
dd = 0.1;         % Distancia entre robots
vMax = 0.15;
wMax = 4*pi;
v_extra = 0.05;

tic
while t < tf          
    pd1 = [ pl(1)+dd*cos(thetal+pi/2) ; pl(2)+dd*sin(thetal+pi/2)       ];
    pd2 = [ pl(1)+dd*cos(thetal-pi/2) ; pl(2)+dd*sin(thetal-pi/2)   ];
    pdl = [ x_path(counter)      ; y_path(counter)       ];
    

    %% Cálculo de errores
    d_l = norm(pl-pdl);        % Distancia líder pos objetivo
    d_1l = norm(p1-pl);        % Distancia robot 1 a líder
    d_1d = norm(p1-pd1);       % Distancia robot 1 a pos objetivo
    d_2l = norm(p2-pl);        % Distancia robot 2 a lider
    d_2d = norm(p2-pd2);       % Distancia robot 2 a pos objetivo
    
    de_1 = d_1l + d_1d - dd;   % Error distancia robot 1
    de_2 = d_2l + d_2d - dd;   % Error distancia robot 2 

    %% Cálculo de referencias
    theta1d = atan2( ( pd1(2) - p1(2) ), ( pd1(1) - p1(1) ) );
    theta2d = atan2( ( pd2(2) - p2(2) ), ( pd2(1) - p2(1) ) );
    thetald = atan2( ( pdl(2) - pl(2) ), ( pdl(1) - pl(1) ) );

    %% Cálculo de errores
    thetae1 = theta1 - theta1d;
    if abs(thetae1) > pi
        thetae1 = thetae1 - sign(thetae1)*2*pi;
    end

    thetae2 = theta2 - theta2d;
    if abs(thetae2) > pi
        thetae2 = thetae2 - sign(thetae2)*2*pi;
    end

    thetael = thetal - thetald;
    if abs(thetael) > pi
        thetael = thetael - sign(thetael)*2*pi;
    end

    %% Condicional para reversa
    [thetae1, de_1] = differential_reverse(thetae1, de_1);
    [thetae2, de_2] = differential_reverse(thetae2, de_2);
    [thetael, d_l] = differential_reverse(thetael, d_l);

    %% Leyes de control
    v1 = vMax* tanh( ( kpd*de_1 ) / vMax) + sign(de_1) * v_extra;
    w1 = -wMax * tanh( ( kpr*thetae1 ) / wMax);
    
    v2 = vMax* tanh( ( kpd*de_2 ) / vMax) + sign(de_1) * v_extra;
    w2 = -wMax * tanh( ( kpr*thetae2 ) / wMax);

    vl = vMax* tanh( ( kpd*d_l ) / vMax);
    wl = -wMax * tanh( ( kpr*thetael ) / wMax);

    if abs(thetae1) > pi/32
        v1 = 0;
    else
        v1 = vMax* tanh( ( kpd*d_l ) / vMax);
    end

    if abs(thetae2) > pi/16
        v2 = 0;
    else
        v2 = vMax* tanh( ( kpd*d_l ) / vMax);
    end

    if abs(thetael) > pi/16
        vl = 0;
    else
        vl = vMax* tanh( ( kpd*d_l ) / vMax);
    end

    if abs(de_1) < 0.02
       v1 = 0;
       w1 = 0;
    end

    if abs(de_2) < 0.02
       v2 = 0;
       w2 = 0;
    end

    % if counter > length(x_path)
    %     vl = 0;
    %     wl = 0;
    %     v1 = 0;
    %     w1 = 0;
    %     v2 = 0;
    %     w2 = 0;
    % end

    twistMsg1.linear.x = v1;
    twistMsg1.angular.z = w1;
    twistMsg2.linear.x = v2;
    twistMsg2.angular.z = w2;
   
    % twistMsg1.linear.x = 0;
    % twistMsg1.angular.z = 0;
    % twistMsg2.linear.x = 0;
    % twistMsg2.angular.z = 0;

    

    %% Simulación del robot
    p1_dot = [v1*cos(theta1); v1*sin(theta1)];
    theta1_dot = w1;

    p2_dot = [v2*cos(theta2); v2*sin(theta2)];
    theta2_dot = w2;

    pl_dot = [vl*cos(thetal); vl*sin(thetal)];
    thetal_dot = wl;
    
    p1 = p1 + p1_dot * dt;
    theta1 = theta1 + theta1_dot*dt;

    p2 = p2 + p2_dot * dt;
    theta2 = theta2 + theta2_dot*dt;
    
    pl = pl + pl_dot * dt;
    thetal = thetal + thetal_dot*dt;

    v_extra = 0.05;
    %% Change to next objective point
    if abs(d_l) < 0.02  
       counter = rem(counter,length(x_path))+1;
       % counter = counter + 1;
       v_extra = 0;
    end

    send(twistPub1, twistMsg1);
    send(twistPub2, twistMsg2);

    dt = toc;
    tic
    t = dt + t;

    figure(1)
    scatter(p1(1),p1(2),'color','blue','LineWidth',2);
    hold on
    scatter(p2(1),p2(2),'color','red','LineWidth',2);
    scatter(pl(1),pl(2),'color','black','LineWidth',2);
    scatter(pd1(1),pd1(2),'color','green','LineWidth',1);
    scatter(pd2(1),pd2(2),'color','green','LineWidth',1);
    plot([p1(1),p1(1)+0.1*cos(theta1)],[p1(2),p1(2)+0.1*sin(theta1)],'color','red','LineWidth',2)
    plot([p2(1),p2(1)+0.1*cos(theta2)],[p2(2),p2(2)+0.1*sin(theta2)],'color','red','LineWidth',2)
    plot([pl(1),pl(1)+0.1*cos(thetal)],[pl(2),pl(2)+0.1*sin(thetal)],'color','red','LineWidth',2)

    hold off
    grid on
    axis([-2,2,-2,2]);
end

% Clean up
clear movementNode twistPub twistMsg;

function [] = arucoCallback(msg)
    p1 = [msg.data(1); msg.data(2)] / 1000.0;
    p2 = [msg.data(3); msg.data(4)] / 1000.0;
end

