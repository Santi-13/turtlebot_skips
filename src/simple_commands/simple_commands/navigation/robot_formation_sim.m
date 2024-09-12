close all; clear all; clc;

%% Tiempo
tf = 15;
t = 0;
dt = 0.01;

%% Estados Iniciales de Robots
% Robot 1
% p1 = [rand()*4-2; rand()*4-2];
p1 = [0; 0.3];
theta1 = 0;

% Robot 2
% p2 = [rand()*4-2; rand()*4-2];
p2 = [0; -0.3];
theta2 = 0;

% Robot Lider (Virtual)
pl = [0; 0];
thetal = 0;

%% Objetivos Lider (Robot Virtual)
x_path = [rand()*2-1,     rand()*2-1,    rand()*2-1,    rand()*2-1    ];
y_path = [rand()*2-1,     rand()*2-1,    rand()*2-1,    rand()*2-1    ];
counter = 1;    % Contador para elegir que punto objetivo hay que seguir

% Posiciones deseadas
pd1 = [0; 0];
pd2 = [0; 0];
pdl = [0; 0];

%% Ganancias
kpd = 8;
kpr = 16;

%% Variables
dd = 0.3;         % Distancia entre robots
vMax = 2;
wMax = 8*pi;
v_extra = 0.5;

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
    
    de_1 = d_1l - dd;   % Error distancia robot 1
    de_2 = d_2l - dd;   % Error distancia robot 2 

    thetald = atan2( ( pdl(2) - pl(2) ), ( pdl(1) - pl(1) ) );

    thetael = thetal - thetald;
    if abs(thetael) > pi
        thetael = thetael - sign(thetael)*2*pi;
    end

    if abs(thetael) > pi/2
        % Reverse direction
         thetael = thetael - sign(thetael) * pi;
         d_l = -d_l;
    end  

    %% Leyes de control Velocidad
    v1_d = -vMax* tanh( ( ( ( kpd*d_1d )^3 ) / vMax )^3 ) * ( ( p1 - pd1 ) / d_1d );
    v1_l = -vMax* tanh( ( ( ( kpd*de_1 )^3 ) / vMax )^3 ) * ( ( p1 - pl )  / d_1l );
    v1 = v1_d + v1_l;
    
    v2_d = -vMax* tanh( ( ( ( kpd*d_2d )^3 ) / vMax )^3 ) * ( ( p2 - pd2 ) / d_2d );
    v2_l = -vMax* tanh( ( ( ( kpd*de_2 )^3 ) / vMax )^3 ) * ( ( p2 - pl )  / d_2l );
    v2 = v2_d + v2_l;

    vl = vMax* tanh( ( kpd*d_l ) / vMax);
    
    %% Cálculo de referencias
    theta1d = atan2( v1(2), v1(1) );
    theta2d = atan2( v2(2), v2(1) );

    %% Cálculo de errores
    thetae1 = theta1 - theta1d;
    if abs(thetae1) > pi
        thetae1 = thetae1 - sign(thetae1)*2*pi;
    end

    thetae2 = theta2 - theta2d;
    if abs(thetae2) > pi
        thetae2 = thetae2 - sign(thetae2)*2*pi;
    end
    
    %% Condicional para reversa
    v1_norm = norm(v1);
    v2_norm = norm(v2);

    [thetae1, v1_norm] = differential_reverse(thetae1, v1_norm);
    [thetae2, v2_norm] = differential_reverse(thetae2, v2_norm);

    %% Leyes de control velocidad angular
    w1 = -wMax * tanh( ( kpr*thetae1 ) / wMax);
    w2 = -wMax * tanh( ( kpr*thetae2 ) / wMax);
    wl = -wMax * tanh( ( kpr*thetael ) / wMax);
    
    if abs(thetael) > pi/32
        vl = 0;
    else
        vl = vMax* tanh( ( kpd*d_l ) / vMax);
    end

 

    %% Simulación del robot
    p1_dot = [v1_norm*cos(theta1); v1_norm*sin(theta1)];
    theta1_dot = w1;

    p2_dot = [v2_norm*cos(theta2); v2_norm*sin(theta2)];
    theta2_dot = w2;

    pl_dot = [vl*cos(thetal); vl*sin(thetal)];
    thetal_dot = wl;
    
    p1 = p1 + p1_dot * dt;
    theta1 = theta1 + theta1_dot*dt;

    p2 = p2 + p2_dot * dt;
    theta2 = theta2 + theta2_dot*dt;
    
    pl = pl + pl_dot * dt;
    thetal = thetal + thetal_dot*dt;

    v_extra = 0.5;
    %% Change to next objective point
    if abs(d_l) < 0.02  
       counter = rem(counter,length(x_path))+1;
       % counter = counter + 1;
       v_extra = 0;
    end

    t = t + dt;

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
    legend('Robot 1', 'Robot 2', 'Virtual Leader')
   

    hold off
    grid on
    axis([-2,2,-2,2]);
end