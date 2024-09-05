% Initialize ROS 2 node and publishers
movementNode = ros2node('/movement_publisher_node');
twistPub1 = ros2publisher(movementNode,'/tb3_0/cmd_vel','geometry_msgs/Twist');
twistMsg1 = ros2message('geometry_msgs/Twist');
twistPub2 = ros2publisher(movementNode,'/cmd_vel','geometry_msgs/Twist');
twistMsg2 = ros2message('geometry_msgs/Twist');

% Time vector
tf = 20;
t = 0;
dt = 0.01;

% Set up constants
v = 0.0; % Constant speed (m/s)
duration = 10; % Duration to publish constant speed (seconds)

% Create Twist message
twistMsg1.linear.x = v;
twistMsg1.angular.z = 0;
twistMsg2.linear.x = v;
twistMsg2.angular.z = 0;

tic
while t < tf
    send(twistPub1, twistMsg1);
    send(twistPub2, twistMsg2);

    dt = toc;
    tic
    t = dt + t;
end

% Clean up
clear movementNode twistPub twistMsg;
