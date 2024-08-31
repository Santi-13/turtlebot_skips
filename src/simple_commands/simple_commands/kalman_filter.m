
% Subscribe to IMU topic
imuSub = rossubscriber('/imu/data', 'sensor_msgs/Imu');

% Kalman filter initialization
x = zeros(6,1);
A = eye(6);
H = [eye(3), zeros(3,3)];
Q = 0.01 * eye(6);
R = 0.1 * eye(3);
P = eye(6);

while true
    % Prediction step
    x = A * x;
    P = A * P * A' + Q;
    
    % Update step
    imuData = receive(imuSub, 10);
    z = [imuData.Orientation.X; imuData.Orientation.Y; imuData.Orientation.Z];
    y = z - H * x;
    S = H * P * H' + R;
    K = P * H' / S;
    x = x + K * y;
    P = (eye(6) - K * H) * P;
    
    % Display or use the estimated state
    disp(x);
end
