% Iniciar el nodo ROS 2
rosNode = ros2node("/aruco_subscriber");

% Crear un suscriptor al tópico "/aruco_coordinates"
arucoSub = ros2subscriber(rosNode, "/aruco_coordinates", "std_msgs/Float64MultiArray", @arucoCallback);

% Callback para procesar los datos recibidos
function arucoCallback(msg)
    coordinates = msg.data; % Obtener las coordenadas

    % Las coordenadas están en un array plano [X1, Y1, X2, Y2, ...]
    % Mostrar las coordenadas recibidas
    fprintf("Coordenadas recibidas:\n");
    for i = 1:2:length(coordinates)
        fprintf("X: %.2f mm, Y: %.2f mm\n", coordinates(i), coordinates(i+1));
    end
end
