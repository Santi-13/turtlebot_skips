% Iniciar el nodo
rosNode = ros2node("/aruco_publisher");

% Se crea el publicador
arucoPub = ros2publisher(rosNode, "/aruco_coordinates", "std_msgs/Float64MultiArray");

% Se crea el mensaje que se va a publicar (se va a actualizar con las coordenadas)
arucoMsg = ros2message("std_msgs/Float64MultiArray");

% Configurar la cámara
cam = webcam(2); % Cambiar el índice según la cámara disponible

% Parámetros de configuración
minMarkerSize = 20; % Tamaño mínimo en píxeles para considerar un marcador válido
thresholdConfidence = 0.8; % Umbral de confianza para detección

% Cargar los parámetros de calibración de la cámara
load('calibrationSession.mat'); % Ajusta la ruta al archivo .mat con los parámetros de calibración
intrinsics = cameraParams.Intrinsics;

% Verificar si hay problemas con los intrínsecos
if isempty(intrinsics)
    error('Los parámetros intrínsecos no se cargaron correctamente. Verifica la calibración.');
end

% Factor de conversión píxeles a milímetros (ajustar según calibración)
pixelsToMM = 0.3; % Valor de ejemplo, ajustar basado en el patrón de referencia

figure;
set(gcf, 'Name', 'Detección de ArUco con Cámara Web');

while ishandle(gcf)
    % Capturar una imagen desde la cámara web
    img = snapshot(cam);
    
    % Corregir la distorsión de la imagen usando los intrínsecos
    imgUndistorted = undistortImage(img, intrinsics);

    % Mostrar la imagen corregida
    imshow(imgUndistorted);
    hold on;

    % Detectar los marcadores ArUco en la imagen corregida
    [ids, locs, detectedFamily, confidences] = readArucoMarker(imgUndistorted);

    % Verificar que hay marcadores detectados
    if ~isempty(ids)
        % Filtrar marcadores por tamaño y confianza
        validMarkers = [];
        coordinates = []; % Inicializar una lista para almacenar coordenadas
        for i = 1:length(ids)
            corners = locs(:,:,i);
            markerSize = sqrt(sum(diff(corners([1, 2], :), 1, 1).^2)); 
            if markerSize >= minMarkerSize && confidences(i) >= thresholdConfidence
                validMarkers = [validMarkers, i];
            end
        end

        % Dibujar y procesar cada marcador válido
        for i = validMarkers
            corners = locs(:,:,i);
            centroidX = mean(corners(:,1));
            centroidY = mean(corners(:,2));
            
            % Convertir a coordenadas del mundo (en mm)
            worldCoords = [centroidX * pixelsToMM, centroidY * pixelsToMM];
            coordinates = [coordinates; worldCoords]; % Almacenar las coordenadas

            % Dibujar en la imagen (opcional)
            plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], 'r-', 'LineWidth', 2);
            plot(centroidX, centroidY, 'go', 'MarkerSize', 10, 'LineWidth', 2);
            text(centroidX, centroidY + 20, sprintf('(%.1f mm, %.1f mm)', worldCoords(1), worldCoords(2)), ...
                'Color', 'cyan', 'FontSize', 10, 'HorizontalAlignment', 'center');
        end

        % Publicar las coordenadas en el tópico ROS 2 si se encontraron marcadores
        arucoMsg.data = coordinates(:); % Aplanar las coordenadas
        send(arucoPub, arucoMsg);
    end

    hold off;
    pause(0.1);
    drawnow;
end

% Liberar la cámara y el nodo
clear cam;
clear rosNode;
