function aruco_publisher(cameraParams)
    % Iniciar el nodo
    rosNode = ros2node("/aruco_publisher");
    
    % Se crea el publicador
    arucoPub = ros2publisher(rosNode, "/aruco_coordinates", "std_msgs/Float64MultiArray");
    
    % Se crea el mensaje que se va a publicar (se va a actualizar con las coordenadas)
    arucoMsg = ros2message("std_msgs/Float64MultiArray");
    
    % Configurar la cámara
    cam = webcam(2); % Cambiar el índice según la cámara disponible
    
    disp(cam.AvailableResolutions);
    % Parámetros de configuración
    minMarkerSize = 20; % Tamaño mínimo en píxeles para considerar un marcador válido
    thresholdConfidence = 0.8; % Umbral de confianza para detección
    gridSize = 50; % Tamaño de cada celda de la cuadrícula en milímetros
    
    % Cargar los parámetros de calibración de la cámara
    intrinsics = cameraParams.Intrinsics;
    
    % Verificar si hay problemas con los intrínsecos
    if isempty(intrinsics)
        error('Los parámetros intrínsecos no se cargaron correctamente. Verifica la calibración.');
    end
    
    % Factor de conversión píxeles a milímetros (ajustar según calibración)
    pixelsToMM = 0.3; % Valor de ejemplo, ajustar basado en el patrón de referencia
    
    angleArray = [0; 0];
    
    % figure;
    % set(gcf, 'Name', 'Detección de ArUco con Cámara Web');
    
    coordinates = zeros([2,2]);
    
    while ishandle(gcf)
        % Capturar una imagen desde la cámara web
        img = snapshot(cam);
        
        % Corregir la distorsión de la imagen usando los intrínsecos
        imgUndistorted = undistortImage(img, intrinsics);
        
        % Obtener el tamaño de la imagen para la cuadrícula
        [imgHeight, imgWidth, ~] = size(imgUndistorted);        
        
        % Detectar los marcadores ArUco en la imagen corregida
        [ids, locs, detectedFamily, confidences] = readArucoMarker(imgUndistorted);
    
        % Filtrar marcadores por tamaño y confianza
        validMarkers = [];
        for i = 1:length(ids)
            corners = locs(:,:,i);
            markerSize = sqrt(sum(diff(corners([1, 2], :), 1, 1).^2)); 
            if markerSize >= minMarkerSize && confidences(i) >= thresholdConfidence
                validMarkers = [validMarkers, i];
            end
        end
        
        for i = validMarkers
            corners = locs(:,:,i);
                        
            centroidX = mean(corners(:,1));
            centroidY = mean(corners(:,2));            
            
            % Convertir los puntos del marcador a coordenadas del mundo en milímetros
            worldCoords = [centroidX * pixelsToMM, centroidY * pixelsToMM];
            coordinates = [coordinates; worldCoords]; % Almacenar las coordenadas
    
            % Identificar las esquinas inferiores del marcador y calcular ángulo
            bottomLineX = [corners(3,1), corners(4,1)];
            bottomLineY = [corners(3,2), corners(4,2)];
            
            % Calcular el ángulo de la línea inferior con respecto a la línea horizontal
            deltaX = bottomLineX(2) - bottomLineX(1);
            deltaY = bottomLineY(2) - bottomLineY(1);
            angleRad = atan2(deltaY, deltaX); % Ángulo en radianes
            angleArray(i) = angleRad;
            angleDeg = rad2deg(angleRad); % Convertir a grados
            
            % Mostrar el ángulo en la imagen
            midPointX = mean(bottomLineX);
            midPointY = mean(bottomLineY);

            % Mostrar la imagen corregida
            % imshow(imgUndistorted);
            % hold on;
            % 
            % % Dibujar la cuadrícula con coordenadas positivas desde la esquina superior izquierda
            % for x = 0:gridSize:imgWidth
            %     % Dibujar las líneas de la cuadrícula verticales
            %     line([x, x], [1, imgHeight], 'Color', [0, 1, 0, 0.5], 'LineWidth', 0.5);
            %     % Mostrar las coordenadas en milímetros ajustadas
            %     text(x, 20, sprintf('%.1f mm', x * pixelsToMM), 'Color', 'black', 'FontSize', 10, 'HorizontalAlignment', 'center');
            % end
            % 
            % for y = 0:gridSize:imgHeight
            %     % Dibujar las líneas de la cuadrícula horizontales
            %     line([1, imgWidth], [y, y], 'Color', [0, 1, 0, 0.5], 'LineWidth', 0.5);
            %     % Mostrar las coordenadas en milímetros ajustadas
            %     text(20, y, sprintf('%.1f mm', y * pixelsToMM), 'Color', 'black', 'FontSize', 10, 'HorizontalAlignment', 'right');
            % end    
            % 
            % plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], 'r-', 'LineWidth', 2);
            % 
            % plot(centroidX, centroidY, 'go', 'MarkerSize', 10, 'LineWidth', 2);
            % text(centroidX, centroidY, sprintf('ID: %d', ids(i)), 'Color', 'yellow', 'FontSize', 12, 'HorizontalAlignment', 'center');
            % 
            % plot(bottomLineX, bottomLineY, 'b-', 'LineWidth', 2);
            % text(midPointX, midPointY, sprintf('%.2f°', angleDeg), 'Color', 'cyan', 'FontSize', 12, 'HorizontalAlignment', 'center');
            % 
            % hold off;
            % 
            % drawnow;

            fprintf('ID: %d, World X: %.2f mm, World Y: %.2f mm\n', ids(i), worldCoords(1), worldCoords(2));
            text(centroidX, centroidY + 20, sprintf('(%.1f mm, %.1f mm)', worldCoords(1), worldCoords(2)), ...
                'Color', 'cyan', 'FontSize', 10, 'HorizontalAlignment', 'center');
            % Imprimir el ángulo en la consola
            fprintf('ID: %d, Ángulo con la horizontal: %.2f°\n', ids(i), angleDeg);
            disp(ids)
        end
         % Publicar las coordenadas en el tópico ROS 2 si se encontraron marcadores
        
        zeroIdIndex = 0;
        oneIdIndex = 0;
        for i=1: length(ids)
            if ids(i) == 0
                zeroIdIndex = i;
            end
            if ids(i) == 1
                oneIdIndex = i;
            end
        end

        if zeroIdIndex && oneIdIndex && not(isempty(validMarkers)) && length(coordinates) <= 2
            if zeroIdIndex < oneIdIndex
                arucoMsg.data(1) = coordinates(1,1);
                arucoMsg.data(2) = coordinates(1,2);
                arucoMsg.data(3) = angleArray(1);

                arucoMsg.data(4) = coordinates(2,1);
                arucoMsg.data(5) = coordinates(2,2);
                arucoMsg.data(6) = angleArray(2);
            else
                arucoMsg.data(1) = coordinates(2,1);
                arucoMsg.data(2) = coordinates(2,2);
                arucoMsg.data(3) = angleArray(2);

                arucoMsg.data(4) = coordinates(1,1);
                arucoMsg.data(5) = coordinates(1,2);
                arucoMsg.data(6) = angleArray(1);
            end

            arucoMsg.data(7) = 1;   % Detected ID0
            arucoMsg.data(8) = 1;   % Detected ID1

        elseif zeroIdIndex && not(isempty(validMarkers)) && length(coordinates) <= 2
                arucoMsg.data(1) = coordinates(1,1);
                arucoMsg.data(2) = coordinates(1,2);
                arucoMsg.data(3) = angleArray(1);

                arucoMsg.data(4) = 0;
                arucoMsg.data(5) = 0;
                arucoMsg.data(6) = 0;

                arucoMsg.data(7) = 1;   % Detected ID0
                arucoMsg.data(8) = 0;   % Detected ID1

        elseif oneIdIndex && not(isempty(validMarkers)) && length(coordinates) <= 2
                arucoMsg.data(1) = 0;
                arucoMsg.data(2) = 0;
                arucoMsg.data(3) = 0;

                arucoMsg.data(4) = coordinates(1,1);
                arucoMsg.data(5) = coordinates(1,2);
                arucoMsg.data(6) = angleArray(1);

                arucoMsg.data(7) = 0;   % Detected ID0
                arucoMsg.data(8) = 1;   % Detected ID1
        end

        
        
        send(arucoPub, arucoMsg);
   
        coordinates = [];
    
        
    end
        
    % Liberar la cámara y el nodo
    clear cam;
    clear rosNode;
end