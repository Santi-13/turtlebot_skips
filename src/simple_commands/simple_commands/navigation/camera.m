% Configurar la cámara
cam = webcam(2); % Cambiar el índice según la cámara disponible

% Configurar una figura para visualizar el video en tiempo real
figure;
set(gcf, 'Name', 'Detección de ArUco con Cámara Web');

% Parámetros de configuración
minMarkerSize = 20; % Tamaño mínimo en píxeles para considerar un marcador válido
thresholdConfidence = 0.8; % Umbral de confianza para detección
gridSize = 50; % Tamaño de cada celda de la cuadrícula en milímetros

% Inicializar un arreglo para guardar las coordenadas de los centroides
centroidCoordinates = {}; % Usamos una celda para guardar coordenadas de múltiples ArUcos

% Cargar los parámetros de calibración de la cámara
load('calibrationSession.mat'); % Ajusta la ruta al archivo .mat con los parámetros de calibración
intrinsics = cameraParams.Intrinsics;

% Verificar si hay problemas con los intrínsecos
if isempty(intrinsics)
    error('Los parámetros intrínsecos no se cargaron correctamente. Verifica la calibración.');
end

% Factor de conversión píxeles a milímetros (ajustar según calibración)
pixelsToMM = 0.3; % Valor de ejemplo, ajustar basado en el patrón de referencia

while ishandle(gcf)
    % Capturar una imagen desde la cámara web
    img = snapshot(cam);
    
    % Corregir la distorsión de la imagen usando los intrínsecos
    imgUndistorted = undistortImage(img, intrinsics);

    % Mostrar la imagen corregida
    imshow(imgUndistorted);
    hold on;
    
    % Obtener el tamaño de la imagen para la cuadrícula
    [imgHeight, imgWidth, ~] = size(imgUndistorted);
    
    % Dibujar la cuadrícula con coordenadas positivas desde la esquina superior izquierda
    for x = 0:gridSize:imgWidth
        % Dibujar las líneas de la cuadrícula verticales
        line([x, x], [1, imgHeight], 'Color', [0, 1, 0, 0.5], 'LineWidth', 0.5);
        % Mostrar las coordenadas en milímetros ajustadas
        text(x, 20, sprintf('%.1f mm', x * pixelsToMM), 'Color', 'black', 'FontSize', 10, 'HorizontalAlignment', 'center');
    end
    
    for y = 0:gridSize:imgHeight
        % Dibujar las líneas de la cuadrícula horizontales
        line([1, imgWidth], [y, y], 'Color', [0, 1, 0, 0.5], 'LineWidth', 0.5);
        % Mostrar las coordenadas en milímetros ajustadas
        text(20, y, sprintf('%.1f mm', y * pixelsToMM), 'Color', 'black', 'FontSize', 10, 'HorizontalAlignment', 'right');
    end
    
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
    
    % Dibujar marcadores detectados y proyectar sus coordenadas
    for i = validMarkers
        corners = locs(:,:,i);
        plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], 'r-', 'LineWidth', 2);
        
        centroidX = mean(corners(:,1));
        centroidY = mean(corners(:,2));
        
        plot(centroidX, centroidY, 'go', 'MarkerSize', 10, 'LineWidth', 2);
        text(centroidX, centroidY, sprintf('ID: %d', ids(i)), 'Color', 'yellow', 'FontSize', 12, 'HorizontalAlignment', 'center');
        
        % Convertir los puntos del marcador a coordenadas del mundo en milímetros
        worldCoords = [centroidX * pixelsToMM, centroidY * pixelsToMM];
        fprintf('ID: %d, World X: %.2f mm, World Y: %.2f mm\n', ids(i), worldCoords(1), worldCoords(2));
        text(centroidX, centroidY + 20, sprintf('(%.1f mm, %.1f mm)', worldCoords(1), worldCoords(2)), ...
            'Color', 'cyan', 'FontSize', 10, 'HorizontalAlignment', 'center');
        
        % Identificar las esquinas inferiores del marcador y calcular ángulo
        bottomLineX = [corners(3,1), corners(4,1)];
        bottomLineY = [corners(3,2), corners(4,2)];
        plot(bottomLineX, bottomLineY, 'b-', 'LineWidth', 2);
        
        % Calcular el ángulo de la línea inferior con respecto a la línea horizontal
        deltaX = bottomLineX(2) - bottomLineX(1);
        deltaY = bottomLineY(2) - bottomLineY(1);
        angleRad = atan2(deltaY, deltaX); % Ángulo en radianes
        angleDeg = rad2deg(angleRad); % Convertir a grados
        
        % Mostrar el ángulo en la imagen
        midPointX = mean(bottomLineX);
        midPointY = mean(bottomLineY);
        text(midPointX, midPointY, sprintf('%.2f°', angleDeg), 'Color', 'cyan', 'FontSize', 12, 'HorizontalAlignment', 'center');
        
        % Imprimir el ángulo en la consola
        fprintf('ID: %d, Ángulo con la horizontal: %.2f°\n', ids(i), angleDeg);
    end
    hold off;
    pause(0.1);
    drawnow;
end

% Liberar la cámara
clear cam;