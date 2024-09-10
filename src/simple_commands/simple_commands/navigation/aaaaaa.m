% Configurar la cámara (asegúrate de seleccionar el índice correcto)
cam = webcam(2); 

% Mostrar las resoluciones disponibles y seleccionar la resolución deseada
disp(cam.AvailableResolutions);
cam.Resolution = '2592x1944'; % Cambia esto a la resolución deseada

% Especificar la cantidad de fotos a capturar
n = 20; % Cambia este valor para capturar más o menos fotos

% Especificar la carpeta donde se guardarán las fotos
outputFolder = '/home/sanmaster/turtlebot_development/src/simple_commands/simple_commands/navigation/Photos'; % Cambia la ruta según tu sistema
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder); % Crea la carpeta si no existe
end

% Capturar y guardar las fotos
for i = 1:n
    img = snapshot(cam); % Captura la imagen
    filename = fullfile(outputFolder, ['foto_' num2str(i) '.png']); % Nombre del archivo
    imwrite(img, filename); % Guarda la imagen
    fprintf('Foto %d guardada en %s\n', i, filename); % Muestra un mensaje de confirmación
    imshow(img);
    pause(5); % Pausa de 1 segundo entre capturas (puedes ajustar o eliminar)
end

% Liberar la cámara
clear cam;
disp('Captura de fotos completada.');