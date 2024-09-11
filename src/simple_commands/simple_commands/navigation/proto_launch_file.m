

function run_parallel_nodes(cameraParams)
    % twoPlusTwo = @(x) x+2;

    % Crear un pool de procesadores en segundo plano con 2 workers
    parpool('local', 2);
    
    % Ejecutar el nodo de formación de robots en paralelo
    future1 = parfeval(@aruco_publisher,0, cameraParams);
    
    % Ejecutar el nodo de publicación de ArUco en paralelo
    future2 = parfeval(@twoPlusTwo,0);
    
    % Esperar a que ambas tareas terminen
    wait(future1);
    wait(future2);
    disp(future1.OutputArguments);
    
    disp('Termine')
end


run_parallel_nodes(cameraParams);

