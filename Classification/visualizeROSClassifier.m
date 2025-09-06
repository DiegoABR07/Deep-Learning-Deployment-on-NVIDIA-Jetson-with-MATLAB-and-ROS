%% visualizeROSClassifier.m
% Visualiza el flujo de cámara y la predicción de clase enviada desde Jetson.
% 
% – Suscribe al tópico '/network_view' para recibir imágenes (sensor_msgs/Image).
% – Suscribe al tópico '/network_out' para recibir índices de clase (std_msgs/UInt32 o similar).
% – Obtiene los nombres de clase leyendo el archivo MAT que contiene la red.
% – Muestra en tiempo real la imagen y el nombre de la clase.
%
% NOTAS:
% 1) Asegúrate de que el archivo modelFile (.mat) esté disponible en la carpeta
%    de MATLAB tanto local como en el Jetson (según dónde ejecutes el script).
% 2) El índice de clase publicado en '/network_out' se asume 1-based.
%
% -------------------------------------------------------------------------

clc;
clear;

%% ------------ Configuración ROS y modelo ------------
% Dirección IP de la Jetson (ajusta si cambia).
jetsonIP   = "192.168.0.149";

% Nombre del archivo MAT que contiene la red entrenada.
% Ejemplos: "alexnet.mat", "mnistCNN.mat", "resnet18.mat", etc.
modelFile  = "Models/alexnet.mat";

% ------------------------------------------------------

% 1) Conectar a ROS en Jetson
rosshutdown;                      % Cierra cualquier conexión previa
rosinit(jetsonIP);                % Se conecta al maestro ROS del Jetson

% 2) Cargar dinámicamente la red y extraer nombres de clase
data      = load(modelFile);      % Carga el .mat que contiene la red
fn        = fieldnames(data);     
net       = data.(fn{1});         % Obtiene el objeto SeriesNetwork / DAGNetwork

try
    classNamesAll = cellstr(net.Layers(end).Classes);
catch
    classNamesAll = {};           % Si no hay property 'Classes'
end

% 3) Crear suscriptores a los tópicos ROS
imgSub   = rossubscriber("/network_view", "sensor_msgs/Image");
% Asumimos que el mensaje de clase viene en un std_msgs/UInt32 o similar
classSub = rossubscriber("/network_out", "std_msgs/UInt32");

%% ------------ Bucle de visualización ------------
figure("Name","ROS Image + Clasificación","NumberTitle","off");
while true
    %--- 1. Recibir imagen desde '/network_view' -------------
    if ~isempty(imgSub.LatestMessage)
        imMsg = imgSub.LatestMessage;
        disp("encoding: " + string(imMsg.Encoding))  % <- útil para verificar
        img   = readImage(imMsg); % ahora coincidirá con 'rgb8'
    else
        % Si no hay mensaje nuevo, dibujamos un placeholder negro
        % con formato [H W C] según la red (p.ej. [227 227 3] o [28 28 1]).
        inputSize = net.Layers(1).InputSize;  % [H W C]
        if inputSize(3) == 3
            img = zeros(inputSize(1), inputSize(2), 3, 'uint8');
        else
            img = zeros(inputSize(1), inputSize(2), 1, 'uint8');
        end
    end

    %--- 2. Recibir índice de clase desde '/network_out' ------
    if ~isempty(classSub.LatestMessage)
        classMsg = classSub.LatestMessage;
        classNum = double(classMsg.Data);  % Aseguramos que sea double
        % Validar rango del índice
        if 1 <= classNum && classNum <= numel(classNamesAll)
            className = classNamesAll{classNum};
        else
            className = "Índice fuera de rango";
        end
    else
        className = "Esperando dato...";
    end

    %--- 3. Mostrar imagen y nombre de clase -----------------
    imshow(img);
    title("Clase detectada: " + className, "FontSize", 14);
    drawnow;

    %--- 4. Pequeña pausa (opcional) para no saturar el bucle ----
    %pause(0.02);
end
