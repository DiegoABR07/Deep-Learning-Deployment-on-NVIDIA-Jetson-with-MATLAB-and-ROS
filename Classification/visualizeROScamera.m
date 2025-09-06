%% visualizeROSClassifier_DEMO
% Visualiza un flujo de cámara remoto para probar la webcam.
%
% (c) 2025 – Proyecto genérico de clasificación
% -------------------------------------------------------------------------

clc; clear;

%--- Parámetros -----------------------------------------------------------
jetsonIP  = "192.168.0.149";         % Dirección IP

%--- Conexión ROS ---------------------------------------------------------
rosshutdown;                         % Cierra conexiones previas
rosinit(jetsonIP);                   % Conecta con Jetson

imgSub = rossubscriber("/network_view","sensor_msgs/Image");

%--- Bucle principal ------------------------------------------------------
while true
    % Espera hasta recibir un mensaje (timeout opcional)
    imMsg = receive(imgSub, 10);    % 10 s de espera máx.
    
    % Convierte sensor_msgs/Image --> matriz RGB de MATLAB
    img = readImage(imMsg);
    
    imshow(img);
    title('Webcam en vivo');
    drawnow;
end
