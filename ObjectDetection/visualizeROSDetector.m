%% visualizeROSDetector.m
% Visualiza en tiempo real las detecciones YOLO publicadas desde Jetson.
%
% Suscripciones:
%   - /network_view              (sensor_msgs/Image)
%   - /network_detections        (std_msgs/Float32MultiArray)
% Formato detecciones (fila por objeto):
%   [x, y, w, h, score, labelIdx]
%
% Requisitos:
%   - ROS Toolbox en MATLAB conectado al master de Jetson
%   - MAT del detector para extraer det.ClassNames
%
% (c) 2025

clc; clear;

%% --- Configuración ---
jetsonIP       = "192.168.0.149";          % IP del master ROS en Jetson
modelFile      = "Models/yoloV4_coco.mat";        % MAT con el detector
imgTopic       = "/network_view";
detTopic       = "/network_detections";
loopHz         = 20;                       % tasa de refresco del bucle

%% --- ROS: iniciar sesión con Jetson ---
try
    rosshutdown;
catch
end
rosinit(char(jetsonIP));  % conexión al master

% Asegurar cierre limpio al terminar
cleanupObj = onCleanup(@() rosshutdown);

%% --- Cargar nombres de clase del modelo ---
S  = load(modelFile);
fn = fieldnames(S);
det = S.(fn{1});
classNames = det.ClassNames;    % cell array con nombres de clase

%% --- Suscriptores ---
imgSub    = rossubscriber(imgTopic,    "sensor_msgs/Image");
detectSub = rossubscriber(detTopic,    "std_msgs/Float32MultiArray");

%% --- Figura y primera imagen (bloqueante sólo 1 vez) ---
% Intentar recibir un frame inicial (hasta 2 s). Si no llega, usar placeholder.
try
    firstMsg = receive(imgSub, 2.0);     % bloquea hasta 2 s
    try
        frame0 = rosReadImage(firstMsg); % preferido
    catch
        frame0 = readImage(firstMsg);    % fallback (deprecated)
    end
catch
    % Placeholder con el tamaño de entrada de la red (sólo para no fallar)
    inSz = det.Network.Layers(1).InputSize; % [H W C]
    if inSz(3) == 3
        frame0 = zeros(inSz(1), inSz(2), 3, 'uint8');
    else
        frame0 = zeros(inSz(1), inSz(2), 1, 'uint8');
    end
end

if size(frame0,3) == 1
    frame0 = repmat(frame0, 1,1,3);
end

hFig = figure("Name","Detección ROS (YOLO)","NumberTitle","off");
hAx  = axes('Parent', hFig);
hIm  = imshow(frame0, 'Parent', hAx);
title(hAx, "Detecciones en tiempo real");

%% --- Bucle principal (control de ritmo) ---
r = rateControl(loopHz);

while ishandle(hIm)
    % 1) Obtener imagen actual (no bloqueante)
    imMsg = imgSub.LatestMessage;   % devuelve [] si aún no hay nuevo
    if ~isempty(imMsg)
        try
            frame = rosReadImage(imMsg); % preferido
        catch
            frame = readImage(imMsg);    % fallback
        end
        if size(frame,3) == 1
            frame = repmat(frame, 1,1,3);
        end
    else
        frame = get(hIm, 'CData');  % reusar último frame
    end

    % 2) Obtener detecciones actuales (no bloqueante)
    detMsg = detectSub.LatestMessage;
    if ~isempty(detMsg)
        dataArr = double(detMsg.Data);   % vector [x y w h score lbl]*N
        nVals = numel(dataArr);
        if mod(nVals, 6) == 0
            N = nVals / 6;
            M = reshape(dataArr, 6, N).';    % Nx6
            bboxes    = M(:,1:4);
            scores    = M(:,5);
            labelsIdx = uint32(M(:,6));
        else
            % Formato inesperado
            bboxes = zeros(0,4);
            scores = zeros(0,1);
            labelsIdx = zeros(0,1,'uint32');
            warning("Formato de detecciones incorrecto (numel!=6k).");
        end
    else
        bboxes = zeros(0,4);
        scores = zeros(0,1);
        labelsIdx = zeros(0,1,'uint32');
    end

    % 3) Componer anotaciones
    imOut = frame;
    if ~isempty(bboxes)
        n = size(bboxes,1);
        txt = strings(n,1);
        for i = 1:n
            li = labelsIdx(i);
            if li >= 1 && li <= numel(classNames)
                cls = classNames{li};
            else
                cls = "Desconocido";
            end
            txt(i) = sprintf("%s: %.2f", cls, scores(i));
        end

        % insertObjectAnnotation devuelve una imagen anotada
        imOut = insertObjectAnnotation(frame, "rectangle", bboxes, txt, ...
                  "Color","green", "TextBoxOpacity",0.8, "FontSize",12);
    end

    % 4) Actualizar imagen en pantalla (sin recrear la figura)
    set(hIm, 'CData', imOut);
    drawnow limitrate;

    % 5) Ritmo del bucle (~loopHz Hz)
    waitfor(r);
end
