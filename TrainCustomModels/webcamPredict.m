%% webcamPredict.m
% Uso:  ejecutar este script tras haber entrenado y guardado
%       trainedDetectorYOLOv4.mat en la misma carpeta.
%
% Requiere:
%   • Support Package for USB Webcams (función webcam)
%   • Computer Vision Toolbox + modelo YOLO v4

clear, clc, close all

%% 1) Cargar el detector entrenado -----------------------------
load trainedDetectorYOLOv4.mat   % variable "detector"

%% 2) Conectarse a la webcam ------------------------------------
cam = webcam;                    % primer dispositivo disponible
preview(cam);                    % pequeña ventana para enfocar (opcional)

% Reducir resolución si la webcam es muy grande → mejor FPS
cam.Resolution = cam.AvailableResolutions{end};   % último = menor

%% 3) Preparar figura para visualizar ---------------------------
hFig = figure('Name','YOLO v4 webcam','NumberTitle','off');
movegui(hFig,'center');

fps = []; tic                    % contador para mostrar FPS

fprintf("Pulsa Q en la ventana para salir.\n");

%% 4) Bucle principal -------------------------------------------
while ishandle(hFig)
    % a) Capturar cuadro
    I = snapshot(cam);

    % b) Detección
    [b,s,l] = detect(detector,I,Threshold=0.70);

    % c) Dibujar resultados
    Iout = insertObjectAnnotation(I,'rectangle',b, ...
            strcat(string(l),": ",compose("%.2f",s)), ...
            'LineWidth',2,'FontSize',10);

    % d) Mostrar
    imshow(Iout,'Parent',gca), title(sprintf('FPS: %.1f',fps));
    drawnow

    % e) Calcular FPS cada iteración
    fps = 0.9*fps + 0.1*(1/toc); tic

    % f) Salir si el usuario pulsa Q
    if ~ishandle(hFig) || any(get(hFig,'CurrentCharacter')=='q')
        break
    end
end

%% 5) Liberar recursos -------------------------------------------
clear cam
close(hFig)
fprintf("Sesión webcam finalizada.\n");
