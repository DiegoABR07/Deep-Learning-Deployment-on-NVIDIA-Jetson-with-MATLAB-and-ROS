%% runYOLOv4.m
clear, clc;
%% 1. Definir directorios y modelo
localDir = pwd;
modelName = "tiny-yolov4-coco"; % csp-darknet53-coco
datasetFolder  = 'datasets';
projectName = 'drowsiness';
dataDir = fullfile(localDir, datasetFolder, projectName);
checkpointsFolder = fullfile(localDir, 'checkpoints', projectName);

%% 2. Seleccionar características de la red a entrenar


%% 3. Entrenar la red
detector = trainBoxesYOLOv4(dataDir, checkpointsFolder, modelName);

%% 4. Hacer predicciones sobre la red entrenada
I = imread(fullfile(dataDir,'test', ...
   'imagen_96_jpg.rf.79adb8064c554b8bc6493c2e52091d16.jpg'));
[b,s,l] = detect(detector,I,Threshold=0.20);

figure, imshow(I), hold on
for k = 1:size(b,1)
    rectangle('Position',b(k,:),'EdgeColor','g','LineWidth',1)
    text(b(k,1),b(k,2)-10,sprintf('%s: %.2f',string(l(k)),s(k)), ...
        'Color','yellow','FontSize',8,'FontWeight','bold')
end
title('Detecciones YOLO v4')
