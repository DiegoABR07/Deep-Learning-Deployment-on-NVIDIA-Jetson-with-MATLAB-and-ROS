%% testClassifier  –  Prueba de inferencia en CPU con cualquier modelo .mat
% SINTAXIS:
%   testClassifier
clc; clear; close all;
clear myClassifier
clear myClassifierGPU

% --- Ajustes ------------------------------------------------------------
modelFile = "Models/vgg16.mat";    % Cambia a tu modelo .mat
imgPath   = "Samples/peppers.jpg";    % Ruta de la imagen de prueba

% --- Leer imagen --------------------------------------------------------
im = imread(imgPath);

% --- Clasificar usando la función genérica ------------------------------
[classIdx, classNames] = myClassifier(im, modelFile);

% Nombre de clase robusto (si no hay classNames en el .mat)
if isempty(classNames)
    className = "Class " + string(classIdx);
else
    % classNames es cellstr
    className = string(classNames{classIdx});
end

% --- Mostrar resultado --------------------------------------------------
figure; imshow(im);
title("Clasificada como: " + className);
disp("Image classified as: " + className)
disp("Class Index: " + num2str(classIdx))
