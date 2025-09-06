% Script to test AlexNet inference function

clc;
clear;
close all;

% 1. Cargar imagen y redimensionarla a 227×227
im = imread('peppers.jpg');          
imResized = imresize(im, [227, 227]);

% 2. Predecir con AlexNet
classIdx = myAlexNet(imResized);                % Llamada a la función de clasificación
% classIdx = myAlexNetGPU(im);           % GPU Coder compatible function
load classNames
className = classNames{classIdx};
                                  
% 3. Mostrar imagen anotada
figure
imshow(imResized)
title(['Clasificada como: ', className]);
disp(['Image classified as: ', className])
disp(['Class Index: ', num2str(classIdx)])