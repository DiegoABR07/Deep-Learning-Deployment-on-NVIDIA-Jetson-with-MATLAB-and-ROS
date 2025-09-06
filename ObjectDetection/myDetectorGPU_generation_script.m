%% myDetectorGPU_generation_script.m
% Genera la biblioteca GPU Coder (para Jetson) de myDetectorGPU.
% Requisitos: coder.gpuConfig("dll","ecoder",true) y firma con 3 args.
clc; clear;

% 1) Archivo del detector (usa char para evitar cambios de tipo)
detectorFile = 'Models/yoloV4_coco.mat';   % <-- char, no string

% 2) Cargar detector provisionalmente para conocer el tamaño de entrada
temp = load(detectorFile);
fn   = fieldnames(temp);
det  = temp.(fn{1});
inputSize = det.Network.Layers(1).InputSize;  % p.ej. [416 416 3]

% 3) Configuración GPU Coder (dll + Embedded Coder)
cfg = coder.gpuConfig("dll","ecoder",true);
cfg.GenerateReport = true;
cfg.Hardware = coder.hardware("NVIDIA Jetson");
cfg.HardwareImplementation.TargetHWDeviceType = ...
    "ARM Compatible->ARM 64-bit (LP64)";

% 4) Prototipos de entrada:
%    - im: uint8, tamaño FIJO de [H W 3] (recomendado para Jetson y ROS)
%    - detectorFile: ***constante de compilación*** (char)
%    - threshold: double escalar
imType     = coder.typeof(uint8(0), inputSize, [false false false]); % fijo
threshType = coder.typeof(0);  % double escalar

ARGS = { imType, coder.Constant(detectorFile), threshType };

% 5) Generar
codegen -config cfg -o myDetectorGPU myDetectorGPU -args ARGS -report

% Salidas esperadas:
%   libmyDetectorGPU.so, myDetectorGPU.h/cpp, binarios de pesos, informe HTML.
