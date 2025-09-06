%% myClassifierGPU_generation_script
% Genera una biblioteca ARM/CUDA para la función myClassifierGPU.
clc; clear; close all;
clear myClassifier
clear myClassifierGPU
% --- Ajusta según tu modelo --------------------------------------------
modelFile = 'Models/alexnet.mat';   % usa char para codegen

cfg = coder.gpuConfig('dll', 'ecoder', true);
cfg.GenerateReport = true;
cfg.Hardware = coder.hardware('NVIDIA Jetson');
cfg.HardwareImplementation.TargetHWDeviceType = ...
    'ARM Compatible->ARM 64-bit (LP64)';

% --- Obtener tamaño de entrada -----------------------------------------
tmpData = load(modelFile);
fnames  = fieldnames(tmpData);
tmpNet  = tmpData.(fnames{1});

if isfield(tmpData, 'inputSize')
    inputSz = tmpData.inputSize;       % recomendable guardarlo en tu .mat
elseif isprop(tmpNet, 'Layers') && ~isempty(tmpNet.Layers)
    inputSz = tmpNet.Layers(1).InputSize;  % Series/DAG
else
    error(['No pude determinar el tamaño de entrada. ' ...
           'Agrega "inputSize = [H W C]" al .mat o usa una red con Layers(1).InputSize.']);
end

% --- Definir prototipos de entrada -------------------------------------
% imType con tamaño FIJO (el de la red)
imType = coder.typeof(uint8(0), inputSz, false);  % 'false' => no variable-size

ARGS = { ...
    imType, ...
    coder.Constant(modelFile), ...
    coder.Constant(inputSz) ...
};

% --- Generar la librería -----------------------------------------------
codegen -config cfg ...
        -o myClassifierGPU ...
        myClassifierGPU -args ARGS -nargout 1
