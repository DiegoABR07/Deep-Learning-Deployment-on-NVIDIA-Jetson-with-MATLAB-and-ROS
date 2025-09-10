% deployJetson_MNIST.m
close all; clc; clear;

%% Conexión
ipaddress = "192.168.0.149";
username  = "ucspjason";
password  = "JasonNano10";
hwobj = jetson(ipaddress, username, password);
disp(hwobj);

%% (Opcional) Chequeo GPU/Deep Learning
gpuEnv = coder.gpuEnvConfig('jetson');
gpuEnv.DeepLibTarget  = 'cudnn';    % o 'tensorrt' si prefieres
gpuEnv.HardwareObject = hwobj;
results = coder.checkGpuInstall(gpuEnv); 
disp(results);

%% Cámara (usa resoluciones soportadas por /dev/video0)
camlist = getCameraList(hwobj);
camName = char(camlist{1,"Camera Name"});
camResolution = [640 360];

%% Config de código (EXE para Jetson)
cfg = coder.gpuConfig('exe');
cfg.GenerateReport = true;
cfg.Hardware = coder.hardware('NVIDIA Jetson');
cfg.Hardware.BuildDir = '~/remoteBuildDir';
cfg.GenerateExampleMain = 'GenerateCodeAndCompile';

% Acelera inferencia con TensorRT (o cuDNN):
cfg.DeepLearningConfig = coder.DeepLearningConfig('cudnn');  % Jetson
% alternativa: cfg.DeepLearningConfig = coder.DeepLearningConfig('tensorrt');

% Entradas constantes del entry-point
inputArgs = {coder.Constant(camName), ...
    coder.Constant(camResolution)};

% 'mnistNet.mat' debe estar en el path (se usa en codegen)
codegen('-config', cfg, 'mnistDetection', ...
    '-args', inputArgs, '-report');

%% Autorizar X11 y lanzar!!!
% Prueba ambos displays y da permiso al usuario ante el servidor X.
for dispVal = ["0.0","1.0"]
    try 
        setDisplayEnvironment(hwobj, char(dispVal)); 
    end
    try
        dshort = extractBefore(dispVal,'.');
        system(hwobj, ...
            sprintf('bash -lc "DISPLAY=:%s xhost +SI:localuser:%s"', ...
            dshort, username));
    end
end

% pid = runApplication(hwobj,'mnistLive');   % ventana aparece en la Jetson
% killApplication(hwobj,'mnistLive');      % para detener
