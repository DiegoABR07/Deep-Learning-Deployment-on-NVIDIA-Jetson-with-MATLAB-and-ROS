close all; clc; clear;
%% Conexión
ipaddress = "192.168.0.149";
username  = "ucspjason";
password  = "JasonNano10";
hwobj = jetson(ipaddress, username, password);

%% Chequeo entorno GPU
gpuEnv = coder.gpuEnvConfig('jetson');
gpuEnv.ExecTimeout = 1200;
gpuEnv.BasicCodegen = true; gpuEnv.BasicCodeexec = true;
gpuEnv.DeepLibTarget = 'cudnn';
gpuEnv.DeepCodegen = true;  gpuEnv.DeepCodeexec = true;
gpuEnv.HardwareObject = hwobj;
results = coder.checkGpuInstall(gpuEnv); disp(results);

%% Cámara
% system(hwobj,'v4l2-ctl -d /dev/video0 --list-formats-ext')
camlist = getCameraList(hwobj);
camName = char(camlist{1,"Camera Name"});
camResolution = [640 360];

%% Config de código (EXE para Jetson)
cfg = coder.gpuConfig('exe');
cfg.Hardware = coder.hardware('NVIDIA Jetson');
cfg.Hardware.BuildDir = '~/remoteBuildDir';
cfg.GenerateReport = true;
cfg.GenerateExampleMain = 'GenerateCodeAndCompile';

% Entradas constantes del entry-point
inputArgs = {coder.Constant(camName), coder.Constant(camResolution)};

% Compilar y desplegar
codegen('-config', cfg, 'sobelEdgeDetection', '-args', inputArgs, '-report');

%% --- Mostrar en el escritorio de la Jetson ---

% 1) Ajusta DISPLAY: algunas L4T usan '0.0', otras '1.0'
for dispVal = ["0.0","1.0"]
    try
        setDisplayEnvironment(hwobj, char(dispVal));      % aplica a runApplication
    catch, end
    % 2) Autoriza al usuario ante el X server (evita "No protocol specified")
    %    Requiere x11-xserver-utils (xhost)
    try
        % xhost necesita DISPLAY; prefija DISPLAY=:0 o :1 según dispVal
        displayShort = extractBefore(dispVal,'.');        % "0" o "1"
        cmd = sprintf('bash -lc "DISPLAY=:%s xhost +SI:localuser:%s"', displayShort, username);
        system(hwobj, cmd);
    catch, end
end

% 3) Lanza la app; la ventana aparece en la pantalla de la Jetson
% pid = runApplication(hwobj,'sobelEdgeDetection');

% Para detener la app:
% killApplication(hwobj, 'sobelEdgeDetection');
