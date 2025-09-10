function mnistDetection(cameraName, resolution) %#codegen
% Clasificación MNIST en vivo en Jetson con overlay del dígito
% Requiere: Deep Learning Toolbox + GPU Coder + Support Pkg Jetson

% 1) Cargar la red una sola vez (embebida en el ejecutable)
persistent net;
if isempty(net)
    % El string debe ser constante en compilación
    % net = coder.loadDeepLearningNetwork('Models/mnistNet.mat');
    net = coder.loadDeepLearningNetwork('Models/mnistMLP.mat');
end

% 2) I/O de la Jetson para código generado
hw   = jetson;
cam  = camera(hw, cameraName, resolution);
disp = imageDisplay(hw);

% 3) Bucle principal
for k = 1:1000
    frameRGB = snapshot(cam);     % uint8 HxWx3
    % --- Preprocesado a 28x28x1 (MNIST) ---
    gray   = rgb2gray(frameRGB);                           % HxW uint8
    small  = imresize(gray, [28 28], 'bilinear');          % 28x28
    % in     = im2single(small);                             % single [0,1]
    in     = reshape(small, [28 28 1]);                    % 28x28x1

    % --- Inferencia ---
    scores = predict(net, in);                             % 1x10
    [~, idx] = max(scores);                                % 1..10
    digitChar = char('0' + (idx-1));                       % '0'..'9'

    % --- Overlay del resultado (insertText soporta codegen;
    %     Font/FontSize deben ser constantes de compilación) ---
    labelStr = ['Digit: ' digitChar];
    frameOut = insertText(frameRGB, [20 20], labelStr, ...
                   'FontSize', coder.const(32), ...
                   'BoxOpacity', coder.const(0.6));        % devuelve RGB

    % --- Mostrar en la pantalla de la Jetson (SDL + X11) ---
    image(disp, permute(frameOut, [2 1 3]));
end
end
