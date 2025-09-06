function classIdx = myClassifierGPU(im, modelFile, inputSize)
% myClassifierGPU  Inferencia GPU genérica con tamaño constante.
%   classIdx = myClassifierGPU(im, modelFile, inputSize)
%   - im        : uint8 image [H W C] fijo (declarado en ARGS del script).
%   - modelFile : char/string constante con .mat de la red.
%   - inputSize : [H W C] constante (p.ej., [224 224 3]).
%
%   classIdx : índice 1-based de la clase con mayor score.

    %#codegen
    persistent netPersist

    % Asegurar tipo de modelFile para el loader
    if isstring(modelFile)
        modelFile = char(modelFile);
    end

    %–– Carga única de la red ––%
    if isempty(netPersist)
        % Si tu .mat tiene una variable 'net', puedes usar:
        % netPersist = coder.loadDeepLearningNetwork(modelFile,'net');
        netPersist = coder.loadDeepLearningNetwork(modelFile);
    end

    %–– Ajuste de canales para coincidir con inputSize ––%
    C = inputSize(3);
    if C == 1
        if size(im,3) == 3
            imProc = rgb2gray(im);
        else
            imProc = im;
        end
    elseif C == 3
        if size(im,3) == 1
            imProc = repmat(im, 1, 1, 3);
        else
            imProc = im;
        end
    else
        imProc = im; % casos raros: usuario responsable
    end

    %–– Normalización CONSISTENTE → single [0,1] ––%
    %imProc = im2single(imProc);

    %–– Redimensionar con tamaño CONSTANTE ––%
    outSz     = coder.const(inputSize(1:2));
    imResized = imresize(imProc, outSz);

    %–– Inferencia en GPU ––%
    scores = predict(netPersist, imResized);
    [~, classIdx] = max(scores);
end
