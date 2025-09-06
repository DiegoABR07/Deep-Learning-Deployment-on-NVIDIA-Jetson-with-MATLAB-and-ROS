function [classIdx, classNames] = myClassifier(im, modelFile)
% myClassifier  Clasifica una imagen usando la red almacenada en 'modelFile'.
%   [classIdx, classNames] = myClassifier(im, modelFile)
%   - im        : RGB o gris (uint8/uint16/single/double) de cualquier tamaño.
%   - modelFile : .mat con SeriesNetwork/DAGNetwork/dlnetwork u objeto compatible.
%
%   classIdx   : Índice (1-based) de la clase con mayor probabilidad.
%   classNames : Celda de strings con nombres de las clases (si disponible).

    %-------------------- Persistentes -----------------------------------
    persistent netPersist inputSizePersist classNamesPersist namePersist

    % Asegurar 'modelFile' char (evita sorpresas en strcmp y load)
    if isstring(modelFile); modelFile = char(modelFile); end

    needReload = isempty(netPersist) || isempty(namePersist) || ~strcmp(namePersist, modelFile);
    if needReload
        S = load(modelFile);
        f = fieldnames(S);
        netPersist = S.(f{1});  % primer objeto del .mat

        % 1) Determinar input size:
        if isfield(S, 'inputSize')
            inputSizePersist = S.inputSize;  % preferente si lo guardaste junto a la red
        else
            % SeriesNetwork/DAGNetwork suelen tener Layers(1).InputSize
            if isprop(netPersist, 'Layers') && ~isempty(netPersist.Layers) && isprop(netPersist.Layers(1), 'InputSize')
                inputSizePersist = netPersist.Layers(1).InputSize;
            else
                error(['No pude determinar el tamaño de entrada. ' ...
                       'Guarda en el .mat una variable "inputSize = [H W C]" o usa una red con Layers(1).InputSize.']);
            end
        end

        % 2) Determinar class names:
        if isfield(S, 'classNames')
            classNamesPersist = S.classNames(:);
            if ~iscell(classNamesPersist), classNamesPersist = cellstr(string(classNamesPersist)); end
        else
            try
                classNamesPersist = cellstr(netPersist.Layers(end).Classes);
            catch
                classNamesPersist = {}; % puede ser dlnetwork o no-clasificador puro
            end
        end

        namePersist = modelFile;
    end

    %-------------------- Preprocesamiento --------------------------------
    % Forzar 3 o 1 canal según pida la red
    C = inputSizePersist(3);
    if C == 1
        if size(im,3) == 3
            imProc = rgb2gray(im);
        else
            imProc = im;
        end
    elseif C == 3
        if size(im,3) == 1
            imProc = cat(3, im, im, im);
        else
            imProc = im;
        end
    else
        imProc = im; % casos >3 canales: el usuario es responsable
    end

    % Normalización CONSISTENTE → single [0,1]
    %Revisar, no siempre es el caso, en resnet no requiere normalización
    %imProc = im2single(imProc); 

    % Redimensionar al tamaño [H W] requerido
    imResized = imresize(imProc, inputSizePersist(1:2));

    %-------------------- Inferencia --------------------------------------
    % Nota: para dlnetwork puro, normalmente se usa dlarray + forward.
    % Muchos .mat guardan una wrapper función o un objeto compatible con predict.
    scores = predict(netPersist, imResized);
    [~, classIdx] = max(scores);

    %-------------------- Salida ------------------------------------------
    classNames = classNamesPersist;
end
