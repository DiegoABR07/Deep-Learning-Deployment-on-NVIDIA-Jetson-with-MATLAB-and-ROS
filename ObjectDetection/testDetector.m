%% testDetector.m
% Prueba de detección de objetos en MATLAB para CPU y GPU.
% - Lee una imagen, ejecuta myDetector (CPU) o myDetectorGPU (GPU).
% - Normaliza las etiquetas (categorical/cell/string/índices) a cellstr.
% - Dibuja bounding boxes con "etiqueta: score".
%
% Modo de uso:
%   - Cambia runMode a "CPU" o "GPU".
%   - Ajusta rutas de detectorFile e imgPath.
%
% (c) 2025

clc; clear; close all;

% 1) Limpiar persistentes para forzar recarga
clear myDetector
clear myDetectorGPU

% 2) Parámetros de prueba
runMode      = "GPU";  % "CPU" o "GPU"
detectorFile = "Models/yoloV4_coco.mat";  % Usa slash para portabilidad
imgPath      = "Samples/car.jpg";
umbral       = 0.75;

% 3) Cargar imagen
im = imread(imgPath);

% 4) Cargar detector sólo para obtener ClassNames (mapeo de índices)
tmp = load(detectorFile);
fn  = fieldnames(tmp);
det = tmp.(fn{1});
classNamesAll = det.ClassNames; % cell array de nombres de clase

% 5) Ejecutar el detector según el modo
switch runMode
    case "CPU"
        [bboxes, scores, labels] = myDetector(im, detectorFile, umbral);
    case "GPU"
        [bboxes, scores, labels] = myDetectorGPU(im, detectorFile, umbral); % aquí labels = índices
    otherwise
        error("runMode debe ser 'CPU' o 'GPU'.");
end

% 6) Normalizar etiquetas a cellstr
lblsCell = normalizeLabels(labels, classNamesAll);

% 7) Construir anotaciones y dibujar
if ~isempty(bboxes)
    % Asegurar consistencia de longitudes
    n = min([size(bboxes,1), numel(scores), numel(lblsCell)]);
    bboxes = bboxes(1:n, :);
    scores = scores(1:n);
    lblsCell = lblsCell(1:n);

    annotations = strings(n,1);
    for i = 1:n
        annotations(i) = sprintf("%s: %.2f", lblsCell{i}, scores(i));
    end

    imAnnotated = insertObjectAnnotation( ...
        im, "rectangle", bboxes, annotations, ...
        "Color","yellow","TextBoxOpacity",0.9,"FontSize",12);
else
    imAnnotated = im;
end

% 8) Mostrar
figure("Name","Resultado de Detección","NumberTitle","off");
imshow(imAnnotated);
title(sprintf("Detecciones (%s)", runMode), "FontSize", 16);

%----------------------------------------------------------------------
% Función local: normaliza etiquetas a cellstr
function lblsCell = normalizeLabels(labels, classNamesAll)
    % Devuelve un cellstr de igual longitud que labels (o 0 si vacío)

    if isempty(labels)
        lblsCell = cell(0,1);
        return;
    end

    if iscategorical(labels)
        lblsCell = cellstr(labels);
        return;
    end

    if iscell(labels)
        % Asumimos cellstr
        lblsCell = labels(:);
        return;
    end

    if isstring(labels)
        lblsCell = cellstr(labels);
        return;
    end

    if isnumeric(labels)
        % Mapeo índices (1-based) -> nombres
        idx = double(labels(:));
        lblsCell = cell(numel(idx),1);
        C = classNamesAll(:); % cellstr
        for k = 1:numel(idx)
            if ~isnan(idx(k)) && idx(k) >= 1 && idx(k) <= numel(C)
                lblsCell{k} = C{idx(k)};
            else
                lblsCell{k} = "unknown";
            end
        end
        return;
    end

    % Fallback
    lblsCell = repmat({"unknown"}, numel(labels), 1);
end
