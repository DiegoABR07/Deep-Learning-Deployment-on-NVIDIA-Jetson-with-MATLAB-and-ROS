function [bboxes, scores, labelsIdx] = myDetectorGPU(im, detectorFile, threshold)
% myDetectorGPU  Detección en GPU con YOLO y GPU Coder (firma compatible).
%
%   [bboxes, scores, labelsIdx] = myDetectorGPU(im, detectorFile, threshold)
%
% Entradas:
%   im           : Imagen RGB o escala de grises (uint8), tamaño libre (sin resize manual).
%   detectorFile : Ruta del .mat con el detector (string o char).
%                  En codegen pásalo como coder.Constant(char).
%   threshold    : double [0,1].
%
% Salidas:
%   bboxes    : Mx4 double [x y w h] en coords de la imagen original.
%   scores    : Mx1 double.
%   labelsIdx : Mx1 double (índice 1-based de la clase).
%
% Notas:
%   - En MATLAB (simulación): recarga el detector si cambia 'detectorFile'.
%   - En codegen/Jetson: 'detectorFile' es constante y se carga 1 vez.
%
% (c) 2025

%#codegen

persistent detPersist namePersist

% Copia local en char sin cambiar el tipo de la entrada (codegen-safe)
if isstring(detectorFile)
    dfc = char(detectorFile);
else
    dfc = detectorFile; % ya es char
end

if nargin < 3
    error("myDetectorGPU requiere 3 argumentos: im, detectorFile, threshold.");
end

% === Carga del detector ===
if coder.target('MATLAB')
    % En MATLAB: permitir cambiar de archivo entre llamadas.
    if isempty(detPersist) || isempty(namePersist) || ~strcmp(namePersist, dfc)
        tmp = load(dfc);
        fn  = fieldnames(tmp);
        detPersist  = tmp.(fn{1});
        namePersist = dfc;
    end
else
    % En código generado: cargar una sola vez (dfc es coder.Constant).
    if isempty(detPersist)
        detPersist = coder.loadDeepLearningNetwork(dfc);
        % No usamos namePersist en codegen para evitar restricciones con persistent.
    end
end

% Garantizar 3 canales (el detector espera RGB)
if size(im,3) == 1
    imRGB = cat(3, im, im, im);
else
    imRGB = im;
end

% Detectar DIRECTO sobre la imagen original (sin resize manual)
[bboxes, scores, labelsCat] = detect(detPersist, imRGB, ...
                                     "Threshold", threshold, ...
                                     "MiniBatchSize", 1);

% Normalizar tipos de salida
bboxes = double(bboxes);
scores = double(scores);

% === categorical -> índices 1-based (codegen-safe) ===
% Codegen NO soporta char/string sobre categorical; SÍ soporta cast a numérico.
% Ref: Use 'double(labelsCat)' para obtener códigos de categoría (1..K).
if isempty(labelsCat)
    labelsIdx = zeros(0,1);
else
    labelsIdx = double(labelsCat);   % <-- la clave: soportado en codegen
    labelsIdx = double(labelsIdx(:)); % asegurar columna double
end

end
