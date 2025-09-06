function [bboxes, scores, labels] = myDetector(im, detectorFile, threshold)
% myDetector  Detecta objetos en una imagen usando un detector YOLO guardado.
%
%   [bboxes, scores, labels] = myDetector(im, detectorFile, threshold)
%
%   im           : Imagen RGB o escala de grises (uint8 o double).
%   detectorFile : Archivo .mat con el objeto yolov4ObjectDetector o yoloxObjectDetector.
%   threshold    : Escalar (double) entre 0 y 1 que fija el umbral mínimo de detección.
%
% SALIDAS:
%   bboxes : Mx4 array de bounding boxes [x y ancho alto].
%   scores : Mx1 vector de puntajes.
%   labels : Mx1 cell array de strings (etiquetas).
%
% DESCRIPCIÓN:
%   • Si 'detectorFile' cambia, recarga el detector desde MAT.  
%   • Convierte la imagen a 3 canales si viene en 1 canal.  
%   • Llama a detect(detPersist, imRGB, "Threshold", threshold).
%
% Ejemplo de uso:
%   [bbs,scrs,lbls] = myDetector(im, "yoloV4_coco.mat", 0.7);
% -------------------------------------------------------------------------

    persistent detPersist namePersist

    % Convertir detectorFile a char (por si viene como string)
    if isstring(detectorFile)
        detectorFile = char(detectorFile);
    end

    % Validar que recibimos el umbral
    if nargin < 3
        error("myDetector requiere 3 argumentos: im, detectorFile, threshold.");
    end

    % Si nunca se cargó o cambió el detector, recargar
    if isempty(detPersist) || isempty(namePersist) || ~strcmp(namePersist, detectorFile)
        tmp = load(detectorFile);
        fn  = fieldnames(tmp);
        detPersist  = tmp.(fn{1});  % El detector
        namePersist = detectorFile;
    end

    % Convertir a RGB si la imagen viene en escala de grises
    if size(im,3) == 1
        imRGB = cat(3, im, im, im);
    else
        imRGB = im;
    end

    % Llamar a detect con el umbral especificado
    [bboxes, scores, labels] = detect(detPersist, imRGB, ...
                                      "Threshold", threshold, ...
                                      "MiniBatchSize", 1);
end
