function saveDetectorToFile(detector, filename)
% saveDetectorToFile  Guarda un detector de objetos YOLO en un archivo MAT.
%
%   saveDetectorToFile(detector, filename)
%
% ENTRADAS:
%   detector : Objeto de tipo yolov4ObjectDetector o yoloxObjectDetector
%              (entrenado previamente con las clases deseadas).
%   filename : Nombre o ruta (char o string) del archivo .mat de salida.
%
% DESCRIPCIÓN:
%   Esta función toma un objeto detector entrenado (empleando
%   yolov4ObjectDetector o yoloxObjectDetector) y lo almacena en un MAT
%   bajo la variable 'detector'. No se guardan archivos separados de
%   anclas ni clases porque el objeto detector ya los incluye.
%
% EJEMPLO:
%   det = yolov4ObjectDetector("csp-darknet53-coco", "cocoClasses", ...
%                              "AnchorBoxes", anchorBoxes);
%   saveDetectorToFile(det, "yoloV4_coco.mat");
%
%   % Para YOLOX:
%   det2 = yoloxObjectDetector("nano-coco");
%   saveDetectorToFile(det2, "yoloX_nano_coco.mat");
%
% NOTA:
%   - Asegúrate de que 'detector' sea un objeto válido (loaded o entrenado).
%   - Para GPU Coder, es conveniente que MAT contenga únicamente 'detector'.
% -------------------------------------------------------------------------

    if nargin < 2
        error("saveDetectorToFile requiere dos argumentos: detector y filename.");
    end

    % Guardar el objeto detector:
    save(filename, 'detector');
end
