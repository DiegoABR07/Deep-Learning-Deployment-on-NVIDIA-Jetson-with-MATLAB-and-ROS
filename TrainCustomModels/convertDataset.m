%% convertDataset.m
% Limpieza
clear; clc;

% 1. Directorio base de tu proyecto
localDir     = pwd;
datasetFolder= fullfile('datasets','drowsiness');
dataDir      = fullfile(localDir, datasetFolder);

% 2. Define los splits según tus carpetas
splits = {'train','valid','test'};

for i = 1:numel(splits)
    sp       = splits{i};
    splitDir = fullfile(dataDir, sp);
    
    % 3. Rutas: JSON de anotaciones e imágenes (mismo folder)
    jsonFile = fullfile(splitDir, '_annotations.coco.json');
    imgFolder= splitDir;
    
    % 4. Comprobaciones básicas
    assert(exist(jsonFile,'file')==2,  'No existe %s', jsonFile);
    assert(exist(imgFolder,'dir')==7,  'No existe %s', imgFolder);
    
    % 5. Generar la tabla
    tbl = coco2table(jsonFile, imgFolder);
    
    % 6. Guardar la tabla en .mat dentro de su carpeta
    outName = sprintf('%s_cocoTable.mat', sp);        % 'train_cocoTable.mat' o 'val_cocoTable.mat'
    outFile = fullfile(splitDir, outName);
    save(outFile, 'tbl');
    
    % 7. Mensaje de confirmación
    fprintf('✔ Guardada tabla en "%s" con %d filas.\n', outFile, height(tbl));
end

