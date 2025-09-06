function detector = trainBoxesYOLOv4(dataDir, checkpointsFolder, modelName)
%TRAINBOXESYOLOV4  Entrena y evalúa un detector YOLO v4 (MATLAB R2023a +)
%
%  ─────────────────────────────────────────────────────────────────────────
%  Estructura esperada de carpetas
%      dataDir/train  → train_cocoTable.mat  + imágenes
%      dataDir/valid  → valid_cocoTable.mat  + imágenes
%      dataDir/test   → test_cocoTable.mat   + imágenes
%
%  Requisitos
%      • Deep Learning Toolbox
%      • Computer Vision Toolbox
%      • Add-on “Computer Vision Toolbox Model for YOLO v4”
%
%  Pasos:
%      1) Carga de tablas y limpieza de categorías vacías
%      2) Recorte (“clip”) o descarte (“drop”) de cajas fuera del encuadre
%      3) Creación de datastores y función de formateo
%      4) Estimación de anchors y definición del detector base
%      5) Entrenamiento, evaluación y curvas P-R
%  ─────────────────────────────────────────────────────────────────────────
%
%  Autor: Universidad Católica San Pablo – julio 2025
%  ------------------------------------------------------------------------

%% 0) Verificar existencia de directorio de checkpoints
if ~exist(checkpointsFolder,"dir")
    mkdir(checkpointsFolder);
end

%% 1) Cargar tablas
tblTrain = load(fullfile(dataDir,"train","train_cocoTable.mat")).tbl;
tblVal   = load(fullfile(dataDir,"valid","valid_cocoTable.mat")).tbl;
tblTest  = load(fullfile(dataDir,"test" ,"test_cocoTable.mat" )).tbl;

%%% VISUAL DEBUG >>>  copia “cruda” para dibujar más tarde
tblTrainRaw = tblTrain;  tblValRaw = tblVal;  tblTestRaw = tblTest;
%%% <<<

%% 1-bis) eliminar categorías vacías
tblTrain.objectClass = cellfun(@removecats,tblTrain.objectClass,'uni',false);
tblVal .objectClass  = cellfun(@removecats,tblVal .objectClass,'uni',false);
tblTest.objectClass  = cellfun(@removecats,tblTest.objectClass,'uni',false);

%% 2) CLIP / DROP de cajas fuera de la imagen
fprintf("\n🔧  Recortando / filtrando bounding-boxes inválidas…\n");
clipMode      = "clip";                       % "clip" o "drop"
datasetNames  = ["TRAIN","VAL","TEST"];
tbls          = {tblTrain,tblVal,tblTest};

for t = 1:numel(tbls)
    tbl     = tbls{t};   removed = 0;
    for k = 1:height(tbl)
        info = imfinfo(tbl.imageFilename{k});                    % imagen real
        w = info.Width;  h = info.Height;

        boxes  = tbl.objectBoundingBoxes{k};
        labels = tbl.objectClass{k};

        % recortar al rango [1,w]×[1,h]
        x1 = max(1 , boxes(:,1));
        y1 = max(1 , boxes(:,2));
        x2 = min(w , boxes(:,1)+boxes(:,3));
        y2 = min(h , boxes(:,2)+boxes(:,4));
        newW = x2 - x1;   newH = y2 - y1;
        keep = newW > 1 & newH > 1; % evita cajas 1 px

        if clipMode == "clip"
            boxes  = [x1(keep) y1(keep) newW(keep) newH(keep)];
            labels = labels(keep);
        else                       % modo "drop"
            boxes  = boxes(keep,:); labels = labels(keep);
        end
        removed = removed + sum(~keep);
        tbl.objectBoundingBoxes{k} = boxes;
        tbl.objectClass{k}         = labels;
    end
    opts   = ["eliminadas", "recortadas"];
    idx    = double(clipMode == "clip") + 1;      % clip → 2, drop → 1
    action = opts(idx);
    fprintf("   • %s: %d cajas inválidas %s\n", ...
            datasetNames(t), removed, action);
    tbls{t} = tbl;
end
[tblTrain,tblVal,tblTest] = deal(tbls{:});

%% 2-bis) verificación numérica rápida
rng default
idx = randperm(height(tblTrain), min(150,height(tblTrain)));
bad = 0;
for k = idx
    B = tblTrain.objectBoundingBoxes{k};
    if any(B(:,1)<1 | B(:,2)<1 | (B(:,1)+B(:,3))>480 | (B(:,2)+B(:,4))>480)
        bad = bad+1;
    end
end
fprintf("🛈  Post-clipping: %d / %d imágenes aún con cajas fuera de 480×480\n", ...
        bad, numel(idx));

%% VISUAL DEBUG – cajas antes/después del clipping
% visualizeExample(tblTrainRaw,tblTrain,"TRAIN");
% visualizeExample(tblValRaw ,tblVal ,"VAL");
% visualizeExample(tblTestRaw,tblTest,"TEST");

%% VISUAL RESIZE DEBUG – alineación 480 → 416 px
% showResizeExample(tblTrainRaw,tblTrain,"TRAIN");
% showResizeExample(tblValRaw ,tblVal ,"VAL" );
% showResizeExample(tblTestRaw,tblTest,"TEST");

%% 3) Crear datastores
imdsTrain = imageDatastore(tblTrain.imageFilename);
bldsTrain = boxLabelDatastore(tblTrain(:,{'objectBoundingBoxes','objectClass'}));
imdsVal   = imageDatastore(tblVal.imageFilename);
bldsVal   = boxLabelDatastore(tblVal(:,{'objectBoundingBoxes','objectClass'}));
imdsTest  = imageDatastore(tblTest.imageFilename);
bldsTest  = boxLabelDatastore(tblTest(:,{'objectBoundingBoxes','objectClass'}));

dsTrain = transform(combine(imdsTrain,bldsTrain),@formatForYOLO);
dsVal   = transform(combine(imdsVal  ,bldsVal  ),@formatForYOLO);
dsTest  = transform(combine(imdsTest ,bldsTest ),@formatForYOLO);

%% 4) Estimar anchors
inputSize = [416 416 3];
scale      = inputSize(1)/480;
bboxResized = cellfun(@(b)bboxresize(b,scale),tblTrain.objectBoundingBoxes,'uni',false);

if modelName == "tiny-yolov4-coco"
    % ❶ ask for SIX anchors, not nine
    numAnchors = 6;
    bldsAnch   = boxLabelDatastore(table(bboxResized,'VariableNames',{'BBox'}));
    [anchors,meanIoU] = estimateAnchorBoxes(bldsAnch,numAnchors);
    
    % ❷ sort by area and split into TWO groups (3 + 3)
    area       = anchors(:,1).*anchors(:,2);
    [~,idx]    = sort(area,"descend");
    anchors    = anchors(idx,:);                 % largest first
    anchorBoxes = {anchors(1:3,:); anchors(4:6,:)};  % 2-by-1 cell array

else
    bldsAnch   = boxLabelDatastore(table(bboxResized,'VariableNames',{'BBox'}));
    [anchors,meanIoU] = estimateAnchorBoxes(bldsAnch,9);
    [~,idx] = sort(prod(anchors,2),"descend"); anchors = anchors(idx,:);
    anchorBoxes = {anchors(1:3,:); anchors(4:6,:); anchors(7:9,:)};
end

fprintf("⚓  Anchors OK (mean IoU %.2f)\n",meanIoU);

%% 5) Definir detector base
presentCats = unique(string(vertcat(tblTrain.objectClass{:})));
classNames  = cellstr(sort(presentCats));
detector = yolov4ObjectDetector(modelName, ...
            classNames, anchorBoxes, InputSize=inputSize);

%% 6) Opciones de entrenamiento
imagesPerEpoch = height(tblTrain);
mb = 32;                                     % MiniBatchSize mayor
valFreq = floor(0.25*imagesPerEpoch/mb);

% opts = trainingOptions("adam", ... 
%     InitialLearnRate   = 1e-5, ...
%     MiniBatchSize      = mb, ...
%     MaxEpochs          = 32, ...
%     ValidationData     = dsVal, ...
%     ValidationFrequency= valFreq, ...
%     ValidationPatience = 3, ...
%     OutputNetwork      = "best-validation", ...
%     ResetInputNormalization=false, ...
%     CheckpointPath     = checkpointsFolder, ...
%     VerboseFrequency   = 50, ...
%     Plots              = "training-progress");

opts = trainingOptions("adam", ...
    MiniBatchSize           = 64, ...
    InitialLearnRate        = 1e-4, ...
    MaxEpochs               = 120, ...
    L2Regularization        = 5e-4, ...
    GradientDecayFactor     = 0.9, ...
    SquaredGradientDecayFactor = 0.999, ...
    GradientThresholdMethod = "l2norm", ...
    GradientThreshold       = 5, ...
    DispatchInBackground    = true, ...
    ExecutionEnvironment    = "gpu", ...
    ValidationData          = dsVal, ...
    ValidationFrequency= valFreq, ...
    OutputNetwork           = "best-validation", ...
    CheckpointPath          = checkpointsFolder, ...
    Shuffle                 = "every-epoch", ...
    VerboseFrequency        = 50, ...
    Plots                   = "training-progress");

%% 7) Entrenar
fprintf("\n🚀  Comienza entrenamiento YOLO v4…\n");
[detector,info] = trainYOLOv4ObjectDetector(dsTrain,detector,opts);
save(fullfile(dataDir,"trainedDetectorYOLOv4.mat"),"detector");

%% 8) Evaluar (compatible R2024a)
dets     = detect(detector, dsTest, Threshold=0.01);    % Obtener detecciones
[ap, recall, precision] = evaluateDetectionPrecision(...
                              dets, dsTest, 0.5);      % Calcular mAP@0.5, recall y precision
fprintf("✅  mAP@0.5 IoU = %.3f\n", ap);                  % Mostrar el AP

%% 9) Curvas Precisión–Recall
figure; hold on; grid on
for i = 1:numel(classNames)
    plot(recall{i}, precision{i}, ...
         'LineWidth',1.5, 'DisplayName', classNames{i});
end
xlabel('Recall');
ylabel('Precision');
title(sprintf("YOLO v4 – mAP %.3f", ap));
legend('show');

% ─────────────────── Auxiliares ───────────────────
    function dataOut = formatForYOLO(dataIn)
        I = dataIn{1};
        if numel(dataIn)==3, B=dataIn{2}; L=dataIn{3};
        else, T=dataIn{2}; if iscell(T),T=T{1}; end
             B=T.objectBoundingBoxes{1}; L=T.objectClass{1};
        end
        L = categorical(string(L),classNames);
        dataOut = {I,B,L};
    end

    function visualizeExample(tblB,tblA,tag)
        idx = find(~cellfun(@isempty,tblB.objectBoundingBoxes),1);
        if isempty(idx), return, end
        I = imread(tblB.imageFilename{idx});
        B0=tblB.objectBoundingBoxes{idx}; B1=tblA.objectBoundingBoxes{idx};
        figure('Name',sprintf('%s CLIP',tag),'NumberTitle','off');
        tiledlayout(1,2,"Padding","compact","TileSpacing","compact")
        nexttile,imshow(I),title([tag ' ANTES']),hold on
        for b=1:size(B0,1),rectangle('Position',B0(b,:),'EdgeColor','r');end,hold off
        nexttile,imshow(I),title([tag ' DESPUÉS']),hold on
        for b=1:size(B1,1),rectangle('Position',B1(b,:),'EdgeColor','g');end,hold off
    end

    function showResizeExample(tblB,tblA,tag)
        idx=find(~cellfun(@isempty,tblB.objectBoundingBoxes),1); if isempty(idx),return,end
        I=imread(tblB.imageFilename{idx}); B1=tblA.objectBoundingBoxes{idx};
        scale=416/size(I,1); I416=imresize(I,scale);
        B416=bboxresize(B1,scale);
        fprintf("   [%s] caja(1) 480→%s  | 416→%s\n",tag,mat2str(round(B1(1,:))),mat2str(round(B416(1,:))));
        figure('Name',sprintf('%s RESIZE',tag),'NumberTitle','off');
        tiledlayout(1,2,"Padding","compact","TileSpacing","compact")
        nexttile,imshow(I),title([tag ' 480 px']),hold on
        rectangle('Position',B1(1,:),'EdgeColor','g','LineWidth',2),hold off
        nexttile,imshow(I416),title([tag ' 416 px']),hold on
        rectangle('Position',B416(1,:),'EdgeColor','c','LineWidth',2),hold off
    end
end
