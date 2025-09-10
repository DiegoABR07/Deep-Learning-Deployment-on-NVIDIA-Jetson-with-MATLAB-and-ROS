function trainingMNIST_MLP(outputMat)
% Entrena un MLP sencillo para MNIST (sin escalar a [0,1]) y guarda .mat.
% Muestra la curva de entrenamiento y una matriz de confusión
% (sobre el conjunto de validación) al finalizar.
%
% Guarda: net, inputSize, classNames

if nargin < 1
    outputMat = fullfile('Models','mnistMLP.mat');
end
if ~exist(fileparts(outputMat),'dir'); mkdir(fileparts(outputMat)); end

% ------------------------------------------------------------
% Dataset de dígitos (28x28x1, etiquetas 0–9)
% ------------------------------------------------------------
dataFolder = fullfile(toolboxdir('nnet'),'nndemos','nndatasets','DigitDataset');
imds = imageDatastore(dataFolder, ...
    'IncludeSubfolders',true, 'LabelSource','foldernames');   % datos etiquetados

% Split train/val
[imdsTrain, imdsVal] = splitEachLabel(imds, 0.9, 'randomized');

inputSize  = [28 28 1];
classNames = categories(imdsTrain.Labels);

% ------------------------------------------------------------
% MLP puro: (FC-ReLU) x 2 -> FC -> softmax
% ¡OJO! Sin normalización: 'Normalization','none'
% ------------------------------------------------------------
% SIN escala a [0,1] (rescale-zero-one)
layers = [
    imageInputLayer(inputSize,'Normalization','none')
    fullyConnectedLayer(numel(classNames))
    softmaxLayer
    classificationLayer];

% Curva de entrenamiento: 'Plots','training-progress'
mb = 1024;
opts = trainingOptions('adam', ...
    'MaxEpochs',15, ...
    'MiniBatchSize',mb, ...
    'InitialLearnRate',0.0005, ...
    'Shuffle','every-epoch', ...
    'ValidationData',imdsVal, ...
    'ValidationFrequency',max(1, floor(numel(imdsTrain.Files)/mb)), ...
    'Verbose',false, ...
    'Plots','training-progress');   % muestra la UI con pérdidas/accuracies

% Entrenar
net = trainNetwork(imdsTrain, layers, opts);

% Guardar
save(outputMat,'net','inputSize','classNames','-v7.3');
fprintf('Saved %s\n', outputMat);

% ------------------------------------------------------------
% Evaluación: matriz de confusión (validación)
% ------------------------------------------------------------
% Predicción por lotes en validación
[predVal, ~] = classify(net, imdsVal, 'MiniBatchSize', mb);
trueVal = imdsVal.Labels;

% Accuracy simple
accVal = mean(predVal == trueVal);
fprintf('Validation accuracy: %.2f%%\n', 100*accVal);

% Matriz de confusión (normalizada por filas)
figure('Name','MNIST MLP - Confusion Matrix');
cm = confusionchart(trueVal, predVal, 'Normalization','row-normalized');
cm.Title = sprintf('MNIST MLP (val) — Accuracy: %.2f%%', 100*accVal);
cm.ColumnSummary = 'column-normalized';
cm.RowSummary    = 'row-normalized';

end
