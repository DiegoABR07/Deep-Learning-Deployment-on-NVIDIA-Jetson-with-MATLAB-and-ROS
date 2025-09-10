function trainingMNIST(outputMat)
% trainingMNIST  Train a basic CNN on MNIST and save .mat with metadata.
%   trainingMNIST(outputMat)
%   - outputMat : path to save the trained network (default 'Models/mnistNet.mat')
%
% Notes:
% - Uses MATLAB R2024a built-in digit dataset helper. If not available,
%   download MNIST or point 'dsTrain/dsVal' to your folders.
% - Saves variables: 'net', 'inputSize', 'classNames'.

    if nargin < 1
        outputMat = fullfile('Models','mnistNet.mat');
    end
    if ~exist(fileparts(outputMat),'dir')
        mkdir(fileparts(outputMat));
    end

    %------------------------------------------------------------------
    % Load MNIST digits (28x28 grayscale). Fallback if example data missing.
    %------------------------------------------------------------------
    try
        digitDatasetPath = fullfile(matlabroot,'toolbox','nnet','nndemos', ...
                                     'nndatasets','DigitDataset');
        imds = imageDatastore(digitDatasetPath, ...
            'IncludeSubfolders',true,'LabelSource','foldernames');
    catch
        error(["MNIST example dataset not found. Provide your own dataset \n" ...
               "as an imageDatastore with labels, or install support files."]);
    end

    % Split train/validation
    [imdsTrain, imdsVal] = splitEachLabel(imds, 0.9, 'randomized');

    inputSize = [28 28 1];
    classNames = categories(imdsTrain.Labels);

    %------------------------------------------------------------------
    % Define a compact CNN (sufficient for demo; tweak as needed)
    %------------------------------------------------------------------
    layers = [
        imageInputLayer(inputSize,'Normalization','none')

        convolution2dLayer(3,16,'Padding','same')
        batchNormalizationLayer
        reluLayer
        maxPooling2dLayer(2,'Stride',2)

        convolution2dLayer(3,32,'Padding','same')
        batchNormalizationLayer
        reluLayer
        maxPooling2dLayer(2,'Stride',2)

        convolution2dLayer(3,64,'Padding','same')
        batchNormalizationLayer
        reluLayer
        dropoutLayer(0.25)

        fullyConnectedLayer(numel(classNames))
        softmaxLayer
        classificationLayer];

    opts = trainingOptions('adam', ...
        'MaxEpochs',8, ...
        'MiniBatchSize',256, ...
        'InitialLearnRate',1e-3, ...
        'Shuffle','every-epoch', ...
        'ValidationData',imdsVal, ...
        'ValidationFrequency',floor(numel(imdsTrain.Files)/256), ...
        'Verbose',false, ...
        'Plots','none');

    net = trainNetwork(imdsTrain, layers, opts);

    % Save network and metadata
    save(outputMat, 'net', 'inputSize', 'classNames', '-v7.3');
    fprintf('Saved trained network to %s\n', outputMat);
end
