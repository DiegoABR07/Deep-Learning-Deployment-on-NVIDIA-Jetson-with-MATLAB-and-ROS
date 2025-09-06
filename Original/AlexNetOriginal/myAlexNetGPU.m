function classIdx = myAlexNetGPU(im)
% MYALEXNETGPU Accepts a 227x227x3 image to the deep neural network AlexNet
% and returns the class index of the maximum confidence classification.

% Since the function "alexnet" is not supported for generation we load it
% from a MAT-file using coder.loadDeepLearningNetwork
persistent net
if isempty(net)
    net = coder.loadDeepLearningNetwork('alexnet.mat');
end

% Predict with AlexNet
output = predict(net, im);

% Determine the class index with the highest probability
[~,classIdx] = max(output);

end