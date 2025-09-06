function classIdx = myAlexNet(im)
% MYALEXNET Accepts a 227x227x3 image to the deep neural network AlexNet
% and returns the class index of the maximum confidence classification.

% If AlexNet has not been loaded once before, load it from the MAT file.
persistent net
if isempty(net)
    load('alexnet.mat','net');
end

% Predict with AlexNet
output = predict(net, im);

% Determine the class index with the highest probability
[~,classIdx] = max(output);

end

