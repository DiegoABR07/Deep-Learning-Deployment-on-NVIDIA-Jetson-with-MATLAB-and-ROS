% Saves AlexNet network to MAT-file for use with GPU Coder
%
% NOTE: This requires the Deep Learning Toolbox Model for AlexNet Network
% support package.

function saveAlexNetToFile()
    
    % Save network to MAT-file
    net = alexnet;
    save alexnet net
    
    % Save class names to MAT-file
    classNames = cellstr(net.Layers(end).Classes);
    save classNames classNames
    
end