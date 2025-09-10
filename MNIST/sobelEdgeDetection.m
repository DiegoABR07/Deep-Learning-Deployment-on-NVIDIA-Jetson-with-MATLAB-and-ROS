function sobelEdgeDetection(cameraName,resolution) %#codegen
%SOBELEDGEDETECTION() Entry-point function for Sobel edge detection
%   This function is the entry-point function that supports examples in
%   MATLAB Coder Support Package for NVIDIA Jetson and NVIDIA DRIVE 
%   Platforms that use Sobel algorithms for edge detection.

hwobj = jetson;
camObj = camera(hwobj,cameraName,resolution);
dispObj = imageDisplay(hwobj);

% Sobel kernel
kern = [1 2 1; 0 0 0; -1 -2 -1];

% Main loop
for k = 1:1000
    % Capture the image from the camera on hardware.
    img = snapshot(camObj);
    
    % Finding horizontal and vertical gradients.
    h = conv2(img(:,:,2),kern,'same');
    v = conv2(img(:,:,2),kern','same');
    
    % Finding magnitude of the gradients.
    e = sqrt(h.*h + v.*v);
    
    % Threshold the edges
    edgeImg = uint8((e > 100) * 240);
    
    % Display image.
    image(dispObj,edgeImg');
end

end