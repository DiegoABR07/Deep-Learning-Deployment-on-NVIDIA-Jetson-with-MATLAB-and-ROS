% Script to test AlexNet inference deployed as ROS node

clc;
clear;
%% Setup
% Connect to ROS master on the Jetson
rosshutdown;
rosinit('192.168.0.149'); % Jetson's IP address

% Load class name information
load classNames

% Subscribe to the ROS topics
imgSub = rossubscriber('/network_view');
classSub = rossubscriber('/network_out');

%% Continue receiving messages from ROS
while true
 
    % Receive image message
    if ~isempty(imgSub.LatestMessage)
        imMsg = imgSub.LatestMessage;
        img = readImage(imMsg);
    else
        img = zeros(227,227,3);
    end
    
    % Receive class index message
    if ~isempty(classSub.LatestMessage)
        classMsg = classSub.LatestMessage;
        classNum = classMsg.Data;
        className = classNames{classNum};
    else
        className = 'NaN';
    end

    % Display image and detected class name
    imshow(img);
    title(className);
    drawnow;
    
end  