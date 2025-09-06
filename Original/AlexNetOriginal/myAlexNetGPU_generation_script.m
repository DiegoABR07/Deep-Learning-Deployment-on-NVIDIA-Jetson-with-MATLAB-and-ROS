% MYALEXNETGPU_GENERATION_SCRIPT   Generate library myAlexNetGPU from myAlexnetGPU.
% 
% Script generated from project 'myAlexNetGPU.prj'.
% 
% See also CODER, CODER.CONFIG, CODER.TYPEOF, CODEGEN.

%% Create configuration object of class 'coder.EmbeddedCodeConfig'.
cfg = coder.gpuConfig('lib','ecoder',true);
cfg.GenerateReport = true;
cfg.ReportPotentialDifferences = false;
cfg.InitFltsAndDblsToZero = false;
cfg.BuildConfiguration = 'Faster Builds';
cfg.Hardware = coder.hardware('NVIDIA Jetson');
cfg.HardwareImplementation.TargetHWDeviceType = 'ARM Compatible->ARM 64-bit (LP64)';

%% Define argument types for entry-point 'myAlexnetGPU'.
ARGS = cell(1,1);
ARGS{1} = cell(1,1);
ARGS{1}{1} = coder.typeof(uint8(0),[227 227  3]);

%% Invoke MATLAB Coder.
codegen -config cfg -o myAlexNetGPU myAlexnetGPU -args ARGS{1} -nargout 1

