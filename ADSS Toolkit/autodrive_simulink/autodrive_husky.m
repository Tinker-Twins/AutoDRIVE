function autodrive_husky(block)
% AutoDRIVE Husky Simulink API
% This is a Level-2 MATLAB S-Function wrapper for AutoDRIVE Husky API

% Copyright 2025 AutoDRIVE Ecosystem

%% The setup method is used to set up the basic attributes of the
%% S-function such as ports, parameters, etc. Do not add any other
%% calls to the main body of the function.

setup(block);

%endfunction

%% Function: setup
%% Abstract:
%% Set up the basic characteristics of the S-function block such as:
%% - Input ports
%% - Output ports
%% - Dialog parameters
%% - Options
%%
%% Required: Yes
%% C MEX counterpart: mdlInitializeSizes

function setup(block)

% Allow more than 2D signals
block.AllowSignalsWithMoreThan2D = 1;

% Register number of ports
block.NumInputPorts = 10;
block.NumOutputPorts = 12;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
% Co-Simulation Mode
block.InputPort(1).Dimensions = 1;
block.InputPort(1).DatatypeID = 3; % uint8
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).DirectFeedthrough = true;
% Position X Component
block.InputPort(2).Dimensions = 1;
block.InputPort(2).DatatypeID = 0; % double
block.InputPort(2).Complexity = 'Real';
block.InputPort(2).DirectFeedthrough = true;
% Position Y Component
block.InputPort(3).Dimensions = 1;
block.InputPort(3).DatatypeID = 0; % double
block.InputPort(3).Complexity = 'Real';
block.InputPort(3).DirectFeedthrough = true;
% Position Z Component
block.InputPort(4).Dimensions = 1;
block.InputPort(4).DatatypeID = 0; % double
block.InputPort(4).Complexity = 'Real';
block.InputPort(4).DirectFeedthrough = true;
% Orientation Quaternion X Component
block.InputPort(5).Dimensions = 1;
block.InputPort(5).DatatypeID = 0; % double
block.InputPort(5).Complexity = 'Real';
block.InputPort(5).DirectFeedthrough = true;
% Orientation Quaternion Y Component
block.InputPort(6).Dimensions = 1;
block.InputPort(6).DatatypeID = 0; % double
block.InputPort(6).Complexity = 'Real';
block.InputPort(6).DirectFeedthrough = true;
% Orientation Quaternion Z Component
block.InputPort(7).Dimensions = 1;
block.InputPort(7).DatatypeID = 0; % double
block.InputPort(7).Complexity = 'Real';
block.InputPort(7).DirectFeedthrough = true;
% Orientation Quaternion W Component
block.InputPort(8).Dimensions = 1;
block.InputPort(8).DatatypeID = 0; % double
block.InputPort(8).Complexity = 'Real';
block.InputPort(8).DirectFeedthrough = true;
% Linear Velocity Command
block.InputPort(9).Dimensions = 1;
block.InputPort(9).DatatypeID = 0; % double
block.InputPort(9).Complexity = 'Real';
block.InputPort(9).DirectFeedthrough = true;
% Angular Velocity Command
block.InputPort(10).Dimensions = 1;
block.InputPort(10).DatatypeID = 0; % double
block.InputPort(10).Complexity = 'Real';
block.InputPort(10).DirectFeedthrough = true;

% Override output port properties
% Vehicle ID
block.OutputPort(1).Dimensions = 1;
block.OutputPort(1).DatatypeID = 5; % uint16
block.OutputPort(1).Complexity = 'Real';
% Throttle Feedback
block.OutputPort(2).Dimensions = 1;
block.OutputPort(2).DatatypeID = 0; % double
block.OutputPort(2).Complexity = 'Real';
% Differential Feedback
block.OutputPort(3).Dimensions = 1;
block.OutputPort(3).DatatypeID = 0; % double
block.OutputPort(3).Complexity = 'Real';
% Encoder Ticks
block.OutputPort(4).Dimensions = [2 1];
block.OutputPort(4).DatatypeID = 6; % int32
block.OutputPort(4).Complexity = 'Real';
% Encoder Angles
block.OutputPort(5).Dimensions = [2 1];
block.OutputPort(5).DatatypeID = 0; % double
block.OutputPort(5).Complexity = 'Real';
% Position
block.OutputPort(6).Dimensions = [3 1];
block.OutputPort(6).DatatypeID = 0; % double
block.OutputPort(6).Complexity = 'Real';
% Orientation (Quaternion)
block.OutputPort(7).Dimensions = [4 1];
block.OutputPort(7).DatatypeID = 0; % double
block.OutputPort(7).Complexity = 'Real';
% Orientation (Euler Angles)
block.OutputPort(8).Dimensions = [3 1];
block.OutputPort(8).DatatypeID = 0; % double
block.OutputPort(8).Complexity = 'Real';
% Angular Velocity
block.OutputPort(9).Dimensions = [3 1];
block.OutputPort(9).DatatypeID = 0; % double
block.OutputPort(9).Complexity = 'Real';
% Linear Acceleration
block.OutputPort(10).Dimensions = [3 1];
block.OutputPort(10).DatatypeID = 0; % double
block.OutputPort(10).Complexity = 'Real';
% LIDAR Pointcloud
block.OutputPort(11).DimensionsMode = 'Variable';
block.OutputPort(11).Dimensions = [57600,3]; % max. size
block.OutputPort(11).DatatypeID = 1; % single
block.OutputPort(11).Complexity = 'Real';
% Camera Frame
block.OutputPort(12).Dimensions = [720,1280,3];
block.OutputPort(12).DatatypeID = 3; % uint8
block.OutputPort(12).Complexity = 'Real';

% Register parameters
block.NumDialogPrms = 1;

% Register sample times
% [0 offset] : Continuous sample time
% [positive_num offset] : Discrete sample time
% [-1, 0] : Inherited sample time
% [-2, 0] : Variable sample time
block.SampleTimes = [1 0];

% Specify the block simStateCompliance. The allowed values are:
% 'UnknownSimState', < The default setting; warn and assume DefaultSimState
% 'DefaultSimState', < Same sim state as a built-in block
% 'HasNoSimState', < No sim state
% 'CustomSimState', < Has GetSimState and SetSimState methods
% 'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.

% block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
% block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs); % Required
% block.RegBlockMethod('Update', @Update);
% block.RegBlockMethod('Derivatives', @Derivatives);
block.RegBlockMethod('Terminate', @Terminate); % Required

%end setup

%% PostPropagationSetup:
%% Functionality: Setup work areas and state variables. Can
%% also register run-time methods here
%% Required: No
%% C MEX counterpart: mdlSetWorkWidths

% function DoPostPropSetup(block)

%end DoPostPropSetup(block)

%% InitializeConditions:
%% Functionality: Called at the start of simulation and if it is 
%% present in an enabled subsystem configured to reset 
%% states, it will be called when the enabled subsystem
%% restarts execution to reset the states.
%% Required: No
%% C MEX counterpart: mdlInitializeConditions

function InitializeConditions(block)
global autodrive
autodrive = server_husky(block.DialogPrm(1).Data);

%end InitializeConditions

%% Start:
%% Functionality: Called once at start of model execution. If you
%% have states that should be initialized once, this 
%% is the place to do it.
%% Required: No
%% C MEX counterpart: mdlStart

% function Start(block)

%end Start

%% Outputs:
%% Functionality: Called to generate block outputs in
%% simulation step
%% Required: Yes
%% C MEX counterpart: mdlOutputs

function Outputs(block)
global autodrive
% Parse inputs
autodrive.husky_1.cosim_mode = block.InputPort(1).Data;
autodrive.husky_1.posX_command = block.InputPort(2).Data;
autodrive.husky_1.posY_command = block.InputPort(3).Data;
autodrive.husky_1.posZ_command = block.InputPort(4).Data;
autodrive.husky_1.rotX_command = block.InputPort(5).Data;
autodrive.husky_1.rotY_command = block.InputPort(6).Data;
autodrive.husky_1.rotZ_command = block.InputPort(7).Data;
autodrive.husky_1.rotW_command = block.InputPort(8).Data;
autodrive.husky_1.lin_vel_command = block.InputPort(9).Data;
autodrive.husky_1.ang_vel_command = block.InputPort(10).Data;
% Configure outputs
block.OutputPort(1).Data = autodrive.husky_1.id;
block.OutputPort(2).Data = autodrive.husky_1.throttle;
block.OutputPort(3).Data = autodrive.husky_1.differential;
block.OutputPort(4).Data = autodrive.husky_1.encoder_ticks;
block.OutputPort(5).Data = autodrive.husky_1.encoder_angles;
block.OutputPort(6).Data = autodrive.husky_1.position;
block.OutputPort(7).Data = autodrive.husky_1.orientation_quaternion;
block.OutputPort(8).Data = autodrive.husky_1.orientation_euler_angles;
block.OutputPort(9).Data = autodrive.husky_1.angular_velocity;
block.OutputPort(10).Data = autodrive.husky_1.linear_acceleration;
block.OutputPort(11).CurrentDimensions = [size(autodrive.husky_1.lidar_pointcloud,1),3];
block.OutputPort(11).Data = autodrive.husky_1.lidar_pointcloud;
block.OutputPort(12).Data = autodrive.husky_1.camera_image;

%end Outputs

%% Update:
%% Functionality: Called to update discrete states
%% during simulation step
%% Required: No
%% C MEX counterpart: mdlUpdate

% function Update(block)

%end Update

%% Derivatives:
%% Functionality: Called to update derivatives of
%% continuous states during simulation step
%% Required: No
%% C MEX counterpart: mdlDerivatives

% function Derivatives(block)

%end Derivatives

%% Terminate:
%% Functionality: Called at the end of simulation for cleanup
%% Required: Yes
%% C MEX counterpart: mdlTerminate

function Terminate(block)
global autodrive
autodrive.stop;
autodrive.delete;

%end Terminate