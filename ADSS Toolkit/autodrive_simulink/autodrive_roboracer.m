function autodrive_roboracer(block)
% AutoDRIVE RoboRacer Simulink API
% This is a Level-2 MATLAB S-Function wrapper for AutoDRIVE RoboRacer API

% Copyright 2024 AutoDRIVE Ecosystem

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
block.NumInputPorts = 3;
block.NumOutputPorts = 18;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
% Throttle Command
block.InputPort(1).Dimensions = 1;
block.InputPort(1).DatatypeID = 0; % double
block.InputPort(1).Complexity = 'Real';
block.InputPort(1).DirectFeedthrough = true;
% Steering Command
block.InputPort(2).Dimensions = 1;
block.InputPort(2).DatatypeID = 0; % double
block.InputPort(2).Complexity = 'Real';
block.InputPort(2).DirectFeedthrough = true;
% Reset Command
block.InputPort(3).Dimensions = 1;
block.InputPort(3).DatatypeID = 5; % uint16
block.InputPort(3).Complexity = 'Real';
block.InputPort(3).DirectFeedthrough = true;

% Override output port properties
% Vehicle ID
block.OutputPort(1).Dimensions = 1;
block.OutputPort(1).DatatypeID = 5; % uint16
block.OutputPort(1).Complexity = 'Real';
% Throttle Feedback
block.OutputPort(2).Dimensions = 1;
block.OutputPort(2).DatatypeID = 0; % double
block.OutputPort(2).Complexity = 'Real';
% Steering Feedback
block.OutputPort(3).Dimensions = 1;
block.OutputPort(3).DatatypeID = 0; % double
block.OutputPort(3).Complexity = 'Real';
% Speed Feedback
block.OutputPort(4).Dimensions = 1;
block.OutputPort(4).DatatypeID = 0; % double
block.OutputPort(4).Complexity = 'Real';
% Encoder Ticks
block.OutputPort(5).Dimensions = [2 1];
block.OutputPort(5).DatatypeID = 6; % int32
block.OutputPort(5).Complexity = 'Real';
% Encoder Angles
block.OutputPort(6).Dimensions = [2 1];
block.OutputPort(6).DatatypeID = 0; % double
block.OutputPort(6).Complexity = 'Real';
% Position
block.OutputPort(7).Dimensions = [3 1];
block.OutputPort(7).DatatypeID = 0; % double
block.OutputPort(7).Complexity = 'Real';
% Orientation (Quaternion)
block.OutputPort(8).Dimensions = [4 1];
block.OutputPort(8).DatatypeID = 0; % double
block.OutputPort(8).Complexity = 'Real';
% Orientation (Euler Angles)
block.OutputPort(9).Dimensions = [3 1];
block.OutputPort(9).DatatypeID = 0; % double
block.OutputPort(9).Complexity = 'Real';
% Angular Velocity
block.OutputPort(10).Dimensions = [3 1];
block.OutputPort(10).DatatypeID = 0; % double
block.OutputPort(10).Complexity = 'Real';
% Linear Acceleration
block.OutputPort(11).Dimensions = [3 1];
block.OutputPort(11).DatatypeID = 0; % double
block.OutputPort(11).Complexity = 'Real';
% LIDAR Laser Scan
block.OutputPort(12).Dimensions = [1081,1];
block.OutputPort(12).DatatypeID = 0; % double
block.OutputPort(12).Complexity = 'Real';
% Camera
block.OutputPort(13).Dimensions = [108,192,3];
block.OutputPort(13).DatatypeID = 3; % uint8
block.OutputPort(13).Complexity = 'Real';
% Lap Count
block.OutputPort(14).Dimensions = 1;
block.OutputPort(14).DatatypeID = 5; % uint16
block.OutputPort(14).Complexity = 'Real';
% Lap Time
block.OutputPort(15).Dimensions = 1;
block.OutputPort(15).DatatypeID = 0; % double
block.OutputPort(15).Complexity = 'Real';
% Last Lap Time
block.OutputPort(16).Dimensions = 1;
block.OutputPort(16).DatatypeID = 0; % double
block.OutputPort(16).Complexity = 'Real';
% Best Lap Time
block.OutputPort(17).Dimensions = 1;
block.OutputPort(17).DatatypeID = 0; % double
block.OutputPort(17).Complexity = 'Real';
% Collision Count
block.OutputPort(18).Dimensions = 1;
block.OutputPort(18).DatatypeID = 5; % uint16
block.OutputPort(18).Complexity = 'Real';

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
autodrive = server_roboracer(block.DialogPrm(1).Data);

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
autodrive.roboracer_1.throttle_command = block.InputPort(1).Data;
autodrive.roboracer_1.steering_command = block.InputPort(2).Data;
if(block.InputPort(3).Data == 0)
    autodrive.roboracer_1.reset_command = "False";
else
    autodrive.roboracer_1.reset_command = "True";
end
% Configure outputs
block.OutputPort(1).Data = autodrive.roboracer_1.id;
block.OutputPort(2).Data = autodrive.roboracer_1.throttle;
block.OutputPort(3).Data = autodrive.roboracer_1.steering;
block.OutputPort(4).Data = autodrive.roboracer_1.speed;
block.OutputPort(5).Data = autodrive.roboracer_1.encoder_ticks;
block.OutputPort(6).Data = autodrive.roboracer_1.encoder_angles;
block.OutputPort(7).Data = autodrive.roboracer_1.position;
block.OutputPort(8).Data = autodrive.roboracer_1.orientation_quaternion;
block.OutputPort(9).Data = autodrive.roboracer_1.orientation_euler_angles;
block.OutputPort(10).Data = autodrive.roboracer_1.angular_velocity;
block.OutputPort(11).Data = autodrive.roboracer_1.linear_acceleration;
block.OutputPort(12).Data = autodrive.roboracer_1.lidar_range_array;
block.OutputPort(13).Data = autodrive.roboracer_1.front_camera_image;
block.OutputPort(14).Data = autodrive.roboracer_1.lap_count;
block.OutputPort(15).Data = autodrive.roboracer_1.lap_time;
block.OutputPort(16).Data = autodrive.roboracer_1.last_lap_time;
block.OutputPort(17).Data = autodrive.roboracer_1.best_lap_time;
block.OutputPort(18).Data = autodrive.roboracer_1.collision_count;

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