function Dynamics(block)
setup(block);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %{
 This Level-2 S-Function simulates the behavior of a quadcopter based on
 performance parameters and RPM inputs.

    Additional quadcopter modeling and simulation materials available at:
    github.com/dch33/Quad-Sim

Copyright (C) 2014 D. Hartman, K. Landis, S. Moreno, J. Kim, M. Mehrer

License:
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU LESSER GENERAL PUBLIC LICENSE as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU LESSER GENERAL PUBLIC LICENSE
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
%}
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Function: setup ===================================================
% Abstract:
%   Set up the S-function block's basic characteristics such as:
%   - Input ports
%   - Output ports
%   - Dialog parameters
%   - Options
% 
%   Required         : Yes
%   C-Mex counterpart: mdlInitializeSizes
%
function setup(block)

  % Register the number of ports.
  %------
  block.NumInputPorts  = 8;
  %------
  block.NumOutputPorts = 14;
  
  % Set up the port properties to be inherited or dynamic.
  
  for i = 1:8 % These are the motor inputs
  block.InputPort(i).Dimensions        = 1;
  block.InputPort(i).DirectFeedthrough = false;
  block.InputPort(i).SamplingMode      = 'Sample';
  end
  %------
  % This is the disturbance input
%   block.InputPort(5).Dimensions        = 6; % torques x,y,z; forces x,y,z.
%   block.InputPort(5).DirectFeedthrough = false;
%   block.InputPort(5).SamplingMode      = 'Sample';
  %------
  for i = 1:12
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end
  
  block.OutputPort(13).Dimensions      = 3;
  block.OutputPort(13).SamplingMode     = 'Sample';
  
  block.OutputPort(14).Dimensions      = 3;
  block.OutputPort(14).SamplingMode     = 'Sample';
  
%   % Override the input port properties.
%   block.InputPort(1).DatatypeID  = 0;  % double
%   block.InputPort(1).Complexity  = 'Real';
%   
%   % Override the output port properties.
%   block.OutputPort(1).DatatypeID  = 0; % double
%   block.OutputPort(1).Complexity  = 'Real';

  % Register the parameters.
  block.NumDialogPrms = 2;
  
  % Set up the continuous states.
  block.NumContStates = 12;

  % Register the sample times.
  %  [0 offset]            : Continuous sample time
  %  [positive_num offset] : Discrete sample time
  %
  %  [-1, 0]               : Inherited sample time
  %  [-2, 0]               : Variable sample time
  block.SampleTimes = [0 0];
  
  % -----------------------------------------------------------------
  % Options
  % -----------------------------------------------------------------
  % Specify if Accelerator should use TLC or call back to the 
  % MATLAB file
  block.SetAccelRunOnTLC(false);
  
  % Specify the block simStateCompliance. The allowed values are:
  %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
  %    'DefaultSimState', < Same SimState as a built-in block
  %    'HasNoSimState',   < No SimState
  %    'CustomSimState',  < Has GetSimState and SetSimState methods
  %    'DisallowSimState' < Errors out when saving or restoring the SimState
  block.SimStateCompliance = 'DefaultSimState';
  
  % -----------------------------------------------------------------
  % The MATLAB S-function uses an internal registry for all
  % block methods. You should register all relevant methods
  % (optional and required) as illustrated below. You may choose
  % any suitable name for the methods and implement these methods
  % as local functions within the same file.
  % -----------------------------------------------------------------
   
  % -----------------------------------------------------------------
  % Register the methods called during update diagram/compilation.
  % -----------------------------------------------------------------
  
%   % 
%   % CheckParameters:
%   %   Functionality    : Called in order to allow validation of the
%   %                      block dialog parameters. You are 
%   %                      responsible for calling this method
%   %                      explicitly at the start of the setup method.
%   %   C-Mex counterpart: mdlCheckParameters
%   %
   block.RegBlockMethod('CheckParameters', @CheckPrms);

%   %
%   % SetInputPortSamplingMode:
%   %   Functionality    : Check and set input and output port 
%   %                      attributes and specify whether the port is operating 
%   %                      in sample-based or frame-based mode
%   %   C-Mex counterpart: mdlSetInputPortFrameData.
%   %   (The DSP System Toolbox is required to set a port as frame-based)
%   %
%   block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
%   %
%   % SetInputPortDimensions:
%   %   Functionality    : Check and set the input and optionally the output
%   %                      port dimensions.
%   %   C-Mex counterpart: mdlSetInputPortDimensionInfo
%   %
%   block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);

%   %
%   % SetOutputPortDimensions:
%   %   Functionality    : Check and set the output and optionally the input
%   %                      port dimensions.
%   %   C-Mex counterpart: mdlSetOutputPortDimensionInfo
%   %
%   block.RegBlockMethod('SetOutputPortDimensions', @SetOutPortDims);
  
%   %
%   % SetInputPortDatatype:
%   %   Functionality    : Check and set the input and optionally the output
%   %                      port datatypes.
%   %   C-Mex counterpart: mdlSetInputPortDataType
%   %
%   block.RegBlockMethod('SetInputPortDataType', @SetInpPortDataType);
  
%   %
%   % SetOutputPortDatatype:
%   %   Functionality    : Check and set the output and optionally the input
%   %                      port datatypes.
%   %   C-Mex counterpart: mdlSetOutputPortDataType
%   %
%   block.RegBlockMethod('SetOutputPortDataType', @SetOutPortDataType);
  
%   %
%   % SetInputPortComplexSignal:
%   %   Functionality    : Check and set the input and optionally the output
%   %                      port complexity attributes.
%   %   C-Mex counterpart: mdlSetInputPortComplexSignal
%   %
%   block.RegBlockMethod('SetInputPortComplexSignal', @SetInpPortComplexSig);
%   
%   %
%   % SetOutputPortComplexSignal:
%   %   Functionality    : Check and set the output and optionally the input
%   %                      port complexity attributes.
%   %   C-Mex counterpart: mdlSetOutputPortComplexSignal
%   %
%   block.RegBlockMethod('SetOutputPortComplexSignal', @SetOutPortComplexSig);
  
%   %
%   % PostPropagationSetup:
%   %   Functionality    : Set up the work areas and the state variables. You can
%   %                      also register run-time methods here.
%   %   C-Mex counterpart: mdlSetWorkWidths
%   %
%   block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
% 
%   % -----------------------------------------------------------------
%   % Register methods called at run-time
%   % -----------------------------------------------------------------
%   
%   % 
%   % ProcessParameters:
%   %   Functionality    : Call to allow an update of run-time parameters.
%   %   C-Mex counterpart: mdlProcessParameters
%   %  
%   block.RegBlockMethod('ProcessParameters', @ProcessPrms);

  % 
  % InitializeConditions:
  %   Functionality    : Call to initialize the state and the work
  %                      area values.
  %   C-Mex counterpart: mdlInitializeConditions
  % 
  block.RegBlockMethod('InitializeConditions', @InitializeConditions);
  
%   % 
%   % Start:
%   %   Functionality    : Call to initialize the state and the work
%   %                      area values.
%   %   C-Mex counterpart: mdlStart
%   %
%   block.RegBlockMethod('Start', @Start);

  % 
  % Outputs:
  %   Functionality    : Call to generate the block outputs during a
  %                      simulation step.
  %   C-Mex counterpart: mdlOutputs
  %
  block.RegBlockMethod('Outputs', @Outputs);

%   % 
%   % Update:
%   %   Functionality    : Call to update the discrete states
%   %                      during a simulation step.
%   %   C-Mex counterpart: mdlUpdate
%   %
%   block.RegBlockMethod('Update', @Update);

  % 
  % Derivatives:
  %   Functionality    : Call to update the derivatives of the
  %                      continuous states during a simulation step.
  %   C-Mex counterpart: mdlDerivatives
  %
  block.RegBlockMethod('Derivatives', @Derivatives);
%   % 
%   % Projection:
%   %   Functionality    : Call to update the projections during a
%   %                      simulation step.
%   %   C-Mex counterpart: mdlProjections
%   %
%   block.RegBlockMethod('Projection', @Projection);
  
%   % 
%   % SimStatusChange:
%   %   Functionality    : Call when simulation enters pause mode
%   %                      or leaves pause mode.
%   %   C-Mex counterpart: mdlSimStatusChange
%   %
%   block.RegBlockMethod('SimStatusChange', @SimStatusChange);
%   
%   % 
%   % Terminate:
%   %   Functionality    : Call at the end of a simulation for cleanup.
%   %   C-Mex counterpart: mdlTerminate
%   %
%   block.RegBlockMethod('Terminate', @Terminate);

%   %
%   % GetSimState:
%   %   Functionality    : Return the SimState of the block.
%   %   C-Mex counterpart: mdlGetSimState
%   %
%   block.RegBlockMethod('GetSimState', @GetSimState);
%   
%   %
%   % SetSimState:
%   %   Functionality    : Set the SimState of the block using a given value.
%   %   C-Mex counterpart: mdlSetSimState
%   %
%   block.RegBlockMethod('SetSimState', @SetSimState);

  % -----------------------------------------------------------------
  % Register the methods called during code generation.
  % -----------------------------------------------------------------
  
%   %
%   % WriteRTW:
%   %   Functionality    : Write specific information to model.rtw file.
%   %   C-Mex counterpart: mdlRTW
%   %
%   block.RegBlockMethod('WriteRTW', @WriteRTW);
% %endfunction

% -------------------------------------------------------------------
% The local functions below are provided to illustrate how you may implement
% the various block methods listed above.
% -------------------------------------------------------------------

 function CheckPrms(block)
     model   = block.DialogPrm(1).Data;
     IC     = block.DialogPrm(2).Data;
%      if ~exist(model)
%        me = MSLException(block.BlockHandle, message('Simulink:blocks:invalidParameter'));
%        throw(me);
%      end
     
%     if ~strcmp(class(a), 'double')
%       me = MSLException(block.BlockHandle, message('Simulink:blocks:invalidParameter'));
%       throw(me);
%     end
% %endfunction
% 
% function ProcessPrms(block)
% 
%   block.AutoUpdateRuntimePrms;
%  
% %endfunction
% 
% function SetInpPortFrameData(block, idx, fd)
%   
%   block.InputPort(idx).SamplingMode = fd;
%   block.OutputPort(1).SamplingMode  = fd;
%   
% %endfunction
% 
% function SetInpPortDims(block, idx, di)
%   
%   block.InputPort(idx).Dimensions = di;
%   block.OutputPort(1).Dimensions  = di;
% 
% %endfunction
% 
% function SetOutPortDims(block, idx, di)
%   
%   block.OutputPort(idx).Dimensions = di;
%   block.InputPort(1).Dimensions    = di;
% 
% %endfunction
% 
% function SetInpPortDataType(block, idx, dt)
%   
%   block.InputPort(idx).DataTypeID = dt;
%   block.OutputPort(1).DataTypeID  = dt;
% 
% %endfunction
%   
% function SetOutPortDataType(block, idx, dt)
% 
%   block.OutputPort(idx).DataTypeID  = dt;
%   block.InputPort(1).DataTypeID     = dt;
% 
% %endfunction  
% 
% function SetInpPortComplexSig(block, idx, c)
%   
%   block.InputPort(idx).Complexity = c;
%   block.OutputPort(1).Complexity  = c;
% 
% %endfunction 
%   
% function SetOutPortComplexSig(block, idx, c)
% 
%   block.OutputPort(idx).Complexity = c;
%   block.InputPort(1).Complexity    = c;
% 
% %endfunction 
%     
% function DoPostPropSetup(block)
%
%   block.NumDworks = 2;
%   
%   block.Dwork(1).Name            = 'x1';
%   block.Dwork(1).Dimensions      = 1;
%   block.Dwork(1).DatatypeID      = 0;      % double
%   block.Dwork(1).Complexity      = 'Real'; % real
%   block.Dwork(1).UsedAsDiscState = true;
%   
%   block.Dwork(2).Name            = 'numPause';
%   block.Dwork(2).Dimensions      = 1;
%   block.Dwork(2).DatatypeID      = 7;      % uint32
%   block.Dwork(2).Complexity      = 'Real'; % real
%   block.Dwork(2).UsedAsDiscState = true;
%   
%   % Register all tunable parameters as runtime parameters.
%   block.AutoRegRuntimePrms;
% 
% %endfunction

function InitializeConditions(block)
% Initialize 12 States

IC = block.DialogPrm(2).Data;

% IC.P, IC.Q, IC.R are in deg/s ... convert to rad/s
P = IC.P*pi/180; Q = IC.Q*pi/180; R = IC.R*pi/180; 
% IC.Phi, IC.The, IC.Psi are in deg ... convert to rads
Phi = IC.Phi*pi/180; The = IC.The*pi/180; Psi = IC.Psi*pi/180;
U = IC.U; V = IC.V; W = IC.W; 
X = IC.X; Y = IC.Y; Z = IC.Z;

init = [P,Q,R,Phi,The,Psi,U,V,W,X,Y,Z];
for i=1:12
block.OutputPort(i).Data = init(i);
block.ContStates.Data(i) = init(i);
end

block.OutputPort(13).Data = zeros(3,1);
block.OutputPort(14).Data = zeros(3,1);

% function Start(block)
% 
%   block.Dwork(1).Data = 0;
%   block.Dwork(2).Data = uint32(1); 
%    
% %endfunction
% 
% function WriteRTW(block)
%   
%    block.WriteRTWParam('matrix', 'M',    [1 2; 3 4]);
%    block.WriteRTWParam('string', 'Mode', 'Auto');
   
%endfunction

function Outputs(block)
for i = 1:12
  block.OutputPort(i).Data = block.ContStates.Data(i);
end

%endfunction

% function Update(block)
%   
%   block.Dwork(1).Data = block.InputPort(1).Data;
  
%endfunction

function Derivatives(block)
% Name all the states and motor inputs

% Load model data selected in parameter block
%which('model')

model = block.DialogPrm(1).Data;

% P Q R in units of rad/sec in body frame
p = block.ContStates.Data(1);
q = block.ContStates.Data(2);
r = block.ContStates.Data(3);
% Phi The Psi in radians in inertial/earth frame
Phi = block.ContStates.Data(4);
The = block.ContStates.Data(5);
Psi = block.ContStates.Data(6);
% U V W in units of m/s in body frame
U = block.ContStates.Data(7);
V = block.ContStates.Data(8);
W = block.ContStates.Data(9);
% X Y Z in units of m in inertial/earth frame
X = block.ContStates.Data(10);
Y = block.ContStates.Data(11);
Z = block.ContStates.Data(12);
% w values in rev/min! NOT radians/s!!!!
w1 = block.InputPort(1).Data;
w2 = block.InputPort(2).Data;
w3 = block.InputPort(3).Data;
w4 = block.InputPort(4).Data;
w5 = block.InputPort(5).Data;
w6 = block.InputPort(6).Data;
w7 = block.InputPort(7).Data;
w8 = block.InputPort(8).Data;
w  = [w1; w2; w3; w4;w5; w6; w7; w8];

%to fix numerical issues
w=round(w,2);

% Disturbances Input------
% Dist_tau = block.InputPort(5).Data(1:3);
% Dist_F   = block.InputPort(5).Data(4:6);
%------

%% Notation
% b - body frame
% e - intertial/earth frame
% Theta - Euler angles vector, Thetadot - Euler angles derivatives
% The -> pitch, Psi -> yaw, Phi -> roll

% transformation and rotation matrices
% T_b_e - transformation matrix from body frame to intertial frame for Euler derivatives
% R_b_e - rotation matrix from body frame to intertial frame 
% R_e_b - rotation matrix from intertial frame to body frame 
% E_m_b - tilted mixing matrix Thrust from motor frame to body frame 
% R_m - displacement matrix Torque/Moment of motor frame to Centre Structure Frame 

%% Parameters
% model.dl - long arm separation (m)
% model.ds - short arm separation (m)
% model.ct - coefficient of thrust (Ns^2)
% model.cq - drag coefficient (Nms^2)
% model.mass - total mass (Kg)
% model.Jb - total intertia matrix in the center structure frame (Kg*m^2)
% octomodel.g - gravity acceleration

%% rotation matrix
% Z-Y-X rotation convention 
R_b_e = [cos(Psi)*cos(The) cos(Psi)*sin(The)*sin(Phi)-sin(Psi)*cos(Phi) cos(Psi)*sin(The)*cos(Phi)+sin(Psi)*sin(Phi);
       sin(Psi)*cos(The) sin(Psi)*sin(The)*sin(Phi)+cos(Psi)*cos(Phi) sin(Psi)*sin(The)*cos(Phi)-cos(Psi)*sin(Phi);
       -sin(The)         cos(The)*sin(Phi)                            cos(The)*cos(Phi)]; % Rib
R_e_b = R_b_e'; % Rbi  
gamma=model.tiltmotorangle;
alpha=model.separationmotorangle;
E_m_b = [0 -sin(alpha)*sin(gamma) sin(gamma) -cos(alpha)*sin(gamma) 0 sin(alpha)*sin(gamma) -sin(gamma) cos(alpha)*sin(gamma);
         -sin(gamma)  cos(alpha)*sin(gamma) 0 -sin(alpha)*sin(gamma) sin(gamma) -cos(alpha)*sin(gamma) 0 sin(alpha)*sin(gamma);
         -cos(gamma) -cos(gamma) -cos(gamma) -cos(gamma) -cos(gamma) -cos(gamma) -cos(gamma) -cos(gamma)];
dl=model.dl;constant=(1/sqrt(2));
ds=model.ds;
R_m=[dl 0 0; ...
    constant*ds constant*ds 0; ...
    0 dl 0; ...
    -constant*ds constant*ds 0; ...
    -dl 0 0; ...
    -constant*ds -constant*ds 0; ...
    0 -dl 0; ...
    constant*ds -constant*ds 0]';

%% Force created by motor
%through the lumped ct parameter
Fw=(model.ct .*(w.^2));
if w(2)>3940
    see=1;
end

Ftotal=zeros(3,1);
for i=1:8
    e_i_b=E_m_b(:,i);
    Ftemp=Fw(i).*e_i_b;
    Ftotal=Ftotal+Ftemp;
end

%% Torque created by motor
%if using lumped cq parameter
Qw=(model.cq .*(w.^2));

%using experimental calculation according to the propeller size
% if sum(w) > 24000
%     Qw = (1.065e-08).*w.^2 + (-3.45e-06).*w + (0.0064).*ones(length(w),1);
% end

Qtotal=zeros(3,1);
for i=1:8
    e_i_b=E_m_b(:,i);
    Qtemp1=((-1)^(i+1)*Qw(i)).*e_i_b;
    
    r_m=R_m(:,i);
    Qtemp2=cross( r_m,e_i_b ).*Fw(i);
    
    Qtotal=Qtotal + (Qtemp1+Qtemp2);
end

%% Moments in the body frame (N*m)
% 1-Gyroscopic Moment/Precession
% tau_motorGyro = [Q*model.Jm*2*pi/60*(-w1-w3+w2+w4); P*model.Jm*2*pi/60*(w1+w3-w2-w4); 0];

% 2-moment created by motor
% Mb = (model.dctcq*(w.^2)); % Mb = (model.dctcq*(w.^2))+ tau_motorGyro + (Dist_tau);  % Mb = [tau1 tau2 tau3]'
Mb = Qtotal;

% dynamic equation of the angular velocity - obtain dP dQ dR
w_b = [p; q; r]; % omb_bi
w_x_b = [ 0,-r, q; % OMb_bi
           r, 0,-p;
          -q, p, 0];
% dynamic equation of motion for angular acceleration
wdot_b = model.Jbinv * (Mb - (w_x_b * model.Jb * w_b)); % b_omdotb_bi

% angular acceleration
dP = wdot_b(1);
dQ = wdot_b(2);
dR = wdot_b(3);

%angular velocity - Euler angles derivatives in the intertial frame
T_b_e = [1,tan(The)*sin(Phi), tan(The)*cos(Phi);
         0,         cos(Phi),         -sin(Phi);
         0,sin(Phi)/cos(The),cos(Phi)/cos(The)];  
Thetadot = T_b_e*w_b;     
dPhi = Thetadot(1);
dTheta = Thetadot(2);
dPsi = Thetadot(3);

%% -Forces calculated in the body frame (Newtons)
% 1-Thrust due to motor speed
% Thrust force in the body frame generated by the motors
% Ftotal considers the tilted angle of the motors, Thrust_b does not consider this issue
% Thrust_b = [0; 0; sum(model.ct*(w.^2))];   % Thrust= Ct*w^2

% 2-gravitational force
ge = [0; 0; -model.g];
Fg = R_e_b*ge; % gb

% Linear Velocity in the body frame
v_b = [U;V;W];

%net thrust acceleration in body frame
Acc_b=(1/model.mass)*Ftotal;
%to deal with numerical issues
Acc_b=round(Acc_b,2);

% dynamic equation of motion for linear acceleration
%with disturbance
% Dist_Fb = R_e_b*Dist_F;
% b_dv = (1/model.mass)*Thrust_b+Fg+Dist_Fb-w_x_b*v_b; % Acceleration in body frame (FOR VELOCITY)

%without disturbance
% vdot_b = (1/model.mass)*Thrust_b + Fg - w_x_b*v_b; % Acceleration in body frame (FOR VELOCITY)
vdot_b = Acc_b + Fg - w_x_b*v_b; % Acceleration in body frame (FOR VELOCITY)



%simulating a force equal to the gravity force
% vdot_b =  - w_x_b*v_b; % Acceleration in body frame (FOR VELOCITY)


% linear acceleration
dU = vdot_b(1);
dV = vdot_b(2);
dW = vdot_b(3);

% velocity vector in the intertial frame
v_e = R_b_e*v_b; % Units OK SI: Velocity of body frame w.r.t inertia frame (FOR POSITION)

dX = v_e(1);
dY = v_e(2);
dZ = v_e(3);

% Rough rule to impose a "ground" boundary...could easily be improved...
if ((Z<=0) && (dZ<=0)) % better  version then before?
    dZ = 0;
    block.ContStates.Data(12) = 0;
end

block.OutputPort(13).Data = Ftotal;
block.OutputPort(14).Data = Qtotal;

%% all variables
f = [dP dQ dR dPhi dTheta dPsi dU dV dW dX dY dZ].';
  %This is the state derivative vector
block.Derivatives.Data(1:12) = f;




%endfunction