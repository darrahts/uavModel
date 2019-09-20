function batterySfun3(block)
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
  block.NumInputPorts  = 1;
  %------
  block.NumOutputPorts = 5;
  
  % Set up the port properties to be inherited or dynamic.
  
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode      = 'Sample';
  
  
  for i = 1:5;
  block.OutputPort(i).Dimensions       = 1;
  block.OutputPort(i).SamplingMode     = 'Sample';
  end

%   % Override the input port properties.
%   block.InputPort(1).DatatypeID  = 0;  % double
%   block.InputPort(1).Complexity  = 'Real';
%   
%   % Override the output port properties.
%   block.OutputPort(1).DatatypeID  = 0; % double
%   block.OutputPort(1).Complexity  = 'Real';

  % Register the parameters.
  block.NumDialogPrms     = 3;
  
  % Set up the continuous states.
 % block.NumContStates = 3;

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
   block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup);
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

  
%  block.RegBlockMethod('SetInputPortSamplingMode', @SetInpPortFrameData);
  
%   % 
%   % Update:
%   %   Functionality    : Call to update the discrete states
%   %                      during a simulation step.
%   %   C-Mex counterpart: mdlUpdate
%   %
   block.RegBlockMethod('Update', @Update);

  % 
  % Derivatives:
  %   Functionality    : Call to update the derivatives of the
  %                      continuous states during a simulation step.
  %   C-Mex counterpart: mdlDerivatives
  %
%  block.RegBlockMethod('Derivatives', @Derivatives);
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
  block.RegBlockMethod('Terminate', @Terminate);

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
     battery   = block.DialogPrm(1).Data;
     soc_ocv   = block.DialogPrm(2).Data;
     dt        = block.DialogPrm(3).Data;
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
function DoPostPropSetup(block)

  block.NumDworks = 1;
  
  block.Dwork(1).Name            = 'x1';
  block.Dwork(1).Dimensions      = 3;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
%   block.Dwork(2).Name            = 'numPause';
%   block.Dwork(2).Dimensions      = 1;
%   block.Dwork(2).DatatypeID      = 7;      % uint32
%   block.Dwork(2).Complexity      = 'Real'; % real
%   block.Dwork(2).UsedAsDiscState = true;
  
  % Register all tunable parameters as runtime parameters.
%   block.AutoRegRuntimePrms;
% 
% %endfunction

% 
% function SetInpPortFrameData(block, idx, fd)
% 
%     block.InputPort(idx).SamplingMode = fd;
%     for i = 1:block.NumOutputPorts
%         block.OutputPort(i).SamplingMode = fd;
%     end


function InitializeConditions(block)
% Initialize 5 states and outputs
battery = block.DialogPrm(1).Data;

init = [battery.z, battery.Ir, battery.h, battery.v, battery.SOC ];
for i=1:3
block.Dwork.Data(i) = init(i);
end

for i=1:5
block.OutputPort(i).Data = init(i);
end

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
    battery = block.DialogPrm(1).Data;
    soc_ocv = block.DialogPrm(2).Data;
    %parameters
    n = battery.n;
    Q = battery.Q;
    g = battery.g;
    Mo = battery.M0;
    M = battery.M;
    R0 = battery.R0;
    R1 = battery.R1;
    C1 = battery.C1;
    EOD = battery.EOD;
    %input current
    u = block.InputPort(1).Data;
    
    z=block.Dwork(1).Data;
    block.OutputPort(1).Data = z;

    Ir=block.Dwork(2).Data;
    block.OutputPort(2).Data = Ir;

    h=block.Dwork(3).Data;
    block.OutputPort(3).Data = h;
    
    soc = (z + 31.8816) /(1.0 + 31.8816);
    idx = ceil(soc*100) +1;
    v = (soc_ocv(idx) + Mo*sign(u) + M*h - sum(R1*Ir) - R0*u);
    block.OutputPort(4).Data = v;
    block.OutputPort(5).Data = soc;

%endfunction

function Update(block)
    
    battery = block.DialogPrm(1).Data;
    dt = block.DialogPrm(3).Data;

    %states
    z = block.Dwork(1).Data;
    Ir = block.Dwork(2).Data;
    h = block.Dwork(3).Data;
    %input current
    u = block.InputPort(1).Data;
    %parameters
    n = battery.n;
    Q = battery.Q;
    g = battery.g;
    Mo = battery.M0;
    M = battery.M;
    R0 = battery.R0;
    R1 = battery.R1;
    C1 = battery.C1;
    EOD = battery.EOD;


    Arc = zeros(length(R1));
    for i = 1:length(R1)
        Arc(i,i) = exp(-dt/(R1*C1));
    end
    Brc = zeros(length(R1), 1);
    for i = 1:length(R1)
        Brc(i,1) = 1 - exp(-dt/(R1*C1));
    end

    H(1) = exp(-abs((n*.1*g*dt)/Q));

    z = z - ((n*dt)/Q)*u;
    Ir = Arc*Ir + Brc*u;
    H = exp(-abs((n*u*g*dt)/Q));
    h = H*h + (H-1)*sign(u);
    
    block.Dwork(1).Data=z;
    block.Dwork(2).Data=Ir;
    block.Dwork(3).Data=h;
%   
%   block.Dwork(1).Data = block.InputPort(1).Data;
  
%endfunction

% function Derivatives(block)
% Name all the states and motor inputs



%endfunction

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
function Terminate(block)

%end Terminate

















