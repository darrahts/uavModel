function batteryModel_Sfcn(block)
% Level-2 MATLAB file S-Function for inherited sample time demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 2;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode      = 'Sample';
  
  for i = 1:2
    block.OutputPort(i).Dimensions       = 1;
    block.OutputPort(i).SamplingMode     = 'Sample';
  end
  
  block.NumDialogPrms     = 2;
  
  %% Set block sample time to inherited
%    block.SampleTimes = [-1 0];
  block.SampleTimes = [0.01 0];
  
  %% Set the block simStateCompliance to default (i.e., same as a built-in block)
  block.SimStateCompliance = 'DefaultSimState';

  %% Register methods
  block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
  block.RegBlockMethod('InitializeConditions',    @InitConditions);  
  block.RegBlockMethod('Outputs',                 @Output);  
  block.RegBlockMethod('Update',                  @Update);  
  
%endfunction

function DoPostPropSetup(block)

  %% Setup Dwork
  block.NumDworks = 1;
  block.Dwork(1).Name = 'x0'; 
  block.Dwork(1).Dimensions      = 3;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
    %% Initialize Dwork
    % Initialize 5 states and outputs
    battery = block.DialogPrm(1).Data;
    init = [battery.z, battery.Ir, battery.h];
    for i=1:3
        block.Dwork(1).Data(i) = init(i);
    end
    
    block.OutputPort(1).Data = battery.v;
    block.OutputPort(2).Data = battery.z;
  
%endfunction

function Output(block)
    %get arguments
    battery = block.DialogPrm(1).Data;
    dt = block.DialogPrm(2).Data;
    
    %parameters
    M0 = battery.M0;
    M = battery.M;
    R0 = battery.R0;
    R = battery.R;
    
    %input current
    u = block.InputPort(1).Data;
    
    z=block.Dwork(1).Data(1);
    Ir=block.Dwork(1).Data(2);
    h=block.Dwork(1).Data(3);
    
    v = getOCV(z, battery) + M*h + M0*sign(u) - R*Ir - R0*u;
      
    block.OutputPort(1).Data = v;
    block.OutputPort(2).Data = z;
  
%endfunction

function Update(block)
    %get arguments
    battery = block.DialogPrm(1).Data;
    dt = block.DialogPrm(2).Data;
    
    %parameters
    n = battery.n;
    Q = battery.Q;
    G = battery.G;
    RC = exp(-dt/abs(battery.RC))';
    
    %input current
    u = block.InputPort(1).Data;
    
    z=block.Dwork(1).Data(1);
    Ir=block.Dwork(1).Data(2);
    h=block.Dwork(1).Data(3);
    
    H = exp(-abs(n*u*G*dt/(3600*Q)));
    
    Ir_hat = RC*Ir + (1-diag(RC))*u;
    h_hat = H.*h + (H-1).*sign(u);
    z_hat = z - n*u/3600/Q;
    
    z_hat = min(1.05, max(-.05,z_hat));
    h_hat = min(1,max(-1,h_hat));
    
    block.Dwork(1).Data(1)=z_hat;
    block.Dwork(1).Data(2)=Ir_hat;
    block.Dwork(1).Data(3)=h_hat;

  
%endfunction

