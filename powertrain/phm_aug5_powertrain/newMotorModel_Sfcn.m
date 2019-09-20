function newMotorModel_Sfcn(block)
% Level-2 MATLAB file S-Function for inherited sample time demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts  = 2; %voltage, Df
  block.NumOutputPorts = 2; %current, rpm

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  for i = 1:2
      block.InputPort(i).Dimensions        = 1;
      block.InputPort(i).DirectFeedthrough = false;
      block.InputPort(i).SamplingMode      = 'Sample';
  end
  
  for i = 1:2
    block.OutputPort(i).Dimensions       = 1;
    block.OutputPort(i).SamplingMode     = 'Sample';
  end
  
  block.NumDialogPrms     = 1; % motorParams  
  
  %% Set block sample time to inherited
%   block.SampleTimes = [-1 0];
  block.SampleTimes = [0 0];
  
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
  block.Dwork(1).Dimensions      = 1; % y
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
    %% Initialize Dwork
    % Initialize 5 states and outputs
    motor = block.DialogPrm(1).Data;
    init = [0];
    for i=1:1
        block.Dwork(1).Data(i) = init(i);
    end
    
    block.OutputPort(1).Data = 0;
    block.OutputPort(2).Data = 0;
  
%endfunction

function Output(block)
    % current
    block.OutputPort(1).Data = 1.234; 
    
    %rpm
    block.OutputPort(2).Data = block.Dwork(1).Data(1) * (60 / (2*pi));
  
%endfunction

function Update(block)
    %get arguments
    motor = block.DialogPrm(1).Data;
    dt = motor.dt;
    
    %parameters
    %Df = motor.Df; % viscous damping coefficient or dynamic Friction coefficient
    Ke = motor.Ke; %back electromotive force constant
    Tf = motor.Tf; % static friction torque
    R  = motor.Req; % quivalent electric resistance of the coils
    Jm = motor.Jm; % inertia 
    dt = motor.dt;
    
    a  = motor.a;
    b  = motor.b;
    c  = motor.c;
    
    %input voltage and friction
    u  = block.InputPort(1).Data;
    Df = block.InputPort(2).Data;
    
    % state matricies
    A = -(Df + (Ke^2/R))/Jm;
    B = Ke/(R*Jm);
    C = 1;
    D = 0;
    
    % update equation
    [t,y]=ode45(@(t,x) sysmot(t,x,A,B,a,b,c,u),[0 dt],block.Dwork(1).Data(1));
    %x = block.Dwork(1).Data(1);
    %xdot =(a*x^3+b*x^2+(c+A)*x)+B*u;
    
    block.Dwork(1).Data(1)=y(end);

  
%endfunction

function xdot=sysmot(t,x,A,B,a,b,c,u)
 xdot =(a*x^3+b*x^2+(c+A)*x)+B*u;

