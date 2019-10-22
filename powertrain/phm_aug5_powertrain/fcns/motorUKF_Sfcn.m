function motorUKF_Sfcn(block)
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
  block.Dwork(1).Dimensions      = 3; % y, sigmaX, xhat
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
    %% Initialize Dwork
    % Initialize 5 states and outputs
    motor = block.DialogPrm(1).Data;
    init = [0 0 200];
    for i=1:2
        block.Dwork(1).Data(i) = init(i);
    end
    
    block.OutputPort(1).Data = 0;
    block.OutputPort(2).Data = 0;
  
%endfunction

function Output(block)
    
    % params
    motor = block.DialogPrm(1).Data;
    xhat  = block.Dwork(1).Data(3);
    u  = block.InputPort(1).Data;
    Ke = motor.Ke; %back electromotive force constant
    R  = motor.Req; % equivalent electric resistance of the coils   
    % current
    current = (u - (Ke*xhat))/(4*R);
    block.OutputPort(1).Data = current; 
    
    %rpm
    block.OutputPort(2).Data = block.Dwork(1).Data(1) * (60 / (2*pi));
    
    
    
    
%endfunction

function Update(block)
    %get arguments
    motor = block.DialogPrm(1).Data;
    
    %parameters
    Ke = motor.Ke; %back electromotive force constant
    Tf = motor.Tf; % static friction torque
    R  = motor.Req; % equivalent electric resistance of the coils
    Jm = motor.Jm; % inertia 
    dt = motor.dt*10;
    
    a  = motor.a;
    b  = motor.b;
    c  = motor.c;
    
    %input voltage and friction
    u  = block.InputPort(1).Data;
    Df = block.InputPort(2).Data;
    
    % state matricies
    A = -(Df + (Ke^2/(.77*R)))/Jm;
    B = Ke/(.77*R*Jm);
    C = 1;
    D = 0;
    
    % ekf equations
    Chat=1;
    Bhat=1;
    Dhat=1;

    %simulation variables
    SigmaW = 1e-5; % Process-noise covariance
    SigmaV = 10; % Sensor-noise covariance
    SigmaX = block.Dwork(1).Data(2); % Initialize Kalman filter covariance
    xhat   = block.Dwork(1).Data(3); 
    
    w = chol(SigmaW,'lower')*(randn(length(xhat))/4); % rand. process noise
    v = chol(SigmaV,'lower')*(randn(length(Chat*xhat))/4); % rand. sensor noise

    
    
    % update equation
    [t,y] = ode45(@(t,xhat) sysmot(t,xhat,A,B,a,b,c,u,w),[0 dt],xhat);
    xhat  = y(end);
    %[t,y] = ode45(@(t,xhat) sysmothat(t,xhat,A,B,a,b,c),[0 dt],xhat);
    Ahat  = xhat; y(end);
    
    % EKF Step 1b: Error-covariance time update
    SigmaX = Ahat*SigmaX*Ahat' + Bhat*SigmaW*Bhat';     
    
    % KF Step 1c: Estimate system output
    ytrue = C*xhat + D*u;
    yhat  = Chat*xhat + D*u + v;  
    
    % EKF Step 2a: Compute Kalman gain matrix L(k)
    SigmaY = Chat*SigmaX*Chat' + Dhat*SigmaV*Dhat';
    L = SigmaX*Chat'/SigmaY;
  
    % EKF Step 2b: State-estimate measurement update
    xhat = xhat + L*(ytrue - yhat);
    
    % EKF Step 2c: Error-covariance measurement update 
    % Help to keep robust: to ensure that the covariance matrices remain symmetric and positive semidefinite.
    SigmaX = SigmaX - L*SigmaY*L';
    [~,S,V] = svd(SigmaX);
    HH = V*S*V';
    SigmaX = (SigmaX + SigmaX' + HH + HH')/4; %
    
    block.Dwork(1).Data(1) = y(end);
    block.Dwork(1).Data(2) = SigmaX;
    block.Dwork(1).Data(3) = xhat;

  
%endfunction

function xdot=sysmot(t,x,A,B,a,b,c,u,w)
 xdot =(a*x^3+b*x^2+(c+A)*x)+B*u+w;
