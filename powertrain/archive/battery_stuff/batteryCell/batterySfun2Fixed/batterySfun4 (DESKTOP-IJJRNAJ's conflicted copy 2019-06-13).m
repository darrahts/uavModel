function msfcn_dsc(block)
% Level-2 MATLAB file S-Function for inherited sample time demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 5;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode      = 'Sample';
  
  for i = 1:5
    block.OutputPort(i).Dimensions       = 1;
    block.OutputPort(i).SamplingMode     = 'Sample';
  end
  
  block.NumDialogPrms     = 3;
  
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
  block.Dwork(1).Dimensions      = 3;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
    %% Initialize Dwork
    % Initialize 5 states and outputs
    battery = block.DialogPrm(1).Data;
    init = [battery.z, battery.Ir, battery.h, battery.v, battery.SOC ];
    for i=1:3
        block.Dwork(1).Data(i) = init(i);
    end

    for i=1:5
        block.OutputPort(i).Data = init(i);
    end
  
%endfunction

function Output(block)
    %get arguments
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
    
    z=block.Dwork(1).Data(1);
    block.OutputPort(1).Data = z;

    Ir=block.Dwork(1).Data(2);
    block.OutputPort(2).Data = Ir;

    h=block.Dwork(1).Data(3);
    block.OutputPort(3).Data = h;
    
    soc = (z + 31.8816) /(1.0 + 31.8816);
    idx = ceil(soc*100) +1;
    v = (soc_ocv(idx) + Mo*sign(u) + M*h - sum(R1*Ir) - R0*u);
    block.OutputPort(4).Data = v;
    block.OutputPort(5).Data = soc;
  
%endfunction

function Update(block)
    %get arguments
    battery = block.DialogPrm(1).Data;
    dt = block.DialogPrm(3).Data;

    %states
    z = block.Dwork(1).Data(1);
    Ir = block.Dwork(1).Data(2);
    h = block.Dwork(1).Data(3);
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
    
    block.Dwork(1).Data(1)=z;
    block.Dwork(1).Data(2)=Ir;
    block.Dwork(1).Data(3)=h;

  
%endfunction

