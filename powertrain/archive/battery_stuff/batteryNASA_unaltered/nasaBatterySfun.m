function msfcn_dsc(block)
% Level-2 MATLAB file S-Function for inherited sample time demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts  = 1;
  block.NumOutputPorts = 6;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode      = 'Sample';
  
  for i = 1:6
    block.OutputPort(i).Dimensions       = 1;
    block.OutputPort(i).SamplingMode     = 'Sample';
  end
  
  block.NumDialogPrms     = 1; % use params = Parameters in the workspace, pass params
  
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
  block.Dwork(1).Name = 'x'; 
  block.Dwork(1).Dimensions      = 5;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
    %% Initialize Dwork
    % Initialize 5 states and outputs
    battery = block.DialogPrm(1).Data;
    init = [battery.x0.Tb, battery.x0.qb, battery.x0.qcp, battery.x0.qcs, battery.V0];
    for i=1:5
        block.Dwork(1).Data(i) = init(i);
    end

    for i=1:5
        block.OutputPort(i).Data = init(i);
    end
    block.OutputPort(6).Data = 0;
  
%endfunction

function Output(block)
    %get arguments
    battery = block.DialogPrm(1).Data;
    
    Tb = block.Dwork(1).Data(1);
    qb = block.Dwork(1).Data(2);
    qcp = block.Dwork(1).Data(3);
    qcs = block.Dwork(1).Data(4);
  
    i = block.InputPort(1).Data;
    
    % Constraints
    Tbm = Tb;
    Vcs = qcs./battery.Cs;
    Vcp = qcp./battery.Ccp;
    SOC = (battery.CMax - battery.qMax + qb)./battery.CMax;
    Cb = battery.Cbp0.*SOC.^3 + battery.Cbp1.*SOC.^2 + battery.Cbp2.*SOC + battery.Cbp3;
    Vb = qb./Cb;
    Vp = Vb - Vcp - Vcs;
    Vm = Vp;
    
    block.OutputPort(1).Data = Tbm;
    block.OutputPort(2).Data = qb;
    block.OutputPort(3).Data = qcp;    
    block.OutputPort(4).Data = qcs;
    block.OutputPort(5).Data = Vm;
    block.OutputPort(6).Data = SOC;
  
%endfunction

function Update(block)
    %get arguments
    battery = block.DialogPrm(1).Data;

    Tb  = block.Dwork(1).Data(1);
    qb  = block.Dwork(1).Data(2);
    qcp = block.Dwork(1).Data(3);
    qcs = block.Dwork(1).Data(4);
    Vm  = block.Dwork(1).Data(5);
  
    i = block.InputPort(1).Data;
    
    dt = battery.sampleTime;
    
    Vcs = qcs./battery.Cs;
    Vcp = qcp./battery.Ccp;
    SOC = (battery.CMax - battery.qMax + qb)./battery.CMax;
    Cb = battery.Cbp0.*SOC.^3 + battery.Cbp1.*SOC.^2 + battery.Cbp2.*SOC + battery.Cbp3;
    Rcp = battery.Rcp0 + battery.Rcp1.*exp(battery.Rcp2.*(-SOC + 1));
    Vb = qb./Cb;
    Tbdot = (Rcp.*battery.Rs.*battery.ha.*(battery.Ta - Tb) + Rcp.*Vcs.^2.*battery.hcs + battery.Rs.*Vcp.^2.*battery.hcp)./(battery.Jt.*Rcp.*battery.Rs);
    Vp = Vb - Vcp - Vcs;
    ip = Vp./battery.Rp;
    ib = i + ip;
    icp = ib - Vcp./Rcp;
    qcpdot = icp;
    qbdot = -ib;
    ics = ib - Vcs./battery.Rs;
    qcsdot = ics;
    
    block.Dwork(1).Data(1) = Tb  + Tbdot*dt;
    block.Dwork(1).Data(2) = qb  + qbdot*dt;
    block.Dwork(1).Data(3) = qcp + qcpdot*dt;
    block.Dwork(1).Data(4) = qcs + qcsdot*dt;
    block.Dwork(1).Data(5) = Vm;
  
%endfunction






















