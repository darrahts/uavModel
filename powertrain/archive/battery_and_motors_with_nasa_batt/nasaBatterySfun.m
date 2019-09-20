function nasaBatterySfun(block)

  setup(block);
  
%endfunction

function setup(block)
  
  % current, simulation time (to capture the time when eod is reached)
  block.NumInputPorts  = 2;
  
  % voltage, soc, eod
  block.NumOutputPorts = 3;

  % default
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  for i = 1:2
      block.InputPort(i).Dimensions        = 1;
      block.InputPort(i).DirectFeedthrough = false;
      block.InputPort(i).SamplingMode      = 'Sample';
  end
  
  for i = 1:3
    block.OutputPort(i).Dimensions       = 1;
    block.OutputPort(i).SamplingMode     = 'Sample';
  end
  
  block.NumDialogPrms     = 1; % use batteryParams.mat
  
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

  % set up internal states
  block.NumDworks = 1;
  block.Dwork(1).Name = 'x'; 
  
  % Tb, qb, qcp, qcs
  block.Dwork(1).Dimensions      = 4;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;

%endfunction

function InitConditions(block)
    %% Initialize Dwork (discrete time simulation block) with 4 states and 3 outputs
    
    % load the battery params
    battery = block.DialogPrm(1).Data;
    
    % set states to initial conditions
    init = [battery.x0.Tb, battery.x0.qb, battery.x0.qcp, battery.x0.qcs];
    for i=1:4
        block.Dwork(1).Data(i) = init(i);
    end

    % set outputs to initial conditions
    block.OutputPort(1).Data = battery.V0;
    block.OutputPort(2).Data = battery.soc;
    block.OutputPort(3).Data = battery.eod
  
%endfunction

function Output(block)
    % get battery params
    battery = block.DialogPrm(1).Data;
    
    % get internal states
    qb = block.Dwork(1).Data(2);
    qcp = block.Dwork(1).Data(3);
    qcs = block.Dwork(1).Data(4);
    
    % calculate SOC
    SOC = (battery.CMax - battery.qMax + qb)./battery.CMax;
    
    % calculate voltage
    Cb = battery.Cbp0.*SOC.^3 + battery.Cbp1.*SOC.^2 + battery.Cbp2.*SOC + battery.Cbp3;
    Vcs = qcs./battery.Cs;
    Vcp = qcp./battery.Ccp;
    Vb = qb./Cb;
    V = Vb - Vcp - Vcs;
    
    % calculate eod (flag)
    eod = 0;
    if V < 3.03
        % simulation time when eod is reached
        eod = block.InputPort(2).Data;
    end
    
    % set output ports
    block.OutputPort(1).Data = V;
    block.OutputPort(2).Data = SOC;
    block.OutputPort(3).Data = eod;    
  
%endfunction

function Update(block)
    % get battery params
    battery = block.DialogPrm(1).Data;

    % get internal states
    Tb  = block.Dwork(1).Data(1);
    qb  = block.Dwork(1).Data(2);
    qcp = block.Dwork(1).Data(3);
    qcs = block.Dwork(1).Data(4);
  
    % get input (current)
    i = block.InputPort(1).Data;
    
    dt = battery.sampleTime;
    
    % calculate internal parameter values
    Vcs = qcs./battery.Cs;
    Vcp = qcp./battery.Ccp;
    SOC = (battery.CMax - battery.qMax + qb)./battery.CMax;
    Cb = battery.Cbp0.*SOC.^3 + battery.Cbp1.*SOC.^2 + battery.Cbp2.*SOC + battery.Cbp3;
    Rcp = battery.Rcp0 + battery.Rcp1.*exp(battery.Rcp2.*(-SOC + 1));
    Vb = qb./Cb;
    Vp = Vb - Vcp - Vcs;
    ip = Vp./battery.Rp;
    
    % calculate state update values
    Tb_dot = (Rcp.*battery.Rs.*battery.ha.*(battery.Ta - Tb) + Rcp.*Vcs.^2.*battery.hcs + battery.Rs.*Vcp.^2.*battery.hcp)./(battery.Jt.*Rcp.*battery.Rs);
    qb_dot = -(i + ip);
    qcp_dot = qb_dot - Vcp./Rcp;
    qcs_dot = qb_dot - Vcs./battery.Rs;
    
    % update the states
    block.Dwork(1).Data(1) = Tb  + Tb_dot*dt;
    block.Dwork(1).Data(2) = qb  + qb_dot*dt;
    block.Dwork(1).Data(3) = qcp + qcp_dot*dt;
    block.Dwork(1).Data(4) = qcs + qcs_dot*dt;
  
%endfunction






















