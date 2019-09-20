function batteryUKF(block)
% Level-2 MATLAB file S-Function for inherited sample time demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts  = 2;
  block.NumOutputPorts = 3;

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  % voltage measurement
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode      = 'Sample';
  
  % current measurement
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).DirectFeedthrough = false;
  block.InputPort(2).SamplingMode      = 'Sample';

  % soc estimate
  block.OutputPort(1).Dimensions       = 1;
  block.OutputPort(1).SamplingMode     = 'Sample';
  
  % soc bounds
  block.OutputPort(2).Dimensions       = 1;
  block.OutputPort(2).SamplingMode     = 'Sample';
  
  % voltage estimate
  block.OutputPort(3).Dimensions       = 1;
  block.OutputPort(3).SamplingMode     = 'Sample';
  
  block.NumDialogPrms     = 3; % battery, dt, ukfBatteryParams
  
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
  block.NumDworks = 4;
  
  block.Dwork(1).Name = 'x0'; 
  block.Dwork(1).Dimensions      = 7;
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name = 'sigmax1'; 
  block.Dwork(2).Dimensions      = 3;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name = 'sigmax2'; 
  block.Dwork(3).Dimensions      = 3;
  block.Dwork(3).DatatypeID      = 0;
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name = 'sigmax3'; 
  block.Dwork(4).Dimensions      = 3;
  block.Dwork(4).DatatypeID      = 0;
  block.Dwork(4).Complexity      = 'Real';
  block.Dwork(4).UsedAsDiscState = true;

  
%endfunction

function InitConditions(block)
    %% Initialize Dwork
    % Initialize 3 states and outputs
    battery = block.DialogPrm(1).Data;
    init = [battery.z, battery.Ir, battery.h];
    for i=1:3
        block.Dwork(1).Data(i) = init(i);
    end
    
    params = block.DialogPrm(3).Data;
    
    block.Dwork(2).Data(1:3) = params.sigmax(1,:);
    block.Dwork(3).Data(1:3) = params.sigmax(2,:);
    block.Dwork(4).Data(1:3) = params.sigmax(3,:);

    % prev current
    block.Dwork(1).Data(4) = 0;
    
    % bounds
    block.Dwork(1).Data(5) = 0;
    
    % voltage
    block.Dwork(1).Data(6) = battery.v0;
    
    % sign
    block.Dwork(1).Data(7) = 1;
    
    % gets the soc given the current voltage
    block.OutputPort(1).Data = find(battery.soc_ocv>=battery.v0,1);
  
%endfunction

function Output(block)
    % soc
    block.OutputPort(1).Data = block.Dwork(1).Data(1);
    
    % soc bounds
    block.OutputPort(2).Data = block.Dwork(1).Data(5);
    
    % voltage estimate
    block.OutputPort(3).Data = block.Dwork(1).Data(6);
  
%endfunction

function Update(block)
    % get block arguments
    battery = block.DialogPrm(1).Data;
    dt      = block.DialogPrm(2).Data;
    params  = block.DialogPrm(3).Data;
    
    % parameters
    n = battery.n;
    Q = battery.Q;
    G = battery.G;
    M0 = battery.M0;
    M = battery.M;
    R0 = battery.R0;
    R = battery.R;
    RC = exp(-dt/abs(battery.RC))';
    
    % input current and voltage
    vk = block.InputPort(1).Data;
    ik = block.InputPort(2).Data;
    
    if ik<0, ik=ik*n; end;
    
    % states
    z  = block.Dwork(1).Data(1);
    Ir = block.Dwork(1).Data(2);
    h  = block.Dwork(1).Data(3);
    
    xhat = [Ir h z]';
    
    I = ik; %block.Dwork(1).Data(4);
    
    sigmax = zeros(3,3);
    sigmax(1,:) = block.Dwork(2).Data(1:3);
    sigmax(2,:) = block.Dwork(3).Data(1:3);
    sigmax(3,:) = block.Dwork(4).Data(1:3);
    
    % Get data stored in params structure
    sigmaw = params.sigmaw;
    snoise = params.snoise;
    qbump = params.qbump;
    nx = params.nx;
    nw = params.nw;
    nv = params.nv;
    na = params.na;
    wc = params.wc;
    wm = params.wm;
    ht = params.h; % tuning factor
    
    if abs(ik)>Q/100, block.Dwork(1).Data(7) = sign(ik); end;
    
    signIk = block.Dwork(1).Data(7);
      
      
       % Step 1a: State estimate time update
  %          - Create xhatminus augmented SigmaX points
  %          - Extract xhatminus state SigmaX points
  %          - Compute weighted average xhatminus(k)

  % Step 1a-1: Create augmented SigmaX and xhat
  [sigmaXa,p] = chol(sigmax,'lower'); 
  if p>0
    fprintf('Cholesky error.  Recovering...\n');
    theAbsDiag = abs(diag(sigmax));
    sigmaXa = diag(max(SQRT(theAbsDiag),SQRT(sigmaw)));
  end

  sigmaXa=[real(sigmaXa) zeros([nx nw+nv]); zeros([nw+nv nx]) snoise];
  xhata = [xhat; zeros([nw+nv 1])];
  % NOTE: sigmaXa is lower-triangular

  % Step 1a-2: Calculate SigmaX points (strange indexing of xhata to 
  % avoid "repmat" call, which is very inefficient in MATLAB)
  Xa = xhata(:,ones([1 2*na+1])) + ...
       ht*[zeros([na 1]), sigmaXa, -sigmaXa];

  % Step 1a-3: Time update from last iteration until now
  %     stateEqn(xold,current,xnoise)
  % Xx = stateEqn(Xa(1:Nx,:),I,Xa(Nx+1:Nx+Nw,:)); 
  
    xold = Xa(1:nx,:);
    current = I;
    xnoise = Xa(nx+1:nx+nw,:);
    
    current = current + xnoise; % noise adds to current
    xnew = 0*xold;
    xnew(1,:) = RC*xold(1,:) + (1-diag(RC))*current;
    Ah = exp(-abs(current*G*dt/(3600*Q)));  % hysteresis factor
    xnew(2,:) = Ah.*xold(2,:) + (Ah-1).*sign(current);
    xnew(3,:) = xold(3,:) - current/3600/Q;
    xnew(2,:) = min(1,max(-1,xnew(2,:)));
    xnew(3,:) = min(1.05,max(-0.05,xnew(3,:)));
    
    Xx = xnew;
    
  xhat = Xx*wm;

  % Step 1b: Error covariance time update
  %          - Compute weighted covariance sigmaminus(k)
  %            (strange indexing of xhat to avoid "repmat" call)
  Xs = Xx - xhat(:,ones([1 2*na+1]));
  sigmax = Xs*diag(wc)*Xs';
  
  % Step 1c: Output estimate
  %          - Compute weighted output estimate yhat(k)
  I = ik; yk = vk;
  %Y = outputEqn(Xx,u,Xa(nx+nw+1:end,:),battery);
  
    ynoise = Xa(nx+nw+1:end,:);
    Y = getOCV_UKF(xhat(3,:), battery);
    Y = Y + M*xhat(2,:) + M0*signIk;
    Y = Y - R*xhat(1,:) - R0*I + ynoise(1,:);
    
    yhat = Y*wm;

  % Step 2a: Estimator gain matrix
  Ys = Y - yhat(:,ones([1 2*na+1]));
  SigmaXY = Xs*diag(wc)*Ys';
  SigmaY = Ys*diag(wc)*Ys';
  L = SigmaXY/SigmaY; 

  % Step 2b: State estimate measurement update
  r = yk - yhat; % residual.  Use to check for sensor errors...
  if r^2 > 100*SigmaY, L(:,1)=0.0; end 
  xhat = xhat + L*r; 
  xhat(3)=min(1.05,max(-0.05,xhat(3)));
  xhat(2) = min(1,max(-1,xhat(2)));

  % Step 2c: Error covariance measurement update
  sigmax = sigmax - L*SigmaY*L';
  [~,S,V] = svd(sigmax);
  HH = V*S*V';
  sigmax = (sigmax + sigmax' + HH + HH')/4; % Help maintain robustness
  
  % Q-bump code
  if r^2>4*SigmaY, % bad voltage estimate by 2-SigmaX, bump Q 
    fprintf('Bumping sigmax\n');
    sigmax(3,3) = sigmax(3,3)*qbump;
  end
  
    block.Dwork(1).Data(1)=xhat(3);
    block.Dwork(1).Data(2)=xhat(1);
    block.Dwork(1).Data(3)=xhat(2);
    block.Dwork(1).Data(4) = ik;
    block.Dwork(1).Data(5) = 3*sqrt(sigmax(3,3));
    block.Dwork(1).Data(6) = sum(yhat)/length(yhat);
    block.Dwork(1).Data(7) = signIk;
  
    block.Dwork(2).Data(1:3) = sigmax(1,:);
    block.Dwork(3).Data(1:3) = sigmax(2,:);
    block.Dwork(4).Data(1:3) = sigmax(3,:);
    
%endfunction

