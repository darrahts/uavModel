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
  block.SampleTimes = [.125 0];
  
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
  block.Dwork(1).Dimensions      = 5;
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
    %init = [.95, .05, .002];
    for i=1:3
        block.Dwork(1).Data(i) = init(i);
    end
    
    ukfBatteryParams = block.DialogPrm(3).Data;
    
    block.Dwork(2).Data(1:3) = ukfBatteryParams.sigma_x(1,:);
    block.Dwork(3).Data(1:3) = ukfBatteryParams.sigma_x(2,:);
    block.Dwork(4).Data(1:3) = ukfBatteryParams.sigma_x(3,:);
    
    % bounds
    block.Dwork(1).Data(4) = 0;
    
    % voltage
    block.Dwork(1).Data(5) = battery.v0;
    
    % gets the soc given the current voltage
    block.OutputPort(1).Data = find(battery.soc_ocv>=battery.v0,1);
  
%endfunction

function Output(block)
    % soc
    block.OutputPort(1).Data = block.Dwork(1).Data(1);
    
    % soc bounds
    block.OutputPort(2).Data = block.Dwork(1).Data(4);
    
    % voltage estimate
    block.OutputPort(3).Data = block.Dwork(1).Data(5);
  
%endfunction

function Update(block)
    % get block arguments
    battery           = block.DialogPrm(1).Data;
    dt                = block.DialogPrm(2).Data;
    ukfBatteryParams  = block.DialogPrm(3).Data;

    % parameters
    n  = battery.n;
    Q  = battery.Q;
    G  = battery.G;
    M0 = battery.M0;
    M  = battery.M;
    R0 = battery.R0;
    R  = battery.R;
    RC = exp(-dt/abs(battery.RC))';

    % input current and voltage
    voltage = block.InputPort(1).Data;
    current = block.InputPort(2).Data;

    if current<0
        current = current*n; 
    end

    % states
    z  = block.Dwork(1).Data(1);
    Ir = block.Dwork(1).Data(2);
    h  = block.Dwork(1).Data(3);

    x_hat = [Ir h z]';

    sigma_x = zeros(3,3);
    sigma_x(1,:) = block.Dwork(2).Data(1:3);
    sigma_x(2,:) = block.Dwork(3).Data(1:3);
    sigma_x(3,:) = block.Dwork(4).Data(1:3);

    % Get data stored in params structure
    sigma_w = ukfBatteryParams.sigma_w;
    sigma_noise = ukfBatteryParams.sigma_noise;
    bump_val  = ukfBatteryParams.bump_val;
    gamma  = ukfBatteryParams.gamma;

    x_len   = ukfBatteryParams.x_len;
    w_len   = ukfBatteryParams.w_len;
    v_len   = ukfBatteryParams.v_len;
    aux_len = ukfBatteryParams.aux_len;

    cov_weights = ukfBatteryParams.cov_weights;
    mu_weights  = ukfBatteryParams.mu_weights;

    u_sign = sign(current);

    % create augmented sigma_x and x_hat
    [sigma_x_aux,p] = chol(sigma_x,'lower'); 
    if p > 0
        fprintf('Cholesky error.  Recovering...\n');
        diag_sigma_x = abs(diag(sigma_x));
        sigma_x_aux = diag(max(SQRT(diag_sigma_x),SQRT(sigma_w)));
    end

    sigma_x_aux=[real(sigma_x_aux) zeros([x_len w_len+v_len]); zeros([w_len+v_len x_len]) sigma_noise];
    x_hat_aux = [x_hat; zeros([w_len+v_len 1])];
    % sigma_x_aux is lower-triangular

    % calculate sigma points
    aux_state_mat = x_hat_aux(:,ones([1 2*aux_len+1])) + ...
       gamma*[zeros([aux_len 1]), sigma_x_aux, -sigma_x_aux];

    % state update
    x_hat_pre = aux_state_mat(1:x_len,:);
    xnoise = aux_state_mat(x_len+1:x_len+w_len,:);
    current = current + xnoise;
    H_mat = exp(-abs(n*current*G*dt/(3600*Q))); 

    x_hat_post = 0*x_hat_pre;
    x_hat_post(1,:) = RC*x_hat_pre(1,:) + (1-diag(RC))*current; 
    x_hat_post(2,:) = H_mat.*x_hat_pre(2,:) + (H_mat-1).*sign(current);
    x_hat_post(3,:) = x_hat_pre(3,:) - n*current/3600/Q;

    x_hat_post(2,:) = min(1,max(-1,x_hat_post(2,:)));
    x_hat_post(3,:) = min(1.05,max(-0.05,x_hat_post(3,:)));

    x_hat = x_hat_post*mu_weights;

    % error cov update
    cov_pre = x_hat_post - x_hat(:,ones([1 2*aux_len+1]));
    sigma_x = cov_pre*diag(cov_weights)*cov_pre';

    % output estimate
    ynoise = aux_state_mat(x_len+w_len+1:end,:); 
    Y = getOCV_UKF(x_hat_post(3,:), battery) + M*x_hat(2,:) + M0*u_sign - R*x_hat(1,:) - R0*current + ynoise(1,:);

    y_hat = Y*mu_weights;

    % update kalman gain
    residual = Y - y_hat(:,ones([1 2*aux_len+1]));
    innov = cov_pre*diag(cov_weights)*residual';
    sigma_y = residual*diag(cov_weights)*residual';
    K = innov/sigma_y; 

    % measurement update
    r = voltage - y_hat; % residual.  Use to check for sensor errors...
    if r^2 > 100*sigma_y, K(:,1)=0.0; end 
    x_hat = x_hat + K*r; 
    x_hat(3)=min(1.05,max(-0.05,x_hat(3)));
    x_hat(2) = min(1,max(-1,x_hat(2)));

    % error cov measurement update, use hager method to ensure positive
    % semidefinite
    sigma_x = sigma_x - K*sigma_y*K';
    [~,S,V] = svd(sigma_x);
    HH = V*S*V';
    sigma_x = (sigma_x + sigma_x' + HH + HH')/4; % Help maintain robustness

    % Q-bump code
    if r^2>1*sigma_y, % bad voltage estimate by 2-SigmaX, bump Q 
        fprintf('Bumping sigmax\n');
        sigma_x(3,3) = sigma_x(3,3)*bump_val;
    end

    block.Dwork(1).Data(1)=x_hat(3);
    block.Dwork(1).Data(2)=x_hat(1);
    block.Dwork(1).Data(3)=x_hat(2);

    block.Dwork(1).Data(4) = 3*sqrt(sigma_x(3,3));
    block.Dwork(1).Data(5) = sum(y_hat)/length(y_hat);

    block.Dwork(2).Data(1:3) = sigma_x(1,:);
    block.Dwork(3).Data(1:3) = sigma_x(2,:);
    block.Dwork(4).Data(1:3) = sigma_x(3,:);

%endfunction

