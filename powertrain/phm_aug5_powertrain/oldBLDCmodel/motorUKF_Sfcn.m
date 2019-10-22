function batteryUKF(block)
% Level-2 MATLAB file S-Function for inherited sample time demo.
%   Copyright 1990-2009 The MathWorks, Inc.

  setup(block);
  
%endfunction

function setup(block)
  
  %% Register number of input and output ports
  block.NumInputPorts  = 2; % voltage, friction
  block.NumOutputPorts = 4; % rpm, current, torque, bounds

  %% Setup functional port properties to dynamically
  %% inherited.
  block.SetPreCompInpPortInfoToDynamic;
  block.SetPreCompOutPortInfoToDynamic;
 
  % voltage input
  block.InputPort(1).Dimensions        = 1;
  block.InputPort(1).DirectFeedthrough = false;
  block.InputPort(1).SamplingMode      = 'Sample';
  
  % friction input
  block.InputPort(2).Dimensions        = 1;
  block.InputPort(2).DirectFeedthrough = false;
  block.InputPort(2).SamplingMode      = 'Sample';
  
  % rpm
  block.OutputPort(1).Dimensions       = 1;
  block.OutputPort(1).SamplingMode     = 'Sample';
  
  % current
  block.OutputPort(2).Dimensions       = 1;
  block.OutputPort(2).SamplingMode     = 'Sample';
  
  % torque
  block.OutputPort(3).Dimensions       = 1;
  block.OutputPort(3).SamplingMode     = 'Sample';
    
  % estimation bounds
  block.OutputPort(4).Dimensions       = 1;
  block.OutputPort(4).SamplingMode     = 'Sample';

  block.NumDialogPrms     = 2; % motor params, ukf params
  
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
  block.NumDworks = 3;
  
  block.Dwork(1).Name = 'x0'; 
  block.Dwork(1).Dimensions      = 3; % omega, current, bounds
  block.Dwork(1).DatatypeID      = 0;
  block.Dwork(1).Complexity      = 'Real';
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name = 'sigmax1'; 
  block.Dwork(2).Dimensions      = 2;
  block.Dwork(2).DatatypeID      = 0;
  block.Dwork(2).Complexity      = 'Real';
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name = 'sigmax2'; 
  block.Dwork(3).Dimensions      = 2;
  block.Dwork(3).DatatypeID      = 0;
  block.Dwork(3).Complexity      = 'Real';
  block.Dwork(3).UsedAsDiscState = true;

  
%endfunction

function InitConditions(block)
    %% Initialize Dwork
    % Initialize 3 states and outputs
    ukfMotorParams = block.DialogPrm(2).Data;

    % omega
    block.Dwork(1).Data(1) = 0;
    
    % current
    block.Dwork(1).Data(2) = 0;
    
    % bounds
    block.Dwork(1).Data(3) = 0;
    
    block.Dwork(2).Data(1:2) = ukfMotorParams.sigma_x(1,:);
    block.Dwork(3).Data(1:2) = ukfMotorParams.sigma_x(2,:);

    % initialize outputs (rpm, current, torque, bounds)
    block.OutputPort(1).Data = 0;
    block.OutputPort(2).Data = 0;
    block.OutputPort(3).Data = 0;
    block.OutputPort(4).Data = 0;
  
%endfunction

function Output(block)

    rpm = block.Dwork(1).Data(1) * 60/(2*pi);
    
    % rpm
    block.OutputPort(1).Data = rpm;
    
    % current
    block.OutputPort(2).Data = block.Dwork(1).Data(2);
    
    % torque
    block.OutputPort(3).Data = 4*(10^-14)*rpm^3 +  8*(10^-12)*rpm^2 + 3*(10^-6)*rpm;
 
    % bounds
    block.OutputPort(4).Data = block.Dwork(1).Data(3);
    
%endfunction

function Update(block)
    % get block arguments
    motor           = block.DialogPrm(1).Data;
    ukfMotorParams  = block.DialogPrm(2).Data;

    % parameters
    R  = motor.R;
    Ke = motor.Ke;
    Tf = motor.Tf;
    Df = motor.Df;
    d  = motor.d;
    j  = motor.j;

    % input voltage and friction
    voltage  = block.InputPort(1).Data;
    friction = block.InputPort(2).Data;

    % states
    omega   = block.Dwork(1).Data(1);
    current = block.Dwork(1).Data(2);
    
    x_hat = [omega current]';

    sigma_x = zeros(2,2);
    sigma_x(1,:) = block.Dwork(2).Data(1:2);
    sigma_x(2,:) = block.Dwork(3).Data(1:2);

    % Get data stored in params structure
    sigma_w     = ukfMotorParams.sigma_w;
    sigma_v     = ukfMotorParams.sigma_v;
    bump_val    = ukfMotorParams.bump_val;
    gamma       = ukfMotorParams.gamma;

    x_len       = ukfMotorParams.x_len;
    w_len       = ukfMotorParams.w_len;
    v_len       = ukfMotorParams.v_len;
    aux_len     = ukfMotorParams.aux_len;

    cov_weights = ukfMotorParams.cov_weights;
    mu_weights  = ukfMotorParams.mu_weights;
    
    sigma_noise = real(chol(diag([sigma_w; sigma_v]),'lower'));

    % create augmented sigma_x and x_hat
    [sigma_x_aux,p] = chol(sigma_x,'lower'); 
    if p > 0
        fprintf('Cholesky error.  Recovering...\n');
        diag_sigma_x = abs(diag(sigma_x));
        sigma_x_aux  = diag(max(SQRT(diag_sigma_x),SQRT(sigma_w)));
    end

    sigma_x_aux = [real(sigma_x_aux) zeros([x_len w_len+v_len]); zeros([w_len+v_len x_len]) sigma_noise];
    x_hat_aux   = [x_hat; zeros([w_len+v_len 1])];
    % sigma_x_aux is lower-triangular

    % calculate sigma points
    aux_state_mat = x_hat_aux(:,ones([1 2*aux_len+1])) + ...
       gamma*[zeros([aux_len 1]), sigma_x_aux, -sigma_x_aux];

    % state update
    x_hat_pre = aux_state_mat(1:x_len,:);
    xnoise    = aux_state_mat(x_len+1:x_len+w_len,:);
    voltage   = voltage + xnoise;
    
    x_hat_post = 0*x_hat_pre;
    %
    %
    %   STATE FUNCTION
    %   todo: implement state function here
    %
    %
    x_hat_post(1,:) = 1.23; % omega
    x_hat_post(2,:) = 4.56; % current

    x_hat = x_hat_post*mu_weights;

    % error cov update
    cov_pre = x_hat_post - x_hat(:,ones([1 2*aux_len+1]));
    sigma_x = cov_pre*diag(cov_weights)*cov_pre';

    % output estimate
    ynoise = aux_state_mat(x_len+w_len+1:end,:); 
    %
    %
    %   OUTPUT FUNCTION
    %   todo: add output function here
    %
    %
    Y = 1.23;

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
%    [~,S,V] = svd(sigma_x);
%    HH = V*S*V';
%    sigma_x = (sigma_x + sigma_x' + HH + HH')/4; % Help maintain robustness

    % Q-bump code
    if r^2>1*sigma_y, % bad voltage estimate by 2-SigmaX, bump Q 
        fprintf('Bumping sigmax\n');
        sigma_x(2,2) = sigma_x(2,2)*bump_val;
    end

    block.Dwork(1).Data(1) = x_hat(1); % omega
    block.Dwork(1).Data(2) = x_hat(2); % current
    block.Dwork(1).Data(3) = 3*sqrt(sigma_x(2,2)); % bounds

    block.Dwork(2).Data(1:2) = sigma_x(1,:);
    block.Dwork(3).Data(1:2) = sigma_x(2,:);

%endfunction

