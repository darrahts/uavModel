function [sys,x0,str,ts,simStateCompliance] = s_func_test(t,x,u,flag)
%SFUNTMPL General MATLAB S-Function Template
%   With MATLAB S-functions, you can define you own ordinary differential
%   equations (ODEs), discrete system equations, and/or just about
%   any type of algorithm to be used within a Simulink block diagram.
%
%   The general form of an MATLAB S-function syntax is:
%       [SYS,X0,STR,TS,SIMSTATECOMPLIANCE] = SFUNC(T,X,U,FLAG,P1,...,Pn)
%
%   What is returned by SFUNC at a given point in time, T, depends on the
%   value of the FLAG, the current state vector, X, and the current
%   input vector, U.
%
%   FLAG   RESULT             DESCRIPTION
%   -----  ------             --------------------------------------------
%   0      [SIZES,X0,STR,TS]  Initialization, return system sizes in SYS,
%                             initial state in X0, state ordering strings
%                             in STR, and sample times in TS.
%   1      DX                 Return continuous state derivatives in SYS.
%   2      DS                 Update discrete states SYS = X(n+1)
%   3      Y                  Return outputs in SYS.
%   4      TNEXT              Return next time hit for variable step sample
%                             time in SYS.
%   5                         Reserved for future (root finding).
%   9      []                 Termination, perform any cleanup SYS=[].
%
%
%   The state vectors, X and X0 consists of continuous states followed
%   by discrete states.
%
%   Optional parameters, P1,...,Pn can be provided to the S-function and
%   used during any FLAG operation.
%
%   When SFUNC is called with FLAG = 0, the following information
%   should be returned:
%
%      SYS(1) = Number of continuous states.
%      SYS(2) = Number of discrete states.
%      SYS(3) = Number of outputs.
%      SYS(4) = Number of inputs.
%               Any of the first four elements in SYS can be specified
%               as -1 indicating that they are dynamically sized. The
%               actual length for all other flags will be equal to the
%               length of the input, U.
%      SYS(5) = Reserved for root finding. Must be zero.
%      SYS(6) = Direct feedthrough flag (1=yes, 0=no). The s-function
%               has direct feedthrough if U is used during the FLAG=3
%               call. Setting this to 0 is akin to making a promise that
%               U will not be used during FLAG=3. If you break the promise
%               then unpredictable results will occur.
%      SYS(7) = Number of sample times. This is the number of rows in TS.
%
%
%      X0     = Initial state conditions or [] if no states.
%
%      STR    = State ordering strings which is generally specified as [].
%
%      TS     = An m-by-2 matrix containing the sample time
%               (period, offset) information. Where m = number of sample
%               times. The ordering of the sample times must be:
%
%               TS = [0      0,      : Continuous sample time.
%                     0      1,      : Continuous, but fixed in minor step
%                                      sample time.
%                     PERIOD OFFSET, : Discrete sample time where
%                                      PERIOD > 0 & OFFSET < PERIOD.
%                     -2     0];     : Variable step discrete sample time
%                                      where FLAG=4 is used to get time of
%                                      next hit.
%
%               There can be more than one sample time providing
%               they are ordered such that they are monotonically
%               increasing. Only the needed sample times should be
%               specified in TS. When specifying more than one
%               sample time, you must check for sample hits explicitly by
%               seeing if
%                  abs(round((T-OFFSET)/PERIOD) - (T-OFFSET)/PERIOD)
%               is within a specified tolerance, generally 1e-8. This
%               tolerance is dependent upon your model's sampling times
%               and simulation time.
%
%               You can also specify that the sample time of the S-function
%               is inherited from the driving block. For functions which
%               change during minor steps, this is done by
%               specifying SYS(7) = 1 and TS = [-1 0]. For functions which
%               are held during minor steps, this is done by specifying
%               SYS(7) = 1 and TS = [-1 1].
%
%      SIMSTATECOMPLIANCE = Specifices how to handle this block when saving and
%                           restoring the complete simulation state of the
%                           model. The allowed values are: 'DefaultSimState',
%                           'HasNoSimState' or 'DisallowSimState'. If this value
%                           is not speficified, then the block's compliance with
%                           simState feature is set to 'UknownSimState'.


%   Copyright 1990-2010 The MathWorks, Inc.

%
% The following outlines the general structure of an S-function.
%
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 0; 
sizes.NumDiscStates  = 3; %z, Ir, h
sizes.NumOutputs     = 2; %v, soc
sizes.NumInputs      = 1; % Io
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

Q    =  3800; % total cell capacity in mAh
n    =  .9929; % coulombic efficiency
M    =  0; % dynamic hysteresis volts factor
Mo   =  0; % instant hysteresis volts factor
Ro   =  .0112; % internal resistance
R    =  .1; % diffusion / warburg impedence, pairs with C
C    =  250; % diffusion / warburg impedence, pairs with R
g    =  .1199;  % gamma, tunes decay in hysteresis
dt   =  1/36; % delta t
eod  =  3.04; % final voltage

if length(R) ~= length(C)
    error("R and C must be the same length");
end

Arc = zeros(length(R));
for i = 1:length(R)
    Arc(i,i) = exp(-dt/(R(i)*C(i)));
end
Brc = zeros(length(R), 1);
for i = 1:length(R)
    Brc(i,1) = 1 - exp(-dt/(R(i)*C(i)));
end

% initialize the H matrix
H(1) = exp(-abs((n(1)*.1*g*dt)/Q));

Io = 0.0;

z = 1.0; % soc
Ir = zeros(length(R), 1);
h = 0.0;

v = 4.2;
soc = z;

%
% initialize the initial conditions
%
x0  = [z Ir h];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u)

sys = [];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

Q    =  3800; % total cell capacity in mAh
n    =  .9929; % coulombic efficiency
R    =  .1; % diffusion / warburg impedence, pairs with C
C    =  250; % diffusion / warburg impedence, pairs with R
g    =  .1199;  % gamma, tunes decay in hysteresis
dt   =  .001; % delta t
eod  =  3.04; % final voltage

if length(R) ~= length(C)
    error("R and C must be the same length");
end

Arc = zeros(length(R));
for i = 1:length(R)
    Arc(i,i) = exp(-dt/(R(i)*C(i)));
end
Brc = zeros(length(R), 1);
for i = 1:length(R)
    Brc(i,1) = 1 - exp(-dt/(R(i)*C(i)));
end

% initialize the H matrix
H(1) = exp(-abs((n(1)*.1*g*dt)/Q));

z_dot = z - ((n*dt)/Q)*current;
Ir_dot = Arc*Ir + Brc*current;
H = exp(-abs((n*current*g*dt)/Q));
h_dot = H*h + (H-1)*sign(current);

z = z_dot;
Ir = Ir_dot;
h = h_dot;

sys = [z Ir h];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)
soc_ocv = [3.04,3.101,3.1589,3.2115,3.2812,3.34998,3.39958,3.46048,3.51074,3.580156,3.649925,3.7,3.75,3.78,3.81,3.84,3.86,3.88,3.9,3.92,3.94,3.95,3.96,3.97,3.98,3.988,3.995,4,4.007,4.015,4.02,4.023,4.027,4.035,4.04,4.042,4.043,4.045,4.047,4.048,4.049,4.049,4.05,4.0505,4.051,4.0515,4.052,4.0521,4.0529,4.0532,4.0539,4.0541,4.0547,4.055,4.056,4.0565,4.0569,4.0575,4.0578,4.058,4.0581,4.0582,4.0583,4.0587,4.0589,4.059,4.0591,4.0592,4.0593,4.0594,4.0595,4.0596,4.0597,4.0598,4.06,4.0605,4.061,4.062,4.064,4.065,4.068,4.07,4.071,4.0715,4.073,4.077,4.085,4.09,4.094,4.098,4.103,4.112,4.13,4.14,4.15,4.16,4.17,4.18,4.19,4.2,4.2];
M    =  0; % dynamic hysteresis volts factor
Mo   =  0; % instant hysteresis volts factor
Ro   =  .0112; % internal resistance
R    =  .1; % diffusion / warburg impedence, 

soc = (x(1) + 31.8816) /(1.0 + 31.8816);
idx = ceil(soc*100) +1;
v = (soc_ocv(idx) + Mo*sign(u) + M*x(2) - sum(R*x(2)) - Ro*u);

sys = [soc v];

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate
