clear;
%Parameters of the simplified BLDCM
d= 1.6e-08; % viscous damping coefficient or dynamic Friction coefficient
K_e= 0.0068; %back electromotive force constant
T_fric=1e-08; % static friction torque
R=0.2; % quivalent electric resistance of the coils
Jm=4.9e-06; % inertia 

%linear continuous model
A=-(d + (K_e^2/R))/Jm;
B=K_e/(R*Jm);
C=1;
D=0;

%transforming to linear discrete model with Ts=1 second
Ad = expm(A); 
Bd = A\(Ad-1 )*B; 
Cd = C; 
Dd = D; 

%simulation variables
SigmaW = 1e-5; % Process-noise covariance
SigmaV = 0.1; % Sensor-noise covariance

xtrue = 150; % Initialize true system initial state
xhat = 150; % Initialize Kalman filter initial estimate
SigmaX = 0; % Initialize Kalman filter covariance
u = 1; % Initial driving input, u[0]
% Reserve storage for variables we might want to plot/evaluate
maxIter = 900; % Number of simulation time steps
xstore = zeros(maxIter+1,length(xtrue)); xstore(1,:) = xtrue;
xhatstore = zeros(maxIter,length(xhat));
SigmaXstore = zeros(maxIter,length(xhat)^2);

%running the linear Kalman Filter
for k = 1:maxIter
    % KF Step 1a: State-prediction time update
    xhat = Ad*xhat + Bd*u;    
    
    % KF Step 1b: Error-covariance time update
    SigmaX = Ad*SigmaX*Ad' + SigmaW;
    % [Implied operation of system in background, with
    % input signal u, and output signal z]
%     u = 5+randn(1); % just some interesting input
    u = 0.0115*k+1; % just some interesting input
   
    w = chol(SigmaW,'lower')*randn(length(xtrue)); % rand. process noise
    v = chol(SigmaV,'lower')*randn(length(C*xtrue)); % rand. sensor noise
   
    ytrue = C*xtrue + D*u + v; % based on present x and u
    
    xtrue = Ad*xtrue + Bd*u+ w; % future x is based on present u
    
    % KF Step 1c: Estimate system output
    yhat = C*xhat + D*u;
    % KF Step 2a: Compute Kalman gain matrix
    SigmaY = C*SigmaX*C' + SigmaV;
    L = SigmaX*C'/SigmaY;
    % KF Step 2b: State-estimate measurement update
    xhat = xhat + L*(ytrue - yhat);
    % KF Step 2c: Error-covariance measurement update
    SigmaX = SigmaX - L*SigmaY*L';
    % [Store information for evaluation/plotting purposes]
    xstore(k+1,:) = xtrue; %multiply by (60/(2*pi)) to achieve rpm
    xhatstore(k,:) = xhat; %multiply by (60/(2*pi)) to achieve rpm
    SigmaXstore(k,:) = SigmaX(:);
end

figure; clf;
plot(0:maxIter-1,xstore(1:maxIter),'k-',0:maxIter-1,xhatstore,'b--', ...
    0:maxIter-1,xhatstore+3*sqrt(SigmaXstore),'m-.',...
    0:maxIter-1,xhatstore-3*sqrt(SigmaXstore),'m-.'); grid;
legend('True','Estimate','Bounds','location','northeast');
title('Kalman filter example');
xlabel('Iteration'); ylabel('State');
xlim([1 maxIter])
figure; clf;
plot(0:maxIter-1,xstore(1:maxIter)-xhatstore,'b-',0:maxIter-1, ...
    3*sqrt(SigmaXstore),'m--',0:maxIter-1,-3*sqrt(SigmaXstore),'m--');
grid; legend('Error','Bounds','location','northeast');
title('Error with bounds');
xlabel('Iteration'); ylabel('Estimation error');
xlim([1 maxIter])

