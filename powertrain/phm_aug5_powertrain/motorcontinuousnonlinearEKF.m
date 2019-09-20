% clear;
%Parameters of the simplified BLDCM
Df= 1.6e-08; % viscous damping coefficient or dynamic Friction coefficient
Ke= 0.0068; %back electromotive force constant
Tf =1e-08; % static friction torque
R=0.8; % quivalent electric resistance of the coils
Jm=4.9e-06; % inertia 

%nonlinear curve representing the relationship between load and angular velocity in discrete time
%(60/(2*pi)) from rad/s to rpm 
%From T_load=a*(w)^3 + b*(w)^2 + c*w;
a=-(4e-14*(60/(2*pi))^3)/Jm; 
b=-(8e-12*(60/(2*pi))^2)/Jm;
c=-(3e-6*(60/(2*pi)))/Jm;

%parameters of the linear continuous model
A=-(Df + (Ke^2/R))/Jm;
B=Ke/(R*Jm);
C=1;
D=0;

% ekf equations
Chat=1;
Bhat=1;
Dhat=1;

%simulation variables
SigmaW = 1e-5; % Process-noise covariance
SigmaV = 10; % Sensor-noise covariance
SigmaX = 0; % Initialize Kalman filter covariance

u = 1; % Initial driving input, u[0]
xtrue = 0; % Initialize true system initial state
xhat = 0; % Initialize Kalman filter initial estimate

% Reserve storage for variables we might want to plot/evaluate
maxIter = 900; % Number of simulation time steps to go from 1 V to 11.4 V
xstore = zeros(maxIter+1,length(xtrue)); xstore(1,:) = xtrue;
xhatstore = zeros(maxIter,length(xhat));
SigmaXstore = zeros(maxIter,length(xhat)^2);

%running the Extended Kalman Filter
for k = 1:maxIter
    % EKF Step 1a: State-prediction time update    
    tspan = [0 0.5 1]; %to simulate the continuous system for one second
    [t,y]=ode45(@(t,xhat) sysmot(t,xhat,A,B,a,b,c,u,w),tspan,xhat);
    [t,y2]=ode45(@(t,xhat) sysmothat(t,xhat,A,B,a,b,c),tspan,xhat);
    xhat=y(end);    
    Ahat=y(end);
    
    % EKF Step 1b: Error-covariance time update
    SigmaX = Ahat*SigmaX*Ahat' + Bhat*SigmaW*Bhat';
    
    % input signal u
    u = 0.0115*k+1; % just some interesting input
   
    w = chol(SigmaW,'lower')*randn(length(xtrue)); % rand. process noise
    v = chol(SigmaV,'lower')*randn(length(C*xtrue)); % rand. sensor noise
   
    ytrue = C*xhat + D*u; % based on present x and u    
    [t,y]=ode45(@(t,xtrue) sysmotnoise(t,xtrue,A,B,a,b,c,u,w),tspan,xtrue);
    xtrue = y(end); % future x is based on present u
    
    % KF Step 1c: Estimate system output
    yhat = Chat*xhat + D*u + v;
    
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
    vv{k} = v;
    ww{k} = w;
    xx{k} = SigmaX;
    
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

function xdot=sysmot(t,x,A,B,a,b,c,u,w)
 xdot =(a*x^3+b*x^2+(c+A)*x)+B*u+w;
end

function xdot=sysmotnoise(t,x,A,B,a,b,c,u,w)
 xdot =(a*x^3+b*x^2+(c+A)*x)+B*u+w;
end

function xdot=sysmothat(t,x,A,B,a,b,c)
xdot=3*a*x^2 + 2*b*x +(c+A);
end

