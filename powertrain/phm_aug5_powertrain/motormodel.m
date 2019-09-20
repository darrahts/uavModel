
%Parameters of the simplified BLDCM
d= 1.6e-08; % viscous damping coefficient or dynamic Friction coefficient
K_e= 0.0068; %back electromotive force constant
T_fric=1e-08; % static friction torque
R=0.2; % quivalent electric resistance of the coils
Jm=4.9e-06; % inertia 

%nonlinear curve representing the relationship between load and angular velocity in discrete time
%(60/(2*pi)) from rad/s to rpm 
%From T_load=a*(w)^3 + b*(w)^2 + c*w;
a=-(4e-14*(60/(2*pi))^3)/Jm; 
b=-(8e-12*(60/(2*pi))^2)/Jm;
c=-(3e-6*(60/(2*pi)))/Jm;

%linear continuous model
A=-(d + (K_e^2/R))/Jm;
B=K_e/(R*Jm);
C=1;
D=0;

maxIter = 2500; 
u = 0; 
x = 0;

for k = 1:maxIter
    % EKF Step 1a: State-prediction time update    
    tspan = [0 .0001]; %to simulate the continuous system for one second
    [t,y]=ode45(@(t,x) sysmot(t,x,A,B,a,b,c,u),tspan,x);
    x=y(end);
    
    % input signal u
    u = 0.0115*k+1; % just some interesting input
    if u >= 11.4
        u = 11.4;
    end
    store(k) = x * (60 / (2*pi)); %multiply by (60/(2*pi)) to achieve rpm
end

plot(store);

function xdot=sysmot(t,x,A,B,a,b,c,u)
 xdot =(a*x^3+b*x^2+(c+A)*x)+B*u;
end