function quadcopterDynamicsSFunctionMod(block)
setup(block);

function setup(block)

    block.NumInputPorts  = 5;
    block.NumOutputPorts = 9;

    % Set up the port properties to be inherited or dynamic.

    for i = 1:4; % These are the motor inputs
        block.InputPort(i).Dimensions        = 1;
        block.InputPort(i).DirectFeedthrough = false;
        block.InputPort(i).SamplingMode      = 'Sample';
    end

    % This is the disturbance input
    block.InputPort(5).Dimensions        = 6; % torques x,y,z; forces x,y,z.
    block.InputPort(5).DirectFeedthrough = false;
    block.InputPort(5).SamplingMode      = 'Sample';
    %------
    for i = 1:9;
        block.OutputPort(i).Dimensions       = 1;
        block.OutputPort(i).SamplingMode     = 'Sample';
    end

    block.NumDialogPrms = 2;

    block.NumContStates = 12;

    block.SampleTimes = [0 0];

    % -----------------------------------------------------------------
    % Options
    % -----------------------------------------------------------------
    block.SetAccelRunOnTLC(false);
    block.SimStateCompliance = 'DefaultSimState';

    block.RegBlockMethod('CheckParameters', @CheckPrms);
    block.RegBlockMethod('InitializeConditions', @InitializeConditions);
    block.RegBlockMethod('Outputs', @Outputs);
    block.RegBlockMethod('Derivatives', @Derivatives);


% -------------------------------------------------------------------
% Functions
% -------------------------------------------------------------------

function CheckPrms(block)
    quad   = block.DialogPrm(1).Data;
    IC     = block.DialogPrm(2).Data;


function InitializeConditions(block)
    % Initialize 12 States

    IC = block.DialogPrm(2).Data;

    % IC.P, IC.Q, IC.R are in deg/s ... convert to rad/s
    P = IC.P*pi/180; 
    Q = IC.Q*pi/180; 
    R = IC.R*pi/180; 

    % IC.Phi, IC.The, IC.Psi are in deg ... convert to rads
    Phi = IC.Phi*pi/180;
    The = IC.The*pi/180; 
    Psi = IC.Psi*pi/180;

    U = IC.U; V = IC.V; W = IC.W; 
    X = IC.X; Y = IC.Y; Z = IC.Z;

    init = [P,Q,R,Phi,The,Psi,U,V,W,X,Y,Z];
    for i=1:12
        block.ContStates.Data(i) = init(i);
    end
    
    out = [P,Q,R,Phi,The,Psi,X,Y,Z];
    for i=1:9
        block.OutputPort(i).Data = out(i);
    end


function Outputs(block)
    for i = 1:6
        block.OutputPort(i).Data = block.ContStates.Data(i);
    end
    
    for i = 7:9
        block.OutputPort(i).Data = block.ContStates.Data(i+3);
    end

function Derivatives(block)

    quad = block.DialogPrm(1).Data;

    % P Q R in units of rad/sec
    P = block.ContStates.Data(1);
    Q = block.ContStates.Data(2);
    R = block.ContStates.Data(3);

    % Phi The Psi in radians
    Phi = block.ContStates.Data(4);
    The = block.ContStates.Data(5);
    Psi = block.ContStates.Data(6);

    % U V W in units of m/s
    U = block.ContStates.Data(7);
    V = block.ContStates.Data(8);
    W = block.ContStates.Data(9);

    % X Y Z in units of m
    X = block.ContStates.Data(10);
    Y = block.ContStates.Data(11);
    Z = block.ContStates.Data(12);

    % w values in rev/min! NOT radians/s!!!!
    w1 = block.InputPort(1).Data;
    w2 = block.InputPort(2).Data;
    w3 = block.InputPort(3).Data;
    w4 = block.InputPort(4).Data;
    w  = [w1; w2; w3; w4];

    %------
    %Dist_tau = block.InputPort(5).Data(1:3);
    %Dist_F   = block.InputPort(5).Data(4:6);
    Dist_F   = block.InputPort(5).Data(1:3);
    
    % CALCULATE MOMENT AND THRUST FORCES

    % rpm constant
    KV = 1400;
    % per motor
    rpm = [0 0 0 0]; 
    % thrust = [0 0 0 0];
    % thrust2 = [0 0 0 0];
    thrust3 = [0 0 0 0];
    torque = [0 0 0 0];
    current = [0 0 0 0];

    for i=1:4
        % empirically derived equation given sample data
        rpm(i) = w(i);
        %coefficient of thrust, empirically derived
        coef_t = 2*(10^-15)*rpm(i)^3 - 4*(10^-11)*rpm(i)^2 + 3*(10^-7)*rpm(i) + .1013;
        %thrust = coef_t * density of air * velocity^2 * propeller diameter (m) ^4
        %     thrust(i) = coef_t*1.225*((2*pi*rpm(i)/60)^2)*(0.2^4);
        %     thrust2(i) = 4.18*(10^-5)*(rpm(i))^2;
        thrust3(i) = coef_t*1.225*((0.1016)^2)*(0.1016^2)*((2*pi*rpm(i)/60)^2);
        % empirically derived
        torque(i) = 4*(10^-14)*rpm(i)^3 +  8*(10^-12)*rpm(i)^2 + 3*(10^-6)*rpm(i);
        % current = rpm constant * torque
        current(i) = KV*torque(i);
    end

    Mb=[ -thrust3(1)+thrust3(2)+thrust3(3)-thrust3(4); ...
    -thrust3(1)-thrust3(2)+thrust3(3)+thrust3(4);...
    -torque(1)+torque(2)-torque(3)+torque(4)];

    %gyroscopic effect of rotation
    tau_motorGyro = [Q*quad.Jm*2*pi/60*(-w1-w3+w2+w4); P*quad.Jm*2*pi/60*(w1+w3-w2-w4); 0]; 
    Mb=Mb+tau_motorGyro;

    % Thrust due to motor speed
    % Force should be in units of Newtons for simplicity in calculating
    % the acceleration in the angular velocity state equation
    % Fb = [0; 0; sum(quad.ct*(w.^2))];   %[0, 0, sum(ct*w.^2)]'

    % my variant
    Fb = [0; 0; sum(thrust3)];   %[0, 0, sum(ct*w.^2)]'

    % Obtain dP dQ dR - angular accelerations
    omb_bi = [P; Q; R];
    OMb_bi = [ 0,-R, Q;
    R, 0,-P;
    -Q, P, 0];

    b_omdotb_bi = quad.Jbinv*(Mb-OMb_bi*quad.Jb*omb_bi);
    H_Phi = [1,tan(The)*sin(Phi), tan(The)*cos(Phi);
    0,         cos(Phi),         -sin(Phi);
    0,sin(Phi)/cos(The),cos(Phi)/cos(The)];   
    Phidot = H_Phi*omb_bi;

    % Compute Rotation Matrix
    % We use a Z-Y-X rotation
    Rib = [cos(Psi)*cos(The) cos(Psi)*sin(The)*sin(Phi)-sin(Psi)*cos(Phi) cos(Psi)*sin(The)*cos(Phi)+sin(Psi)*sin(Phi);
           sin(Psi)*cos(The) sin(Psi)*sin(The)*sin(Phi)+cos(Psi)*cos(Phi) sin(Psi)*sin(The)*cos(Phi)-cos(Psi)*sin(Phi);
          -sin(The)          cos(The)*sin(Phi)                            cos(The)*cos(Phi)];
    
    Rbi = Rib';
    ge = [0; 0; -quad.g];
    gb = Rbi*ge;
    Dist_Fb = Rbi*Dist_F;

    % Compute Velocity and Position derivatives of body frame
    vb = [U;V;W];
    b_dv = (1/quad.mass)*Fb+gb+Dist_Fb-OMb_bi*vb; % Acceleration in body frame (FOR VELOCITY)
    i_dp = Rib*vb; % Units OK SI: Velocity of body frame w.r.t inertia frame (FOR POSITION)

    dP = b_omdotb_bi(1);
    dQ = b_omdotb_bi(2);
    dR = b_omdotb_bi(3);
    dPhi = Phidot(1);
    dTheta = Phidot(2);
    dPsi = Phidot(3);
    dU = b_dv(1);
    dV = b_dv(2);
    dW = b_dv(3);
    dX = i_dp(1);
    dY = i_dp(2);
    dZ = i_dp(3);
    
    % impose a "ground" boundary
    if ((Z<=0) && (dZ<=0)) %
        dZ = 0;
        block.ContStates.Data(12) = 0;
    end
    
    %This is the state derivative vector
    block.Derivatives.Data = [dP dQ dR dPhi dTheta dPsi dU dV dW dX dY dZ].';


    %endfunction