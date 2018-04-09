% initial states

Va = 35;
gamma = 30*pi/180;
R = Inf;

DX0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va/R; 0; 0; 0];
IDX = [3; 4; 5; 6 ;7; 8; 9; 10; 11; 12];
X0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
IX0 = [];
U0 = [0; 0; 0; 1];
IU0 = [];
Y0 = [Va; gamma; 0];
IY0 = [1,3];

[X_trim,U_trim,Y,DX] = trim('mavsim_trim',X0, U0, Y0, IX0, IU0, IY0, DX0, IDX);

% compute linear state space equations
[A,B,C,D] = linmod('mavsim_trim', X_trim, U_trim);

E1_lat = [0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 0; 1 0 0 0 0; 0 0 0 0 0;...
      0 1 0 0 0; 0 0 0 0 0; 0 0 1 0 0; 0 0 0 1 0; 0 0 0 0 0; 0 0 0 0 1];

E2_lat = [0 0; 1 0; 0 1; 0 0];

E1_lon = [0 0 0 0 0; 0 0 0 0 0; 1 0 0 0 0; 0 1 0 0 0; 0 0 0 0 0; 0 0 1 0 0;...
      0 0 0 0 0; 0 0 0 1 0; 0 0 0 0 0; 0 0 0 0 0; 0 0 0 0 1; 0 0 0 0 0];

E2_lon = [1 0; 0 0; 0 0; 0 1];

A_lat = E1_lat'*A*E1_lat;
A_lon = E1_lon'*A*E1_lon;

B_lat = E1_lat'*B*E2_lat;
B_lon = E1_lon'*B*E2_lon;

P.va0 = Va;
P.pn0 = 0;
P.pe0 = 0;
P.pd0 = -100;
P.u0 = X_trim(4);
P.v0 = X_trim(5);
P.w0 = X_trim(6);
P.phi0 = X_trim(7);
P.th0 = X_trim(8);
P.psi0 = X_trim(9);
P.p0 = X_trim(10);
P.q0 = X_trim(11);
P.r0 = X_trim(12);
P.vg0 = sqrt(P.u0^2 + P.v0^2 + P.w0^2);
P.alpha0 = atan(P.w0/P.u0);
P.beta0 = asin(P.v0/P.va0);
P.chi0 = P.psi0 + P.beta0;

P.gamma_max = .707;

P.delta_e = U_trim(1);
P.delta_a = U_trim(2);
P.delta_r = U_trim(3);
P.delta_t = U_trim(4);

P.mass = 25;
P.g = 9.81;
P.Jx = 0.8244;
P.Jy = 1.135;
P.Jz = 1.759;
P.Jxz = 0.1204;
P.S = 0.55;
P.b = 2.8956;
P.c = 0.18994;
P.Sprop = 0.2027;
P.rho = 1.2682;
P.kmotor = 80;
P.kTp = 0;
P.komega = 0 ;
P.e = 0.9;
P.CL0 = 0.28;
P.CD0 = 0.03;
P.Cm0 = -0.02338;
P.CLalpha = 3.45;
P.CDalpha = 0.30;
P.Cmalpha = -0.38;
P.CLq = 0;
P.CDq = 0;
P.Cmq = -3.6;
P.CLdele = -0.36;
P.CDdele = 0;
P.Cmdele = -0.5;
P.Cprop = 1.0;
P.M = 50;
P.alpha0 = 0.4712;
P.eps = 0.1592;
P.CDp = 0.0437;
P.Cndelr = -0.032;
P.CY0 = 0;
P.Cl0 = 0;
P.Cn0 = 0;
P.CYbeta = -0.98;
P.Clbeta = -0.12;
P.Cnbeta = 0.25;
P.CYp = 0 ;
P.Clp = -0.26;
P.Cnp = 0.022;
P.CYr = 0;
P.Clr = 0.14;
P.Cnr = -0.35;
P.CYdela = 0;
P.Cldela = 0.08;
P.Cndela = 0.06;
P.CYdelr = -0.17;
P.Cldelr = 0.105;

% wind components
P.lu = 200;
P.lv = P.lu;
P.lw = 50;
P.Ts = 0.01;
P.sigmau = 0.0;
P.sigmav = 0.0;
P.sigmaw = 0.0;
P.wind_n = 0.0;
P.wind_e = 0.0;
P.wind_d = 0.0;

P.gamma = P.Jx*P.Jz-P.Jxz^2;
P.gamma1 = (P.Jxz*(P.Jx-P.Jy+P.Jz))/P.gamma;
P.gamma2 = (P.Jz*(P.Jz-P.Jy)+P.Jxz^2)/P.gamma;
P.gamma3 = P.Jz/P.gamma;
P.gamma4 = P.Jxz/P.gamma;
P.gamma5 = (P.Jz-P.Jx)/P.Jy;
P.gamma6 = P.Jxz/P.Jy;
P.gamma7 = ((P.Jx-P.Jy)*P.Jx+P.Jxz^2)/P.gamma;
P.gamma8 = P.Jx/P.gamma;

P.Cp0 = P.gamma3*P.Cl0 + P.gamma4*P.Cn0;
P.Cpbeta = P.gamma3*P.Clbeta + P.gamma4*P.Cnbeta;
P.Cpp = P.gamma3*P.Clp + P.gamma4*P.Cnp;
P.Cpr = P.gamma3*P.Clr + P.gamma4*P.Cnr;
P.Cpdela = P.gamma3*P.Cldela + P.gamma4*P.Cndela;
P.Cpdelr = P.gamma3*P.Cldelr + P.gamma4*P.Cndelr;
P.Cr0 = P.gamma4*P.Cl0 + P.gamma8*P.Cn0;
P.Crbeta = P.gamma4*P.Clbeta + P.gamma8*P.Cnbeta;
P.Crp = P.gamma*P.Clp + P.gamma8*P.Cnp;
P.Crr = P.gamma4*P.Clr + P.gamma8*P.Cnr;
P.Crdela = P.gamma4*P.Cldela + P.gamma8*P.Cndela;
P.Crdelr = P.gamma4*P.Cldelr + P.gamma8*P.Cndelr;

% parameters for sensors
P.P0 = 101325;
P.T0 = 288.15;
P.L0 = -0.0065;
P.R = 8.31432;
P.Mol = 0.0289644;
P.Ts_gps = 1;
P.Kgps = 1/1100;

P.bias_gyro_x = 0;        
P.bias_gyro_y = 0;
P.bias_gyro_z = 0;

% compute transfer functions 
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(X_trim,U_trim,P);

% values used for computing PID gains
a_phi1 = -0.5*P.rho*P.va0^2*P.S*P.b*P.Cpp*P.b/(2*P.va0);
a_phi2 = 0.5*P.rho*P.va0^2*P.S*P.b*P.Cpdela;

a_beta1 = -(P.rho*P.va0*P.S)/(2*P.mass)*P.CYbeta;
a_beta2 = (P.rho*P.va0*P.S)/(2*P.mass)*P.CYdelr;

a_theta1 = -(P.rho*P.va0^2*P.c*P.S)/(2*P.Jy)*P.Cmq*(P.c/(2*P.va0));
a_theta2 = -(P.rho*P.va0^2*P.c*P.S)/(2*P.Jy)*P.Cmalpha;
a_theta3 = (P.rho*P.va0^2*P.c*P.S)/(2*P.Jy)*P.Cmdele;

a_V1 = (P.rho*P.va0*P.S)/(P.mass)*(P.CD0 + P.CDalpha*P.alpha0 + P.CDdele*P.delta_e) + (P.rho*P.Sprop)/(P.mass)*P.Cprop*P.va0;
a_V2 = (P.rho*P.Sprop)/(P.mass)*P.Cprop*P.kmotor^2*P.delta_t;
a_V3 = P.g*cos(P.th0-P.chi0);

% low level autopilot gains
P.tau = 5;  % gain on dirty derivative
P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 10;
P.theta_c_max = 30*pi/180; % maximum pitch angle command
P.climb_out_trottle = 0.61;
P.phi_max = 45*pi/180;

% select gains for roll loop
    % get transfer function data for delta_a to phi
    [num,den]=tfdata(T_phi_delta_a,'v');
    a_phi2 = num(3);
    a_phi1 = den(2);
    % maximum possible aileron command
    delta_a_max = 45*pi/180;
    % Maximum anticipated roll error
    phi_max = 20*pi/180;
    % pick damping ratio for roll loop
    zeta_roll = 2;
    
    % set roll control gains based on zeta and phi_max
    P.roll_kp = delta_a_max/phi_max;
    wn_roll = sqrt(P.roll_kp*a_phi2);
    P.roll_kd = (2*zeta_roll*wn_roll - a_phi1)/a_phi2;
    %P.roll_kd = P.roll_kd+.2; % add extra roll damping
    P.roll_ki = 0;
    
% select gains for course loop
   zeta_course = 0.9;
   wn_course = wn_roll/8;
   P.course_kp = 2*zeta_course*wn_course*P.va0/P.g;
   P.course_ki = wn_course^2*P.va0/P.g;
   P.course_kd = 0;
   
% select gains for sideslip hold
    % get transfer function data for delta_r to vr
    [num,den]=tfdata(T_v_delta_r,'v');
    a_beta2 = num(2);
    a_beta1 = den(2);
    % maximum possible rudder command
    delta_r_max = 20*pi/180;
    % Roll command when delta_r_max is achieved
    vr_max = 3;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_beta = 0.707;
    P.beta_kp = delta_r_max/vr_max;
    wn_beta = (a_beta2*P.beta_kp+a_beta1)/2/zeta_beta;
    P.beta_ki = 0;%wn_beta^2/a_beta2;
    P.beta_kd = 0;

   
% select gains for the pitch loop
   % get transfer function delta_e to theta
   [num,den]=tfdata(T_theta_delta_e,'v');
   a_theta1 = den(2);
   a_theta2 = den(3);
   a_theta3 = num(3);
   % maximum possible elevator command
   delta_e_max = 45*pi/180;
   % Pitch command when delta_e_max is achieved
   theta_max = 10*pi/180;
   % pick natural frequency to achieve delta_e_max for step of theta_max
   zeta_pitch = 0.9;%0.9;
   % set control gains based on zeta and wn
   P.pitch_kp = -delta_e_max/theta_max;
   wn_pitch = sqrt(a_theta2+P.pitch_kp*a_theta3);
   P.pitch_kd = (2*zeta_pitch*wn_pitch - a_theta1)/a_theta3;
   P.pitch_ki = 0.0;
   P.K_theta_DC = P.pitch_kp*a_theta3/(a_theta2+P.pitch_kp*a_theta3);

% select gains for altitude loop
   zeta_altitude = .9;%.707;
   wn_altitude = wn_pitch/40;
   P.altitude_kp = 2*zeta_altitude*wn_altitude/P.K_theta_DC/P.va0;
   P.altitude_ki = wn_altitude^2/P.K_theta_DC/P.va0;
%    P.altitude_kp = 0.0114;
%    P.altitude_ki = 0.0039;
   P.altitude_kd = 0;%-.001;
 
% airspeed hold using pitch
   [num,den]=tfdata(T_Va_theta,'v');
   a_V1 = den(2);
   zeta_airspeed_pitch = .10;%0.707;1;
   wn_airspeed_pitch = wn_pitch/10;
   P.airspeed_pitch_kp = (a_V1-2*zeta_airspeed_pitch*wn_airspeed_pitch)/P.K_theta_DC/P.g;
   P.airspeed_pitch_ki = -wn_airspeed_pitch^2/P.K_theta_DC/P.g;
 
% airspeed hold using throttle
   [num,den]=tfdata(T_Va_delta_t,'v');
   a_Vt1 = den(2);
   a_Vt2 = num(2);
   zeta_airspeed_throttle = 2;%0.707;
%    wn_airspeed_throttle = 5;   % a value of 5 causes instability...
   wn_airspeed_throttle = 3;
   P.airspeed_throttle_kp = (2*zeta_airspeed_throttle*wn_airspeed_throttle-a_Vt1)/a_Vt2;
   P.airspeed_throttle_ki = wn_airspeed_throttle^2/a_Vt2;
%   P.airspeed_throttle_integrator_gain = a_Vt1/a_Vt2/P.airspeed_throttle_ki;
 
% gains for slideslip
   P.sideslip_kp = .1;
   P.sideslip_kd = -.5;
   P.sideslip_ki = 0;
 
% TECS gains
    % throttle (unitless)
    P.TECS_E_kp = 1;
    P.TECS_E_ki = .5;
  
    % pitch command (unitless)
    P.TECS_L_kp = 1;
    P.TECS_L_ki = .1;
    
    % saturated altitude error
    P.TECS_h_error_max = 10; % meters
    
    
    
% simplified version gains
P.b_chidot = 4;
P.b_chi = 1;
P.b_phi = 1;
P.b_hdot = 2*0.7;
P.b_h = 0.7^2;
P.b_Va = 2;

% need to add the following to your parameter file:
 
% chapter 11 - path manager
% number of waypoints in data structure
P.size_waypoint_array = 100;
P.R_min = P.va0^2/P.g/tan(P.phi_max);

% create random city map
city_width      = 2000;  % the city is of size (width)x(width)
building_height = 1;   % maximum height of buildings
%building_height = 1;   % maximum height of buildings (for camera)
num_blocks      = 5;    % number of blocks in city
street_width    = .8;   % percent of block that is street.
P.pd0           = P.pd0;  % initial height of MAV
P.map = createWorld(city_width, building_height, num_blocks, street_width);


% target parameters
P.target_velocity = 5;  % (m/s)
P.target_size = 2;          % size of target 


% gimbal parameters
P.az0 = 0;      % initial azimuth angle
P.el0 = -pi/2;  % initial elevation angle (pointing down)
P.az_limit = 180*(pi/180);  % azimuth angle limit
P.el_limit = 180*(pi/180);  % elevation angle limit
P.az_gain  = 1;  % gain on azimuth dynamics (azdot = az_gain*u_az)
P.el_gain  = 1;  % gain on elevation dynamics (eldot = el_gain*u_el)
P.k_az     = 10; % proportional control gain for gimbal azimuth
P.k_el     = 10; % proportional control gain for gimbal elevation

% camera parameters
P.cam_fps = 10;  % frames per second 
P.cam_pix = 480;                      % size of (square) pixel array
P.cam_fov   = 10*(pi/180);            % field of view of camera
P.f = (P.cam_pix/2)/tan(P.cam_fov/2); % focal range
P.pixelnoise = 0;                     % (pixels) - variance of the pixel noise

% measurement model for GPS (used in geolocation algorithm)
P.sigma_measurement_n = 0.547; % standard deviation of gps measurement error in m
P.sigma_measurement_e = 0.547; % standard deviation of gps measurement error in m
P.sigma_measurement_h = 1.14; % standard deviation of gps measurement error in m

