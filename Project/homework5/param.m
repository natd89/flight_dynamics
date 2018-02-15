% initial states

Va = 17;
gamma = 0*pi/180;
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

P.delta_e = U_trim(1);
P.delta_a = U_trim(2);
P.delta_r = U_trim(3);
P.delta_t = U_trim(4);

P.mass = 13.5;
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
P.Cmq = -0.5;
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
P.Ts = 0.005;
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

% kd_phi and kp_phi parameters
e_phi_max = 15*pi/180;
delta_a_max = 45*pi/180;
om_n_phi = sqrt(abs(a_phi2)*delta_a_max/e_phi_max);
zeta_phi = 1.0; % tune this parameter
P.kp_phi = delta_a_max/e_phi_max*sign(a_phi2);
P.kd_phi = (2*zeta_phi*om_n_phi-a_phi1)/(a_phi2);

% kp_chi and ki_chi parameters
W_chi = 5; % design parameter usually bigger than 5
om_n_chi = 1/W_chi*om_n_phi;
zeta_chi = 1; % tune this parameters
P.kp_chi = 2*zeta_chi*om_n_chi*P.vg0/P.g;
P.ki_chi = om_n_chi^2*P.vg0/P.g;

% kp_theta and kd_theta parameters
delta_e_max = 45*pi/180;
e_theta_max = 10*pi/180;
om_n_theta = sqrt(a_theta2 + delta_e_max/e_theta_max*abs(a_theta3));
zeta_theta = 1.0; % tune this parameter
P.kp_theta = delta_e_max/e_theta_max*sign(a_theta3);
P.kd_theta = (2*zeta_theta*om_n_theta-a_theta1)/a_theta3;
K_theta_DC = P.kp_theta*a_theta3/(a_theta2 + P.kp_theta*a_theta3);

% kp_h and ki_h parameters
W_h = 10; % usually between 5 and 15
om_n_h = 1/W_h*om_n_theta;
zeta_h = 1.0; % tune this parameter
P.ki_h = om_n_h^2/(K_theta_DC*P.va0);
P.kp_h = (2*zeta_h*om_n_h)/(K_theta_DC*P.va0);

% kp_v2 and ki_v2
W_v2 = 5; % tune this parameter
om_n_v2 = 1/W_v2*om_n_theta;
zeta_v2 = 1; % tune this parameter
P.ki_v2 = om_n_v2^2/(K_theta_DC*P.g);
P.kp_v2 = (a_V1-2*zeta_v2*om_n_v2)/(K_theta_DC*P.g);

% ki_v and kp_v parameters
om_n_v = 5; % tune this parameter
zeta_v = 1; % tune this parameter
P.ki_v = om_n_v^2/a_V2;
P.kp_v = (2*zeta_v*om_n_v-a_V1)/a_V2;
