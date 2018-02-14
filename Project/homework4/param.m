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

