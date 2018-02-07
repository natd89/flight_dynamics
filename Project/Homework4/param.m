% initial states

Va = 17;
gamma = 0*pi/180;
R = 50;

DX0 = [0; 0; -Va*sin(gamma); 0; 0; 0; 0; 0; Va/R; 0; 0; 0];
IDX = [3; 4; 5; 6 ;7; 8; 9; 10; 11; 12];
X0 = [0; 0; 0; Va; 0; 0; 0; gamma; 0; 0; 0; 0];
IX0 = [];
U0 = [0; 0; 0; 1];
IU0 = [];
Y0 = [Va; gamma; 0];
IY0 = [1,3];

[X,U,Y,DX] = trim('mavsim_trim',X0, U0, Y0, IX0, IU0, IY0, DX0, IDX);

P.delta_e = U(1);
P.delta_a = U(2);
P.delta_r = U(3);
P.delta_t = U(4);

P.va0 = Va;
P.pn0 = 0;
P.pe0 = 0;
P.pd0 = -100;
P.u0 = X(4);
P.v0 = X(5);
P.w0 = X(6);
P.phi0 = X(7);
P.th0 = X(8);
P.psi0 = X(9);
P.p0 = X(10);
P.q0 = X(11);
P.r0 = X(12);

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