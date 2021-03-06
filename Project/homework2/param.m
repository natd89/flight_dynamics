% initial states
P.va0 = 0;
P.pn0 = 0;
P.pe0 = 0;
P.pd0 = 0;
P.u0 = P.va0;
P.v0 = 0;
P.w0 = 0;
P.phi0 = 0;
P.th0 = 0;
P.psi0 = 0;
P.p0 = 0;
P.q0 = 0;
P.r0 = 0;

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
P.Ts = 0.0005;
P.sigmau = 1.06;
P.sigmav = P.sigmau;
P.sigmaw = 0.7;
P.wind_n = -5.0;
P.wind_e = 0.0;
P.wind_d = 0.0;