function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
    pn      = x_trim(1);
    pe      = x_trim(2);
    pd      = x_trim(3);
    u       = x_trim(4);
    v       = x_trim(5);
    w       = x_trim(6);
    phi     = x_trim(7);
    theta   = x_trim(8);
    psi     = x_trim(9);
    p       = x_trim(10);
    q       = x_trim(11);
    r       = x_trim(12);

% u_trim is the trimmed input

    delta_e = u_trim(1);
    delta_a = u_trim(2);
    delta_r = u_trim(3);
    delta_t = u_trim(4);

alpha = atan(w/v);
beta = asin(v/(u^2+v^2+w^2));

chi = psi;

% add stuff here
a_phi1 = -0.5*P.rho*P.va0^2*P.S*P.b*P.Cpp*P.b/(2*P.va0);
a_phi2 = 0.5*P.rho*P.va0^2*P.S*P.b*P.Cpdela;

a_beta1 = -(P.rho*P.va0*P.S)/(2*P.mass)*P.CYbeta;
a_beta2 = (P.rho*P.va0*P.S)/(2*P.mass)*P.CYdelr;

a_theta1 = -(P.rho*P.va0^2*P.c*P.S)/(2*P.Jy)*P.Cmq*(P.c/(2*P.va0));
a_theta2 = -(P.rho*P.va0^2*P.c*P.S)/(2*P.Jy)*P.Cmalpha;
a_theta3 = (P.rho*P.va0^2*P.c*P.S)/(2*P.Jy)*P.Cmdele;

a_V1 = (P.rho*P.va0*P.S)/(P.mass)*(P.CD0 + P.CDalpha*alpha + P.CDdele*delta_e) + (P.rho*P.Sprop)/(P.mass)*P.Cprop*P.va0;
a_V2 = (P.rho*P.Sprop)/(P.mass)*P.Cprop*P.kmotor^2*delta_t;
a_V3 = P.g*cos(theta-chi);

% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.g/P.va0],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([P.va0],[1,0]);
T_h_Va          = tf([theta],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([P.va0*a_beta2],[1,a_beta1]);

