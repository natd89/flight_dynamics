% sensors.m
%   Compute the output of rate gyros, accelerometers, and pressure sensors
%
%  Revised:
%   3/5/2010  - RB 
%   5/14/2010 - RB

function y = sensors(uu, P)

    % relabel the inputs
%    pn      = uu(1);
%    pe      = uu(2);
    pd      = uu(3);
%    u       = uu(4);
%    v       = uu(5);
%    w       = uu(6);
    phi     = uu(7);
    theta   = uu(8);
%    psi     = uu(9);
    p       = uu(10);
    q       = uu(11);
    r       = uu(12);
    F_x     = uu(13);
    F_y     = uu(14);
    F_z     = uu(15);
%    M_l     = uu(16);
%    M_m     = uu(17);
%    M_n     = uu(18);
    Va      = uu(19);
%    alpha   = uu(20);
%    beta    = uu(21);
%    wn      = uu(22);
%    we      = uu(23);
%    wd      = uu(24);
    

    eta_gyro_x = randn*0.13*pi/180;
    eta_gyro_y = randn*0.13*pi/180;
    eta_gyro_z = randn*0.13*pi/180;

    eta_accel_x = randn*0.0025;
    eta_accel_y = randn*0.0025;
    eta_accel_z = randn*0.0025;

    beta_abs_pres = 125;
    eta_abs_pres = randn*2;

    % simulate rate gyros (units are rad/sec)
    y_gyro_x = p + eta_gyro_x;
    y_gyro_y = q + eta_gyro_y;
    y_gyro_z = r + eta_gyro_z;

    % simulate accelerometers (units of g)
    y_accel_x = F_x/P.mass + P.g*sin(theta) + eta_accel_x;
    y_accel_y = F_y/P.mass - P.g*cos(theta)*sin(phi) + eta_accel_y;
    y_accel_z = F_z/P.mass - P.g*cos(theta)*cos(phi) + eta_accel_z;

    % simulate pressure sensors
    y_static_pres = P.P0 * (P.T0/(P.T0+P.L0*pd))^(P.g*P.Mol/(P.R*P.L0));
    y_diff_pres = P.rho*P.g*pd + beta_abs_pres + eta_abs_pres;

    % construct total output
    y = [...
        y_gyro_x;...
        y_gyro_y;...
        y_gyro_z;...
        y_accel_x;...
        y_accel_y;...
        y_accel_z;...
        y_static_pres;...
        y_diff_pres;...
    ];

end



