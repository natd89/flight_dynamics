% gps.m
%   Compute the output of gps sensor
%
%  Revised:
%   3/5/2010 - RB 
%   5/14/2010 - RB

function y = gps(uu, P)
    persistent vn
    persistent ve
    persistent vh
    
    % relabel the inputs
    Va      = uu(1);
%    alpha   = uu(2);
%    beta    = uu(3);
    wn      = uu(4);
    we      = uu(5);
%    wd      = uu(6);
    pn      = uu(7);
    pe      = uu(8);
    pd      = uu(9);
%    u       = uu(10);
%    v       = uu(11);
%    w       = uu(12);
%    phi     = uu(13);
%    theta   = uu(14);
    psi     = uu(15);
%    p       = uu(16);
%    q       = uu(17);
%    r       = uu(18);
    t       = uu(19);
    
    sigma_vn = randn*0.21;
    sigma_ve = randn*0.21;
    sigma_vh = randn*0.4;
    
    if t==0
        vn = 0;
        ve = 0;
        vh = 0;
    else
        vn = exp(-P.Kgps*P.Ts_gps)*vn + sigma_vn;
        ve = exp(-P.Kgps*P.Ts_gps)*ve + sigma_ve;
        vh = exp(-P.Kgps*P.Ts_gps)*vh + sigma_vh;
    end
    
    % construct North, East, and altitude GPS measurements
    y_gps_n = pn + vn;
    y_gps_e = pe + ve; 
    y_gps_h = -pd + vh; 
    
    Vn = Va*cos(psi)+wn;
    Ve = Va*sin(psi)+we;
    
    sigma_vg = sqrt((Vn^2*sigma_vn^2 + Ve^2*sigma_ve^2)/(Vn^2 + Ve^2));
    sigma_chi = sqrt((Vn^2*sigma_ve^2 + Ve^2*sigma_vn^2)/(Vn^2 + Ve^2));
    
    % construct groundspeed and course measurements
    y_gps_Vg     = sqrt(Vn^2 + Ve^2) + sigma_vg^2;
    y_gps_course = atan2(Ve,Vn)+sigma_chi^2;

    % construct total output
    y = [...
        y_gps_n;...
        y_gps_e;...
        y_gps_h;...
        y_gps_Vg;...
        y_gps_course;...
        ];
end



