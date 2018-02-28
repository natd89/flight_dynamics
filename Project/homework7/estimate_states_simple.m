% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 
% Modified:  3/15/2010 - RB
%            5/18/2010 - RB
%

function xhat = estimate_states(uu, P)

   persistent lpf_y_gyro
   persistent lpf_x_gyro
   persistent lpf_z_gyro
   persistent lpf_x_accel
   persistent lpf_y_accel
   persistent lpf_z_accel
   persistent lpf_static
   persistent lpf_diff
   persistent lpf_pn_gps
   persistent lpf_pe_gps
   persistent lpf_chi_gps
   persistent lpf_Vg_gps
   
   
   
   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
    lpf_a = 50;
    lpf_a1 = 20;
    if t==0

        alpha = exp(-lpf_a*P.Ts);
        alpha1 = exp(-lpf_a1*P.Ts);
        lpf_x_gyro = 0;
        lpf_y_gyro = 0;
        lpf_z_gyro = 0;
        lpf_static = P.rho*P.g*(-P.pd0);
        lpf_diff = 0.5*P.rho*P.g*P.va0;
        lpf_x_accel = 0;
        lpf_y_accel = 0;
        lpf_z_accel = 0;
        lpf_pn_gps = P.pn0;
        lpf_pe_gps = P.pe0;
        
        
    else
   
        lpf_x_gyro = alpha*lpf_x_gyro + (1-alpha)*y_gyro_x;
        lpf_y_gyro = alpha*lpf_y_gyro + (1-alpha)*y_gyro_y;
        lpf_z_gyro = alpha*lpf_z_gyro + (1-alpha)*y_gyro_z;
        
        lpf_x_accel = alpha1*lpf_x_accel + (1-alpha1)*y_accel_x;
        lpf_y_accel = alpha1*lpf_y_accel + (1-alpha1)*y_accel_y;
        lpf_z_accel = alpha1*lpf_z_accel + (1-alpha1)*y_accel_z;
        
        lpf_static = alpha*lpf_static + (1-alpha)*y_static_pres;
        lpf_diff = alpha*lpf_diff + (1-alpha)*y_diff_pres;
        
        lpf_pn_gps = alpha*lpf_pn_gps + (1-alpha)*y_gps_n;
        lpf_pe_gps = alpha*lpf_pe_gps + (1-alpha)*y_gps_e;
        
        lpf_chi_gps = alpha*lpf_chi_gps + (1-alpha)*y_gps_course;
        lpf_Vg_gps = alpha*lpf_Vg_gps + (1-alpha)*y_gps_Vg;
        
    end
        
    phat = lpf_x_gyro;
    qhat = lpf_y_gyro;
    rhat = lpf_z_gyro;
    hhat = lpf_static/P.rho/P.g;
    Vahat = sqrt(2/P.rho*lpf_diff);
    phihat = atan2(lpf_y_accel/lpf_z_accel);
    thetahat = asin(lpf_x_accel/P.g);
    pnhat = lpf_pn_gps;
    pehat = lpf_pe_gps;
    chihat = lpf_chi_gps;
    Vghat = lpf_Vg_gps;
    psihat = chihat;
    
    
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        bxhat;...
        byhat;...
        bzhat;...
        ];
end
