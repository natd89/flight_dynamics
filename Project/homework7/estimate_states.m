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

    persistent Tout
    persistent N
    persistent xhat_attitude
    persistent xhat_GPS
    persistent phi
    persistent theta
    persistent psi
    persistent chi
    persistent Q_attitude
    persistent Q_GPS
    persistent S_attitude
    persistent S_GPS
    persistent f_attitude
    persistent f_GPS
    persistent pn
    persistent pe
    persistent Vg
    persistent wn
    persistent we
   
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
    N = 10;
    Tout = 1000;
    
    p = y_gyro_x;
    q = y_gyro_y;
    r = y_gyro_z;
    Va = sqrt(2/P.rho*y_diff_pres);
    
    for i=1:N
        
        if t==0
            %**attitude estimation**%
            %***********************%
            phi = P.phi0;
            theta = P.theta0;
            psi = pis0;
            chi = P.chi0;
            pn = P.pn0;
            pe = P.pe0;
            Vg = P.vg0;
            wn = 0;
            we = 0;            
            xhat_attitude = [phi;theta];
            xhat_GPS = [pn;pe;Vg;wn;we;psi];
            Q_attitude = eye(2)*1e-3; % model uncertainty
            Q_GPS = eye(2)*1e1; % measurement uncertainty
            S_attitude = eye(2); % attitude covariance matrix
            S_GPS = eye(2); % GPS covariance matrix
            %***********************%
            
            %*****GPS Smoothing*****%
            %***********************%
            
            %***********************%
            
        else
            %**attitude estimation**%
            %***********************%
            f_attitude = [p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);...
                 q*cos(phi)-r*sin(phi)];
            xhat_attitude = xhat_attitude + Tout/N*f_attitude;
            A = [q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta), (q*sin(phi)-r*cos(phi))/(cos(theta)^2);...
                 -q*sin(phi)-r*cos(phi), 0];
            S_attitude = S_attitude + (Tout/N)*(A*S_attitude + S_attitude*A' + Q_attitude);               
            %***********************%
            
            %*****GPS Smoothing*****%
            %***********************%
            psidot = q*(sin(phi)/cos(theta))+r*(cos(phi)/cos(theta));
            Vgdot = ((Va*cos(psi)+wn)*(-Va*psidot*sin(psi))+(Va*sin(psi)+we)*(Va*psidot*cos(psi)))/(Vg);
            dVgdot_dpsi = (-psidot*Va*(wn*cos(psi)+we+sin(psi)))/(Vg);
            dchidot_dVg = (-P.g/Vg^2)*tan(phi)*cos(chi-psi);
            dchidot_dchi = (-P.g/Vg)*tan(phi)*sin(chi-psi);
            dchidot_dpsi = (P.g/Vg)*tan(phi)*sin(chi-psi);            
            f_GPS = [Vg*cos(chi);...
                     Vg*sin(chi);...
                     Vgdot;...
                     (P.g/Vg)*tan(phi)*cos(chi-psi);...
                     0;...
                     0;...
                     q*(sin(phi)/cos(theta)) + r*(cos(phi)*cos(theta))];
            A = [0,  0,  cos(chi), -Vg*sin(chi), 0,  0,  0;...
                 0,  0,  sin(chi), -Vg*cos(chi), 0,  0,  0;...
                 0,  0,  Vgdot/Vg,            0, -(psidot*Va*sin(psi))/Vg, (psidot*Va*cos(psi))/Vg, dVgdot_dpsi;...
                 0,  0,  dchidot_dVg, dchidot_dchi, 0, 0, dchidot_dpsi;...
                 0,  0,  0,   0,   0,  0,  0;...
                 0,  0,  0,   0,   0,  0,  0;...
                 0,  0,  0,   0,   0,  0,  0]; 
            xhat_GPS = xhat_GPS + (Tout/N)*f_GPS;            
            %***********************%
        end
    end
    
    % perform the Kalman filter correction
    
    
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
