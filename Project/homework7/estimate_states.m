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
    persistent xhat_att
    persistent xhat_GPS
    persistent phi
    persistent theta
    persistent psi
    persistent chi
    persistent Q_att
    persistent Q_GPS
    persistent S_att
    persistent S_GPS
    persistent f_att
    persistent f_GPS
    persistent pn
    persistent pe
    persistent Vg
    persistent Va
    persistent wn
    persistent we
    persistent R_att
    persistent R_GPS
   
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
    Tout = 1/100;
    
    p = y_gyro_x;
    q = y_gyro_y;
    r = y_gyro_z;
    Va = sqrt(2/P.rho*y_diff_pres);
    y_att = [y_accel_x; y_accel_y; y_accel_z];
    y_GPS = [y_gps_n; y_gps_e; y_gps_Vg; y_gps_course]; 
    
    if t==0
        %**attitude estimation**%
        %***********************%
        phi = P.phi0;
        theta = P.th0;
        psi = P.psi0;
        chi = P.chi0;
        pn = P.pn0;
        pe = P.pe0;
        Vg = P.vg0;
        Va = P.va0;
        wn = 0;
        we = 0;            
        xhat_att = [phi;theta];
        xhat_GPS = [pn;pe;Vg;chi;psi];
        Q_att = eye(2)*1e-9; % model uncertainty
        Q_GPS = [5, 0, 0, 0, 0;... % model uncertainty
                 0, 5, 0, 0, 0;...
                 0, 0, 5, 0, 0;...
                 0, 0, 0, 0.002, 0;...
                 0, 0, 0, 0, .002];
        S_att = eye(2); % attitude covariance matrix
        S_GPS = eye(5); % GPS covariance matrix
        R_att = [1e-1, 1e-1, 1e-1]; % sensor model uncertainty
        R_GPS = [10, 10, 1, 1]; % sensor model uncertainty
        %***********************%

        for i=1:N
            %**attitude estimation**%
            %***********************%
            Va = sqrt(2/P.rho*y_diff_pres);
            f_att = [p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);...
                 q*cos(phi)-r*sin(phi)];
            xhat_att = xhat_att + Tout/N*f_att;
            A = [q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta), (q*sin(phi)-r*cos(phi))/(cos(theta)^2);...
                 -q*sin(phi)-r*cos(phi), 0];
            phi = xhat_att(1);
            theta = xhat_att(2);
            S_att = S_att + (Tout/N)*(A*S_att + S_att*A' + Q_att);               
            %***********************%
            
            %*****GPS Smoothing*****%
            %***********************%
            psidot = q*(sin(phi)/cos(theta))+r*(cos(phi)/cos(theta));
            Vgdot = Va/Vg*(-wn*sin(psi)+we*cos(psi));
            dVgdot_dpsi = -psidot*Va*cos(chi-psi);
            dVgdot_dchi = psidot*Va*cos(chi-psi);
            dchidot_dVg = (-P.g/Vg^2)*tan(phi)*cos(chi-psi);
            dchidot_dchi = (-P.g/Vg)*tan(phi)*sin(chi-psi);
            dchidot_dpsi = (P.g/Vg)*tan(phi)*sin(chi-psi);            
            f_GPS = [Vg*cos(chi);...
                     Vg*sin(chi);...
                     psidot*Va*sin(chi-psi);...
                     (P.g/Vg)*tan(phi)*cos(chi-psi);...
                     q*(sin(phi)/cos(theta)) + r*(cos(phi)/cos(theta))];
            xhat_GPS = xhat_GPS + (Tout/N)*f_GPS;
            A = [0,  0,  cos(chi), -Vg*sin(chi), 0;...
                 0,  0,  sin(chi), Vg*cos(chi), 0;...
                 0,  0,  -Vgdot/Vg, dVgdot_dchi, dVgdot_dpsi;...
                 0,  0,  dchidot_dVg, dchidot_dchi, dchidot_dpsi;...
                 0,  0,  0,   0,   0];                  
            S_GPS = S_GPS + (Tout/N)*(A*S_GPS + S_GPS*A' + Q_GPS);
            %***********************%
            
            % update the wind estimates
            wn = Vg*cos(chi)-Va*cos(psi);
            we = Vg*sin(chi)-Va*sin(psi);
            
            phi = xhat_att(1);
            theta =  xhat_att(2);
            
            pn = xhat_GPS(1);
            pe =  xhat_GPS(2);
            Vg = xhat_GPS(3);
            chi =  xhat_GPS(4);
            psi =  xhat_GPS(5);
        end
        
    end
    
            
    if t~=0
        for i=1:N
            %**attitude estimation**%
            %***********************%
            Va = sqrt(2/P.rho*y_diff_pres);
            f_att = [p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);...
                 q*cos(phi)-r*sin(phi)];
            xhat_att = xhat_att + Tout/N*f_att;
            phi = xhat_att(1);
            theta = xhat_att(2);
            A = [q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta), (q*sin(phi)-r*cos(phi))/(cos(theta)^2);...
                 -q*sin(phi)-r*cos(phi), 0];
            S_att = S_att + (Tout/N)*(A*S_att + S_att*A' + Q_att);               
            %***********************%
            
            if t==6
               o=1; 
            end
            
            %*****GPS Smoothing*****%
            %***********************%
            psidot = q*(sin(phi)/cos(theta))+r*(cos(phi)/cos(theta));
            Vgdot = Va/Vg*(-wn*sin(psi)+we*cos(psi));
            dVgdot_dpsi = -psidot*Va*cos(chi-psi);
            dVgdot_dchi = psidot*Va*cos(chi-psi);
            dchidot_dVg = (-P.g/Vg^2)*tan(phi)*cos(chi-psi);
            dchidot_dchi = (-P.g/Vg)*tan(phi)*sin(chi-psi);
            dchidot_dpsi = (P.g/Vg)*tan(phi)*sin(chi-psi);            
            f_GPS = [Vg*cos(chi);...
                     Vg*sin(chi);...
                     psidot*Va*sin(chi-psi);...
                     (P.g/Vg)*tan(phi)*cos(chi-psi);...
                     q*(sin(phi)/cos(theta)) + r*(cos(phi)/cos(theta))];
            xhat_GPS = xhat_GPS + (Tout/N)*f_GPS;
            A = [0,  0,  cos(chi), -Vg*sin(chi), 0;...
                 0,  0,  sin(chi), Vg*cos(chi), 0;...
                 0,  0,  -Vgdot/Vg, dVgdot_dchi, dVgdot_dpsi;...
                 0,  0,  dchidot_dVg, dchidot_dchi, dchidot_dpsi;...
                 0,  0,  0,   0,   0];                  
            S_GPS = S_GPS + (Tout/N)*(A*S_GPS + S_GPS*A' + Q_GPS);
            %***********************%
        end
        
        dhdx_att = [0,  q*Va*cos(theta)+P.g*cos(theta);...
                     -P.g*cos(phi)*cos(theta), -r*Va*sin(theta)-p*Va*cos(theta)+P.g*sin(phi)*sin(theta);...
                     P.g*sin(phi)*cos(theta), (q*Va+P.g*cos(phi))*sin(phi)];
                 
        dhdx_GPS = [1, 0, 0, 0, 0;...
                    0, 1, 0, 0, 0;...
                    0, 0, 1, 0, 0;...
                    0, 0, 0, 1, 0];
                
        h_att = [q*Va*sin(theta)+P.g*sin(theta);...
                 r*Va*cos(theta)-p*Va*sin(theta)-P.g*cos(theta)*sin(phi);...
                 -q*Va*cos(theta)-P.g*cos(theta)*cos(phi)];
             
        h_GPS = [pn;...
                 pe;...
                 Vg;...
                 chi];
             
        for i=1:3
            C = dhdx_att(i,:);
            L = (R_att(i) + C*S_att*C')\S_att*C';
            S_att = (eye(2)-L*C)*S_att;
            xhat_att = xhat_att + L*(y_att(i) - h_att(i));
        end
        
        for i=1:4
            C = dhdx_GPS(i,:);
            L = (R_GPS(i) + C*S_GPS*C')\S_GPS*C';
            S_GPS = (eye(5)-L*C)*S_GPS;
            xhat_GPS = xhat_GPS + L*(y_GPS(i) - h_GPS(i));
        end
        
            % update the wind estimates
            wn = Vg*cos(chi)-Va*cos(psi);
            we = Vg*sin(chi)-Va*sin(psi);
            
            phi = xhat_att(1);
            theta =  xhat_att(2);
            
            pn = xhat_GPS(1);
            pe =  xhat_GPS(2);
            Vg = xhat_GPS(3);
            chi =  xhat_GPS(4);
            psi =  xhat_GPS(5);
         
       
    end
    
    % perform the Kalman filter correction
    
    
    % not estimating these states 
    alphahat = 0;
    betahat  = 0;
    bxhat    = 0;
    byhat    = 0;
    bzhat    = 0;
    
    pnhat = xhat_GPS(1);
    pehat = xhat_GPS(2);
    hhat = y_static_pres/P.rho/P.g + 3;
    Vahat = sqrt(2/P.rho*y_diff_pres);
    phihat = xhat_att(1);
    thetahat = xhat_att(2);
    psihat = xhat_GPS(5);
    chihat = xhat_GPS(4);
    phat = p;
    qhat = q;
    rhat = r;
    Vghat = xhat_GPS(3);
    wnhat = wn;
    wehat = we;
        
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
