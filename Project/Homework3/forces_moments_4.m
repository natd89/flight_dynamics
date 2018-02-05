% forces_moments.m
%   Computes the forces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%

function out = forces_moments(x, delta, wind, P)


    mass = P.mass;
    g = P.g;
    Jx = P.Jx;
    Jx = P.Jx;
    Jy = P.Jy;
    Jz = P.Jz;
    Jxz = P.Jxz;
    S = P.S;
    b = P.b;
    c = P.c;
    Sprop = P.Sprop;
    rho = P.rho;
    kmotor = P.kmotor;
    kTp = P.kTp;
    komega = P.komega;
    e = P.e;
    CL0 = P.CL0;
    CD0 = P.CD0;
    Cm0 = P.Cm0;
    CLalpha = P.CLalpha;
    CDalpha = P.CDalpha;
    Cmalpha = P.Cmalpha;
    CLq = P.CLq;
    CDq = P.CDq;
    Cmq = P.Cmq;
    CLdele = P.CLdele;
    CDdele = P.CDdele;
    Cmdele = P.Cmdele;
    Cprop = P.Cprop;
    M = P.M;
    alpha0 = P.alpha0;
    eps = P.eps;
    CDp = P.CDp;
    Cndelr = P.Cndelr;
    CY0 = P.CY0;
    Cl0 = P.Cl0;
    Cn0 = P.Cn0;
    CYbeta = P.CYbeta;
    Clbeta = P.Clbeta;
    Cnbeta = P.Cnbeta;
    CYp = P.CYp;
    Clp = P.Clp;
    Cnp = P.Cnp;
    CYr = P.CYr;
    Clr = P.Clr;
    Cnr = P.Cnr;
    CYdela = P.CYdela;
    Cldela = P.Cldela;
    Cndela = P.Cndela;
    CYdelr = P.CYdelr;
    Cldelr = P.Cldelr;



    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    % compute wind data in NED
    R_ptp = [...
    cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
    cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
    -sin(theta)         sin(phi)*cos(theta)                             cos(phi)*cos(theta)                           ];
    
    W = R_ptp*[w_ns;w_es;w_ds]+[u_wg;v_wg;w_wg];
    
    uw = W(1);
    vw = W(2);
    ww = W(3);
    
    ur = u-uw;
    vr = v-vw;
    wr = w-ww;
    
    % compute air data
    va = sqrt(ur^2+vr^2+wr^2);
    alpha = atan(wr/ur);
    beta = asin(vr/va);
    
    W_ned = R_ptp'*[uw; vw; ww];
    
    w_n = W_ned(1);
    w_e = W_ned(2);
    w_d = W_ned(3);
    
    
    sigma_alpha = (1+exp(-M*(alpha-alpha0))+exp(M*(alpha+alpha0)))/...
                  ((1+exp(-M*(alpha-alpha0)))*(1+exp(M*(alpha+alpha0))));
    
    AR = (b^2)/S;
              
    CL_alpha = (1-sigma_alpha)*(CL0+CLalpha*alpha)+sigma_alpha*(2*sign(alpha)*sin(alpha)^2*cos(alpha)); 
    CD_alpha = CDp+((CL0+CLalpha*alpha)^2)/(pi*e*AR);
    
    
    CXalpha = -CD_alpha*cos(alpha)+CL_alpha*sin(alpha);
    CXqalpha = -CDq*cos(alpha)+CLq*sin(alpha);
    CXdelealpha = -CDdele*cos(alpha)+CLdele*sin(alpha);
    CZalpha = -CD_alpha*sin(alpha)-CL_alpha*cos(alpha);
    CZqalpha = -CDq*sin(alpha)-CLq*cos(alpha);
    CZdelealpha = -CDdele*sin(alpha)-CLdele*cos(alpha);
    
    % compute external forces and torques on aircraft
    Force(1) =  -mass*g*sin(theta) + 0.5*rho*va^2*S*(CXalpha+CXqalpha*(c/(2*va))*q+CXdelealpha*delta_e) + 0.5*rho*Sprop*Cprop*((kmotor*delta_t)^2-va^2);
    Force(2) =  mass*g*cos(theta)*sin(phi) + 0.5*rho*va^2*S*(CY0+CYbeta*beta+CYp*(b/(2*va))*p+CYr*(b/(2*va))*r+CYdela*delta_a+CYdelr*delta_r);
    Force(3) =  mass*g*cos(theta)*cos(phi) + 0.5*rho*va^2*S*(CZalpha+CZqalpha*(c/(2*va))*q+CZdelealpha*delta_e);
    
    Torque(1) = 0.5*rho*va^2*S*(b*(Cl0+Clbeta*beta+Clp*(b/(2*va))*p+Clr*(b/(2*va))*r+Cldela*delta_a+Cldelr*delta_r)) + (-kTp*(komega*delta_t)^2);
    Torque(2) = 0.5*rho*va^2*S*(c*(Cm0+Cmalpha*alpha+Cmq*(c/(2*va))*q+Cmdele*delta_e));   
    Torque(3) = 0.5*rho*va^2*S*(b*(Cn0+Cnbeta*beta+Cnp*(b/(2*va))*p+Cnr*(b/(2*va))*r+Cndela*delta_a+Cndelr*delta_r));
   
    out = [Force'; Torque'; va; alpha; beta; w_n; w_e; w_d];
end



