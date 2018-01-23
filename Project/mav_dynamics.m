function [sys,x0,str,ts,simStateCompliance] = mav_dynamics(t,x,u,flag,P)

switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P);

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,P);

  %%%%%%%%%%
  % Update %
  %%%%%%%%%%
  case 2,
    sys=mdlUpdate(t,x,u);

  %%%%%%%%%%%
  % Outputs %
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u);

  %%%%%%%%%%%%%%%%%%%%%%%
  % GetTimeOfNextVarHit %
  %%%%%%%%%%%%%%%%%%%%%%%
  case 4,
    sys=mdlGetTimeOfNextVarHit(t,x,u);

  %%%%%%%%%%%%%
  % Terminate %
  %%%%%%%%%%%%%
  case 9,
    sys=mdlTerminate(t,x,u);

  %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end sfuntmpl

%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes(P)

%
% call simsizes for a sizes structure, fill it in and convert it to a
% sizes array.
%
% Note that in this example, the values are hard coded.  This is not a
% recommended practice as the characteristics of the block are typically
% defined by the S-function parameters.
%
sizes = simsizes;

sizes.NumContStates  = 12;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 6;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

%
% initialize the initial conditions
%
x0  = [...
    P.pn0 = 0;...
    P.pe0 = 0;...
    P.pd0 = 0;...
    P.u0 = 0;...
    P.v0 = 0;...
    P.w0 = 0;...
    P.phi0 = 0;...
    P.th0 = 0;...
    P.psi0 = 0;...
    P.p0 = 0;...
    P.q0 = 0;...
    P.r0 = 0;...
    ];

%
% str is always an empty matrix
%
str = [];

%
% initialize the array of sample times
%
ts  = [0 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'DisallowSimState' < Error out when saving or restoring the model sim state
simStateCompliance = 'UnknownSimState';

% end mdlInitializeSizes

%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,uu, P)

    phi   = x(1);
    th    = x(2);
    psi   = x(3);
    pn    = x(4);
    pe    = x(5);
    pe    = x(6);
    u     = x(7);
    v     = x(8);
    w     = x(9);
    p     = x(10);
    q     = x(11);
    r     = x(12);
    fx    = uu(1);
    fy    = uu(2);
    fz    = uu(3);
    ell   = uu(4);
    m     = uu(5);
    n     = uu(6);
    

gamma = Jx*Jz-Jxz*Jxz;
gamma1 = Jxz*(Jx-Jy+Jz)/gamma;
gamma2 = (Jz*(Jz-Jy)+Jxz*Jxz)/gamma;
gamma3 = Jz/gamma;
gamma4 = Jxz/gamma;
gamma5 = (Jz-Jx)/Jy;
gamma6 = Jxz/Jy;
gamma7 = ((Jx-Jy)*Jx+Jxz*Jxz)/gamma;
gamma8 = Jx/gamma;


ned_dot = [...
    cos(th)*cos(psi) sin(phi)*sin(th)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(th)*cos(psi)+sin(phi)*sin(psi);
    cos(th)*sin(psi) sin(phi)*sin(th)*sin(psi)-cos(phi)*cos(psi) cos(phi)*sin(th)*sin(psi)+sin(phi)*cos(psi);
    -sin(th)         sin(phi)*cos(th)                             cos(phi)*cos(th)                           ]...
    *[u;v;w];

pn_dot = ned_dot(1);
pe_dot = ned_dot(2);
pd_dot = ned_dot(3);

uvw_dot = [r*v-q*w;p*w-r*u; q*u-p*v] + 1/m*[f_x;f_y;f_z];

u_dot = uvw_dot(1);
v_dot = uvw_dot(2);
w_dot = uvw_dot(3);

ptp_dot = [...
    1 sin(phi)*tan(th) cos(phi)*tan(th);
    0 cos(phi)         -sin(phi);
    0 sin(phi)/cos(th) cos(phi)/cos(th)]...
    *[p;q;r];

phi_dot = ptp_dot(1);
th_dot = ptp_dot(2);
psi_dot = ptp_dot(3);

pqr_dot = [gamma1*p*q-gamma2*q*r; gamma5*p*r-gamma6*(p*p-r*r);gamma7*p*q-gamma1*q*r] + ...
    [gamma3*l+gamma4*n; 1/Jy*m; gamma4*l+gamma8*n];

p_dot = pqr_dot(1);
q_dot = pqr_dot(2);
r_dot = pqr_dot(3);

sys = [psi_dot;th_dot;phi_dot;pn_dot;pe_dot;pd_dot;u_dot;v_dot;w_dot;p_dot;q_dot;r_dot];

% end mdlDerivatives

%
%=============================================================================
% mdlUpdate
% Handle discrete state updates, sample time hits, and major time step
% requirements.
%=============================================================================
%
function sys=mdlUpdate(t,x,u)

sys = [];

% end mdlUpdate

%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs

%
%=============================================================================
% mdlGetTimeOfNextVarHit
% Return the time of the next hit for this block.  Note that the result is
% absolute time.  Note that this function is only used when you specify a
% variable discrete-time sample time [-2 0] in the sample time array in
% mdlInitializeSizes.
%=============================================================================
%
function sys=mdlGetTimeOfNextVarHit(t,x,u)

sampleTime = 1;    %  Example, set the next hit to be one second later.
sys = t + sampleTime;

% end mdlGetTimeOfNextVarHit

%
%=============================================================================
% mdlTerminate
% Perform any end of simulation tasks.
%=============================================================================
%
function sys=mdlTerminate(t,x,u)

sys = [];

% end mdlTerminate