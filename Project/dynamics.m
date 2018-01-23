function dxdt = dynamics(t,x,forces,moments,params)

psi = x(1);
th = x(2);
phi = x(3);
pn = x(4);
pe = x(5);
pd = x(6);
u = x(7);
v = x(8);
w = x(9);
p = x(10);
q = x(11);
r = x(12);

m = params(1);
Jx = params(2);
Jy = params(3);
Jz = params(4);
Jxz = params(5);

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

dxdt = [psi_dot;th_dot;phi_dot;pn_dot;pe_dot;pd_dot;u_dot;v_dot;w_dot;p_dot;q_dot;r_dot];

end