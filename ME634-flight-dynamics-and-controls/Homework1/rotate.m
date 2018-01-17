function pnts = rotate(pnts,phi,tht,psi)

R1 = [...
    cos(psi) -sin(psi) 0;
    sin(psi) cos(psi) 0;
    0         0        1];

R2 = [...
    cos(tht) 0 sin(tht);
    0        1  0;
    -sin(tht) 0 cos(tht)];

R3 = [...
    1        0         0;
    0 cos(phi)  -sin(phi);
    0 sin(phi) cos(phi)];

pnts = R3*R2*R1*pnts;
end
