function drawAirCraft(u,pnts,faces)
persistent handle 
coder.extrinsic('patch')
psi = u(1);
tht = u(2);
phi = u(3);
pn = u(4);
pe = u(5);
pd = u(6);

% rotate the aircraft
pnts = rotate(pnts',phi,tht,psi);

% translate the aircraft
pnts = translate(pnts',pn,pe,pd);

% transform vertices from NED to XYZ
R = [...
    0,1,0;
    1,0,0;
    0,0,-1];

pnts = R*pnts';

if isempty(handle)
   handle = patch('Vertices',pnts','Faces',faces,'FaceColor','b');
   axis([-2,2,-2,2,-2,2]);
   grid on
else
   set(handle,'Vertices',pnts','Faces',faces);
%    axis([pn-1,pn+1,pe-1,pe+1,pd-1,pd+1]);
end