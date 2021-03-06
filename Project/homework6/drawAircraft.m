function drawAirCraft(u,V,F)

persistent handle
persistent pnts 
persistent faces

coder.extrinsic('patch')
pn = u(1);
pe = u(2);
pd = u(3);
phi = u(7);
tht = u(8);
psi = u(9);
t = u(13);

if t==0
   [pnts, faces] = pnts_faces;

   % transform vertices from NED to XYZ
   R = [...
       0,1,0;
       1,0,0;
       0,0,-1];
   pnts_f = R*pnts';
   figure(1)
   clf
   handle = patch('Vertices',pnts_f','Faces',faces,'FaceColor','b');
   axis(1*[-5,5,-5,5,-5,5]);
   grid on
else
    
   % rotate the aircraft
    pnts_r = rotate_4(pnts',phi,tht,psi);

    % translate the aircraft
    pnts_t = translate_4(pnts_r',pn,pe,pd);

    % transform vertices from NED to XYZ
    R = [...
        0,-1,0;
        1,0,0;
        0,0,-1];

%     R = [...
%         1,0,0;
%         0,1,0;
%         0,0,1];
    
    
    pnts_f = R*pnts_t'; 
   set(handle,'Vertices',pnts_f','Faces',faces);
   axis([-pe-1,-pe+1,pn-1,pn+1,-pd-1,-pd+1]);
%    axis([pn-1,pn+1,pe-1,pe+1,pd-1,pd+1]);
end

end

function [V, F] = pnts_faces

V = 1*[...
    0.25,     0,       0;
    .125,    .1,     -.1;
    .125,   -.1,     -.1;
    .125,   -.1,      .1;
    .125,    .1,      .1;
    -.75,      0,      0;
       0,     .5,      0;
    -.25,     .5,      0;
    -.25,    -.5,      0;
       0,    -.5,      0;
     -.5,    .25,      0;
    -.75,    .25,      0;
    -.75,   -.25,      0;
     -.5,   -.25,      0;
     -.5,      0,      0;
    -.75,      0,    -.25];

F = [...
    1, 2, 3, NaN;
    1, 3, 4, NaN;
    1, 4, 5, NaN;
    1, 2, 5, NaN;
    3, 4, 6, NaN;
    2, 3, 6, NaN;
    2, 5, 6, NaN;
    4, 5, 6, NaN;
    6, 15, 16, NaN;
    11, 12, 13, 14;
    7, 8, 9, 10];

end
