
x0 = 0;
phigh = 0.99;
plow = 0.01;
L = -4;
k = 1;

x = (L)/2:0.01:-(L)/2;

f = L./(1+exp(-k*(x-x0)));

plot(x,f)
% axis([-(alpha*L)/2,(alpha*L)/2,0,L])
