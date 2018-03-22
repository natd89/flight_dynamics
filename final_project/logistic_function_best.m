
x0 = 0;
phigh = 0.99;
plow = 0.01;
L = 4;
alpha = 10;
k = -1/(L*alpha)*log(1/phigh-1)+1/(L*alpha)*log(1/plow-1);

x = -(L*alpha)/2:0.01:(L*alpha)/2;

f = L./(1+exp(-k*(x-x0)));

plot(x,f)
axis([-(alpha*L)/2,(alpha*L)/2,0,L])
