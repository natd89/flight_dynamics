
x0 = 0;
phigh = 0.99;
plow = 0.01;
L = -10;
alpha = 0.2;
k = -1/(L*alpha)*log(1/phigh-1)+1/(L*alpha)*log(1/plow-1);

x = 0:0.01:abs(L*alpha);

f = L./(1+exp(-sign(L)*k*(x-x0-abs(L*alpha)/2)));

plot(x,f)
% axis([0,abs(alpha*L),0,L])
