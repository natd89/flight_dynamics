function h_c = sigmoid_trajectory_altitude(u,P)

h_d = u(1);
rate = u(2);
t = u(3);

persistent h_d_old
persistent h_d_temp
persistent h_c_temp
persistent t0
persistent L
persistent flag

if t==49.98
    o = 1;
end

if t==0
    t0=0;
    h_d_old = -P.pd0;
    h_c = -P.pd0;
    h_c_temp = h_c;
    
%     if h_d~=-P.pd0
%         % h_d_temp = -P.pd0;
%         h_d = -P.pd0;
%     else
%         h_d_temp = h_d;
%     end
    h_d_temp = h_d;

end

% if t~=0

if h_d ~= h_d_temp
    t0 = t;
    L = h_d-h_d_temp;
    h_d_old = h_c_temp;
    flag = 1; % this means that h_c needs to change
end

if flag
    phigh = 0.99;
    plow = 0.01;
    % climb_rate = 1; % meters/second
    alpha = 1/rate;

    k = -1/(L*alpha)*log(1/phigh-1)+1/(L*alpha)*log(1/plow-1);

    f = L/(1+exp(-sign(L)*k*(t-t0-abs(L*alpha)/2))); % the L*alpha/2 shifts the center of the function so it starts at t=0

    if abs(f) < abs(L*0.01) % may need to change this to absolute value of something
       f =  L*0.01;       
    elseif abs(f) >= abs(L*0.99)
        f = L;
        flag = 0; % this means that we've reached the desired position
    end

    h_c = f + h_d_old;
    h_c_temp = h_c;
else
    h_c = h_d;
    h_c_temp = h_c;
end

% end

h_d_temp = h_d;

end