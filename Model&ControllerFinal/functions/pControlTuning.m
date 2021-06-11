function dydt = pControlTuning(t,y,tau_s,Kp,Ki,p_ref,dt)
global error_sum


p1 = y(1);
p2 = y(2);

p = [p1;p2];

error = p_ref-p
error_sum = error_sum + error*dt;
Pp = diag([Kp,Kp])*error;
Ip = diag([Ki,Ki])*error_sum

if Ip(1,1) >=1
    Ip(1,1) = 0;
end
if Ip(2,1) >=1
    Ip(2,1) = 0;
end

U = Pp + Ip;
V = min(max(U,0),12);
K1 = PWC(V(1,1));
K2 = PWC(V(2,1));



dydt = diag([-1/tau_s,-1/tau_s])*p + diag([K1/tau_s,K2/tau_s])*V;