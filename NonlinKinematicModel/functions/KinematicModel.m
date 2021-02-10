function dxdt = KinematicModel(t,x,invD,H,L0,Nmode,shape,space_step,r)

k = x(1);                  % current actuator length
e = x(2);

J = funcJacobian(k,e,L0,Nmode,shape,space_step);
J  = J(2:2:6,:);

% Calculate stiffness at instant x
Kk = bendStiffness(k);
Ke = elongStiffness(e);
K = diag([Kk,Ke]);        % Stiffness matrix


% Controller here
u = JacobianController(J,L0,k,e,r);
p = H\u;
p = max(0,p);
p = min(80,p);
p = p*smoothstep(t); 


figure(11)
plot(t,p(1),'o')
hold on; grid on;
plot(t,p(2),'x')
xlabel('Time [s]');ylabel('Pressure Input [kPa]')



dxdt = (-invD*K)*x + (invD*H)*p;

end


function y = smoothstep(X)
y = zeros(length(X),1); 
for ii = 1:length(X)
    x = X(ii);
    if x<=0, y(ii,1) = 0;
    elseif (x>0 && x<=1), y(ii,1) = 3*x.^2 -2*x.^3;
    else, y(ii,1) = 1;
    end
end
end

