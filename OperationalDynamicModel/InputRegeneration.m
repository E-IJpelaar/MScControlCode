clear all;close all;clc;


load 'simulationk10e03.mat'

k = x(:,1);
e = x(:,2);
dk = x(:,3);
de = x(:,4);
p1 = x(:,5);
p2 = x(:,6);

figure(100)
plot(t,p1)
hold on;grid on;
plot(t,p2)
xlabel('Time [s]');ylabel('Pressure [kPa]')

L0 = 0.0645;

K_poly1 = [  -0.0014 ;   0.0578 ; -0.8828 ;   5.8426 ;  -7.2264];
K_poly2 = [  -0.0014 ;   0.0578 ; -0.8828 ;   5.8426 ;  -7.2264];

tau = zeros(2,length(k));
p_set = zeros(2,length(k));
p_error = zeros(2,length(k));
V = zeros(2,length(k));
p_error_sum = zeros(2,length(k));
p = zeros(2,length(k));
p_rec = zeros(2,length(k)+1);
error = zeros(2,length(k));
tau_error_sum = zeros(2,length(k));

for ii = 1:length(k)

p(:,ii) = [p1(ii,1);p2(ii,1)];
% q = [k(ii);e(ii)];

[J]  = JacobiMatrix(k(ii),e(ii),L0,Nmode,shape,space_step);
Jc = J(4:2:6,:);
 
%% Jacobian controller
r = ts2cs(k(ii),e(ii),L0);   % [kappa;epsilon]==>[theta(degrees); x;y]

error(:,ii) = r_ref-r;
tau_error_sum(:,ii) = tau_error_sum(:,ii) + error(:,ii)*dt;


Kpt = Kp*error(:,ii);
Kit = Ki*tau_error_sum(:,ii);

tau(:,ii) = Jc'*(Kpt + Kit);
 
%% Pressure controller

p_set(:,ii) = inv(H)*tau(:,ii);
% p_set(:,ii) = max(p_set(:,ii),0);
p_error(:,ii) = p_set(:,ii) - p(:,ii);
p_error_sum(:,ii) = p_error_sum(:,ii) + p_error(:,ii)*dt;


Pp = Kpp*p_error(:,ii);
Ip = Kip*p_error_sum(:,ii);


% Pressure PI
V(:,ii) = Pp + Ip;
U(:,ii) = min(max(V(:,ii),0),12);


end

%% Reconstruct control input

figure(2)
plot(t,tau(1,:),'LineWidth',1.5);
hold on; grid on; box on;
plot(t,tau(2,:),'LineWidth',1.5);
legend('Input Moment','Input Force')
xlabel('Time [s]');ylabel('Input [F/M]')

figure(3)
plot(t,p_set(1,:),'LineWidth',1.5);
hold on; grid on; box on;
plot(t,p_set(2,:),'LineWidth',1.5);
plot(t,p(1,:),'LineWidth',1.5);
plot(t,p(2,:),'LineWidth',1.5);
plot(t,p_error(1,:),'LineWidth',1.5);
plot(t,p_error(2,:),'LineWidth',1.5);
legend('p_{set,1}','p_{set,2}','p_1','p_2','error_{p1}','error_{p2}')
xlabel('Time [s]');ylabel('Pressure [kPa]')

% figure(4)
% yyaxis left
% plot(t,e,'LineWidth',1.5)
% ylabel('\epsilon [-]')
% hold on;grid on;
% plot(t,(r_ref(2)/L0)-1*ones(length(e),1))
% yyaxis right
% plot(t,k,'LineWidth',1.5)
% xlabel('Time [s]');ylabel('\kappa [1/m]')
% legend('\epsilon [-]','\kappa [1/m]')
% 
% figure(5)
% yyaxis left
% plot(t,de,'LineWidth',1.5)
% ylabel('\dot{\epsilon} [1/s]')
% hold on;grid on;
% yyaxis right
% plot(t,dk,'LineWidth',1.5)
% xlabel('Time [s]');ylabel('\dot{\kappa} [1/ms]')
% legend('\epsilon [-]','\kappa [1/m]')





for ii = 1:length(k)   
r_pos = ts2cs(k(ii),e(ii),L0);
x_pos(ii) = r_pos(1);
y_pos(ii) = r_pos(2);
end

% figure(6)
% plot(t,r_ref(1)-x_pos,'r','LineWidth',1.5)
% hold on; grid on;
% plot(t,r_ref(2)-y_pos,'b','LineWidth',1.5)
% xlabel('Time [s]');ylabel('Position error [m]')


figure(7)
plot(t,x_pos,'LineWidth',1.5)
hold on;grid on;
plot(t,r_ref(1)*ones(1,length(x_pos)),'r-','LineWidth', 1.5)
xlabel('Time [s]');ylabel('Position [m]')
legend('x - position','Reference','FontSize',12)


figure(8)
plot(t,y_pos,'LineWidth',1.5)
hold on;grid on;
plot(t,r_ref(2)*ones(1,length(y_pos)),'r-','LineWidth', 1.5)
xlabel('Time [s]');ylabel('Position [m]')
legend('y - position','Reference','FontSize',12)

figure(9)
subplot(3,1,1)
plot(t,r_ref(1)-x_pos,'b','LineWidth',1.5)
hold on; grid on;ylabel('Position error x [m]')
yline(0,'r.-')

subplot(3,1,2)
plot(t,r_ref(2)-y_pos,'b','LineWidth',1.5)
hold on;grid on;

yline(0,'r.-')
ylabel('Position error y [m]')

subplot(3,1,3)
plot(t,U1(1,1:end-1),'bx','MarkerSize',1);
hold on; grid on; box on;
plot(t,U2(1,1:end-1),'rx','MarkerSize',1);
xlabel('Time [s]');ylabel('Control input [V]')
legend('V_1','V_2')


