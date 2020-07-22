clr;
%% assign free DOF
mdl = Model([0,1,1,0,0,0],'NModal',4,'NDisc',2);

mdl = mdl.set('Jacobian',true,'Movie',false,'MovieAxis',[-0.75 0.75 -0.75 0.75 -1.75 .25]*0.85);
% gravity put to -9.81


%% generate dynamic model
mdl = mdl.generate();

%% assign controllers
mdl.point = [1 1 1 1 1 1 1 1];%[0,0,0,0.4,0.,0.7];
mdl.Pressure = @(t) 0*[0,0,0,0,0.5,0,0,0];

%% simulate soft robot
mdl = mdl.csolve(); 

%% show simulation
figure(102)
t = mdl.get('t');
q = mdl.g;
u = mdl.tau;
ge = mdl.ge;

subplot(2,1,1);
plot(t,q,'-','linewidth',1.0);
xlabel('Time [s]')


subplot(2,1,2);
plot(t,u,'-','linewidth',1.0);

% figure(15);
% plot3(ge(:,7),ge(:,6),-ge(:,5),'-','linewidth',1.0);
% axis equal;
% t = state(:,1);s
% z = state(:,2:end);
%plot(t,z);
% l0 = 0.064;
% g(:,3) = (l0 - x(:,1))/l0;
% g(:,1) = x(:,2)*l0;
% g(:,2) = x(:,3)*l0;
% % 
% mdl = mdl.set('t',t);
% mdl = mdl.set('g',g);
% % % 
% mdl.showModel();



function tau = Controller(t,g)
kp = 1e-4;
tau = zeros(6,1);
tau(4:6) = kp*(g - [0.3;0.5;0.5]*clamp(t/5,1e-2,1));
end