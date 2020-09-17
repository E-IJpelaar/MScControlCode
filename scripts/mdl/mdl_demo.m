clr;
%% assign free DOF
mdl = Model([0,1,1,0,0,0],'NModal',4,'NDisc',2);
mdl = mdl.set('MovieAxis',[-0.75 0.75 -0.75 0.75 -1.75 .25]*0.85);
mdl = mdl.set('Texture', grey);

%% generate dynamic model
mdl = mdl.generate();

%% simulate soft robot
mdl = mdl.csolve(); 

%% show sim
figure(102)
t = mdl.get('t');
q = mdl.q;
u = mdl.tau;
ge = mdl.ge;
dq = mdl.dq;
xd = mdl.get('xd');

subplot(3,4,[1 2 5 6]);
plot(t,u,'linewidth',1.0);
subplot(3,4,[3 4 7 8]);
%plot(t,xd(:,4:end),'k--','linewidth',1.0);hold on;
plot(t,ge,'linewidth',1.0); 
subplot(3,4,9:12);
%semilogy(t,abs(ge(:,5:end) - xd(:,4:end)),'linewidth',1.0); 

mdl.showModel();

