clr;
%% generate mesh from sdf
sdf = @(x) PneuNet(x,20,40,2,20);

msh = Mesh(sdf,'BdBox',[0,20,0,40],'Quads',[10 50]);
msh = msh.generate();

%% show generated mesh
fem = Fem(msh,'VolumeInfill',0.3,'Penal',4,'FilterRadius',4,...
              'Nonlinear',false,'TimeStep',1/3,...
              'OptimizationProblem','Compliant',...
              'MaxIterationMMA',80);

%% set spatial settings
fem = fem.set('Periodic',[1/2, 0],'Repeat',[]);

%% add boundary condition
id = fem.FindNodes('Left'); 
fem = fem.AddConstraint('Support',id,[1,1]);

id = fem.FindNodes('Right'); 
fem = fem.AddConstraint('Spring',id,[0,1]);
fem = fem.AddConstraint('Output',id,[0,-1]);

id = fem.FindNodes('Bottom'); 
fem = fem.AddConstraint('Spring',id,[0,1]);

id = fem.FindElements('Location',[10,25],1);
fem = fem.AddConstraint('PressureCell',id,[1e-3,0]);

%% set density
fem = fem.initialTopology('Hole',[10,25],1);

%% material
%fem.Material = Dragonskin10A;
fem.Material = YeohMaterial('C1',0.11,'C2',0.02,'C3',0,...
    'D1',1,'D2',20,'D3',30);

%% solving
fem.optimize();

function Dist = PneuNet(P,W,H,E,T)
R1 = dRectangle(P,0,W,0,H);
R2 = dRectangle(P,-W/2,E,T,H+H/2);
R3 = dRectangle(P,W-E,W+W/2,T,H+H/2);
C1 = dCircle(P,0,T + 0.5,1);
C2 = dCircle(P,W,T + 0.5,1);
Dist = dDiff(dDiff(dDiff(dDiff(R1,R2),R3),C1),C2);
end