clr;
%% generate mesh
msh1 = Mmesh(COIL1,'NElem',2500,'NSegment',1);
%msh2 = Mmesh(COIL2,'NElem',1500,'NSegment',1);
msh1 = msh1.generate();
%msh2 = msh2.generate();

%% adjust
deg = linspace(-pi/3,pi/3,25);
Y = [];

for i = 1:25
    msh1 = msh1.reset();
    msh1 = Blender(msh1,'PCC+',{0,deg(i),1});
    msh1 = msh1.inductance;
    Y = [Y; msh1.Inductance];
    tic; msh1.render(); toc;
end


function d = COIL1
c1 = cSquareHelix(-0.0125,0,0,0.01,0.02,10,0.05,20);  
c2 = cSquareHelix(0.0125,0,0.0,0.01,0.02,10,0.05,-20); 

d = cUnion(c1,c2);
end
