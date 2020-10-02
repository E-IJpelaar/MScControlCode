function xi = strainFieldfnc(l,g,t)

q = [1;0.5;0.2;1;0.5;0.2];

phi = [1,l,2*l^2-1,0,0,0;
       0,0,0,1,l,2*l^2-1];
   
xi = phi*1;


    


