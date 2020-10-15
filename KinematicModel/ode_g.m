function dgds = ode_g(l,g)

K_x = [ 0   , -0.4  , 0.6;
        0.4 ,  0    , 0.2;
       -0.6 , -0.2  , 0];

E= [0.5;0.5;0.2];

O3 = zeros(1,3);
xi_hat = [K_x,E
          O3,0];
      
g = [g(1) ,g(2), g(3) ,g(4) ;
     g(5) ,g(6), g(7) ,g(8) ;
     g(9) ,g(10),g(11),g(12);
     g(13),g(14),g(15),g(16)];    


dgds = g*xi_hat;
dgds = reshape(dgds,[],1);


        
   
      
      
      
    




