function dgds = forwardIntegration(l,g,xi)

g  = [ g(1) , g(2) , g(3) ,g(4);
       g(5) , g(6) , g(7) ,g(8);
       g(9) , g(10), g(11),g(12);
       g(13), g(14), g(15),g(16)];
  
eta_hat = [0,0,0,0;               % this should be automated to fill in all d.o.f.
           0,0,-xi(1,1),0;
           0,xi(1,1),0,xi(2,1);
           0,0,0,0];

dgds = g*eta_hat;
dgds = reshape(dgds,[],1);