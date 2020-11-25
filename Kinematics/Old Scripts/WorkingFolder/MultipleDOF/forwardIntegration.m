function dgdl = forwardIntegration(l,g,q,Ba)

[xi_hat] = strainField(l,q,Ba);


%% Xi
g  = [ g(1) , g(2) , g(3) ,g(4);
       g(5) , g(6) , g(7) ,g(8);
       g(9) , g(10), g(11),g(12);
       g(14), g(14), g(15),g(16)];
      
        
dgds_ = g*xi_hat;

dgdl= [dgds_(1);dgds_(2);dgds_(3);dgds_(4);
        dgds_(5);dgds_(6);dgds_(7);dgds_(8);
        dgds_(9);dgds_(10);dgds_(11);dgds_(12);
        dgds_(13);dgds_(14);dgds_(15);dgds_(16)];

check = inv(g)*dgds_    
        








