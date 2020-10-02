function dgds = forwardIntegration(l,g,q,dq,ddq,Ba)

[xi,dxi,ddxi] = strainField(l,q,dq,ddq,Ba);

%% Xi
g  = [ g(1) , g(2) , g(3) ,g(4);
       g(5) , g(6) , g(7) ,g(8);
       g(9) , g(10), g(11),g(12);
       g(13), g(14), g(15),g(16)];
  

xi_hat = [ 0       , -xi(3),  xi(2) , xi(4);               
            xi(3) ,  0      , -xi(1) , xi(5);
           -xi(2) ,  xi(1)  ,  0     , xi(6);
            0       ,0      ,  0     , 0   ];
        
dgds = g*xi_hat;
dgds = reshape(dgds,[],1);
%% Eta

% detads = -(xi_hat*eta_hat - eta_hat*xi_hat) + dxi;

% detadsdt = 



%% dEta


        
        









