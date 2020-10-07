function dgdl = forwardKinematics(l,g,q,Ba,shape,Nmode)
%% Reorder g to obtain square matrix

[xi] = strainField(l,q,Ba,shape,Nmode);        % calculate strain/curvature

g = [g(1) ,g(2) ,g(3) ,g(4) ;                  % resize g to 4x4
     g(5) ,g(6) ,g(7) ,g(8) ;
     g(9) ,g(10),g(11),g(12);
     g(13),g(14),g(15),g(16)];
 
xi_hat =  [  0    , -xi(3)  ,  xi(2) , xi(4);  % strain and curvatures              
            xi(3) ,  0      , -xi(1) , xi(5);
           -xi(2) ,  xi(1)  ,  0     , xi(6);
            0     ,  0      ,  0     , 0   ]; 
        
dgdl_ = g*xi_hat;                              % g'=g*xi
 
dgdl = [dgdl_(1) ;dgdl_(5) ;dgdl_(9)  ;dgdl_(13) ;  % put g in collum again (pay attention to how this is done)
        dgdl_(2) ;dgdl_(6) ;dgdl_(10) ;dgdl_(14) ;  % row and collum might seem odd, but this is the way
        dgdl_(3) ;dgdl_(7) ;dgdl_(11) ;dgdl_(15) ;
        dgdl_(4) ;dgdl_(8) ;dgdl_(12) ;dgdl_(16)];    

