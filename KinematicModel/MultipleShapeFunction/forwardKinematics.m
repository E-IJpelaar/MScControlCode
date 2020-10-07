function dydl = forwardKinematics(l,g,q,Ba,shape,Nmode)
%% Reorder g to obtain square matrix

[xi] = strainField(l,q,Ba,shape,Nmode);

g = [g(1) ,g(2) ,g(3) ,g(4) ;
     g(5) ,g(6) ,g(7) ,g(8) ;
     g(9) ,g(10),g(11),g(12);
     g(13),g(14),g(15),g(16)];
 
xi_hat =  [  0    , -xi(3)  ,  xi(2) , xi(4);               
            xi(3) ,  0      , -xi(1) , xi(5);
           -xi(2) ,  xi(1)  ,  0     , xi(6);
            0     ,  0      ,  0     , 0   ]; 
        
dydl_ = g*xi_hat;       
 
dydl = [dydl_(1) ;dydl_(2) ;dydl_(3) ;dydl_(4) ;
        dydl_(5) ;dydl_(6) ;dydl_(7) ;dydl_(8) ;
        dydl_(9) ;dydl_(10);dydl_(11);dydl_(12);
        dydl_(13);dydl_(14);dydl_(15);dydl_(16)];

check = inv(g)*dydl_
% g_rot = g(1:3,1:3); 
% g_trans = g(1:3,4);


% xi_hat =  [  0    , -xi(3)  ,  xi(2) , xi(4);               
%             xi(3) ,  0      , -xi(1) , xi(5);
%            -xi(2) ,  xi(1)  ,  0     , xi(6);
%             0     ,  0      ,  0     , 0   ];
%         
% xi_rot = [  0    , -xi(3)  ,  xi(2) ;               
%             xi(3) ,  0      , -xi(1);
%            -xi(2) ,  xi(1)  ,  0    ];
%        
% xi_trans = [xi(4);xi(5);xi(6)];
%         
% dydl_rot = g_rot*xi_rot;
% dydl_trans = g_rot*xi_trans;
        
        
% dydl = g*xi_hat;
% dydl = [dydl_rot,dydl_trans;0,0,0,0];
% check = inv(g)*dydl
% % dydl = reshape(dydl,16,1);
% dydl = [g(1) ;g(2) ;g(3) ;g(4) ;
%         g(5) ;g(6) ;g(7) ;g(8) ;
%         g(9) ;g(10);g(11);g(12);
%         g(13);g(14);g(15);g(16)];
