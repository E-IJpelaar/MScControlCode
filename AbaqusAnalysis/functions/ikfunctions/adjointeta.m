function adeta = adjointeta(eta)


w = [  0    ,-eta(3), eta(2);
      eta(3),  0    ,-eta(1);
     -eta(2), eta(1),  0   ];  % code Brandon Liegroup.h
 
 
W = [ 0     ,-eta(6), eta(5);
      eta(6),  0    ,-eta(4);
     -eta(5), eta(4),  0   ];  
 
O = zeros(3,3);

adeta = [w, O ;
         W, w];
