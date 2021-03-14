function adxi = adjointxi(xi)


w = [  0    ,-xi(3), xi(2);
      xi(3),  0    ,-xi(1);
     -xi(2), xi(1),  0   ];  % code Brandon Liegroup.h
 
 
W = [ 0     ,-xi(6), xi(5);
      xi(6),  0    ,-xi(4);
     -xi(5), xi(4),  0   ];  
 
O = zeros(3,3);

adxi = [ w, O ;
         W, w];
