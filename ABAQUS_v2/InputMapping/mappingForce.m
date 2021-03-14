function error = mappingForce(x,p,F)
error = sum((F-(2*x*p)).^2);