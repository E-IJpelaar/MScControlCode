function error = mappingForce(x,p,F)

error = sum((F-x*p).^2);