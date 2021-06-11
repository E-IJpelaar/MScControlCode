function r = ts2cs(k,e,L0)

theta = L0*(1+e)*k;

if abs(theta)<1e-3
    x = 0;                    % pure elongation
    y = L0*(1+e);                            % no side displacement
               
else
    x = 1/k*(1-cos(theta));
    y = 1/k*sin(theta);
    
end


r = [x;y];