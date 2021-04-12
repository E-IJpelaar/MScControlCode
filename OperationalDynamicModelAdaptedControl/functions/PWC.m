function K = PWC(V)


Vmax = 12;
dV = 3;


if V <= dV
    K = 0;
elseif V > 3 && V < Vmax
    K = 0.0166*V + 6.3949;
else
    K = 0.0166*Vmax + 6.3949;
end

