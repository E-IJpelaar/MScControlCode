clear all;

kappa = linspace(0,20,1000);

for ii = 1:length(kappa)
    
    Ks(ii) = bendStiffness(kappa(ii));
    
end

figure
plot(kappa,Ks)