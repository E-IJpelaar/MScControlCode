clr;
%%

s = linspace(0,1,100);
y = @(x,n) legendre(x,n).';

hold on;
for ii = [5,4,3,2,1]
    subplot(5,1,ii);
    hold on;
    shade(s,y(s,ii-1),'Color',col(ii),'linewidth',1);
    shade(s,0*y(s,ii-1),'Color',1.05*col(ii),'linewidth',0.5);
    axis([0,1,-1.0,1.0]);
    set(gca,'linewidth',1)
    box on;
end

