clear all;close all;clc;


p =  [0; 5; 10; 20; 30; 40; 50; 60; 70; 80; 90; 100];
x_coord = [0 ; -0.0895 ; -0.1275 ; -0.1238 ;  -0.0765 ; -0.0095 ;  0.0680  ;   0.1524  ;  0.2413  ;  0.3340  ;  0.4298 ;0.5303  ]; 
y_coord = [0 ;  6.7059 ; 11.4679 ; 17.9628 ;  22.4614 ; 25.9270 ;  28.7604 ;  31.1654  ; 33.2600  ;  35.1187 ; 36.7918 ;38.3149];
z_coord = [0 ; -0.0851 ; -0.1297 ; -0.1679 ; -0.1849 ; -0.1958 ; -0.2046  ;  -0.2126  ;  -0.2201 ; -0.2275   ; -0.2348 ;-0.2419];

p_vals = 0:0.05:120;

[a,s] = polyfit(p,y_coord,5);
fit = polyval(a,p_vals);



figure(1)
plot(p,y_coord,'o','MarkerSize',8,'MarkerFaceColor','b')
hold on; grid on; box on;
plot(p_vals,fit,'r','LineWidth',2)
% plot(p_vals,simp_fit,'k','LineWidth',2)
xlabel('Bellow pressure [kPa]')
ylabel('Elongation in y-direction [mm]')
legend('Simulated','Polynomial fit')
