clc;
clear all
close all

% Ground Distances from UAV in meters
g_d1 = 100;
g_d2 = 500;
g_d3 = 800;

% BSs Heights
h_BS1 = 40;
h_BS2 = 50;
h_BS3 = 30;

% UAV height in meters
height_UAV = 45;

% Los Distance between BS and UAV
LoS_Dis_UAV_BS1 = sqrt(g_d1^2 + (abs(height_UAV-h_BS1))^2);
LoS_Dis_UAV_BS2 = sqrt(g_d2^2 + (abs(height_UAV-h_BS2))^2);
LoS_Dis_UAV_BS3 = sqrt(g_d3^2 + (abs(height_UAV-h_BS3))^2);

% Angle

angle_UAV_BS1 = asin((abs(height_UAV-h_BS1))/LoS_Dis_UAV_BS1);
angle_UAV_BS2 = asin((abs(height_UAV-h_BS2))/LoS_Dis_UAV_BS2);
angle_UAV_BS3 = asin((abs(height_UAV-h_BS3))/LoS_Dis_UAV_BS3);

% Angle-depend rician factor for Users and BSs
A1 = 1;
A2 = (log(db2pow(60)/A1))/(pi/2);

K_UAV_BS1 = A1*exp(A2*angle_UAV_BS1);
K_UAV_BS2 = A1*exp(A2*angle_UAV_BS2);
K_UAV_BS3 = A1*exp(A2*angle_UAV_BS3);

% Rician Fading for Users and BSs

N = 10^5;
g = sqrt(1/2)*(randn(1,N)+1i*randn(1,N));

g_UAV_BS1 = sqrt(K_UAV_BS1/(1+K_UAV_BS1))*g + sqrt(1/(1+K_UAV_BS1))*g;
g_UAV_BS2 = sqrt(K_UAV_BS2/(1+K_UAV_BS2))*g + sqrt(1/(1+K_UAV_BS2))*g;
g_UAV_BS3 = sqrt(K_UAV_BS3/(1+K_UAV_BS3))*g + sqrt(1/(1+K_UAV_BS3))*g;

% Avarage Channel Power Gain

% Assume Path Loss Componet = 4
% Assume Average Channel Power Gain = -60dB

eta = 4;    % Path Loss Component
b0 = db2pow(0);  % Average channel power gain at a reference deistance d0 = 1m


% Rician fading for UAVs and Users

chPow_UAV_BS1 = b0*((LoS_Dis_UAV_BS1)^(-eta));
chPow_UAV_BS2 = b0*((LoS_Dis_UAV_BS2)^(-eta));
chPow_UAV_BS3 = b0*((LoS_Dis_UAV_BS3)^(-eta));

% Channel Coefficeint

h_UAV_BS1 = sqrt(chPow_UAV_BS1)*g_UAV_BS1;
h_UAV_BS2 = sqrt(chPow_UAV_BS2)*g_UAV_BS2;
h_UAV_BS3 = sqrt(chPow_UAV_BS3)*g_UAV_BS3;

abs_h_UAV_BS1 = (abs(h_UAV_BS1)).^2;
abs_h_UAV_BS2 = (abs(h_UAV_BS2)).^2;
abs_h_UAV_BS3 = (abs(h_UAV_BS3)).^2;

Pt = 60;     %in dB
pt = db2pow(Pt);	%in linear scale

B = 10^6;
No = -174 + 10*log10(B);
no = (10^-3)*db2pow(No);

eff = 0.7;  %power harvesting efficiency

sigma = 0:0.05:0.9;

for u = 1:length(sigma)
    % By Channel Power
    C1 = B*log2(1 + (1-sigma(u))*pt.*abs_h_UAV_BS1./(no + (1-sigma(u))*pt.*(abs_h_UAV_BS2 + abs_h_UAV_BS3)));
    C1_mean(u) = mean(C1);
    
    C2 = B*log2(1 + (1-sigma(u))*pt.*abs_h_UAV_BS2./(no + (1-sigma(u))*pt.*abs_h_UAV_BS3));
    C2_mean(u) = mean(C2);
    
    C3 = B*log2(1 + (1-sigma(u))*pt.*abs_h_UAV_BS3./(no));
    C3_mean(u) = mean(C3);
    
    ph = pt*(abs_h_UAV_BS1).*sigma(u)*eff;
    ph_mean(u) = mean(ph);

end

figure;
plot(sigma,C1_mean,'-^','linewidth',1); hold on; grid on;
plot(sigma,C2_mean,'-^','linewidth',1);
plot(sigma,C3_mean,'-^','linewidth',1);

figure;
plot(sigma,ph_mean,'-^','linewidth',1); hold on; grid on;

