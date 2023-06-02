clc;
clear all;
close all;

% Ground Distances from UAV in meters
g_d1 = 100;
g_d2 = 300;
g_d3 = 400;

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

% Number of bits
N = 10^5;

% Rician Fading for Users and BSs
hLOS1 = exp(1i*2*pi*rand(1,N/2));
hLOS2 = exp(1i*2*pi*rand(1,N/2));
hLOS3 = exp(1i*2*pi*rand(1,N/2));

hNLOS1 = sqrt(1/2)*(randn(1,N/2)+1i*randn(1,N/2));
hNLOS2 = sqrt(1/2)*(randn(1,N/2)+1i*randn(1,N/2));
hNLOS3 = sqrt(1/2)*(randn(1,N/2)+1i*randn(1,N/2));


g_UAV_BS1 = sqrt(K_UAV_BS1/(1+K_UAV_BS1))*hLOS1 + sqrt(1/(1+K_UAV_BS1))*hNLOS1;
g_UAV_BS2 = sqrt(K_UAV_BS2/(1+K_UAV_BS2))*hLOS2 + sqrt(1/(1+K_UAV_BS2))*hNLOS2;
g_UAV_BS3 = sqrt(K_UAV_BS3/(1+K_UAV_BS3))*hLOS3 + sqrt(1/(1+K_UAV_BS3))*hNLOS3;


% Avarage Channel Power Gain

% Assume Path Loss Componet = 4
% Assume Average Channel Power Gain = -60dB

eta = 4;    % Path Loss Component
b0 = db2pow(0);  % Average channel power gain at a reference deistance d0 = 1m


% Rician fading for UAVs and BSs

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

Pt = 46;                    %in dBm
pt = (10^-3)*db2pow(Pt);	%in linear scale

y_p = pt.*(abs_h_UAV_BS1+abs_h_UAV_BS2+abs_h_UAV_BS3);


