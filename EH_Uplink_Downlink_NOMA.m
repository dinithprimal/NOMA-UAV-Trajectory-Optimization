clear all
close all
clc
%% Simulation of Perfomance analysis for signals form BS to Users via UAV

% BS Transmit Power in dBm
Pt_BS = 46;

% BS Transmit Power in linear scale
pt_BS = (10^-3)*db2pow(Pt_BS);

% Ground distances form UAV to BSs
g_d_BS1 = 100;
g_d_BS2 = 500;
g_d_BS3 = 800;

% BSs Heights
h_BS1 = 40;
h_BS2 = 50;
h_BS3 = 30;

% Ground distances form UAV to Users
g_d_User1 = 50;
g_d_User2 = 150;
g_d_User3 = 500;

% UAV height in meters
height_UAV = 45;

% Los Distance between BS and UAV
LoS_Dis_UAV_BS1 = sqrt(g_d_BS1^2 + (abs(height_UAV-h_BS1))^2);
LoS_Dis_UAV_BS2 = sqrt(g_d_BS2^2 + (abs(height_UAV-h_BS2))^2);
LoS_Dis_UAV_BS3 = sqrt(g_d_BS3^2 + (abs(height_UAV-h_BS3))^2);

% Angle UAV-BSs
angle_UAV_BS1 = asin((abs(height_UAV-h_BS1))/LoS_Dis_UAV_BS1);
angle_UAV_BS2 = asin((abs(height_UAV-h_BS2))/LoS_Dis_UAV_BS2);
angle_UAV_BS3 = asin((abs(height_UAV-h_BS3))/LoS_Dis_UAV_BS3);

% Angle-depend rician factor for Users and BSs
A1 = 1;
A2 = (log(db2pow(60)/A1))/(pi/2);

K_UAV_BS1 = A1*exp(A2*angle_UAV_BS1);
K_UAV_BS2 = A1*exp(A2*angle_UAV_BS2);
K_UAV_BS3 = A1*exp(A2*angle_UAV_BS3);




