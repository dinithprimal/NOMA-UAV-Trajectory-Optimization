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

% Los Distance between Users and UAV
LoS_Dis_UAV_User1 = sqrt(g_d_User1^2 + height_UAV^2);
LoS_Dis_UAV_User2 = sqrt(g_d_User2^2 + height_UAV^2);
LoS_Dis_UAV_User3 = sqrt(g_d_User3^2 + height_UAV^2);

% Angle UAV-BSs
angle_UAV_BS1 = asin((abs(height_UAV-h_BS1))/LoS_Dis_UAV_BS1);
angle_UAV_BS2 = asin((abs(height_UAV-h_BS2))/LoS_Dis_UAV_BS2);
angle_UAV_BS3 = asin((abs(height_UAV-h_BS3))/LoS_Dis_UAV_BS3);

% Angle UAV-Users
angle_UAV_User1 = asin(height_UAV/LoS_Dis_UAV_User1);
angle_UAV_User2 = asin(height_UAV/LoS_Dis_UAV_User2);
angle_UAV_User3 = asin(height_UAV/LoS_Dis_UAV_User3);

% Angle-depend rician factor for Users and BSs
A1 = 1;
A2 = (log(db2pow(60)/A1))/(pi/2);

K_UAV_BS1 = A1*exp(A2*angle_UAV_BS1);
K_UAV_BS2 = A1*exp(A2*angle_UAV_BS2);
K_UAV_BS3 = A1*exp(A2*angle_UAV_BS3);

K_UAV_User1 = A1*exp(A2*angle_UAV_User1);
K_UAV_User2 = A1*exp(A2*angle_UAV_User2);
K_UAV_User3 = A1*exp(A2*angle_UAV_User3);


% Rician Fading for Users and BSs

N = 10^5;
g = sqrt(1/2)*(randn(1,N)+1i*randn(1,N));

g_UAV_BS1 = sqrt(K_UAV_BS1/(1+K_UAV_BS1))*g + sqrt(1/(1+K_UAV_BS1))*g;
g_UAV_BS2 = sqrt(K_UAV_BS2/(1+K_UAV_BS2))*g + sqrt(1/(1+K_UAV_BS2))*g;
g_UAV_BS3 = sqrt(K_UAV_BS3/(1+K_UAV_BS3))*g + sqrt(1/(1+K_UAV_BS3))*g;

g_UAV_User1 = sqrt(K_UAV_User1/(1+K_UAV_User1))*g + sqrt(1/(1+K_UAV_User1))*g;
g_UAV_User2 = sqrt(K_UAV_User2/(1+K_UAV_User2))*g + sqrt(1/(1+K_UAV_User2))*g;
g_UAV_User3 = sqrt(K_UAV_User3/(1+K_UAV_User3))*g + sqrt(1/(1+K_UAV_User3))*g;


% Avarage Channel Power Gain

% Assume Path Loss Componet = 4
% Assume Average Channel Power Gain = -60dB

eta = 4;    % Path Loss Component
b0 = db2pow(0);  % Average channel power gain at a reference deistance d0 = 1m

% Rician fading for UAVs and Users

chPow_UAV_BS1 = b0*((LoS_Dis_UAV_BS1)^(-eta));
chPow_UAV_BS2 = b0*((LoS_Dis_UAV_BS2)^(-eta));
chPow_UAV_BS3 = b0*((LoS_Dis_UAV_BS3)^(-eta));

chPow_UAV_User1 = b0*((LoS_Dis_UAV_User1)^(-eta));
chPow_UAV_User2 = b0*((LoS_Dis_UAV_User2)^(-eta));
chPow_UAV_User3 = b0*((LoS_Dis_UAV_User3)^(-eta));


% Channel Coefficeint

h_UAV_BS1 = sqrt(chPow_UAV_BS1)*g_UAV_BS1;
h_UAV_BS2 = sqrt(chPow_UAV_BS2)*g_UAV_BS2;
h_UAV_BS3 = sqrt(chPow_UAV_BS3)*g_UAV_BS3;

h_UAV_User1 = sqrt(chPow_UAV_User1)*g_UAV_User1;
h_UAV_User2 = sqrt(chPow_UAV_User2)*g_UAV_User2;
h_UAV_User3 = sqrt(chPow_UAV_User3)*g_UAV_User3;

abs_h_UAV_BS1 = (abs(h_UAV_BS1)).^2;
abs_h_UAV_BS2 = (abs(h_UAV_BS2)).^2;
abs_h_UAV_BS3 = (abs(h_UAV_BS3)).^2;

abs_h_UAV_User1 = (abs(h_UAV_User1)).^2;
abs_h_UAV_User2 = (abs(h_UAV_User2)).^2;
abs_h_UAV_User3 = (abs(h_UAV_User3)).^2;

% Power Coefficeint by Channel gains

mean_abs_h_UAV_User1 = mean(abs_h_UAV_User1);
mean_abs_h_UAV_User2 = mean(abs_h_UAV_User2);
mean_abs_h_UAV_User3 = mean(abs_h_UAV_User3);

powerCoef_in1 = 1 - (mean_abs_h_UAV_User1/(mean_abs_h_UAV_User1+mean_abs_h_UAV_User2+mean_abs_h_UAV_User3));
powerCoef_in2 = 1 - (mean_abs_h_UAV_User2/(mean_abs_h_UAV_User1+mean_abs_h_UAV_User2+mean_abs_h_UAV_User3));
powerCoef_in3 = 1 - (mean_abs_h_UAV_User3/(mean_abs_h_UAV_User1+mean_abs_h_UAV_User2+mean_abs_h_UAV_User3));
    
powerCoef1 = (powerCoef_in1/(powerCoef_in1+powerCoef_in2+powerCoef_in3));
powerCoef2 = (powerCoef_in2/(powerCoef_in1+powerCoef_in2+powerCoef_in3));
powerCoef3 = (powerCoef_in3/(powerCoef_in1+powerCoef_in2+powerCoef_in3));


% kfmsekfms
T = 1*10^-6;

neta = 0.5;

B = 10^6;
No = -174 + 10*log10(B);
no = (10^-3)*db2pow(No);

psy = 0.1:0.01:0.9;
alpha = 0.1:0.01:0.9;

for i=1:length(psy)
    for j = 1:length(alpha)
    
        P_H = psy(i)*neta*pt_BS.*(abs_h_UAV_BS1+abs_h_UAV_BS2+abs_h_UAV_BS3) + psy(i)*neta*no;
        E_H = P_H*alpha(j)*T;
        
        pt = E_H/((1-alpha(j))*T);
        
        C1 = B*log2(1 + pt*powerCoef1.*abs_h_UAV_User1./(no));
        C1_mean(i,j) = mean(C1);
        
        C2 = B*log2(1 + pt*powerCoef2.*abs_h_UAV_User2./(no + pt*powerCoef1.*abs_h_UAV_User2));
        C2_mean(i,j) = mean(C2);
        
        C3 = B*log2(1 + pt*powerCoef3.*abs_h_UAV_User3./(no + pt*(powerCoef1 + powerCoef2).*abs_h_UAV_User3));
        C3_mean(i,j) = mean(C3);
        
        %sum_rate(i,j) = C1_mean+C2_mean+C3_mean;

    end
end

%mesh(psy,alpha,sum_rate);
figure;
mesh(psy,alpha,C1_mean);
figure;
mesh(psy,alpha,C2_mean);
figure;
mesh(psy,alpha,C3_mean);

%[X,Y] = meshgrid(psy,alpha) ;

%surf(X,Y,sum_rate)
%shading interp 
%colorbar
