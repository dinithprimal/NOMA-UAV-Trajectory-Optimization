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
hLOS1 = exp(1i*2*pi*rand(1,1));
hLOS2 = exp(1i*2*pi*rand(1,1));
hLOS3 = exp(1i*2*pi*rand(1,1));
% 
% hLOS1_QAM_4 = exp(1i*2*pi*rand(1,N/2));
% hLOS2_QAM_4 = exp(1i*2*pi*rand(1,N/2));
% hLOS3_QAM_4 = exp(1i*2*pi*rand(1,N/2));


hNLOS1 = sqrt(1/2)*(randn(1,1)+1i*randn(1,1));
hNLOS2 = sqrt(1/2)*(randn(1,1)+1i*randn(1,1));
hNLOS3 = sqrt(1/2)*(randn(1,1)+1i*randn(1,1));

% hNLOS1_QAM_4 = sqrt(1/2)*(randn(1,N/2)+1i*randn(1,N/2));
% hNLOS2_QAM_4 = sqrt(1/2)*(randn(1,N/2)+1i*randn(1,N/2));
% hNLOS3_QAM_4 = sqrt(1/2)*(randn(1,N/2)+1i*randn(1,N/2));


g_UAV_BS1 = sqrt(K_UAV_BS1/(1+K_UAV_BS1))*hLOS1 + sqrt(1/(1+K_UAV_BS1))*hNLOS1;
g_UAV_BS2 = sqrt(K_UAV_BS2/(1+K_UAV_BS2))*hLOS2 + sqrt(1/(1+K_UAV_BS2))*hNLOS2;
g_UAV_BS3 = sqrt(K_UAV_BS3/(1+K_UAV_BS3))*hLOS3 + sqrt(1/(1+K_UAV_BS3))*hNLOS3;

% g_UAV_BS1_QAM_4 = sqrt(K_UAV_BS1/(1+K_UAV_BS1))*hLOS1_QAM_4 + sqrt(1/(1+K_UAV_BS1))*hNLOS1_QAM_4;
% g_UAV_BS2_QAM_4 = sqrt(K_UAV_BS2/(1+K_UAV_BS2))*hLOS2_QAM_4 + sqrt(1/(1+K_UAV_BS2))*hNLOS2_QAM_4;
% g_UAV_BS3_QAM_4 = sqrt(K_UAV_BS3/(1+K_UAV_BS3))*hLOS3_QAM_4 + sqrt(1/(1+K_UAV_BS3))*hNLOS3_QAM_4;

% Avarage Channel Power Gain

% Assume Path Loss Componet = 4
% Assume Average Channel Power Gain = -60dB

eta = 4;    % Path Loss Component
b0 = db2pow(-50);  % Average channel power gain at a reference deistance d0 = 1m


% Rician fading for UAVs and BSs

chPow_UAV_BS1 = b0*((LoS_Dis_UAV_BS1)^(-eta));
chPow_UAV_BS2 = b0*((LoS_Dis_UAV_BS2)^(-eta));
chPow_UAV_BS3 = b0*((LoS_Dis_UAV_BS3)^(-eta));

% Channel Coefficeint

h_UAV_BS1 = sqrt(chPow_UAV_BS1)*g_UAV_BS1;
h_UAV_BS2 = sqrt(chPow_UAV_BS2)*g_UAV_BS2;
h_UAV_BS3 = sqrt(chPow_UAV_BS3)*g_UAV_BS3;

% h_UAV_BS1_QAM_4 = sqrt(chPow_UAV_BS1)*g_UAV_BS1_QAM_4;
% h_UAV_BS2_QAM_4 = sqrt(chPow_UAV_BS2)*g_UAV_BS2_QAM_4;
% h_UAV_BS3_QAM_4 = sqrt(chPow_UAV_BS3)*g_UAV_BS3_QAM_4;

%abs_h_UAV_BS1 = (abs(h_UAV_BS1)).^2;
%abs_h_UAV_BS2 = (abs(h_UAV_BS2)).^2;
%abs_h_UAV_BS3 = (abs(h_UAV_BS3)).^2;

Pt = 100;                    %in dBm
pt = (10^-3)*db2pow(Pt);	%in linear scale

B = 10^6;                   % Bandwidth
No = -174 + 10*log10(B);    % Noise in dBm
no = (10^-3)*db2pow(No);    % Noise in linear scale

%Generate noise samples
n1 = sqrt(no)*(randn(N,1) + 1i*randn(N,1))/sqrt(2);
%n1_QAM_4 = sqrt(no)*(randn(N/2,1) + 1i*randn(N/2,1))/sqrt(2);

%eff = 0.7;  %power harvesting efficiency

%Generate random binary message data from the three BSs
x1 = randi([0 1],N,1);
x2 = randi([0 1],N,1);
x3 = randi([0 1],N,1);

%Create QPSKModulator and QPSKDemodulator objects
%QPSKmod = comm.QPSKModulator('BitInput',true); 
%QPSKdemod = comm.QPSKDemodulator('BitOutput',true); 

%Perform QPSK modulation
xmod1 = 2*x1 - 1;
xmod2 = 2*x2 - 1;
xmod3 = 2*x3 - 1;

% Perform 4-Array QAM modulation
% M = 4;
% 
% xmod4QAM1 = qammod(x1,M,'InputType','bit','UnitAveragePower',true);
% xmod4QAM2 = qammod(x2,M,'InputType','bit','UnitAveragePower',true);
% xmod4QAM3 = qammod(x3,M,'InputType','bit','UnitAveragePower',true);



sigma = 0:0.05:0.9;

for u = 1:length(sigma)
    % Received signal at UAV
    y = sqrt(1-sigma(u))*(sqrt(pt)*xmod1*h_UAV_BS1 + sqrt(pt)*xmod2*h_UAV_BS2 + sqrt(pt)*xmod3*h_UAV_BS3) + n1;
    %y_QAM_4 = sqrt(1-sigma(u))*(sqrt(pt)*xmod4QAM1.*transpose(h_UAV_BS1_QAM_4) + sqrt(pt)*xmod4QAM2.*transpose(h_UAV_BS2_QAM_4) + sqrt(pt)*xmod4QAM3.*transpose(h_UAV_BS3_QAM_4)) + n1_QAM_4;
    
    % Perform equalization and decoding for BS1 (Near BS)
    eq1 = y/(h_UAV_BS1+h_UAV_BS2+h_UAV_BS3);
    %eq1_QAM_4 = y_QAM_4./(transpose(h_UAV_BS1_QAM_4));
    
    dec_BS1 = zeros(N,1);
    dec_BS1(eq1>0) = 1;
    %dec_BS1 = step(QPSKdemod, eq1);     % Directly demodulate the signal
    %dec_BS1_QAM4 = qamdemod(eq1_QAM_4,M,'bin','OutputType','bit'); %step(QPSKdemod, eq1);
    
    
    
    % Perform equalization and decoding for BS2 (Middle BS)
    %eq2 = y./(transpose(h_UAV_BS1+h_UAV_BS2+h_UAV_BS3));
    %eq2_QAM_4 = y_QAM_4./(transpose(h_UAV_BS2_QAM_4));
    
    %dec_BS1_2 = step(QPSKdemod, eq2);		%Direct demodulation to get BS1's data
    %dec_BS1_2_QAM_4 = qamdemod(eq2_QAM_4,M,'bin','OutputType','bit');
    
    dec12_BS1_2_remod = 2*dec_BS1 - 1;  %step(QPSKmod, dec_BS1);%dec_BS1_2);		%Remodulation of BS1's data
    %dec12_BS1_2_remod_QAM_4 = qammod(dec_BS1_2_QAM_4,M,'InputType','bit','UnitAveragePower',true);
    
    y2 = y - sqrt(pt)*dec12_BS1_2_remod*h_UAV_BS1;	%SIC to remove BS1's data
    %rem_BS2_QAM_4 = eq2_QAM_4 - sqrt(0.5)*sqrt(pt)*dec12_BS1_2_remod_QAM_4;
    
    eq2 = y2/h_UAV_BS2+h_UAV_BS3;
    
    dec_BS2 = zeros(N,1);
    dec_BS2(eq2>0) = 1;
    %dec_BS2 = step(QPSKdemod, rem_BS2);		%Direct demodulation of remaining signal
    %dec_BS2_QAM_4 = qamdemod(rem_BS2_QAM_4,M,'bin','OutputType','bit');
    
    % Perform equalization and decoding for BS3 (Far BS)
    %eq3 = y./(transpose(h_UAV_BS1+h_UAV_BS2+h_UAV_BS3));
    %dec_BS1_3 = step(QPSKdemod, eq3);		%Direct demodulation to get BS1's data
    %dec_BS1_3_remod = step(QPSKmod, dec_BS1_3);		%Remodulation of BS1's data
    %rem_BS31 = eq3 - sqrt(0.75)*dec_BS1_3_remod;	%SIC to remove BS1's data
    %dec_BS2_3 = step(QPSKdemod, rem_BS31);		%Direct demodulation of remaining signal to get BS2's data
    dec_BS2_3_remod = 2*dec_BS2 - 1; %step(QPSKmod, dec_BS2);		%Remodulation of BS2's data
    y3 = y2 - sqrt(pt)*dec_BS2_3_remod*h_UAV_BS3;	%SIC to remove BS2's data
    %dec_BS3 = step(QPSKdemod, rem_BS3);		%Demodulate remaining signal to get BS3's data
    eq3 = y3/h_UAV_BS3;
    
    dec_BS3 = zeros(N,1);
    dec_BS3(eq3>0) = 1;
    %BER calculation
    ber1(u) = biterr(dec_BS1, x1)/N;
    ber2(u) = biterr(dec_BS2, x2)/N;
    ber3(u) = biterr(dec_BS3, x3)/N;
    
%     ber1_QAM_4(u) = biterr(dec_BS1_QAM4, x1)/N;
%     ber2_QAM_4(u) = biterr(dec_BS2_QAM_4, x2)/N;
    
end

plot(sigma, ber1, '-^', 'linewidth', 1); hold on; grid on;
plot(sigma, ber2, '-^', 'linewidth', 1);
plot(sigma, ber3, '-^', 'linewidth', 1);

% semilogy(sigma, ber1_QAM_4, '-*', 'linewidth', 2);
% semilogy(sigma, ber2_QAM_4, '-*', 'linewidth', 2);

xlabel('Power Harvesting Fraction');
ylabel('BER');
legend('BS 1 (Near BS)', 'BS 2', 'BS 3 (Far BS)');

