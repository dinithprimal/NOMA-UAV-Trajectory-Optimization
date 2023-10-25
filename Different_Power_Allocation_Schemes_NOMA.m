clc;
clear all
close all

% Ground Distances from UAV in meters
g_d1 = 100;
g_d2 = 500;
g_d3 = 1000;

% UAV height in meters
height_UAV = 45;

% Los Distance between BS and Ground Users
LoS_Dis_UAV_User1 = sqrt(g_d1^2 + height_UAV^2);
LoS_Dis_UAV_User2 = sqrt(g_d2^2 + height_UAV^2);
LoS_Dis_UAV_User3 = sqrt(g_d3^2 + height_UAV^2);

% Angle

angle_UAV_User1 = atan(height_UAV/g_d1);
angle_UAV_User2 = atan(height_UAV/g_d2);
angle_UAV_User3 = atan(height_UAV/g_d3);

% Angle-depend rician factor for Users and BSs
A1 = 1;
A2 = (log(db2pow(60)/A1))/(pi/2);

K_UAV_User1 = A1*exp(A2*angle_UAV_User1);
K_UAV_User2 = A1*exp(A2*angle_UAV_User2);
K_UAV_User3 = A1*exp(A2*angle_UAV_User3);

% Rician Fading for Users and BSs

N = 10^5;
g = sqrt(1/2)*(randn(1,N)+1i*randn(1,N));

g_UAV_User1 = sqrt(K_UAV_User1/(1+K_UAV_User1))*g + sqrt(1/(1+K_UAV_User1))*g;
g_UAV_User2 = sqrt(K_UAV_User2/(1+K_UAV_User2))*g + sqrt(1/(1+K_UAV_User2))*g;
g_UAV_User3 = sqrt(K_UAV_User3/(1+K_UAV_User3))*g + sqrt(1/(1+K_UAV_User3))*g;

% Avarage Channel Power Gain

% Assume Path Loss Componet = 4
% Assume Average Channel Power Gain = -60dB

eta = 4;    % Path Loss Component
b0 = (10^-3)*db2pow(20);  % Average channel power gain at a reference deistance d0 = 1m


% Rician fading for UAVs and Users

chPow_UAV_User1 = b0*((LoS_Dis_UAV_User1)^(-eta));
chPow_UAV_User2 = b0*((LoS_Dis_UAV_User2)^(-eta));
chPow_UAV_User3 = b0*((LoS_Dis_UAV_User3)^(-eta));

% Channel Coefficeint

h_UAV_Users1 = sqrt(chPow_UAV_User1)*g_UAV_User1;
h_UAV_Users2 = sqrt(chPow_UAV_User2)*g_UAV_User2;
h_UAV_Users3 = sqrt(chPow_UAV_User3)*g_UAV_User3;

abs_h_UAV_Users1 = (abs(h_UAV_Users1)).^2;
abs_h_UAV_Users2 = (abs(h_UAV_Users2)).^2;
abs_h_UAV_Users3 = (abs(h_UAV_Users3)).^2;

%% Channel coeffiecent value sorting

mean_abs_h_UAV_Users1 = mean(abs_h_UAV_Users1);
mean_abs_h_UAV_Users2 = mean(abs_h_UAV_Users2);
mean_abs_h_UAV_Users3 = mean(abs_h_UAV_Users3);

%% Power Coefficeint by Channel gains

powerCoef_in1 = 1 - (mean_abs_h_UAV_Users1/(mean_abs_h_UAV_Users1+mean_abs_h_UAV_Users2+mean_abs_h_UAV_Users3));
powerCoef_in2 = 1 - (mean_abs_h_UAV_Users2/(mean_abs_h_UAV_Users1+mean_abs_h_UAV_Users2+mean_abs_h_UAV_Users3));
powerCoef_in3 = 1 - (mean_abs_h_UAV_Users3/(mean_abs_h_UAV_Users1+mean_abs_h_UAV_Users2+mean_abs_h_UAV_Users3));
    
powerCoef1 = (powerCoef_in1/(powerCoef_in1+powerCoef_in2+powerCoef_in3));
powerCoef2 = (powerCoef_in2/(powerCoef_in1+powerCoef_in2+powerCoef_in3));
powerCoef3 = (powerCoef_in3/(powerCoef_in1+powerCoef_in2+powerCoef_in3));

%% Achievable Rate Calculation

% Fixed Power Allocations

a1 = 0.1;
a2 = 0.2;
a3 = 0.7;

av3 = 0.8;
av2 = 0.8*(1 - av3);
av1 = 1 - (av2 + av3);

Pt = 0:2:60;     %in dBm
pt = (10^-3)*db2pow(Pt);	%in linear scale

B = 10^6;
No = -174 + 10*log10(B);
no = (10^-3)*db2pow(No);

for u = 1:length(pt)
    % By Channel Power
    C1 = B*log2(1 + pt(u)*powerCoef1.*abs_h_UAV_Users1./(no));
    C1_mean(u) = mean(C1);
    
    C2 = B*log2(1 + pt(u)*powerCoef2.*abs_h_UAV_Users2./(no + pt(u)*powerCoef1.*abs_h_UAV_Users2));
    C2_mean(u) = mean(C2);
    
    C3 = B*log2(1 + pt(u)*powerCoef3.*abs_h_UAV_Users3./(no + pt(u)*(powerCoef1 + powerCoef2).*abs_h_UAV_Users3));
    C3_mean(u) = mean(C3);
    
    % Fixed Power
    C1_f = B*log2(1 + pt(u)*a1.*abs_h_UAV_Users1./(no));
    C1_mean_f(u) = mean(C1_f);
    
    C2_f = B*log2(1 + pt(u)*a2.*abs_h_UAV_Users2./(no + pt(u)*a1.*abs_h_UAV_Users2));
    C2_mean_f(u) = mean(C2_f);
    
    C3_f = B*log2(1 + pt(u)*a3.*abs_h_UAV_Users3./(no + pt(u)*(a1 + a2).*abs_h_UAV_Users3));
    C3_mean_f(u) = mean(C3_f);
    
    % Variable Fixed Power
    C1_vf = B*log2(1 + pt(u)*av1.*abs_h_UAV_Users1./(no));
    C1_mean_vf(u) = mean(C1_vf);
    
    C2_vf = B*log2(1 + pt(u)*av2.*abs_h_UAV_Users2./(no + pt(u)*av1.*abs_h_UAV_Users2));
    C2_mean_vf(u) = mean(C2_vf);
    
    C3_vf = B*log2(1 + pt(u)*av3.*abs_h_UAV_Users3./(no + pt(u)*(av1 + av2).*abs_h_UAV_Users3));
    C3_mean_vf(u) = mean(C3_vf);
end

%% Plot

figure;
plot(Pt,C1_mean,'g-^','linewidth',1.1); hold on; grid on;
plot(Pt,C2_mean,'g-square','linewidth',1.1);
plot(Pt,C3_mean,'g-diamond','linewidth',1.1);

plot(Pt,C1_mean_f,'m--^','linewidth',1.1); hold on; grid on;
plot(Pt,C2_mean_f,'m--square','linewidth',1.1);
plot(Pt,C3_mean_f,'m--diamond','linewidth',1.1);

plot(Pt,C1_mean_vf,'c:^','linewidth',1.1); hold on; grid on;
plot(Pt,C2_mean_vf,'c:square','linewidth',1.1);
plot(Pt,C3_mean_vf,'c:diamond','linewidth',1.1);

title('Achievable Rate for GUs in different Power Coefficeint Allocation Schemes')
ylabel('Achievable Rate in bps')
xlabel('Transmit Power in dBm')

legend({'User1 - Near User (Channel gain based power coefficent allocation)','User2 - Middle User (Channel gain based power coefficent allocation)','User3 - Far User (Channel gain based power coefficient allocation)','User1 - Near User (Fraction power coefficeint allocation)','User2 - Middle User (Fraction power coefficeint allocation)','User3 - Far User (Fraction power coefficeint allocation)','User1 - Near User (Fixed power coefficeint allocation)','User2 - Middle User (Fixed power coefficeint allocation)','NOMA User3 - Far User (Fixed power coefficeint allocation)'},'location','northwest');


sum_c = C1_mean + C2_mean + C3_mean;
sum_c_f = C1_mean_f + C2_mean_f + C3_mean_f;
sum_c_vf = C1_mean_vf + C2_mean_vf + C3_mean_vf;

figure;
plot(Pt,sum_c,'g-^','linewidth',1.1); hold on; grid on;
plot(Pt,sum_c_f,'g-square','linewidth',1.1);
plot(Pt,sum_c_vf,'g-diamond','linewidth',1.1);

title('Sum Rate for GUs in different Power Coefficeint Allocation Schemes')
ylabel('Achievable Rate in bps')
xlabel('Transmit Power in dBm')
legend({'Channel gain based power coefficent allocation','Fraction power coefficeint allocation','Fixed power coefficeint allocation'},'location','northwest');

%% BER Calculation

% BER over QPSK Modulation

x1 = randi([0 1],N,1);
x2 = randi([0 1],N,1);
x3 = randi([0 1],N,1);

QPSKmod = comm.QPSKModulator('BitInput',true);       %Modulator object
QPSKdemod = comm.QPSKDemodulator('BitOutput',true);  %Demodulator object

xmod1 = step(QPSKmod, x1);
xmod2 = step(QPSKmod, x2);
xmod3 = step(QPSKmod, x3);

n1 = sqrt(no)*(randn(N/2,1) + 1i*randn(N/2,1))/sqrt(2);
n2 = sqrt(no)*(randn(N/2,1) + 1i*randn(N/2,1))/sqrt(2);
n3 = sqrt(no)*(randn(N/2,1) + 1i*randn(N/2,1))/sqrt(2);

x = sqrt(powerCoef1)*xmod1 + sqrt(powerCoef2)*xmod2 + sqrt(powerCoef3)*xmod3;

g1 = sqrt(1/2)*(randn(1,N/2)+1i*randn(1,N/2));

g_UAV_User11 = sqrt(K_UAV_User1/(1+K_UAV_User1))*g1 + sqrt(1/(1+K_UAV_User1))*g1;
g_UAV_User21 = sqrt(K_UAV_User2/(1+K_UAV_User2))*g1 + sqrt(1/(1+K_UAV_User2))*g1;
g_UAV_User31 = sqrt(K_UAV_User3/(1+K_UAV_User3))*g1 + sqrt(1/(1+K_UAV_User3))*g1;

% Avarage Channel Power Gain

% Assume Path Loss Componet = 4
% Assume Average Channel Power Gain = -60dB

% Channel Coefficeint

h_UAV_Users11 = sqrt(chPow_UAV_User1)*g_UAV_User11;
h_UAV_Users21 = sqrt(chPow_UAV_User2)*g_UAV_User21;
h_UAV_Users31 = sqrt(chPow_UAV_User3)*g_UAV_User31;



for u = 1:length(pt)
    
    y1 = sqrt(pt(u))*x.*h_UAV_Users11' + n1;
    y2 = sqrt(pt(u))*x.*h_UAV_Users21' + n2;
    y3 = sqrt(pt(u))*x.*h_UAV_Users31' + n3;
    
    eq1 = y1./h_UAV_Users11';
    eq2 = y2./h_UAV_Users21';
    eq3 = y3./h_UAV_Users31';
    
    dec1 = step(QPSKdemod, eq1);
    
    dec12 = step(QPSKdemod, eq2);
    
    dec12_remod = step(QPSKmod, dec12);
    
    rem2 = eq2 - sqrt(powerCoef1*pt(u))*dec12_remod;
    dec2 = step(QPSKdemod, rem2);    %Demodulate rem2 to get dec2
    
    dec13 = step(QPSKdemod, eq3);
    dec13_remod = step(QPSKmod, dec13);
    rem31 = eq3 - sqrt(powerCoef1*pt(u))*dec12_remod;
    
    dec23 = step(QPSKdemod, rem31);
    dec23_remod = step(QPSKmod, dec23);
    rem3 = rem31 - sqrt(powerCoef2*pt(u))*dec23_remod;   
    
    dec3 = step(QPSKdemod, rem3);    %Demodulate rem3 to get dec3
    
    ber1(u) = biterr(dec1, x1)/N;
    ber2(u) = biterr(dec2, x2)/N;
    ber3(u) = biterr(dec3, x3)/N;
        
end

figure;
semilogy(Pt,ber1,'g-^','linewidth',1.1); hold on; grid on;
semilogy(Pt,ber2,'c-square','linewidth',1.1);
semilogy(Pt,ber3,'m-diamond','linewidth',1.1);