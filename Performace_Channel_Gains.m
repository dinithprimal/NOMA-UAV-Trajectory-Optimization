clc;
clear variables;
close all;

%% Simulation Environment

enWidth = 2000;             % Simulation Environment Width (X axis)
enLength = 2000;            % Simulation Environment Length (Y axis)
enHeight = 320;             % Simulation Environment Height (Z axis)
maxHeighUAV = 300;          % Maximum Height of UAV
minHeighUAV = 150;           % Minimum Height of UAV
noUsers = 3;                % Number of Users
noBS = 3;                   % Number of Base Sations
noUAV = 1;                  % Number of UAVs

%% Users, Base Sataions and UAV Position

% UAV Position
for i=1:noUAV
    xUAV_S(i) = 1000;       % UAV position in X axis
    yUAV_S(i) = enLength;      % UAV position in Y axis
    zUAV_S(i) = 200;%randi([minHeighUAV,maxHeighUAV]);   % UAV position in Z axis
end

% UAV End Position
for i=1:noUAV
    xUAV_E(i) = 1000;       % UAV position in X axis
    yUAV_E(i) = 0;      % UAV position in Y axis
    zUAV_E(i) = 200;%randi([minHeighUAV,maxHeighUAV]);   % UAV position in Z axis
end

% UAV Position
for i=1:noUAV
    xUAV(i) = 1000;       % UAV position in X axis
    yUAV(i) = enLength;      % UAV position in Y axis
    zUAV(i) = zUAV_S(i);   % UAV position in Z axis
end

% BS positions
xBS = [2000,0,2000];
yBS = [2000,1000,0];
zBS = [20, 30, 25];

% Users' posision
for i=1:noUsers
    xUser(i) = randi(enWidth);      % User position in X axis
    yUser(i) = randi(enLength);     % User position in Y axis
    zUser(i) = 0;                   % User position in Z axis = 0
end

%% Figure Plot in Initial State

% User plotting
figure,
userPlot = plot3(xUser,yUser,zUser,'r*','linewidth',3); hold on;
for o=1:noUsers
    textUsers(o) = text(xUser(o)-10,yUser(o)-10,zUser(o)+10,['U',num2str(o)],'FontSize', 8);
    textUsAngle(o) = text(xUser(o)+10,yUser(o)+10,['U_{\theta_{',num2str(o),'}}'],'FontSize', 6);
end

% UAVs Plotting
plot3(xUAV_S, yUAV_S, zUAV_S, 'bh','linewidth', 3);
for o=1:noUAV
    text(xUAV_S(o) + 20, yUAV_S(o) + 10, zUAV_S(o) + 10,['UAV',num2str(o),' Start Position']);
end

plot3(xUAV_E, yUAV_E, zUAV_E, 'bh','linewidth', 3);
for o=1:noUAV
    text(xUAV_E(o) + 20, yUAV_E(o) + 10, zUAV_E(o) + 10,['UAV',num2str(o),' End Position']);
end

% BSs Plotting
plot3(xBS, yBS, zBS, 'b*','linewidth', 3);
for o=1:noBS
    plot3([xBS(o) xBS(o)], [yBS(o) yBS(o)],[zBS(o) 0], 'bl-','linewidth', 1); drawnow
    text(xBS(o) + 20, yBS(o) + 10, zBS(o) + 10,['BS', num2str(o)],'FontSize', 8);
    textBSAngle(o) = text(xBS(o)+10,yBS(o)+10,zBS(o)-5,['BS_{\theta_{',num2str(o),'}}'],'FontSize', 6);
end

% Plotting LoS Channel between BSs and UAVs
for i = 1:noUAV
    for directBS = 1:noBS
        plot3([xUAV(i) xBS(directBS)], [yUAV(i) yBS(directBS)],[zUAV(i) zBS(directBS)], 'b:','linewidth', 0.1); drawnow
    end
end



% Plotting LoS Channel between users and UAVs
for i = 1:noUAV
    for directUsers = 1:noUsers
        plot3([xUAV(i) xUser(directUsers)], [yUAV(i) yUser(directUsers)],[zUAV(i) zUser(directUsers)], 'r--','linewidth', 0.1); drawnow
    end
end

% Plotting UAV default path
for i = 1:noUAV
    plot3([xUAV_S(i) xUAV_E(i)], [yUAV_S(i) yUAV_E(i)],[zUAV_S(i) zUAV_E(i)], 'y--','linewidth', 2); drawnow
end

mx = (zUAV_S - zUAV_E)/(yUAV_S - yUAV_E);
c = zUAV_S - mx*yUAV_S;

%eqY = m*x + c;

% 3D Plot labels
xlim([0 2050])
ylim([0 2050])
zlim([0 320])
xlabel('Width (m)');
ylabel('Length (m)');
zlabel('Height (m)');
grid on; hold on; drawnow

index = 1;

%% Information for path loss model

eta = 2.5;              % Path Loss Component
b_0dB = -50;            % Reference Channel Gain in dB
b_0 = db2pow(b_0dB);    % Reference Channel Gain in linear scale
k = 0.01;               % Additional attenuation for NLoS

%% Infromation for Rician Fading Model

% Rician factor
K_min = 4;      % K_min value in dB
K_max = 12;     % K_max value in dB

A1 = db2pow(K_min);
A2 = (2/pi)*log((db2pow(K_max))/A1);

%N = 10^5;
g = sqrt(1/2)*(randn(1,1)+1i*randn(1,1));

%% Simulation

for j = enLength:-100:0
    for i=1:noUAV
        zUAV(i) = mx*j + c;
        yUAV(i) = j;
        plot3(xUAV(i),  yUAV(i),  zUAV(i), 'gh','linewidth', 1);hold on;

        % LoS distance between UAVs and BSs
        for bm=1:noBS
            groundDisUAV_BS(i,bm) = sqrt((xUAV(i)-xBS(bm))^2 + (yUAV(i)-yBS(bm))^2);
            DisUAV_BS(i,bm) = sqrt(groundDisUAV_BS(i,bm)^2 + (zBS(bm)-zUAV(i))^2);
            
            % Elavation Angle in radiant between UAVs and Users
            angleUAV_BS(i,bm) = atan(abs(zBS(bm)-zUAV(i))/groundDisUAV_BS(i,bm))*(180/pi);
            
            PLoS_BS(index,i,bm) = 1/(1+(10*exp(-0.6*(angleUAV_BS(i,bm)-10))));
            
            pow_LoS_BS = b_0*(DisUAV_BS(i,bm)^(-eta));
            pow_NLoS_BS = k*b_0*(DisUAV_BS(i,bm)^(-eta));
            Ch_pow_LoS_BS(index,i,bm) = pow2db(pow_LoS_BS); 
            Ch_pow_NLoS_BS(index,i,bm) = pow2db(pow_NLoS_BS);
            
            % Expected Path Loss Channel gain
            E_bd_BS = PLoS_BS(index,i,bm)*pow_LoS_BS + (1 - PLoS_BS(index,i,bm))*pow_NLoS_BS;
            E_bd_dB_BS(index,i,bm) = pow2db(E_bd_BS); % in dB
            
            % Angle depend rician factor
            K_UAV_BS(i,bm) = A1*exp(A2*angleUAV_BS(i,bm)*(pi/180));
            
            g_UAV_BS(i,bm) = sqrt(K_UAV_BS(i,bm)/(1+K_UAV_BS(i,bm)))*g + sqrt(1/(1+K_UAV_BS(i,bm)))*g;
            
            h_UAV_BS(index,i,bm) = sqrt(pow_LoS_BS)*g_UAV_BS(i,bm);
            
            h_UAV_BS_dB(index,i,bm) = pow2db(abs(h_UAV_BS(index,i,bm))^2);
            
        end
        
        % LoS distance between UAVs and Users
        for m=1:noUsers
            groundDisUAV_User(i,m) = sqrt((xUAV(i)-xUser(m))^2 + (yUAV(i)-yUser(m))^2);
            DisUAV_User(i,m) = sqrt(groundDisUAV_User(i,m)^2 + zUAV(i)^2);
            
            % Elavation Angle in radiant between UAVs and Users
            angleUAV_User(i,m) = atan(zUAV(i)/groundDisUAV_User(i,m))*(180/pi);
            
            
            PLoS(index,i,m) = 1/(1+(10*exp(-0.6*(angleUAV_User(i,m)-10))));
            
            pow_LoS = b_0*(DisUAV_User(i,m)^(-eta));
            pow_NLoS = k*b_0*(DisUAV_User(i,m)^(-eta));
            Ch_pow_LoS(index,i,m) = pow2db(pow_LoS); 
            Ch_pow_NLoS(index,i,m) = pow2db(pow_NLoS);
            
            % Expected Path Loss Channel gain
            E_bd = PLoS(index,i,m)*pow_LoS + (1 - PLoS(index,i,m))*pow_NLoS;
            E_bd_dB(index,i,m) = pow2db(E_bd); % in dB
            
            
            % Angle depend rician factor
            K_UAV_User(i,m) = A1*exp(A2*angleUAV_User(i,m)*(pi/180));
            
            g_UAV_User(i,m) = sqrt(K_UAV_User(i,m)/(1+K_UAV_User(i,m)))*g + sqrt(1/(1+K_UAV_User(i,m)))*g;
            
            h_UAV_Users(index,i,m) = sqrt(pow_LoS)*g_UAV_User(i,m);
            
            h_UAV_Users_dB(index,i,m) = pow2db(abs(h_UAV_Users(index,i,m))^2);
           
        end
        
        % Power coefficeints calculation for users
        [pow_coef_array_ch(index,:), pow_coef_array_fr(index,:)] = findPowCoeff(abs(h_UAV_Users(index,i,:)),noUsers);
        
        % Achievable Rate Calculations for Users
        [achievableRate_ch(index,:), achievableRate_fr(index,:)] = findAchievableRate(h_UAV_Users(index,i,:),pow_coef_array_ch(index,:),pow_coef_array_fr(index,:),noUsers);
        
        % Achievable Rate Calculations for BSs
        achievableRate_BS(index,:) = findAchievableRate_BS(h_UAV_BS(index,i,:),noBS);
        
        % Achievable Rate Calculations for Users in SWIPT model
        [achievableRate_ch_SWIPT(index,:), achievableRate_fr_SWIPT(index,:)] = findAchievableRate_SWIPT(h_UAV_Users(index,i,:),h_UAV_BS(index,i,:),pow_coef_array_ch(index,:),pow_coef_array_fr(index,:),noUsers);
        
    end
    index = index +1;
end

% PLoS for Users
steps = 1:21;
figure;
plot(steps,PLoS(:,1,1),'linewidth', 1);hold on;
plot(steps,PLoS(:,1,2),'linewidth', 1);
plot(steps,PLoS(:,1,3),'linewidth', 1);
xlim([1 21])
ylim([0 1])
legend('U 1', 'U 2', 'U 3');
grid on;

% PLoS for BSs
figure;
plot(steps,PLoS_BS(:,1,1),'linewidth', 1);hold on;
plot(steps,PLoS_BS(:,1,2),'linewidth', 1);
plot(steps,PLoS_BS(:,1,3),'linewidth', 1);
xlim([1 21])
ylim([0 0.2])
legend('BS 1', 'BS 2', 'BS 3');
grid on;

% Channel power for users
figure;
plot(steps,Ch_pow_LoS(:,1,1),'linewidth', 1);hold on;
plot(steps,Ch_pow_LoS(:,1,2),'linewidth', 1);
plot(steps,Ch_pow_LoS(:,1,3),'linewidth', 1);
plot(steps,Ch_pow_NLoS(:,1,1),'linewidth', 1);hold on;
plot(steps,Ch_pow_NLoS(:,1,2),'linewidth', 1);
plot(steps,Ch_pow_NLoS(:,1,3),'linewidth', 1);
xlim([1 21])
%ylim([0 1])
legend('U 1', 'U 2', 'U 3', 'U 1 N', 'U 2 N', 'U 3 N');
grid on;

% PLoS Channel power for BSs
figure;
plot(steps,Ch_pow_LoS_BS(:,1,1),'linewidth', 1);hold on;
plot(steps,Ch_pow_LoS_BS(:,1,2),'linewidth', 1);
plot(steps,Ch_pow_LoS_BS(:,1,3),'linewidth', 1);
plot(steps,Ch_pow_NLoS_BS(:,1,1),'linewidth', 1);hold on;
plot(steps,Ch_pow_NLoS_BS(:,1,2),'linewidth', 1);
plot(steps,Ch_pow_NLoS_BS(:,1,3),'linewidth', 1);
xlim([1 21])
%ylim([0 1])
legend('BS 1', 'BS 2', 'BS 3', 'BS 1 N', 'BS 2 N', 'BS 3 N');
grid on;

% Channel power Users
figure;
plot(steps,E_bd_dB(:,1,1),'linewidth', 1);hold on;
plot(steps,E_bd_dB(:,1,2),'linewidth', 1);
plot(steps,E_bd_dB(:,1,3),'linewidth', 1);

plot(steps,h_UAV_Users_dB(:,1,1),'--^','linewidth', 1);hold on;
plot(steps,h_UAV_Users_dB(:,1,2),'--^','linewidth', 1);
plot(steps,h_UAV_Users_dB(:,1,3),'--^','linewidth', 1);

xlim([1 21])
%ylim([0 1])
legend('U 1', 'U 2', 'U 3', 'U 1 R', 'U 2 R', 'U 3 R');
grid on;

% Channel power BSs
figure;
plot(steps,E_bd_dB_BS(:,1,1),'linewidth', 1);hold on;
plot(steps,E_bd_dB_BS(:,1,2),'linewidth', 1);
plot(steps,E_bd_dB_BS(:,1,3),'linewidth', 1);

plot(steps,h_UAV_BS_dB(:,1,1),'--^','linewidth', 1);hold on;
plot(steps,h_UAV_BS_dB(:,1,2),'--^','linewidth', 1);
plot(steps,h_UAV_BS_dB(:,1,3),'--^','linewidth', 1);

xlim([1 21])
%ylim([0 1])
legend('BS 1', 'BS 2', 'BS 3', 'BS 1 R', 'BS 2 R', 'BS 3 R');
grid on;

% Achievable rate for Users
figure;
plot(steps,achievableRate_ch(:,1),'*-','linewidth', 1);hold on;
plot(steps,achievableRate_ch(:,2),'*-','linewidth', 1);
plot(steps,achievableRate_ch(:,3),'*-','linewidth', 1);

plot(steps,achievableRate_fr(:,1),'^-','linewidth', 1);hold on;
plot(steps,achievableRate_fr(:,2),'^-','linewidth', 1);
plot(steps,achievableRate_fr(:,3),'^-','linewidth', 1);

xlim([1 21])
%ylim([0 1])
legend('U 1', 'U 2', 'U 3','U 1 fr', 'U 2 fr', 'U 3 fr');
grid on;


% Achievable rate for BSs
figure;
plot(steps,achievableRate_BS(:,1),'*-','linewidth', 1);hold on;
plot(steps,achievableRate_BS(:,2),'*-','linewidth', 1);
plot(steps,achievableRate_BS(:,3),'*-','linewidth', 1);

xlim([1 21])
%ylim([0 1])
legend('BS 1', 'BS 2', 'BS 3');
grid on;


% Achievable rate for Users SWIPT Model
figure;
plot(steps,achievableRate_ch_SWIPT(:,1),'*-','linewidth', 1);hold on;
plot(steps,achievableRate_ch_SWIPT(:,2),'*-','linewidth', 1);
plot(steps,achievableRate_ch_SWIPT(:,3),'*-','linewidth', 1);

plot(steps,achievableRate_fr_SWIPT(:,1),'^-','linewidth', 1);hold on;
plot(steps,achievableRate_fr_SWIPT(:,2),'^-','linewidth', 1);
plot(steps,achievableRate_fr_SWIPT(:,3),'^-','linewidth', 1);

xlim([1 21])
%ylim([0 1])
legend('U 1', 'U 2', 'U 3','U 1 fr', 'U 2 fr', 'U 3 fr');
grid on;