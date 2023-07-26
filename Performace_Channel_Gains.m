clc;
clear variables;
close all;

%% Simulation Environment

enWidth = 2000;             % Simulation Environment Width (X axis)
enLength = 2000;            % Simulation Environment Length (Y axis)
enHeight = 320;             % Simulation Environment Height (Z axis)
maxHeighUAV = 300;          % Maximum Height of UAV
minHeighUAV = 50;           % Minimum Height of UAV
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
    xUser(i) = randi([enWidth/4, 3*enWidth/4]);      % User position in X axis
    yUser(i) = randi([enLength/4, 3*enLength/4]);     % User position in Y axis
    zUser(i) = 0;                   % User position in Z axis = 0
end

%% Figure Plot in Initial State

% User plotting
figure,
userPlot = plot3(xUser,yUser,zUser,'k*','linewidth',3); hold on;
for o=1:noUsers
    textUsers(o) = text(xUser(o)-10,yUser(o)-10,zUser(o)+10,['U',num2str(o)],'FontSize', 8);
    textUsAngle(o) = text(xUser(o)+10,yUser(o)+10,['U_{\theta_{',num2str(o),'}}'],'FontSize', 6);
end

% UAVs Plotting
plot3(xUAV_S, yUAV_S, zUAV_S, 'kh','linewidth', 3);
for o=1:noUAV
    text(xUAV_S(o) + 20, yUAV_S(o) + 10, zUAV_S(o) + 10,['UAV',num2str(o),' Start Position']);
end

plot3(xUAV_E, yUAV_E, zUAV_E, 'kh','linewidth', 3);
for o=1:noUAV
    text(xUAV_E(o) + 20, yUAV_E(o) + 10, zUAV_E(o) + 10,['UAV',num2str(o),' End Position']);
end

% BSs Plotting
plot3(xBS, yBS, zBS, 'k^','linewidth', 3);
for o=1:noBS
    plot3([xBS(o) xBS(o)], [yBS(o) yBS(o)],[zBS(o) 0], 'k-','linewidth', 1); drawnow
    text(xBS(o) + 20, yBS(o) + 10, zBS(o) + 10,['BS', num2str(o)],'FontSize', 8);
    textBSAngle(o) = text(xBS(o)+10,yBS(o)+10,zBS(o)-5,['BS_{\theta_{',num2str(o),'}}'],'FontSize', 6);
end

% Plotting LoS Channel between BSs and UAVs
for i = 1:noUAV
    for directBS = 1:noBS
        plot3([xUAV(i) xBS(directBS)], [yUAV(i) yBS(directBS)],[zUAV(i) zBS(directBS)], 'k:','linewidth', 0.5); drawnow
    end
end



% Plotting LoS Channel between users and UAVs
for i = 1:noUAV
    for directUsers = 1:noUsers
        plot3([xUAV(i) xUser(directUsers)], [yUAV(i) yUser(directUsers)],[zUAV(i) zUser(directUsers)], 'k-.','linewidth', 0.5); drawnow
    end
end

% Plotting UAV default path
for i = 1:noUAV
    plot3([xUAV_S(i) xUAV_E(i)], [yUAV_S(i) yUAV_E(i)],[zUAV_S(i) zUAV_E(i)], 'y--','linewidth', 0.5); drawnow
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
        plot3(xUAV(i),  yUAV(i),  zUAV(i), 'r+','linewidth', 0.5);hold on;

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
        
        minRate = min(achievableRate_ch(index,:))
        if index>1 && index<21
            updatedPosition = positionUpdate_PSO(xUAV(i),yUAV(i),zUAV(i),minRate,noUsers,xUser,yUser,g);
            up_xUAV(index,i) = xUAV(i) + updatedPosition.position(1);
            up_zUAV(index,i) = zUAV(i) + updatedPosition.position(2);
            up_yUAV(index,i) = yUAV(i);
        else
            up_xUAV(index,i) = xUAV(i);
            up_yUAV(index,i) = yUAV(i);
            up_zUAV(index,i) = zUAV(i);
        end
        plot3(up_xUAV(index,i),  up_yUAV(index,i),  up_zUAV(index,i), 'gh','linewidth', 0.5);hold on;
        
        if index>1 
            plot3([up_xUAV(index-1,i) up_xUAV(index,i)], [up_yUAV(index-1,i) up_yUAV(index,i)],[up_zUAV(index-1,i) up_zUAV(index,i)], 'g--','linewidth', 0.5); drawnow
        end
       % updatedPosition.minRate
        % LoS distance between UAVs and BSs
        for bm=1:noBS
            groundDisUAV_BS_Up(i,bm) = sqrt((up_xUAV(index,i)-xBS(bm))^2 + (up_yUAV(index,i)-yBS(bm))^2);
            DisUAV_BS_Up(i,bm) = sqrt(groundDisUAV_BS_Up(i,bm)^2 + (zBS(bm)-up_zUAV(index,i))^2);
            
            % Elavation Angle in radiant between UAVs and Users
            angleUAV_BS_Up(i,bm) = atan(abs(zBS(bm)-up_zUAV(index,i))/groundDisUAV_BS_Up(i,bm))*(180/pi);
            
            PLoS_BS_Up(index,i,bm) = 1/(1+(10*exp(-0.6*(angleUAV_BS_Up(i,bm)-10))));
            
            pow_LoS_BS_Up = b_0*(DisUAV_BS_Up(i,bm)^(-eta));
            pow_NLoS_BS_Up = k*b_0*(DisUAV_BS_Up(i,bm)^(-eta));
            Ch_pow_LoS_BS_Up(index,i,bm) = pow2db(pow_LoS_BS_Up); 
            Ch_pow_NLoS_BS_Up(index,i,bm) = pow2db(pow_NLoS_BS_Up);
            
            % Expected Path Loss Channel gain
            E_bd_BS_Up = PLoS_BS_Up(index,i,bm)*pow_LoS_BS_Up + (1 - PLoS_BS_Up(index,i,bm))*pow_NLoS_BS_Up;
            E_bd_dB_BS_Up(index,i,bm) = pow2db(E_bd_BS_Up); % in dB
            
            % Angle depend rician factor
            K_UAV_BS_Up(i,bm) = A1*exp(A2*angleUAV_BS_Up(i,bm)*(pi/180));
            
            g_UAV_BS_Up(i,bm) = sqrt(K_UAV_BS_Up(i,bm)/(1+K_UAV_BS_Up(i,bm)))*g + sqrt(1/(1+K_UAV_BS_Up(i,bm)))*g;
            
            h_UAV_BS_Up(index,i,bm) = sqrt(pow_LoS_BS_Up)*g_UAV_BS_Up(i,bm);
            
            h_UAV_BS_dB_Up(index,i,bm) = pow2db(abs(h_UAV_BS_Up(index,i,bm))^2);
            
        end
        
        % LoS distance between UAVs and Users
        for m=1:noUsers
            groundDisUAV_User_Up(i,m) = sqrt((up_xUAV(index,i)-xUser(m))^2 + (up_yUAV(index,i)-yUser(m))^2);
            DisUAV_User_Up(i,m) = sqrt(groundDisUAV_User_Up(i,m)^2 + up_zUAV(index,i)^2);
            
            % Elavation Angle in radiant between UAVs and Users
            angleUAV_User_Up(i,m) = atan(up_zUAV(index,i)/groundDisUAV_User_Up(i,m))*(180/pi);
            
            
            PLoS_Up(index,i,m) = 1/(1+(10*exp(-0.6*(angleUAV_User_Up(i,m)-10))));
            
            pow_LoS_Up = b_0*(DisUAV_User_Up(i,m)^(-eta));
            pow_NLoS_Up = k*b_0*(DisUAV_User_Up(i,m)^(-eta));
            Ch_pow_LoS_Up(index,i,m) = pow2db(pow_LoS_Up); 
            Ch_pow_NLoS_Up(index,i,m) = pow2db(pow_NLoS_Up);
            
            % Expected Path Loss Channel gain
            E_bd_Up = PLoS_Up(index,i,m)*pow_LoS_Up + (1 - PLoS_Up(index,i,m))*pow_NLoS_Up;
            E_bd_dB_Up(index,i,m) = pow2db(E_bd_Up); % in dB
            
            
            % Angle depend rician factor
            K_UAV_User_Up(i,m) = A1*exp(A2*angleUAV_User_Up(i,m)*(pi/180));
            
            g_UAV_User_Up(i,m) = sqrt(K_UAV_User_Up(i,m)/(1+K_UAV_User_Up(i,m)))*g + sqrt(1/(1+K_UAV_User_Up(i,m)))*g;
            
            h_UAV_Users_Up(index,i,m) = sqrt(pow_LoS_Up)*g_UAV_User_Up(i,m);
            
            h_UAV_Users_dB_Up(index,i,m) = pow2db(abs(h_UAV_Users_Up(index,i,m))^2);
           
        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        minRate_SWIPT = min(achievableRate_ch_SWIPT(index,:))
        if index>1 && index<21
            updatedPosition_SWIPT = positionUpdate_PSO_SWIPT(xUAV(i),yUAV(i),zUAV(i),xBS,yBS,zBS,minRate,noUsers,xUser,yUser,g,noBS);
            up_xUAV_SWIPT(index,i) = xUAV(i) + updatedPosition_SWIPT.position(1);
            up_zUAV_SWIPT(index,i) = zUAV(i) + updatedPosition_SWIPT.position(2);
            up_yUAV_SWIPT(index,i) = yUAV(i);
        else
            up_xUAV_SWIPT(index,i) = xUAV(i);
            up_yUAV_SWIPT(index,i) = yUAV(i);
            up_zUAV_SWIPT(index,i) = zUAV(i);
        end
        plot3(up_xUAV_SWIPT(index,i),  up_yUAV_SWIPT(index,i),  up_zUAV_SWIPT(index,i), 'gp','linewidth', 0.5);hold on;
        
        if index>1 
            plot3([up_xUAV_SWIPT(index-1,i) up_xUAV_SWIPT(index,i)], [up_yUAV_SWIPT(index-1,i) up_yUAV_SWIPT(index,i)],[up_zUAV_SWIPT(index-1,i) up_zUAV_SWIPT(index,i)], 'm--','linewidth', 0.5); drawnow
        end
       % updatedPosition.minRate
        % LoS distance between UAVs and BSs
        for bm=1:noBS
            groundDisUAV_BS_Up_SWIPT(i,bm) = sqrt((up_xUAV_SWIPT(index,i)-xBS(bm))^2 + (up_yUAV_SWIPT(index,i)-yBS(bm))^2);
            DisUAV_BS_Up_SWIPT(i,bm) = sqrt(groundDisUAV_BS_Up_SWIPT(i,bm)^2 + (zBS(bm)-up_zUAV_SWIPT(index,i))^2);
            
            % Elavation Angle in radiant between UAVs and Users
            angleUAV_BS_Up_SWIPT(i,bm) = atan(abs(zBS(bm)-up_zUAV_SWIPT(index,i))/groundDisUAV_BS_Up_SWIPT(i,bm))*(180/pi);
            
            PLoS_BS_Up_SWIPT(index,i,bm) = 1/(1+(10*exp(-0.6*(angleUAV_BS_Up_SWIPT(i,bm)-10))));
            
            pow_LoS_BS_Up_SWIPT = b_0*(DisUAV_BS_Up_SWIPT(i,bm)^(-eta));
            pow_NLoS_BS_Up_SWIPT = k*b_0*(DisUAV_BS_Up_SWIPT(i,bm)^(-eta));
            Ch_pow_LoS_BS_Up_SWIPT(index,i,bm) = pow2db(pow_LoS_BS_Up_SWIPT); 
            Ch_pow_NLoS_BS_Up_SWIPT(index,i,bm) = pow2db(pow_NLoS_BS_Up_SWIPT);
            
            % Expected Path Loss Channel gain
            E_bd_BS_Up_SWIPT = PLoS_BS_Up_SWIPT(index,i,bm)*pow_LoS_BS_Up_SWIPT + (1 - PLoS_BS_Up_SWIPT(index,i,bm))*pow_NLoS_BS_Up_SWIPT;
            E_bd_dB_BS_Up_SWIPT(index,i,bm) = pow2db(E_bd_BS_Up_SWIPT); % in dB
            
            % Angle depend rician factor
            K_UAV_BS_Up_SWIPT(i,bm) = A1*exp(A2*angleUAV_BS_Up_SWIPT(i,bm)*(pi/180));
            
            g_UAV_BS_Up_SWIPT(i,bm) = sqrt(K_UAV_BS_Up_SWIPT(i,bm)/(1+K_UAV_BS_Up_SWIPT(i,bm)))*g + sqrt(1/(1+K_UAV_BS_Up_SWIPT(i,bm)))*g;
            
            h_UAV_BS_Up_SWIPT(index,i,bm) = sqrt(pow_LoS_BS_Up_SWIPT)*g_UAV_BS_Up_SWIPT(i,bm);
            
            h_UAV_BS_dB_Up_SWIPT(index,i,bm) = pow2db(abs(h_UAV_BS_Up_SWIPT(index,i,bm))^2);
            
        end
        
        % LoS distance between UAVs and Users
        for m=1:noUsers
            groundDisUAV_User_Up_SWIPT(i,m) = sqrt((up_xUAV_SWIPT(index,i)-xUser(m))^2 + (up_yUAV_SWIPT(index,i)-yUser(m))^2);
            DisUAV_User_Up_SWIPT(i,m) = sqrt(groundDisUAV_User_Up_SWIPT(i,m)^2 + up_zUAV_SWIPT(index,i)^2);
            
            % Elavation Angle in radiant between UAVs and Users
            angleUAV_User_Up_SWIPT(i,m) = atan(up_zUAV_SWIPT(index,i)/groundDisUAV_User_Up_SWIPT(i,m))*(180/pi);
            
            
            PLoS_Up_SWIPT(index,i,m) = 1/(1+(10*exp(-0.6*(angleUAV_User_Up_SWIPT(i,m)-10))));
            
            pow_LoS_Up_SWIPT = b_0*(DisUAV_User_Up_SWIPT(i,m)^(-eta));
            pow_NLoS_Up_SWIPT = k*b_0*(DisUAV_User_Up_SWIPT(i,m)^(-eta));
            Ch_pow_LoS_Up_SWIPT(index,i,m) = pow2db(pow_LoS_Up_SWIPT); 
            Ch_pow_NLoS_Up_SWIPT(index,i,m) = pow2db(pow_NLoS_Up_SWIPT);
            
            % Expected Path Loss Channel gain
            E_bd_Up_SWIPT = PLoS_Up_SWIPT(index,i,m)*pow_LoS_Up_SWIPT + (1 - PLoS_Up_SWIPT(index,i,m))*pow_NLoS_Up_SWIPT;
            E_bd_dB_Up_SWIPT(index,i,m) = pow2db(E_bd_Up_SWIPT); % in dB
            
            
            % Angle depend rician factor
            K_UAV_User_Up_SWIPT(i,m) = A1*exp(A2*angleUAV_User_Up_SWIPT(i,m)*(pi/180));
            
            g_UAV_User_Up_SWIPT(i,m) = sqrt(K_UAV_User_Up_SWIPT(i,m)/(1+K_UAV_User_Up_SWIPT(i,m)))*g + sqrt(1/(1+K_UAV_User_Up_SWIPT(i,m)))*g;
            
            h_UAV_Users_Up_SWIPT(index,i,m) = sqrt(pow_LoS_Up_SWIPT)*g_UAV_User_Up_SWIPT(i,m);
            
            h_UAV_Users_dB_Up_SWIPT(index,i,m) = pow2db(abs(h_UAV_Users_Up_SWIPT(index,i,m))^2);
           
        end

        
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Power coefficeints calculation for users
        [pow_coef_array_ch_Up(index,:), pow_coef_array_fr_Up(index,:)] = findPowCoeff(abs(h_UAV_Users_Up(index,i,:)),noUsers);
        
        % Achievable Rate Calculations for Users
        [achievableRate_ch_Up(index,:), achievableRate_fr_Up(index,:)] = findAchievableRate(h_UAV_Users_Up(index,i,:),pow_coef_array_ch_Up(index,:),pow_coef_array_fr_Up(index,:),noUsers);
        
        % Achievable Rate Calculations for BSs
        achievableRate_BS_Up(index,:) = findAchievableRate_BS(h_UAV_BS_Up(index,i,:),noBS);
        
        % Achievable Rate Calculations for Users in SWIPT model
        %[achievableRate_ch_SWIPT_Up(index,:), achievableRate_fr_SWIPT_Up(index,:)] = findAchievableRate_SWIPT(h_UAV_Users_Up(index,i,:),h_UAV_BS_Up(index,i,:),pow_coef_array_ch_Up(index,:),pow_coef_array_fr_Up(index,:),noUsers);
        
        minAchRate(index) = min(achievableRate_ch(index,:));
        minAchRate_Up(index) = min(achievableRate_ch_Up(index,:));
    
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        % Power coefficeints calculation for users
        [pow_coef_array_ch_Up_SWIPT(index,:), pow_coef_array_fr_Up_SWIPT(index,:)] = findPowCoeff(abs(h_UAV_Users_Up_SWIPT(index,i,:)),noUsers);
        
        % Achievable Rate Calculations for Users
        %[achievableRate_ch_Up_SWIPT(index,:), achievableRate_fr_Up_SWIPT(index,:)] = findAchievableRate(h_UAV_Users_Up_SWIPT(index,i,:),pow_coef_array_ch_Up_SWIPT(index,:),pow_coef_array_fr_Up_SWIPT(index,:),noUsers);
        
        % Achievable Rate Calculations for BSs
        %achievableRate_BS_Up_SWIPT(index,:) = findAchievableRate_BS(h_UAV_BS_Up_SWIPT(index,i,:),noBS);
        
        % Achievable Rate Calculations for Users in SWIPT model
        [achievableRate_ch_SWIPT_Up(index,:), achievableRate_fr_SWIPT_Up(index,:)] = findAchievableRate_SWIPT(h_UAV_Users_Up_SWIPT(index,i,:),h_UAV_BS_Up_SWIPT(index,i,:),pow_coef_array_ch_Up_SWIPT(index,:),pow_coef_array_fr_Up_SWIPT(index,:),noUsers);
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        minAchRate_SWIPT(index) = min(achievableRate_ch_SWIPT(index,:));
        minAchRate_Up_SWIPT(index) = min(achievableRate_ch_SWIPT_Up(index,:));
        
        
    end
    index = index +1;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLoS for Users
steps = 1:21;
figure;
plot(steps,PLoS(:,1,1),'k--^','linewidth', 1);hold on;
plot(steps,PLoS(:,1,2),'k--square','linewidth', 1);
plot(steps,PLoS(:,1,3),'k--diamond','linewidth', 1);

title('Line of Sight Probablility between UAV and GUs in each Step')
xlabel('Steps')
ylabel('Probability')
xlim([1 21])
ylim([0 1])
legend('User 1', 'User 2', 'User 3');
grid on;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLoS for BSs
figure;
plot(steps,PLoS_BS(:,1,1),'k--o','linewidth', 1);hold on;
plot(steps,PLoS_BS(:,1,2),'k--*','linewidth', 1);
plot(steps,PLoS_BS(:,1,3),'k--x','linewidth', 1);

xlim([1 21])
ylim([0 0.2])
title('Line of Sight Probablility between UAV and BSs in each Step')
xlabel('Steps')
ylabel('Probability')
legend('BS 1', 'BS 2', 'BS 3');
grid on;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Channel power for users
% figure;
% plot(steps,Ch_pow_LoS(:,1,1),'k--^','linewidth', 1);hold on;
% plot(steps,Ch_pow_LoS(:,1,2),'k--square','linewidth', 1);
% plot(steps,Ch_pow_LoS(:,1,3),'k--diamond','linewidth', 1);
% 
% plot(steps,Ch_pow_NLoS(:,1,1),'k-^','linewidth', 1);hold on;
% plot(steps,Ch_pow_NLoS(:,1,2),'k-square','linewidth', 1);
% plot(steps,Ch_pow_NLoS(:,1,3),'k-diamond','linewidth', 1);
% 
% title('Line of Sight and Non-Line of Sight Channel Power between UAV and GUs in each Step')
% xlabel('Steps')
% ylabel('Channel power in dB')
% xlim([1 21])
% %ylim([0 1])
% legend('User 1', 'User 2', 'User 3', 'User 1 (Optimized)', 'User 2 (Optimized)', 'User 3 (Optimized)');
% grid on;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLoS Channel power for BSs
% figure;
% plot(steps,Ch_pow_LoS_BS(:,1,1),'m--^','linewidth', 1);hold on;
% plot(steps,Ch_pow_LoS_BS(:,1,2),'g--^','linewidth', 1);
% plot(steps,Ch_pow_LoS_BS(:,1,3),'c--^','linewidth', 1);
% 
% plot(steps,Ch_pow_NLoS_BS(:,1,1),'b-^','linewidth', 1);hold on;
% plot(steps,Ch_pow_NLoS_BS(:,1,2),'r-^','linewidth', 1);
% plot(steps,Ch_pow_NLoS_BS(:,1,3),'k-^','linewidth', 1);
% 
% title('Line of Sight and Non-Line of Sight Channel Power between UAV and BSs in each Step')
% xlabel('Steps')
% ylabel('Channel power in dB')
% xlim([1 21])
% %ylim([0 1])
% legend('BS 1 (LoS)', 'BS 2 (LoS)', 'BS 3 (LoS)', 'BS 1 (LoS)', 'BS 2 (LoS)', 'BS 3 (LoS)');
% grid on;
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Channel power Users
figure;
% plot(steps,E_bd_dB(:,1,1),'m-^','linewidth', 1);hold on;
% plot(steps,E_bd_dB(:,1,2),'g-^','linewidth', 1);
% plot(steps,E_bd_dB(:,1,3),'c-^','linewidth', 1);

plot(steps,h_UAV_Users_dB(:,1,1),'k--^','linewidth', 1);hold on;
plot(steps,h_UAV_Users_dB(:,1,2),'k--square','linewidth', 1);
plot(steps,h_UAV_Users_dB(:,1,3),'k--diamond','linewidth', 1);

title('Rician Channel Power between UAV and GUs in each Step')
xlabel('Steps')
ylabel('Channel power in dB')
xlim([1 21])
%ylim([0 1])
legend('User 1', 'User 2', 'User 3');
grid on;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Channel power BSs
% figure;
% plot(steps,E_bd_dB_BS(:,1,1),'m--^','linewidth', 1);hold on;
% plot(steps,E_bd_dB_BS(:,1,2),'g--^','linewidth', 1);
% plot(steps,E_bd_dB_BS(:,1,3),'c--^','linewidth', 1);
% 
% plot(steps,h_UAV_BS_dB(:,1,1),'b-^','linewidth', 1);hold on;
% plot(steps,h_UAV_BS_dB(:,1,2),'r-^','linewidth', 1);
% plot(steps,h_UAV_BS_dB(:,1,3),'k-^','linewidth', 1);
% 
% title('Path Loss Channel Model Power and Rician Channel Model Power between UAV and BSs in each Step')
% xlabel('Steps')
% ylabel('Channel power in dB')
% xlim([1 21])
% %ylim([0 1])
% legend('BS 1 (Path Loss Model)', 'BS 2 (Path Loss Model)', 'BS 3 (Path Loss Model)', 'BS 1 (Rician Channel Model)', 'BS 2 (Rician Channel Model)', 'BS 3 (Rician Channel Model)');
% grid on;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Achievable rate for Users
% figure;
% plot(steps,achievableRate_ch(:,1),'m^--','linewidth', 1);hold on;
% plot(steps,achievableRate_ch(:,2),'g^--','linewidth', 1);
% plot(steps,achievableRate_ch(:,3),'c^--','linewidth', 1);
% 
% plot(steps,achievableRate_ch_Up(:,1),'b-^','linewidth', 1);hold on;
% plot(steps,achievableRate_ch_Up(:,2),'r-^','linewidth', 1);
% plot(steps,achievableRate_ch_Up(:,3),'k-^','linewidth', 1);
% 
% title('Achievable Rate for GUs in each Step (With Channel based Power Coefficeints)')
% xlim([1 21])
% %ylim([0 1])
% xlabel('Steps')
% ylabel('Achievable Rate in bps')
% legend('User 1', 'User 2', 'User 3','User 1 (Optimized)', 'User 2 (Optimized)', 'User 3 (Optimized)');
% grid on;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure
% plot(steps,achievableRate_fr(:,1),'m^--','linewidth', 1);hold on;
% plot(steps,achievableRate_fr(:,2),'g^--','linewidth', 1);
% plot(steps,achievableRate_fr(:,3),'c^--','linewidth', 1);
% 
% plot(steps,achievableRate_fr_Up(:,1),'b^-','linewidth', 1);hold on;
% plot(steps,achievableRate_fr_Up(:,2),'r^-','linewidth', 1);
% plot(steps,achievableRate_fr_Up(:,3),'k^-','linewidth', 1);
% 
% title('Achievable Rate for GUs in each Step (With Fraction value based Power Coefficeints)')
% xlim([1 21])
% %ylim([0 1])
% xlabel('Steps')
% ylabel('Achievable Rate in bps')
% legend('User 1', 'User 2', 'User 3','User 1 (Optimized)', 'User 2 (Optimized)', 'User 3 (Optimized)');
% grid on;
% 
% 
% % Achievable rate for BSs
% % figure;
% % plot(steps,achievableRate_BS(:,1),'*--','linewidth', 1);hold on;
% % plot(steps,achievableRate_BS(:,2),'*--','linewidth', 1);
% % plot(steps,achievableRate_BS(:,3),'*--','linewidth', 1);
% % 
% % plot(steps,achievableRate_BS_Up(:,1),'*-','linewidth', 1);hold on;
% % plot(steps,achievableRate_BS_Up(:,2),'*-','linewidth', 1);
% % plot(steps,achievableRate_BS_Up(:,3),'*-','linewidth', 1);
% % 
% % xlim([1 21])
% % %ylim([0 1])
% % legend('BS 1', 'BS 2', 'BS 3','BS 1 Up', 'BS 2 Up', 'BS 3 Up');
% % grid on;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 
% % Achievable rate for Users SWIPT Model
% figure;
% plot(steps,achievableRate_ch_SWIPT(:,1),'m--^','linewidth', 1);hold on;
% plot(steps,achievableRate_ch_SWIPT(:,2),'g--^','linewidth', 1);
% plot(steps,achievableRate_ch_SWIPT(:,3),'c--^','linewidth', 1);
% 
% plot(steps,achievableRate_ch_SWIPT_Up(:,1),'b*-','linewidth', 1);hold on;
% plot(steps,achievableRate_ch_SWIPT_Up(:,2),'r*-','linewidth', 1);
% plot(steps,achievableRate_ch_SWIPT_Up(:,3),'k*-','linewidth', 1);
% 
% title('Achievable Rate for GUs in each Step in SWIPT Model (With Channel based Power Coefficeints)')
% xlim([1 21])
% %ylim([0 1])
% xlabel('Steps')
% ylabel('Achievable Rate in bps')
% legend('User 1', 'User 2', 'User 3','User 1 (Optimized)', 'User 2 (Optimized)', 'User 3 (Optimized)');
% grid on;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure;
% plot(steps,achievableRate_fr_SWIPT(:,1),'m^--','linewidth', 1);hold on;
% plot(steps,achievableRate_fr_SWIPT(:,2),'g^--','linewidth', 1);
% plot(steps,achievableRate_fr_SWIPT(:,3),'c^--','linewidth', 1);
% 
% plot(steps,achievableRate_fr_SWIPT_Up(:,1),'b^-','linewidth', 1);hold on;
% plot(steps,achievableRate_fr_SWIPT_Up(:,2),'r^-','linewidth', 1);
% plot(steps,achievableRate_fr_SWIPT_Up(:,3),'k^-','linewidth', 1);
% 
% title('Achievable Rate for GUs in each Step in SWIPT Model (With Fraction value based Power Coefficeints)')
% xlim([1 21])
% %ylim([0 1])
% xlabel('Steps')
% ylabel('Achievable Rate in bps')
% legend('User 1', 'User 2', 'User 3','User 1 (Optimized)', 'User 2 (Optimized)', 'User 3 (Optimized)');
% grid on;
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% figure;
% plot(steps,minAchRate(:),'m^--','linewidth', 1);hold on;
% 
% plot(steps,minAchRate_Up(:),'c^-','linewidth', 1);
% 
% title('Minimum Achievable Rate of Network in each Step (With Channel based based Power Coefficeints)')
% xlim([1 21])
% %ylim([0 1])
% xlabel('Steps')
% ylabel('Achievable Rate in bps')
% legend('Minimum Rate (before Optimized)', 'Minimum Rate (After Optimized)');
% grid on;