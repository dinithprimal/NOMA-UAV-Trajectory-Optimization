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
%noBS = 3;                   % Number of Base Sations
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

        % LoS distance between UAVs and Users
        for m=1:noUsers
            groundDisUAV_User(i,m) = sqrt((xUAV(i)-xUser(m))^2 + (yUAV(i)-yUser(m))^2);
            DisUAV_User(i,m) = sqrt(groundDisUAV_User(i,m)^2 + zUAV^2);
            
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
            
            h_UAV_Users(i,m) = sqrt(pow_LoS)*g_UAV_User(i,m);
            
            h_UAV_Users_dB(index,i,m) = pow2db(abs(h_UAV_Users(i,m))^2);
            
        end
    end
    index = index +1;
end

steps = 1:21;
figure;
plot(steps,PLoS(:,1,1),'linewidth', 1);hold on;
plot(steps,PLoS(:,1,2),'linewidth', 1);
plot(steps,PLoS(:,1,3),'linewidth', 1);
xlim([1 21])
ylim([0 1])
legend('U 1', 'U 2', 'U 3');
grid on;

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