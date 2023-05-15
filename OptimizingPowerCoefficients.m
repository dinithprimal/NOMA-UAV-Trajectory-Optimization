clc
clear variables
close all

%% Description

% This simulation environment has 2000m width, 2000m lenght and 500m height
%  
% In this simulation has 3 users, 3 base stations and 1 UAV
% In this step, users, base stations and UAV positioned randomly by manually
% Assumed UAV in randomly assigned static position in XYZ plane (In first step)
% Assumed users position randomly assigned and height is 0
% Base station position randomly assigned edges of the envoronment
% Base station height randomly assigned between 15m to 50m
% This simulation shows how users performace differ according to the
% differnet optimum power allocation methods

%% Simulation Environment

enWidth = 1000;             % Simulation Environment Width (X axis)
enLength = 1000;            % Simulation Environment Length (Y axis)
enHeight = 320;             % Simulation Environment Height (Z axis)
maxHeighUAV = 300;          % Maximum Height of UAV
minHeighUAV = 150;           % Minimum Height of UAV
noUsers = 3;                % Number of Users
noBS = 3;                   % Number of Base Sations
noUAV = 1;                  % Number of UAVs

%% Users, Base Sataions and UAV Position

% UAV Start Position
for i=1:noUAV
    xUAV_S(i) = 0;       % UAV position in X axis
    yUAV_S(i) = enLength;      % UAV position in Y axis
    zUAV_S(i) = randi([minHeighUAV,maxHeighUAV]);   % UAV position in Z axis
end

% UAV End Position
for i=1:noUAV
    xUAV_E(i) = enWidth;       % UAV position in X axis
    yUAV_E(i) = 0;      % UAV position in Y axis
    zUAV_E(i) = randi([minHeighUAV,maxHeighUAV]);   % UAV position in Z axis
end

% UAV Position
for i=1:noUAV
    xUAV(i) = 0;       % UAV position in X axis
    yUAV(i) = enLength;      % UAV position in Y axis
    zUAV(i) = zUAV_S(i);   % UAV position in Z axis
end

% Users' posision
for i=1:noUsers
    xUser(i) = randi(enWidth);      % User position in X axis
    yUser(i) = randi([enLength - xUser(i),enLength]);     % User position in Y axis
    zUser(i) = 0;                   % User position in Z axis = 0
end

% BSs' posision
x = [0, 0, 500];
y = [500, 0, 0];
for i=1:noBS
    xBS(i) = x(i);          % Base station position in X axis
    yBS(i) = y(i);          % Base station position in Y axis
    zBS(i) = randi([15, 50]);           % Base station position in Z axis
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

plot3(xUAV, yUAV, zUAV, 'yh','linewidth', 1);
for o=1:noUAV
    text(xUAV(o) + 20, yUAV(o) + 10, zUAV(o) + 10,['UAV', num2str(o)]);
end

% BSs Plotting
plot3(xBS, yBS, zBS, 'b*','linewidth', 3);
for o=1:noBS
    plot3([xBS(o) xBS(o)], [yBS(o) yBS(o)],[zBS(o) 0], 'bl-','linewidth', 1); drawnow
    text(xBS(o) + 20, yBS(o) + 10, zBS(o) + 10,['BS', num2str(o)],'FontSize', 8);
    textBSAngle(o) = text(xBS(o)+10,yBS(o)+10,zBS(o)-5,['BS_{\theta_{',num2str(o),'}}'],'FontSize', 6);
end

% Plotting LoS Channel between users and UAVs
for i = 1:noUAV
    for directUsers = 1:noUsers
        plot3([xUAV(i) xUser(directUsers)], [yUAV(i) yUser(directUsers)],[zUAV(i) zUser(directUsers)], 'r--','linewidth', 0.1); drawnow
    end
end

% Plotting LoS Channel between BSs and UAVs
for i = 1:noUAV
    for directBS = 1:noBS
        plot3([xUAV(i) xBS(directBS)], [yUAV(i) yBS(directBS)],[zUAV(i) zBS(directBS)], 'g--','linewidth', 0.1); drawnow
    end
end

% Plotting UAV default path
for i = 1:noUAV
    plot3([xUAV_S(i) xUAV_E(i)], [yUAV_S(i) yUAV_E(i)],[zUAV_S(i) zUAV_E(i)], 'y--','linewidth', 2); drawnow
end

% 3D Plot labels
xlim([0 1050])
ylim([0 1050])
zlim([0 320])
xlabel('Length (m)');
ylabel('Width (m)');
zlabel('Height (m)');
grid on; drawnow

%% Simulation Constraits

maxDist = 20;   % max distance that UAV can travel in a one unit time


%% LoS distance between UAV and Users/Base Stations

for i = 1:noUAV
    % LoS distance between UAVs and Users
    for m=1:noUsers
        groundDisUAV_User = sqrt((xUAV(i)-xUser(m))^2 + (yUAV(i)-yUser(m))^2);
        DisUAV_User(i,m) = sqrt(groundDisUAV_User^2 + zUAV^2);
    end
    
    % LoS distance between UAVs and BSs
    for m=1:noBS
        groundDisUAV_BS = sqrt((xUAV(i)-xBS(m))^2 + (yUAV(i)-yBS(m))^2);
        DisUAV_BS(i,m) = sqrt(groundDisUAV_BS^2 + (zUAV(i) - zBS(m))^2);
    end
end

%% Elavation Angle between UAV and Users/Base Stations

for i = 1:noUAV
    % Elavation Angle in radiant between UAVs and Users
    for m=1:noUsers
        angleUAV_User(i,m) = asin(zUAV(i)/DisUAV_User(i,m));%*(pi/180);
    end
    % Elavation Angle in radiant between UAVs and BSs
    for m=1:noBS
        angleUAV_BS(i,m) = asin(abs(zUAV(i)-zBS(m))/DisUAV_BS(i,m));%*(pi/180);
    end
end

%% Angle-depend rician factor for Users and BSs
A1 = 1;
A2 = (log(db2pow(60)/A1))/(pi/2);
for i = 1:noUAV
    % Angle-depend rician factor for UAVs and Users
    for m=1:noUsers
        K_UAV_User(i,m) = A1*exp(A2*angleUAV_User(i,m));
    end
    % Angle-depend rician factor for UAVs and BSs
    for m=1:noBS
        K_UAV_BS(i,m) = A1*exp(A2*angleUAV_BS(i,m));
    end
end

%% Rician Fading for Users and BSs

N = 10^5;
g = sqrt(1/2)*(randn(1,N)+1i*randn(1,N));

for i = 1:noUAV
    % Rician fading for UAVs and Users
    for m=1:noUsers
        g_UAV_User(i,m,:) = sqrt(K_UAV_User(i,m)/(1+K_UAV_User(i,m)))*g + sqrt(1/(1+K_UAV_User(i,m)))*g;
    end
    % Rician fading for UAVs and BSs
    for m=1:noBS
        g_UAV_BS(i,m,:) = sqrt(K_UAV_BS(i,m)/(1+K_UAV_BS(i,m)))*g + sqrt(1/(1+K_UAV_BS(i,m)))*g;
    end
end

%% Avarage Channel Power Gain

% Assume Path Loss Componet = 4
% Assume Average Channel Power Gain = -60dB

eta = 2;    % Path Loss Component
b0 = db2pow(0);  % Average channel power gain at a reference deistance d0 = 1m

for i = 1:noUAV
    % Rician fading for UAVs and Users
    for m=1:noUsers
        chPow_UAV_User(i,m) = b0*((DisUAV_User(i,m))^(-eta));
    end
    % Rician fading for UAVs and BSs
    for m=1:noBS
        chPow_UAV_BS(i,m) = b0*((DisUAV_BS(i,m))^(-eta));
    end
end

%% Channel Coefficeint

for i = 1:noUAV
    % Rician fading for UAVs and Users
    for m=1:noUsers
        h_UAV_Users = sqrt(chPow_UAV_User(i,m))*g_UAV_User(i,m,:);
        abs_h_UAV_Users(i,m,:) = (abs(h_UAV_Users)).^2;
    end
    % Rician fading for UAVs and BSs
    for m=1:noBS
        h_UAV_BS = sqrt(chPow_UAV_BS(i,m))*g_UAV_BS(i,m,:);
        abs_h_UAV_BS(i,m,:) = (abs(h_UAV_BS)).^2;
    end
end

%Channel coeffiecent value sorting
for i = 1:noUAV
    % Mean value of Rician fading for UAVs and Users
    for m=1:noUsers
        mean_abs_h_UAV_Users(i,m) = mean(abs_h_UAV_Users(i,m,:));
    end
    % Mean value of Rician fading for UAVs and BSs
    for m=1:noBS
        mean_abs_h_UAV_BS(i,m) = mean(abs_h_UAV_BS(i,m,:));
    end
end

for i = 1:noUAV
    % Mean value of Rician fading for UAVs and Users
    sort_mean_abs_h_UAV_Users(i,:) = sort(mean_abs_h_UAV_Users(i,:));

    % Mean value of Rician fading for UAVs and BSs
    sort_mean_abs_h_UAV_BS(i,:) = sort(mean_abs_h_UAV_BS(i,:));
end

for i = 1:noUAV
    for m=1:noUsers
        % Allocate user number for UAV
        indexUser(i,m) = find(mean_abs_h_UAV_Users(i,:) == sort_mean_abs_h_UAV_Users(i,m));
    end
    for m=1:noBS
        % Allocate BS number for UAV
        indexBS(i,m) = find(mean_abs_h_UAV_BS(i,:) == sort_mean_abs_h_UAV_BS(i,m));
    end
end

% BS Transmit power in dBm
P_BS = 46;

% BS Transmit power in linear scale
p_BS = (10^-3)*db2pow(P_BS);



