clc;
clear all
close all

%% Description

% This simulation environment has 2000m width, 2000m lenght and 500m height
%  
% This simulation has 3 users, 1 base station and they are 
% positioned randomly in XYZ plane 
% 
% Throughout this simulation analyze the perfomance of allocating power
% coefficients for gound users in Downlink NOMA via 3 different ways
%           1. Manually assigned power coeficients
%           2. Regarding the channel gains
%           3. Using PSO (Particla Swarm Optimization) tool
%
%% Simulation Environment

enWidth = 500;             % Simulation Environment Width (X axis)
enLength = 500;            % Simulation Environment Length (Y axis)
enHeight = 320;             % Simulation Environment Height (Z axis)
noUsers = 3;                % Number of Users
noBS = 1;                   % Number of Base Sations

%% Users, Base Sataions and UAV Position

% Users' posision
for i=1:noUsers
    xUser(i) = randi(enWidth);      % User position in X axis
    yUser(i) = randi([enLength - xUser(i),enLength]);     % User position in Y axis
    zUser(i) = 0;                   % User position in Z axis = 0
end

% BSs' posision
x = 10;
y = 10;
for i=1:noBS
    xBS(i) = x(i);          % Base station position in X axis
    yBS(i) = y(i);          % Base station position in Y axis
    zBS(i) = randi([15, 50]);           % Base station position in Z axis
end

%% Simulation Environment Plot

% User plotting
figure,
userPlot = plot3(xUser,yUser,zUser,'r*','linewidth',3); hold on;
for o=1:noUsers
    textUsers(o) = text(xUser(o)-10,yUser(o)-10,zUser(o)+10,['U',num2str(o)],'FontSize', 8);
    textUsAngle(o) = text(xUser(o)+10,yUser(o)+10,['U_{\theta_{',num2str(o),'}}'],'FontSize', 6);
end

% BSs Plotting
plot3(xBS, yBS, zBS, 'b*','linewidth', 3);
for o=1:noBS
    plot3([xBS(o) xBS(o)], [yBS(o) yBS(o)],[zBS(o) 0], 'bl-','linewidth', 1); drawnow
    text(xBS(o) + 20, yBS(o) + 10, zBS(o) + 10,['BS', num2str(o)],'FontSize', 8);
    textBSAngle(o) = text(xBS(o)+10,yBS(o)+10,zBS(o)-5,['BS_{\theta_{',num2str(o),'}}'],'FontSize', 6);
end

% Plotting LoS Channel between users and BSs
for i = 1:noBS
    for directUsers = 1:noUsers
        plot3([xBS(i) xUser(directUsers)], [yBS(i) yUser(directUsers)],[zBS(i) zUser(directUsers)], 'g--','linewidth', 1); drawnow
    end
end

% 3D Plot labels
xlim([0 enWidth+10])
ylim([0 enLength+10])
zlim([0 60])
xlabel('Length (m)');
ylabel('Width (m)');
zlabel('Height (m)');
grid on; drawnow

%% LoS distance between BS and Users

for i = 1:noBS
    % LoS distance between BS and Users
    for m=1:noUsers
        groundDisBS_User = sqrt((xBS(i)-xUser(m))^2 + (yBS(i)-yUser(m))^2);
        DisBS_User(i,m) = sqrt(groundDisBS_User^2 + zBS^2);
    end
    
end

%% Elavation Angle between BS and Users

for i = 1:noBS
    % Elavation Angle in radiant between BS and Users
    for m=1:noUsers
        angleBS_User(i,m) = asin(zBS(i)/DisBS_User(i,m));%*(180/pi);
    end
end

%% Angle-depend rician factor for Users and BSs
A1 = 1;
A2 = (log(db2pow(60)/A1))/(pi/2);
for i = 1:noBS
    % Angle-depend rician factor for BS and Users
    for m=1:noUsers
        K_BS_User(i,m) = A1*exp(A2*angleBS_User(i,m));
    end
end

%% Rician Fading for Users and BSs

N = 10^5;
g = sqrt(1/2)*(randn(1,N)+1i*randn(1,N));

for i = 1:noBS
    % Rician fading for BS and Users
    for m=1:noUsers
        g_BS_User(i,m,:) = sqrt(K_BS_User(i,m)/(1+K_BS_User(i,m)))*g + sqrt(1/(1+K_BS_User(i,m)))*g;
    end
end

%% Avarage Channel Power Gain

% Assume Path Loss Componet = 4
% Assume Average Channel Power Gain = -60dB

eta = 4;    % Path Loss Component
b0 = db2pow(-30);  % Average channel power gain at a reference deistance d0 = 1m

for i = 1:noBS
    % Rician fading for UAVs and Users
    for m=1:noUsers
        chPow_BS_User(i,m) = b0*((DisBS_User(i,m))^(-eta));
    end
end

%% Channel Coefficeint

for i = 1:noBS
    % Rician fading for BS and Users
    for m=1:noUsers
        h_BS_Users = sqrt(chPow_BS_User(i,m))*g_BS_User(i,m,:);
        abs_h_BS_Users(i,m,:) = (abs(h_BS_Users)).^2;
    end
end

%Channel coeffiecent value sorting
for i = 1:noBS
    % Mean value of Rician fading for UAVs and Users
    for m=1:noUsers
        mean_abs_h_BS_Users(i,m) = mean(abs_h_BS_Users(i,m,:));
    end
end

% for i = 1:noBS
%     % Mean value of Rician fading for BS and Users
%     for m=1:noUsers
%         sort_mean_abs_h_BS_Users(i,:) = sort(mean_abs_h_BS_Users(i,:));
%     end
% end
% 
% for i = 1:noBS
%     for m=1:noUsers
%         % Allocate user number for UAV
%         indexUser(i,m) = find(mean_abs_h_BS_Users(i,:) == sort_mean_abs_h_BS_Users(i,m));
%     end
% end

%% Power Coefficeint by Channel gains

for i = 1:noBS
    for m=1:noUsers
        % Allocate power coefficeints for users
        powerCoef_in(i,m) = 1 - (mean_abs_h_BS_Users(i,m)/sum(mean_abs_h_BS_Users(i,:)));
    end
    for m=1:noUsers
        % Allocate power coefficeints for users
        powerCoef(i,m) = (powerCoef_in(i,m)/sum(powerCoef_in(i,:)));
    end
end

%% Achievable Rate Calculation

Pt = -60:5:60;     %in dB
pt = db2pow(Pt);	%in linear scale

B = 10^6;
No = -174 + 10*log10(B);
no = (10^-3)*db2pow(No);

for i = 1:noBS
    sort_pow(i,:) = sort(powerCoef(i,:));
end

for i = 1:noBS
    for m=1:noUsers
        % Allocate user number for UAV
        indexUser(i,m) = find(powerCoef(i,:) == sort_pow(i,m));
    end
end

for u = 1:length(pt)
    for i = 1:noBS
        for m=1:noUsers
            if mean_abs_h_BS_Users(i,m) == max(mean_abs_h_BS_Users(i,:))
                C = B*log2(1 + pt(u)*sort_pow(i,1).*abs_h_BS_Users(i,m,:)./(no));
                C_mean(u,i,m) = mean(C);
            else
                C_mean(u,i,m) = mean(C);
            end
            
        end
    end
end

figure;
for i = 1:noBS
    for m=1:noUsers
        plot(Pt,C_mean(:,i,m),'linewidth',2); hold on; grid on;
    end
end