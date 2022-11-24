clc
clear variables
close all

%% Desription
% This simulation environment has 2000m width, 2000m lenght and 500m height
%  
% In this simulation has 3 users, 3 base stations and 1 UAV
% As the first step, users, base stations and UAV positioned randomly by manually
% Assumed UAV in randomly assigned static position in XYZ plane (In first step)
% Assumed users position randomly assigned and height is 0
% Base station position randomly assigned edges of the envoronment
% Base station height randomly assigned between 15m to 50m

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

% UAV Position
for i=1:noUAV
    xUAV(i) = randi(enWidth);       % UAV position in X axis
    yUAV(i) = randi(enLength);      % UAV position in Y axis
    zUAV(i) = randi([minHeighUAV,maxHeighUAV]);   % UAV position in Z axis
end

% Users' posision
for i=1:noUsers
    xUser(i) = randi(enWidth);      % User position in X axis
    yUser(i) = randi(enLength);     % User position in Y axis
    zUser(i) = 0;                   % User position in Z axis = 0
end

% BSs' posision
x = [0, 500, 1000];
y = [500, 0, 500];
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
plot3(xUAV, yUAV, zUAV, 'bh','linewidth', 3);
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
        plot3([xUAV(i) xUser(directUsers)], [yUAV(i) yUser(directUsers)],[zUAV(i) zUser(directUsers)], 'r-','linewidth', 1); drawnow
    end
end

% Plotting LoS Channel between BSs and UAVs
for i = 1:noUAV
    for directBS = 1:noBS
        plot3([xUAV(i) xBS(directBS)], [yUAV(i) yBS(directBS)],[zUAV(i) zBS(directBS)], 'g-','linewidth', 1); drawnow
    end
end

% 3D Plot labels
xlim([0 1100])
ylim([0 1100])
zlim([0 320])
xlabel('Length (m)');
ylabel('Width (m)');
zlabel('Height (m)');
grid on; drawnow

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

eta = 4;    % Path Loss Component
b0 = db2pow(-60);  % Average channel power gain at a reference deistance d0 = 1m

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

%% Analysis Result for Uplink and Downlink

ptdb = 10:5:110;                   %Transmission power of BS in dBm
pt = (10^-3)*db2pow(ptdb);        %Transmission power of BS in linear scale

BW = 10^6;                      %System bandwidht
No = -174 + 10*log10(BW);       %Noise power dBm
no = (10^-3)*db2pow(No);       %Noise power linear scale

snr = pt./no;               %SNR in linear scale
snrdb = 10*log(snr);       %SNR in dbm
%snrdb = snrdbm - 30;        %SNR in db

%pR1 = zeros(1,length(snr));      %counter for Relay 1
%pR2 = zeros(1,length(snr));      %counter for Relay 2
pRUP = zeros(length(ptdb),noUAV,noBS);
pROMAUP = zeros(length(ptdb),noUAV,noBS);

pRDwn = zeros(length(ptdb),noUAV,noUsers);
pROMADwn = zeros(length(ptdb),noUAV,noUsers);

rate = 1;                      %Achievable Rete for signals

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
    for m=1:noUsers
        sort_mean_abs_h_UAV_Users(i,:) = sort(mean_abs_h_UAV_Users(i,:));
    end
    % Mean value of Rician fading for UAVs and BSs
    for m=1:noBS
        sort_mean_abs_h_UAV_BS(i,:) = sort(mean_abs_h_UAV_BS(i,:));
    end
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

%-------------------------------------------------------------------------------------------------------------------
%-------------------------------------------------------------------------------------------------------------------
% Simulation for Uplink

for u = 1:length(ptdb)
    for i = 1:noUAV
        for m=1:noBS
            if m==1
                for p = 1:N
                    arr(p) = abs_h_UAV_BS(i,indexBS(i,m),p);
                end
                achRateUP(u,i,indexBS(i,m),:) = log2(1 + snr(u).*arr);
            else
                abs_h = zeros(1,N);
                for j = 1:(m-1)
                    for p = 1:N
                        arr(p) = abs_h_UAV_BS(i,indexBS(i,j),p);
                    end
                    abs_h = abs_h + arr;
                end
                for p = 1:N
                    arr(p) = abs_h_UAV_BS(i,indexBS(i,m),p);
                end
                achRateUP(u,i,indexBS(i,m),:) = log2(1 + ((snr(u).*arr)./(1 + (snr(u).*abs_h))));
            end
            for p = 1:N
                arr(p) = abs_h_UAV_BS(i,m,p);
            end
            achRateOMAUP(u,i,m,:) = (1/noBS)*log2(1 + (snr(u).*arr)); 
        end
    end
end


for u = 1:length(ptdb)
    for i = 1:noUAV
        for m=1:noBS
            for k = 1:N
                if (achRateUP(u,i,m,k) < rate)
                    pRUP(u,i,m) = pRUP(u,i,m) + 1;
                end
                if (achRateOMAUP(u,i,m,k) < rate)
                    pROMAUP(u,i,m) = pROMAUP(u,i,m) + 1;
                end
            end
        end
    end
end

for u = 1:length(ptdb)
    for i = 1:noUAV
        for m=1:noBS
            for k = 1:N
                pRoutUP(u,i,m) = pRUP(u,i,m)/N;
                pRoutOMAUP(u,i,m) = pROMAUP(u,i,m)/N;
            end
        end
    end
end

% for i = 1:N
%     arr(i) = abs_h_UAV_BS(1,3,i);
% end
figure
semilogy(ptdb, pRoutUP(:,1,1),':', 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutUP(:,1,2),':', 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutUP(:,1,3),':', 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutOMAUP(:,1,1), 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutOMAUP(:,1,2), 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutOMAUP(:,1,3), 'linewidth', 1.5); hold on;

legend('BS 1 -> UAV (NOMA)','BS 2 -> UAV (NOMA)','BS 3 -> UAV (NOMA)','BS 1 -> UAV (OMA)','BS 2 -> UAV (OMA)','BS 3 -> UAV (OMA)');
xlabel('Transmit Power (dBm)');
ylabel('Outage Probability');
grid on; drawnow
title('Outage Probability in AWGN and Rician Fading channel for Uplink (BS -> UAV)');

%-------------------------------------------------------------------------------------------------------------------
%-------------------------------------------------------------------------------------------------------------------
% Simulation for Downlink

for i = 1:noUsers
    if i == 1
        powCoef(i) = 0.7;
    elseif i > 1
        if i == noUsers
            powCoef(i) = 1 - sum(powCoef);
        else
            powCoef(i) = 0.8*(1 - sum(powCoef));
        end
    end
end

powCoef = sort(powCoef);

for u = 1:length(ptdb)
    for i = 1:noUAV
        for m=1:noUsers
            if m==noUsers
                for p = 1:N
                    arr(p) = abs_h_UAV_Users(i,indexUser(i,m),p);
                end
                achRateDwn(u,i,indexUser(i,m),:) = log2(1 + powCoef(m)*snr(u).*arr);
            else
                ap = 0;
                for j = 1:(i-1)
                    ap = ap + powCoef(j);
                end
                for p = 1:N
                    arr(p) = abs_h_UAV_Users(i,indexUser(i,m),p);
                end
                achRateDwn(u,i,indexUser(i,m),:) = log2(1 + ((powCoef(m)*snr(u).*arr)./(1 + (ap*snr(u).*arr))));
            end
            for p = 1:N
                arr(p) = abs_h_UAV_Users(i,m,p);
            end
            achRateOMADwn(u,i,m,:) = (1/noUsers)*log2(1 + (snr(u).*arr)); 
        end
    end
end

for u = 1:length(ptdb)
    for i = 1:noUAV
        for m=1:noUsers
            for k = 1:N
                if (achRateDwn(u,i,m,k) < rate)
                    pRDwn(u,i,m) = pRDwn(u,i,m) + 1;
                end
                if (achRateOMADwn(u,i,m,k) < rate)
                    pROMADwn(u,i,m) = pROMADwn(u,i,m) + 1;
                end
            end
        end
    end
end

for u = 1:length(ptdb)
    for i = 1:noUAV
        for m=1:noUsers
            for k = 1:N
                pRoutDwn(u,i,m) = pRDwn(u,i,m)/N;
                pRoutOMADwn(u,i,m) = pROMADwn(u,i,m)/N;
            end
        end
    end
end

figure
semilogy(ptdb, pRoutDwn(:,1,1),':', 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutDwn(:,1,2),':', 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutDwn(:,1,3),':', 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutOMADwn(:,1,1), 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutOMADwn(:,1,2), 'linewidth', 1.5); hold on;
semilogy(ptdb, pRoutOMADwn(:,1,3), 'linewidth', 1.5); hold on;

legend('UAV -> User 1 (NOMA)','UAV -> User 2 (NOMA)','UAV -> User 3 (NOMA)','UAV -> User 1 (OMA)','UAV -> User 2 (OMA)','UAV -> User 3 (OMA)');
xlabel('Transmit Power (dBm)');
ylabel('Outage Probability');
grid on; drawnow
title('Outage Probability in AWGN and Rician Fading channel for Downlink (UAV -> Users)');

% 
% for u = 1:length(ptdb)
%     for i = 1:noUAV
%         for m=1:noUsers
%             for k = 1:N
%                 pRoutAll(u,i,m) = pRoutUP(u,i,m)*pRoutDwn(u,i,m);
%                 pRoutOMAAll(u,i,m) = pRoutOMAUP(u,i,m)*pRoutOMADwn(u,i,m);
%             end
%         end
%     end
% end
% 
% 
% figure
% semilogy(ptdb, pRoutAll(:,1,1),':', 'linewidth', 1.5); hold on;
% semilogy(ptdb, pRoutAll(:,1,2),':', 'linewidth', 1.5); hold on;
% semilogy(ptdb, pRoutAll(:,1,3),':', 'linewidth', 1.5); hold on;
% semilogy(ptdb, pRoutOMAAll(:,1,1), 'linewidth', 1.5); hold on;
% semilogy(ptdb, pRoutOMAAll(:,1,2), 'linewidth', 1.5); hold on;
% semilogy(ptdb, pRoutOMAAll(:,1,3), 'linewidth', 1.5); hold on;
% 
% legend('BS 1 -> User 1 (NOMA)','BS 2 -> User 2 (NOMA)','BS 3 -> User 3 (NOMA)','BS 1 -> User 1 (OMA)','BS 2 -> User 2 (OMA)','BS 3 -> User 3 (OMA)');
% xlabel('Transmit Power (dBm)');
% ylabel('Outage Probability');
% grid on; drawnow
% title('Outage Probability in AWGN and Rician Fading channel (BSs -> Users)');
% 
% 
% %% Analysis Result for Downlink
% 
