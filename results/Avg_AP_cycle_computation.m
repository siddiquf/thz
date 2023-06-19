% Script to compute the performance metrics

clc
clear

%% Parameters
handshake_ways = 3; % 0, 1, 2 or 3 way handshake (0: CSMA, 1: ADAPT-1, 2: CSMA/CA, 3: ADAPT-3)
nodeNum = 50;       % Number of client nodes
Tia = 500;          % [us] Mean inter-arrival time
seedNum=1;
config=29;
protocol = "tcp";
%packetSize=59000;

final_avg=0;
for seedN = 1:seedNum
    
%% Load simulation results
filename =sprintf('AP_cycle_result_%uway_%un_%uus_Config%u_%u_%s.txt',handshake_ways,nodeNum,Tia,config,seedN,protocol);
fileID = fopen(filename, 'r');
timestamps = fscanf(fileID, '%f');
fclose(fileID);

% Compute the differences between consecutive timestamps
differences = diff(timestamps);

% Calculate the average of the differences
averageDifference = mean(differences);

% Display the average difference
disp(['Average difference between consecutive timestamps: ', num2str(averageDifference)]);
final_avg= final_avg + averageDifference;
end
fprintf(['Average AP cycle time for %u config %u nodes %s protocol ' ...
         'is %f \n'], config,nodeNum, protocol, final_avg/seedNum);
