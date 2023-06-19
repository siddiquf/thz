% Script to compute the performance metrics

%clc
%clear

%% Parameters
handshake_ways = 3; % 0, 1, 2 or 3 way handshake (0: CSMA, 1: ADAPT-1, 2: CSMA/CA, 3: ADAPT-3)
nodeNum =50;       % Number of client nodes
Tia = 500;          % [us] Mean inter-arrival time
seedNum=1;
config=29;
protocol = "tcp";
%packetSize=59000;

throughput_avg_trials = 0;
packet_time_avg_trials = 0;

for seedN = 1:seedNum
    
%% Load simulation results
filename = sprintf('result_%uway_%un_%uus_Config%u_%u_%s.txt',handshake_ways,nodeNum,Tia,config,seedN,protocol);
fileID = fopen(filename,'r');
data = fscanf(fileID,'%u %u %u %u %u',[5 Inf]);
fclose(fileID);

%% Compute metrics
data = data.';

% Throughput calculation
throughput_node = zeros(1,nodeNum); % [Gbps] Throughput of each node
for n = 1:nodeNum
    % Select only data from node n and succesfull packets
    succ_data = data(data(:,1) == n & data(:,4) == 1,:);
    throughput_node(n) = mean(succ_data(:,2)*8./(succ_data(:,3)));
    if size(succ_data,1) == 0
        throughput_node(n) = 0;
    end
end
throughput = mean(throughput_node); % [Gbps] Average throughput

% Discard rate
discard_rate = sum(data(:,5))/size(data,1);

% [us] Average packet time
succ_data = data(data(:,4) == 1,:);
average_packet_time = mean(succ_data(round(end/10):end,3))*1e-3;




%% Print out results
fprintf('Throughput = %.2f Gbps\n',throughput)
fprintf('Discard rate = %.2f\n',discard_rate)
fprintf('Average packet time = %.2f us\n',average_packet_time)
fprintf('\n---------------------------------\n')
throughput_avg_trials = throughput_avg_trials + throughput;
packet_time_avg_trials = packet_time_avg_trials + average_packet_time;
end
fprintf(['\nAverage throughput -> trials  %u protocol %s no. of nodes ' ...
'%u config %u is %.2f Gbps\n'], seedNum,protocol, nodeNum, config, throughput_avg_trials/seedNum);
fprintf(['\nAverage packet time -> trials  %u protocol %s no. of ' ...
         'nodes %u config %u is %.2f us\n'], seedNum, protocol, ...
        nodeNum,config, packet_time_avg_trials/seedNum');
fprintf('\n\n')