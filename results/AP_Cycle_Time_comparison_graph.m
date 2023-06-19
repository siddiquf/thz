% Read the data from the text file
% Read the data from the text file
data = dlmread('Avg_AP_CycleTIme_udp_tcp_config_20_21_26_29.txt');

% Extract the columns from the data
nodes = data(:, 1);
udpAPCycle18m = data(:, 2);
tcpAPCycle18m = data(:, 3);
udpAPCycle16_7m = data(:, 4);
tcpAPCycle16_7m = data(:, 5);
udpAPCycle40m = data(:, 6);
tcpAPCycle40m = data(:, 7);
udpAPCycle7_5m = data(:, 8);
tcpAPCycle7_5m = data(:, 9);

% Create a figure with larger size
fig = figure;
%fig.Position(3) = 800; % Set the desired width of the figure
fig.Position(4) = 600; % Set the desired height of the figure

% Plot the graphs on the same x-y axis
plot(nodes, udpAPCycle18m, 'b-x', 'LineWidth', 2);
hold on;
plot(nodes, tcpAPCycle18m, 'b--x', 'LineWidth', 2);

plot(nodes, udpAPCycle16_7m, 'r-x', 'LineWidth', 2);
plot(nodes, tcpAPCycle16_7m, 'r--x', 'LineWidth', 2);

plot(nodes, udpAPCycle40m, 'g-x', 'LineWidth', 2);
plot(nodes, tcpAPCycle40m, 'g--x', 'LineWidth', 2);


plot(nodes, udpAPCycle7_5m, 'm-x', 'LineWidth', 2);
plot(nodes, tcpAPCycle7_5m, 'm--x', 'LineWidth', 2);

% Set labels and title
xlabel('Number of Nodes');
ylabel('AP Cycle Time (Nanoseconds)');
%title('Average AP Cycle Time for 18m and 7.5m range (UDP and TCP)');

% Add a legend
%legend('UDP - 18m', 'TCP - 18m', 'UDP - 7.5m', 'TCP - 7.5m');
legend('UDP - 18m', 'TCP - 18m', 'UDP - 16.7m', 'TCP - 16.7m', 'UDP - 40m', 'TCP - 40m','UDP - 7.5m', 'TCP - 7.5m', ...
       'FontSize', 12, 'AutoUpdate', 'on');

% Increase font size of markers
set(gca, 'FontSize', 14);

% Adjust the plot appearance
grid on;
hold off;
