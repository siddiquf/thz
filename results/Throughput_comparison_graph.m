% Read the data from the text file
data = dlmread('Avg_throughput_udp_tcp_config_20_21_26_29.txt');


% Extract the columns from the data
nodes = data(:, 1);
udpThroughput18m = data(:, 2);
tcpThroughput18m = data(:, 3);
udpThroughput16_7m = data(:, 4);
tcpThroughput16_7m = data(:, 5);
udpThroughput40m = data(:, 6);
tcpThroughput40m = data(:, 7);
udpThroughput7_5m = data(:, 8);
tcpThroughput7_5m = data(:, 9);

% Create a figure with larger size
fig = figure;
%fig.Position(3) = 800; % Set the desired width of the figure
fig.Position(4) = 600; % Set the desired height of the figure

% Plot the graphs on the same x-y axis with markers
plot(nodes, udpThroughput18m, 'b-o', 'LineWidth', 2);
hold on;
plot(nodes, tcpThroughput18m, 'b--o', 'LineWidth', 2, 'Color', [0 0 1 0.5]);

plot(nodes, udpThroughput16_7m, 'r-o', 'LineWidth', 2);
plot(nodes, tcpThroughput16_7m, 'r--o', 'LineWidth', 2);

plot(nodes, udpThroughput40m, 'g-o', 'LineWidth', 2);
plot(nodes, tcpThroughput40m, 'g--o', 'LineWidth', 2);

plot(nodes, udpThroughput7_5m, 'm-o', 'LineWidth', 2);
plot(nodes, tcpThroughput7_5m, 'm--o', 'LineWidth', 2);

% Set labels and title
xlabel('Number of Nodes','FontSize', 14);
ylabel('Average Throughput (Gbps)', 'FontSize', 14);
%title('Average Throughput for 18m and 7.5m range (UDP and TCP)');

% Add a legend
legend('UDP - 18m', 'TCP - 18m', 'UDP - 16.7m', 'TCP - 16.7m', 'UDP - 40m', 'TCP - 40m','UDP - 7.5m', 'TCP - 7.5m', ...
       'FontSize', 12, 'AutoUpdate', 'on');



% Turn off the hold mode
hold off;

% Set the y-axis limits
%ylim([0, 250]);
yMax = max([udpThroughput18m; tcpThroughput18m; udpThroughput16_7m; tcpThroughput16_7m; udpThroughput40m; tcpThroughput40m; udpThroughput7_5m; tcpThroughput7_5m]);
ylim([0, yMax + 10]);

% Increase font size of markers
set(gca, 'FontSize', 14);

% Adjust the plot appearance
grid on;
hold off;

