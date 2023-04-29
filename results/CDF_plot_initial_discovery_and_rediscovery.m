
configs = [21, 28, 29];

colors = {'#4DBEEE', 'red', 'black'};
lineWidth = 3;
fontSize = 16;
for c = 1:numel(configs)
    config = configs(c);

    data = [];
    for i = 1:10
        %Add the correct path to file
        file = sprintf('results/DiscoveryTime_result_3way_50n_500us_Config%d_%d.txt',config, i);
        delay_values = dlmread(file, '\t', 0, 3); % Read data starting from the first row and fourth column
        data = [data; delay_values]; % Concatenate data from each file
    end
    cdf = sort(data) / 1e6; % Convert from nanoseconds to milliseconds
    y = linspace(0, 1, numel(data));
    hold on;
   plot(cdf, y, 'Color', colors{c}, 'LineWidth', lineWidth,'DisplayName', "Initial discovery - config "  + num2str(config));
   
   
   % Add code to plot rediscovery time
    
    rediscovery_data = [];
    for i = 1:10
        %Add the correct path to file
        rediscovery_file = sprintf('results/RediscoveryTime_result_3way_50n_500us_Config%d_%d.txt',config, i);
        rediscovery_values = dlmread(rediscovery_file, '\t', 0, 1); % Read data starting from the first row and second column
        rediscovery_data = [rediscovery_data; rediscovery_values]; % Concatenate data from each file
    end
    rediscovery_cdf = sort(rediscovery_data) / 1e6; % Convert from nanoseconds to milliseconds
    rediscovery_y = linspace(0, 1, numel(rediscovery_data));
    plot(rediscovery_cdf, rediscovery_y, 'Color', colors{c}, 'LineWidth', lineWidth,  'DisplayName', "Rediscovery - config " + num2str(config), 'LineStyle', '--');

end

xlabel('Initial discovery/ rediscovery time (ms)', 'FontSize', fontSize);
ylabel('CDF', 'FontSize', fontSize);
legend('FontSize', fontSize);
set(gca, 'FontSize', fontSize, 'YLim', [0,1.05]);
grid on;
hold off;