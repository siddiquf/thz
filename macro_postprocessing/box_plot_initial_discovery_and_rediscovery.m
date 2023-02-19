configurations = [21, 28, 29];
discovery_color = 'b'; % blue
rediscovery_color = 'm'; % magenta
figure
hold on

for i = 1:length(configurations)
% Load data for discovery time
data_discovery = [];
for j = 1:10
    %Enter the correct path and the file name
    file_discovery = sprintf(['/scratch/DiscoveryTime_result_3way_50n_500us_Config%d_%d.txt'], configurations(i), j);

    trial_data_discovery = load(file_discovery);

    %the discovery time is located in column 4 in the file
    data_discovery = [data_discovery; trial_data_discovery(:, 4)];
end

%converting time recorded from ns to ms
data_discovery = data_discovery * 1e-6;

% Load data for rediscovery time
data_rediscovery = [];
for j = 1:10
    %Enter the correct path and the file name
    file_rediscovery = sprintf(['/scratch/RediscoveryTime_result_3way_50n_500us_Config%d_%d.txt'], configurations(i), j);
  
    trial_data_rediscovery = load(file_rediscovery);

    %the discovery time is located in column 2 in the file
    data_rediscovery = [data_rediscovery; trial_data_rediscovery(:, 2)];
end

%converting time recorded from ns to ms
data_rediscovery = data_rediscovery * 1e-6;


% Draw box plots for discovery time
boxplot(data_discovery, 'positions', i-0.2, 'widths', 0.2, 'Notch', 'on', 'Color', discovery_color)


% Draw box plots for rediscovery time
boxplot(data_rediscovery, 'positions', i+0.2, 'widths', 0.2, 'Notch', ...
        'on', 'Color', rediscovery_color  )

    


% Add mean points and connect them with lines for discovery time
mean_discovery = mean(data_discovery);
plot(i-0.2 * ones(1, length(mean_discovery)), mean_discovery, 'o', 'Color', 'black', 'MarkerSize', 8)


% Add mean points and connect them with lines for rediscovery time
mean_rediscovery = mean(data_rediscovery);
plot(i+0.2 * ones(1, length(mean_rediscovery)), mean_rediscovery, 'o', 'Color', 'black', 'MarkerSize', 8)




end

set(gca, 'XTick', 1:length(configurations), 'XTickLabel', configurations)
xlabel('Configurations', 'FontSize', 16)
ylabel('Initial discovery/ rediscovery time (ms)', 'FontSize', 16)

% make line widths thicker
set(findall(gca, 'type', 'line'), 'LineWidth', 1.5)

% make numbers on the axis larger
set(gca, 'FontSize', 16)
                    
boxes = findobj(gca, 'Tag', 'Box');
legend(boxes([end 1]), 'Initial discovery', 'Rediscovery')

hold off


