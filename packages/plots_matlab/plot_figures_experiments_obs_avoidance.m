%% PLOTTING FIGURES
% Script for plotting and saving the different figures
%% EVOLUTION OF DRONES
if n_drones == 3
    time_start_recording = max([start_time_1, start_time_2, start_time_3]);
else
    time_start_recording = max([start_time_1, start_time_2]);
end

% Obstacles position
obstacles = [3.5, 0; -3.5, 0;
             9.5, 0; -9.5, 0;
             5,   5; -5,   5;
             5,  -5; -5,  -5];

% Start evaluating when the drones start moving. Since then, check the
% following time_vector instants of time
time_vector = [6, 12, 18, 30, 40, 50];

% Getting increment time of pose topic
inc_time_pose = d1_pose.t(2) - d1_pose.t(1);

% The real start time for the experiment is when the leader drone starts
% moving
ind_start1 = 1;
while (mission_status.d(ind_start1, 1) ~= 1)
    ind_start1 = ind_start1 + 1;
end

% Give extra seconds to let the drones go to the cylinder
time_start_experiment = mission_status.t(ind_start1, 1) + 110;

% Get closer sample to time_starting_experiment for each drone
[value1, ind_start1] = min(sqrt(abs(d1_pose.t.^2 - time_start_experiment^2)));
[value2, ind_start2] = min(sqrt(abs(d2_pose.t.^2 - time_start_experiment^2)));
if n_drones == 3
    [value3, ind_start3] = min(sqrt(abs(d3_pose.t.^2 - time_start_experiment^2)));
end

for i = 1:size(time_vector,2)
    
    % Get closer sample to time_starting_recording for each drone
    [value1, ind_finish1] = min(sqrt(abs(d1_pose.t.^2 - (time_start_experiment + time_vector(i))^2)));
    [value2, ind_finish2] = min(sqrt(abs(d2_pose.t.^2 - (time_start_experiment + time_vector(i))^2)));
    if n_drones == 3
        [value3, ind_finish3] = min(sqrt(abs(d3_pose.t.^2 - (time_start_experiment + time_vector(i))^2)));
    end
    [value_p2i, ind_finish_p2i] = min(sqrt(abs(points_to_inspect.t.^2 - (time_start_experiment + time_vector(i))^2)));
    
    
    title_plot = ['Evolution of the trajectories at time t = ', num2str(time_vector(i)), ' s'];
    if i == 1
        figure(1); set(gcf, 'Position', get(0, 'Screensize'));
    end
    if i == 1
        subplot(3, 2, 1);
    else
        if i == 2
            subplot(3, 2, 2);
        else
            if i == 3
                subplot(3, 2, 3);
            else
                if i == 4
                    subplot(3, 2, 4);
                else
                    if i == 5
                        subplot(3, 2, 5);
                    else
                        if i == 6
                            subplot(3, 2, 6);
                        end
                    end
                end
            end
        end
    end
    plot(d1_pose.p(ind_start1:ind_finish1, 1), d1_pose.p(ind_start1:ind_finish1, 2), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(d2_pose.p(ind_start2:ind_finish2, 1), d2_pose.p(ind_start2:ind_finish2, 2), 'r', 'LineWidth', 1.2);
    if n_drones == 3
        plot(d3_pose.p(ind_start2:ind_finish2, 1), d3_pose.p(ind_start2:ind_finish2, 2), 'g', 'LineWidth', 1.2);
    end
    plot(points_to_inspect.p(ind_finish_p2i, 1), points_to_inspect.p(ind_finish_p2i, 1), 'k*', 'MarkerSize', 16, 'LineWidth', 2);
    plot(obstacles(:, 1), obstacles(:, 2), 'k.', 'MarkerSize', 50, 'LineWidth', 2);
    hold off;
    
    if i >= 5
        xlabel('X (m)','FontSize',18,'FontWeight','bold'); 
    end
    
    if i == 3 || i == 4
        ylabel('Y (m)','FontSize',18,'FontWeight','bold');
    end
    
    title(title_plot,'FontSize',16,'FontWeight','bold');
    
    if i == 1
        if n_drones == 3
            legend({'Drone 1', 'Drone 2', 'Drone 3', 'Point to inspect', 'Obstacles'}, 'FontSize', 15, 'Location', 'Best');
        else
            legend({'Drone 1', 'Drone 2', 'Point to inspect', 'Obstacles'}, 'FontSize', 15, 'Location', 'Best');
        end
    end
    axis equal;
end

if save_plots
    saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Drones_traj_mosaic.png']);
end
