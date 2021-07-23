%% PLOTTING FIGURES
% Script for plotting and saving the different figures
%% EVOLUTION OF DRONES
if n_drones == 3
    time_start_recording = max([start_time_1, start_time_2, start_time_3]);
else
    time_start_recording = max([start_time_1, start_time_2]);
end

% Start evaluating when the drones start moving. Since then, check the
% following time_vector instants of time
time_vector = [10, 20, 30, 40, 50, 60];

% Getting increment time of pose topic
inc_time_pose = d1_pose.t(2) - d1_pose.t(1);

% The real start time for the experiment is when the leader drone starts
% moving
ind_start1 = 1;
while (sqrt(d1_velocity.l(ind_start1,1)^2 + d1_velocity.l(ind_start1,2)^2) < 0.4)
    ind_start1 = ind_start1 + 1;
end

time_start_experiment = d1_pose.t(ind_start1, 1);
% Get closer sample to time_starting_experiment for each drone
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
    
    
    title_plot = ['Evolution of the trajectories at time \Deltat = ', num2str(time_vector(i)), ' s'];
    figure(1); set(gcf, 'Position', get(0, 'Screensize'));
    plot(d1_pose.p(ind_start1:ind_finish1, 1), d1_pose.p(ind_start1:ind_finish1, 2), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(d1_pose.p(ind_start2:ind_finish2, 1), d1_pose.p(ind_start2:ind_finish2, 2), 'r', 'LineWidth', 1.2);
    if n_drones == 3
        plot(d1_pose.p(ind_start2:ind_finish2, 1), d1_pose.p(ind_start2:ind_finish2, 2), 'g', 'LineWidth', 1.2);
    end
    plot(points_to_inspect.p(ind_finish_p2i, 1), points_to_inspect.p(ind_finish_p2i, 1), 'k*', 'MarkerSize', 16, 'LineWidth', 2);
    hold off;
    xlabel('Position X (m)'); ylabel('Position Y (m)'); title(title_plot);
    if n_drones == 3
        legend('Drone 1', 'Drone 2', 'Drone 3', 'Point to inspect', 'Location', 'Best');
    else
        legend('Drone 1', 'Drone 2', 'Point to inspect', 'Location', 'Best');
    end
%     axis tight;


    if save_plots
        saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Drones_traj_', num2str(time_vector(i)), 's.png']);
    end
    
%     pause
end

%% Get drones accelerations by derivating the position twice
d1_acc.t = d1_pose.t;
d2_acc.t = d2_pose.t;
if n_drones == 3
    d3_acc.t = d3_pose.t;
end


% for i = 1:(size(d1_pose.p, 1) - 1)
%     d1_der_vel.l(i,:) = (d1_pose.p(i+1,:) - d1_pose.p(i,:))/inc_time_pose;
%     d2_der_vel.l(i,:) = (d2_pose.p(i+1,:) - d2_pose.p(i,:))/inc_time_pose;
% end
% 
% d1_der_vel.l(size(d1_pose.p, 1), :) = d1_der_vel.l(i, :);
% d2_der_vel.l(size(d1_pose.p, 1), :) = d2_der_vel.l(i, :);


for i = 1:(size(d1_pose.p, 1) - 1)
    d1_acc.l(i, :) = (d1_velocity.l(i+1, :) - d1_velocity.l(i, :))/inc_time_pose;
    d2_acc.l(i, :) = (d2_velocity.l(i+1, :) - d2_velocity.l(i, :))/inc_time_pose;
end

d1_acc.l(size(d1_pose.p, 1), :) = d1_acc.l(i, :);
d2_acc.l(size(d1_pose.p, 1), :) = d2_acc.l(i, :);

if n_drones == 3
%     for i = 1:(size(d1_pose.p, 1) - 1)
%         d3_der_vel.l(i, :) = (d3_pose.p(i+1, :) - d3_pose.p(i, :))/inc_time_pose;
%     end
%     
%     d3_der_vel.l(size(d1_pose.p, 1), :) = d3_der_vel.l(i-1, :);
    
    for i = 1:(size(d1_pose.p, 1) - 1)
        d3_acc.l(i, :) = (d3_velocity.l(i+1, :) - d3_velocity.l(i, :))/inc_time_pose;
    end
    
    d3_acc.l(size(d1_pose.p, 1), :) = d3_acc.l(i-1, :);
    
end

% Module of acceleration
d1_mod_acc = sqrt(d1_acc.l(:,1).^2 + d1_acc.l(:,2).^2 + d1_acc.l(:,3).^2);
d2_mod_acc = sqrt(d2_acc.l(:,1).^2 + d2_acc.l(:,2).^2 + d2_acc.l(:,3).^2);

figure(2); set(gcf, 'Position', get(0, 'Screensize'));
if n_drones == 3
    d3_mod_acc = sqrt(d3_acc.l(:,1).^2 + d3_acc.l(:,2).^2 + d3_acc.l(:,3).^2);
    subplot(3,1,1);
    plot(d1_acc.t, d1_mod_acc, 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 1');
    legend('Drone_{1}', 'Location', 'Best'); axis tight;

    subplot(3,1,2);
    plot(d2_acc.t, d2_mod_acc, 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 2');
    legend('Drone_{2}', 'Location', 'Best'); axis tight;

    subplot(3,1,3);
    plot(d3_acc.t, d3_mod_acc, 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 3');
    legend('Drone_{3}', 'Location', 'Best'); axis tight;
else
    subplot(2,1,1);
    plot(d1_acc.t, d1_mod_acc, 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 1');
    legend('Drone_{1}', 'Location', 'Best'); axis tight;

    subplot(2,1,2);
    plot(d2_acc.t, d2_mod_acc, 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 2');
    legend('Drone_{2}', 'Location', 'Best'); axis tight;
end

if save_plots
    saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Drones_acceleration.png']);
end

%% EVOLUTION OF DISTANCE TO INSPECTION POINT
% Calculate the evolution of the distance to inspection point
d1_d2ip = zeros(size(d1_pose.p,1), 1);
d2_d2ip = zeros(size(d2_pose.p,1), 1);

if n_drones == 3
    d3_d2ip(i) = zeros(size(d3_pose.p,1));
    for i = 1:size(d1_pose.p,1)
        [value_d2ip, ind_d2ip] = min(sqrt(abs(d1_pose.t(i,1)^2 - points_to_inspect.t.^2)));
        d1_d2ip(i) = sqrt((d1_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1))^2 + (d1_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2))^2);

        [value_d2ip, ind_d2ip] = min(sqrt(abs(d2_pose.t(i,1)^2 - points_to_inspect.t.^2)));
        d2_d2ip(i) = sqrt((d2_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1))^2 + (d2_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2))^2);
        
        [value_d2ip, ind_d2ip] = min(sqrt(abs(d3_pose.t(i,1)^2 - points_to_inspect.t.^2)));
        d3_d2ip(i) = sqrt((d3_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1))^2 + (d3_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2))^2);
        
    end
    
    
else
    for i = 1:size(d1_pose.p,1)
        [value_d2ip, ind_d2ip] = min(sqrt(abs(d1_pose.t(i,1)^2 - points_to_inspect.t.^2)));
        d1_d2ip(i) = sqrt((d1_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1))^2 + (d1_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2))^2);

        [value_d2ip, ind_d2ip] = min(sqrt(abs(d2_pose.t(i,1)^2 - points_to_inspect.t.^2)));
        d2_d2ip(i) = sqrt((d2_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1))^2 + (d2_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2))^2);
         
    end
end

figure(3); set(gcf, 'Position', get(0, 'Screensize'));
if n_drones == 3
    subplot(3,1,1);
    plot(d1_pose.t, d1_d2ip, 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t, distance_inspection.d, 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Evolution of the distance to inspection point by Drone 1');
    legend('Drone_{1}', 'Reference', 'Location', 'Best');
    axis tight;
    
    subplot(3,1,2);
    plot(d2_pose.t, d2_d2ip, 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t, distance_inspection.d, 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Evolution of the distance to inspection point by Drone 2');
    legend('Drone_{2}', 'Reference', 'Location', 'Best');
    axis tight;
    
    subplot(3,1,1);
    plot(d3_pose.t, d3_d2ip, 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t, distance_inspection.d, 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Evolution of the distance to inspection point by Drone 3');
    legend('Drone_{3}', 'Reference', 'Location', 'Best');
    axis tight;
else
    subplot(2,1,1);
    plot(d1_pose.t, d1_d2ip, 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t, distance_inspection.d, 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Evolution of the distance to inspection point by Drone 1');
    legend('Drone_{1}', 'Reference', 'Location', 'Best');
    axis tight;
    
    subplot(2,1,2);
    plot(d2_pose.t, d2_d2ip, 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t, distance_inspection.d, 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Evolution of the distance to inspection point by Drone 2');
    legend('Drone_{2}', 'Reference', 'Location', 'Best');
    
    axis tight;
end

if save_plots
    saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Distance_to_inspection_point.png']);
end
    
%% EVOLUTION OF RELATIVE ANGLE
% Calculate the evolution of the distance to inspection point
d12_rel_ang = zeros(size(d1_pose.p,1), 1);

for i = 1:size(d1_pose.p,1)
    [value_d2ip, ind_d2ip] = min(sqrt(abs(d2_pose.t(i,1)^2 - points_to_inspect.t.^2)));
    angle_1 = atan2((d1_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2)), (d1_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1)));
    if angle_1 < 0
        angle_1 = angle_1 + 2*pi;
    end
    angle_2 = atan2((d2_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2)), (d2_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1)));
    if angle_2 < 0
        angle_2 = angle_2 + 2*pi;
    end
    
    if (abs(angle_2 - angle_1) < pi)
        d12_rel_ang(i) = abs(angle_2 - angle_1);
    else
        if angle_2 > angle_1
            d12_rel_ang(i) = abs(2*pi - (angle_2 - angle_1));
        else
            d12_rel_ang(i) = abs(2*pi - (angle_1 - angle_2));
        end
    end
end

if n_drones == 3
    d13_d2ip(i) = zeros(size(d3_pose.p,1));
    for i = 1:size(d1_pose.p,1)
        [value_d2ip, ind_d2ip] = min(sqrt(abs(d3_pose.t(i,1)^2 - points_to_inspect.t.^2)));
        angle_1 = atan2((d1_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2)), (d1_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1)));
        if angle_1 < 0
            angle_1 = angle_1 + 2*pi;
        end
        
        angle_3 = atan2((d3_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2)), (d3_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1)));
        if angle_3 < 0
            angle_3 = angle_3 + 2*pi;
        end
        
        d1_d2ip(i) = sqrt((d1_pose.p(i, 1) - points_to_inspect.p(ind_d2ip, 1))^2 + (d1_pose.p(i, 2) - points_to_inspect.p(ind_d2ip, 2))^2);
        
        if (abs(angle_2 - angle_1) < pi)
            d13_rel_ang(i) = abs(angle_2 - angle_1);
        else
            if angle_3 > angle_1
                d13_rel_ang(i) = abs(2*pi - (angle_3 - angle_1));
            else
                d13_rel_ang(i) = abs(2*pi - (angle_1 - angle_3));
            end
        end
        
    end
end

figure(4); set(gcf, 'Position', get(0, 'Screensize'));
if n_drones == 3
    subplot(2,1,1);
    plot(d2_pose.t, d12_rel_ang, 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(relative_angle.t, relative_angle.d, 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Angle (rad)');
    title('Evolution of the relative angle by Drone 2');
    legend('Drone_{1}', 'Reference', 'Location', 'Best');
    axis tight;
    
    subplot(2,1,2);
    plot(d3_pose.t, d13_rel_ang, 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(relative_angle.t, relative_angle.d, 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Angle (rad)');
    title('Evolution of the relative angle by Drone 3');
    legend('Drone_{2}', 'Reference', 'Location', 'Best');
    axis tight;

else
    plot(d2_pose.t, d12_rel_ang, 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(relative_angle.t, relative_angle.d, 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Angle (rad)');
    title('Evolution of the relative angle by Drone 2');
    legend('Drone_{1}', 'Reference', 'Location', 'Best');
    axis tight;
    
    axis tight;
end


if save_plots
    saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Relative_angles.png']);
end




