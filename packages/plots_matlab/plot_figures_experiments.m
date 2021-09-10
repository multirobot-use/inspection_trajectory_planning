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
while (mission_status.d(ind_start1, 1) ~= 1)
    ind_start1 = ind_start1 + 1;
end

% Give extra seconds to let the drones go to the cylinder
time_start_experiment = mission_status.t(ind_start1, 1) + 32;

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
    
    
    title_plot = ['Evolution of the trajectories at time \Deltat = ', num2str(time_vector(i)), ' s'];
    figure(1); set(gcf, 'Position', get(0, 'Screensize'));
    plot(d1_pose.p(ind_start1:ind_finish1, 1), d1_pose.p(ind_start1:ind_finish1, 2), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(d2_pose.p(ind_start2:ind_finish2, 1), d2_pose.p(ind_start2:ind_finish2, 2), 'r', 'LineWidth', 1.2);
    if n_drones == 3
        plot(d3_pose.p(ind_start2:ind_finish2, 1), d3_pose.p(ind_start2:ind_finish2, 2), 'g', 'LineWidth', 1.2);
    end
    plot(points_to_inspect.p(ind_finish_p2i, 1), points_to_inspect.p(ind_finish_p2i, 1), 'k*', 'MarkerSize', 16, 'LineWidth', 2);
    hold off;
    xlabel('Position X (m)'); ylabel('Position Y (m)'); title(title_plot);
    if n_drones == 3
        legend('Drone 1', 'Drone 2', 'Drone 3', 'Point to inspect', 'Location', 'Best');
    else
        legend('Drone 1', 'Drone 2', 'Point to inspect', 'Location', 'Best');
    end
    axis equal;


    if save_plots
        saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Drones_traj_', num2str(time_vector(i)), 's.png']);
    else
        pause
    end
end

%% Get drones accelerations by derivating the position twice
% LINEAR: X, Y, Z
d1_acc.t = d1_velocity.t;
d2_acc.t = d2_velocity.t;
if n_drones == 3
    d3_acc.t = d3_velocity.t;
end

% % Filtering velocity
B = 1/30*ones(30,1);
d1_velocity.l = filter(B, 1, d1_velocity.l);
d1_velocity.a = filter(B, 1, d1_velocity.a);
d2_velocity.l = filter(B, 1, d2_velocity.l);
d2_velocity.a = filter(B, 1, d2_velocity.a);

if n_drones == 3
    d3_velocity.l = filter(B, 1, d3_velocity.l);
    d3_velocity.a = filter(B, 1, d3_velocity.a);
end

for i = 1:(size(d1_velocity.l, 1) - 1)
    d1_acc.l(i, :) = (d1_velocity.l(i+1, :) - d1_velocity.l(i, :))/inc_time_pose;
    d1_acc.a(i, 2:3) = (d1_velocity.a(i+1, 2:3) - d1_velocity.a(i, 2:3))/inc_time_pose;
end

d1_acc.l(size(d1_velocity.l, 1), :) = d1_acc.l(i, :);
d1_acc.a(size(d1_velocity.a, 1), 2:3) = d1_acc.a(i, 2:3);

for i = 1:(size(d2_velocity.l, 1) - 1)
    d2_acc.l(i, :) = (d2_velocity.l(i+1, :) - d2_velocity.l(i, :))/inc_time_pose;
    d2_acc.a(i, 2:3) = (d2_velocity.a(i+1, 2:3) - d2_velocity.a(i, 2:3))/inc_time_pose;
end

d2_acc.l(size(d2_velocity.l, 1), :) = d2_acc.l(i, :);
d2_acc.a(size(d2_velocity.a, 1), 2:3) = d2_acc.a(i, 2:3);

if n_drones == 3
    
    for i = 1:(size(d3_velocity.l, 1) - 1)
        d3_acc.l(i, :) = (d3_velocity.l(i+1, :) - d3_velocity.l(i, :))/inc_time_pose;
        d3_acc.a(i, 2:3) = (d3_velocity.a(i+1, 2:3) - d3_velocity.a(i, 2:3))/inc_time_pose;
    end
    
    d3_acc.l(size(d3_velocity.l, 1), :) = d3_acc.l(i-1, :);
    d3_acc.a(size(d3_velocity.a, 1), 2:3) = d3_acc.a(i, 2:3);
    
end

d1_acc.l = filter(B, 1, d1_acc.l);
d1_acc.a = filter(B, 1, d1_acc.a);
d2_acc.l = filter(B, 1, d2_acc.l);
d2_acc.a = filter(B, 1, d2_acc.a);

if n_drones == 3
    d3_acc.l = filter(B, 1, d3_acc.l);
    d3_acc.a = filter(B, 1, d3_acc.a);
end

% Get closer sample to time_starting_recording for each drone
[~, ind_start1] = min(sqrt(abs(d1_acc.t.^2 - (time_start_experiment)^2)));
[~, ind_start2] = min(sqrt(abs(d2_acc.t.^2 - (time_start_experiment)^2)));
if n_drones == 3
    [~, ind_start3] = min(sqrt(abs(d3_acc.t.^2 - (time_start_experiment)^2)));
end

% Module of acceleration
d1_mod_acc = sqrt(d1_acc.l(:,1).^2 + d1_acc.l(:,2).^2 + d1_acc.l(:,3).^2);
d2_mod_acc = sqrt(d2_acc.l(:,1).^2 + d2_acc.l(:,2).^2 + d2_acc.l(:,3).^2);

figure(2); set(gcf, 'Position', get(0, 'Screensize'));
if n_drones == 3
    d3_mod_acc = sqrt(d3_acc.l(:,1).^2 + d3_acc.l(:,2).^2 + d3_acc.l(:,3).^2);
    subplot(3,1,1);
    plot(d1_acc.t(ind_start1:end, 1), d1_mod_acc(ind_start1:end, 1), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 1');
    legend('Drone_{1}', 'Location', 'Best'); axis tight;

    subplot(3,1,2);
    plot(d2_acc.t(ind_start2:end, 1), d2_mod_acc(ind_start2:end, 1), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 2');
    legend('Drone_{2}', 'Location', 'Best'); axis tight;

    subplot(3,1,3);
    plot(d3_acc.t(ind_start3:end, 1), d3_mod_acc(ind_start3:end, 1), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 3');
    legend('Drone_{3}', 'Location', 'Best'); axis tight;
else
    subplot(2,1,1);
    plot(d1_acc.t(ind_start1:end, 1), d1_mod_acc(ind_start1:end, 1), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 1');
    legend('Drone_{1}', 'Location', 'Best'); axis tight;

    subplot(2,1,2);
    plot(d2_acc.t(ind_start2:end, 1), d2_mod_acc(ind_start2:end, 1), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Linear acceleration (m/s²)'); title('Evolution of the linear acceleration of Drone 2');
    legend('Drone_{2}', 'Location', 'Best'); axis tight;
end

if save_plots
    saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Drones_linear_acceleration.png']);
end




% ANGULAR: Y (Pitch) and Z (Yaw)
% PITCH
figure(3); set(gcf, 'Position', get(0, 'Screensize'));
if n_drones == 3
    subplot(3,1,1);
    plot(d1_acc.t(ind_start1:end, 1), d1_acc.a(ind_start1:end, 2), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the pitch acceleration of Drone 1');
    legend('Drone_{1}', 'Location', 'Best'); axis tight;

    subplot(3,1,2);
    plot(d2_acc.t(ind_start2:end, 1), d2_acc.a(ind_start2:end, 2), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the pitch acceleration of Drone 2');
    legend('Drone_{2}', 'Location', 'Best'); axis tight;

    subplot(3,1,3);
    plot(d3_acc.t(ind_start3:end, 1), d3_acc.a(ind_start3:end, 2), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the pitch acceleration of Drone 3');
    legend('Drone_{3}', 'Location', 'Best'); axis tight;
else
    subplot(2,1,1);
    plot(d1_acc.t(ind_start1:end, 1), d1_acc.a(ind_start1:end, 2), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the pitch acceleration of Drone 1');
    legend('Drone_{1}', 'Location', 'Best'); axis tight;

    subplot(2,1,2);
    plot(d2_acc.t(ind_start2:end, 1), d2_acc.a(ind_start2:end, 2), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the pitch acceleration of Drone 2');
    legend('Drone_{2}', 'Location', 'Best'); axis tight;
end

if save_plots
    saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Drones_pitch_acceleration.png']);
end



% YAW
figure(4); set(gcf, 'Position', get(0, 'Screensize'));
if n_drones == 3
    subplot(3,1,1);
    plot(d1_acc.t(ind_start1:end, 1), d1_acc.a(ind_start1:end, 3), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the yaw acceleration of Drone 1');
    legend('Drone_{1}', 'Location', 'Best'); axis tight;

    subplot(3,1,2);
    plot(d2_acc.t(ind_start2:end, 1), d2_acc.a(ind_start2:end, 3), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the yaw acceleration of Drone 2');
    legend('Drone_{2}', 'Location', 'Best'); axis tight;

    subplot(3,1,3);
    plot(d3_acc.t(ind_start3:end, 1), d3_acc.a(ind_start3:end, 3), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the yaw acceleration of Drone 3');
    legend('Drone_{3}', 'Location', 'Best'); axis tight;
else
    subplot(2,1,1);
    plot(d1_acc.t(ind_start1:end, 1), d1_acc.a(ind_start1:end, 3), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the yaw acceleration of Drone 1');
    legend('Drone_{1}', 'Location', 'Best'); axis tight;

    subplot(2,1,2);
    plot(d2_acc.t(ind_start2:end, 1), d2_acc.a(ind_start2:end, 3), 'b', 'LineWidth', 1.5); grid on;
    xlabel('Time (s)'); ylabel('Angular acceleration (rad/s²)'); title('Evolution of the yaw acceleration of Drone 2');
    legend('Drone_{2}', 'Location', 'Best'); axis tight;
end

if save_plots
    saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Drones_yaw_acceleration.png']);
end




%% EVOLUTION OF DISTANCE TO INSPECTION POINT
% Calculate the evolution of the distance to inspection point
d1_d2ip = zeros(size(d1_pose.p,1), 1);
d2_d2ip = zeros(size(d2_pose.p,1), 1);

for i = 1:size(d1_pose.p,1)
    [value1_d2ip, ind1_d2ip] = min(sqrt(abs(d1_pose.t(i,1)^2 - points_to_inspect.t.^2)));
    d1_d2ip(i) = sqrt((d1_pose.p(i, 1) - points_to_inspect.p(ind1_d2ip, 1))^2 + (d1_pose.p(i, 2) - points_to_inspect.p(ind1_d2ip, 2))^2);

end

for i = 1:size(d2_pose.p,1)

    [value2_d2ip, ind2_d2ip] = min(sqrt(abs(d2_pose.t(i,1)^2 - points_to_inspect.t.^2)));
    d2_d2ip(i) = sqrt((d2_pose.p(i, 1) - points_to_inspect.p(ind2_d2ip, 1))^2 + (d2_pose.p(i, 2) - points_to_inspect.p(ind2_d2ip, 2))^2);

end

if n_drones == 3
    d3_d2ip = zeros(size(d3_pose.p,1), 1);
    for i = 1:size(d3_pose.p,1)
        
        [value3_d2ip, ind3_d2ip] = min(sqrt(abs(d3_pose.t(i,1)^2 - points_to_inspect.t.^2)));
        d3_d2ip(i) = sqrt((d3_pose.p(i, 1) - points_to_inspect.p(ind3_d2ip, 1))^2 + (d3_pose.p(i, 2) - points_to_inspect.p(ind3_d2ip, 2))^2);
        
    end
end

% Get closer sample to time_starting_recording for each drone
[~, ind_start1] = min(sqrt(abs(d1_pose.t.^2 - (time_start_experiment)^2)));
[~, ind_start2] = min(sqrt(abs(d2_pose.t.^2 - (time_start_experiment)^2)));
if n_drones == 3
    [~, ind_start3] = min(sqrt(abs(d3_pose.t.^2 - (time_start_experiment)^2)));
end

[~, ind_start_d] = min(sqrt(abs(distance_inspection.t.^2 - (time_start_experiment)^2)));




figure(5); set(gcf, 'Position', get(0, 'Screensize'));
if n_drones == 3
    subplot(3,1,1);
    plot(d1_pose.t(ind_start1:end), d1_d2ip(ind_start1:end), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t(ind_start_d:end), distance_inspection.d(ind_start_d:end), 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Evolution of the distance to inspection point by Drone 1');
    legend('Drone_{1}', 'Reference', 'Location', 'Best');
    axis tight;
    
    subplot(3,1,2);
    plot(d2_pose.t(ind_start2:end), d2_d2ip(ind_start2:end), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t(ind_start_d:end), distance_inspection.d(ind_start_d:end), 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Evolution of the distance to inspection point by Drone 2');
    legend('Drone_{2}', 'Reference', 'Location', 'Best');
    axis tight;
    
    subplot(3,1,3);
    plot(d3_pose.t(ind_start3:end), d3_d2ip(ind_start3:end), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t(ind_start_d:end), distance_inspection.d(ind_start_d:end), 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Evolution of the distance to inspection point by Drone 3');
    legend('Drone_{3}', 'Reference', 'Location', 'Best');
    axis tight;
else
    subplot(2,1,1);
    plot(d1_pose.t(ind_start1:end), d1_d2ip(ind_start1:end), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t(ind_start_d:end), distance_inspection.d(ind_start_d:end), 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Distance (m)');
    title('Evolution of the distance to inspection point by Drone 1');
    legend('Drone_{1}', 'Reference', 'Location', 'Best');
    axis tight;
    
    subplot(2,1,2);
    plot(d2_pose.t(ind_start2:end), d2_d2ip(ind_start2:end), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(distance_inspection.t(ind_start_d:end), distance_inspection.d(ind_start_d:end), 'r', 'LineWidth', 1.2); hold off;
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
if ( size(d1_pose.p,1) == size(d2_pose.p,1) )
    d12_rel_ang = zeros(size(d1_pose.p,1), 1);
    
else
    if ( size(d1_pose.p,1) >= size(d2_pose.p,1) )
        d12_rel_ang = zeros(size(d2_pose.p,1), 1);
        
    else
        d12_rel_ang = zeros(size(d1_pose.p,1), 1);
        
    end
end

size12 = size(d12_rel_ang,1);

for i = 1:size(d12_rel_ang,1)
    [value1_d2ip, ind1_d2ip] = min(sqrt(abs(d1_pose.t(i,1)^2 - points_to_inspect.t.^2)));
    [value2_d2ip, ind2_d2ip] = min(sqrt(abs(d2_pose.t(i,1)^2 - points_to_inspect.t.^2)));
    
    angle_1 = atan2((d1_pose.p(i, 2) - points_to_inspect.p(ind1_d2ip, 2)), (d1_pose.p(i, 1) - points_to_inspect.p(ind1_d2ip, 1)));
    if angle_1 < 0
        angle_1 = angle_1 + 2*pi;
    end
    
    angle_2 = atan2((d2_pose.p(i, 2) - points_to_inspect.p(ind2_d2ip, 2)), (d2_pose.p(i, 1) - points_to_inspect.p(ind2_d2ip, 1)));
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
    if ( size(d1_pose.p,1) == size(d3_pose.p,1) )
        d13_rel_ang = zeros(size(d1_pose.p,1), 1);

    else
        if ( size(d1_pose.p,1) >= size(d3_pose.p,1) )
            d13_rel_ang = zeros(size(d3_pose.p,1), 1);

        else
            d13_rel_ang = zeros(size(d1_pose.p,1), 1);

        end
    end
    
    size13 = size(d13_rel_ang,1);
    
    for i = 1:size(d13_rel_ang,1)
        [value1_d2ip, ind1_d2ip] = min(sqrt(abs(d1_pose.t(i,1)^2 - points_to_inspect.t.^2)));
        [value3_d2ip, ind3_d2ip] = min(sqrt(abs(d3_pose.t(i,1)^2 - points_to_inspect.t.^2)));
        
        angle_1 = atan2((d1_pose.p(i, 2) - points_to_inspect.p(ind1_d2ip, 2)), (d1_pose.p(i, 1) - points_to_inspect.p(ind1_d2ip, 1)));
        if angle_1 < 0
            angle_1 = angle_1 + 2*pi;
        end
        
        angle_3 = atan2((d3_pose.p(i, 2) - points_to_inspect.p(ind3_d2ip, 2)), (d3_pose.p(i, 1) - points_to_inspect.p(ind3_d2ip, 1)));
        if angle_3 < 0
            angle_3 = angle_3 + 2*pi;
        end
        
        if (abs(angle_3 - angle_1) < pi)
            d13_rel_ang(i) = abs(angle_3 - angle_1);
        else
            if angle_3 > angle_1
                d13_rel_ang(i) = abs(2*pi - (angle_3 - angle_1));
            else
                d13_rel_ang(i) = abs(2*pi - (angle_1 - angle_3));
            end
        end
        
    end
end

% Get closer sample to time_starting_recording for each drone
[~, ind_start2] = min(sqrt(abs(d2_pose.t.^2 - (time_start_experiment)^2)));
if n_drones == 3
    [~, ind_start3] = min(sqrt(abs(d3_pose.t.^2 - (time_start_experiment)^2)));
end

[~, ind_start_ra] = min(sqrt(abs(relative_angle.t.^2 - (time_start_experiment)^2)));


figure(6); set(gcf, 'Position', get(0, 'Screensize'));
if n_drones == 3
    subplot(2,1,1);
    plot(d2_pose.t(ind_start2:size12), d12_rel_ang(ind_start2:size12)*(180/pi), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(relative_angle.t(ind_start_ra:end), relative_angle.d(ind_start_ra:end)*(180/pi), 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Angle (rad)');
    title('Evolution of the relative angle by Drone 2');
    legend('Drone_{2}', 'Reference', 'Location', 'Best');
    axis tight;
    
    subplot(2,1,2);
    plot(d3_pose.t(ind_start3:size13), d13_rel_ang(ind_start3:size13)*(180/pi), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(relative_angle.t(ind_start_ra:end), relative_angle.d(ind_start_ra:end)*(180/pi), 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Angle (rad)');
    title('Evolution of the relative angle by Drone 3');
    legend('Drone_{3}', 'Reference', 'Location', 'Best');
    axis tight;

else
    plot(d2_pose.t(ind_start2:size12), d12_rel_ang(ind_start2:size12)*(180/pi), 'b', 'LineWidth', 1.5); grid on; hold on;
    plot(relative_angle.t(ind_start_ra:end), relative_angle.d(ind_start_ra:end)*(180/pi), 'r', 'LineWidth', 1.2); hold off;
    xlabel('Time (s)'); ylabel('Angle (rad)');
    title('Evolution of the relative angle by Drone 2');
    legend('Drone_{2}', 'Reference', 'Location', 'Best');
    axis tight;

end


if save_plots
    saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Relative_angles.png']);
end




