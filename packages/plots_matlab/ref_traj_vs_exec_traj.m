%% MATLAB script for plotting reference_trajectory vs executed trajectory

% Times of interest

vector_time = [100, 102, 104, 106, 108];
    
for i = 1:size(vector_time, 2)-1
    % Get closer sample to time_starting_recording for each drone
    [~, ind_traj_start1] = min(sqrt(abs(d1_ref_traj.t.^2 - (vector_time(i))^2)));
    [~, ind_traj_start2] = min(sqrt(abs(d2_ref_traj.t.^2 - (vector_time(i))^2)));
    if n_drones == 3
        [~, ind_traj_start3] = min(sqrt(abs(d3_ref_traj.t.^2 - (vector_time(i))^2)));
    end
    
    % Start of pose
    [~, ind_pose_start1] = min(sqrt(abs(d1_pose.t.^2 - (d1_ref_traj.t(ind_traj_start1))^2)));
    [~, ind_pose_start2] = min(sqrt(abs(d2_pose.t.^2 - (d2_ref_traj.t(ind_traj_start2))^2)));
    if n_drones == 3
        [~, ind_pose_start3] = min(sqrt(abs(d3_pose.t.^2 - (d3_ref_traj.t(ind_traj_start3))^2)));
    end
    
    % Final of pose
    [~, ind_pose_final1] = min(sqrt(abs(d1_pose.t.^2 - (d1_ref_traj.t(ind_traj_start1) + 2)^2)));
    [~, ind_pose_final2] = min(sqrt(abs(d2_pose.t.^2 - (d2_ref_traj.t(ind_traj_start2) + 2)^2)));
    if n_drones == 3
        [~, ind_pose_final3] = min(sqrt(abs(d3_pose.t.^2 - (d3_ref_traj.t(ind_traj_start3) + 2)^2)));
    end
    
    title_plot = ['Reference traj vs Executed traj from t = ', num2str(vector_time(i)), ' s to t = ', num2str(vector_time(i)+2), ' s'];
    if i == 1
        figure(7); set(gcf, 'Position', get(0, 'Screensize'));
    end
    if i == 1
        subplot(2, 2, 1);
    else
        if i == 2
            subplot(2, 2, 2);
        else
            if i == 3
                subplot(2, 2, 3);
            else
                if i == 4
                    subplot(2, 2, 4);
                end
            end
        end
    end
    
    % Plot reference
    plot(d1_ref_traj.p(ind_traj_start1, 1:end-11, 1), d1_ref_traj.p(ind_traj_start1, 1:end-11, 2), 'y', 'LineWidth', 3.5); grid on; hold on;
    plot(d2_ref_traj.p(ind_traj_start2, 1:end-11, 1), d2_ref_traj.p(ind_traj_start2, 1:end-11, 2), 'k', 'LineWidth', 3.5);
    if n_drones == 3
        plot(d3_ref_traj.p(ind_traj_start3, 1:end-11, 1), d3_ref_traj.p(ind_traj_start3, 1:end-11, 2), 'm', 'LineWidth', 3.5);
    end
    
    % Plot pose (executed traj)
    plot(d1_pose.p(ind_pose_start1:ind_pose_final1, 1), d1_pose.p(ind_pose_start1:ind_pose_final1, 2), 'b', 'LineWidth', 2.5);
    plot(d2_pose.p(ind_pose_start2:ind_pose_final2, 1), d2_pose.p(ind_pose_start2:ind_pose_final2, 2), 'c', 'LineWidth', 2.5);
    if n_drones == 3
        plot(d3_pose.p(ind_pose_start3:ind_pose_final3, 1), d3_pose.p(ind_pose_start3:ind_pose_final3, 2), 'g', 'LineWidth', 2.5);
    end

    hold off;
    
    if i >= 3
        xlabel('X (m)','FontSize',18,'FontWeight','bold'); 
    end
    
    if i == 1 || i == 3
        ylabel('Y (m)','FontSize',18,'FontWeight','bold');
    end
    
    title(title_plot,'FontSize',16,'FontWeight','bold');
    
    if i == 1
        if n_drones == 3
            legend({'Ref1', 'Ref2', 'Ref3', 'Drone 1', 'Drone 2', 'Drone 3'}, 'FontSize', 15, 'Location', 'Best');
        else
            legend({'Ref1', 'Ref2', 'Drone 1', 'Drone 2'}, 'FontSize', 15, 'Location', 'Best');
        end
    end
    axis equal;
    
end


if save_plots
    saveas(gcf, ['~/bagfiles/plots/Experiment_', experiment_date, '_Ref_traj_vs_executed_traj.png']);
else
    pause
end

