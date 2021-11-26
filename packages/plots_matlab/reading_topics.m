%% READING TOPICS
% Summing up, this script reads the topics and generates new variables that
% can be used for plotting results

% Note: all of them are STRUCTS
% TRAJECTORY TO FOLLOW:
% d1_traj_to_follow(.p(sample, horizon, 3) --> position)
% (.q(sample, horizon, 4) --> orientation) (.t(sample, horizon, 1) --> time
% of sample) 
% d2_traj_to_follow
% d3_traj_to_follow (if n_drones == 3)

% POINTS TO INSPECT:
% points_to_inspect (.p(sample, 3) --> position) (.q(sample, 4) -->
% orientation) (.t(sample,1) --> time of sample)

% REFERENCE TRAJECTORY:
% d1_ref_traj(.p(sample, horizon, 3) --> position)
% (.q(sample, horizon, 4) --> orientation) (.t(sample, horizon, 1) --> time
% of sample) 
% d2_ref_traj
% d3_ref_traj (if n_drones == 3)

% SOLVED TRAJECTORY:
% d1_solved_traj(.p(sample, horizon, 3) --> position)
% (.q(sample, horizon, 4) --> orientation) (.t(sample, horizon, 1) --> time
% of sample) 
% d2_solved_traj
% d3_solved_traj (if n_drones == 3)

% POSE:
% d1_pose (.p(sample, 3) --> position) (.q(sample, 4) -->
% orientation) (.t(sample,1) --> time of sample)
% d2_pose
% d3_pose

% VELOCITY:
% d1_velocity (.l(sample, 3) --> linear) (.a(sample, 4) -->
% angular) (.t(sample,1) --> time of sample)
% d2_velocity
% d3_velocity

%% TRAJECTORY TO FOLLOW
% Selection of topics
topic_select = select(bagfile, 'Topic', topic_traj_to_follow(1));
traj_to_follow1 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(traj_to_follow1);
interpolate = 1;
horizon = size(traj_to_follow1{1,1}.Poses, 2);

for k = start:interpolate:final
    for l=1:horizon
        pose = traj_to_follow1{k,1}.Poses(l).Pose.Position;
        orientation = traj_to_follow1{k,1}.Poses(l).Pose.Orientation;

        % Get time
        t = double(traj_to_follow1{k,1}.Header.Stamp.Sec) + double(traj_to_follow1{k,1}.Header.Stamp.Nsec) * 1e-9;
        d1_traj_to_follow.t(k, :, 1) = t; % May need a fix for plotting

        % Position
        d1_traj_to_follow.p(k, l, 1) = pose.X;
        d1_traj_to_follow.p(k, l, 2) = pose.Y;
        d1_traj_to_follow.p(k, l, 3) = pose.Z;

        % Orientation
        d1_traj_to_follow.q(k, l, 1) = orientation.W;
        d1_traj_to_follow.q(k, l, 2) = orientation.X;
        d1_traj_to_follow.q(k, l, 3) = orientation.Y;
        d1_traj_to_follow.q(k, l, 4) = orientation.Z;
    end
end

% Selection of topics
topic_select = select(bagfile, 'Topic', topic_traj_to_follow(2));
traj_to_follow2 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(traj_to_follow2);
interpolate = 1;
horizon = size(traj_to_follow2{1,1}.Poses, 2);

for k = start:interpolate:final
    for l=1:horizon
        pose = traj_to_follow2{k,1}.Poses(l).Pose.Position;
        orientation = traj_to_follow2{k,1}.Poses(l).Pose.Orientation;

        % Get time
        t = double(traj_to_follow2{k,1}.Header.Stamp.Sec) + double(traj_to_follow2{k,1}.Header.Stamp.Nsec) * 1e-9;
        d2_traj_to_follow.t(k, :, 1) = t; % May need a fix for plotting

        % Position
        d2_traj_to_follow.p(k, l, 1) = pose.X;
        d2_traj_to_follow.p(k, l, 2) = pose.Y;
        d2_traj_to_follow.p(k, l, 3) = pose.Z;

        % Orientation
        d2_traj_to_follow.q(k, l, 1) = orientation.W;
        d2_traj_to_follow.q(k, l, 2) = orientation.X;
        d2_traj_to_follow.q(k, l, 3) = orientation.Y;
        d2_traj_to_follow.q(k, l, 4) = orientation.Z;
    end
end

if n_drones == 3
    % Selection of topics
    topic_select = select(bagfile, 'Topic', topic_traj_to_follow(3));
    traj_to_follow3 = readMessages(topic_select, 'DataFormat', 'struct');
    start = 1;
    final = length(traj_to_follow3);
    interpolate = 1;
    horizon = size(traj_to_follow3{1,1}.Poses, 2);

    for k = start:interpolate:final
        for l=1:horizon
            pose = traj_to_follow3{k,1}.Poses(l).Pose.Position;
            orientation = traj_to_follow3{k,1}.Poses(l).Pose.Orientation;

            % Get time
            t = double(traj_to_follow3{k,1}.Header.Stamp.Sec) + double(traj_to_follow3{k,1}.Header.Stamp.Nsec) * 1e-9;
            d3_traj_to_follow.t(k, :, 1) = t; % May need a fix for plotting

            % Position
            d3_traj_to_follow.p(k, l, 1) = pose.X;
            d3_traj_to_follow.p(k, l, 2) = pose.Y;
            d3_traj_to_follow.p(k, l, 3) = pose.Z;

            % Orientation
            d3_traj_to_follow.q(k, l, 1) = orientation.W;
            d3_traj_to_follow.q(k, l, 2) = orientation.X;
            d3_traj_to_follow.q(k, l, 3) = orientation.Y;
            d3_traj_to_follow.q(k, l, 4) = orientation.Z;
        end
    end
end

%% MISSION STATUS (should be the same for all the topics)
% Selection of topics
topic_select = select(bagfile, 'Topic', topic_mission_status(1));
mission_status_ = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(mission_status_);
interpolate = 1;

for k = start:interpolate:final
    % Get time
    t = double(mission_status_{k,1}.Header.Stamp.Sec) + double(mission_status_{k,1}.Header.Stamp.Nsec) * 1e-9;
    mission_status.t(k, 1) = t;
    
    % Status
    mission_status.d(k,1)  = mission_status_{k,1}.Data;
end


%% POINTS TO INSPECT (should be the same for all the topics)
% Selection of topics
topic_select = select(bagfile, 'Topic', topic_points_to_inspect(1));
points_to_inspect_ = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(points_to_inspect_);
interpolate = 1;

for k = start:interpolate:final
    pose = points_to_inspect_{k,1}.Pose.Position;
    orientation = points_to_inspect_{1,1}.Pose.Orientation;
    
    % Get time
    t = double(points_to_inspect_{k,1}.Header.Stamp.Sec) + double(points_to_inspect_{k,1}.Header.Stamp.Nsec) * 1e-9;
    points_to_inspect.t(k, 1) = t;
    
    % Position
    points_to_inspect.p(k,1) = pose.X;
    points_to_inspect.p(k,2) = pose.Y;
    points_to_inspect.p(k,3) = pose.Z;
    
    % Orientation
    points_to_inspect.q(k,1) = orientation.W;
    points_to_inspect.q(k,2) = orientation.X;
    points_to_inspect.q(k,3) = orientation.Y;
    points_to_inspect.q(k,4) = orientation.Z;
end

%% DISTANCE TO INSPECTION POINT (should be the same for all the topics)
% Selection of topics
topic_select = select(bagfile, 'Topic', topic_distance_to_inspect(1));
distance_inspection_ = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(distance_inspection_);
interpolate = 1;

for k = start:interpolate:final
    distance = distance_inspection_{k,1}.Data;
    
    % Get time
    t = double(distance_inspection_{k,1}.Header.Stamp.Sec) + double(distance_inspection_{k,1}.Header.Stamp.Nsec) * 1e-9;
    distance_inspection.t(k, 1) = t;
    
    % Position
    distance_inspection.d(k, 1) = distance;

end


%% RELATIVE ANGLE (should be the same for all the topics)
% Selection of topics
topic_select = select(bagfile, 'Topic', topic_relative_angle(1));
relative_angle_ = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(relative_angle_);
interpolate = 1;

for k = start:interpolate:final
    distance = relative_angle_{k,1}.Data;
    
    % Get time
    t = double(relative_angle_{k,1}.Header.Stamp.Sec) + double(relative_angle_{k,1}.Header.Stamp.Nsec) * 1e-9;
    relative_angle.t(k, 1) = t;
    
    % Position
    relative_angle.d(k, 1) = distance;

end


%% REFERENCE TRAJECTORY
% Selection of topics
topic_select = select(bagfile, 'Topic', topic_ref_traj(1));
ref_traj1 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(ref_traj1);
interpolate = 1;
horizon = size(ref_traj1{1,1}.Poses, 2);

for k = start:interpolate:final
    for l=1:horizon
        pose = ref_traj1{k,1}.Poses(l).Pose.Position;
        orientation = ref_traj1{k,1}.Poses(l).Pose.Orientation;

        % Get time
        t = double(ref_traj1{k,1}.Header.Stamp.Sec) + double(ref_traj1{k,1}.Header.Stamp.Nsec) * 1e-9;
        d1_ref_traj.t(k, :, 1) = t;

        % Position
        d1_ref_traj.p(k, l, 1) = pose.X;
        d1_ref_traj.p(k, l, 2) = pose.Y;
        d1_ref_traj.p(k, l, 3) = pose.Z;

        % Orientation
        d1_ref_traj.q(k, l, 1) = orientation.W;
        d1_ref_traj.q(k, l, 2) = orientation.X;
        d1_ref_traj.q(k, l, 3) = orientation.Y;
        d1_ref_traj.q(k, l, 4) = orientation.Z;
    end
end

% Selection of topics
topic_select = select(bagfile, 'Topic', topic_ref_traj(2));
ref_traj2 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(ref_traj2);
interpolate = 1;
horizon = size(ref_traj2{1,1}.Poses, 2);

for k = start:interpolate:final
    for l=1:horizon
        pose = ref_traj2{k,1}.Poses(l).Pose.Position;
        orientation = ref_traj2{k,1}.Poses(l).Pose.Orientation;

        % Get time
        t = double(ref_traj2{k,1}.Header.Stamp.Sec) + double(ref_traj2{k,1}.Header.Stamp.Nsec) * 1e-9;
        d2_ref_traj.t(k, :, 1) = t;

        % Position
        d2_ref_traj.p(k, l, 1) = pose.X;
        d2_ref_traj.p(k, l, 2) = pose.Y;
        d2_ref_traj.p(k, l, 3) = pose.Z;

        % Orientation
        d2_ref_traj.q(k, l, 1) = orientation.W;
        d2_ref_traj.q(k, l, 2) = orientation.X;
        d2_ref_traj.q(k, l, 3) = orientation.Y;
        d2_ref_traj.q(k, l, 4) = orientation.Z;
    end
end

if n_drones == 3
    % Selection of topics
    topic_select = select(bagfile, 'Topic', topic_ref_traj(3));
    ref_traj3 = readMessages(topic_select, 'DataFormat', 'struct');
    start = 1;
    final = length(ref_traj3);
    interpolate = 1;
    horizon = size(ref_traj3{1,1}.Poses, 2);
    
    for k = start:interpolate:final
        for l=1:horizon
            pose = ref_traj3{k,1}.Poses(l).Pose.Position;
            orientation = ref_traj3{k,1}.Poses(l).Pose.Orientation;

            % Get time
            t = double(ref_traj3{k,1}.Header.Stamp.Sec) + double(ref_traj3{k,1}.Header.Stamp.Nsec) * 1e-9;
            d3_ref_traj.t(k, :, 1) = t;

            % Position
            d3_ref_traj.p(k, l, 1) = pose.X;
            d3_ref_traj.p(k, l, 2) = pose.Y;
            d3_ref_traj.p(k, l, 3) = pose.Z;

            % Orientation
            d3_ref_traj.q(k, l, 1) = orientation.W;
            d3_ref_traj.q(k, l, 2) = orientation.X;
            d3_ref_traj.q(k, l, 3) = orientation.Y;
            d3_ref_traj.q(k, l, 4) = orientation.Z;
        end
    end
end


%% SOLVED TRAJECTORY
% Selection of topics
topic_select = select(bagfile, 'Topic', topic_solved_traj(1));
solved_traj1 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(solved_traj1);
interpolate = 1;
horizon = size(solved_traj1{1,1}.Poses, 2);

for k = start:interpolate:final
    for l=1:horizon
        pose = solved_traj1{k,1}.Poses(l).Pose.Position;
        orientation = solved_traj1{k,1}.Poses(l).Pose.Orientation;

        % Get time
        t = double(solved_traj1{k,1}.Header.Stamp.Sec) + double(solved_traj1{k,1}.Header.Stamp.Nsec) * 1e-9;
        d1_solved_traj.t(k, :, 1) = t;

        % Position
        d1_solved_traj.p(k, l, 1) = pose.X;
        d1_solved_traj.p(k, l, 2) = pose.Y;
        d1_solved_traj.p(k, l, 3) = pose.Z;

        % Orientation
        d1_solved_traj.q(k, l, 1) = orientation.W;
        d1_solved_traj.q(k, l, 2) = orientation.X;
        d1_solved_traj.q(k, l, 3) = orientation.Y;
        d1_solved_traj.q(k, l, 4) = orientation.Z;
    end
end

% Selection of topics
topic_select = select(bagfile, 'Topic', topic_solved_traj(2));
solved_traj2 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(solved_traj2);
interpolate = 1;
horizon = size(solved_traj2{1,1}.Poses, 2);

for k = start:interpolate:final
    for l=1:horizon
        pose = solved_traj2{k,1}.Poses(l).Pose.Position;
        orientation = solved_traj2{k,1}.Poses(l).Pose.Orientation;

        % Get time
        t = double(solved_traj2{k,1}.Header.Stamp.Sec) + double(solved_traj2{k,1}.Header.Stamp.Nsec) * 1e-9;
        d2_solved_traj.t(k, :, 1) = t;

        % Position
        d2_solved_traj.p(k, l, 1) = pose.X;
        d2_solved_traj.p(k, l, 2) = pose.Y;
        d2_solved_traj.p(k, l, 3) = pose.Z;

        % Orientation
        d2_solved_traj.q(k, l, 1) = orientation.W;
        d2_solved_traj.q(k, l, 2) = orientation.X;
        d2_solved_traj.q(k, l, 3) = orientation.Y;
        d2_solved_traj.q(k, l, 4) = orientation.Z;
    end
end

if n_drones == 3
    % Selection of topics
    topic_select = select(bagfile, 'Topic', topic_solved_traj(3));
    solved_traj3 = readMessages(topic_select, 'DataFormat', 'struct');
    start = 1;
    final = length(solved_traj3);
    interpolate = 1;
    horizon = size(solved_traj3{1,1}.Poses, 2);
    
    for k = start:interpolate:final
        for l=1:horizon
            pose = solved_traj3{k,1}.Poses(l).Pose.Position;
            orientation = solved_traj3{k,1}.Poses(l).Pose.Orientation;

            % Get time
            t = double(solved_traj3{k,1}.Header.Stamp.Sec) + double(solved_traj3{k,1}.Header.Stamp.Nsec) * 1e-9;
            d3_solved_traj.t(k, :, 1) = t;

            % Position
            d3_solved_traj.p(k, l, 1) = pose.X;
            d3_solved_traj.p(k, l, 2) = pose.Y;
            d3_solved_traj.p(k, l, 3) = pose.Z;

            % Orientation
            d3_solved_traj.q(k, l, 1) = orientation.W;
            d3_solved_traj.q(k, l, 2) = orientation.X;
            d3_solved_traj.q(k, l, 3) = orientation.Y;
            d3_solved_traj.q(k, l, 4) = orientation.Z;
        end
    end
end


%% POSE
% Selection of topics
topic_select = select(bagfile, 'Topic', topic_pose(1));
pose1 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(pose1);
interpolate = 1;

for k = start:interpolate:final
    pose = pose1{k,1}.Pose.Position;
    orientation = pose1{k,1}.Pose.Orientation;
    
    % Get time
    t = double(pose1{k,1}.Header.Stamp.Sec) + double(pose1{k,1}.Header.Stamp.Nsec) * 1e-9;
    d1_pose.t(k, 1) = t;
    
    % Position
    d1_pose.p(k,1) = pose.X;
    d1_pose.p(k,2) = pose.Y;
    d1_pose.p(k,3) = pose.Z;
    
    % Orientation
    d1_pose.q(k,1) = orientation.W;
    d1_pose.q(k,2) = orientation.X;
    d1_pose.q(k,3) = orientation.Y;
    d1_pose.q(k,4) = orientation.Z;
end

% Selection of topics
topic_select = select(bagfile, 'Topic', topic_pose(2));
pose2 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(pose2);
interpolate = 1;

for k = start:interpolate:final
    pose = pose2{k,1}.Pose.Position;
    orientation = pose2{k,1}.Pose.Orientation;
    
    % Get time
    t = double(pose2{k,1}.Header.Stamp.Sec) + double(pose2{k,1}.Header.Stamp.Nsec) * 1e-9;
    d2_pose.t(k, 1) = t;
    
    % Position
    d2_pose.p(k,1) = pose.X;
    d2_pose.p(k,2) = pose.Y;
    d2_pose.p(k,3) = pose.Z;
    
    % Orientation
    d2_pose.q(k,1) = orientation.W;
    d2_pose.q(k,2) = orientation.X;
    d2_pose.q(k,3) = orientation.Y;
    d2_pose.q(k,4) = orientation.Z;
end

if n_drones == 3
    % Selection of topics
    topic_select = select(bagfile, 'Topic', topic_pose(3));
    pose3 = readMessages(topic_select, 'DataFormat', 'struct');
    start = 1;
    final = length(pose3);
    interpolate = 1;

    for k = start:interpolate:final
        pose = pose3{k,1}.Pose.Position;
        orientation = pose3{k,1}.Pose.Orientation;

        % Get time
        t = double(pose3{k,1}.Header.Stamp.Sec) + double(pose3{k,1}.Header.Stamp.Nsec) * 1e-9;
        d3_pose.t(k, 1) = t;

        % Position
        d3_pose.p(k,1) = pose.X;
        d3_pose.p(k,2) = pose.Y;
        d3_pose.p(k,3) = pose.Z;

        % Orientation
        d3_pose.q(k,1) = orientation.W;
        d3_pose.q(k,2) = orientation.X;
        d3_pose.q(k,3) = orientation.Y;
        d3_pose.q(k,4) = orientation.Z;
    end
end

%% VELOCITY
% Selection of topics
topic_select = select(bagfile, 'Topic', topic_velocity(1));
velocity1 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(velocity1);
interpolate = 1;

for k = start:interpolate:final
    linear  = velocity1{k,1}.Twist.Linear;
    angular = velocity1{k,1}.Twist.Angular;
    
    % Get time
    t = double(velocity1{k,1}.Header.Stamp.Sec) + double(velocity1{k,1}.Header.Stamp.Nsec) * 1e-9;
    d1_velocity.t(k, 1) = t;
    
    % Position
    d1_velocity.l(k,1) = linear.X;
    d1_velocity.l(k,2) = linear.Y;
    d1_velocity.l(k,3) = linear.Z;
    
    % Orientation
    d1_velocity.a(k,1) = angular.X;
    d1_velocity.a(k,2) = angular.Y;
    d1_velocity.a(k,3) = angular.Z;
end

% Selection of topics
topic_select = select(bagfile, 'Topic', topic_velocity(2));
velocity2 = readMessages(topic_select, 'DataFormat', 'struct');
start = 1;
final = length(velocity2);
interpolate = 1;

for k = start:interpolate:final
    linear  = velocity2{k,1}.Twist.Linear;
    angular = velocity2{k,1}.Twist.Angular;
    
    % Get time
    t = double(velocity2{k,1}.Header.Stamp.Sec) + double(velocity2{k,1}.Header.Stamp.Nsec) * 1e-9;
    d2_velocity.t(k, 1) = t;
    
    % Position
    d2_velocity.l(k,1) = linear.X;
    d2_velocity.l(k,2) = linear.Y;
    d2_velocity.l(k,3) = linear.Z;
    
    % Orientation
    d2_velocity.a(k,1) = angular.X;
    d2_velocity.a(k,2) = angular.Y;
    d2_velocity.a(k,3) = angular.Z;
end

if n_drones == 3

    % Selection of topics
    topic_select = select(bagfile, 'Topic', topic_velocity(3));
    velocity3 = readMessages(topic_select, 'DataFormat', 'struct');
    start = 1;
    final = length(velocity3);
    interpolate = 1;

    for k = start:interpolate:final
        linear  = velocity3{k,1}.Twist.Linear;
        angular = velocity3{k,1}.Twist.Angular;

        % Get time
        t = double(velocity3{k,1}.Header.Stamp.Sec) + double(velocity3{k,1}.Header.Stamp.Nsec) * 1e-9;
        d3_velocity.t(k, 1) = t;

        % Position
        d3_velocity.l(k,1) = linear.X;
        d3_velocity.l(k,2) = linear.Y;
        d3_velocity.l(k,3) = linear.Z;

        % Orientation
        d3_velocity.a(k,1) = angular.X;
        d3_velocity.a(k,2) = angular.Y;
        d3_velocity.a(k,3) = angular.Z;
    end
end