clc; clear; close all;

%% Simulation parameters
dt = 0.05;      
T = 22;         
N = T/dt;

%% Initial poses [x, y, theta]
pose_omni = [0; 0; 0];     
pose_2w   = [0; 0; 0];    

%% Trajectories
traj_omni = zeros(N,3);
traj_2w   = zeros(N,3);

%% Speeds
v_omni = 0.5;       
v2 = 0.5;            
omega2_max = 1.0;   % max rotation rate for 2-wheel

%% Setup figure
figure('Position',[100 100 800 400]); 
hold on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); grid on;
xlim([-1 10]); ylim([-3 3]);

% Plot handles
h_omni = plot(nan,nan,'b','LineWidth',2);
h_2w   = plot(nan,nan,'r','LineWidth',2);
traj_plot_omni = plot(nan,nan,'--b','LineWidth',1.5);
traj_plot_2w   = plot(nan,nan,'--r','LineWidth',1.5);

%% Define waypoints for zig-zag (straight-line segments)
waypoints = [0 0;
             3 0;
             3 2;
             6 2;
             6 -1.5;
             9 -1.5;
             9 1;
             10 1]; % [x, y]

segment_index = 1;
current_heading = atan2(waypoints(2,2)-waypoints(1,2), waypoints(2,1)-waypoints(1,1));

%% === Video Recording Setup ===
v = VideoWriter('zigzag_sim.mp4','MPEG-4');  % Output video
v.FrameRate = 60;                            % Frames per second
open(v);

%% Simulation loop
for i = 1:N
    %% Omniwheel robot: straight-line motion along current segment
    dx = waypoints(segment_index+1,1) - pose_omni(1);
    dy = waypoints(segment_index+1,2) - pose_omni(2);
    dist = sqrt(dx^2 + dy^2);
    
    % Check if reached end of current segment
    if dist < v_omni*dt && segment_index < size(waypoints,1)-1
        segment_index = segment_index + 1;
        current_heading = atan2(waypoints(segment_index+1,2)-waypoints(segment_index,2), ...
                                waypoints(segment_index+1,1)-waypoints(segment_index,1));
        dx = waypoints(segment_index+1,1) - pose_omni(1);
        dy = waypoints(segment_index+1,2) - pose_omni(2);
    end
    
    vx_omni = v_omni * cos(current_heading);
    vy_omni = v_omni * sin(current_heading);
    
    % Update omniwheel pose
    pose_omni(1) = pose_omni(1) + vx_omni*dt;
    pose_omni(2) = pose_omni(2) + vy_omni*dt;
    pose_omni(3) = current_heading;
    traj_omni(i,:) = pose_omni';
    
    %% 2-wheel robot: move toward omniwheel (non-holonomic)
    desired_theta = atan2(pose_omni(2) - pose_2w(2), pose_omni(1) - pose_2w(1));
    theta_err = atan2(sin(desired_theta - pose_2w(3)), cos(desired_theta - pose_2w(3)));
    omega2 = max(min(theta_err, omega2_max), -omega2_max);  % limit rotation
    
    % Forward motion along heading
    vx2 = v2 * cos(pose_2w(3));
    vy2 = v2 * sin(pose_2w(3));
    
    % Update 2-wheel pose (real non-holonomic motion constraint)
    pose_2w(1) = pose_2w(1) + vx2*dt;
    pose_2w(2) = pose_2w(2) + vy2*dt;
    pose_2w(3) = pose_2w(3) + omega2*dt;
    traj_2w(i,:) = pose_2w';
    
    %% Update plots
    set(h_omni,'XData',pose_omni(1),'YData',pose_omni(2));
    set(h_2w,'XData',pose_2w(1),'YData',pose_2w(2));
    set(traj_plot_omni,'XData',traj_omni(1:i,1),'YData',traj_omni(1:i,2));
    set(traj_plot_2w,'XData',traj_2w(1:i,1),'YData',traj_2w(1:i,2));
    
    drawnow;

    % === Capture Frame for Video ===
    frame = getframe(gcf);
    writeVideo(v, frame);
end

%% Finalize video
close(v);

title('Omniwheel vs 2-Wheel Balancing: Zig-Zag Sharp Turn Comparison');
legend('Omniwheel','2-Wheel','Omni Traj','2W Traj');
