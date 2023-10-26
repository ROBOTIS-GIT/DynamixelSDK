% This script shows an example of the roboticArm in the kinematic
% simulation. The arm moves to a non singularity position and then tries to
% follow a trajectory.

clear()
clc
close

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\SimulatedRobot')

%% Setup simulated robot
sr = SimulatedRobot();


%% Set the robot to a non-singularity position
sr.setQ([0.3; 0.3; 0.5; 0.5])
sr.draw(0)

%% Create trajectory
R = 120;                 % [mm] Radius of the circle
Z_mean = 400;            % [mm] Mean value of z
Z_amplitude = 50;        % [mm] Amplitude of the sine wave (change based on desired amplitude)
k = 5;                   % Number of waves per full circle
t_total = 10;            % [s] time for whole trajectory
dt = 0.01;               % [s] Time increment between points of the trajectory

% Calculate number of points based on total time and time increment
num_points = round(t_total/dt);

% Calculate the trajectory
theta = linspace(0, 2*pi, num_points);
x_d(1,:) = R * cos(theta);             % x-coordinates
x_d(2,:) = R * sin(theta);             % y-coordinates
x_d(3,:) = Z_mean + Z_amplitude * sin(k * theta);  % z-coordinates

% Calculate the velocity (time derivative of position)
v_d = zeros(size(x_d));

% Using forward differences for all but the last point
for timesteps = 1:num_points-1
    v_d(:,timesteps) = (x_d(:,timesteps+1) - x_d(:,timesteps)) / dt;
end

% Using backward difference for the last point
v_d(:,num_points) = (x_d(:,num_points) - x_d(:,num_points-1)) / dt;

% %Calc max speed in mm/s
% max_speed = 0;
% for timesteps = 1:num_points
%     speed = sqrt(v_d(:,timesteps)'*v_d(:,timesteps));
%     if speed >= max_speed
%         max_speed = speed;
%     end
% end
% fprintf("Max Endeffector Speed in the trajectory is : %.2f km/h\n\n", (max_speed/1000)*3.6);

% Plot the desired trajectory
plot3(x_d(1,:),x_d(2,:),x_d(3,:));

%% Control Loop
% P  gain
Kp = 1;

tcp_positions = zeros(3,num_points);
outerTic = tic;
dt = 0.01;
for timesteps = 1:num_points
    
    % Get current end-effector position
    x_current = SimulatedRobot.forwardKinematicsNumeric(sr.getQ);
    tcp_positions(:,timesteps) = x_current;
    
    % Compute Error
    x_e = x_d(:,timesteps)-x_current;
    fprintf("Position Error: %.f mm\n", norm(x_e));

    % Compute effective workspace velocity
    v_d_eff = x_e*Kp + v_d(:,timesteps);

    % Compute Jacobian
    J = SimulatedRobot.getJacobianNumeric(sr.getQ);

    % Compute pseudo inverse
    pinvJ = pinv(J);
    
    % Compute desired joint velocities
    q_dot = pinvJ*v_d_eff;

    % Check for singularity
    if norm(J)*norm(pinvJ) > 25
        disp('Warning: Close to singularity');
        break
    end

    % Update joint angles based on computed joint velocities
    q = sr.getQ;
    sr.setQ(q + q_dot*dt)

    % Display the robot
    plot3(tcp_positions(1,1:timesteps), tcp_positions(2,1:timesteps), tcp_positions(3,1:timesteps), 'k');
    sr.draw(0);
    sr.frames(end).draw;
    drawnow limitrate

    % Wait if too fast
    if toc(outerTic) < timesteps*dt
        pause(timesteps*dt-toc(outerTic))
    end
    
end
toc(outerTic)