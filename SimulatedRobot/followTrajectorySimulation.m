clearvars -except simulatedRobot
clc
close all

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\SimulatedRobot\src')

%% Setup simulated robot, controller, planner, and trajectory generator

% Initialize the simulatedRobot
if ~exist('simulatedRobot','var')
    simulatedRobot = SimulatedRobot();
end
% Set the simulatedRobot to a non-singularity position
simulatedRobot.setQ([0.3; 0.3; 0.5; 0.5])

% Initialize the controller
controller = NullspaceController(simulatedRobot);
 
%% Create a trajectroy
v_average = 50; %[mm/s]
dt = 0.15;
traj_z = 300;

% Initialize the planner
planner = PathPlanner2D(simulatedRobot, traj_z);
planner.drawPath;
waypoint_list = planner.getPath;

% Initialize the trajectory generator
trajectoryGenerator = TrajectoryGenerator(simulatedRobot, waypoint_list, v_average,dt);
[x_d, v_d, t] = trajectoryGenerator.getTrajectory;
total_timesteps = ceil(t(end)/dt);

% Plot the desired trajectory
simulatedRobot.draw(0)
plot3(x_d(1,:),x_d(2,:),x_d(3,:),'m');
scatter3(waypoint_list(1,:),waypoint_list(2,:),waypoint_list(3,:), 30, 'filled', 'm');
figure(simulatedRobot.fig);

% Plot the workspace
simulatedRobot.visualizeWorkspace;

%% Control Loop


% Init array for storing tcp positions
tcp_positions = zeros(3,total_timesteps);

%% Loop
loopBeginTime = tic;
step = 1;
while step < total_timesteps

    % Simulation
    q = simulatedRobot.getQ;
    q_dot = controller.computeDesiredJointVelocity(simulatedRobot, x_d(:,step),  NaN , v_d(:,step));
    simulatedRobot.setQ(q + q_dot*dt)

    % Display the simulatedRobot
    tcp_positions(:,step) = simulatedRobot.forwardKinematicsNumeric(q);
    plot3(tcp_positions(1,1:step), tcp_positions(2,1:step), tcp_positions(3,1:step), 'k');
    simulatedRobot.draw(0);
    simulatedRobot.frames(end).draw;
    drawnow limitrate

    % Wait if simulation is faster than real time
    passedRealTime = toc(loopBeginTime);
    if passedRealTime < t(step)
        pause(t(step)-passedRealTime)
    end
    step = step +1;
end

