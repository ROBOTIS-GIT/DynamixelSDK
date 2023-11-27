clearvars -except simulatedRobot
clc
close all

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\RealRobot\src')
addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\SimulatedRobot\src')

%% Setup simulated robot, controller, planner, and trajectory generator

% Initialize the robot
if ~exist('simulatedRobot','var')
    simulatedRobot = SimulatedRobot();
end

% Initialize the controller
controller = NullspaceController(simulatedRobot);

%% Connect real robot
realRobot = RealRobot();

% Initial position setup for Real robot
realRobot.torqueEnableDisable(0);
realRobot.setOperatingMode('velocity');
realRobot.setZeroPositionToCurrentPosition;
realRobot.torqueEnableDisable(1);
realRobot.setJointVelocities([0.02,0.02,0.1,0.1]);
pause(1)
realRobot.setJointVelocities([0,0,0,0]);

% Set simulated Robot to same config as real robot
simulatedRobot.setQ(realRobot.getQ)
 
%% Create a trajectroy
v_average = 50; %[mm/s]
dt = 0.15; % Robot cant receive and send information in a shorter time than 0.15 seconds 
traj_z = 400;

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
trajectoryStartTime = tic; % Start timing the trajectory
step = 1;
while step < total_timesteps

    % Update the simulated Robot
    q = realRobot.getQ;
    simulatedRobot.setQ(q);

    % Compute q_dot with controller
    q_dot = controller.computeDesiredJointVelocity(simulatedRobot, x_d(:,step),  NaN , v_d(:,step));

    % Set q_dot to real Robot
    realRobot.setJointVelocities(q_dot);

    % Display the robot
    tcp_positions(:,step) = simulatedRobot.forwardKinematicsNumeric(q);
    plot3(tcp_positions(1,1:step), tcp_positions(2,1:step), tcp_positions(3,1:step), 'k');
    simulatedRobot.draw(0);
    simulatedRobot.frames(end).draw;
    drawnow limitrate


    step = step +1;
end
realRobot.setJointVelocities([0;0;0;0]);
