clearvars -except robot
clc
close all

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\SimulatedRobot\src')

%% Setup simulated robot, controller, planner, and trajectory generator

% Initialize the robot
if ~exist('robot','var')
    robot = SimulatedRobot();
end
% Set the robot to a non-singularity position
robot.setQ([0.3; 0.3; 0.5; 0.5])

% Initialize the controller
controller = NullspaceController(robot);
 
%% Create a trajectroy
v_average = 50; %[mm/s]
dt = 0.01;
traj_z = 100;

% Initialize the planner
planner = PathPlanner2D(robot, traj_z);
planner.drawPath;
waypoint_list = planner.getPath;

% Initialize the trajectory generator
trajectoryGenerator = TrajectoryGenerator(robot, waypoint_list, v_average,dt);
[x_d, v_d, t] = trajectoryGenerator.getTrajectory;
total_timesteps = ceil(t(end)/dt);

% Plot the desired trajectory
robot.draw(0)
plot3(x_d(1,:),x_d(2,:),x_d(3,:));
scatter3(waypoint_list(1,:),waypoint_list(2,:),waypoint_list(3,:), 30, 'filled', 'm');
figure(robot.fig);

% Plot the workspace
robot.visualizeWorkspace;

%% Control Loop


% Init array for storing tcp positions
tcp_positions = zeros(3,total_timesteps);

%% Loop
loopBeginTime = tic;
step = 1;
while step < total_timesteps

    % Simulation
    q = robot.getQ;
    q_dot = controller.computeDesiredJointVelocity(robot, x_d(:,step),  NaN , v_d(:,step));
    robot.setQ(q + q_dot*dt)

    % Display the robot
    tcp_positions(:,step) = robot.forwardKinematicsNumeric(q);
    plot3(tcp_positions(1,1:step), tcp_positions(2,1:step), tcp_positions(3,1:step), 'k');
    robot.draw(0);
    robot.frames(end).draw;
    drawnow limitrate

    % Wait if simulation is faster than real time
    passedRealTime = toc(loopBeginTime);
    if passedRealTime < t(step)
        pause(t(step)-passedRealTime)
    end
    step = step +1;
end

