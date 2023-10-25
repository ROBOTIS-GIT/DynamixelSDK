%This script shows an example of the roboticArm in the kinematic
% simulation. The arm moves to a non singularity position and then tries to
% reach a desired endeffector position. It shall use the Nullspace operator
% to perform additional task (orientation of the endeffector, better
% configurations, avoid singularities, etc...)

clear()
clc
close

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\simulatedRobot')

%% Setup simulated robot
simulatedRobot = SimulatedRobot();

%% Move the robot to a non-singularity initial position
simulatedRobot.moveInitPos(0);
simulatedRobot.draw(0)


%% Use inverse jacobian to reach a goal position
% Desired position
x_desired =  [-300, -300, 300]';

% Plot the desired goal position
epsilon = 5;
scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'm', 'filled');


%% Use nullspace operator to manipulate other stuff
max_timesteps = 10000;
tcp_positions = zeros(3,max_timesteps);
outerTic = tic;
dt = 0.003;

for timesteps = 1:max_timesteps
    % Get current end-effector position
    x_current = simulatedRobot.forwardKinematicsNumeric;
    tcp_positions(:,timesteps) = x_current;

    % Compute error
    error = x_desired - x_current;
    
    % Compute control input (PID)
    Kp = 1;
    u = Kp * error;
    
    % Compute the Jacobian for the current robot configuration
    J = simulatedRobot.getJacobianNumeric();    
    
    % Compute pseudo inverse
    pinvJ = pinv(J);

    % Compute Nullspace Operator
    % DO SOME STUFF HERE
    q_dot = pinvJ * u;
    
    % Update joint angles based on computed joint velocities
    q = simulatedRobot.getQ;
    simulatedRobot.setQ(q + q_dot*dt)

    % Display the robot
    % plot3(tcp_positions(1,1:timesteps), tcp_positions(2,1:timesteps), tcp_positions(3,1:timesteps), 'k');
    simulatedRobot.draw(0);
    simulatedRobot.frames(end).draw;
    drawnow limitrate

    % Wait if too fast
    if toc(outerTic) < timesteps*dt
        pause(timesteps*dt-toc(outerTic))
    end
    
end

