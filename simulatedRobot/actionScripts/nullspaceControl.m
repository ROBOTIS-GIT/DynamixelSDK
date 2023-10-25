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
simulatedRobot.display(0)


%% Use inverse jacobian to reach a goal position
% Desired position
x_desired =  [-300, -300, 300]';

% Plot the desired goal position
epsilon = 5;
scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');

outerTic = tic;
dt = 0.01;
timesteps = 0;
while 1
    timesteps = timesteps +1;
    % Get current end-effector position
    x_current = simulatedRobot.forwardKinematicsNumeric;

    % Compute error
    error = x_desired - x_current;
    
    % Compute control input (PID)
    Kp = 1;
    u = Kp * error;
    
    % Compute the Jacobian for the current robot configuration
    J = simulatedRobot.getJacobianNumeric();    
    
    % Compute pseudo inverse
    pinvJ = pinv(J);
    q_dot = pinvJ * u;
    
    % Update joint angles based on computed joint velocities
    q = simulatedRobot.getQ;
    simulatedRobot.setQ(q + q_dot*dt)
    
    % Print the distance to the goal
    distance_to_goal = norm(error);
    fprintf('Distance to goal: %.0f mm \n', distance_to_goal);
    
    % Display the robot
    simulatedRobot.display(0);
    drawnow limitrate
    
    % Break condition: stop if error is small
    if norm(error) < 5 %mm
        disp('Reached the goal position.');
        break;
    end

    % Wait if too fast
    if toc(outerTic) < timesteps*dt
        pause(timesteps*dt-toc(outerTic))
    end


end

%% Use nullspace operator to manipulate other stuff
disp("Nullspace Control:")
outerTic = tic;
dt = 0.01;
timesteps = 0;
while 1
    timesteps = timesteps +1;
    % Get current end-effector position
    x_current = simulatedRobot.forwardKinematicsNumeric;

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
    % Update joint angles based on computed joint velocities
    q = simulatedRobot.getQ;
    simulatedRobot.setQ(q + q_dot*dt)

    % Display the robot and the endeffector frame
    simulatedRobot.display(1);
    drawnow limitrate

    % Wait if too fast
    if toc(outerTic) < timesteps*dt
        pause(timesteps*dt-toc(outerTic))
    end
    
end

