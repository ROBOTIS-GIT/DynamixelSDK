% This script sets the robot to a starting position relative to its initial
% (zero) position. Then it trys to set a velocity to the endeffector
% defined by the keyboard by calculating the Jacobian.

clear
clc
close

addpath('C:\Users\samue\Documents\Git\DynamixelSDK\realRobot')
addpath('C:\Users\samue\Documents\Git\DynamixelSDK\simulatedRobot')

%% Setup simulated robot
simulatedRobot = SimulatedRobot();

%% Connect real robot
realRobot = RealRobot();


%% Main
% Initial position setup
realRobot.torqueEnableDisable(0);
realRobot.setOperatingMode('velocity');
realRobot.setZeroPositionToCurrentPosition;
realRobot.torqueEnableDisable(1);
realRobot.setJointVelocities(0.2,0.2,-0.4,-0.6);
pause(1)
realRobot.setJointVelocities(0,0,0,0);


trajectory = [];

% Create a new figure and set the KeyPressFcn
fig = figure('KeyPressFcn', @(fig_obj, eventDat) setappdata(fig_obj, 'key', eventDat.Key), ...
             'KeyReleaseFcn', @(fig_obj, eventDat) setappdata(fig_obj, 'key', ''));

% Create a velocity variable
v = 50; % [mm/s]
draw_frames = 0;

figure(fig);
pause(0.5);  % wait for 0.5 seconds for the window to gain focus

% Loop while the 'c' key has not been pressed
while ~strcmp(getappdata(fig, 'key'), 'c')

    key = getappdata(fig, 'key');
    % Update the velocity according to the key pressed
    if ~isempty(key)
        switch key
            case 'leftarrow'
                x_dot = [-v; 0; 0];
            case 'rightarrow'
                x_dot = [v; 0; 0];
            case 'uparrow'
                x_dot = [0; v; 0];
            case 'downarrow'
                x_dot = [0; -v; 0];
            case 'w'
                x_dot = [0; 0; v];
            case 's'
                x_dot = [0; 0; -v];
        end
    else
        x_dot = [0; 0; 0];
    end

    % Update joint angles for the simulated robot
    currentJointAngles = realRobot.getJointAngles();
    for i = 1:4
        simulatedRobot.joints(i).setAngle(currentJointAngles(i));
    end

    % Calculate the Jacobian in the current (modeled robot = real robot) configuration numerically
    J = simulatedRobot.getJacobianNumeric;

    % Check for singularity and elevation limit
    if cond(pinv(J)) > 15
        disp('Warning: Close to singularity');
        realRobot.goToZeroPosition();
        break
    end
    if rad2deg(simulatedRobot.getShoulderElevation) < 45
        disp('Warning: Shoulder joint minimum elevation limit reached')
        realRobot.goToZeroPosition();
        break;
    end

    % Calculate respective joint velocity
    q_dot = pinv(J) * x_dot;

    % Set the joint velocity to the real robot
    realRobot.setJointVelocities(q_dot);

    % Visualize the simulated robot which
    % should be a mirror image of the real robot
    simulatedRobot.display(draw_frames);
    drawnow

    % Get endeffector info for the trajectory
    display_info = 0;
    x_current = simulatedRobot.forwardKinematics();

    % Append the endeffector postion to an array and plot the trajectory
    trajectory = [trajectory x_current];
    plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'k');

end
% Return to initial zero positoin
realRobot.goToZeroPosition();

