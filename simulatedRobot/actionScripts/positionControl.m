% This script shows an example of the roboticArm in the kinematic
% simulation. The arm moves to a non singularity position and then tries to
% reach a desired endeffector position given in global coordinates. It uses
% inverse kinematics together with a PID controller.

clear()
clc
close

% Get the full path of the current script
currentScriptFullPath = mfilename('fullpath');
% Get the parent folder
[parentFolder, ~, ~] = fileparts(fileparts(currentScriptFullPath));
% Add the parent folder to the MATLAB path
addpath(parentFolder);


%% Setup Frames and Joints of the simulated robot
orig_frame = CustomFrame([0; 0; 0], [], 'Origin');
joint1 = CustomJoint([0; 0; 83.51], orig_frame, 'Joint 1', 'y');
joint2 = CustomJoint([0;0;0], joint1, 'Joint 2', 'x');
joint3 = CustomJoint([0;0;119.35], joint2, 'Joint 3', 'z');
joint4 = CustomJoint([0;0;163.99], joint3, 'Joint 4', 'x');
endeffector_frame = CustomFrame([0;0;218.86], joint4, 'Endeffector');

%% Setup Links of the simulated robot
link1 = CustomLink(orig_frame, joint1, 'r');  % Red
link2 = CustomLink(joint1, joint2, 'g');  % Green
link3 = CustomLink(joint2, joint3, 'b');  % Blue
link4 = CustomLink(joint3, joint4, 'y');  % Yellow
link5 = CustomLink(joint4, endeffector_frame, 'm');  % Magenta


%% Setup simulated robot
simulatedRobot = SimulatedRobot([joint1, joint2, joint3, joint4], [link1, link2, link3, link4, link5], [orig_frame, endeffector_frame]);


%% Initialize variables for visualization
ref_positions_array = [];  % Array to store end-effector positions
epsilon = 5;  % Radius for the scatter plot of the goal position

% Desired position
x_desired =  [-400, -400, 300]';

%% Move the robot to a non-singularity position
disp("Moving the robot to a non-singularity position.")
for i = 1:50

    simulatedRobot.joints(1).rotate(0.006);
    simulatedRobot.joints(2).rotate(0.006);
    simulatedRobot.joints(3).rotate(0.01);
    simulatedRobot.joints(4).rotate(0.01);

    
    
    % Get current end-effector position for trajectory visualization
    x_current = simulatedRobot.forwardKinematicsNumeric;
    ref_positions_array = [ref_positions_array, x_current];
    
    % Display the robot
    simulatedRobot.display(0);
    
    % Plot the trajectory of the end-effector
    plot3(ref_positions_array(1,:), ref_positions_array(2,:), ref_positions_array(3,:), 'k');
    
    % Plot the desired goal position
    scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');

    
    drawnow;
end
disp("Starting control loop.")
pause(1)



%% Use inverse kinematics to reach a goal position (Endeffector Position Control with PID)
% PID gains
Kp = 1;
Ki = 0.01;
Kd = 0.1;

% Initialize error and integral terms
error_integral = zeros(3,1);
error_prev = zeros(3,1);


% Initialize variables to store total time for each method
totalTime_numeric = 0;
totalTime_symbolic = 0;
numIterations = 0;
J_sym = simulatedRobot.getJacobianSymbolic();

dt = 0.01;
while 1
    numIterations = numIterations + 1;
    % Get current end-effector position
    x_current = simulatedRobot.forwardKinematicsNumeric;


    % Compute error
    error = x_desired - x_current;
    
    % Compute integral and derivative of error
    error_integral = error_integral + error;
    error_derivative = error - error_prev;
    
    % Compute control input (PID)
    u = Kp * error + Ki * error_integral + Kd * error_derivative;
    
    % Compute the Jacobian for the current robot configuration
    tic; % Start timer
    J = simulatedRobot.getJacobianNumeric();
    elapsedTime_numeric = toc; % Stop timer and get elapsed time
    totalTime_numeric = totalTime_numeric + elapsedTime_numeric; % Accumulate total time
    

    % % Slower alternative: Compute Jacobian by substituting in the symbolic
    % % expression
    % tic; % Start timer
    % sAlpha = simulatedRobot.joints(1).angle;
    % sBeta = simulatedRobot.joints(2).angle;
    % sGamma = simulatedRobot.joints(3).angle;
    % sDelta = simulatedRobot.joints(4).angle;
    % J_from_symbolic_by_subs = subs(J_sym);
    % elapsedTime_symbolic = toc; % Stop timer and get elapsed time
    % totalTime_symbolic = totalTime_symbolic + elapsedTime_symbolic; % Accumulate total time

    
    % Compute joint velocities
    pinvJ = pinv(J);
    q_dot = pinvJ * u;

    % Check for singularity
    if norm(J)*norm(pinvJ) > 25
        disp('Warning: Close to singularity');
        break
    end
    
    % Update joint angles based on computed joint velocities
    for j = 1:4
        angle = simulatedRobot.joints(j).angle;
        simulatedRobot.joints(j).setAngle(angle + q_dot(j)*dt);
    end
    
    % Update previous error
    error_prev = error;
    
    % Store the current end-effector position for trajectory visualization
    ref_positions_array = [ref_positions_array, x_current];
    
    % Print the distance to the goal
    distance_to_goal = norm(error);
    fprintf('Distance to goal: %.0f mm \n', distance_to_goal);
    
    % Display the robot
    simulatedRobot.display(0);
    
    % Plot the trajectory of the end-effector
    plot3(ref_positions_array(1,:), ref_positions_array(2,:), ref_positions_array(3,:), 'k');
    
    % Plot the desired goal position
    scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');
    
    drawnow;
    
    % Break condition: stop if error is small
    if norm(error) < 5 %mm
        break;
    end

    pause(dt)
end

% % Calculate average time for each method
% averageTime_numeric = totalTime_numeric / numIterations;
% averageTime_symbolic = totalTime_symbolic / numIterations;
% 
% % Calculate how much more time-intensive the symbolic variant is in percent
% percentage_increase = ((averageTime_symbolic - averageTime_numeric) / averageTime_numeric) * 100;
% 
% % Display the average times
% fprintf('Average time for numeric method: %f seconds\n', averageTime_numeric);
% fprintf('Average time for symbolic method: %f seconds\n', averageTime_symbolic);
% fprintf('The symbolic method is %f%% more time-intensive than the numeric method.\n', percentage_increase);
% 

disp('Reached the goal position.');