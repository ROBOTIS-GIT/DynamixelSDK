% This script moves the robot to a starting position and then tries to
% reach a goal position with the endeffector using a PID controller.

clear
clc
close

addpath('C:\Users\samue\Documents\Git\DynamixelSDK\realRobot')
addpath('C:\Users\samue\Documents\Git\DynamixelSDK\simulatedRobot')

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


%% Params

% Desired position
x_desired =  [-315.301974, -83.883586, 189.013096]';
trajectory = [];

% PID gains
P_gain = 10;
D_gain = 4; 
I_gain = 0; 

% Other
epsilon = 5; %mm
draw_frames = 0;
max_speed = 70;  % [mm/s] Set the maximum endeffector speed as per your requirements.
%%

% Initialize variables
x_error_integral = zeros(3,1); 
x_error_previous = zeros(3,1); 
x_error_derivative = zeros(3,1); 
reached_positions_counter = 0;
distance_to_goal_previous = inf;
distance_not_changing_counter = 0;
I_gain_enabled = false;

while 1
    % Update joint angles for the simulated robot
    currentJointAngles = realRobot.getJointAngles();
    for i = 1:4
        simulatedRobot.joints(i).setAngle(currentJointAngles(i));
    end

    % Compute the Jacobian of the simulated robot
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

    % Compute the current position
    display_info = 0;
    x_current = simulatedRobot.forwardKinematicsNumeric();
    x_error = x_desired - x_current;

    % Print the distance to the goal
    distance_to_goal = norm(x_error);
    fprintf('Distance to goal: %.0f mm \n', distance_to_goal);

    % Check if the distance to the goal is not changing much
    if abs(distance_to_goal - distance_to_goal_previous) < 2
        distance_not_changing_counter = distance_not_changing_counter + 1;
    else
        distance_not_changing_counter = 0; % Reset the counter if the change is more than 1 mm
    end
    distance_to_goal_previous = distance_to_goal;

    % If the distance is not changing for more than 3 timesteps, enable the I_gain
    if distance_not_changing_counter > 3 && ~I_gain_enabled
        I_gain = 1;
        x_error_integral = zeros(3,1); % clear x_error_integral
        I_gain_enabled = true; % set the flag to avoid multiple activations
        disp('I-Gain has been enabled');
    end

    % Check if the desired position is reached
    if distance_to_goal < epsilon
        reached_positions_counter = reached_positions_counter + 1;
        if reached_positions_counter > 5
            disp('Reached position within epsilon');
            realRobot.setJointVelocities([0,0,0,0]);
            pause(2)
            realRobot.goToZeroPosition();
            break;
        end
    end

    % Compute the PID control law
    x_error_integral = x_error_integral + x_error; 
    x_error_derivative = x_error - x_error_previous; 
    x_dot = P_gain*x_error + D_gain*x_error_derivative + I_gain*x_error_integral;

    % Update the error
    x_error_previous = x_error; 

    % Cap the speed endeffector speed a maximum value.
    if norm(x_dot) > max_speed
        x_dot = max_speed * (x_dot / norm(x_dot));
    end

    % Compute the joint velocities
    q_dot = pinv(J) * x_dot;

    % Set the joint velocities for the real robot
    realRobot.setJointVelocities(q_dot);

    % Store the current position at the end of the trajectory
    trajectory = [trajectory x_current];

    % Display the simulated robot and the desired position
    simulatedRobot.display(draw_frames);
    plot3(trajectory(1,:),trajectory(2,:),trajectory(3,:),'k');
    scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');
    drawnow
end
