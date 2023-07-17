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
realRobot.setJointVelocity(1,2);
realRobot.setJointVelocity(2,2);
realRobot.setJointVelocity(3,-4);
realRobot.setJointVelocity(4,6);
pause(1)
realRobot.setJointVelocity(1,0);
realRobot.setJointVelocity(2,0);
realRobot.setJointVelocity(3,0);
realRobot.setJointVelocity(4,0);

% Desired position
x_desired =  [-315.301974, -83.883586, 189.013096]';
ref_positions_array = [];

% PID gains
P_gain = 10;
D_gain = 4; 
I_gain = 0; 

% Initialize error parameters
x_error_integral = zeros(3,1); 
x_error_previous = zeros(3,1); 
x_error_derivative = zeros(3,1); 

% Initialize joint angles for the simulated robot
for i = 1:4
    simulatedRobot.joints(i).setAngle(realRobot.getJointAngle(i));
end

% Display parameters
epsilon = 5; %mm
clearFig = 0;
draw_frames = 0;

% Display the simulated robot
simulatedRobot.display(clearFig, draw_frames);
scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');

% pause(2)

last_position_change = inf;
reached_positions_counter = 0;

% Initialize extra variables
distance_to_goal_previous = inf;
distance_not_changing_counter = 0;
I_gain_enabled = false;






% Number of points
n_points = 10;
% Radius of the circle in mm
radius = 100;  
% Center of the circle
center = [-radius, -radius, 500];

% Create an array of angular positions
theta = linspace(0, 2*pi, n_points);

% Use the parametric equations of a circle
x = center(1) + radius * cos(theta);
y = center(2) + radius * sin(theta);
z = center(3) * ones(size(theta));

% Create the positions array
positions_array = [x; y; z];



for i = 1:size(positions_array, 2)

    x_desired = positions_array(:, i);

    % Reset all the needed variables before trying a new position
    ref_positions_array = [];
    x_error_integral = zeros(3,1); 
    x_error_previous = zeros(3,1); 
    x_error_derivative = zeros(3,1); 
    last_position_change = inf;
    reached_positions_counter = 0;
    distance_to_goal_previous = inf;
    distance_not_changing_counter = 0;
    I_gain_enabled = false;

    while 1
        % Update joint angles for the simulated robot
        for j = 1:4
            simulatedRobot.joints(j).setAngle(realRobot.getJointAngle(j));
        end
    
        % Compute the Jacobian of the simulated robot
        J = simulatedRobot.getJacobianNumeric;
    
        % Check for singularity condition and limit reached condition
        if cond(pinv(J)) > 15
            disp('Warning: Close to singularity');
            % realRobot.goToZeroPosition(0.5);
            break
        end
        if rad2deg(realRobot.getBevelElevation) < 45
            disp('Warning: Bevel elevation limit reached')
            % realRobot.goToZeroPosition(0.5);
            break;
        end
    
        % Compute the current position
        display_info = 0;
        [ref_position, ~, ~] = endeffector_frame.getInfo(display_info);
        x_current = ref_position;
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
                for i = 1:4
                    realRobot.setJointVelocity(i,0);
                end
                % pause(2)
                % realRobot.goToZeroPosition(0.5);
                break;
            end
        end
    
        % Compute the PID control law
        x_error_integral = x_error_integral + x_error; 
        x_error_derivative = x_error - x_error_previous; 
        x_dot = P_gain*x_error + D_gain*x_error_derivative + I_gain*x_error_integral;
    
        % Update the error
        x_error_previous = x_error; 
    
        % Cap the speed at a maximum value.
        max_speed = 700;  % Set the maximum speed as per your requirements.
        if norm(x_dot) > max_speed
            x_dot = max_speed * (x_dot / norm(x_dot));
        end
    
        % Compute the joint velocities
        q_dot = pinv(J) * x_dot;
    
    
        % Set the joint velocities for the real robot
        for j = 1:4
            realRobot.setJointVelocity(j,q_dot(j));
        end
    
        % Store the current position
        ref_positions_array = [ref_positions_array ref_position];
    
        % Display the simulated robot and the desired position
        simulatedRobot.display(clearFig, draw_frames);
        plot3(ref_positions_array(1,:),ref_positions_array(2,:),ref_positions_array(3,:),'k');
        scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');
        drawnow
    end
end

realRobot.goToZeroPosition;
