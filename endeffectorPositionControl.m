% Clear variables and command window
clear
clc

% Add paths to the robot models
addpath('C:\Users\samue\Documents\Git\DynamixelSDK\realRobot')
addpath('C:\Users\samue\Documents\Git\DynamixelSDK\simulatedRobot')

%% Setup Frames and Joints
% Create the frames and joints for the robot model
orig_frame = CustomFrame([0; 0; 0], [], 'Origin');
joint1 = Joint([0; 0; 83.51], orig_frame, 'Joint 1', 'y');
joint2 = Joint([0;0;0], joint1, 'Joint 2', 'x');
joint3 = Joint([0;0;119.35], joint2, 'Joint 3', 'z');
joint4 = Joint([0;0;163.99], joint3, 'Joint 4', 'x');
endeffector_frame = CustomFrame([0;0;218.86], joint4, 'Endeffector');

%% Setup Links
% Create the links for the robot model
link1 = CustomLink(orig_frame, joint1, 'r');  % Red
link2 = CustomLink(joint1, joint2, 'g');  % Green
link3 = CustomLink(joint2, joint3, 'b');  % Blue
link4 = CustomLink(joint3, joint4, 'y');  % Yellow
link5 = CustomLink(joint4, endeffector_frame, 'm');  % Magenta

%% Setup Robot
% Initialize both the simulated and real robot
simulatedRobot = SimulatedRobot([joint1, joint2, joint3, joint4], [link1, link2, link3, link4, link5], [orig_frame, endeffector_frame]);
realRobot = RealRobot();

%% Main
% Initial position setup
realRobot.setZeroPositionToCurrentPosition;
realRobot.robotTorqueEnableDisable(1);
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
x_desired = [-319.311633, -8.206300, 172.201658]';
ref_positions_array = [];

% PID gains
P_gain = 15;
D_gain = 5; 
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
epsilon = 3; %mm
clearFig = 0;
draw_frames = 0;

% Display the simulated robot
simulatedRobot.display(clearFig, draw_frames);
scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');

pause(2)

last_position_change = inf;
reached_positions_counter = 0;
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
        realRobot.goToZeroPosition(0);
        break
    end
    if rad2deg(realRobot.getBevelElevation) < 50
        disp('Warning: Bevel elevation limit reached')
        realRobot.goToZeroPosition(0);
        break;
    end

    % Compute the current position
    display_info = 0;
    [ref_position, ~, ~] = endeffector_frame.getInfo(display_info);
    x_current = ref_position;
    x_error = x_desired - x_current;

    % Print the distance to the goal
    fprintf('Distance to goal: %.0f mm \n', norm(x_error));

    % Check if the desired position is reached
    if norm(x_error) < epsilon
        reached_positions_counter = reached_positions_counter + 1;
        if reached_positions_counter > 10
            disp('Reached position within epsilon');
            for i = 1:4
                realRobot.setJointVelocity(i,0);
            end
            pause(2)
            realRobot.goToZeroPosition(0);
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
