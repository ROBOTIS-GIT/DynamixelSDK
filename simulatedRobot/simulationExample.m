% This script shows an example of the roboticArm in the kinematic
% simulation

clear()
clc
close

%% Setup Frames and Joints of the simulated robot (actual distances)
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

% Initialize variables for animation and trajectory plotting
draw_frames = 1;
epsilon = 5;
P_gain = 0.1;
D_gain = 0.01;
I_gain = 0.001;
x_error_integral = zeros(3,1);
x_error_previous = zeros(3,1);
ref_positions_array = [];

% Set the initial non-singularity position (x, y, z) in mm
x_initial = [50; 50; 50];

% Set the desired position (x, y, z) in mm
x_desired = [100; 100; 100];

% Initialize joint angles
joint_angles = zeros(4, 1);

% Animate initial movement to a non-singularity position
while 1
    simulatedRobot = updateRobotPosition(simulatedRobot, x_initial, joint_angles, x_error_integral, x_error_previous, P_gain, D_gain, I_gain);
    
    x_current = simulatedRobot.forwardKinematicsNumeric();
    
    % Display the simulated robot
    simulatedRobot.display(draw_frames);
    drawnow


    % Exit condition: If the end effector is close to the initial position
    if norm(x_initial - x_current) < epsilon
        disp('Reached initial non-singularity position');
        break;
    end
    
    pause(0.1)
end

% Reset integral and previous error for next movement
x_error_integral = zeros(3,1);
x_error_previous = zeros(3,1);

% Animate movement to the desired position
while 1
    simulatedRobot = updateRobotPosition(simulatedRobot, x_initial, joint_angles, x_error_integral, x_error_previous, P_gain, D_gain, I_gain);
    
    % Store the current position for plotting
    ref_positions_array = [ref_positions_array x_current];
    
    % Display the simulated robot and the desired position
    simulatedRobot.display(draw_frames);
    plot3(ref_positions_array(1,:), ref_positions_array(2,:), ref_positions_array(3,:), 'k');
    scatter3(x_desired(1), x_desired(2), x_desired(3), (epsilon^2) * pi, 'g', 'filled');
    drawnow
    
    % Exit condition: If the end effector is close to the desired position
    if norm(x_desired - x_current) < epsilon
        disp('Reached desired position within epsilon');
        break;
    end
    
    pause(0.1)
end

% Function to update robot position based on PID control
function simulatedRobot = updateRobotPosition(simulatedRobot, target_position, joint_angles, x_error_integral, x_error_previous, P_gain, D_gain, I_gain)
    % Compute the current position using forward kinematics
    x_current = simulatedRobot.forwardKinematicsNumeric();
    x_error = target_position - x_current;

    % PID Controller
    x_error_integral = x_error_integral + x_error;
    x_error_derivative = x_error - x_error_previous;
    x_dot = P_gain * x_error + D_gain * x_error_derivative + I_gain * x_error_integral;

    % Compute the Jacobian and joint velocities
    J = simulatedRobot.getJacobianNumeric();
    q_dot = pinv(J) * x_dot;

    % Update joint angles based on joint velocities
    joint_angles = joint_angles + q_dot;

    % Update the simulated robot's joint angles
    for j = 1:4
        
        simulatedRobot.joints(j).setAngle(joint_angles(j));

    end

    % Update the previous error
    x_error_previous = x_error;
end
