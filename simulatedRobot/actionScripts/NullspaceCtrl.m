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

dt = 0.01;
while 1
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
    for j = 1:4
        simulatedRobot.joints(j).rotate(q_dot(j)*dt);
    end
    
    % Print the distance to the goal
    distance_to_goal = norm(error);
    fprintf('Distance to goal: %.0f mm \n', distance_to_goal);
    
    % Display the robot
    simulatedRobot.display(0);
    drawnow
    
    % Break condition: stop if error is small
    if norm(error) < 5 %mm
        disp('Reached the goal position.');
        break;
    end

end

%% Use nullspace operator to manipulate other stuff
dt = 0.01;
disp("Nullspace Ctrl:")
while 1
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
    for j = 1:4
        simulatedRobot.joints(j).rotate(q_dot(j)*dt);
    end

    % Calculate the Endeffector Rotation matrix from Basis Rotations
    q = [simulatedRobot.joints(1).angle; simulatedRobot.joints(2).angle; simulatedRobot.joints(3).angle; simulatedRobot.joints(4).angle];
    

    g_A_E_calc = roty(q(1))*rotx(q(2))*rotz(q(3))*rotx(q(4));
    endeffectorFrame = simulatedRobot.frames(end);
    g_A_E_frame = endeffectorFrame.rotation;

    if ~(g_A_E_frame == g_A_E_frame)
        disp("Error")
    end
    
    % Display the robot and the endeffector frame
    simulatedRobot.display(1);
    % endeffectorFrame.display();
    drawnow
    

end

function rotx = rotx(alpha)
    rotx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
end

function roty = roty(beta)
    roty = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
end

function rotz = rotz(gamma)
    rotz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
end
