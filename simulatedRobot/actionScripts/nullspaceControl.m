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
dt = 0.01;

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

    %% Nullspace Operator
    % Possible additional optimization goals:
    % 1. Penalize high joint angles (maybe quadratic, towards the max more, specific joints more)
    % 2. Desire a specific z-Axis of the TCP
    % 3. Penalize low bevel gear elevations

    % Calculate Nullspace Operator
    N = (eye(4) - pinvJ * J);
    q = simulatedRobot.getQ;

    % 1. 
    % alpha = 0.1;
    % q_dot = pinvJ * u - alpha*N*q;


    % % 2 
    alpha = 10;
    z_desired = [sin(timesteps/100); -sin(timesteps/100); cos(timesteps/100)];
    z_desired = z_desired/norm(z_desired);


    % Small change in joint angles
    delta_q = 1e-6;

    % Initialize dHdQ
    dHdQ = [0, 0, 0, 0];

    % For each joint angle
    for i = 1:4
        % Add small change to joint i
        q_plus = q;
        q_plus(i) = q_plus(i) + delta_q;

        % Subtract small change from joint i
        q_minus = q;
        q_minus(i) = q_minus(i) - delta_q;

        % Compute H(q) = 1/2 * (z(q) - z_desired) ^2
        H_plus = computeH(q_plus,z_desired);
        H_minus = computeH(q_minus,z_desired);

        % Compute derivative
        dHdQ(i) = (H_plus - H_minus) / (2 * delta_q);
    end

    q_dot = pinvJ * u - alpha*N*dHdQ';






    % % 3. maximize bevel elevation --> straight line start to finish
    % % alpha = 1; % use -1 to minimize elevation
    % % dHdQ = [(cos(q(2))*sin(q(1)))/sqrt(1-(cos(q(2))*cos(q(1)))^2), (cos(q(1))*sin(q(2)))/sqrt(1-(cos(q(2))*cos(q(1)))^2), 0, 0];
    % % q_dot = pinvJ * u - alpha*N*dHdQ';
    % 


    % do nothing with the nullspace --> straight line start to finish
    % q_dot = pinvJ * u;



    %%
   
    % Update joint angles based on computed joint velocities
    simulatedRobot.setQ(q + q_dot*dt)

    % Display the robot
    % plot3(tcp_positions(1,1:timesteps), tcp_positions(2,1:timesteps), tcp_positions(3,1:timesteps), 'k');
    simulatedRobot.draw(0);
    % simulatedRobot.frames(end).draw;
    drawnow limitrate

    % Print the distance to the goal
    % distance_to_goal = norm(error);
    % fprintf('Distance to goal: %.0f mm \n', distance_to_goal);

    % Print bevel elevation
    % elevation = simulatedRobot.getShoulderElevation;
    % fprintf('Shoulder Elevation: %.2f ° \n', elevation*180/pi);

    % Print H(q)
    fprintf('H(q) = %.2f \n', computeH(q,z_desired))

    % Wait if too fast
    if toc(outerTic) < timesteps*dt
        pause(timesteps*dt-toc(outerTic))
    end
    
end

% Compute H(q) = 1/2 * (z(q) - z_desired) ^2
function H = computeH(q,z_desired)
    
    g_A_tcp = roty(q(1)) * rotx(q(2)) * rotz(q(3)) * rotx(q(4));
    z_q = g_A_tcp * [0;0;1];
    H = 1/2 * ((z_q(1)-z_desired(1))^2 + (z_q(2)-z_desired(2))^2 + (z_q(3)-z_desired(3))^2);
    
    function rotx = rotx(alpha)
        rotx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
    end
    
    function roty = roty(beta)
        roty = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
    end
    
    function rotz = rotz(gamma)
        rotz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0 0 1];
    end
end


