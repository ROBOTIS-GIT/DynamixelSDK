clear
clc
close


%% Frames and Joints
orig_frame = Frame([0; 0; 0], [], 'Origin');
joint1 = Joint([0; 0; 83.51], orig_frame, 'Joint 1', 'x');
joint2 = Joint([0;0;0], joint1, 'Joint 2', 'y');
joint3 = Joint([0;0;119.35], joint2, 'Joint 3', 'z');
joint4 = Joint([0;0;163.99], joint3, 'Joint 4', 'x');
endeffector_frame = Frame([0;0;218.86], joint4, 'Endeffector');

%% Links
link1 = Link(orig_frame, joint1);
link2 = Link(joint1, joint2);
link3 = Link(joint2, joint3);
link4 = Link(joint3, joint4);
link5 = Link(joint4, endeffector_frame);

%% Robot
robot = Robot([joint1, joint2, joint3, joint4], [link1, link2, link3, link4, link5], [orig_frame, endeffector_frame]);


%% Rotations
joint1.rotate(0)
joint2.rotate(0)
joint3.rotate(0)
joint4.rotate(pi/2)

%Get symbolic Jacobian
J = robot.getJacobian;

% Desired endeffector velocity
x_dot = [0;0;1]; % move straight up with 1 mm /s

% Time increment
dt = 1; % s

while 1

    % Get current joint angles
    sAlpha = robot.joints(1).angle;
    sBeta = robot.joints(2).angle;
    sGamma = robot.joints(3).angle;
    sDelta = robot.joints(4).angle;
    
    %Get numeric Jacobian
    J = double(subs(J));

    % Calculate necessary joint velocity
    q_dot = pinv(J) * x_dot;

    %Apply joint rotation increment 
    robot.joints(1).rotate(q_dot(1)*dt);
    robot.joints(2).rotate(q_dot(2)*dt);
    robot.joints(3).rotate(q_dot(3)*dt);
    robot.joints(4).rotate(q_dot(4)*dt);

    % Visualize the robot
    robot.display();
    drawnow

    % Get endeffector info
    [ref_position, ref_rotation, ref_frame] = endeffector_frame.getInfo();

    pause(dt)

end
