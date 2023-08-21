clear;
clc
close


%% Setup Frames and Joints of the simulated robot
orig_frame = CustomFrame([0; 0; 0], [], 'Origin');
joint1 = CustomJoint([0; 0; 93.58], orig_frame, 'Joint 1', 'y');
joint2 = CustomJoint([0;0;0], joint1, 'Joint 2', 'x');
joint3 = CustomJoint([0;0;104.57], joint2, 'Joint 3', 'z');
joint4 = CustomJoint([0;0;176.99], joint3, 'Joint 4', 'x');
endeffector_frame = CustomFrame([0;0;211.86], joint4, 'Endeffector');

%% Setup Links of the simulated robot
link1 = CustomLink(orig_frame, joint1, 'r');  % Red
link2 = CustomLink(joint1, joint2, 'g');  % Green
link3 = CustomLink(joint2, joint3, 'b');  % Blue
link4 = CustomLink(joint3, joint4, 'y');  % Yellow
link5 = CustomLink(joint4, endeffector_frame, 'm');  % Magenta


%% Setup simulated robot
% This creates the robot in the default configuration
simulatedRobot = SimulatedRobot([joint1, joint2, joint3, joint4], [link1, link2, link3, link4, link5], [orig_frame, endeffector_frame]);

% Display the robot in the default configuration
clear = 0;
draw_frames = 1;
simulatedRobot.display(clear,draw_frames)

% Rotate joint 1 around its configured rotation axis (y)




