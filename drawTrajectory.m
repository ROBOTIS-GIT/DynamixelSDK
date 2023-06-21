% This script simply draws the trajectory of the endeffector while the
% robot is moved by hand.


clear
clc
close

addpath('C:\Users\samue\Documents\Git\DynamixelSDK\realRobot')
addpath('C:\Users\samue\Documents\Git\DynamixelSDK\simulatedRobot')


%% Setup Frames and Joints
orig_frame = CustomFrame([0; 0; 0], [], 'Origin');
joint1 = Joint([0; 0; 83.51], orig_frame, 'Joint 1', 'y');
joint2 = Joint([0;0;0], joint1, 'Joint 2', 'x');
joint3 = Joint([0;0;119.35], joint2, 'Joint 3', 'z');
joint4 = Joint([0;0;163.99], joint3, 'Joint 4', 'x');
endeffector_frame = CustomFrame([0;0;218.86], joint4, 'Endeffector');

%% Setup Links
link1 = CustomLink(orig_frame, joint1, 'r');  % Red
link2 = CustomLink(joint1, joint2, 'g');  % Green
link3 = CustomLink(joint2, joint3, 'b');  % Blue
link4 = CustomLink(joint3, joint4, 'y');  % Yellow
link5 = CustomLink(joint4, endeffector_frame, 'm');  % Magenta


%% Setup simulated and real robot
simulatedRobot = SimulatedRobot([joint1, joint2, joint3, joint4], [link1, link2, link3, link4, link5], [orig_frame, endeffector_frame]);
realRobot = RealRobot();


%% Main

% Zero the robot and move to a non singularity position
realRobot.setZeroPositionToCurrentPosition;
realRobot.robotTorqueEnableDisable(1);
realRobot.setJointVelocity(1,1);
realRobot.setJointVelocity(2,1);
realRobot.setJointVelocity(3,-2);
realRobot.setJointVelocity(4,4);
pause(1)
realRobot.setJointVelocity(1,0);
realRobot.setJointVelocity(2,0);
realRobot.setJointVelocity(3,0);
realRobot.setJointVelocity(4,0);

% Disable Torque so the robot can be guided by hand
realRobot.robotTorqueEnableDisable(0);

ref_positions_array = [];
while 1

    % Get current joint angles and set them to the simulated robot
    simulatedRobot.joints(1).setAngle(realRobot.getJointAngle(1));
    simulatedRobot.joints(2).setAngle(realRobot.getJointAngle(2));
    simulatedRobot.joints(3).setAngle(realRobot.getJointAngle(3));
    simulatedRobot.joints(4).setAngle(realRobot.getJointAngle(4));

    % Visualize the simulated robot which
    % should be a mirror image of the real robot
    clearFig = 0;
    draw_frames = 0;
    simulatedRobot.display(clearFig, draw_frames);
    drawnow

    % Get endeffector info for the trajectory
    display_info = 1;
    [ref_position, ref_rotation, ref_frame] = endeffector_frame.getInfo(display_info);

    % Append the endeffector postion to an array and plot the trajectory
    ref_positions_array = [ref_positions_array ref_position];
    plot3(ref_positions_array(1,:),ref_positions_array(2,:),ref_positions_array(3,:),'k');

end

