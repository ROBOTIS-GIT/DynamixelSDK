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
joint3.rotate(pi/4)
joint4.rotate(pi/2)

%% Display
robot.display();
[ref_position, ref_rotation, ref_frame] = endeffector_frame.getInfo();


%% Plot settings

hold off