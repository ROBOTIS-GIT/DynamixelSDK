clear
clc
close


%% Frames
orig_frame = Frame([0; 0; 0], [], 'Origin');
joint1_frame = Frame([0; 0; 83.51], orig_frame, 'Joint 1');
joint2_frame = Frame([0;0;0], joint1_frame, 'Joint 2');
joint3_frame = Frame([0;0;119.35], joint2_frame, 'Joint 3');
joint4_frame = Frame([0;0;163.99], joint3_frame, 'Joint 4');
endeffector_frame = Frame([0;0;218.86], joint4_frame, 'Endeffector');

%% Links
link1 = Link(orig_frame, joint1_frame);
link2 = Link(joint1_frame, joint2_frame);
link3 = Link(joint2_frame, joint3_frame);
link4 = Link(joint3_frame, joint4_frame);
link5 = Link(joint4_frame, endeffector_frame);

%% Robot
robot = Robot([orig_frame, joint1_frame, joint2_frame, joint3_frame, joint4_frame, endeffector_frame], [link1, link2, link3, link4, link5]);

%% Rotations
joint1_frame.rotate(pi/4, 'x');
joint2_frame.rotate(0, 'y');
joint3_frame.rotate(pi/2, 'z');
joint4_frame.rotate(pi/2, 'x');
joint4_frame.rotate(pi/2, 'z');

%% Display
robot.display();


%% Plot settings

hold off
