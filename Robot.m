clear
clc
close

figure;
hold on;
grid on;

%% Frames

% Origin is located at the bottom of the servo below the axis crossing of
% the two bevel gera axis
orig_frame = Frame([0; 0; 0], eye(3), [], 'Origin');

% Joint 1 can rotate around his x Axis
joint1_pos = orig_frame.position + [0; 0; 83.51]; % Displacement in the z-Axis
% of the global frame measured from the bottom of the servo to the joint axis
joint1_frame = Frame(joint1_pos, eye(3), orig_frame, 'Joint 1');

% Joint 2 can rotate around his y Axis 
joint2_pos = joint1_frame.position; % joint 2 is at joint 1
joint2_frame = Frame(joint2_pos, eye(3), joint1_frame, 'Joint 2');

% Joint 3 can rotate around his z Axis
joint3_pos = joint2_frame.position + joint2_frame.rotation * [0;0;119.35]; % Displacement
% in the z-Axis of joint2_frame, measured from the joint axis to the top of the red plastic of the plain bearing
joint3_frame = Frame(joint3_pos, eye(3), joint2_frame, 'Joint 3');

% Joint 4 can rotate around his x Axis
joint4_pos = joint3_frame.position + joint3_frame.rotation * [0;0;163.99]; % Displacement
% in the z-Axis of joint3_frame, measured from the top of the red plastic of the plain bearing to the joint axis
joint4_frame = Frame(joint4_pos, eye(3), joint3_frame, 'Joint 4');

% Endeffector 5
endeffector_pos = joint4_frame.position + joint4_frame.rotation * [0;0;218.86]; % Displacement
% in the z-Axis of joint4_frame, measured from the axis to the top of the endeffector mounting plate
endeffector_frame = Frame(endeffector_pos, eye(3), joint4_frame, 'Endeffector');


%% Links
link1 = Link(orig_frame, joint1_frame);
link2 = Link(joint1_frame, joint2_frame);
link3 = Link(joint2_frame, joint3_frame);
link4 = Link(joint3_frame, joint4_frame);
link5 = Link(joint4_frame, endeffector_frame);



%% Rotations

joint1_frame.rotate(pi/8, 'x'); %Rotation about its own x-Axis
joint2_frame.rotate(pi/8, 'y'); %Rotation about its own y-Axis
joint3_frame.rotate(0, 'z'); %Rotation about its own z-Axis

% Here is an error... I have to reset the joint3 position and joint4 pos
% after rotating the parent frames or they wont be placed correctly.
% Need to somehow update the position of the child frame position when ANY
% of the parent frames are rotated...

% joint3_pos = joint2_frame.position + joint2_frame.rotation * [0;0;119.35]; % Displacement
% % in the z-Axis of joint2_frame, measured from the joint axis to the top of the red plastic of the plain bearing
% joint3_frame = Frame(joint3_pos, eye(3), joint2_frame, 'Joint 3');
% 
% % Joint 4 can rotate around his x Axis
% joint4_pos = joint3_frame.position + joint3_frame.rotation * [0;0;163.99]; % Displacement
% % in the z-Axis of joint3_frame, measured from the top of the red plastic of the plain bearing to the joint axis
% joint4_frame = Frame(joint4_pos, eye(3), joint3_frame, 'Joint 4');


joint4_frame.rotate(pi/4, 'x'); % Rotation about its own x-Axis




%% Display all frames and links, might need to do this OOP ( robot class with joints and links) later...

orig_frame.display();
joint1_frame.display();
joint2_frame.display();
joint3_frame.display();
joint4_frame.display();
endeffector_frame.display();

link1.display();
link2.display();
link3.display();
link4.display();
link5.display();



%% Plot settings
view(45, 45);
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Robot Joints');
hold off