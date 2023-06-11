clear
clc
close

orig_frame = Frame([0; 0; 0], eye(3), [], 'Origin');
figure;
hold on;
grid on;

joint1_pos = orig_frame.position + [0; 0; 1]; % Displacement in the z-Axis of the global frame
joint1_frame = Frame(joint1_pos, eye(3), orig_frame, 'Joint 1');
joint1_frame.rotate(pi/2, 'x'); % Rotation about the global x-Axis

joint2_pos = joint1_frame.position + joint1_frame.rotation * [0; 0; 1]; % Displacement in the z-Axis of the parent frame
joint2_frame = Frame(joint2_pos, eye(3), joint1_frame, 'Joint 2');
joint2_frame.rotate(pi/4, 'y');  % Rotation about its own y-Axis

% Display
orig_frame.display();
fprintf("\n")
joint1_frame.display();
fprintf("\n")
joint2_frame.display();

% Links
link1 = Link(joint1_frame, joint2_frame);
link1.display();

link2 = Link(orig_frame, joint1_frame);
link2.display();

% Plot settings
view(45, 45);
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Robot Joints');

% Test
joint1_frame.rotate(-pi/2, 'x');

% Update and display frames
joint1_frame.display();
joint2_frame.display();

% Update and display links
link1.update();
link2.update();

hold off;
