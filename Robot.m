orig_frame = Frame([0, 0, 0], eye(3), []);
figure;
hold on;
grid on;
orig_frame.display('Origin');

joint1_pos = orig_frame.position + [0, 0, 1];
joint1_frame = Frame(joint1_pos, eye(3), orig_frame);
joint1_frame.rotate(pi/2, 'x');
% joint1_frame.display('Joint 1');

Link(orig_frame, joint1_frame).display();

%Need to adjust the Code to not rotate around origin axis but around
%current local axis
joint2_pos = joint1_frame.position;
joint2_frame = Frame(joint2_pos, eye(3), joint1_frame);
joint2_frame.rotate(pi/4, 'y');
joint2_frame.display('Joint 2');

Link(joint1_frame, joint2_frame).display();

view(45, 45);
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Robot Joints');
hold off;