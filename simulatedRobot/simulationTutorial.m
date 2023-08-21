% This script serves as a basic tutorial on how the SimulatedRobot with
% its Frames Joints and Links works.

clear()
clc
close


%% Create an origin frame (frame = coordinate system)
orig_frame = CustomFrame([0; 0; 0], [], 'Origin');
% The origin frame has no parent frame and is positioned at global
% coordinates [0; 0; 0]. Its orientation is represented as a 3x3 rotational
% matrix which is assigned the identity matrix.

%% Create a SimulatedRobot and add the frame
simulatedRobot = SimulatedRobot([],[],orig_frame);
simulatedRobot.display(1)


%% Rotate the origin frame by 45° around its z axis
orig_frame.rotate(pi/4,'z') 
simulatedRobot.display(1)


%% Add another frame relative to the orig_frame --> partent = orig_frame
second_frame = CustomFrame([0;0;100], orig_frame, 'second frame');
simulatedRobot.frames = [simulatedRobot.frames, second_frame];
simulatedRobot.display(1)
% The second frame is positioned relative to the origin frame, namely 100
% units in the direction of the z-axis of the origin frame.
% The rotational matrix of the second frame is set to the same
% rotational matrix as the origin_frame. The origin frame is the parent of
% the second frame.

%% Add a link between the two frames
first_link = CustomLink(orig_frame,second_frame,'cyan');
simulatedRobot.links = [simulatedRobot.links, first_link];
simulatedRobot.display(1)
% The link connects both frames by a colored line


%% Rotate the origin frame by 45° around its x axis
orig_frame.rotate(pi/4,'x') 
simulatedRobot.display(1)
% The origin frame is rotated 45° around its x axis. Then all of it's
% decendents (children, grandchildren, ...) are also rotated by 45° around the x axis of the
% origin frame. This means the relative position of the origin frame
% relative to all its decendents does not change.


%% Add a joint relative to the second frame --> parent = second_frame (grandparent = orig_frame ..)
joint = CustomJoint([100;0;0],second_frame,'joint','x');
simulatedRobot.joints = [simulatedRobot.joints, joint];
simulatedRobot.display(1)
% A joint is just a frame with one extra property : rotationAxisLabel
% The rotationAxisLabel defines the one local rotation axis the joint is
% allowed to rotate about.

%% Add a link from the second frame to the joint
second_link = CustomLink(second_frame,joint,'yellow');
simulatedRobot.links = [simulatedRobot.links, second_link];
simulatedRobot.display(1)

%% Add an endeffector frame and link at the end of the kinematic chain
endeffector_frame = CustomFrame([0;100;0],joint,'endeffector frame');
third_link = CustomLink(joint,endeffector_frame,'magenta');
simulatedRobot.links = [simulatedRobot.links, third_link];
simulatedRobot.frames = [simulatedRobot.frames, endeffector_frame];
simulatedRobot.display(1)

%% Rotate the joint around its allowed axis
joint.rotate(pi/8) 
simulatedRobot.display(1)
% This performes a relative rotation around the joints defined local axis (x)

%% Rotate the joint again --> rotates further
joint.rotate(pi/8) 
simulatedRobot.display(1)
% The rotation is added to the previous rotation

%% The joint angle can also be set absolutely
joint.setAngle(0)
simulatedRobot.display(1)
% The rotation is overwritten, previous rotation does not matter

%% Rotate the joint again --> rotates further
joint.rotate(pi/8) 
simulatedRobot.display(1)
% The rotation is added to the previous rotation


%% Calculate the position and rotation of the endeffector frame in global coordinates
[global_position, global_orientation] = endeffector_frame.getInfo(1);
% This method outputs the position and rotation of a frame relative to any
% given frame (here the orig_frame = global frame). It also displays the
% parent and children of the frame. (forward kinematics)

%% Calculate the position and rotation of the endeffector frame relative to the second frame
[rel_position, rel_orientation] = endeffector_frame.getInfo(1,second_frame);



%% Note regarding the forwardKinematics and getJacobian methods of simulatedRobot:
% These methods assumes the robot has its normal composition of frames /
% joints / links. Meaning these methods won't work in this scripts
% configuration. See simulationExample for those.

