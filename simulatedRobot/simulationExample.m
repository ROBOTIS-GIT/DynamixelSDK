% This script shows an example of the roboticArm in the kinematic
% simulation

clear()
clc
close

%% Setup Frames
frame1 = CustomFrame([0;0;0], [], 'frame1');
frame2 = CustomFrame([0;0;100], frame1, 'frame2');


%% Setup Links of the simulated robot
link = CustomLink(frame1, frame2, 'b');  % Blue


%% Setup simulated robot
simulatedRobot = SimulatedRobot([], [link], [frame1, frame2]);

% Animate initial movement to a non-singularity position
for step = 1:100
   

    % simulatedRobot.joints(1).setAngle(simulatedRobot.joints(1).angle + 0.01)
    frame1.rotate(0.01,'x')

    frame2.rotate(0.01,'z')
    
    % simulatedRobot.display(draw_frames)
    % drawnow

end


simulatedRobot.display(1)


