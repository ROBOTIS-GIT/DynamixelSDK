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


%% Setup Robot
simulatedRobot = SimulatedRobot([joint1, joint2, joint3, joint4], [link1, link2, link3, link4, link5], [orig_frame, endeffector_frame]);
realRobot = RealRobot();

%% Main

% Zero the robot and move to a non singularity position
realRobot.setZeroPositionToCurrentPosition;
realRobot.robotTorqueEnableDisable(1);
realRobot.setJointVelocity(1,2);
realRobot.setJointVelocity(2,2);
realRobot.setJointVelocity(3,-4);
realRobot.setJointVelocity(4,6);
pause(1)
realRobot.setJointVelocity(1,0);
realRobot.setJointVelocity(2,0);
realRobot.setJointVelocity(3,0);
realRobot.setJointVelocity(4,0);

x_desired = [-89.106495, -148.362929, 520.512059]';


ref_positions_array = [];

P_gain = 5;
D_gain = 1; % You may have to adjust this to suit your application
x_error_previous = zeros(3,1); % To store previous error for derivative action


clearFig = 1;
draw_frames = 0;
simulatedRobot.joints(1).setAngle(realRobot.getJointAngle(1));
simulatedRobot.joints(2).setAngle(realRobot.getJointAngle(2));
simulatedRobot.joints(3).setAngle(realRobot.getJointAngle(3));
simulatedRobot.joints(4).setAngle(realRobot.getJointAngle(4));
simulatedRobot.display(clearFig, draw_frames);
scatter3(x_desired(1),x_desired(2),x_desired(3),'g','filled');

pause(2)


for i = 1:1000

    simulatedRobot.joints(1).setAngle(realRobot.getJointAngle(1));
    simulatedRobot.joints(2).setAngle(realRobot.getJointAngle(2));
    simulatedRobot.joints(3).setAngle(realRobot.getJointAngle(3));
    simulatedRobot.joints(4).setAngle(realRobot.getJointAngle(4));

    J = simulatedRobot.getJacobianNumeric;

    if cond(pinv(J)) > 15
        realRobot.setJointVelocity(1,0);
        realRobot.setJointVelocity(2,0);
        realRobot.setJointVelocity(3,0);
        realRobot.setJointVelocity(4,0);
        disp('Warning: Close to singularity!');
        break
    end

    if rad2deg(realRobot.getBevelElevation) < 50
        realRobot.setJointVelocity(1,0);
        realRobot.setJointVelocity(2,0);
        realRobot.setJointVelocity(3,0);
        realRobot.setJointVelocity(4,0);
        disp('Warning: Bevel Gear Elevation limit reached!')
        break
    end


    display_info = 0;
    [ref_position, ref_rotation, ref_frame] = endeffector_frame.getInfo(display_info);

    x_current = ref_position;
    x_error = x_desired - x_current;
    fprintf('Distance to goal: %.0f mm \n', norm(x_error));

    if norm(x_error) < 10
        realRobot.setJointVelocity(1,0);
        realRobot.setJointVelocity(2,0);
        realRobot.setJointVelocity(3,0);
        realRobot.setJointVelocity(4,0);
        disp('Reached position within epsilon');
        break;
    end

    x_dot = P_gain*x_error + D_gain*(x_error_previous - x_error);
    x_error_previous = x_error; % Update previous errorr


    q_dot = pinv(J) * x_dot;

    realRobot.setJointVelocity(1,q_dot(1));
    realRobot.setJointVelocity(2,q_dot(2));
    realRobot.setJointVelocity(3,q_dot(3));
    realRobot.setJointVelocity(4,q_dot(4));

    % Appen trajectory
    ref_positions_array = [ref_positions_array ref_position];

    %Plot
    clearFig = 0;
    draw_frames = 0;
    simulatedRobot.display(clearFig, draw_frames);
    plot3(ref_positions_array(1,:),ref_positions_array(2,:),ref_positions_array(3,:),'k');
    scatter3(x_desired(1),x_desired(2),x_desired(3),'g','filled');
    drawnow


end
