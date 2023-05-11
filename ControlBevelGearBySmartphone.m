%% Tip: To stop the plotting, click the console window and press Strg+C

clc
close all
clear

%Enable orientation sensor
addpath(genpath('C:\Users\samue\Documents\Git\mobile_sensors_to_matlab_simulink'))
m = init_sensors(0,1,0,0,0);

%% 3D Draw the pose of the smartphone in world frame

%Set an initial yaw so the x-Axis of world frame and smartphone frame are alligned at startup.
starting_rotation = 90; %[DEG]
initial_yaw = deg2rad(m.Orientation(1) + starting_rotation);

B = BevelGear('COM3');
B.setComposingServos(3,4);
B.torqueEnableDisable(1);
B.pointStraightUp;

while(1)
    
    %Convert Yaw Roll and Pitch to RAD and calculate yaw in relation to initial_yaw
    yaw = initial_yaw-deg2rad(m.Orientation(1));
    pitch = deg2rad(m.Orientation(2));
    roll = deg2rad(m.Orientation(3));
   
    %Calculate the transformation matrix from smartphone frame to world frame
    R_smartphone_to_world = Rz(yaw)*Ry(pitch)*Rx(roll)*Rz(-pi/2);
    
    %Calculate the transformation matrix fro world frame to smartphone frame
    R_world_to_smartphone = inv(R_smartphone_to_world);

    %Define world frame
    world_frame = [1 0 0;
                   0 1 0; 
                   0 0 1];
    
    %Calculate smartphone frame
    smartphone_frame = R_world_to_smartphone * world_frame;
   
    
    %Plot smartphone frame
    quiver3(0, 0, 0, smartphone_frame(1,1), smartphone_frame(1,2), smartphone_frame(1,3) ,'r', 'MaxHeadSize', 3, 'LineWidth', 3);
    hold on
    quiver3(0, 0, 0, smartphone_frame(2,1), smartphone_frame(2,2), smartphone_frame(2,3) ,'g', 'MaxHeadSize', 3, 'LineWidth', 3);
    quiver3(0, 0, 0, smartphone_frame(3,1), smartphone_frame(3,2), smartphone_frame(3,3) ,'b', 'MaxHeadSize', 3, 'LineWidth', 3);
    axis([-2 2 -2 2 -2 2]);
    hold off
    drawnow

    fprintf("SmartphoneX: %.3f \t SmartphoneY: %.3f \t SmartphoneZ: %.3f \t  \n", smartphone_frame(3,1), smartphone_frame(3,2), smartphone_frame(3,3));
    v = [smartphone_frame(3,1), -smartphone_frame(3,2), smartphone_frame(3,3)];

    B.setVector(v);



end


%% Rotatoin Matrix Definition

function Rz = Rz(rotZ)
    
        Rz = [cos(rotZ) -sin(rotZ) 0;
             sin(rotZ)  cos(rotZ) 0;
              0        0     1;];

end

function Ry = Ry(rotY)
    
        Ry = [cos(rotY) 0 sin(rotY);
              0         1       0;
             -sin(rotY) 0 cos(rotY)];

end

function Rx = Rx(rotX)
    
        Rx = [1      0             0;
              0 cos(rotX)  -sin(rotX);
              0 sin(rotX)  cos(rotX)];

end

