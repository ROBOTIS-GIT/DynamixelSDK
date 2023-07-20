clear
close all
clc
%Load the data from the laptop
load('C:\Users\samue\Desktop\before_calibration_laptop\33.mat');
calculatedPositions = ref_positions_array;

%Load the data from the motion capture
load('C:\Users\samue\Desktop\before_calibration_record\33.mat');
recordedPositions = out.pos_amree;
recordedOrientations = out.orient_arm_ee;

clc

% Read out the position of the robot pointing straight up in global
% coordinates
% plot3(recordedPositions.data(:,1),recordedPositions.data(:,2),recordedPositions.data(:,3))


% Cut away everything from the recordedPositions except the circle
% cutRecordedPositions = recordedPositions.data(3000:(end-5000),:); % for 55
cutRecordedPositions = recordedPositions.data(1800:(end-3000),:);
% plot3(cutRecordedPositions(:,1), cutRecordedPositions(:,2), cutRecordedPositions(:,3));


clearvars -except calculatedPositions cutRecordedPositions x_offset y_offset



%Scale the calculatedPositions to meter
calculatedPositions = calculatedPositions/1000;

%Cut away everything from the calculatedPositions except the circle
calculatedPositions = calculatedPositions(:,5:end);

%Move the calculataedPositions to the same location as the
% %recordedPositions (offset can be read out in the commented out plot above)
calculatedPositions(1,:) = calculatedPositions(1,:) - 1.8185;
calculatedPositions(2,:) = calculatedPositions(2,:) + 0.5218;
calculatedPositions(3,:) = calculatedPositions(3,:) + (0.6245 - 0.5857100);


% Plot recorded positions X-Y plane (top-view)
plot(cutRecordedPositions(:,1), cutRecordedPositions(:,2));

hold on
%Plot calculatedPositions
plot(calculatedPositions(1,:), calculatedPositions(2,:));

grid on
axis equal
xlabel("X in [m]")
ylabel("Y in [m]")

% Add a legend
legend('Recorded Positions', 'Calculated Positions', 'Location', 'best');


