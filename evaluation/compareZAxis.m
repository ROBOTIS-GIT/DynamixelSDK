clear
close all
clc

%% Loading

%Load the data from the laptop
load('C:\Users\samue\Desktop\Semesterarbeit\First_Testing_Robotic_Arm\before_calibration_laptop\44.mat');
calculatedPositions = ref_positions_array;

%Load the data from the motion capture
load('C:\Users\samue\Desktop\Semesterarbeit\First_Testing_Robotic_Arm\before_calibration_record\44.mat');
recordedPositions = out.pos_amree;
recordedOrientations = out.orient_arm_ee;

clc

% Read out the position of the robot pointing straight up in global
% coordinates
% plot3(recordedPositions.data(:,1),recordedPositions.data(:,2),recordedPositions.data(:,3))


% Cut away everything from the recordedPositions except the circle
% cutRecordedPositions = recordedPositions.data(3000:(end-5000),:); % for 55
cutRecordedPositions = recordedPositions.data(2000:(end-3000),:);
% plot3(cutRecordedPositions(:,1), cutRecordedPositions(:,2), cutRecordedPositions(:,3));


clearvars -except calculatedPositions cutRecordedPositions x_offset y_offset


%Scale the calculatedPositions to meter
calculatedPositions = calculatedPositions/1000;

%Cut away everything from the calculatedPositions except the circle
calculatedPositions = calculatedPositions(:,5:(end));

%Move the calculataedPositions to the same location as the
% %recordedPositions (offset can be read out in the commented out plot above)
calculatedPositions(1,:) = calculatedPositions(1,:) - 1.80;
calculatedPositions(2,:) = calculatedPositions(2,:) + 0.53;
calculatedPositions(3,:) = calculatedPositions(3,:) + 0.05;

calculatedPositions = calculatedPositions';

% Generate new x-axis values for upsampling
new_x = linspace(1, length(calculatedPositions), length(cutRecordedPositions));

% Use interp1 to upsample calculatedPositions
upsampled_calculatedPositions = interp1(1:length(calculatedPositions), calculatedPositions(:,3), new_x, 'linear');

% Plot only z with increased line width
plot(cutRecordedPositions(:,3), 'LineWidth', 2);
hold on
plot(upsampled_calculatedPositions, 'LineWidth', 2);

grid on
xlabel("time   [ms]")
ylabel("Z   [m]")

% Add a legend with increased font size
lgd = legend('Recorded Positions', 'Calculated Positions', 'Location', 'best');
lgd.FontSize = 36;

% Set x-axis limits
xlim([0 25000]);

% Increase the font size of the axis tick labels
ax = gca;
ax.FontSize = 26;


