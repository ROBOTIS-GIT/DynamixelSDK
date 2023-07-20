clear
close all
clc

%Load the data from the laptop
load('C:\Users\samue\Desktop\Semesterarbeit\First_Testing_Robotic_Arm\before_calibration_laptop\22.mat');
calculatedPositions = ref_positions_array;

%Load the data from the motion capture
load('C:\Users\samue\Desktop\Semesterarbeit\First_Testing_Robotic_Arm\before_calibration_record\22.mat');
recordedPositions = out.pos_amree;
recordedOrientations = out.orient_arm_ee;


clc

% Read out the position of the robot pointing straight up in global
% coordinates
plot3(recordedPositions.data(:,1),recordedPositions.data(:,2),recordedPositions.data(:,3))


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


%Get the length of cutRecordedPositions and calculatedPositions
len_cutRecorded = size(cutRecordedPositions, 1);
len_calculated = size(calculatedPositions, 2);

%Generate new sample points
newSamplePoints = linspace(1, len_calculated, len_cutRecorded);

%Interpolate the calculatedPositions to match length of cutRecordedPositions
resampledPositions = interp1(1:len_calculated, calculatedPositions', newSamplePoints)';


% Plot recorded position
plot(cutRecordedPositions(:,3));
hold on
% Plot calculatedPosition
plot(resampledPositions(3,:));

% Add a legend
legend('Recorded Positions', 'Calculated Positions', 'Location', 'best');

% Y-label
ylabel("Global Z [m]")
