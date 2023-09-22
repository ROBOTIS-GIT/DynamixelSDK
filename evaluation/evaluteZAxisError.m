clear
close all


%% Loading

%Load the data from the laptop
load('C:\Users\samue\Desktop\Semesterarbeit\First_Testing_Robotic_Arm\before_calibration_laptop\44.mat');
calculatedPositions = ref_positions_array;

%Load the data from the motion capture
load('C:\Users\samue\Desktop\Semesterarbeit\First_Testing_Robotic_Arm\before_calibration_record\44.mat');
recordedPositions = out.pos_amree;
recordedOrientations = out.orient_arm_ee;

%Load the calculated distance from [0; 0; 500];
load('C:\Users\samue\Desktop\dist_to_orig_array.mat', 'distance_to_origin_array')
distanceOrigin = distance_to_origin_array';

% Cut away everything from the recordedPositions except the circle
recordedPositions = recordedPositions.data(2000:(end-3000),:);

calculatedPositions = calculatedPositions(:,5:end)';

clc

clearvars -except calculatedPositions recordedPositions distanceOrigin


%% Upsampling

% Number of rows for each array
n_calculatedPositions = size(calculatedPositions, 1);
n_distanceOrigin = size(distanceOrigin, 1);
n_recordedPositions = size(recordedPositions, 1);

% Create original index vectors for calculatedPositions and distanceOrigin
original_index_calculatedPositions = linspace(1, n_recordedPositions, n_calculatedPositions);
original_index_distanceOrigin = linspace(1, n_recordedPositions, n_distanceOrigin);

% Create target index vector based on the size of recordedPositions
target_index = 1:n_recordedPositions;

% Interpolate to upsample calculatedPositions
calculatedPositions = interp1(original_index_calculatedPositions, calculatedPositions, target_index, 'linear', 'extrap');

% Interpolate to upsample distanceOrigin
distanceOrigin = interp1(original_index_distanceOrigin, distanceOrigin, target_index, 'linear', 'extrap')';

clearvars -except calculatedPositions recordedPositions distanceOrigin

%% Extraction

calculated_z = calculatedPositions(:,3)/1000;
recorded_z = recordedPositions(:,3);

%Move the calculataedPositions to the same location as the
% %recordedPositions (offset can be read out in the commented out plot above)
calculated_z = calculated_z + 0.05;

%Convert distanceOrigin to meters
distanceOrigin = distanceOrigin/1000;
    
clearvars -except calculated_z recorded_z distanceOrigin

%% Error calculation of the z-Value

% Calculate the smoothed error in the z-value
z_error = abs(calculated_z - recorded_z);
z_error = smooth(z_error,2000);

clearvars -except z_error distanceOrigin


%% Plotting

% Create a new figure
figure;

% Create a new figure
figure;

% Create a new figure
figure;

% Get the default color order
default_colors = get(groot, 'defaultAxesColorOrder');

% First subplot: Plotting distanceOrigin
subplot(2, 1, 1);  % 2 rows, 1 column, first plot
plot(distanceOrigin*100, 'Color', default_colors(1, :), 'LineWidth', 1.5);
title('Distance from Origin', 'FontSize', 16);
xlabel('time [ms]', 'FontSize', 14);
ylabel('Distance [cm]', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);

% Second subplot: Plotting the error in Z-Value
subplot(2, 1, 2);  % 2 rows, 1 column, second plot
plot(z_error*100, 'Color', default_colors(2, :), 'LineWidth', 1.5);
title('Sag in z-Axis', 'FontSize', 16);
xlabel('time [ms]', 'FontSize', 14);
ylabel('Error [cm]', 'FontSize', 14);
grid on;
set(gca, 'FontSize', 12);



