clear
close all
clc

%Load the first data from the laptop
load('C:\Users\samue\Desktop\before_calibration_record\33.mat');
recordedPositionsFirst =  out.pos_amree;

%Load the second data from the laptop
load('C:\Users\samue\Desktop\before_calibration_record\44.mat');
recordedPositionsSecond =  out.pos_amree;

%Load the third data from the laptop
load('C:\Users\samue\Desktop\after_calibration_record\calibrated55.mat');
recordedPositionsThird =  out.pos_amree;

clc


% Cut away everything from the recordedPositions except the circle
% cutRecordedPositions = recordedPositions.data(3000:(end-5000),:); % for 55
recordedPositionsFirst = recordedPositionsFirst.data(1800:(end-3000),:);
% plot3(cutRecordedPositions(:,1), cutRecordedPositions(:,2), cutRecordedPositions(:,3));

% Cut away everything from the recordedPositions except the circle
% cutRecordedPositions = recordedPositions.data(3000:(end-5000),:); % for 55
recordedPositionsSecond = recordedPositionsSecond.data(1800:(end-3000),:);
% plot3(cutRecordedPositions(:,1), cutRecordedPositions(:,2), cutRecordedPositions(:,3));

% Cut away everything from the recordedPositions except the circle
% cutRecordedPositions = recordedPositions.data(3000:(end-5000),:); % for 55
recordedPositionsThird = recordedPositionsThird.data(1800:(end-3000),:);
% plot3(cutRecordedPositions(:,1), cutRecordedPositions(:,2), cutRecordedPositions(:,3));


recordedPositionsThird(:,1) = recordedPositionsThird(:,1) -  0.09;
recordedPositionsThird(:,2) = recordedPositionsThird(:,2) -  0.10;


%Plot recordedPositions
plot(recordedPositionsFirst(:,1), recordedPositionsFirst(:,2));
hold on
plot(recordedPositionsSecond(:,1), recordedPositionsSecond(:,2));
plot(recordedPositionsThird(:,1), recordedPositionsThird(:,2));

grid on
axis equal
xlabel("X in [m]")
ylabel("Y in [m]")

% Add a legend
legend('33 Recorded Positions', '44 Recorded Positions', '55 Recorded Positions', 'Location', 'best');


