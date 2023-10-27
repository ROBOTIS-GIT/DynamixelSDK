clear()
clc
close

addpath('C:\Users\samue\Documents\Git\Robotic-Arm-Prototype\SimulatedRobot')

%% Setup simulated robot and controller
sr = SimulatedRobot();
controller = NullspaceController();

%% Set the robot to a non-singularity position
sr.setQ([0.3; 0.3; 0.5; 0.5])
sr.draw(0)

%% Desired position and allowed deviation in [mm]
% x_desired =  [300; 400; 400];
% z_desired = [-1;0;0];
% scatter3(x_desired(1), x_desired(2), x_desired(3), (5^2) * pi, 'm', 'filled');

%% Create trajectory
R = 120;                 % [mm] Radius of the circle
Z_mean = 400;            % [mm] Mean value of z
Z_amplitude = 50;        % [mm] Amplitude of the sine wave (change based on desired amplitude)
k = 2;                   % Number of waves per full circle
t_total = 20;            % [s] time for whole trajectory
dt = 0.01;               % [s] Time increment between points of the trajectory

% Calculate number of points based on total time and time increment
num_points = round(t_total/dt);

% Calculate the trajectory
theta = linspace(0, 2*pi, num_points);
x_d(1,:) = R * cos(theta);             % x-coordinates
x_d(2,:) = R * sin(theta);             % y-coordinates
x_d(3,:) = Z_mean + Z_amplitude * sin(k * theta);  % z-coordinates

% Calculate the velocity (time derivative of position)
v_d = zeros(size(x_d));

% Using forward differences for all but the last point
for timesteps = 1:num_points-1
    v_d(:,timesteps) = (x_d(:,timesteps+1) - x_d(:,timesteps)) / dt;
end

% Using backward difference for the last point
v_d(:,num_points) = (x_d(:,num_points) - x_d(:,num_points-1)) / dt;

% %Calc max speed in mm/s
% max_speed = 0;
% for timesteps = 1:num_points
%     speed = sqrt(v_d(:,timesteps)'*v_d(:,timesteps));
%     if speed >= max_speed
%         max_speed = speed;
%     end
% end
% fprintf("Max Endeffector Speed in the trajectory is : %.2f km/h\n\n", (max_speed/1000)*3.6);

% Plot the desired trajectory
plot3(x_d(1,:),x_d(2,:),x_d(3,:));

%% Control Loop
loopBeginTime = tic;
t = 0;
n = 1;
tcp_positions = zeros(3,num_points);
while n < num_points
    t = t + dt;
    tcp_positions(:,n) = sr.forwardKinematicsNumeric(sr.getQ);

    q_dot = controller.computeDesiredJointVelocity(sr, x_d(:,n), NaN , 0);
    sr.setQ(sr.getQ + q_dot*dt)
    
        
    % Display the robot
    plot3(tcp_positions(1,1:n), tcp_positions(2,1:n), tcp_positions(3,1:n), 'k');
    sr.draw(0);
    sr.frames(end).draw;
    drawnow limitrate

    % Wait if simulation is faster than real time
    passedRealTime = toc(loopBeginTime);
    if passedRealTime < t
        pause(t-passedRealTime)
    end
    n = n +1;


end

