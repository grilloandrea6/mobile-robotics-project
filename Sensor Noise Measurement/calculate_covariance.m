% Calculate the measurement noise covariance
% Mehmet Furkan Dogan
clc;clear;close all;
addpath('./speed sensor data')
%% Read measurements
% motor speeds for 0 100 200 300 400 500 -100 -200 -300 -400 -500

fileID = fopen('motor_speed_0.txt');
C = textscan(fileID,'%d %d');
measurement_error_left = C{1}(end-4949:end);
measurement_error_right = C{2}(end-4949:end);

fileID = fopen('motor_speed_100.txt');
C = textscan(fileID,'%d %d');
speed = 100;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];

fileID = fopen('motor_speed_200.txt');
C = textscan(fileID,'%d %d');
speed = 200;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];

fileID = fopen('motor_speed_300.txt');
C = textscan(fileID,'%d %d');
speed = 300;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];

fileID = fopen('motor_speed_400.txt');
C = textscan(fileID,'%d %d');
speed = 400;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];

fileID = fopen('motor_speed_500.txt');
C = textscan(fileID,'%d %d');
speed = 500;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];

fileID = fopen('motor_speed_-100.txt');
C = textscan(fileID,'%d %d');
speed = -100;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];

fileID = fopen('motor_speed_-200.txt');
C = textscan(fileID,'%d %d');
speed = -200;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];

fileID = fopen('motor_speed_-300.txt');
C = textscan(fileID,'%d %d');
speed = -300;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];

fileID = fopen('motor_speed_-400.txt');
C = textscan(fileID,'%d %d');
speed = -400;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];

fileID = fopen('motor_speed_-500.txt');
C = textscan(fileID,'%d %d');
speed = -500;
measurement_error_left = [measurement_error_left;C{1}(end-4949:end)-speed];
measurement_error_right = [measurement_error_right;C{2}(end-4949:end)-speed];
%% Scatter Plot
alpha_val = 0.1; % Set the desired transparency value
alpha_data = ones(size(measurement_error_left)) * alpha_val;
size_val = 10;
size_data = ones(size(measurement_error_left)) * size_val;
set(gca, 'ColorOrder', [0 0 0], 'NextPlot', 'add'); % Keep marker colors the same
scatter(measurement_error_left, measurement_error_right,size_data, 'filled',...
    'MarkerEdgeAlpha', alpha_val, 'MarkerFaceAlpha', alpha_val, 'AlphaData', alpha_data);
xlabel('measurement error left')
ylabel('measurement error right')
%% Calculate Covariance
disp('Covariance matrix for speed sensor: (right,left)');
disp(cov(double(measurement_error_right),double(measurement_error_left)));