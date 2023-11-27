% Visualize speed error
% Mehmet Furkan Dogan
clc;clear;close all;
addpath('./speed sensor data')
%% Read measurements
% motor speeds for 0 100 200 300 400 500 -100 -200 -300 -400 -500
motor_left_speed = zeros(4950,11);
motor_right_speed = zeros(4950,11);

fileID = fopen('motor_speed_0.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,1) = C{1}(end-4949:end);
motor_right_speed(:,1) = C{2}(end-4949:end);

fileID = fopen('motor_speed_100.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,2) = C{1}(end-4949:end);
motor_right_speed(:,2) = C{2}(end-4949:end);

fileID = fopen('motor_speed_200.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,3) = C{1}(end-4949:end);
motor_right_speed(:,3) = C{2}(end-4949:end);

fileID = fopen('motor_speed_300.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,4) = C{1}(end-4949:end);
motor_right_speed(:,4) = C{2}(end-4949:end);

fileID = fopen('motor_speed_400.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,5) = C{1}(end-4949:end);
motor_right_speed(:,5) = C{2}(end-4949:end);

fileID = fopen('motor_speed_500.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,6) = C{1}(end-4949:end);
motor_right_speed(:,6) = C{2}(end-4949:end);

fileID = fopen('motor_speed_-100.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,7) = C{1}(end-4949:end);
motor_right_speed(:,7) = C{2}(end-4949:end);

fileID = fopen('motor_speed_-200.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,8) = C{1}(end-4949:end);
motor_right_speed(:,8) = C{2}(end-4949:end);

fileID = fopen('motor_speed_-300.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,9) = C{1}(end-4949:end);
motor_right_speed(:,9) = C{2}(end-4949:end);

fileID = fopen('motor_speed_-400.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,10) = C{1}(end-4949:end);
motor_right_speed(:,10) = C{2}(end-4949:end);

fileID = fopen('motor_speed_-500.txt');
C = textscan(fileID,'%d %d');
motor_left_speed(:,11) = C{1}(end-4949:end);
motor_right_speed(:,11) = C{2}(end-4949:end);

%%
figure
hold on;% grid on;
set(gca, 'YGrid', 'on', 'XGrid', 'off');
histogram(motor_left_speed,Normalization='probability',BinWidth=1,FaceColor='b',EdgeColor='none');
histogram(motor_right_speed,Normalization="probability",BinWidth=1,FaceColor='r',EdgeColor='none');

legend('left','right',Location='best');
fclose('all');