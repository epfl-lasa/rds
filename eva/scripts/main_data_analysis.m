%% Created by Diego F. Paez
% Date: June 6, 2020
% Function for evaluating similarity of commands in shared control

% % Call example:
% [fluency] = user_effort(Command_U ,Command_R, Vel_max, Omega_max);


%% 
clear all; close all; clc; 
    
    CLEAR_PLOT = 0;
    PLOT_FLAG = 1;
    RECORD_FLAG = 0;
    Omega_max = 0.8;    % Using the test maximum values [ real maximum 1.04]
    Vel_max = 0.8;  % Use the test maximum [real max value 1.5]
    
% Get all available datasets
%     filenames = loadDataNames();
    filenames = dir(fullfile(pwd,'jun_10_1'));
% Select datase to evaluate (number from 3 to N)
    datasetNumber = 5; 
    dataset = filenames(datasetNumber).name;
    data_folder = fullfile(pwd,'jun_10_1',dataset);
    plot_folder = fullfile(pwd,'Plot', dataset);
%     test_folders = dir(fullfile(data_folder))';
    
% Get the name of all files in the folder
    test_data = dir((data_folder));
%     for ii=1:size(test_data)-2
%         load(fullfile(data_folder,test_data(ii).name))
%     end
%      dataTypeNumber = 5;
%     load(fullfile(test_data(dataTypeNumber).name,'command.mat'));


%% Evaluating metrics to the set test

    load(fullfile(data_folder,'command.mat'));
    time = data(:,1)';
    Command_U = data(:,2:3)';
    Command_R = data(:,4:5)';
    
    [linear_diff,directional_agreement,disagreement ,Contribution] = similarity(Command_U ,Command_R, Vel_max, Omega_max);
    directional_agreement
    [fluency] = user_fluency(Command_U, Vel_max, Omega_max)
    Contribution_score = [mean(Contribution); std(Contribution)]
    
%% Loading lrf points and evaluation of mean and minimal distance 

    load(fullfile(data_folder,'lrf_object.mat'));
    lrf_points=data;
    for ii=1:size(lrf_points,1)
    iend = find(isnan(lrf_points(ii,:)),1);
        kk=0;
        for jj=1:2:iend-2
            kk = kk+1;
            points_distance(kk) = norm(lrf_points(ii,jj),lrf_points(ii,jj+1));
        end
        closest_point(ii) = min(points_distance);
        mean_distance(ii) = mean(points_distance);
%         clear points_distance;
    end

%% Loading tracker objects and analyzing risk measurement
    % A time window for analyzing the risk in the inmedieate future
    t_window = 30; 

   load(fullfile(data_folder,'tracker_object.mat'));
    tracker_object=data;
    for ii=1:size(tracker_object,1)
    iend = find(isnan(tracker_object(ii,:)),1);
        kk=0;
        for jj=1:4:iend-4
            kk = kk+1;
            object_location(kk) = norm(tracker_object(ii,jj),tracker_object(ii,jj+1));
            object_vel(kk) = norm(tracker_object(ii,jj+2),tracker_object(ii,jj+3));
            angle_R = atan2(Command_R(1,:),Command_R(2,:))
            
            robot_line = [0, 0; Command_R(1)*t_window, Command_R(2)*t_window];
        end


        [xi,yi] = polyxpoly(x,y,box,ybox);

        closest_point(ii) = min(points_distance);
        mean_distance(ii) = mean(points_distance);
%         clear points_distance;
    end

    

