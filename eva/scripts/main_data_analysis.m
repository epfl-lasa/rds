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
    dataset = filenames(5).name;
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

    load(fullfile(data_folder,'command.mat'));
    time = data(:,1)';
    Command_U = data(:,2:3)';
    Command_R = data(:,4:5)';
    
    [linear_diff,heading,disagreement ,Contribution] = similarity(Command_U ,Command_R, Vel_max, Omega_max)
    [fluency] = user_fluency(Command_U, Vel_max, Omega_max)
    Contribution_score = [mean(Contribution); std(Contribution)]

