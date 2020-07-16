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
    dataDate = 'jun_10_1';
    % % % 4 - overcoming / 5 - backwards / 8 - intersection / 
    
    %     dataDate='jun_10_2';
    % % % 7 - intersection / 10 - narrow corridor    
    
    datasetNumber = 8; 
    
    filenames = dir(fullfile(pwd,dataDate));
    Mean_table = table();
    Std_table = table();
%%
%     for datasetNumber=4:length(filenames)
        
        dataset = filenames(datasetNumber).name
        data_folder = fullfile(pwd,dataDate,dataset);
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

        [results.linear_diff,results.directional_agreement,results.disagreement ,Contribution] = similarity(Command_U ,Command_R, Vel_max, Omega_max);
        results.fluency = user_fluency(Command_U, Vel_max, Omega_max);
        results.Contribution_score = [mean(Contribution); std(Contribution)];

    %% Loading lrf points and evaluation of mean and minimal distance 

    %     load(fullfile(data_folder,'lrf_object.mat'));
    %     lrf_points=data;
    %     for ii=1:size(lrf_points,1)
    %     iend = find(isnan(lrf_points(ii,:)),1);
    %         kk=0;
    %         for jj=1:2:iend-2
    %             kk = kk+1;
    %             points_distance(kk) = norm(lrf_points(ii,jj),lrf_points(ii,jj+1));
    %         end
    %         closest_point(ii) = min(points_distance);
    %         mean_distance(ii) = mean(points_distance);
    % %         clear points_distance;
    %     end
        load(fullfile(data_folder,'t_shortest_distance_lrf_all.mat'));    
        if isnan(data(1,2))
            load(fullfile(data_folder,'t_shortest_distance_track_all.mat'));    
        end
    %     time_dist = 
        closest_dist = data(~isnan(data(:,2)),2);
        results.min_object = [mean(closest_dist); std(closest_dist)];
        results.min_dist = min(closest_dist);


        %%  Exporting results to tables
        indx=1;
        meanTable = 100.*[  results.linear_diff(indx); 
                            results.directional_agreement(indx);
                            results.disagreement(indx);
                            results.fluency(indx);
                            results.Contribution_score(indx); 
                            results.min_object(indx)./100;
                            results.min_dist./100 ...
                            ];
        meanTable = round(meanTable,2)
        indx=2;
        stdTable = 100.*[   results.linear_diff(indx); 
                            results.directional_agreement(indx);
                            results.disagreement(indx);
                            results.fluency(indx);
                            results.Contribution_score(indx); 
                            results.min_object(indx)./100 ...
                            ];
        stdTable = round(stdTable,2)

        if RECORD_FLAG
            if ~exist(plot_folder, 'dir')
                mkdir(plot_folder)
            end
            save(strcat(plot_folder,'results.mat'),'results')
        end
       
        Mean_table.(dataset) = meanTable;
        Std_table.(dataset) = stdTable;
        
%     end
%     save(strcat(dataDate,'_results_table.mat'),'Mean_table','Std_table')
   
% %% Loading tracker objects and analyzing risk measurement
%     % A time window for analyzing the risk in the inmedieate future
%     t_window = 30; 
% 
%    load(fullfile(data_folder,'tracker_object.mat'));
%     tracker_object=data;
%     for ii=1:size(tracker_object,1)
%     iend = find(isnan(tracker_object(ii,:)),1);
%         kk=0;
%         for jj=1:4:iend-4
%             kk = kk+1;
%             object_location(kk,:) = [tracker_object(ii,jj), tracker_object(ii,jj+1)];
%             object_vel(kk,:) = [ tracker_object(ii,jj+2), tracker_object(ii,jj+3)];
%             angle_R = atan2(Command_R(1,:),Command_R(2,:));
%             Vx = Command_R(1,:)*cos(angle_R); Vy = Command_R(1,:)*sin(angle_R); 
%             robot_line = [0, 0; Vx*t_window, Vy*t_window];
%             object_line = [object_location(kk)];
%         end
% 
%         [xi,yi] = polyxpoly(x,y,box,ybox);
% 
%         closest_point(ii) = min(points_distance);
%         mean_distance(ii) = mean(points_distance);
% %         clear points_distance;
%     end
% 
%     

