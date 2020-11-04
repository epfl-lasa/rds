
%% Created by Diego F. Paez
% Date: June 6, 2020
% Function for evaluating similarity of commands in shared control

% % Call example:
% [fluency] = user_effort(Command_U ,Command_R, Vel_max, Omega_max);

% A test decision for the null hypothesis that the data in vectors x and y
% comes from independent random samples from normal distributions
% with equal means and equal but unknown variances.

%% 

clear all; close all; clc; 

% load('simulation_data.mat')
% data = simulation_metrics;

% load('simulation_data_2.mat')
% data = metricsevaluation;

load('simulation_data_4.mat')
data = metricsevaluation2;

data.ORCA_E_t

normplot(data.ORCA_rob_track_err)
figure
normplot(data.RDS_rob_track_err)
h_E_rob_track_err = ttest2(data.ORCA_rob_track_err, data.RDS_rob_track_err)

h_E_t = ttest2(data.ORCA_E_t, data.RDS_E_t)

h_E_v = ttest2(data.ORCA_E_v, data.RDS_E_v)

h_N_ttg = ttest2(data.ORCA_N_ttg, data.RDS_N_ttg)

h_N_v = ttest2(data.ORCA_N_v, data.RDS_N_v)

results=table();
results.mean(1,:)= [ mean(data.ORCA_rob_track_err), ...
                  mean(data.ORCA_E_t), ...
                  mean(data.ORCA_E_v), ...
                  mean(data.ORCA_N_ttg), ...
                  mean(data.ORCA_N_v)];
              
results.mean(2,:) = [ mean(data.RDS_rob_track_err), ...
                  mean(data.RDS_E_t), ...
                  mean(data.RDS_E_v), ...
                  mean(data.RDS_N_ttg), ...
                  mean(data.RDS_N_v)];

results.sd(1,:)= [ std(data.ORCA_rob_track_err), ...
                  std(data.ORCA_E_t), ...
                  std(data.ORCA_E_v), ...
                  std(data.ORCA_N_ttg), ...
                  std(data.ORCA_N_v)];
              
results.sd(2,:) = [ std(data.RDS_rob_track_err), ...
                  std(data.RDS_E_t), ...
                  std(data.RDS_E_v), ...
                  std(data.RDS_N_ttg), ...
                  std(data.RDS_N_v)];
results.collision = [sum(data.ORCA_collisions); sum(data.RDS_collisions)]
              
figure
p_E_rob_track_err = anova1([data.ORCA_rob_track_err, data.RDS_rob_track_err])

p_E_t = anova1([data.ORCA_E_t, data.RDS_E_t])

p_E_v = anova1([data.ORCA_E_v, data.RDS_E_v])

p_N_ttg = anova1([data.ORCA_N_ttg, data.RDS_N_ttg])

p_N_v = anova1([data.ORCA_N_v, data.RDS_N_v])



%% Box plot for paper:
% %    DataM = [ data.ORCA_rob_track_err  data.RDS_rob_track_err, ...
    DataM = [data.ORCA_E_t-data.RDS_E_t,...
             data.ORCA_E_v-data.RDS_E_v,...
             data.ORCA_N_ttg-data.RDS_N_ttg,...
             data.ORCA_N_v-data.RDS_N_v,...
              ];
   Datafields = {'ORCA', 'RDS'};

%     iosr.statistics.functionalBoxPlot 
    
    boxplot(DataM)

%%
   DataM = [ data.ORCA_rob_track_err  data.RDS_rob_track_err]
    Datafields = {'ORCA', 'RDS'};
    boxplot(DataM,Datafields)
    