
%% Created by Diego F. Paez
% Date: June 6, 2020
% Function for evaluating similarity of commands in shared control

% % Call example:
% [fluency] = user_effort(Command_U ,Command_R, Vel_max, Omega_max);


%% 
clear all; close all; clc; 

%res = dlmread(filename,';')
filename = "../../rds/metrics_evaluation.csv";
M = dlmread(filename, ';', 1, 0);
fid = fopen(filename);
C = textscan(fid, '%[^\n]', 1);
fclose(fid);
listC = strsplit(C{1}{1}, ";");
simulation_metrics = struct();
for i = 1:length(listC)
    simulation_metrics.(strtrim(listC{i})) = M(:,i);
end

h_rob_track_err = ttest2(simulation_metrics.ORCA_rob_track_err, simulation_metrics.RDS_rob_track_err)

h_ped_track_err = ttest2(simulation_metrics.ORCA_ped_track_err, simulation_metrics.RDS_ped_track_err)

h_E_v = ttest2(simulation_metrics.ORCA_E_v, simulation_metrics.RDS_E_v)

h_N_v = ttest2(simulation_metrics.ORCA_N_v, simulation_metrics.RDS_N_v)

return;
%load('simulation_data.mat')

h_E_rob_track_err = ttest(simulation_metrics.ORCA_rob_track_err, simulation_metrics.RDS_rob_track_err)

h_E_t = ttest(simulation_metrics.ORCA_E_t, simulation_metrics.RDS_E_t)

h_E_v = ttest(simulation_metrics.ORCA_E_v, simulation_metrics.RDS_E_v)

h_N_ttg = ttest(simulation_metrics.ORCA_N_ttg, simulation_metrics.RDS_N_ttg)

h_N_v = ttest(simulation_metrics.ORCA_N_v, simulation_metrics.RDS_N_v)



p_E_rob_track_err = anova1([simulation_metrics.ORCA_rob_track_err, simulation_metrics.RDS_rob_track_err])

p_E_t = anova1([simulation_metrics.ORCA_E_t, simulation_metrics.RDS_E_t])

p_E_v = anova1([simulation_metrics.ORCA_E_v, simulation_metrics.RDS_E_v])

p_N_ttg = anova1([simulation_metrics.ORCA_N_ttg, simulation_metrics.RDS_N_ttg])

p_N_v = anova1([simulation_metrics.ORCA_N_v, simulation_metrics.RDS_N_v])




