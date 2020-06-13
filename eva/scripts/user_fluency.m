%% Disagreement Measurements

% Created by Diego F. Paez
% Date: Jan 6, 2020
% Function for evaluating similarity of commands in shared control

% % Call example:
% Omega_max = 4.124 /4;
% Vel_max = 1;
% Command_U = commands(2:3,:);
% Command_R = commands(4:5,:);
% 
% [fluency] = user_effort(Command_U ,Command_R, Vel_max, Omega_max);

function [fluency] = user_fluency(Command_U, Vel_max, Omega_max);
    
    Lc=length(Command_U);
    Ccount=0;
    for ii=1:Lc
        if Command_U(1,ii) || Command_U(2,ii)
            Ccount=Ccount+1;
        end
    end

    Command_U_norm(1,:) = Command_U(1,:)./Vel_max;
    Command_U_norm(2,:) = Command_U(2,:)./Omega_max;
    
    jj=1;
    for ii=2:Lc
        if Command_U(1,ii) || Command_U(2,ii)
            fluency_v(:,jj) = 1 - abs(norm(Command_U_norm(:,ii)) - norm(Command_U_norm(:,ii-1)));
            jj=jj+1;
        end
    end
    fluency = [mean(fluency_v') std(fluency_v') ];
    
end
