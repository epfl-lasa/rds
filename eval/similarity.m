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
% [heading,Similarity ,Contribution] = similarity(Command_U ,Command_R, Vel_max, Omega_max);

function [heading,disagreement ,Contribution] = similarity(Command_U ,Command_R, Vel_max, Omega_max)
    
    Lc=length(Command_U);
    Ccount=0;
    for ii=1:Lc
        if Command_U(1,ii) || Command_U(2,ii)
            Ccount=Ccount+1;
        end
    end

    Command_R_norm(1,:) = Command_R(1,:)./Vel_max;
    Command_R_norm(2,:) = Command_R(2,:)./Omega_max;
    Command_U_norm(1,:) = Command_U(1,:)./Vel_max;
    Command_U_norm(2,:) = Command_U(2,:)./Omega_max;
    
    Command_diff = abs(Command_R_norm - Command_U_norm);    
    heading = mean(Command_diff(2,:));
    disagreement = sum(vecnorm(Command_R_norm - Command_U_norm)) / Ccount; % L2-Norm difference
    
    jj=1;
    for ii=1:Lc
        if Command_U(1,ii) || Command_U(2,ii)
            if Command_U(1,ii)==0
                Contribution(jj) = (Command_U_norm(2,ii) - Command_R_norm(2,ii)) ./ Command_U_norm(2,ii);
            elseif Command_U(2,ii)==0
                Contribution(jj) = (Command_U_norm(1,ii) - Command_R_norm(1,ii)) ./ Command_U_norm(1,ii);
            else
%                 Contribution_V(jj) = norm((Command_U_norm(:,ii) - Command_R_norm(:,ii)) ./ (Command_U_norm(:,ii));
                  Contribution(jj) = norm((Command_U_norm(:,ii) - Command_R_norm(:,ii))) ./ norm(Command_U_norm(:,ii));
            end 
            jj=jj+1;
            if jj>38
                break;
            end
        end
    end
%     Contribution = sum(Contribution_V) / length(Contribution_V);
    
end
