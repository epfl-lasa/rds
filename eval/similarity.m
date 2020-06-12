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
% [linear_diff,heading,Similarity ,Contribution] = similarity(Command_U ,Command_R, Vel_max, Omega_max);

function [linear_diff,directional_agreement, disagreement ,Contribution] = similarity(Command_U ,Command_R, Vel_max, Omega_max)
    
    Command_R_norm(1,:) = Command_R(1,:)./Vel_max;
    Command_R_norm(2,:) = Command_R(2,:)./Omega_max;
    Command_U_norm(1,:) = Command_U(1,:)./Vel_max;
    Command_U_norm(2,:) = Command_U(2,:)./Omega_max;
    
    angle_U = atan2(Command_U(1,:),Command_U(2,:));
    angle_R = atan2(Command_R(1,:),Command_R(2,:));
    angle_diff = angle_R - angle_U;
    Lc=length(Command_U);
    Ccount=0;
    for ii=1:Lc
        if Command_U(1,ii) || Command_U(2,ii)
            Ccount=Ccount+1;
            Command_diff(:,Ccount) = abs(Command_R_norm(:,ii) - Command_U_norm(:,ii));    
            agreement_vec(:,Ccount) = 1 - ( abs(angle_diff(ii)) /pi );
            
        end
    end
    directional_agreement = [mean(agreement_vec); std(agreement_vec)];
    
    linear_diff = [mean(Command_diff(1,:)); std(Command_diff(1,:))];
    heading = [mean(Command_diff(2,:)); std(Command_diff(2,:))];
%     disagreement(1,1) = sum(vecnorm(Command_R_norm - Command_U_norm)) / Ccount; % L2-Norm difference
    
    disagreement = [mean(vecnorm(Command_diff)); std(vecnorm(Command_diff))];
    
    
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

        end
    end
%     Contribution = sum(Contribution_V) / length(Contribution_V);
    
end
