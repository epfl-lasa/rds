clc
clear all
close all


load('2019-11-28-07-24-42.bag_commands_log.mat')
time = commands(1,:);
i_start = find(time>458, 1);
i_end = i_start + 100;

position = zeros(2,i_end-i_start+2);
angle = zeros(2,i_end-i_start+2);
for i = 2:(i_end-i_start+2)
    position(:,i) = position(:,i-1) + (time(i_start+i-1)-time(i_start+i-2))*...
       commands(4, i_start+i-2)*[cos(angle(i-1)); sin(angle(i-1))];
    angle(i) = angle(i-1) + (time(i_start+i-1)-time(i_start+i-2))*commands(5, i_start+i-2);
end

plot(position(1,:), position(2,:))

hold on
load('2019-11-28-07-24-42.bag_collision_points_log.mat')
j = 1;
k = 1;
for i = i_start:i_end
    tf_collision_pts = (ones(length(collision_points{i}(1,:)),1)*position(:,k)')' + ...
        [cos(angle(k)-pi/2), -sin(angle(k)-pi/2);sin(angle(k)-pi/2),cos(angle(k)-pi/2)]*collision_points{i}(1:2,:);
     plot(tf_collision_pts(1,:), tf_collision_pts(2,:), 'r.', 'MarkerSize',10)
     plot(tf_collision_pts(1,:), tf_collision_pts(2,:), 'go', 'MarkerSize',2)
     plot(position(1,k), position(2,k), 'ko', 'MarkerSize',5)
     xlim([-4,8])
     ylim([-8,4])
     daspect([1 1 1])
     pause(0.25)
     j = j*1.2;
     k = k + 1;
end

%% Reading Log files
    nfig = 2;

    Omega_max = 4.124 /4;
    Vel_max = 1;
    Command_U = commands(2:3,:);
    Command_R = commands(4:5,:);

    [heading,disagreement ,Contribution] = similarity(Command_U ,Command_R, Vel_max, Omega_max);

    mean(Contribution)
    std(Contribution)

    figure(nfig)
    subplot(2,1,1),plot(time,Command_U(1,:));
    hold on;
    grid on;
    subplot(2,1,1),plot(time,Command_R(1,:))
    title('Linear Speed')
    
    subplot(2,1,2),plot(time,Command_U(2,:));
    hold on;
    grid on;
    subplot(2,1,2),plot(time,Command_R(2,:))
    title('Angular Speed')

