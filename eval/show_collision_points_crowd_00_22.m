load('2019-11-29-09-00-22.bag_collision_points_log.mat')
% D14 Picture from GH011908 @ 8:15

% sync
% GH011908 @ 1:53
% corresponds to
% new_00050.mp4 @ 0:11

% new_00050.mp4 @ 0:37
% corresponds to
% this replay @ 39.98

%play_time_steps = zeros(length(time)-1, 1);
i_start = find(time> 2.98, 1);
for i = i_start:length(collision_points)
    t1 = clock;
    plot(collision_points{i}(1,:), collision_points{i}(2,:), '.')
    xlim([-5,5])
    ylim([-5,5])
    daspect([1 1 1])
    title(sprintf("time: %.2f", time(i)))
    t2 = clock;
    if i < length(collision_points)
        t_remaining = time(i+1) - time(i) - etime(t2, t1);
        pause(t_remaining);
        %play_time_steps(i) = etime(clock, t1);
    end
end

% figure
% hold on
% plot(diff(time))
% plot(play_time_steps)