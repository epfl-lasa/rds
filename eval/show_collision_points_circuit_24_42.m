load('2019-11-28-07-24-42.bag_collision_points_log.mat')

% D14 picture GH011902 @ 0:56

% new_00043.mp4 @ 0:05
% corresponds roughly to
% GH011902.mp4 @ 0:13

% new_0043.mp4 @ 0:16
% corresponds to
% this replay @ 426.16

%play_time_steps = zeros(length(time)-1, 1);
i_start = find(time>410, 1);
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
