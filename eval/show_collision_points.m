%load('collision_points_log.mat')

%play_time_steps = zeros(length(time)-1, 1);
for i = 1:length(collision_points)
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
