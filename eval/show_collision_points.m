load('collision_points_log.mat')
for i = 1:length(collision_points)
    plot(collision_points{i}(1,:), collision_points{i}(2,:), '.')
    xlim([-5,5])
    ylim([-5,5])
    daspect([1 1 1])
    title(sprintf("time: %.2f", time(i)))
    pause(0.25)
end