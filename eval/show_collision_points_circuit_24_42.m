load('2019-11-28-07-24-42.bag_collision_points_log.mat')

% D14 picture GH011902 @ 0:56

% new_00043.mp4 @ 0:05
% corresponds roughly to
% GH011902.mp4 @ 0:13

% new_0043.mp4 @ 0:16
% corresponds to
% this replay @ 426.16
if false
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
end

load('2019-11-28-07-24-42.bag_commands_log.mat')

i_start = find(time>445, 1);
i_end = find(time>470, 1);

normal_statistics = [];
tangent_statistics = [];
distance_sequence = zeros(1,1-i_start+i_end);
for i = i_start:i_end
    ref_point_y = 0.25;
    v_ref_point = [-ref_point_y*commands(5,i); commands(4,i)];
    d_min = nan;
    for j = 1:size(collision_points{i}, 2)
        d_vec = collision_points{i}(:, j) - [0; ref_point_y];
        d = sqrt(d_vec'*d_vec);
        if d > 0
            normal_statistics = [normal_statistics; [d, d_vec'*v_ref_point/d]];
            tangent_statistics = [tangent_statistics; [d, [d_vec(2);-d_vec(1)]'*v_ref_point/d]];
        end
        if ~(d > d_min)
            d_min = d;
        end
        distance_sequence(i-i_start+1) = d_min;
    end
end
figure
hold on
plot(normal_statistics(:,1), normal_statistics(:,2), 'ko')
plot(tangent_statistics(:,1), tangent_statistics(:,2), 'ro')
figure
plot(time(i_start:i_end)-time(i_start), distance_sequence)
ylim([0,2.6])
xlabel('time [s]')
ylabel('normalized shortest distance [-]')
