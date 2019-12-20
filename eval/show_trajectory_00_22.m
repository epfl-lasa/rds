load('2019-11-29-09-00-22.bag_commands_log.mat')

i_start = 1341;
i_end = i_start + 20;

position = zeros(2,i_end-i_start+2);
angle = zeros(2,i_end-i_start+2);
for i = 2:(i_end-i_start+2)
    position(:,i) = position(:,i-1) + (time(i_start+i-1)-time(i_start+i-2))*...
       commands(4, i_start+i-2)*[cos(angle(i-1)); sin(angle(i-1))];
    angle(i) = angle(i-1) + (time(i_start+i-1)-time(i_start+i-2))*commands(5, i_start+i-2);
end

plot(position(1,:), position(2,:))

hold on
load('2019-11-29-09-00-22.bag_collision_points_log.mat')
j = 1;
k = 1;
for i = i_start:i_end
    tf_collision_pts = (ones(length(collision_points{i}(1,:)),1)*position(:,k)')' + ...
        [cos(angle(k)-pi/2), -sin(angle(k)-pi/2);sin(angle(k)-pi/2),cos(angle(k)-pi/2)]*collision_points{i}(1:2,:);
     plot(tf_collision_pts(1,:), tf_collision_pts(2,:), 'r.', 'MarkerSize',j+10)
     plot(tf_collision_pts(1,:), tf_collision_pts(2,:), 'go', 'MarkerSize',2)
     plot(position(1,k), position(2,k), 'ko', 'MarkerSize',5)
     xlim([-4,8])
     ylim([-8,4])
     daspect([1 1 1])
     pause(0.25)
     j = j*1.2;
     k = k + 1;
end