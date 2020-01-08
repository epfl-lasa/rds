%load('commands_log.mat')
figure
subplot(2,1,1)
hold on
plot(commands(1,:), commands(2,:), 'k.-') % nominal linear
plot(commands(1,:), commands(4,:), 'r.-') % corrected linear
legend('Nominal', 'Modified')
title("Linear Command")
subplot(2,1,2)
hold on
plot(commands(1,:), commands(3,:), 'k.-') % nominal angular
plot(commands(1,:), commands(5,:), 'r.-') % corrected angular
legend('Nominal', 'Modified')
title("Angular Command")