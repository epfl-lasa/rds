load('commands_log.mat')
figure
subplot(2,1,1)
hold on
plot(result(1,:), result(2,:), 'k.-') % nominal linear
plot(result(1,:), result(4,:), 'r.-') % corrected linear
legend('Nominal', 'Modified')
title("Linear Command")
subplot(2,1,2)
hold on
plot(result(1,:), result(3,:), 'k.-') % nominal angular
plot(result(1,:), result(5,:), 'r.-') % corrected angular
legend('Nominal', 'Modified')
title("Angular Command")