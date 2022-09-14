function [] = visualize_traj(t, y, u)
%VISUALIZE_TRAJ Summary of this function goes here
%   Detailed explanation goes here

% Individual Coordinates Plot
figure;
subplot(2,1,1)
hold all;
plot(t, y(:,4:5));
xlabel("t[s]")
ylabel("[m]")
grid on;
legend(["$x$", "$y$"],'Interpreter','latex')

subplot(2,1,2)
plot(t, 180 / pi .* y(:,6));
xlabel("t[s]")
ylabel("[deg]")
grid on;
legend(["$\theta$"],'Interpreter','latex')

% Actuation Plots
control = zeros(2,length(y));
for i = 1:length(control)
    control(:,i) = u(y(i,:)');
end

figure;
plot(t, control)
grid on;
legend(["u1", "u2"])


% In plane visualization
figure;
plot(y(:,4),y(:,5))
grid on;
xlim([min(y(:,4)) - 0.2, max(y(:,4) + 0.2)])
ylim([min(y(:,5)) - 0.2, max(y(:,5) + 0.2)])
daspect([1 1 1])
xlabel("x[m]")
ylabel("y[m]")

rot = @(theta) [cos(theta), -sin(theta);
                sin(theta), cos(theta)];

% In plane animation
figure;
bar = 0.1.*[-1, 1; % x
             0, 0]; % y
for i = 1:length(y)
    moved_bar = rot(y(i, 6)) * bar + y(i, 4:5)';
    plot(moved_bar(1,:), moved_bar(2,:))
    grid on;
    xlim([min(y(:,4)) - 0.2, max(y(:,4) + 0.5)])
    ylim([min(y(:,5)) - 0.2, max(y(:,5) + 0.5)])
    xlabel("x[m]")
    ylabel("y[m]")
    pause(0.01)
end

end

