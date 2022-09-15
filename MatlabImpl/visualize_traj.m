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

rot = @(theta) [cos(theta), -sin(theta);
                sin(theta), cos(theta)];

% In plane animation
figure;
bar = 0.1.*[-1, 1; % x
             0, 0]; % y

copter = animatedline;
path = animatedline("Color","b");

grid on;
xlim([min(y(:,4)) - 0.2, max(y(:,4) + 0.5)])
ylim([min(y(:,5)) - 0.2, max(y(:,5) + 0.5)])
xlabel("x[m]")
ylabel("y[m]")

video = VideoWriter('trajectory'); %open video file
video.FrameRate = 25; 
open(video)

for i = 1:length(y)
    % Clear the plot from previous points
    clearpoints(copter)

    % Add New Points
    moved_bar = rot(y(i, 6)) * bar + y(i, 4:5)';
    assert(abs(norm(moved_bar(:,1) - moved_bar(:,2)) - 0.2) < 0.01,...
           "Bar was deformed. Length: " ...
           + num2str(norm(moved_bar(:,1) - moved_bar(:,2))))

    addpoints(path,y(i,4), y(i,5))
    addpoints(copter, moved_bar(1,:), moved_bar(2,:))
    drawnow

    % Grab every 4th frame for the video creation => 25 fps
    if mod(i,4) == 0
        frame = getframe(gcf); %get frame
        writeVideo(video, frame);
    end
end

close(video)

end

