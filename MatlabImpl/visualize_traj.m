function [] = visualize_traj(t, y, u, r, l)
%VISUALIZE_TRAJ Summary of this function goes here
%   Detailed explanation goes here

% Extract states
q_base = y(:, 8:10);
q_mani = y(:, 11:14);
q_dot_base = y(:, 15:17);
q_dot_mani = y(:, 18:21);

% Individual Coordinates Plot
figure;
subplot(2,1,1)
hold all;
plot(t, q_base(:,1:2));
xlabel("t[s]")
ylabel("[m]")
grid on;
legend(["$x$", "$y$"],'Interpreter','latex')

subplot(2,1,2)
plot(t, 180 / pi .* q_base(:,3));
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
bar = r.*[-1, 1;  % x
           0, 0]; % y
link = l.*[0, 0;   % x
           0, -1]; % y

copter = animatedline;
manip = animatedline;
manip_joints = animatedline("Marker","o");

base_path = animatedline("Color","b");
ee_path = animatedline("Color", "r");

grid on;
xlim([min([q_base(:,1) - 1; q_base(:,2) - 1]),...
      max([q_base(:,1) + 1; q_base(:,2) + 1])]);
ylim([min([q_base(:,1) - 1; q_base(:,2) - 1]),...
      max([q_base(:,1) + 1; q_base(:,2) + 1])])
xlabel("x[m]")
ylabel("y[m]")
daspect([1 1 1])

video = VideoWriter('trajectory'); %open video file
video.FrameRate = 25; 
open(video);

for i = 1:length(y)
    % Clear the plot from previous points
    clearpoints(copter)
    clearpoints(manip)
    clearpoints(manip_joints)

    % Add New Points

    % Base Platfom
    moved_bar = rot(q_base(i, 3)) * bar + q_base(i, 1:2)';
    assert(abs(norm(moved_bar(:,1) - moved_bar(:,2)) - 0.2) < 0.01,...
           "Bar was deformed. Length: " ...
           + num2str(norm(moved_bar(:,1) - moved_bar(:,2))))

    addpoints(copter, moved_bar(1,:), moved_bar(2,:))
    
    cummulative_rotation = rot(q_base(i, 3));
    cummulative_translation = q_base(i, 1:2)';
    % Manipulator Links
    for k =1:4
        cummulative_rotation = cummulative_rotation * rot(q_mani(i,k));
        moved_link = cummulative_rotation * link + cummulative_translation;
        
        addpoints(manip, moved_link(1,:), moved_link(2,:));
        addpoints(manip_joints, moved_link(1,1), moved_link(2,1))

        cummulative_translation = cummulative_translation ...
                                  + cummulative_rotation * link(:,2);
    end
    

    % Paths
    addpoints(base_path,q_base(i,1), q_base(i,2))
    addpoints(ee_path, ...
        cummulative_translation(1), cummulative_translation(2))
    drawnow

    % Grab every 4th frame for the video creation => 25 fps
    if mod(i,4) == 0
        frame = getframe(gcf); %get frame
        writeVideo(video, frame);
    end
end

close(video);

end

