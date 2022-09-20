function [] = visualize_traj(t, y, u, r, l,m_base,m_link, x_wall)
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
legend("$\theta$",'Interpreter','latex')

% Actuation Plots
control = zeros(4,length(y));
for i = 1:length(control)
    control(:,i) = u(y(i,:)');
end

figure;
subplot(2,1,1)
plot(t, control(1:2,:))
grid on;
legend(["$u_1$", "$u_2$"], "Interpreter","latex")
xlabel("t[s]")
ylabel("Rotor Force [N]")

subplot(2,1,2)
plot(t, control(3:4,:))
grid on;
legend(["$f_1$","$f_2$"], "Interpreter","latex")
xlabel("t[s]")
ylabel("Tendon Tension [N]")

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
ee_path = animatedline("Color", [0.8500 0.3250 0.0980]);

force_arrow = animatedline("Color",'r','LineStyle','-');


grid on;
xline(x_wall);

% Draw wall region
patch([x_wall,x_wall,x_wall + 2,x_wall+2],...
    [min([q_base(:,1) - 1; q_base(:,2) - 1]),...
    max([q_base(:,1) + 1; q_base(:,2) + 1]),...
    max([q_base(:,1) + 1; q_base(:,2) + 1]),...
    min([q_base(:,1) - 1; q_base(:,2) - 1])],...
    [211,211,211]./255, "FaceAlpha", 0.8);

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

% EE - path
ee = zeros(2,length(y));

for i = 1:length(y)
    % Clear the plot from previous points
    clearpoints(copter)
    clearpoints(manip)
    clearpoints(manip_joints)
    clearpoints(force_arrow)

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

    % Store the EE position
    ee(:,i) = cummulative_translation;
    

    % Paths
    addpoints(base_path,q_base(i,1), q_base(i,2))
    addpoints(ee_path, ...
        cummulative_translation(1), cummulative_translation(2))
    
    % Force Vector
    force = contact_dynamics([q_base(i,:),q_mani(i,:)],...
                         [q_dot_base(i,:),q_dot_base(i,:)],...
                         m_base, m_link,r,l,x_wall);
    force_vector = ee(:,i) + force;


    addpoints(force_arrow, [ee(1,i);force_vector(1)],...
                           [ee(2,i);force_vector(2)]);
    if norm(force) > 0.0
        addpoints(force_arrow, ...
            [force_vector(1)+sign(force_vector(1))*0.025;...
            force_vector(1)+sign(force_vector(1))*0.025; ...
            force_vector(1)], ...
            [force_vector(2)+0.025;...
            force_vector(2)-0.025; ...
            force_vector(2)]);
    end

    % Draw everything
    drawnow

    % Grab every 4th frame for the video creation => 25 fps
    if mod(i,4) == 0
        frame = getframe(gcf); %get frame
        writeVideo(video, frame);
    end
end

close(video);

figure;
hold all;
grid on;
xlabel("t[s]")
ylabel("[m]")
plot(t, ee(1,:));
plot(t, ee(2,:));
yline(x_wall,':');
legend(["$x_{EE}$","$y_{EE}$"],"Interpreter","latex");


figure;
hold all;
grid on;
xlabel("t[s]")
ylabel("[deg]")
plot(t, 180/pi*q_mani(:,1));
plot(t, 180/pi*q_mani(:,2));
plot(t, 180/pi*q_mani(:,3));
plot(t, 180/pi*q_mani(:,4));
legend(["$q_1$","$q_2$","$q_3$","$q_4$"],"Interpreter","latex");

end

