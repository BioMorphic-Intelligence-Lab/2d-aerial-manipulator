function [] = visualize_traj(t, y, p_des, r, l,m_base,m_link,r_tendon, walls)
%VISUALIZE_TRAJ Function that visualizes the simulation

%% Preparation
% Consistent LineWidth (lw) and FontSize (fs)
lw = 2;
fs = 24;

% Rotation matrices
rot = @(theta) [cos(theta), -sin(theta);
                sin(theta), cos(theta)];

% Extract states
q_base = y(:, 8:10);
q_mani = y(:, 11:14);
q_dot_base = y(:, 15:17);
q_dot_mani = y(:, 18:21);

%% Individual Coordinates Plot
figure;
subplot(2,1,1)
hold all;
plot(t, q_base(:,1:2),"LineWidth",lw);
xlabel("t[s]",'FontSize', fs)
ylabel("[m]",'FontSize', fs)
grid on;
legend(["$x$", "$y$"],'Interpreter','latex','FontSize', fs)

subplot(2,1,2)
plot(t, 180 / pi .* q_base(:,3),"LineWidth",lw);
xlabel("t[s]",'FontSize', fs)
ylabel("[deg]",'FontSize', fs)
grid on;
legend("$\theta$",'Interpreter','latex','FontSize', fs)

%% Actuation Plots
control = zeros(4,length(y));
for i = 1:length(control)
    control(:,i) = ctrl(y(i,:)', p_des(t(i)), m_base,m_link,r,l);
end

figure;
subplot(2,1,1)
plot(t, control(1:2,:),"LineWidth",lw)
grid on;
legend(["$u_1$", "$u_2$"], "Interpreter","latex",'FontSize', fs)
xlabel("t[s]",'FontSize', fs)
ylabel("Rotor Force [N]",'FontSize', fs)

subplot(2,1,2)
plot(t, control(3:4,:), "LineWidth",lw)
grid on;
legend(["$f_1$","$f_2$"], "Interpreter","latex",'FontSize', fs)
xlabel("t[s]",'FontSize', fs)
ylabel("Tendon Tension [N]",'FontSize', fs)


%% In plane animation
figure;
bar = r.*[-1, 1;  % x
           0, 0]; % y
link = l.*[0, 0;   % x
           0, -1]; % y

copter = animatedline;
manip = animatedline;
manip_joints = animatedline("Marker","o");
target = animatedline("Marker", "x", "Color","black","LineWidth",lw);

base_path = animatedline("Color","b","LineWidth",lw,...
    "DisplayName","$\mathbf{p}_{Base}$");
ee_path = animatedline("Color", [0.8500 0.3250 0.0980],"LineWidth",lw,...
    "DisplayName","$\mathbf{p}_{EE}$");

force_arrow = animatedline("Color",'r',"LineWidth",lw,'LineStyle','-',...
    "DisplayName","$\mathbf{f}_{Contact}$");

grid on;

% Draw walls region
% Line equation representation of the wall
for i=1:2:length(walls)
 if walls(i,2) == 0
    patch([walls(i+1,1), walls(i+1,1),...
           walls(i+1,1)-sign(walls(i,1)) * 2, walls(i+1,1)-sign(walls(i,1)) * 2], ...
          [min([q_base(:,1) - 0.5; q_base(:,2) - 0.5]),...
           max([q_base(:,1) + 0.5; q_base(:,2) + 0.5]),...
           max([q_base(:,1) + 0.5; q_base(:,2) + 0.5]),...
           min([q_base(:,1) - 0.5; q_base(:,2) - 0.5])],...
           [211,211,211]./255, "FaceAlpha", 0.8,...
           "EdgeColor",[211,211,211]./255, "EdgeAlpha", 0.8,...
           "DisplayName", "Wall " + string(ceil(i/2)))
else
    line = @(x) - walls(i,1)/walls(i,2) * (x - walls(i+1,1)) + walls(i+1,2);
    patch([walls(i+1,1)-2,walls(i+1,1)+4,walls(i+1,1)+4, walls(i+1,1)-2],...
        [line(walls(i+1,1)-2) - 2, line(walls(i+1,1)+4) - 2, line(walls(i+1,1)+4), line(walls(i+1,1)-4)],...
        [211,211,211]./255, "FaceAlpha", 0.8,...
        "EdgeColor",[211,211,211]./255, "EdgeAlpha", 0.8,...
        "DisplayName", "Wall " + string(ceil(i/2)));
end
end

xlim([min([q_base(:,1) - 0.5; q_base(:,2) - 0.5]),...
      max([q_base(:,1) + 0.5; q_base(:,2) + 0.5])]);
ylim([min([q_base(:,1) - 0.5; q_base(:,2) - 0.5]),...
      max([q_base(:,1) + 0.5; q_base(:,2) + 0.5])])
xlabel("x[m]",'FontSize', 1.5*fs)
ylabel("y[m]",'FontSize', 1.5*fs)

% Set Legend of selected lines
f=get(gca,'Children');
legend([f(1:5)],"Interpreter","latex",'FontSize', 1.5*fs,"Location","northwest");

% Fix Aspect Ratio to Equal
daspect([1 1 1])

video = VideoWriter('trajectory'); %open video file
video.Quality = 100;
video.FrameRate = 25; 
open(video);

% EE - path
ee = zeros(2,length(y));

% Force Vector
force = zeros(2,length(y));

for i = 1:length(y)
    % Clear the plot from previous points
    clearpoints(copter)
    clearpoints(manip)
    clearpoints(manip_joints)
    clearpoints(force_arrow)
    clearpoints(target)

    % Add New Points

    % Current Targe
    curr_target = p_des(t(i));
    addpoints(target,curr_target(1), curr_target(2))

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
    force(:,i) = contact_dynamics([q_base(i,:),q_mani(i,:)],...
                         [q_dot_base(i,:),q_dot_mani(i,:)],...
                         m_base, m_link,r,l,walls);
    force_vector = ee(:,i) + force(:,i);


    addpoints(force_arrow, [ee(1,i);force_vector(1)],...
                           [ee(2,i);force_vector(2)]);
    if norm(force) > 0.0
        % Arrow Head Offset
        offset = [0.025, -0.025;
                  -0.025, -0.025];
        
        % Find force angle
        angle = acos(dot(force(:,i)/norm(force(:,i)),[0;1]));
        offset = rot(angle)*offset;
        
        % Arrow Head
        arrow_head =  [
            force_vector(1)+offset(1,1),...
            force_vector(1)+offset(1,2), ...
            force_vector(1); ... % x
            force_vector(2)+offset(2,1),...
            force_vector(2)+offset(2,2),...
            force_vector(2)];

        addpoints(force_arrow, arrow_head(1,:), arrow_head(2,:));
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

%% Plot EE Path
figure;
hold all;
grid on;
ref = p_des(t');
xlabel("t[s]",'FontSize', 1.5*fs)
ylabel("[m]",'FontSize', 1.5*fs)
plot(t, ee(1,:),"LineWidth",lw, "Color", "#0072BD");
plot(t, ee(2,:),"LineWidth",lw, "Color", "#D95319");
plot(t, ref(1,:), "LineWidth",lw,"Color","#0072BD","LineStyle","--");
plot(t, ref(2,:), "LineWidth",lw,"Color","#D95319","LineStyle","--");
yline(walls(2:2:end,1),':',"LineWidth",lw);
legend(["$x_{EE}$","$y_{EE}$","$x_{Ref}$","$y_{Ref}$"],...
    "Interpreter","latex",'FontSize', 1.5*fs, "Location","northwest");

%% Plot Joint Variables
figure;
hold all;
grid on;
xlabel("t[s]",'FontSize', fs)
ylabel("[deg]",'FontSize', fs)
plot(t, 180/pi*q_mani(:,1),"LineWidth",lw);
plot(t, 180/pi*q_mani(:,2),"LineWidth",lw);
plot(t, 180/pi*q_mani(:,3),"LineWidth",lw);
plot(t, 180/pi*q_mani(:,4),"LineWidth",lw);
legend(["$q_1$","$q_2$","$q_3$","$q_4$"],"Interpreter","latex",'FontSize', fs);

%% Compute Estimated Contact Force
f_est = zeros(2,length(y)-1);
f_est_static = zeros(2,length(y)-1);
for i = 1:length(y)-1
    q_ddot = ([q_dot_base(i+1,:),q_dot_mani(i+1,:)] ...
            - [q_dot_base(i,:),q_dot_mani(i,:)])/(t(i+1) - t(i));
    f_est(:,i) = estimate_force([q_base(i,:),q_mani(i,:)],...
                                [q_dot_base(i,:),q_dot_mani(i,:)],...
                                q_ddot, control(:,i), ...
                                m_base,m_link,r,l,r_tendon);
    f_est_static(:,i) = estimate_force([q_base(i,:),q_mani(i,:)],...
                                zeros(7,1),zeros(7,1), control(:,i), ...
                                m_base,m_link,r,l,r_tendon);
end

%% Plot Contact force and estimated contact force
figure;
subplot(2,1,1)
hold all;
grid on;
xlabel("t[s]",'FontSize', fs)
ylabel("Force [N]",'FontSize', fs)
plot(t, force(1,:), "LineWidth",lw)
plot(t(1:end-1), f_est(1,:),"LineStyle",":","LineWidth",lw)
plot(t(1:end-1), f_est_static(1,:),"LineStyle",":","LineWidth",lw)
legend(["$f_{sim;x}$","$f_{dyn,est;x}$","$f_{static,est;x}$"],...
    "Interpreter", "latex",'FontSize', fs)

subplot(2,1,2)
hold all;
grid on;
xlabel("t[s]",'FontSize', fs)
ylabel("Force [N]",'FontSize', fs)
plot(t, force(2,:), "LineWidth",lw)
plot(t(1:end-1), f_est(2,:),"LineStyle",":","LineWidth",lw)
plot(t(1:end-1), f_est_static(2,:),"LineStyle",":","LineWidth",lw)
legend(["$f_{sim;y}$","$f_{dyn,est;y}$","$f_{static,est;y}$"],...
    "Interpreter", "latex",'FontSize', fs)


end

