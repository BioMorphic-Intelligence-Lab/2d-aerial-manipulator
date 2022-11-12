function [] = visualize_traj(logger,f_des,r,l,walls,u_max)
%VISUALIZE_TRAJ Function that visualizes the simulation

%% Preparation
% Consistent LineWidth (lw) and FontSize (fs)
lw = 2;
fs = 24;

% Rotation matrices
rot = @(theta) [cos(theta), -sin(theta);
                sin(theta), cos(theta)];

% Extract states
q_base = logger.q(1:3,:);
q_mani = logger.q(4:7,:);
q_dot_base = logger.qdot(1:3,:);
q_dot_mani = logger.qdot(4:7,:);

%% Actuation Plots
figure;
subplot(2,1,1)
plot(logger.t, logger.u(1:2,:),"LineWidth",lw)
grid on;
legend(["$u_1$", "$u_2$"], "Interpreter","latex",'FontSize', fs)
xlabel("t[s]",'FontSize', fs)
ylabel("Rotor Force [N]",'FontSize', fs)

subplot(2,1,2)
plot(logger.t, logger.u(3:4,:), "LineWidth",lw)
grid on;
legend(["$f_1$","$f_2$"], "Interpreter","latex",'FontSize', fs)
xlabel("t[s]",'FontSize', fs)
ylabel("Tendon Tension [N]",'FontSize', fs)

%% Individual Coordinates Plot
figure;
subplot(2,1,1)
hold all;
plot(logger.t, q_base(1:2,:),"LineWidth",lw);
xlabel("t[s]",'FontSize', fs)
ylabel("[m]",'FontSize', fs)
grid on;
legend(["$x$", "$y$"],'Interpreter','latex','FontSize', fs)

subplot(2,1,2)
hold all;
plot(logger.t, 180 / pi .* q_base(3,:),"LineWidth",lw, "Color", "#0072BD");
xlabel("t[s]",'FontSize', fs)
ylabel("[deg]",'FontSize', fs)
grid on;
legend("$\theta$",'Interpreter','latex','FontSize', fs)


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

thrust_arrow_1 = animatedline("LineWidth",lw,"DisplayName","$\mathbf{u}_{1}$");
thrust_arrow_2 = animatedline("LineWidth",lw,"DisplayName","$\mathbf{u}_{2}$");

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
      max([q_base(:,1) + 2; q_base(:,2) + 2])]);
ylim([min([q_base(:,1) - 0.5; q_base(:,2) - 0.5]),...
      max([q_base(:,1) + 0.5; q_base(:,2) + 0.5])])
xlabel("x[m]",'FontSize', 1.5*fs)
ylabel("y[m]",'FontSize', 1.5*fs)

% Set Legend of selected lines
f=get(gca,'Children');
legend([f(2); f(1); f(5:7)],"Interpreter","latex",'FontSize', 1.5*fs,"Location","northwest");

% Fix Aspect Ratio to Equal
daspect([1 1 1])

video = VideoWriter('trajectory'); %open video file
video.Quality = 100;
video.FrameRate = 25; 
open(video);

% EE - path
ee = zeros(2,length(logger.t));
for i =1:length(logger.t)
    cummulative_rotation = rot(q_base(3, i));
    cummulative_translation = q_base(1:2, i);    
    
    % Manipulator Links
    for k =1:4
        cummulative_rotation = cummulative_rotation * rot(q_mani(k,i));
        moved_link = cummulative_rotation * link + cummulative_translation;
        
        addpoints(manip, moved_link(1,:), moved_link(2,:));
        addpoints(manip_joints, moved_link(1,1), moved_link(2,1))

        cummulative_translation = cummulative_translation ...
                                  + cummulative_rotation * link(:,2);
    end

    % Store the EE position
    ee(:,i) = cummulative_translation;
end

% Force Vector
force = zeros(2,length(logger.t));

for i = 1:100:length(logger.t)
    % Clear the plot from previous points
    clearpoints(copter)
    clearpoints(manip)
    clearpoints(manip_joints)
    clearpoints(force_arrow)
    clearpoints(thrust_arrow_1)
    clearpoints(thrust_arrow_2)
    clearpoints(target)

    % Add New Points
    addpoints(target,logger.ref(1,i), logger.ref(2,i))

    % Base Platfom
    moved_bar = rot(q_base(3,i)) * bar + q_base(1:2,i);
    assert(abs(norm(moved_bar(:,1) - moved_bar(:,2)) - 0.2) < 0.01,...
           "Bar was deformed. Length: " ...
           + num2str(norm(moved_bar(:,1) - moved_bar(:,2))))

    addpoints(copter, moved_bar(1,:), moved_bar(2,:))

    cummulative_rotation = rot(q_base(3, i));
    cummulative_translation = q_base(1:2, i);    
    
    % Manipulator Links
    for k =1:4
        cummulative_rotation = cummulative_rotation * rot(q_mani(k,i));
        moved_link = cummulative_rotation * link + cummulative_translation;
        
        addpoints(manip, moved_link(1,:), moved_link(2,:));
        addpoints(manip_joints, moved_link(1,1), moved_link(2,1))

        cummulative_translation = cummulative_translation ...
                                  + cummulative_rotation * link(:,2);
    end

    

    % Paths
    addpoints(base_path,q_base(1,i), q_base(2,i))
    addpoints(ee_path, ee(1,i), ee(2,i))
    
    % Force Vector
    force(:,i) = logger.fext(:,i);
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

    % Add thrust arrows
    arrow1 = [moved_bar(:,2), moved_bar(:,2) + rot(q_base(3,i))*([0;0.05])];
    arrow2 = [moved_bar(:,1), moved_bar(:,1) + rot(q_base(3,i))*([0;0.05])];
    addpoints(thrust_arrow_1,arrow1(1,:), arrow1(2,:));
    addpoints(thrust_arrow_2,arrow2(1,:), arrow2(2,:));

    % Arrow Head Offset
    offset = [0.01, -0.01;
             -0.01, -0.01];
    offset = rot(q_base(3,i))*offset;
        
    % Arrow Heads
    arrow_head1 =  [
        arrow1(1,2)+offset(1,1),...
        arrow1(1,2)+offset(1,2), ...
        arrow1(1,2); ... % x
        arrow1(2,2)+offset(2,1),...
        arrow1(2,2)+offset(2,2),...
        arrow1(2,2)];
    thrust_arrow_1.Color = [0.0 1.0 0.0] + logger.u(1,i) / u_max.*[1 -1 0];

    arrow_head2 =  [
        arrow2(1,2)+offset(1,1),...
        arrow2(1,2)+offset(1,2), ...
        arrow2(1,2); ... % x
        arrow2(2,2)+offset(2,1),...
        arrow2(2,2)+offset(2,2),...
        arrow2(2,2)];
    thrust_arrow_2.Color = [0.0 1.0 0.0] + logger.u(2,i) / u_max.*[1 -1 0];

    addpoints(thrust_arrow_1,arrow_head1(1,:),arrow_head1(2,:));
    addpoints(thrust_arrow_2,arrow_head2(1,:),arrow_head2(2,:));

    % Draw everything
    drawnow

    % Grab every 4th frame for the video creation => 25 fps
    frame = getframe(gcf); %get frame
    writeVideo(video, frame);
end

close(video);

%% Plot EE Path
figure;
hold all;
grid on;
xlabel("t[s]",'FontSize', 1.5*fs)
ylabel("[m]",'FontSize', 1.5*fs)
plot(logger.t, ee(1,:),"LineWidth",lw, "Color", "#0072BD");
plot(logger.t, ee(2,:),"LineWidth",lw, "Color", "#D95319");
plot(logger.t, logger.ref(1,:), "LineWidth",lw,"Color","#0072BD","LineStyle","--");
plot(logger.t, logger.ref(2,:), "LineWidth",lw,"Color","#D95319","LineStyle","--");
yline(walls(2:2:end,1),':',"LineWidth",lw);
legend(["$x_{EE}$","$y_{EE}$","$x_{Ref}$","$y_{Ref}$"],...
    "Interpreter","latex",'FontSize', 1.5*fs, "Location","northwest");

%% Plot Joint Variables
figure;
hold all;
grid on;
xlabel("t[s]",'FontSize', fs)
ylabel("[deg]",'FontSize', fs)
plot(logger.t, 180/pi*q_mani(1,:),"LineWidth",lw);
plot(logger.t, 180/pi*q_mani(2,:),"LineWidth",lw);
plot(logger.t, 180/pi*q_mani(3,:),"LineWidth",lw);
plot(logger.t, 180/pi*q_mani(4,:),"LineWidth",lw);
legend(["$q_1$","$q_2$","$q_3$","$q_4$"],"Interpreter","latex",'FontSize', fs);

%% Plot Contact force and estimated contact force
figure;
subplot(2,1,1)
hold all;
grid on;
xlabel("t[s]",'FontSize', fs)
ylabel("Force [N]",'FontSize', fs)
plot(logger.t, force(1,:), "LineWidth",lw)
plot(logger.t, logger.fext_est(1,:),"LineStyle",":","LineWidth",lw)
plot(logger.t, f_des(1)*ones(1,length(logger.t)),"LineStyle",":","Color","black","LineWidth",lw)
legend(["$f_{sim;x}$","$f_{dyn,est;x}$","$f_{des,x}$"],...
    "Interpreter", "latex",'FontSize', fs)

subplot(2,1,2)
hold all;
grid on;
xlabel("t[s]",'FontSize', fs)
ylabel("Force [N]",'FontSize', fs)
plot(logger.t, force(2,:), "LineWidth",lw)
plot(logger.t, logger.fext_est(2,:),"LineStyle",":","LineWidth",lw)
plot(logger.t, f_des(2)*ones(1,length(logger.t)),"LineStyle",":","Color","black","LineWidth",lw)
legend(["$f_{sim;y}$","$f_{dyn,est;y}$","$f_{des,y}$"],...
    "Interpreter", "latex",'FontSize', fs)


end

