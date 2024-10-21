% Robotics Assignment-1
% references https://github.com/jbramburger/Data-Science-Methods/blob/879e32fb0a9f608af5892db5e204053c529ddf81/Code/Lecture_28.m#L66
% System Parameters
m1=1; 
m2=1;  
a1=1; 
a2=1; 
g=10;          
k1=1; 
k2=1;  
learning_rate=200000; % a value above 90k gives a decent overall result

%Duration
start=0;
stop=10;
step=0.05;
t=start:step:stop;

% Trajectory
x0=[0,0,0,0];
function [qd,vd]=desired_trajectory(t)
    qd=[pi/3;pi/4];
    vd=[0;0];
    %qd=[pi/4*(sin(t));pi/5*cos(t)];
    %vd=[pi/4*(cos(t));pi/5*(-1)*(sin(t))]
end

% Adaptive PD Controller
function [tau,k1, k2]=adaptive_pd_controller(q,v,qd,vd,k1,k2,learning_rate)
    e=q-qd;
    e_v=v-vd;
    %the act formulae for adaptive pd control acc. to google
    % k1=k1+learning_rate*(e_v'*e');
    % k2=k2+learning_rate*(e'*e_v);
    k1=k1+learning_rate*(e*e');
    k2=k2+learning_rate*(e_v'*e_v);
    tau=-k1*e-k2*e_v;
end

% System motion
function dyn=motion(t,x,m1,m2,a1,a2,g,k1,k2,learning_rate)
    q=x(1:2);
    v=x(3:4);
    [qd,vd]=desired_trajectory(t);
    [tau,~,~]=adaptive_pd_controller(q,v,qd,vd,k1,k2,learning_rate);
    M=[(m1+m2)*a1^2+m2*a2^2+2*m2*a1*a2*cos(q(2)),m2*a2^2+m2*a1*a2*cos(q(2));m2*a2^2+m2*a1*a2*cos(q(2)),m2*a2^2];
    V=[-m2*a1*a2*(2*v(1)*v(2)+v(2)^2)*sin(q(2));m2*a1*a2*v(1)^2*sin(q(2))];
    G=[(m1+m2)*g*a1*cos(q(1))+m2*g*a2*cos(q(1)+q(2));m2*g*a2*cos(q(1)+q(2))];
    a=M\(tau-V-G); %Ma+V+G=tau
    dyn=[v;a];
end

% DE Solver (ode79 and such give the same result to a T, but take longer)
[t,X]=ode45(@(t,x) motion(t,x,m1,m2,a1,a2,g,k1,k2,learning_rate),t,x0);
q1=X(:,1);
q2=X(:,2);
v_act = X(:,3:4);

% Forward Kinematics
function [x,y]=fk(q1,q2, a1, a2)
    x1=a1*cos(q1);
    y1=a1*sin(q1);
    x2=x1+a2*cos(q1+q2);
    y2=y1+a2*sin(q1+q2);
    x=[0,x1,x2];
    y=[0,y1,y2];
end

% Dynamic Graphing
% Video doesn't work cause it messes with the drawnow feature afaik
% video_a=VideoWriter('video_a');
% video_a.FrameRate=5;
% video_b=VideoWriter('video_b');
% video_b.FrameRate=5;
qd_pos=zeros(length(t),2);
vd_pos=zeros(length(t),2);
for i = 1:length(t)
    [qd_temp, vd_temp] = desired_trajectory(t(i));
    qd_pos(i,:) = qd_temp';
    vd_pos(i,:) = vd_temp';
end

figure('Position', [100 100 1400 900], 'Name', 'Animation');
for i=1:10:length(t)
    [x,y]=fk(q1(i),q2(i),a1,a2);
    [xd,yd]=fk(qd_pos(i,1),qd_pos(i,2),a1,a2);

    % Robot Visualization
    subplot(1,3,1);
    plot(x,y,'b-','LineWidth',4); 
    hold on;
    plot(x,y,'ko','MarkerFaceColor','k','MarkerSize',10);
    plot(xd, yd,'g','LineWidth',2);
    plot(xd, yd,'ko','MarkerFaceColor','k','MarkerSize',10);
    % pause(step);
    % frame=getframe(gcf);
    % writeVideo(video_a,frame);
    % writeVideo(video_b,frame);
    axis equal;
    axis([-3,3,-3,3,-3,3]);
    title('Two Link Planar Elbow Arm');
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    % view(3);
    legend('Current Position','Joints','Desired Position','Desired Joints','Location','southwest');
    hold off;
    
    % Joint 1 angle
    subplot(1,3,2);
    plot(t(1:i),q1(1:i),'b',t(1:i),qd_pos(1:i,1),'g');
    axis([0,10,0,3,0,10]);
    title('Joint 1 Angle');
    xlabel('Time (s)');
    ylabel('Angle (radian)');
    legend('Actual','Desired');
    
    % Joint 2 angle
    subplot(1,3,3);
    plot(t(1:i),q2(1:i),'b',t(1:i),qd_pos(1:i,2),'g');
    axis([0,10,0,3,0,10]);
    title('Joint 2 Angle');
    xlabel('Time (s)');
    ylabel('Angle (radian)');
    legend('Actual','Desired');
    drawnow;

end
% close(video_a);

% 3 Graphs per joint
figure('Position',[100 100 1400 900],'Name','Errors and Torque with time');
q_error=[q1-qd_pos(:,1),q2-qd_pos(:,2)];
v_error=[v_act(:,1)-vd_pos(:,1),v_act(:,2)-vd_pos(:,2)];
torque_act=zeros(length(t),2);
for i=1:length(t)
    q=X(i,1:2)';
    v=X(i,3:4)';
    [qd_temp,vd_temp]=desired_trajectory(t(i));
    [tau,~,~]=adaptive_pd_controller(q,v,qd_temp,vd_temp,k1,k2,learning_rate);
    torque_act(i,:)=tau';
end
disp(torque_act)
% Position error plots
subplot(3,2,1);
plot(t, q_error(:,1));
title('Joint 1 Position Error');
xlabel('Time');
ylabel('Error');

subplot(3,2,2);
plot(t, q_error(:,2));
title('Joint 2 Position Error');
xlabel('Time');
ylabel('Error');

% Velocity Errors
subplot(3,2,3);
plot(t,v_error(:,1));
title('Joint 1 Velocity Error');
xlabel('Time');
ylabel('Error');

subplot(3,2,4);
plot(t, v_error(:,2));
title('Joint 2 Velocity Error');
xlabel('Time');
ylabel('Error');

% Torque
subplot(3,2,5);
plot(t, torque_act(:,1));
title('Joint 1');
xlabel('Time');
ylabel('Torque');

subplot(3,2,6);
plot(t, torque_act(:,2));
title('Joint 2 Torque Error');
xlabel('Time');
ylabel('Torque');
