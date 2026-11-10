clc
clear all
close all

%car-like robot parameters
x0=1;y0=0;theta0=0;%initial position of the old reference point
l=0.1;d=0.05;%robot parameter
xref0=x0+d*cos(theta0);yref0=y0+d*sin(theta0);%intial position of the new reference point

%APF strategy parameters
step=0.1;%step length of the robot movement
d0=0.5;%safety distance from the obstacle
ds=0.5; %sensing range
k1=1;k2=0.02;%coefficient of forces

new_trajectory_generation;  %calculate the new trajectory of the robot

%correcting errors
x=x0;y=y0;theta=theta0;%initial position and heading angle of the new reference point
t0=t_store(1);
x_store1=x;%data storage
y_store1=y;
theta_store1=theta;
t_store1=t0;
u1_store1=0;
phi_store1=0;
Error_lastx=0;Errorx=0;Error_derivativex=0;Error_integralx=0;%pid definition,x dimension
Error_lasty=0;Errory=0;Error_derivativey=0;Error_integraly=0;%pid definition,y dimension
Errorx_store1=Errorx;
Errory_store1=Errory;
kpx=1;kix=1;kdx=0.5;kpy=1;kiy=1;kdy=0.5;


for count=1:(length(t_store)-1)
    t=t_store(count);

    %%%
    vxd1=vx_store(count);%set desired robot reference point speed
    vyd1=vy_store(count);

    %reference point current position
    xref=x+d*cos(theta);
    yref=y+d*sin(theta);
    %reference point desired position according to trajectory generation
    xrefd=x_store(count);
    yrefd=y_store(count);
    %error update
    Errorx=xrefd-xref;
    Errory=yrefd-yref;
    %feedback
    Error_integralx=Error_integralx+dt*Errorx;
    Error_integraly=Error_integraly+dt*Errory;
    Error_derivativex=(Errorx-Error_lastx)/dt;
    Error_derivativey=(Errory-Error_lasty)/dt;

    vxd2=kpx*Errorx+kix*Error_integralx+kdx*Error_derivativex;
    vyd2=kpy*Errory+kiy*Error_integraly+kdy*Error_derivativey;
    %overall control=feedforward vxd1+feedback vxd2;
    vxd=vxd1+vxd2;
    vyd=vyd1+vyd2;

    v=[vxd;vyd];%calculate the corresponding control input u1, phi
    A=[cos(theta),-d*sin(theta)/l;...
        sin(theta),d*cos(theta)/l];
    u=inv(A)*v;
    u1=u(1);
    u2=u(2);
    phi=atan(u2/u1);
    u1=u1+0.1*sin(0.1*t);       %error added
    phi=phi+0.1*sin(0.1*t);     %error added
    x=x+dt*u1*cos(theta);%robot kinematics
    y=y+dt*u1*sin(theta);
    theta=theta+dt*u1*tan(phi)/l;
    %%%
    %%%
    x_store1=[x_store1;x];%store latest data
    y_store1=[y_store1;y];
    theta_store1=[theta_store1;theta];
    t_store1=[t_store1;t];
    u1_store1=[u1_store1;u1];
    phi_store1=[phi_store1;phi];
    Errorx_store1=[Errorx_store1;Errorx];
    Errory_store1=[Errory_store1;Errory];
    Error_lastx=Errorx;
    Error_lasty=Errory;
end
% end of correcting errors script


%calculating new waypoints
waypoints_old=[x_store1,y_store1];

number_oldwaypoints=length(waypoints_old);%counts how many waypoints you have madeare made
waypoints_new = waypoints_old(1:7:end,:); %reduce the size of the waypoints
                                           %otherwise the simulation would
                                           %take mlonger

numways_new=length(waypoints_new);%counts reduced size of waypoints
plot(waypoints(:,1),waypoints(:,2),'rx');%plots your waypoints

% Enabling input of obstacles
obstacles=[0,0; 0,1; 0,2; 0,3; 0,4; 0,5; 0,6; 0,7; 0,8; 0,9;...
           0,10; 10,10; 10,9; 10,8; 10,7; 10,6; 10,5; 10,4; 10,3;...
           10,2; 10,1; 10,0; 1,10; 2,0; 2,10; 3,10; 4,10; 5,10; 6,10;...
           7,10; 8,10; 9,10; 2,1; 2,2; 10,7; 9,7; 7,7; 6,7;...
           5,7; 10,9; 9,5; 7,5; 6,5; 5,5; 10,3; 9,3; 5,3];

num_obs=length(obstacles(:,1)); %calculate how many obstacles are in the environment

% Producing the obstacles on the figure
for i=1:num_obs
figure(1),hold on,
circle(obstacles(i,:),d0);
end


% Robot movement
n=0;count0=1500;%count0 to limit itinerations
z=2;
robot=waypoints_new(1,:);%set the robot's starting position to be the first waypoint

while(n<count0)
    target=waypoints_new(z,:);%set the target to the zth waypoint
    n=n+1;
    % Setting initial forces w/out obstacles
    force_goal=-k1*(robot-target);
    force_obs=0;

    % Obstacle forces
    % First, we need to identify the closest obstacle
    for i=1:num_obs
     dis_obs=sqrt((obstacles(i,1)-robot(1)).^2+(obstacles(i,2)-robot(2)).^2);
        if i==1 % Making first obstacle the closest
            dis_min=dis_obs; % distance to nearest obstacle
            num_min=1;
        else
         if (dis_obs<dis_min) % if a new obstacle is closer than prev. obstacle
            dis_min=dis_obs; % new obstacle is closer
            num_min=i; % identifying the number associated with the closest obstalce
         end
       end
    end

    % calculating the obstacle force
    force_obs=k2*(robot-obstacles(num_min,:))/((dis_min^2-d0^2)^2);
    % total force
    force=force_goal+force_obs;
    % new robot position
    robot=robot+force/norm(force)*step;
    % plotting robot position
    figure(1),hold on,
    plot(robot(1),robot(2),'bo');
    plot([robot(1),obstacles(num_min,1)],[robot(2),obstacles(num_min,2)],'cyan--');
    plot(x_store,y_store);
    plot(waypoints(:,1),waypoints(:,2),'x')
    axis equal,pause(0.01);
    title("Robot's Trajectory with Obstacles");
    xlabel('x-axis')
    ylabel('y-axis')



    if norm(target-robot) < 0.01% if the distance to the waypoint is less than 0.01
        z=z+5;  %set the new zth waypoint after 5 waypoints
    else
        z=z+1;%if not, increase z by 1 so that the next waypoint is now the target
    end

    % Ending loop when robot reaches destination
    if z == numways_new %if z is equal to the number of wapoints then stop the simulation
            break
    end
end

for i=1:num_obs
figure(2),hold on,
circle(obstacles(i,:),d0);
end
figure(2),hold on,
plot(x_store1,y_store1);
plot(waypoints(:,1),waypoints(:,2),'x')
axis equal,pause(0.01);
title('Trajectory with Obstacles');
xlabel('x-axis')
ylabel('y-axis')

plot_everything;    %call function to plot all the graphs
