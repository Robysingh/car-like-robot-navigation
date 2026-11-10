%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3),clf(3),
subplot(3,1,1),plot(t_store1,x_store1),title('x (old ref) vs. time');
xlabel('time')
ylabel('x')
subplot(3,1,2),plot(t_store1,y_store1),title('y (old ref) vs. time');
xlabel('time')
ylabel('y')
subplot(3,1,3),plot(t_store1,theta_store1),title('theta vs. time');
xlabel('time')
ylabel('theta')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(4),clf(4),
plot(x_store1,y_store1),title('old ref trajectory')
hold on,
plot(waypoints(:,1),waypoints(:,2),'x')
axis equal
xlabel('x-axis')
ylabel('y-axis')
xlim([0 10])
ylim([-0.5 10])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(5),clf(5),hold on,
plot(x_store1,y_store1),
quiver(x_store1(1:50:end),y_store1(1:50:end),cos(theta_store1(1:50:end)),sin(theta_store1(1:50:end)),0.5),title('trajectory')
axis equal
xlabel('x-axis')
ylabel('y-axis')
xlim([0 10])
ylim([-0.5 10])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(6),clf(6),
subplot(2,1,1),plot(t_store1,x_store1+d*cos(theta_store1)),title('x (new ref) vs. time');
xlabel('time')
ylabel('x-axis')
subplot(2,1,2),plot(t_store1,y_store1+d*sin(theta_store1)),title('y (new ref) vs. time');
xlabel('time')
ylabel('y-axis')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(7),clf(7), hold on,
plot(x_store1+d*cos(theta_store1),y_store1+d*sin(theta_store1)),axis equal, title('Ref trajectory with generated control inputs');
hold on,
plot(waypoints(:,1),waypoints(:,2),'x')
xlabel('x-axis')
ylabel('y-axis')
xlim([0 10])
ylim([-0.5 10])
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(8),clf(8),
subplot(2,1,1),plot(t_store1,u1_store1),title('ctr input u1 vs. time');
xlabel('time')
ylabel('input u1')
subplot(2,1,2),plot(t_store1,phi_store1),title('ctr input phi vs. time');
xlabel('time')
ylabel('input phi')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(9),clf(9),
subplot(2,1,1),plot(t_store1,Errorx_store1),title('tracking error in x direction');
xlabel('time')
ylabel('x-axis')
subplot(2,1,2),plot(t_store1,Errory_store1),title('tracking error in y direction');
xlabel('time')
ylabel('y-axis')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(13),clf(13),
plot(waypoints(:,1),waypoints(:,2),'rx'), title('Map with Waypoints')
xlabel('x-axis')
ylabel('y-axis')
xlim([0 10])
ylim([-0.5 10])
