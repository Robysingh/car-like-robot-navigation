speed_nominal=0.5;
t0=0;
v0=[1,0];%initial velocity
t1=t0;dt=0.01;
v1=v0;

waypoints=[1,0; 1,1; 1,3; 1,4; 1.5,9; 3,9; 6,9; 9,9; 9,6; 9,4;...
           9,2; 9,0; 8,0; 4,0; 3,0; 5,1; 5,4; 5,6; 4,6; 3,5;...
           3,2; 1,3; 1,1; 1,0];

x_store=waypoints(1,1);
y_store=waypoints(1,2);
vx_store=v0(1);
vy_store=v0(2);
t_store=t0;
[m,n]=size(waypoints);
for i=1:m-1
    P1=waypoints(i,:);
    P2=waypoints(i+1,:);
    d1=norm(P2-P1);
    t2=t1+d1/speed_nominal;
    %d: the distance between two consecutive points;
    %t2-t1=d/nominal speed;
    if i==m-1
        v2=[0,0];
    else
        P3=waypoints(i+2,:);
        v2=speed_nominal*(P3-P2)/norm(P3-P2);
    end
    ax=trajgene(P1(1),v1(1),t1,P2(1),v2(1),t2);
    ay=trajgene(P1(2),v1(2),t1,P2(2),v2(2),t2);
    for t=t1:dt:t2
        xt=traj(ax,t);
        yt=traj(ay,t);
        vxt=generated_speed(ax,t);
        vyt=generated_speed(ay,t);
        x_store=[x_store;xt];
        y_store=[y_store;yt];
        vx_store=[vx_store;vxt];
        vy_store=[vy_store;vyt];
        t_store=[t_store;t];
    end
    t1=t2;
    v1=v2;
end

%plotting
figure(10),clf(10),
subplot(2,1,1),plot(t_store,x_store),title('x(t) vs. t');
xlabel('time')
ylabel('x(t)')
subplot(2,1,2),plot(t_store,y_store),title('y(t) vs. t');
xlabel('time')
ylabel('y(t)')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(11),clf(11),
subplot(2,1,1),plot(t_store,vx_store),title('vx(t) vs. t');
xlabel('time')
ylabel('vx(t)')
subplot(2,1,2),plot(t_store,vy_store),title('vy(t) vs. t');
xlabel('time')
ylabel('vy(t)')
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(12),clf(12),
plot(x_store,y_store,'k');title('smooth trajectory: x--waypoints; solid line--generated traj')
hold on,
plot(waypoints(:,1),waypoints(:,2),'x')
xlabel('x-axis')
ylabel('y-axis')
xlim([0 10])
ylim([-0.5 10])
