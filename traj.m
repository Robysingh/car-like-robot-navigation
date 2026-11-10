function xt=traj(a,t)
%Given a0,a1,a2,a3,a4 with specified time t, compute x(t)
a0=a(1);a1=a(2);a2=a(3);a3=a(4);a4=a(5);
xt=a0+a1*t+a2*t^2+a3*t^3+a4*t^4;
