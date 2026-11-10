function vt=generated_speed(a,t)
%Given a0,a1,a2,a3,a4 with specified time t, compute x(t)
a0=a(1);a1=a(2);a2=a(3);a3=a(4);a4=a(5);
vt=a1+2*a2*t+3*a3*t^2+4*a4*t^3;
