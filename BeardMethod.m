%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%BeardMethod(Sliding Mode)%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;clear;clc
%% Initialization
chi_inf  = pi/2;    %course angle for away from path (rad)
alpha  = 1.65;      %positive constant describe the speed of response of course
                    %hold autopilot loop (rad/s)
k      = 0.02;      %positive constant influence the rate of the transition from
                    %x_inf to zero, also control the slope of the slope of the
                    %sliding surface near the origin(m^-1)
kk     = pi/2;      %gain parameter controls the shape of the trajectories onto
                    %the sliding surface.(rad^2/s)
epsi   = 0.1;       %width of the transition region around the sliding surface
                    %which is used to reduce chattering in the control.(rad)
w      = 0.9;       %constant current velocity(m/s)
phiw   = 230/180*pi;%constant current direction(rad)
Va     = 13;        %Longitudinal velocity(m/s)
        

%% simulation (Straight line)
% x_int = 0;y_int = 100;course_int = pi/4;ang = 0;endx = 500;aa = 0;bb=0;i=1;
% simout=sim('StraightLine');
% figure
% plot([0 500],[0 0],'--b')
% hold on
% [vfx,vfy] = meshgrid(0:20:500,-50:20:200);
% wx = w*cos(phiw)*ones(size(vfx));
% wy = w*sin(phiw)*ones(size(vfy));
% quiver(vfx,vfy,wx,wy,0.5,'c','linewidth',0.5)
% plot(x.data,y.data,'k','linewidth',2)
% quiver(x.data,y.data,vx.data,vy.data,0.2,'r','linewidth',0.5)
% title('Straight Line Following trajectory')
% legend('Desired path','wind vector','Real UAV path','Velocity')
% xlabel('x(m)')
% ylabel('y(m)')
% grid on
% axis([0 500 -50 200]),axis equal

%% simulation (Orbit following)
% cx = 50;cy = 50;R=10;
% d_int = 15;course_int = -pi/4; j=-1;
% gamma_int = pi/4;gamma_end = 4*pi;
% simout=sim('OrbitFollowing');
% figure
% plot(x.data,y.data,'k','linewidth',2)
% hold on
% quiver(x.data,y.data,vx.data,vy.data,0.2,'r','linewidth',0.5)
% 
% t = -pi:0.1:pi;
% plot(cx+R*cos(t),cy+R*sin(t),'--b')
% [vfx,vfy] = meshgrid(25:10:70,30:10:80);
% wx = w*cos(phiw)*ones(size(vfx));
% wy = w*sin(phiw)*ones(size(vfy));
% quiver(vfx,vfy,wx,wy,0.5,'c','linewidth',0.5)
% scatter(cx,cy,'*b')
% xlabel('x(m)')
% ylabel('y(m)')
% axis([25 70 30 80]),axis equal
% grid on
% legend('Real path','velocity','Desire path','wind vector')
% title('Orbit Following trajectory')

%% Combination 
x0 = 20; y0 =20;
x1 = 300; y1 = 300;
x2 = 20; y2 = 300;
x3 = 300; y3 = 20;
px = [x0;x1;x2;x3];
py = [y0;y1;y2;y3];

a1 = (y1-y0)/(x1-x0);
a2 = (y2-y1)/(x2-x1);
a3 = (y3-y2)/(x3-x2);
a4 = (y0-y3)/(x0-x3);
a = [a1;a2;a3;a4];

ang1 = atan2((y1-y0),(x1-x0));
ang2 = atan2((y2-y1),(x2-x1));
ang3 = atan2((y3-y2),(x3-x2))+2*pi;
ang4 = -atan2((y0-y3),(x0-x3));
angle = [ang1;ang2;ang3;ang4];

b1 = y1-a1*x1;
b2 = y2-a2*x2;
b3 = y3-a3*x3;
b4 = y0-a4*x0;
b = [b1;b2;b3;b4];

x_int = 10;y_int = 10;course_int = 0;R=10;x_end = x1;y_end = y1;
Rotation = [1 1 -1 -1];
figure
xlabel('x(m)'),ylabel('y(m)')

plot([px;x0],[py;y0],'--b','linewidth',1);
axis([0 350 0 350]),axis equal;grid on
hold on
[vfx,vfy] = meshgrid(0:20:350,0:20:350);
wx = w*cos(phiw)*ones(size(vfx));
wy = w*sin(phiw)*ones(size(vfy));
quiver(vfx,vfy,wx,wy,0.5,'c','linewidth',0.5)
i=1;h=1;CircleCen=[];
while h~=6
    [E1,E2,C,gamma_end]=OrbitCenter([px(mod(i,4)+1),py(mod(i,4)+1)],R,a(i),a(mod(i,4)+1),i);
    CircleCen = [CircleCen,C];
    
    if mod(i,2)==0
        endx = -E1(1)-6;
    else
        endx = E2(1)-6;
    end
    aa = a(i);bb = b(i); ang = angle(i);
    simout=sim('StraightLine');
    
    H = animatedline('color','k','linewidth',2);
    animation(x.data,y.data,H)
    
    hold on
    quiver(x.data,y.data,vx.data,vy.data,0.2,'r','linewidth',0.5)
    pause
    if abs(course.data(end))>3.15
        course_int = course.data(end)-2*pi;
    else
        course_int = course.data(end);
    end
     cx = C(1);cy = C(2);
    d_int = norm([x.data(end),y.data(end)]-[cx,cy]);
    gamma_int = atan2((y.data(end)-cy),(x.data(end)-cx));
    j=Rotation(i);
    simout=sim('OrbitFollowing');
    x_int = x.data(end);y_int = y.data(end);
    course_int = course.data(end);
   
    animation(x.data,y.data,H)
    quiver(x.data,y.data,vx.data,vy.data,0.2,'r','linewidth',0.5)

    pause
    i=i+1;
    h=h+1;
    if i>4
        i = 1;
        course_int = course_int+2*pi;
    end
end
legend('Desired path','wind vector','Real UAV path','velocity')
t=-pi:0.1:pi;
plot(CircleCen(1,1)+R*cos(t),CircleCen(2,1)+R*sin(t),'--g')
plot(CircleCen(1,2)+R*cos(t),CircleCen(2,2)+R*sin(t),'--g')
plot(CircleCen(1,3)+R*cos(t),CircleCen(2,3)+R*sin(t),'--g')
plot(CircleCen(1,4)+R*cos(t),CircleCen(2,4)+R*sin(t),'--g')
scatter(CircleCen(1,:),CircleCen(2,:),'*g')
hold off
title('Combination of Straight Line and Orbit Following')    
    

