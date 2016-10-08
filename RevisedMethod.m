%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%Path Following Algorithm for Time Varying Wind%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Path following algorithm for the time varying wind is developed based on
% Beard's method in paper "Vector field path following for miniature air
% vehicles" (IEEE Transactions on Robotics, 23(3),519-529).
close all;clear;clc;
%% Initialization
chi_inf = pi/2;    %course angle far away from path (rad)
alpha = 1.65;      %positive constant describe the speed of response of course
                    %hold autopilot loop (rad/s)
k = 0.1;      %positive constant influence the rate of the transition from
                    %x_inf to zero, also control the slope of the
                    %sliding surface near the origin(m^-1)
kk = pi/2;      %gain parameter controls the shape of the trajectories onto
                    %the sliding surface.(rad^2/s)
epsi = 0.5;       %width of the transition region around the sliding surface
                    %which is used to reduce chattering in the control.(rad)
Gamma = 80;   %Estimator gain for straight line

W = 6;       %constant wind velocity(m/s)
phiw = 230/180*pi;%constant wind direction(rad)
Va = 13;        %Longitudinal velocity(m/s)
A = 3;     % Time varying wind's amplitude (variance)
phiA = pi;    %Time varying wind's angle (variance)
%% ------------------------------------
% ---------Stright line following------
% -------------------------------------
% Initial conditions
x_int = 0;y_int = 80;course_int = pi/4;%Initial position and posture of UAV
ang = 0; a = 0;b = 0; % Course angle, slop and intercept of desired path
i=-1;% Directon of desired path (i = -1, go right, x increases; i = 1, go left, x decreases)
endx = 300;% stopping condition: the end value of x coordinate
Method =3;% 1: Beard's method, 2: Ideal method, 3: our method
% Initial value of Vg'
Vg0 = InitialVg(A,0,W,phiw,Va,course_int);
% simulation of stright line following
simout=sim('RevisedStraightLine');
% results
figure
[vfx,vfy] = meshgrid(0:20:300,-50:20:150);
wx = W*cos(phiw)*ones(size(vfx));
wy = W*sin(phiw)*ones(size(vfy));
quiver(vfx,vfy,wx,wy,0.5,'c','linewidth',0.5)
hold on
plot(x.data,y.data,'k','linewidth',2)
plot([0 300],[0 0],'--b','linewidth',2)
quiver(x.data(1:50:end),y.data(1:50:end),1*cos(chi_d.data(1:50:end)),1*sin(chi_d.data(1:50:end)),0.4,'r','linewidth',0.5)

title('Straight Line Following trajectory')
xlabel('x[m]')
ylabel('y[m]')
grid on
colormap(jet)
axis equal
legend('constant wind vector','UAV path','Desired path','Desired Course')

% comparison among Beard, ideal and ours
Method = 1; %Beard method
% simulation of stright line following
simout=sim('RevisedStraightLine');
error_M1 = rms(y.data(y.time>10))
figure
plot(y,'linewidth',1)
hold on
% plot([0 300],[0 300],'--b','linewidth',2)
figure
plot(x.data,y.data,'linewidth',1)
hold on

Method =2;% ideal method
simout=sim('RevisedStraightLine');
error_M2 = rms(y.data(y.time>10))
figure(2),plot(y,'linewidth',1)
figure(3),plot(x.data,y.data,'linewidth',1)
Method =3;% our method
simout=sim('RevisedStraightLine');
error_M3 = rms(y.data(y.time>10))
error_ss_M3 = rms(y.data(y.data<0.1))
figure(2),plot(y,'linewidth',1)
figure(3),plot(x.data,y.data,'linewidth',1)

figure(2),xlabel('t[s]')
ylabel('y[m]')
grid on
title('Distance of UAV from desired path')
legend('Standard VF','Ideal VF','Adaptive VF')
colormap(jet)
figure(3)
axis equal
quiver(vfx,vfy,wx,wy,0.5,'c','linewidth',0.5)
legend('Standard VF','Ideal VF','Adaptive VF')
title('Straight Line Following trajectory')
xlabel('x[m]')
ylabel('y[m]')
grid on
colormap(jet)
%performance of estimator
figure
plot(Vg,'b--')
hold on
plot(Vg2,'b','linewidth',1)
plot(Vg2hat,'r','linewidth',1)
legend('Vg','Vg`','Estimated Vg`')
grid on
title('Estimator')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
colormap(jet)
% estimator error
error_est_all = rms(Vg2.data-Vg2hat.data)
error_est_tras = rms(Vg2.data(Vg2.time<10)-Vg2hat.data(Vg2hat.time<10))
error_est_stea = rms(Vg2.data(Vg2.time>10)-Vg2hat.data(Vg2hat.time>10))

% %% -----------------------------
% % ---------Orbit following------
% % ------------------------------
%Initial conditions
cx = 50;cy = 50;R=10;%orbit center and radius
d_int = 15;course_int = -pi/4;gamma_int = pi/4; %initial position and posture in circular coordinate
j=-1; % define the direction of path(-1:clockwise,1:counterclockwise)
gamma_end = 15*pi; %stopping condition: 2pi-->one circle;4pi-->two circle...
Method =3;% 1: Beard's method, 2: Ideal method, 3: our method
% Initial value of Vg
Vg0 = InitialVg(A,0,W,phiw,Va,course_int);
%simulation of orbit following
simout=sim('RevisedOrbitFollowing');
%Result
figure
plot(x.data,y.data,'k','linewidth',2)
hold on
quiver(x.data(1:500:end),y.data(1:500:end),1*cos(chi_d.data(1:500:end)),1*sin(chi_d.data(1:500:end)),0.2,'r','linewidth',0.5)
[vfx,vfy] = meshgrid(35:10:65,35:10:65);
wx = W*cos(phiw)*ones(size(vfx));
wy = W*sin(phiw)*ones(size(vfy));
quiver(vfx,vfy,wx,wy,0.5,'c','linewidth',0.5)

t = -pi:0.1:pi;
plot(cx+R*cos(t),cy+R*sin(t),'--b','linewidth',2)
scatter(cx,cy,'*b')
title('Orbit Following trajectory')
xlabel('x[m]')
ylabel('y[m]')
axis('equal')
legend('UAV path','Desired course','constant wind vector','Desired Path')
grid on
colormap(jet)

% comparison among Bear's, ideal and our method
Method = 1; %Beard method
% simulation of stright line following
simout=sim('RevisedOrbitFollowing');
error_M1 = rms(d.data(d.time>4)-R)
figure
plot(x.data,y.data,'linewidth',1)
hold on
% plot([0 300],[0 300],'--b','linewidth',2)
figure
plot(d,'linewidth',1)
hold on
Method =2;% ideal method
simout=sim('RevisedOrbitFollowing');
error_M2 = rms(d.data(d.time>4)-R)
figure(6),plot(x.data,y.data,'linewidth',1)
figure(7),plot(d,'linewidth',1)
Method =3;% our method
simout=sim('RevisedOrbitFollowing');
error_M3 = rms(d.data(d.time>4)-R)
error_ss_M3 = rms(d.data(d.data<0.1+R)-R)
figure(6),plot(x.data,y.data,'linewidth',1)
figure(7),plot(d,'linewidth',1)
hold off
figure(6),title('Orbit Following trajectory')
xlabel('x[m]')
ylabel('y[m]')
axis('equal')
grid on
legend('Standard VF','Ideal VF','Adaptive VF')
quiver(vfx,vfy,wx,wy,0.5,'c','linewidth',0.5)
t = -pi:0.1:pi;
plot(cx+R*cos(t),cy+R*sin(t),'--g','linewidth',2)
scatter(cx,cy,'*g')
axis equal


figure(7),title('Distance of UAV from orbit center')
xlabel('Time[s]')
ylabel('Distance[m]')
grid on
legend('Standard VF','Ideal VF','Adaptive VF')
colormap(jet)
%% evaluation
%performance of estimator
figure
plot(Vg2,'b','linewidth',1)
hold on,plot(Vg,'b--')
plot(Vg2hat,'r','linewidth',1)
legend('Vg`','Vg','Estimated Vg`')
grid on
title('Estimator')
xlabel('Time [s]')
ylabel('Velocity [m/s]')
colormap(jet)
% estimator error
error_est_all = rms(Vg2.data-Vg2hat.data)
error_est_tras = rms(Vg2.data(Vg2.time<10)-Vg2hat.data(Vg2hat.time<10))
error_est_stea = rms(Vg2.data(Vg2.time>10)-Vg2hat.data(Vg2hat.time>10))

%% -----------------------------
% ---------Combination Path-----
% ------------------------------
% Method = 3;
% x0 = 20; y0 =20;
% x1 = 300; y1 = 300;
% x2 = 20; y2 = 300;
% x3 = 300; y3 = 20;
% px = [x0;x1;x2;x3];
% py = [y0;y1;y2;y3];
% 
% a1 = (y1-y0)/(x1-x0);
% a2 = (y2-y1)/(x2-x1);
% a3 = (y3-y2)/(x3-x2);
% a4 = (y0-y3)/(x0-x3);
% aa = [a1;a2;a3;a4];
% 
% ang1 = atan2((y1-y0),(x1-x0));
% ang2 = atan2((y2-y1),(x2-x1));
% ang3 = atan2((y3-y2),(x3-x2))+2*pi;
% ang4 = -atan2((y0-y3),(x0-x3));
% angle = [ang1,ang2,ang3,ang4];
% 
% b1 = y1-a1*x1;
% b2 = y2-a2*x2;
% b3 = y3-a3*x3;
% b4 = y0-a4*x0;
% bb = [b1;b2;b3;b4];
% line = [-1,1,-1,1];
% 
% x_int = 10;y_int = 10;course_int = 0;R=10;x_end = x1;y_end = y1;
% Rotation = [1 1 -1 -1];
% figure
% xlabel('x(m)'),ylabel('y(m)')
% [vfx,vfy] = meshgrid(0:20:350,0:20:350);
% wx = W*cos(phiw)*ones(size(vfx));
% wy = W*sin(phiw)*ones(size(vfy));
% quiver(vfx,vfy,wx,wy,0.5,'c','linewidth',0.5)
% hold on
% plot([px;x0],[py;y0],'--','linewidth',2);
% axis([0 350 0 350]),axis equal;grid on
% z=1;h=1;CircleCen=[];
% while h~=6
%     [E1,E2,C,gamma_end]=OrbitCenter([px(mod(z,4)+1),py(mod(z,4)+1)],R,aa(z),aa(mod(z,4)+1),z);
%     CircleCen = [CircleCen,C];
%     
%     if mod(z,2)==0
%         endx = -E1(1)-6;
%     else
%         endx = E2(1)-6;
%     end
%     a = aa(z);b = bb(z);ang = angle(z);i = line(z);
%     simout=sim('RevisedStraightLine');
%     
%     
%     H = animatedline('color','k','linewidth',2);
%     animation(x.data,y.data,H)
% %     plot(x.data,y.data,'r','linewidth',1)
%     
%     hold on
% %     quiver(x.data(1:10:end),y.data(1:10:end),0.5*cos(chi_d.data(1:10:end)),0.5*sin(chi_d.data(1:10:end)),0.2,'r','linewidth',0.5)
%     pause
%     if abs(course.data(end))>3.15
%         course_int = course.data(end)-2*pi;
%     else
%         course_int = course.data(end);
%     end
%      cx = C(1);cy = C(2);
%     d_int = norm([x.data(end),y.data(end)]-[cx,cy]);
%     gamma_int = atan2((y.data(end)-cy),(x.data(end)-cx));
%     j=Rotation(z);
%     simout=sim('RevisedOrbitFollowing');
%     x_int = x.data(end);y_int = y.data(end);
%     course_int = course.data(end);
%    
%     animation(x.data,y.data,H)
% %     quiver(x.data(1:10:end),y.data(1:10:end),1*cos(chi_d.data(1:10:end)),1*sin(chi_d.data(1:10:end)),0.2,'r','linewidth',0.5)
% %     plot(x.data,y.data,'r','linewidth',1)
%     pause
%     z=z+1;
%     h=h+1;
%     if z>4
%         z = 1;
%         course_int = course_int+2*pi;
%     end
% end
% 
% legend('Constant wind vector','Desired path','UAV path')
% t=-pi:0.1:pi;
% plot(CircleCen(1,1)+R*cos(t),CircleCen(2,1)+R*sin(t),'--g')
% plot(CircleCen(1,2)+R*cos(t),CircleCen(2,2)+R*sin(t),'--g')
% plot(CircleCen(1,3)+R*cos(t),CircleCen(2,3)+R*sin(t),'--g')
% plot(CircleCen(1,4)+R*cos(t),CircleCen(2,4)+R*sin(t),'--g')
% scatter(CircleCen(1,:),CircleCen(2,:),'*g')
% hold off
% title('Combination of Straight Line and Orbit Following') 
% xlabel('x[m]')
% ylabel('y[m]')



