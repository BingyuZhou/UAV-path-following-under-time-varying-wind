function [ E1,E2,C,gamma_end ] = OrbitCenter( p,R,k1,k2,key)
% find the center and radius of the orbit path from three points on the
% circle
C = zeros(2,1);
x = p(1); y = p(2);

if atan(k1)<0
    ang1 = atan(k1)+pi;
else
    ang1 = atan(k1);
end
if atan(k2)<0
    ang2 = atan(k2)+pi;
else
    ang2 = atan(k2);
end

if abs(ang1-ang2)>pi/2
    alpha = (pi-abs(ang1-ang2))/2;
else
    alpha = abs((ang1-ang2)/2);
end
dx = R*cos(alpha);
dy = R*sin(alpha);

switch key
    case 1,
        E1(1) = x-2*dx;
        E1(2) = 300;
        E2(1) = x-2*dx*cos(alpha*2);
        E2(2) = y-2*dx*sin(alpha*2);
        gamma_end = pi-(pi/2-2*alpha);
        C(1) = x-dx;
        C(2) = y-dy;
    case 2,
        E1(1) = x+2*dx;
        E1(2) = 300;
        E2(1) = x+2*dx*cos(alpha*2);
        E2(2) = y-2*dx*sin(alpha*2);
        gamma_end = pi+(pi/2-2*alpha);
        C(1) = x+dx;
        C(2) = y-dy;
    case 3,
        E1(1) = x-2*dx;
        E1(2) = 20;
        E2(1) = x-2*dx*cos(alpha*2);
        E2(2) = y+2*dx*sin(alpha*2);
        gamma_end = (pi-(pi/2-2*alpha));
        C(1) = x-dx;
        C(2) = y+dy;
    case 4,
        E1(1) = x+2*dx;
        E1(2) = 20;
        E2(1) = x+2*dx*cos(alpha*2);
        E2(2) = y+2*dx*sin(alpha*2);
        gamma_end = (pi+(pi/2-2*alpha));
        C(1) = x+dx;
        C(2) = y+dy;
end
        
end

