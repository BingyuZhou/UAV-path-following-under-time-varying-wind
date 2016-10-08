function animation( x,y,H )
% animation of UAV path

for k=1:length(x)
    addpoints(H,x(k),y(k))
    drawnow
%     pause(0.1)
end

