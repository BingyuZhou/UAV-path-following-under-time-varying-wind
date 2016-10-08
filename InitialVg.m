function [ Vg0 ] = InitialVg( A,phiA,W,phiw,Va,chi2)
% calculate the initial value of Vg

phic = atan2(W*sin(phiw)+A*sin(phiA),W*cos(phiw)+A*cos(phiA));
C = sqrt((W*sin(phiw)+A*sin(phiA))^2+(W*cos(phiw)+A*cos(phiA))^2);
phi = atan2(-C/Va*sin(phic-chi2),sqrt(1-C^2/Va^2*sin(phic-chi2)^2))+chi2;
Vg0 = sqrt((Va*sin(phi)+W*sin(phiw))^2+(Va*cos(phi)+W*cos(phiw))^2);
end

