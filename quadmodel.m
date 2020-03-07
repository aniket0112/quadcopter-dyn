% X = [u; v; w; phi; theta; psi; p; q; r; x; y; z]
% T4->t1 T2->t2 T1->t3 T3->t4
function [tsol,ysol] = quadmodel(t,x0,ixx,iyy,izz,m,l,k,b,rpm)
    K =  [-1717 -1708 -672.3 634 -0.3 133.4 132.9;
         -1613.2 1616.2 -613.1 581.1 0.5 125.5 -125.9;
         1631.4 1623.1 638.8 602.4 -0.3 -126.8 -126.3;
         1719.9 -1723 653.6 -619.5 0.5 -133.8 134.2];
    g = 9.8;
    [tsol,ysol] = ode45(@odefun,t,x0);
    function dX = odefun(t,X)
        dX = zeros(14,1);
        u = X(1); v = X(2); w = X(3); phi = X(4); theta = X(5); psi = X(6); p = X(7); q = X(8); r = X(9); x = X(10); y = X(11); z = X(12); ephi = X(13); etheta = X(14);
        
        X_ = [phi; theta; p; q; r; ephi; etheta];
        deltaw = -K*X_;
        rpm = rpm + deltaw;
        t1 = -k*rpm(1)^2;
        t2 = -k*rpm(2)^2;
        t3 = -k*rpm(3)^2;
        t4 = -k*rpm(4)^2;
        tau1 = b*rpm(1)^2;
        tau2 = b*rpm(2)^2;
        tau3 = b*rpm(3)^2;
        tau4 = b*rpm(4)^2;
        
        dX(1) = r*v-q*w-g*sin(theta);
        dX(2) = -r*u+p*w+g*sin(phi)*cos(theta);
        dX(3) = q*u-p*v+g*cos(phi)*cos(theta)+(t1+t2+t3+t4)/m;
        dX(4) = p+tan(theta)*(q*sin(phi)+r*cos(phi));
        dX(5) = q*cos(phi)-r*sin(phi);
        dX(6) = (q*sin(phi)+r*cos(phi))/cos(theta);
        dX(7) = q*r*(iyy-izz)/ixx+l*(t3+t2-t4-t1)/ixx;
        dX(8) = p*r*(izz-ixx)/iyy+l*(t3-t2-t4+t1)/iyy;
        dX(9) = p*q*(ixx-iyy)/izz+(tau3-tau2+tau4-tau1)/izz;
        dX(10) = u*cos(theta)*cos(psi)+v*(-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi))+w*(sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi));
        dX(11) = u*cos(theta)*sin(psi)+v*(cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi))+w*(-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi));
        dX(12) = -u*sin(theta)+v*sin(phi)*cos(theta)+w*cos(phi)*cos(theta);
        dX(13) = -phi;
        dX(14) = -theta;
   end
end