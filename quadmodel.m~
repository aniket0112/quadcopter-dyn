% X = [u; v; w; phi; theta; psi; p; q; r; x; y; z]
% T4->t1 T2->t2 T1->t3 T3->t4
function [tsol,ysol] = quadmodel(t,x0,U,ixx,iyy,izz,l,m,tu)
    g = 9.8;
    [tsol,ysol] = ode45(@odefun,t,x0);
    function dX = odefun(t,X)
        dX = zeros(12,1);
        u = X(1); v = X(2); w = X(3); phi = X(4); theta = X(5); psi = X(6); p = X(7); q = X(8); r = X(9); x = X(10); y = X(11); z = X(12);
        
        t1 = interp1(tu,U(:,1),t); 
        t2 = interp1(tu,U(:,2),t); 
        t3 = interp1(tu,U(:,3),t); 
        t4 = interp1(tu,U(:,4),t);
        tau1 = interp1(tu,U(:,5),t); 
        tau2 = interp1(tu,U(:,6),t);   
        tau3 = interp1(tu,U(:,7),t); 
        tau4 = interp1(tu,U(:,8),t);
        
        dX(1) = r*v-q*w-g*sin(theta);
        dX(2) = -r*u+p*w+g*sin(phi)*cos(theta);
        dX(3) = q*u-p*v+g*cos(phi)*cos(theta)+(t1+t2+t3+t4)/m;
        dX(4) = p+tan(theta)*(q*sin(phi)+r*cos(phi));
        dX(5) = q*cos(phi)-r*sin(phi);
        dX(6) = (q*sin(phi)+r*cos(phi))/cos(theta);
        dX(7) = q*r*(iyy-izz)/ixx+l*(t1+t2-t3-t4)/ixx;
        dX(8) = p*r*(izz-ixx)/iyy+l*(t1-t2-t3+t4)/iyy;
        dX(9) = p*q*(ixx-iyy)/izz+(tau1-tau2+tau3-tau4)/izz;
        dX(10) = u*cos(theta)*cos(psi)+v*(-cos(phi)*sin(psi)+sin(phi)*sin(theta)*cos(psi))+w*(sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi));
        dX(11) = u*cos(theta)*sin(psi)+v*(cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi))+w*(-sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi));
        dX(12) = -u*sin(theta)+v*sin(phi)*cos(theta)+w*cos(phi)*cos(theta);
   end
end