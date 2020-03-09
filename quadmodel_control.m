% X = [u; v; w; phi; theta; psi; p; q; r; x; y; z]
% T4->t1 T2->t2 T1->t3 T3->t4
function [tsol,ysol] = quadmodel_control(t,x0,ixx,iyy,izz,l,k,b,m,nominal_t,K)
    g = 9.8;
    [tsol,ysol] = ode45(@odefun,t,x0);
    function dX = odefun(t,X)
        dX = zeros(11,1);
        u = X(1); v = X(2); w = X(3);
        phi = X(4); theta = X(5); psi = X(6); p = X(7); q = X(8); r = X(9); ephi = X(10); etheta = X(11);
        X_ = [phi;theta;p;q;r;ephi;etheta];
        T = nominal_t - K*X_;
        t1 = T(1);
        
        dX(1) = r*v-q*w-g*sin(theta);
        dX(2) = -r*u+p*w+g*sin(phi)*cos(theta);
        dX(3) = q*u-p*v+g*cos(phi)*cos(theta)+(t1+t2+t3+t4)/m;
        dX(4) = p+tan(theta)*(q*sin(phi)+r*cos(phi));
        dX(5) = q*cos(phi)-r*sin(phi);
        dX(6) = (q*sin(phi)+r*cos(phi))/cos(theta);
        dX(7) = q*r*(iyy-izz)/ixx+l*(t1+t2-t3-t4)/ixx;
        dX(8) = p*r*(izz-ixx)/iyy+l*(t1-t2-t3+t4)/iyy;
        dX(9) = p*q*(ixx-iyy)/izz+(tau1-tau2+tau3-tau4)/izz;
        dX(10) = -phi;
        dX(11) = -theta;
    end
end
