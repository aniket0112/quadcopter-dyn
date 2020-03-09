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
        
        dX(1) = -g*theta;
        dX(2) = g*phi;
        dX(3) = (T(1)+T(2)+T(3)+T(4))/m;
        dX(4) = p;
        dX(5) = q;
        dX(6) = r;
        dX(7) = l*(T(1)+T(2)-T(3)-T(4))/ixx;
        dX(8) = l*(T(1)-T(2)-T(3)+T(4))/iyy;
        dX(9) = -b/k*(T(1)-T(2)+T(3)-T(4))/izz;
        dX(10) = -phi;
        dX(11) = -theta;
    end
end
