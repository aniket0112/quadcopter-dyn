% X = [u; v; w; phi; theta; psi; p; q; r; x; y; z]
% T4->t1 T2->t2 T1->t3 T3->t4
function [tsol,ysol] = quadmodel_linear(t,x0,ixx,iyy,izz,l,m,U,tu)
    g = 9.8;
    [tsol,ysol] = ode45(@odefun,t,x0);
    function dX = odefun(t,X)
        dX = zeros(12,1);
        u = X(1); v = X(2); w = X(3);
        phi = X(4); theta = X(5); psi = X(6); p = X(7); q = X(8); r = X(9); ephi = X(10); etheta = X(11);
        
        t1 = interp1(tu,U(:,1),t); 
        t2 = interp1(tu,U(:,2),t); 
        t3 = interp1(tu,U(:,3),t); 
        t4 = interp1(tu,U(:,4),t);
        tau1 = interp1(tu,U(:,5),t); 
        tau2 = interp1(tu,U(:,6),t);   
        tau3 = interp1(tu,U(:,7),t); 
        tau4 = interp1(tu,U(:,8),t);
        
        dX(1) = -g*theta;
        dX(2) = g*phi;
        dX(3) = (t1+t2+t3+t4)/m;
        dX(4) = p;
        dX(5) = q;
        dX(6) = r;
        dX(7) = l*(t1+t2-t3-t4)/ixx;
        dX(8) = l*(t1-t2-t3+t4)/iyy;
        dX(9) = (tau1-tau2+tau3-tau4)/izz;
        dX(10) = u;
        dX(11) = v;
        dX(12) = w;
    end
end
