% X = [u; v; w; phi; theta; psi; p; q; r; x; y; z]
% T4->t1 T2->t2 T1->t3 T3->t4
function [tsol,ysol] = quadmodel_control(t,x0,ixx,iyy,izz,l,k,b,m,nominal_radps,K)
    g = 9.8;
    delta_rps = zeros(4,1);
    [tsol,ysol] = ode45(@odefun,t,x0);
    function dX = odefun(t,X)
        dX = zeros(8,1);
        u = X(1); v = X(2); w = X(3);
        phi = X(4); theta = X(5); psi = X(6); p = X(7); q = X(8); r = X(9); ephi = X(10); etheta = X(11);
        X_ = [phi;theta;p;q;r;ephi;etheta];
        delta_rps = -K*X_;
        dX(1) = -g*theta;
        dX(2) = g*phi;
        dX(3) = g-2*k*[nominal_radps;nominal_radps;nominal_radps;nominal_radps]'*delta_rps/m;
        dX(4) = p+tan(theta)*(q*sin(phi)+r*cos(phi));
        dX(5) = q*cos(phi)-r*sin(phi);
        dX(6) = (q*sin(phi)+r*cos(phi))/cos(theta);
        dX(7) = q*r*(iyy-izz)/ixx-2*l*k*[nominal_radps;nominal_radps;-nominal_radps;-nominal_radps]'*delta_rps/ixx;
        dX(8) = p*r*(izz-ixx)/iyy-2*l*k*[nominal_radps;-nominal_radps;-nominal_radps;nominal_radps]'*delta_rps/iyy;
        dX(9) = p*q*(ixx-iyy)/izz+2*l*b*[nominal_radps;-nominal_radps;nominal_radps;-nominal_radps]'*delta_rps/izz;
        dX(10) = -phi;
        dX(11) = -theta;
    end
end