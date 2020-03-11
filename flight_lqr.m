% m = 2.6;
% k = 2.8e-5;
% b = 5e-7;
% l = 0.2;
% ixx = 3.63;
% iyy = 3.25;
% izz = 5.51;
m = 1.02;
ixx = 1.0446e-2;
iyy = 1.0714e-2;
izz = 1.3858e-2;
l = 0.22;
k = 1.9794e-7 * 3600/(2*3.14)^2;
b = 6.5124e-9* 3600/(2*3.14)^2;


nominal_t = -7;
A = [0 0 1 0 0 0 0;
     0 0 0 1 0 0 0;
     0 0 0 0 0 0 0;
     0 0 0 0 0 0 0;
     0 0 0 0 0 0 0;
     -1 0 0 0 0 0 0;
     0 -1 0 0 0 0 0;];
 B = [ 0 0 0 0;
       0 0 0 0;
       l/ixx l/ixx -l/ixx  -l/ixx;
       l/iyy -l/iyy -l/iyy  l/iyy;
       -b/k/izz b/k/izz -b/k/izz  b/k/izz;
       0 0 0 0;
       0 0 0 0];
 Q = diag([1.4e-7,1.4e-7,1e-3,1e-3,1e-3,0.9e3,0.9e3]);
 R = diag([7 7 7 7]);
 K = lqr(A,B,Q,R);
 [tsol,ysol] = quadmodel_control([0:1e-3:20],[0.1523,0.035,0,deg2rad(20),deg2rad(20),0,deg2rad(-4),deg2rad(-4),0,0,0],...
                                  ixx,iyy,izz,l,k,b,m,nominal_t,K);
 x = [ysol(:,4),ysol(:,5),ysol(:,7),ysol(:,8),ysol(:,9),ysol(:,10),ysol(:,11)];
 t = (nominal_t-(K*x')');
 tau = -b/k*t;
 U = [t,tau];
 [tsol_n,ysol_n] = quadmodel([0:1e-3:20],[0.1523,0.035,0,deg2rad(20),deg2rad(20),0,deg2rad(-4),deg2rad(-4),0,0,0,0.3045],U,...
                             ixx,iyy,izz,l,m,tsol);