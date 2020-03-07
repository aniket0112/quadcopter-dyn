m = 2.6;
ixx = 3.63;
iyy = 3.25;
izz = 5.51;
l = 0.2;
k =  2.82e-5;
b = 7e-7;
t = [0:60];
x0 = [0;0;0;0;deg2rad(20);0;0;0;0;0;0;0;0;0];
[tsol,ysol] = quadmodel(t,x0,ixx,iyy,izz,m,l,k,b,[500;500;500;500]);