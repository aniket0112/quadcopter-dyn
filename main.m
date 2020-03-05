m = 1.02;
ixx = 2;
iyy = 2;
izz = 4;
l = 0.22;
dt = 0.1;

mavlink_time = motor.Time-motor.Time(1);
[Y,M,D,hours,minutes,seconds] = datevec(mavlink_time);
experiment_time = hours*3600+minutes*60+seconds;
tu_ = experiment_time(78:100);
tu_ = tu_-tu_(1);
U = [control_input(t1(78:100),tu_,dt),control_input(t2(78:100),tu_,dt),control_input(t3(78:100),tu_,dt),control_input(t4(78:100),tu_,dt),...
    control_input(tau1(78:100),tu_,dt),control_input(tau2(78:100),tu_,dt),control_input(tau3(78:100),tu_,dt),control_input(tau4(78:100),tu_,dt)];
tu = [tu_(1):dt:tu_(end)+1];
% tu = [0:1:60];
% U = [(unitstep(tu,-2,1)+unitstep(tu,2,10)+unitstep(tu,-11,30))',...
%     (unitstep(tu,2,1)+unitstep(tu,-2,10)+unitstep(tu,11,30))',...
%     (unitstep(tu,-2,1)+unitstep(tu,2,10)+unitstep(tu,-11,30))',...
%     (unitstep(tu,-2,1)+unitstep(tu,2,10)+unitstep(tu,-11,30))',...
%     zeros(length(tu),1),...
%     zeros(length(tu),1),...
%     zeros(length(tu),1),...
%     zeros(length(tu),1)];

x0 = [0;0;0;0;0;0;0;0;0;0;0;-0.1];
[tsol,ysol] = quadmodel(tu,x0,U,ixx,iyy,izz,m,l,tu);