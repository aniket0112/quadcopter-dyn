m = 1;
ixx = 1;
iyy = 1;
izz = 2;
l = 0.22;
t = [0:0.1:60];

mavlink_time = motor.Time-motor.Time(1);
[Y,M,D,hours,minutes,seconds] = datevec(mavlink_time);
experiment_time = hours*3600+minutes*60+seconds;
tu = experiment_time(78:100);
tu = tu-tu(1);
U = [control_input(t1(78:100),tu,0.1),control_input(t2(78:100),tu,0.1),control_input(t3(78:100),tu,0.1),control_input(t4(78:100),tu,0.1),...
    control_input(tau1(78:100),tu,0.1),control_input(tau2(78:100),tu,0.1),control_input(tau3(78:100),tu,0.1),control_input(tau4(78:100),tu,0.1)];
tu = [tu(1):0.1:tu(end)];
% tu = [0:1:60];
% U = [(unitstep(tu,-2,1)+unitstep(tu,2,10)+unitstep(tu,-11,30))',...
%     (unitstep(tu,-2,1)+unitstep(tu,2,10)+unitstep(tu,-11,30))',...
%     (unitstep(tu,-2,1)+unitstep(tu,2,10)+unitstep(tu,-11,30))',...
%     (unitstep(tu,-2,1)+unitstep(tu,2,10)+unitstep(tu,-11,30))',...
%     zeros(length(tu),1),...
%     zeros(length(tu),1),...
%     zeros(length(tu),1),...
%     zeros(length(tu),1)];

x0 = [0;0;0;0;0;0;0;0;0;0;0;-0.1];
[tsol,ysol] = quadmodel(t,x0,U,ixx,iyy,izz,m,l,tu);