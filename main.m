m = 1.02;
ixx = 1.0446e-2;
iyy = 1.0714e-2;
izz = 1.3858e-2;
l = 0.22;
dt = 0.1;

mavlink_time = motor.Time-motor.Time(1);
[Y,M,D,hours,minutes,seconds] = datevec(mavlink_time);
experiment_time = hours*3600+minutes*60+seconds;
tu_ = experiment_time(73:148);
tu_ = tu_-tu_(1);
U = [control_input(t1(73:148),tu_,dt),control_input(t2(73:148),tu_,dt),control_input(t3(73:148),tu_,dt),control_input(t4(73:148),tu_,dt),...
    control_input(tau1(73:148),tu_,dt),control_input(tau2(73:148),tu_,dt),control_input(tau3(73:148),tu_,dt),control_input(tau4(73:148),tu_,dt)];
tu = [tu_(1):dt:tu_(end)+1];
% tu = [0:1:60];
% U = [(unitstep(tu,-5,0)+unitstep(tu,0,20)+unitstep(tu,0.01,22)+unitstep(tu,-0.01,24))',...
%     (unitstep(tu,-5,0)+unitstep(tu,0.01,20)+unitstep(tu,-0.01,22)+unitstep(tu,0,24))',...
%     (unitstep(tu,-5,0)+unitstep(tu,0,20)+unitstep(tu,0.01,22)+unitstep(tu,-0.01,24))',...
%     (unitstep(tu,-5,0)+unitstep(tu,0.01,20)+unitstep(tu,-0.01,22)+unitstep(tu,0,24))',...
%     zeros(length(tu),1),...
%     zeros(length(tu),1),...
%     zeros(length(tu),1),...
%     zeros(length(tu),1)];

x0 = [0;0;0;0;0;0;0;0;0;0;0;-0.1];
%[tsol,ysol] = quadmodel(tu,x0,U,ixx,iyy,izz,m,l,tu);
%plot(U(:,1)); hold on; plot(U(:,2)); plot(U(:,3)); plot(U(:,4));
%plot(-ysol(:,12))