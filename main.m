m = 1.02;
ixx = 1.0446e-2;
iyy = 1.0714e-2;
izz = 1.3858e-2;
l = 0.22;
dt = 0.1;

tstart = 108;
tend = 120;

mavlink_time = motor.Time-motor.Time(1);
[Y,M,D,hours,minutes,seconds] = datevec(mavlink_time);
experiment_time = hours*3600+minutes*60+seconds;
tu_ = experiment_time(tstart:tend);
tu_ = tu_-tu_(1);
U = smoothdata([control_input(t3(tstart:tend),tu_,dt),control_input(t2(tstart:tend),tu_,dt),control_input(t4(tstart:tend),tu_,dt),control_input(t1(tstart:tend),tu_,dt),...
                control_input(tau3(tstart:tend),tu_,dt),control_input(tau2(tstart:tend),tu_,dt),control_input(tau4(tstart:tend),tu_,dt),control_input(tau1(tstart:tend),tu_,dt)]);
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
%U = smoothdata([log_t1,log_t2,log_t3,log_t4,log_tau1,log_tau2,log_tau3,log_tau4]);

x0 = [position.vx(tstart);position.vy(tstart);position.vz(tstart);
      attitude.roll(tstart);attitude.pitch(tstart);attitude.yaw(tstart);
      attitude.rollspeed(tstart);attitude.pitchspeed(tstart);attitude.yawspeed(tstart);
      position.lat(tstart);position.lon(tstart);position.alt(tstart)];
[tsol,ysol] = quadmodel_linear(tu,x0,ixx,iyy,izz,l,m,U,tu);
