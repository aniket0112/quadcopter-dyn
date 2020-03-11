m = 1.02;
ixx = 1.0446e-2;
iyy = 1.0714e-2;
izz = 1.3858e-2;
l = 0.22;
dt = 0.1;

tstart = 450;
tend = 650;
%% Telemetry log
% x0 = [position.vx(tstart);position.vy(tstart);position.vz(tstart);
%       attitude.roll(tstart);attitude.pitch(tstart);attitude.yaw(tstart);
%       attitude.rollspeed(tstart);attitude.pitchspeed(tstart);attitude.yawspeed(tstart);
%       position.lat(tstart);position.lon(tstart);position.alt(tstart)];
% mavlink_time = motor.Time-motor.Time(1);
% [Y,M,D,hours,minutes,seconds] = datevec(mavlink_time);
% experiment_time = hours*3600+minutes*60+seconds;
% tu_ = experiment_time(tstart:tend);
% tu_ = tu_-tu_(1);
% U = smoothdata([control_input(t3(tstart:tend),tu_,dt),control_input(t2(tstart:tend),tu_,dt),control_input(t4(tstart:tend),tu_,dt),control_input(t1(tstart:tend),tu_,dt),...
%                 control_input(tau3(tstart:tend),tu_,dt),control_input(tau2(tstart:tend),tu_,dt),control_input(tau4(tstart:tend),tu_,dt),control_input(tau1(tstart:tend),tu_,dt)]);
%% Virtual data
% tu = [0:1:60];
% U = [(unitstep(tu,-5,0)+unitstep(tu,0,20)+unitstep(tu,0.01,22)+unitstep(tu,-0.01,24))',...
%     (unitstep(tu,-5,0)+unitstep(tu,0.01,20)+unitstep(tu,-0.01,22)+unitstep(tu,0,24))',...
%     (unitstep(tu,-5,0)+unitstep(tu,0,20)+unitstep(tu,0.01,22)+unitstep(tu,-0.01,24))',...
%     (unitstep(tu,-5,0)+unitstep(tu,0.01,20)+unitstep(tu,-0.01,22)+unitstep(tu,0,24))',...
%     zeros(length(tu),1),...
%     zeros(length(tu),1),...
%     zeros(length(tu),1),...
%     zeros(length(tu),1)];
%% Dataflash log
tu = t(tstart:tend)-t(tstart);
fc_t = 0.1;
fc_tau = 0.1;
[u1,u2,u3,u4] = fft_filter(tu,fc_t,log_t3(tstart:tend),log_t2(tstart:tend),log_t4(tstart:tend),log_t1(tstart:tend));
[tau1,tau2,tau3,tau4] = fft_filter(tu,fc_tau,log_tau3(tstart:tend),log_tau2(tstart:tend),log_tau4(tstart:tend),log_tau1(tstart:tend));
U = [u3,u2,u4,u1+2,tau3,tau2,tau4,tau1];
x0 = [vx(tstart);vy(tstart);vz(tstart);
      roll(tstart);pitch(tstart);yaw(tstart);
      p(tstart);q(tstart);r(tstart);
      lat(tstart);lng(tstart);alt(tstart)];
  
%% Control law for given initial condition based on LQR
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
[tsol_c,ysol_c] = quadmodel_control([tu(1):1e-3:tu(end)],[x0(1:9);0;0],ixx,iyy,izz,l,k,b,m,nominal_t,K);
 x = [ysol_c(:,4),ysol_c(:,5),ysol_c(:,7),ysol_c(:,8),ysol_c(:,9),ysol_c(:,10),ysol_c(:,11)];
 thrust = (nominal_t-(K*x')');
 tau = -b/k*thrust;
 u = [thrust,tau];
%% Nonlinear quadmodel solve
% close all
%[tsol,ysol] = quadmodel(tu,x0,u,ixx,iyy,izz,l,m,tsol_c);

%% Linear quadmodel solve
close all
[tsol,ysol] = quadmodel_linear([tu(1):1e-3:tu(end)],x0,ixx,iyy,izz,l,m,u,tsol_c);
