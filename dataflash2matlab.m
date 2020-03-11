log = Ardupilog('hover.bin');
close all
MOTOR_VOLTAGE = 12;
MOTOR_KV = 1000;
PWM_TPERIOD = 1000;
DUTYCYCLE2RADS = MOTOR_VOLTAGE/PWM_TPERIOD*MOTOR_KV*2*3.14/60; 
k = 1.9794e-7 * 3600/(2*3.14)^2;
b = 6.5124e-9* 3600/(2*3.14)^2;

u1 = (log.RCOU.C1-1000)*DUTYCYCLE2RADS;
u2 = (log.RCOU.C2-1000)*DUTYCYCLE2RADS;
u3 = (log.RCOU.C3-1000)*DUTYCYCLE2RADS;
u4 = (log.RCOU.C4-1000)*DUTYCYCLE2RADS;
t = log.RCOU.TimeS;
t = t - t(1);
log_t1 = -k*u1.^2;
log_t2 = -k*u2.^2;
log_t3 = -k*u3.^2;
log_t4 = -k*u4.^2;
log_tau1 = b*u1.^2;
log_tau2 = b*u2.^2;
log_tau3 = b*u3.^2;
log_tau4 = b*u4.^2;


lat = log.AHR2.Lat;
lng = log.AHR2.Lng;
alt = log.AHR2.Alt;
vx = log.NKF1.VN;
vy = log.NKF1.VE;
vz = log.NKF1.VD;

roll = log.AHR2.Roll*pi/180;
pitch = log.AHR2.Pitch*pi/180;
yaw = log.AHR2.Yaw*pi/180;

p = log.RATE.R*pi/180;
q = log.RATE.P*pi/180;
r = log.RATE.Y*pi/180;

