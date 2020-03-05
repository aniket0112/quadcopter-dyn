close all
MOTOR_VOLTAGE = 12;
MOTOR_KV = 1000;
PWM_TPERIOD = 1000;
DUTYCYCLE2RADS = MOTOR_VOLTAGE/PWM_TPERIOD*MOTOR_KV*2*3.14/60; 

PROP_DIA = 10; %inch
PROP_PITCH = 4.5; %inch
k = 1.9794e-7 * 3600/(2*3.14)^2;
b = 6.5124e-9* 3600/(2*3.14)^2;

tlogread = mavlinktlog('fdata.tlog');

%attitude log
attitudeMsg = readmsg(tlogread,'MessageName','ATTITUDE');
attitude = attitudeMsg.Messages{1};
attitude.yaw = atan(tan(cast(attitude.yaw,'double')));

%esc duty cycle log to thrust
motorMsg = readmsg(tlogread,'MessageName','SERVO_OUTPUT_RAW');
motor = motorMsg.Messages{1};
u1 = (cast(motor.servo1_raw,'double') - 1000)*DUTYCYCLE2RADS;
u2 = (cast(motor.servo2_raw,'double') - 1000)*DUTYCYCLE2RADS;
u3 = (cast(motor.servo3_raw,'double') - 1000)*DUTYCYCLE2RADS;
u4 = (cast(motor.servo4_raw,'double') - 1000)*DUTYCYCLE2RADS;

t1 = -k*u1.^2;
t2 = -k*u2.^2;
t3 = -k*u3.^2;
t4 = -k*u4.^2;
tau1 = b*u1.^2;
tau2 = b*u2.^2;
tau3 = b*u3.^2;
tau4 = b*u4.^2;

%gps+imu position and position rate log
positionMsg = readmsg(tlogread,'MessageName','GLOBAL_POSITION_INT');
position = positionMsg.Messages{1};
position.lat = cast((position.lat - position.lat(1)),'double')*1e-7*(111.32e3);
position.lon = cast((position.lon - position.lon(1)),'double')*1e-7*(111.32e3);
position.alt = cast(position.alt,'double')*1e-3;
position.vx = cast(position.vx,'double')*1e-2;
position.vy = cast(position.vy,'double')*1e-2;
position.vz = cast(position.vz,'double')*1e-2;

plot3(position.lat,position.lon,position.alt)