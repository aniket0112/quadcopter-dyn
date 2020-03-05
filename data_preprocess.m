load('fdata.mat')
MOTOR_VOLTAGE = 12;
MOTOR_KV = 1000;
PWM_TPERIOD = 1000;
DUTYCYCLE2RPM = MOTOR_VOLTAGE/PWM_TPERIOD*MOTOR_KV; 

PROP_DIA = 10; %inch
PROP_PITCH = 4.5; %inch
t0 = 7.378498326651736e+05;
u1 = (servo1_raw_mavlink_servo_output_raw_t(servo1_raw_mavlink_servo_output_raw_t(:,1)>t0,:)-1000)*DUTYCYCLE2RPM;
u2 = (servo2_raw_mavlink_servo_output_raw_t(servo2_raw_mavlink_servo_output_raw_t(:,1)>t0,:)-1000)*DUTYCYCLE2RPM;
u3 = (servo3_raw_mavlink_servo_output_raw_t(servo3_raw_mavlink_servo_output_raw_t(:,1)>t0,:)-1000)*DUTYCYCLE2RPM;
u4 = (servo4_raw_mavlink_servo_output_raw_t(servo4_raw_mavlink_servo_output_raw_t(:,1)>t0,:)-1000)*DUTYCYCLE2RPM;

x = (lat_mavlink_ahrs3_t(lat_mavlink_ahrs3_t(:,1)>t0,2)-lat_mavlink_ahrs3_t(1,:))*1e-7*(111.32e3);    
y = (lng_mavlink_ahrs3_t(lng_mavlink_ahrs3_t(:,1)>t0,2)-lng_mavlink_ahrs3_t(1,:))*1e-7*(111.32e3);    
z = altitude_mavlink_ahrs3_t(altitude_mavlink_ahrs3_t(:,1)>t0,:);                                          
u = vx_mavlink_global_position_int_t(vx_mavlink_global_position_int_t(:,1)>t0,:)*1e-2;                                   
v = vy_mavlink_global_position_int_t(vy_mavlink_global_position_int_t(:,1)>t0,:)*1e-2;
w = vz_mavlink_global_position_int_t(vz_mavlink_global_position_int_t(:,1)>t0,:)*1e-2;


roll = roll_mavlink_ahrs3_t(roll_mavlink_ahrs3_t(:,1)>t0,:);
pitch = pitch_mavlink_ahrs3_t(pitch_mavlink_ahrs3_t(:,1)>t0,:);
yaw = yaw_mavlink_ahrs3_t(yaw_mavlink_ahrs3_t(:,1)>t0,:);
p = rollspeed_mavlink_attitude_t(rollspeed_mavlink_attitude_t(:,1)>t0,:);
q = pitchspeed_mavlink_attitude_t(pitchspeed_mavlink_attitude_t(:,1)>t0,:);
r = yawspeed_mavlink_attitude_t(yawspeed_mavlink_attitude_t(:,1)>t0,:);

% t1 = 4.39e-8*PROP_DIA^3.5/sqrt(PROP_PITCH)*u1(:,2).*(4.233e-4*PROP_PITCH*u1);
% t2 = 4.39e-8*PROP_DIA^3.5/sqrt(PROP_PITCH)*u2(:,2).*(4.233e-4*PROP_PITCH*u1);
% t3 = 4.39e-8*PROP_DIA^3.5/sqrt(PROP_PITCH)*u3(:,2).*(4.233e-4*PROP_PITCH*u1);
% t4 = 4.39e-8*PROP_DIA^3.5/sqrt(PROP_PITCH)*u4(:,2).*(4.233e-4*PROP_PITCH*u1);
