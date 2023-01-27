#include "IMU.h"
#include "tim.h"
#include "math.h"
#include "MPU6050.h"
#include "ROS_ShareWare.h"
uint16_t Get_Time_Micros(){
	return TIM6->CNT;
}
#define GYRO_FILTER_NUM 10
#define IMU_KALMAN_Q        0.02
#define IMU_KALMAN_R        6.0000


volatile uint16_t lastUpdate, now; // ≤…—˘÷‹∆⁄º∆ ˝ µ•Œª us
float q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // quaternion elements representing the estimated orientation
struct _angle angle;
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
struct _sensor sensor;
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
float Pitch=0;
float Roll=0;
float Yaw=0;
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
//    float hx, hy, hz, bx, bz;
    float vx, vy, vz;//, wx, wy, wz;
    float ex, ey, ez,halfT;
    float tempq0,tempq1,tempq2,tempq3;

    float q0q0 = q0*q0;
    float q0q1 = q0*q1;
    float q0q2 = q0*q2;
//    float q0q3 = q0*q3;
    float q1q1 = q1*q1;
//    float q1q2 = q1*q2;
    float q1q3 = q1*q3;
    float q2q2 = q2*q2;   
    float q2q3 = q2*q3;
    float q3q3 = q3*q3;   

		
	  now = Get_Time_Micros();  //∂¡»° ±º‰ µ•Œª «us   
    if(now<lastUpdate)
    {
    halfT =  ((float)(now + (0xffff- lastUpdate)) / 2000000.0f);   //  uint 0.5s
    }
    else	
    {
       halfT =  ((float)(now - lastUpdate) / 2000000.0f);
    }
		
    lastUpdate = now;	//∏¸–¬ ±º‰

    //øÏÀŸ«Û∆Ω∑Ω∏˘À„∑®
    norm = invSqrt(ax*ax + ay*ay + az*az);       
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // estimated direction of gravity and flux (v and w)
    vx = 2.0f*(q1q3 - q0q2);
    vy = 2.0f*(q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = (ay*vz - az*vy);// + (my*wz - mz*wy);
    ey = (az*vx - ax*vz);// + (mz*wx - mx*wz);
    ez = (ax*vy - ay*vx);// + (mx*wy - my*wx);

    if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
			exInt = exInt + ex * Kii * halfT;
			eyInt = eyInt + ey * Kii * halfT;	
			ezInt = ezInt + ez * Kii * halfT;
			// ”√≤Êª˝ŒÛ≤Ó¿¥◊ˆPI–ﬁ’˝Õ”¬›¡„∆´
			gx = gx + Kp*ex + exInt;
			gy = gy + Kp*ey + eyInt;
			gz = gz + Kp*ez + ezInt;
    }
    // Àƒ‘™ ˝Œ¢∑÷∑Ω≥Ã
    tempq0 = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    tempq1 = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    tempq2 = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    tempq3 = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

    // Àƒ‘™ ˝πÊ∑∂ªØ
    norm = invSqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
    q0 = tempq0 * norm;
    q1 = tempq1 * norm;
    q2 = tempq2 * norm;
    q3 = tempq3 * norm;
		
		angle.yaw= -atan2(2 * q1 * q2 + 2 * q0* q3, -2 * q2*q2 - 2 * q3 * q3 + 1)*RtA; // yaw        -pi----pi
    angle.pitch= -asin(-2 * q1 * q3 + 2 * q0 * q2)*RtA; // pitch    -pi/2    --- pi/2 
    angle.roll= atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RtA; // roll       -pi-----pi 
		Pitch=angle.pitch;
		Roll=angle.roll;
		Yaw=angle.yaw;
}	

	static double KalmanFilter_x(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=‘Î…˘
   kg=p_mid/(p_mid+R); //kgŒ™kalman filter£¨RŒ™‘Î…˘
   x_now=x_mid+kg*(ResrcData-x_mid);//π¿º∆≥ˆµƒ◊Ó”≈÷µ
                
   p_now=(1-kg)*p_mid;//◊Ó”≈÷µ∂‘”¶µƒcovariance       
   p_last = p_now; //∏¸–¬covariance÷µ
   x_last = x_now; //∏¸–¬œµÕ≥◊¥Ã¨÷µ
   return x_now;                
 }
static double KalmanFilter_y(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=‘Î…˘
   kg=p_mid/(p_mid+R); //kgŒ™kalman filter£¨RŒ™‘Î…˘
   x_now=x_mid+kg*(ResrcData-x_mid);//π¿º∆≥ˆµƒ◊Ó”≈÷µ
                
   p_now=(1-kg)*p_mid;//◊Ó”≈÷µ∂‘”¶µƒcovariance       
   p_last = p_now; //∏¸–¬covariance÷µ
   x_last = x_now; //∏¸–¬œµÕ≥◊¥Ã¨÷µ
   return x_now;                
 }
static double KalmanFilter_z(const double ResrcData,double ProcessNiose_Q,double MeasureNoise_R)
{
   double R = MeasureNoise_R;
   double Q = ProcessNiose_Q;
   static double x_last;
   double x_mid = x_last;
   double x_now;
   static double p_last;
   double p_mid ;
   double p_now;
   double kg;        

   x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)
   p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=‘Î…˘
   kg=p_mid/(p_mid+R); //kgŒ™kalman filter£¨RŒ™‘Î…˘
   x_now=x_mid+kg*(ResrcData-x_mid);//π¿º∆≥ˆµƒ◊Ó”≈÷µ
                
   p_now=(1-kg)*p_mid;//◊Ó”≈÷µ∂‘”¶µƒcovariance       
   p_last = p_now; //∏¸–¬covariance÷µ
   x_last = x_now; //∏¸–¬œµÕ≥◊¥Ã¨÷µ
   return x_now;                
}


	
	float Gyro_File_Buf[3][GYRO_FILTER_NUM];

void PrepareForIMU(uint8_t flag)
{
	float sumx,sumy,sumz;//sum_yaw		
	static uint8_t gyro_filter_cnt = 0;
	int i =0;
		
	if(flag == 0) 
			return;
	else
		{
		
		HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_DevAddr, MPU6050_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t*)MPU6050.Buf, 14);//I2C∑¢ÀÕ ˝æ›
		sensor.acc.origin.x = ((((int16_t)MPU6050.Buf[0]) << 8) | MPU6050.Buf[1]) ;
		sensor.acc.origin.y = ((((int16_t)MPU6050.Buf[2]) << 8) | MPU6050.Buf[3]) ;
		sensor.acc.origin.z = ((((int16_t)MPU6050.Buf[4]) << 8) | MPU6050.Buf[5]);
		
		sensor.gyro.origin.x = ((((int16_t)MPU6050.Buf[8]) << 8) | MPU6050.Buf[9])- sensor.gyro.quiet.x;
		sensor.gyro.origin.y = ((((int16_t)MPU6050.Buf[10]) << 8)| MPU6050.Buf[11])- sensor.gyro.quiet.y;
		sensor.gyro.origin.z = ((((int16_t)MPU6050.Buf[12]) << 8)| MPU6050.Buf[13])- sensor.gyro.quiet.z;
		
		Gyro_File_Buf[0][gyro_filter_cnt] = sensor.gyro.origin.x ;
		Gyro_File_Buf[1][gyro_filter_cnt] = sensor.gyro.origin.y ;
		Gyro_File_Buf[2][gyro_filter_cnt] = sensor.gyro.origin.z ;
		MPU6050_Update(&MPU6050);
    MPU6050_UpdateAccel(&MPU6050);
    MPU6050_UpdateGyro(&MPU6050);

			sumx = 0;
			sumy = 0;
			sumz = 0;
		for(i=0;i<GYRO_FILTER_NUM;i++)
		{
			sumx += Gyro_File_Buf[0][i];
			sumy += Gyro_File_Buf[1][i];
			sumz += Gyro_File_Buf[2][i];
		}

		
		gyro_filter_cnt = ( gyro_filter_cnt + 1 ) % GYRO_FILTER_NUM;
		
		sensor.gyro.radian.x  = sumx / (float)GYRO_FILTER_NUM * Gyro_Gr;
		sensor.gyro.radian.y  = sumy / (float)GYRO_FILTER_NUM * Gyro_Gr;
		sensor.gyro.radian.z  = sumz / (float)GYRO_FILTER_NUM * Gyro_Gr;
		
		
		sensor.acc.averag.x = KalmanFilter_x(sensor.acc.origin.x,IMU_KALMAN_Q,IMU_KALMAN_R);  // ACC X÷·ø®∂˚¬¸¬À≤®
		sensor.acc.averag.y = KalmanFilter_y(sensor.acc.origin.y,IMU_KALMAN_Q,IMU_KALMAN_R);  // ACC Y÷·ø®∂˚¬¸¬À≤®
		sensor.acc.averag.z = KalmanFilter_z(sensor.acc.origin.z,IMU_KALMAN_Q,IMU_KALMAN_R);  // ACC Z÷·ø®∂˚¬¸¬À≤
	//”√µΩACC?
		IMUupdate(sensor.gyro.radian.x,
							sensor.gyro.radian.y,
						  sensor.gyro.radian.z,
							sensor.acc.averag.x,
							sensor.acc.averag.y,
							sensor.acc.averag.z);	

	}
}
/***************************************************************************************
  * @∫Ø ˝√Ë ˆ£∫  Õ”¬›“«¡„µ„–£◊º
  * @»Îø⁄≤Œ ˝£∫  Œﬁ.
  * @∑µªÿ÷µ  :   Œﬁ.
****************************************************************************************/
int32_t  first_x=0, first_y=0,first_z=0;
void First_Gyro_OFFEST(void)
{
 
		HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_DevAddr, MPU6050_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t*)MPU6050.Buf, 14);//I2C∑¢ÀÕ ˝æ›
	first_x = ((((int16_t)MPU6050.Buf[8]) << 8) | MPU6050.Buf[9]);
	first_y = ((((int16_t)MPU6050.Buf[10]) << 8)| MPU6050.Buf[11]);
	first_z = ((((int16_t)MPU6050.Buf[12]) << 8)| MPU6050.Buf[13]);	 
  

}

//float gyro_watch; 
   int32_t  Sum_x=0,Sum_y=0,Sum_z=0;	
void Gyro_OFFEST(void)
{

   int cnt_g=50000;
	 int cnt = cnt_g;
	 float  tempgx=0,tempgy=0,tempgz=0;
	
	
	 sensor.gyro.averag.x=0;    //¡„µ„∆´“∆«Â¡„
	 sensor.gyro.averag.y=0;  
	 sensor.gyro.averag.z=0;
//	 sensor2.gyro.averag.z=0;
	 First_Gyro_OFFEST();
		  
	 while(cnt_g--)       //—≠ª∑≤…ºØ1000¥Œ   «Û∆Ωæ˘
	 {
		HAL_I2C_Mem_Read_IT(&hi2c1, MPU6050_DevAddr, MPU6050_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, (uint8_t*)MPU6050.Buf, 14);//I2C∑¢ÀÕ ˝æ›
		 
		  sensor.gyro.origin.x = ((((int16_t)MPU6050.Buf[8]) << 8) | MPU6050.Buf[9]);
	    sensor.gyro.origin.y = ((((int16_t)MPU6050.Buf[10]) << 8)| MPU6050.Buf[11]);
	    sensor.gyro.origin.z = ((((int16_t)MPU6050.Buf[12]) << 8)| MPU6050.Buf[13]);
		 
      tempgx+= sensor.gyro.origin.x;
			tempgy+= sensor.gyro.origin.y;
			tempgz+= sensor.gyro.origin.z;
		 		 
		
		  Sum_x+=(sensor.gyro.origin.x-first_x);
		  Sum_y+=(sensor.gyro.origin.y-first_y);
		  Sum_z+=(sensor.gyro.origin.z-first_z);
		 
 }
	 sensor.gyro.quiet.x=tempgx/cnt;
	 sensor.gyro.quiet.y=tempgy/cnt;
	 sensor.gyro.quiet.z=tempgz/cnt;
		MPU6050_Update(&MPU6050);
    MPU6050_UpdateAccel(&MPU6050);
    MPU6050_UpdateGyro(&MPU6050);

}
#define K_ANGLESPEED_2_ANGLE 0.000015f //Õ”¬›“«ª˝∑÷ªÒµ√Ω«∂»œµ ˝
volatile uint16_t A_lastUpdate, A_now,A_us; // ≤…—˘÷‹∆⁄º∆ ˝ µ•Œª us
#define Err_Ange						0.0000000065//Õ”¬›“«YAW÷·–ﬁ’˝

			float yaw_mid=0;
			float yaw_err=0;
struct Attitude att={0};

void Attitude_Update(struct Attitude *Attitude){
		  A_now = Get_Time_Micros();  //∂¡»° ±º‰ µ•Œª «us   
    if(A_now<A_lastUpdate)
    {
    A_us =  (float)(A_now + (0xffff- A_lastUpdate));   //  1us
    }
    else	
    {
     A_us =  (float)(A_now - A_lastUpdate) ;
    }
		A_lastUpdate=A_now;
//	Attitude.Yaw_speed=-sensor.gyro.radian.z / Gyro_Gr;
//	Attitude.Yaw+= (Attitude.Yaw_speed* K_ANGLESPEED_2_ANGLE);//Õ”¬›“«Ω«ÀŸ∂»ª˝∑÷¿€º”µ√µΩµƒµ±«∞YAWΩ«∂»

	yaw_mid=angle.yaw;
	yaw_err+=A_us*Err_Ange;	
	Attitude->Yaw=yaw_mid+yaw_err;
	Attitude->Roll =-angle.roll;
	Attitude->Pitch =-angle.pitch;
	att.Pitch=Attitude->Pitch;
	att.Roll=Attitude->Roll;
	att.Yaw=Attitude->Yaw;
}


void IMU2PC_Updata(struct PCdataUp *data){
	
	data->Acc_x=MPU6050.Accel_X;
	data->Acc_y=MPU6050.Accel_Y;
	data->Acc_z=MPU6050.Accel_Z;
	data->Gyro_x=MPU6050.Gyro_X;
	data->Gyro_y=MPU6050.Gyro_Y;
	data->Gyro_z=MPU6050.Gyro_Z;
	data->Pitch=att.Pitch;
	data->Yaw=att.Yaw;
	data->Roll=att.Roll;
	

}

