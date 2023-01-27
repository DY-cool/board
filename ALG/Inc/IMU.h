#ifndef __IMU_H_
#define __IMU_H_

#define Kp 1.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Kii 0.01f                     // integral gain governs rate of convergence of gyroscope biases
#define RtA 		57.324841f		//  180/3.1415  角度制 转化为弧度制	
#define Acc_G 		0.0011963f		//  1/32768/4/9.8     加速度量程为4G		
#define Gyro_G 		0.03051756f	//  1/32768/1000      陀螺仪量程为 +—1000			
#define Gyro_Gr		0.0005327f   //  1/32768/1000/57.3 


#define	GYRO_Y		0x45
#define	GYRO_Z		0x47
#include "stm32f1xx.h"

struct _int16{
       int16_t x;
	     int16_t y;
	     int16_t z;};	
typedef struct{
				float X;
				float Y;
				float Z;}FLOAT_XYZ;

struct _float{
	      float x;
				float y;
				float z;};
	

struct _trans{
     struct _int16 origin;  //原始值
	   struct _int16 averag;  //平均值
	   struct _int16 histor;  //历史值
	   struct _int16 quiet;   //静态值
	   struct _float radian;  //弧度值 
          };

struct _sensor{   
	struct _trans acc;
	struct _trans gyro;
              };

extern struct _sensor sensor;	
struct _angle{
        float pitch;
				float roll;
        float yaw;
				float val;
};

struct Attitude{
	float Yaw;
	float Yaw_speed;
	float Pitch;
	float Roll;
	
};


struct PCdataUp{

	float		Acc_x;
	float		Acc_y;
	float		Acc_z;
	float		Gyro_x;
	float		Gyro_y;
	float		Gyro_z;
	float	Yaw;
	float	Pitch;
	float	Roll;
	
};
extern struct Attitude Attitude;
float invSqrt(float x);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
void Gyro_OFFEST(void);
void PrepareForIMU(uint8_t flag);
void IMU2PC_Updata(struct PCdataUp *data);
void Attitude_Update(struct Attitude *Attitude);
#endif
				
