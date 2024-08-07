#ifndef _CALIBRATIONROUTINES_H
#define _CALIBRATIONROUTINES_H

#include "Headfile.h"

typedef struct
{
  float x_sumplain ;
  float x_sumsq;
  float x_sumcube ;
  
  float y_sumplain ;
  float y_sumsq ;
  float y_sumcube ;
  
  float z_sumplain ;
  float z_sumsq ;
  float z_sumcube ;
  
  float xy_sum ;
  float xz_sum ;
  float yz_sum ;
  
  float x2y_sum ;
  float x2z_sum ;
  float y2x_sum ;
  float y2z_sum ;
  float z2x_sum ;
  float z2y_sum ;
  
  unsigned int size;
}Least_Squares_Intermediate_Variable;


int sphere_fit_least_squares(const float x[], const float y[], const float z[],
                             unsigned int size, unsigned int max_iterations,float delta,
                             float *sphere_x, float *sphere_y, float *sphere_z,
                             float *sphere_radius);

void LS_Init(Least_Squares_Intermediate_Variable * pLSQ);
unsigned int LS_Accumulate(Least_Squares_Intermediate_Variable * pLSQ, float x, float y, float z);
void LS_Calculate(Least_Squares_Intermediate_Variable * pLSQ, unsigned int max_iterations, float delta,
                  float *sphere_x, float *sphere_y, float *sphere_z,
                  float *sphere_radius);

extern Least_Squares_Intermediate_Variable Mag_LS;
									
									
									
//typedef struct 
//{
//	float beta[4];
//	float delta[4];
//	float ds[4];
//  float js[4][4];
//	
//	float a,b,c,r;
//}sphere_fit;

//typedef struct
//{
//	float x[3][12];
//	float y[3][12];
//	float z[3][12];
//  unsigned char axis[3][12];
//}spf_data;


//void sphere_fit_namelesstech(sphere_fit *p,uint16_t number,float x[],float y[],float z[]);


#endif

