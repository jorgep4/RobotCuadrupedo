#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "websocket.hpp"
#define L_FEMUR (float) 0.08
#define L_HOMBRO (float)0.005
#define L_RODILLA (float)0.10
// extern float incline[3];//Inclinaciones
// extern float coordinates[3];//Coordenadas
// extern bool spin;//Girar?
// extern int gait;//Tipo de paso

float get_a1_angle(float z, float y);
void compute_kinematics(float coordinates[],float rot[],bool is_right,int id,float theta[3]);
void rotatematrix(float rotation[],float rotated_matrix[][3],int inverse);
// void movimiento(int type,float incline[3],float coordinates[3],bool spin);
void movimiento();
void move_leg1(float theta[3]);
void move_leg2(float theta[3]);
void move_leg3(float theta[3]);
void move_leg4(float theta[3]);
void computepid(float incline_to_write[3]);