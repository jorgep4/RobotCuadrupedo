#include <math.h>
#include "stdio.h"
#include <robotfunctions.hpp>
#include "PCA9685.hpp"
#include "MPU6050.hpp"
#include "globals.hpp"

float pitch_data;
float roll_data;
float yaw_data;
long int last_time;
long int current_time;
long int elapsed_time;
float last_error_pitch;
float last_error_roll;
float last_error_yaw;
float error_p_pitch;
float error_i_pitch;
float error_d_pitch;
float error_p_roll;
float error_i_roll;
float error_d_roll;
float error_p_yaw;
float error_i_yaw;
float error_d_yaw;
float kp,ki,kd;
float offset_gyro,offset_ax,offset_ay,offset_az;


void compute_kinematics(float coordinates[],float rot[],bool is_right,int id,float theta[3]){
    float x= coordinates[0];
    float y=coordinates[1];
    float z=coordinates[2];
    float coordinates_f[3];
    float A;
    float alpha1;
    float alpha2;
    float alpha3;
    float div=2.0;
    float altura=0;
    float anchura=0.125;
    float longitud=0.245;
    float origin_matrix_legs[4][3]={{longitud/div,anchura/div,0},{-longitud/div,anchura/div,0},{-longitud/div,-anchura/div,0},{longitud/div,-anchura/div,0}};
    float origin_matrix_body[4][3]={{longitud/2,anchura/2,altura},{-longitud/2,anchura/2,altura},{-longitud/2,-anchura/2,altura},{longitud/2,-anchura/2,altura}};
    int left_front=0;
    int left_back=1;
    int right_front=2;
    int right_back=3;
    float rotinvmatrix[3][3];
    rotatematrix(rot,rotinvmatrix,1);//*coordinates+leg_origin
    for (int i = 0; i < 3; ++i) {
        coordinates_f[i] = 0;
        for (int j = 0; j < 3; ++j) {
            coordinates_f[i] += rotinvmatrix[i][j] * (coordinates[j]+origin_matrix_legs[id][j]);
        }
        coordinates_f[i]=coordinates_f[i]-origin_matrix_legs[id][i];

    }

    A=sqrt(pow(coordinates_f[1],2)+pow(coordinates_f[2],2));
    // printf("A %f",A);
    alpha1=get_a1_angle(coordinates_f[2],coordinates_f[1]);
    alpha2=asin(sin(M_PI/2.0)*L_HOMBRO/A);
    alpha3=-alpha2+M_PI/2.0;
    // printf("a1 %f\n",alpha1);
    // printf("a2 %f\n",alpha2);
    // printf("a3 %f\n",alpha3);


    if (is_right==1){
        theta[0]=alpha1-alpha3;
    }
    else{
        theta[0]=alpha1+alpha3;
        if (theta[0]>(2*M_PI)){
            theta[0]=-2*M_PI;
        }
    }

    float arti_hombro[3];
    arti_hombro[0]=0;
    arti_hombro[1]=L_HOMBRO*cos(theta[0]);
    arti_hombro[2]=L_HOMBRO*sin(theta[0]);
    float arti_vector[3];
    arti_vector[0]=coordinates_f[0]-arti_hombro[0];
    arti_vector[1]=coordinates_f[1]-arti_hombro[1];
    arti_vector[2]=coordinates_f[2]-arti_hombro[2];
    float rotation[3];
    if (is_right==1){
        rotation[0] = -((theta[0]-M_PI)*180/M_PI);
    }
    else{
        rotation[0] = -((theta[0])*180/M_PI);//+PHI+PI/2
    }
    float rotated_matrix[3][3];
    float axis_rotated[3]={0,0,0};
    rotation[1]=0;
    rotation[2]=0;
    // printf("-R %f\n",rotation[0]);
    rotatematrix(rotation,rotated_matrix,0);
    for(int i = 0; i < 3; ++i) {
        axis_rotated[i] = 0;
        for(int j = 0; j < 3; ++j) {
                axis_rotated[i] += rotated_matrix[i][j]*arti_vector[j];
        }
        // printf("Axis %f\n",axis_rotated[i]);
    }
    float xrot=axis_rotated[0];
    float yrot=axis_rotated[1];
    float zrot=axis_rotated[2];
    float B;
    B=sqrt(pow(xrot,2)+pow(zrot,2));
    // printf("B %f",B);
    if (B>=(L_FEMUR+L_RODILLA)){
        B=(L_FEMUR+L_RODILLA)*0.999;//Coordenada lejana
    }
    // printf("B %f",B);
    float beta1;
    float beta2;
    float beta3;
    beta1=get_a1_angle(z,x);
    beta2=acos((pow(L_FEMUR,2)+pow(B,2)-pow(L_RODILLA,2))/(2*L_FEMUR*B));
    beta3=acos((pow(L_FEMUR,2)-pow(B,2)+pow(L_RODILLA,2))/(2*L_FEMUR*L_RODILLA));
    theta[0]=theta[0]*180/M_PI;
    theta[1]=(beta1-beta2)*180/M_PI;//beta1-beta2
    theta[2]=(M_PI-beta3)*180/M_PI;///180;//M_PI-beta3
    printf("xerot %f\n",theta[0]);
    printf("yerot %f\n",theta[1]);
    printf("zerot %f\n",theta[2]);
}
void rotatematrix(float rotation[],float rotated_matrix[3][3],int inverse){
    float roll=rotation[0]*M_PI/180;
    float pitch=rotation[1]*M_PI/180;
    float yaw=rotation[2]*M_PI/180;
    // Calcular las matrices de rotación
    float rotx[3][3] = {
        {1, 0, 0},
        {0, cos(roll), -sin(roll)},
        {0, sin(roll), cos(roll)}
    };

    float roty[3][3] = {
        {cos(pitch), 0, sin(pitch)},
        {0, 1, 0},
        {-sin(pitch), 0, cos(pitch)}
    };

    float rotz[3][3] = {
        {cos(yaw), -sin(yaw), 0},
        {sin(yaw), cos(yaw), 0},
        {0, 0, 1}
    };
    // Multiplicación de matrices rotz * roty
    float temp_matrix[3][3];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            temp_matrix[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                temp_matrix[i][j] += rotz[i][k] * roty[k][j];
            }

        }
    }

    // Multiplicación de matrices temp_matrix * rotx
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotated_matrix[i][j] = 0;
            for (int k = 0; k < 3; ++k) {
                rotated_matrix[i][j] += temp_matrix[i][k] * rotx[k][j];
                
            }
        }
    }
    if (inverse==1){
        float det=0;
        float adjoint[3][3];
        det = rotated_matrix[0][0] * rotated_matrix[1][1] * rotated_matrix[2][2] +
          rotated_matrix[0][1] * rotated_matrix[1][2] * rotated_matrix[2][0] +
          rotated_matrix[0][2] * rotated_matrix[1][0] * rotated_matrix[2][1] -
          rotated_matrix[0][2] * rotated_matrix[1][1] * rotated_matrix[2][0] -
          rotated_matrix[0][1] * rotated_matrix[1][0] * rotated_matrix[2][2] -
          rotated_matrix[0][0] * rotated_matrix[1][2] * rotated_matrix[2][1];
        //   printf ("DET %f",det);
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float cofactorSign = ((i + j) % 2 == 0) ? 1 : -1;
            float minorMatrix[2][2];
            int minorRow = 0, minorCol = 0;
            for (int row = 0; row < 3; ++row) {
                for (int col = 0; col < 3; ++col) {
                    if (row != i && col != j) {
                        minorMatrix[minorRow][minorCol++] = rotated_matrix[row][col];
                        if (minorCol == 2) {
                            minorCol = 0;
                            minorRow++;
                        }
                    }
                }
            }
            adjoint[j][i] = cofactorSign * (minorMatrix[0][0]*minorMatrix[1][1]- minorMatrix[0][1]*minorMatrix[1][0]);
        }
    }
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotated_matrix[i][j] = adjoint[i][j] / det;
        }
    }

    }
    
    
}
float get_a1_angle(float z, float y){
    if (y > 0 and z >= 0){ 
        return (atan(z/(y)));
    }
    if (y == 0 and z >= 0){ 
        return M_PI/2;
    }
    if (y < 0 and z >= 0){ 
        return (-abs(atan(z/y)) + M_PI);
    }
    if (y < 0 and z < 0){
        return (atan(z/y) + M_PI);
    }
    if (y > 0 and z < 0){ 
        return (-abs(atan(z/y)) + 2*M_PI);
    }
    if (y == 0 and z < 0){
        return (M_PI * 3/2);
    }
    else {
        if (y == 0 and z == 0){
            return (M_PI * 3/2);
        }
    }
    return 0;
}
void movimiento(){
//void movimiento(int gait,float incline[3],float coordinates[3],bool spin){
    float theta[3];
    float init[3]={0,0,-0.15};
    float incline_to_write[3];
    gait=0;
    spin=0;
    //Pruebas avanzar
    incline[0]=0.0;
    incline[1]=0;
    incline[2]=0;
    coordinates[0]=0.00;//0.05
    coordinates[1]=0.0;
    coordinates[2]=-0.15; //0.135ç
        //     compute_kinematics(init,incline,1,3,theta);
        // move_leg1(theta);
        // compute_kinematics(init,incline,0,0,theta);
        // move_leg2(theta);
        // compute_kinematics(init,incline,0,1,theta);
        // move_leg4(theta);
        // compute_kinematics(init,incline,1,2,theta);
        // move_leg3(theta);
        // vTaskDelay(pdMS_TO_TICKS(100));

    //cambiar incline_to_write por incline incline tiene el setpoint, computar pid previo a cualquier paso
    /*
    Tipos 
    WALK 0
    PACE 1
    TROT 2
    GALLOP 3
    STATIC 4
    ROTATE 5
    */
   switch(gait){
        case 0:
        //-0.14
        //Piernas en 0,0
        compute_kinematics(init,incline,1,3,theta); //DCHA TRASERA
        move_leg1(theta);

        compute_kinematics(init,incline,0,0,theta);
        move_leg2(theta);

        compute_kinematics(init,incline,0,1,theta);
        move_leg4(theta);

        compute_kinematics(init,incline,1,2,theta);
        move_leg3(theta);
        vTaskDelay(pdMS_TO_TICKS(100));
        // Avanzar pierna izda trasera
        compute_kinematics(coordinates,incline,0,0,theta);
        move_leg2(theta);
        vTaskDelay(pdMS_TO_TICKS(100));
        ////Piernas en 0,0
        compute_kinematics(init,incline,0,0,theta);
        move_leg2(theta);
        vTaskDelay(pdMS_TO_TICKS(100));
        //Avanzar izda delantera
        compute_kinematics(coordinates,incline,0,1,theta);
        move_leg4(theta);
        vTaskDelay(pdMS_TO_TICKS(100));
        //Volver a origen
        compute_kinematics(init,incline,0,1,theta);
        move_leg4(theta);
        vTaskDelay(pdMS_TO_TICKS(100));
        //Avanzar pierna dcha
        compute_kinematics(coordinates,incline,1,3,theta);
        move_leg1(theta);
        vTaskDelay(pdMS_TO_TICKS(100));
        //Piernas en 0,0
        compute_kinematics(init,incline,1,3,theta);
        move_leg1(theta);
        vTaskDelay(pdMS_TO_TICKS(100));
        //Avanzar pierna dcha delantera
                compute_kinematics(coordinates,incline,1,2,theta);
        move_leg3(theta);
        vTaskDelay(pdMS_TO_TICKS(100));
        //volver origen
                compute_kinematics(init,incline,1,2,theta);
        move_leg3(theta);
        // vTaskDelay(pdMS_TO_TICKS(100));

        if (coordinates[0]<0){
                    printf("WALK!");
        printf("WALF COORDINATES %f %f",coordinates[0],coordinates[1]);

        }
    break;
        case 1:
                //Piernas en 0,0
                              compute_kinematics(init,incline,1,3,theta);
        move_leg1(theta);
        //Avanzar pierna dcha delantera
                compute_kinematics(init,incline,1,2,theta);
        move_leg3(theta);

        // Avanzar pierna izda trasera
                compute_kinematics(coordinates,incline,0,0,theta);
        move_leg2(theta);
        //Avanzar izda delantera
                        compute_kinematics(coordinates,incline,0,1,theta);
        move_leg4(theta);
        vTaskDelay(pdMS_TO_TICKS(200));
        //Volver a origen
                      compute_kinematics(init,incline,0,0,theta);
        move_leg2(theta);
        //Avanzar izda delantera
                        compute_kinematics(init,incline,0,1,theta);
        move_leg4(theta);
        //Avanzar pierna dcha
              compute_kinematics(coordinates,incline,1,3,theta);
        move_leg1(theta);
        //Avanzar pierna dcha delantera
                compute_kinematics(coordinates,incline,1,2,theta);
        move_leg3(theta);
        vTaskDelay(pdMS_TO_TICKS(200));
        //volver origen
        //               compute_kinematics(init,incline,1,3,theta);
        // move_leg1(theta);
        //   vTaskDelay(pdMS_TO_TICKS(100));
        //                 compute_kinematics(init,incline,1,2,theta);
        // move_leg3(theta);
        // vTaskDelay(pdMS_TO_TICKS(200));
    break;
        case 2:
                                //Piernas en 0,0
        compute_kinematics(init,incline,1,3,theta);
        move_leg1(theta);
        compute_kinematics(init,incline,0,0,theta);
        move_leg2(theta);
        compute_kinematics(init,incline,0,1,theta);
        move_leg4(theta);
        compute_kinematics(init,incline,1,2,theta);
        move_leg3(theta);
              //vTaskDelay(pdMS_TO_TICKS(100));
        // Avanzar pierna dcha trasera e izda delantera
                compute_kinematics(coordinates,incline,1,3,theta);
        move_leg1(theta);
                        compute_kinematics(coordinates,incline,0,1,theta);
        move_leg4(theta);
              vTaskDelay(pdMS_TO_TICKS(100));
        //Volver a origen
                        compute_kinematics(coordinates,incline,1,3,theta);
        move_leg1(theta);
                                compute_kinematics(init,incline,0,1,theta);
        move_leg4(theta);

        //Avanzar pierna dcha delantera
                compute_kinematics(coordinates,incline,1,2,theta);
        move_leg3(theta);
               // vTaskDelay(pdMS_TO_TICKS(100));
        //Avanzar pierna izda trasera
                compute_kinematics(coordinates,incline,0,0,theta);
        move_leg2(theta);
                vTaskDelay(pdMS_TO_TICKS(100));
        //volver origen
                        compute_kinematics(init,incline,1,2,theta);
        move_leg3(theta);
                compute_kinematics(init,incline,0,0,theta);
        move_leg2(theta);
    break;
        case 3:

        // Avanzar pierna dhca trasera e izda a origen
                compute_kinematics(coordinates,incline,1,3,theta);
        move_leg1(theta);
                compute_kinematics(init,incline,0,0,theta);
        move_leg2(theta);
                vTaskDelay(pdMS_TO_TICKS(100));

        //delantera izda a origen y dcha delantera avanza
                compute_kinematics(init,incline,0,1,theta);
        move_leg4(theta);
                compute_kinematics(coordinates,incline,1,2,theta);
        move_leg3(theta);
                vTaskDelay(pdMS_TO_TICKS(100));
        //Avanzar pierna izda trasera y trasera dcha a origen
                        compute_kinematics(init,incline,1,3,theta);
        move_leg1(theta);
        compute_kinematics(coordinates,incline,0,0,theta);
        move_leg2(theta);
                vTaskDelay(pdMS_TO_TICKS(100));
        //Avanzar pierna izda delantera y dhca delantera a origen
                        compute_kinematics(init,incline,1,2,theta);
        move_leg3(theta);
        compute_kinematics(coordinates,incline,0,1,theta);
        move_leg4(theta);
                vTaskDelay(pdMS_TO_TICKS(100));

    break;
        case 4:
        // //Solo inclinaciones
        // compute_kinematics(init,incline,1,3,theta);
        // move_leg1(theta);
        // compute_kinematics(init,incline,0,0,theta);
        // move_leg2(theta);
        // compute_kinematics(init,incline,0,1,theta);
        // move_leg4(theta);
        // compute_kinematics(init,incline,1,2,theta);
        // move_leg3(theta);
    break;
        case 5:
         //Si spin 0 rotar usando las dos delanteras y luego a origen
         if (spin==0){
        compute_kinematics(coordinates,incline,0,1,theta);
        move_leg4(theta);
        compute_kinematics(coordinates,incline,1,2,theta);
        move_leg3(theta);
        //delay
        vTaskDelay(pdMS_TO_TICKS(100));
                compute_kinematics(init,incline,0,1,theta);
        move_leg4(theta);
        compute_kinematics(init,incline,1,2,theta);
        move_leg3(theta);
        vTaskDelay(pdMS_TO_TICKS(100));
         }
    //Si spin 1 rotar usando las dos traseras y luego a origen
             if (spin==0){
        compute_kinematics(coordinates,incline,1,3,theta);
        move_leg1(theta);
        compute_kinematics(coordinates,incline,0,0,theta);
        move_leg2(theta);
        //delay
        compute_kinematics(init,incline,1,3,theta);
        move_leg1(theta);
        compute_kinematics(init,incline,0,0,theta);
        move_leg2(theta);
         }

    break;

   }

   vTaskDelay(pdMS_TO_TICKS(10));


}

void move_leg1(float theta[3]){
    pca9685 pca;
    int err;
    float result[3];

    if (theta[0]>180 && theta[0]<300){

        result[0]=345-(((theta[0]-180))*2.272); //347

    }

    if (theta[0]>120 && theta[0]<180){

        result[0]=475+((-(theta[0]-121))*2.272); //Teorico 121

    }

    result[1]=370-(theta[2]*2.272);
    result[2]=((theta[1]-225)*2.272)+66;
    err=pca.pwm_control(0,0,round(result[1]));
    err=pca.pwm_control(1,0,round(result[2]));
    err=pca.pwm_control(2,0,round(result[0]));

}
void move_leg2(float theta[3]){
        pca9685 pca;
    float result[3];
    int err;
                         if ((round(theta[0]+360.0))==0){
        printf("AVISÁ");
        theta[0]=360;
       }
        if (theta[0]>0 && theta[0]<46)
       {
          result[0]=66+(((46-theta[0]))*2.272);
       }
      if (theta[0]>226 && theta[0]<=360)
       {
        result[0]=165+(((360-theta[0]))*2.272);

       }
        result[2]=475-((theta[1]-215)*2.272);
       result[1]=215+(2.272*theta[2]);
        err=pca.pwm_control(3,0,round(result[1]));
        err=pca.pwm_control(4,0,round(result[2]));
        err=pca.pwm_control(5,0,result[0]);
    
}
void move_leg3(float theta[3]){
    pca9685 pca;
    float result[3];
    int err;
                              if (theta[0]>180 && theta[0]<300)
       {
          result[0]=190+(((theta[0]-180))*2.272);
          
       }
      if (theta[0]>120 && theta[0]<180)
       {
        result[0]=190+((-(theta[0]-180))*2.272);

       }

                result[2]=((theta[1]-225)*2.272)+66;//475-((theta3[1]-130)*2.272); //215
       result[1]=350-((theta[2])*2.272);
        err=pca.pwm_control(6,0,round(result[1]));
        err=pca.pwm_control(7,0,round(result[2]));
       // printf("RESULTADO %f",result[0]);
        err=pca.pwm_control(8,0,round(result[0]));
    
}
void move_leg4(float theta[3]){
    pca9685 pca;
    float result[3];
    int err;
    if ((round(theta[0]+360.0))==0){
        theta[0]=360;
    }
    if (theta[0]>0 && theta[0]<64)  {
        result[0]=475-(((64-theta[0]))*2.272);
    }
    if (theta[0]>244 && theta[0]<=360){
result[0]=345-(((360-theta[0]))*2.272); //160
}


result[2]=475-((theta[1]-225)*2.272);
result[1]=155+((theta[2])*2.272);
err=pca.pwm_control(9,0,round(result[1]));
err=pca.pwm_control(10,0,round(result[2]));
err=pca.pwm_control(11,0,round(result[0]));



}

void computepid(float incline_to_write[3]){
    mpu6050 mpu;
    current_time=esp_timer_get_time();
    elapsed_time=current_time-last_time;
    //Obtener datos mpu
    roll_data=atan2(-mpu.accel_x(),mpu.accel_y())*180/M_PI;
    pitch_data=atan2(mpu.accel_y(),sqrt(pow(mpu.accel_x(),2)+pow(mpu.accel_y(),2)))*180/M_PI;
    yaw_data =yaw_data+ (mpu.gyro_z()- offset_gyro) * elapsed_time;
    last_time =current_time;
    //Calculo parámetros PID
    error_p_roll=incline[0]-roll_data;
    error_i_roll=error_i_roll+error_p_roll*elapsed_time;
    error_d_roll=(error_p_roll-last_error_roll)/elapsed_time;
    incline_to_write[0]=kp*error_p_roll+ki*error_i_roll+kd*error_d_roll;
    
    error_p_pitch=incline[1]-pitch_data;
    error_i_pitch=error_i_pitch+error_p_pitch*elapsed_time;
    error_d_pitch=(error_p_pitch-last_error_pitch)/elapsed_time;
    incline_to_write[1]=kp*error_p_pitch+ki*error_i_pitch+kd*error_d_pitch;

    error_p_yaw=incline[2]-yaw_data;
    error_i_yaw=error_i_yaw+error_p_yaw*elapsed_time;
    error_d_yaw=(error_p_yaw-last_error_yaw)/elapsed_time;
    incline_to_write[2]=kp*error_p_yaw+ki*error_i_yaw+kd*error_d_yaw;

    //Modificar la inclinacion del cuadrupedo
    last_error_pitch=error_p_pitch;
    last_error_roll=error_p_roll;
    last_error_yaw=error_p_yaw;

    last_time=current_time;
}
void calibrate_mpu(){
    mpu6050 mpu;
    float ax=0;
    float ay=0;
    float az=0;
    float gz=0;
    for (int i=0;i<1000;i++){

        ax+=ax;
        ay+=ay;
        az+=az;
        gz+=gz;
    }
    offset_ax=ax/1000;
    offset_ay=ay/1000;
    offset_az=az/1000;
    offset_gyro=gz/1000;
}