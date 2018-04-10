/*
*******************************************************************************
* File name     : drive.h 
* Programmer(s) : Jeong-ho Park
* Description   : 
*                 
*******************************************************************************
*/

#ifndef DRIVE_SENSOR_H
#define DRIVE_SENSOR_H


/*
*******************************************************************************
*                              TYPE DEFINITIONS
*******************************************************************************
*/
#define IP_ADDR_TETRA_DS "192.168.51.10"
#define DRIVE_PORT 50010


/*
*******************************************************************************
*                             FUNCTION PROTOTYPES
*******************************************************************************
*/
int main();
bool winsock2_open();
void winsocke2_close();
bool get_encoder_handler();
bool get_odometry_handler();
bool set_velocity_handler(int left_velocity, int right_velocity);

#endif