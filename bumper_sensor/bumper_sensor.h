/*
*******************************************************************************
* File name     : bumper_sensor.h 
* Programmer(s) : Jeong-ho Park
* Description   : 
*                 
*******************************************************************************
*/

#ifndef BUMPER_SENSOR_H
#define BUMPER_SENSOR_H


/*
*******************************************************************************
*                              TYPE DEFINITIONS
*******************************************************************************
*/
#define IP_ADDR_TETRA_DS	"192.168.51.10"
#define BUMPER_PORT			50011


/*
*******************************************************************************
*                             FUNCTION PROTOTYPES
*******************************************************************************
*/
int main();
bool winsock2_open();
void winsocke2_close();	
bool set_bumper_off_mode();
bool set_bumper_dir_mode();
bool bumper_handler();

#endif