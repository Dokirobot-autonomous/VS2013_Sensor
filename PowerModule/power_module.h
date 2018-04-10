/*
*******************************************************************************
* File name     : power_module.h 
* Programmer(s) : Jeong-ho Park
* Description   : 
*                 
*******************************************************************************
*/

#ifndef POWER_MODULE_H
#define POWER_MODULE_H


/*
*******************************************************************************
*                              TYPE DEFINITIONS
*******************************************************************************
*/
#define IP_ADDR_TETRA_DS	"192.168.51.10"
#define POWER_PORT			50017


/*
*******************************************************************************
*                             FUNCTION PROTOTYPES
*******************************************************************************
*/
int main();
bool winsock2_open();
void winsocke2_close();	
bool set_drive_up();
bool set_drive_down();
bool set_drive_reset();
bool set_sensor_up();
bool set_sensor_down();
bool set_sensor_reset();
bool set_lrf_up();
bool set_lrf_down();
bool set_lrf_reset();
bool power_module_handler();

#endif