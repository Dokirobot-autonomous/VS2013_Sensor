/*
*******************************************************************************
* File name     : emergency.h 
* Programmer(s) : Jeong-ho Park
* Description   : emergency event check
*                 
*******************************************************************************
*/

#ifndef EMERGENCY_H
#define EMERGENCY_H


/*
*******************************************************************************
*                              TYPE DEFINITIONS
*******************************************************************************
*/
#define IP_ADDR_TETRA_DS	"192.168.51.10"
#define EMERGENCY_PORT		50022


/*
*******************************************************************************
*                             FUNCTION PROTOTYPES
*******************************************************************************
*/
int main();
bool winsock2_open();
void winsocke2_close();	
bool emergency_handler();

#endif