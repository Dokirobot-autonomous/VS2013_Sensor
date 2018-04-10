/*
*******************************************************************************
* File name     : lrf.h 
* Programmer(s) : Jeong-ho Park
* Description   : emergency event check
*                 
*******************************************************************************
*/

#ifndef LRF_H
#define LRF_H


/*
*******************************************************************************
*                              TYPE DEFINITIONS
*******************************************************************************
*/
#define IP_ADDR_TETRA_DS	"192.168.51.10"
#define LRF_PORT			50014


/*
*******************************************************************************
*                             FUNCTION PROTOTYPES
*******************************************************************************
*/
int main();
bool winsock2_open();
void winsocke2_close();	
bool get_degree_range(int *degree_range);
bool get_min_resolution(int *min_resolution);
bool get_max_resolution(int *max_resolution);
bool lrf_handler(int resolution);

#endif