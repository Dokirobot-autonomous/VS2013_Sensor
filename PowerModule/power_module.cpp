#include <stdio.h>
#include <winsock2.h>	//windows socket version2 init
#include "dsphal.h"		// dssphal header
#include "power_module.h"

int main()
{
	if(!winsock2_open()) return false;

	while(1) {
		power_module_handler();
		Sleep(100);
	}

	winsocke2_close();

	return 0;
}

/*
*******************************************************************************
* Function name : winsock2_open
* Description   : winsock2 활성화
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool winsock2_open()
{
	WSADATA wsad;

	if (WSAStartup(2, &wsad)) {
		printf("WSAStartup() Error in MS Windows\n");
		WSACleanup();
		return false;
	}

	return true;
}

/*
*******************************************************************************
* Function name : winsocke2_close
* Description   : winsock2 비활성화
* Arguments     : none
* Returns       : none
* Notes         : none
*******************************************************************************
*/
void winsocke2_close()
{
	WSACleanup ();
}

/*
*******************************************************************************
* Function name : set_drive_up
* Description   : 구동부 전원 켜기
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool set_drive_up()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "PowerUpDriveModule", NULL);
		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : set_drive_down
* Description   : 구동부 전원 끄기
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool set_drive_down()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "PowerDownDriveModule", NULL);
		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : set_drive_reset
* Description   : 구동부 전원 리셋
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool set_drive_reset()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "ResetDriveModule", NULL);
		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : set_sensor_up
* Description   : 센서부 전원 켜기
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool set_sensor_up()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "PowerUpSensorModule", NULL);
		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : set_sensor_down
* Description   : 센서부 전원 끄기
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool set_sensor_down()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "PowerDownSensorModule", NULL);
		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : set_sensor_reset
* Description   : 센서부 전원 리셋
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool set_sensor_reset()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "ResetSensorModule", NULL);
		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : set_lrf_up
* Description   : LRF 전원 켜기
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool set_lrf_up()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "PowerUpRangefinder", NULL);
		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : set_lrf_down
* Description   : LRF 전원 끄기
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool set_lrf_down()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "PowerDownRangefinder", NULL);
		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : set_lrf_reset
* Description   : LRF 전원 리셋
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool set_lrf_reset()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "ResetRangefinder", NULL);
		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : power_module_handler
* Description   : 파워 모듈 전원 읽기 
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool power_module_handler()
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int connect_state;
	int power_val[16];
	int reserve[10];

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, POWER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadPowerStatus", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}]", 
				&power_val[0],
				&power_val[1],
				&power_val[2],
				&power_val[3],
				&reserve[0],
				&reserve[1],
				&power_val[4],
				&power_val[5],
				&power_val[6],
				&reserve[2],
				&power_val[7],
				&reserve[3],
				&reserve[4],
				&reserve[5],
				&reserve[6],
				&reserve[7]
				);
		
			dsphal_datalist_destroy(datalist_ret);
			dsphal_tcp_client_destroy(tcp_client);

			printf("Battery Volt: %d \t", power_val[0]);
			printf("Charge Volt: %d \n", power_val[1]);
			printf("Charge Current: %d \t", power_val[2]);
			printf("Consumption Current: %d \n", power_val[3]);
			printf("Drive State: %d \t", power_val[4]);
			printf("Sensor State: %d \n", power_val[5]);
			printf("LRF State: %d \t", power_val[6]);
			printf("StarGazer State: %d \n\n", power_val[7]);
			return true;
		}
		else {
			dsphal_tcp_client_destroy(tcp_client);
			printf("no return datalist\n");
			return false;
		}
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}
