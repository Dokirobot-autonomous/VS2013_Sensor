#include <stdio.h>
#include <winsock2.h>	//windows socket version2 init
#include "dsphal.h"		// dssphal header
#include "bumper_sensor.h"

int main()
{
	if(!winsock2_open()) return false;

	while(1) {
		bumper_handler();
		Sleep(100);
	}

	winsocke2_close();

	return 0;
}

/*
*******************************************************************************
* Function name : winsock2_open
* Description   : winsock2 ﾈｰｼｺﾈｭ
* Arguments     : none
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
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
* Function name : winsock2_close
* Description   : winsock2 ｺｰｼｺﾈｭ
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
* Function name : set_bumper_off_mode
* Description   : ｹ・ﾛ ｸ・ｺ・ﾈｰｼｺﾈｭ(ｹ・ﾛ ｰｨﾁﾃｿ｡ｵｵ ﾀﾌｵｿ)
* Arguments     : none
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool set_bumper_off_mode()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, BUMPER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "SetBumperOffMode", NULL);
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
* Function name : set_bumper_off_mode
* Description   : ｹ・ﾛ ｸ・ﾈｰｼｺﾈｭ(ｹ・ﾛ ｰｨﾁﾃ ｱｸｵｿｺﾎﾀﾇ ﾀ・・ｸ昞ﾉ ﾂﾜ, 
				  ﾈｸﾀ・・ﾈﾄﾁ・ｸ昞ﾉｸｸ ｰ｡ｴﾉ)
* Arguments     : none
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool set_bumper_dir_mode()
{
	dsphal_tcp_client_t *tcp_client;	

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, BUMPER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		dsphal_request_method_call(tcp_client, "SetBumperDirMode", NULL);
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
* Function name : bumper_handler
* Description   : ｹ・ﾛ ｻﾂ ﾃｼﾅｩ
* Arguments     : none
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool bumper_handler()
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int bumper_val[2];
	int reserve[6];
	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, BUMPER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);
	
	if(!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadBumperArray", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}{i}{i}{i}{i}{i}]",
				&bumper_val[0],
				&bumper_val[1],
				&reserve[0],
				&reserve[1],
				&reserve[2],
				&reserve[3],
				&reserve[4],
				&reserve[5]
			);

			dsphal_datalist_destroy(datalist_ret);

			if(bumper_val[0]) printf("left bumper pressed \t"); 
			else printf("left bumper released \t"); 

			if(bumper_val[1]) printf("right bumper pressed \n"); 
			else printf("right bumper released \n");

			dsphal_tcp_client_destroy(tcp_client);
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
