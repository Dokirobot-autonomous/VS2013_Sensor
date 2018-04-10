#include <stdio.h>
#include <winsock2.h>	//windows socket version2 init
#include "dsphal.h"		// dssphal header
#include "emergency.h"

int main()
{
	if(!winsock2_open()) return false;

	while(1) {
		emergency_handler();
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
* Function name : winsocke2_close
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
* Function name : emergency_handler
* Description   : ｱ莖ﾞﾁ､ﾁ・ｹｰ ﾃｼﾅｩ(ｱ莖ﾞﾁ､ﾁ・ｹｰﾀﾌ ｴｭｸｮｸ・ｱｸｵｿｺﾎｿ｡ｼｭ ｼﾓｵｵ ｸ昞ﾉ
				  ﾂﾜ
* Arguments     : none
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool emergency_handler()
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int connect_state;
	int emergency_val;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, EMERGENCY_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadEmergencyKey", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}]", &emergency_val);
			dsphal_datalist_destroy(datalist_ret);

			if(emergency_val)
				printf("Emergency On \n");
			else
				printf("Emergency Off \n");
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
