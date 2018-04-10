#include <stdio.h>
#include <winsock2.h>	//windows socket version2 init
#include "dsphal.h"		// dssphal header
#include "stargazer.h"

int main()
{
	if(!winsock2_open()) return false;

	while(1) stargazer_handler();

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
* Function name : winsock2_open
* Description   : 스타게이저 값 얻어오기(id: 랜드마크 id, x: x좌표, y: y좌표, 
										theta: 각도, height: 랜드마크까지 높이)
* Arguments     : none
* Returns       : 성공 true, 실패 false
* Notes         : none
*******************************************************************************
*/
bool stargazer_handler()
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int connect_state;
	int id;
	int x;
	int y;
	int theta;
	int height;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_DSSMP, STARGAZER_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);
	
	if(!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadPosition", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}{i}{i}]",
				&id,
				&x,
				&y,
				&theta,
				&height);

			dsphal_datalist_destroy(datalist_ret);
			dsphal_tcp_client_destroy(tcp_client);

			printf("id: %d \t x: %d \t y: %d \t theta: %d \t height: %d\n", id, x, y, theta);
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
