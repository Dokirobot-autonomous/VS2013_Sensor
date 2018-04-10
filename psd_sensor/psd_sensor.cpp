#include <stdio.h>
#include <winsock2.h>	//windows socket version2 init
#include "dsphal.h"		// dssphal header
#include "psd_sensor.h"

int main()
{
	int winsock2_state;
	printf("psd sensor \n");
	winsock2_state = winsock2_open();
	if(winsock2_state == false) return false;

	while(1) psd_handler();

	winsocke2_close();

	return 0;
}

int winsock2_open()
{
	WSADATA wsad;

	if (WSAStartup(2, &wsad)) {
		printf("WSAStartup() Error in MS Windows\n");
		WSACleanup();
		return false;
	}

	return true;
}

void winsocke2_close()
{
	WSACleanup ();
}

int psd_handler()
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int i;
	int psd_sensor_val[7];
	int reserve[9];
	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_DSSMP, PORT_PSD_SENSOR);
	connect_state = dsphal_tcp_client_connect(tcp_client);
	
	if(!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadPsdSensorArray", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret,  "[{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}{i}]",
				&psd_sensor_val[0],
				&psd_sensor_val[1],
				&psd_sensor_val[2],
				&psd_sensor_val[3],
				&psd_sensor_val[4],
				&psd_sensor_val[5],
				&psd_sensor_val[6],
				&reserve[0],
				&reserve[1],
				&reserve[2],
				&reserve[3],
				&reserve[4],
				&reserve[5],
				&reserve[6],
				&reserve[7],
				&reserve[8]
			);

			dsphal_datalist_destroy(datalist_ret);

			printf("psd sensor test : ");
			for (i = 0; i < 7; i++)
				printf("%4d", psd_sensor_val[i]);

			printf("\n");
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
