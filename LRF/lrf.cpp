#include <stdio.h>
#include <winsock2.h>	//windows socket version2 init
#include "dsphal.h"		// dssphal header
#include "lrf.h"

int main()
{
	int *degree_range;
	int *min_resolution;
	int *max_resolution;

	if(!winsock2_open()) return false;

	// memory malloc
	degree_range = (int *)malloc(sizeof(int));
	min_resolution = (int *)malloc(sizeof(int));
	max_resolution = (int *)malloc(sizeof(int));


	if(!get_degree_range(degree_range)) printf("get degree range false! \n");
	if(!get_min_resolution(min_resolution)) printf("get min resolution false! \n");
	if(!get_max_resolution(max_resolution)) printf("get max resolution false! \n");

	while(1) {
		lrf_handler(*max_resolution);
		Sleep(100);
	}

	free(degree_range);
	free(min_resolution);
	free(max_resolution);

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
* Function name : get_degree_range
* Description   : LRF ｽｺﾄﾋ ｹ・ｧ ｾ錞ﾀｱ・
* Arguments     : *degree_range : LRF ｽｺﾄﾋ ｹ・ｧ ｰｪ
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool get_degree_range(int *degree_range)
{
	dsphal_tcp_client_t *tcp_client;	
	dsphal_datalist_t *datalist_ret;

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, LRF_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "GetDegreeRange", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}]", degree_range);

			dsphal_datalist_destroy(datalist_ret);
			printf("degree range : %d \n", *degree_range);

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

/*
*******************************************************************************
* Function name : get_min_resolution
* Description   : LRF ﾃﾖｼﾒ ｺﾐﾇﾘｴﾉ ｾ錞ﾀｱ・
* Arguments     : *min_resolution : LRF ﾃﾖｼﾒ ｺﾐﾇﾘｴﾉ ｰｪ
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool get_min_resolution(int *min_resolution)
{
	dsphal_tcp_client_t *tcp_client;	
	dsphal_datalist_t *datalist_ret;

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, LRF_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "GetMinimumResolution", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}]", min_resolution);

			dsphal_datalist_destroy(datalist_ret);
			printf("degree range : %d \n", *min_resolution);

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

/*
*******************************************************************************
* Function name : get_min_resolution
* Description   : LRF ﾃﾖｴ・ｺﾐﾇﾘｴﾉ ｾ錞ﾀｱ・
* Arguments     : *max_resolution : LRF ﾃﾖｴ・ｺﾐﾇﾘｴﾉ ｰｪ
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool get_max_resolution(int *max_resolution)
{
	dsphal_tcp_client_t *tcp_client;	
	dsphal_datalist_t *datalist_ret;

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, LRF_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "GetMaximumResolution", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}]", max_resolution);

			dsphal_datalist_destroy(datalist_ret);
			printf("max resolution : %d \n", *max_resolution);

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

/*
*******************************************************************************
* Function name : get_min_resolution
* Description   : argumentsｿ｡ ｵ郞・LRF ｵ･ﾀﾌﾅﾍ ｾ錞ﾀｱ・
* Arguments     : resolution : LRF ｺﾐﾇﾘｴﾉ ｰｪ
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool lrf_handler(int resolution)
{
	dsphal_tcp_client_t *tcp_client;	
	dsphal_datalist_t *datalist_ret;
	dsphal_datalist_t *datalist_arg;
	dsphal_data_t *data;

	int connect_state;
	int lrf_data[600] = {0};
	int i;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, LRF_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false
	
	if(!connect_state) {
		datalist_arg = dsphal_build_root_datalist("[{i}]", resolution);
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadRangeArray", datalist_arg);

		if (datalist_ret) {
			data = dsphal_datalist_get_head(datalist_ret);
			for (i = 0; i < resolution; i++) {
				lrf_data[i] = 
					dsphal_data_int_get((dsphal_data_int_t *)dsphal_data_get_data(data));
				data = dsphal_data_get_next(data);
				printf("LRF DATA[%d]: %d \n", i, lrf_data[i]);
			}

			dsphal_datalist_destroy(datalist_ret);

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
