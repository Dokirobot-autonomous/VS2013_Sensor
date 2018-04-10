/*******************************************************************************
*
* File name     : atr_tsnd121.hpp
* Programmer(s) : Nozomu OHASHI
* Description   : ATRセンサ(TsnD121)のクラス用ヘッダ
*
*******************************************************************************/
#pragma once

#include <vector>
#include <iostream>
#include <sstream>
#include <string>

/*******************************************************************************
* Windowsソケット通信ライブラリの利用設定
*******************************************************************************/
#include <Winsock2.h>
#include <ws2tcpip.h>
#pragma comment( lib, "ws2_32.lib" )
//#pragma comment(lib,"wsock32.lib")
//#pragma comment(lib,"Ws2_32.lib")
//#pragma_comment(lib,"Ws2_32.lib")

/*******************************************************************************
* クラス内変数・関数定義
*******************************************************************************/
class AtrTsnd121
{
	/***** ここから:プライベート変数 *****/
	/* 通信用 */
	struct sockaddr_in dst_addr;
	int dst_socket; // socket?
	/* 制御用 */
	int param_accelgyro[3];		// 加速度計&ジャイロセンサの 0:計測周期, 1:出力周期, 2:記録周期
	int param_geomagnetic[3];	// 地磁気センサの 0:計測周期, 1:出力周期, 2:記録周期
	int param_pressure[3];		// 気圧センサの 0:計測周期, 1:出力周期, 2:記録周期
	bool param_battery[2];		// バッテリ電圧の 0:出力有無, 1:記録有無
	/* 観測用 */
	int time_accelgyro;
	int time_geomagnetic;
	int time_pressure;
	int time_battery;
	double accelation[3];
	double gyro[3];
	double geomagnetic[3];
	double pressure[3];
	double battery[2];
	// データの送受信用配列
	char sendbuf[1024];
	char recvbuf[1024];
	/***** ここまで:プライベート変数 *****/

	/***** ここから:プライベート関数 *****/
	char* unicode2str(const WCHAR* pszWchar);
	bool checkRecvQueue();
	std::string getDataName(char* str);
	void waitOK();
	/***** ここまで:プライベート関数 *****/



public:
	/***** ここから:パブリック関数 *****/
	AtrTsnd121(); // コンストラクタ
	AtrTsnd121(char* server_ip_addr, int ip_port_num); // コンストラクタ
	~AtrTsnd121(); // デストラクタ
	void open(char* server_ip_addr, int ip_port_num);
	void start();
	void stop();
	void setParamAccelgyro(int meas_period, int out_ave_num, int save_ave_num);
	void setParamGeomagnetic(int meas_period, int out_ave_num, int save_ave_num);
	void setParamPressure(int meas_period, int out_ave_num, int save_ave_num);
	void setParamBattery(bool out_battery_, bool save_battery_);
	int update();				// [返り値] 0:ags, 1:geo, 2:pres, 3:batt
	void waitRecv();
	void geoCalibration();
	std::vector<double> getAccelation();
	std::vector<double> getGyro();
	std::vector<double> getGeomagnetic();
	std::vector<double> getPressure();
	std::vector<double> getBattery();
	int getTimeAccelgyro();
	int getTimeGeomagnetic();
	int getTimePressure();
	int getTimeBattery();

	//void readMemdata(int entry_num);
	//void saveMemdata(int entry_num);

	/***** ここまで:パブリック関数 *****/



};

