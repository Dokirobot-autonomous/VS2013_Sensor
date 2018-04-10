/*******************************************************************************
*
* File name     : atr_tsnd121.cpp
* Programmer(s) : Nozomu OHASHI
* Description   : ATRセンサ(TsnD121)のクラス用ヘッダ
*
*******************************************************************************/
#include "stdafx.h"
#include "atr_tsnd121.hpp"


/*******************************************************************************
* Function name  : AtrTsnd121
* Function type  : public
* Input / Output : IP adress, port / non
* Description    : Constructer
*******************************************************************************/
AtrTsnd121::AtrTsnd121(char* server_ip_addr, int ip_port_num){

	// 接続準備
	WSADATA wsad;
	if (WSAStartup(MAKEWORD(2, 0), &wsad)) {
		std::cerr << "WSAStartup() Error in MS Windows" << std::endl;
		WSACleanup();
		exit(-1);	// ソケットオープンエラー
	}
	memset(&dst_addr, 0, sizeof(dst_addr));
	dst_addr.sin_port = htons(ip_port_num);
	dst_addr.sin_family = AF_INET;
	inet_pton(AF_INET, server_ip_addr, &dst_addr.sin_addr.s_addr);

	// ソケットの作成
	dst_socket = socket(AF_INET, SOCK_STREAM, 0);

	//接続
	if (connect(dst_socket, (struct sockaddr *) &dst_addr, sizeof(dst_addr))){
		fprintf(stderr, "Can't connect %s\n", server_ip_addr);
		exit(1);
	}

	return; // 正常終了
}



/*******************************************************************************
* Function name  : ~AtrTsnd121
* Function type  : public
* Input / Output : non / non
* Description    : Destructer
*******************************************************************************/
AtrTsnd121::~AtrTsnd121(){
	// ソケットのクローズと後始末
	WSASendDisconnect(dst_socket, NULL);
	closesocket(dst_socket);
	WSACleanup();
	return;
}



/*******************************************************************************
* Function name  : start
* Function type  : public
* Input / Output : non / non
* Description    : start measurement
*******************************************************************************/
void AtrTsnd121::start(){
	sprintf_s(sendbuf, "start\n");
	send(dst_socket, sendbuf, strlen(sendbuf), 0);
	waitRecv();
	memset(recvbuf, 0, sizeof(recvbuf));
	recv(dst_socket, recvbuf, 1024, 0);
	waitRecv();
	memset(recvbuf, 0, sizeof(recvbuf));
	recv(dst_socket, recvbuf, 1024, 0);


	//if (checkRecvQueue()) {
	//	memset(recvbuf, 0, sizeof(recvbuf));
	//	recv(dst_socket, recvbuf, 1024, 0);
	//}
	//if (checkRecvQueue()) {
	//	memset(recvbuf, 0, sizeof(recvbuf));
	//	recv(dst_socket, recvbuf, 1024, 0);
	//}

	return;
};



/*******************************************************************************
* Function name  : stop
* Function type  : public
* Input / Output : non / non
* Description    : stop measurement
*******************************************************************************/
void AtrTsnd121::stop(){
	sprintf_s(sendbuf, "stop\n");
	send(dst_socket, sendbuf, strlen(sendbuf), 0);
	waitOK();
	if (checkRecvQueue()) {
		memset(recvbuf, 0, sizeof(recvbuf));
		recv(dst_socket, recvbuf, 1024, 0);
	}
	return;
};



/*******************************************************************************
* Function name  : setParamAccelgyro
* Function type  : public
* Input / Output : 計測周期，出力の平均回数，保存の平均回数 / non
* Description    :
計測OFF or 計測周期[1Byte] = 0～255
計測の実施有無及び計測周期を設定
(0:計測OFF、1:計測周期1ms～255:計測周期255ms(1ms 単位指定))
計測データ送信設定[1Byte] = 0～255
計測データ送信の実施有無及び送信時の平均回数を設定
(0:送信しない、1:平均回数1 回～255:平均回数255 回)
計測データ記録設定[1Byte] = 0～255
計測データ記録の実施有無及び記録時の平均回数を設定
(0:記録しない、1:平均回数1 回～255:平均回数255 回)
*******************************************************************************/
void AtrTsnd121::setParamAccelgyro(int meas_period, int out_ave_num, int save_ave_num){
	sprintf_s(sendbuf, "setags %d %d %d\n", meas_period, out_ave_num, save_ave_num);
	send(dst_socket, sendbuf, strlen(sendbuf), 0);
	waitOK();
	return;
};



/*******************************************************************************
* Function name  : setParamGeomagnetic
* Function type  : public
* Input / Output : 計測周期，出力の平均回数，保存の平均回数 / non
* Description    :
計測OFF or 計測周期[1Byte] = 0～255
計測の実施有無及び計測周期を設定
(0:計測OFF、10:計測周期10ms～255:計測周期255ms(1ms 単位指定))
計測データ送信設定[1Byte] = 0～255
計測データ送信の実施有無及び送信時の平均回数を設定
(0:送信しない、1:平均回数1 回～255:平均回数255 回)
計測データ記録設定[1Byte] = 0～255
計測データ記録の実施有無及び記録時の平均回数を設定
(0:記録しない、1:平均回数1 回～255:平均回数255 回)
*******************************************************************************/
void AtrTsnd121::setParamGeomagnetic(int meas_period, int out_ave_num, int save_ave_num){
	sprintf_s(sendbuf, "setgeo %d %d %d\n", meas_period, out_ave_num, save_ave_num);
	send(dst_socket, sendbuf, strlen(sendbuf), 0);
	waitOK();
};



/*******************************************************************************
* Function name  : setParamPressure
* Function type  : public
* Input / Output : 計測周期，出力の平均回数，保存の平均回数 / non
* Description    :
計測OFF or 計測周期[1Byte] = 0～255
計測の実施有無及び計測周期を設定
(0:計測OFF、4:計測周期40ms～255:計測周期2550ms(10ms 単位指定))
計測データ送信設定[1Byte] = 0～255
計測データ送信の実施有無及び送信時の平均回数を設定
(0:送信しない、1:平均回数1 回～255:平均回数255 回)
計測データ記録設定[1Byte] = 0～255
計測データ記録の実施有無及び記録時の平均回数を設定
(0:記録しない、1:平均回数1 回～255:平均回数255 回)
*******************************************************************************/
void AtrTsnd121::setParamPressure(int meas_period, int out_ave_num, int save_ave_num){
	sprintf_s(sendbuf, "setpres %d %d %d\n", meas_period, out_ave_num, save_ave_num);
	send(dst_socket, sendbuf, strlen(sendbuf), 0);
	waitOK();
};



/*******************************************************************************
* Function name  : setParamBattery
* Function type  : public
* Input / Output : 計測出力の有無，保存の有無 / non
* Description    :
計測データ送信設定[1Byte] = 0～1
計測データ送信の実施有無を設定
(0:送信しない、1:送信する)
計測データ記録設定[1Byte] = 0～1
計測データ記録の実施有無を選択
(0:記録しない、1:記録する)
*******************************************************************************/
void AtrTsnd121::setParamBattery(bool out_battery_, bool save_battery_){
	sprintf_s(sendbuf, "setbatt %d %d\n", out_battery_, save_battery_);
	send(dst_socket, sendbuf, strlen(sendbuf), 0);
	waitOK();
};



/*******************************************************************************
* Function name  :
* Function type  :
* Input / Output :
* Description    :
*******************************************************************************/
char* AtrTsnd121::unicode2str(const WCHAR* pszWchar)
{
	int		charCnt;
	char*	resultStr;

	// 必要文字数を計算
	charCnt = ::WideCharToMultiByte(CP_THREAD_ACP, 0, pszWchar, -1, NULL, 0, NULL, NULL);
	resultStr = new char[charCnt];
	// 変換
	charCnt = ::WideCharToMultiByte(CP_THREAD_ACP, 0, pszWchar, wcslen(pszWchar) + 1, resultStr, charCnt, NULL, NULL);
	if (charCnt) return resultStr;

	// エラー時
	delete	resultStr;
	return	NULL;
}



/*******************************************************************************
* Function name  : checkRecvQueue
* Function type  : private
* Input / Output :
* Description    :
*******************************************************************************/
bool AtrTsnd121::checkRecvQueue()
{
	int re;
	struct timeval timeout;
	fd_set fdSet;

	FD_ZERO(&fdSet);
	FD_SET(dst_socket, &fdSet);

	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	re = select(dst_socket + 1, &fdSet, NULL, NULL, &timeout);

	if (re == 0) return false;

	return true;
}



/*******************************************************************************
* Function name  : getDataName
* Function type  : private
* Input / Output : non / data name
* Description    :
*******************************************************************************/
std::string AtrTsnd121::getDataName(char*)
{
	std::string str;
	std::istringstream istr(unicode2str((WCHAR*)recvbuf));
	std::getline(istr, str, ',');
	return str;
}


/*******************************************************************************
* Function name  : waitOK
* Function type  : private
* Input / Output :
* Description    :
*******************************************************************************/
void AtrTsnd121::waitOK(){
	char recvbuf[1024];
	bool okFlag = false;

	while (!okFlag) {
		if (checkRecvQueue()) {
			//パケットの受信
			memset(recvbuf, 0, sizeof(recvbuf));
			recv(dst_socket, recvbuf, 1024, 0);
			if (strcmp(unicode2str((WCHAR*)recvbuf), "OK\n") == 0) okFlag = true;
		}
	}
	return;
}



/*******************************************************************************
* Function name  : waitRecv
* Function type  : public
* Input / Output :
* Description    :
*******************************************************************************/
void AtrTsnd121::waitRecv(){
	while (1) {
		if (checkRecvQueue()) {
			return;
		}
	}
}



/*******************************************************************************
* Function name  :
* Function type  :
* Input / Output :
* Description    :
*******************************************************************************/
//void AtrTsnd121::readMemdata(int entry_num)
//{
//	sprintf_s(sendbuf, "readmemdata %d\n", entry_num);
//	send(dst_socket, sendbuf, strlen(sendbuf), 0);
//	if (checkRecvQueue()) {
//		memset(recvbuf, 0, sizeof(recvbuf));
//		recv(dstSocket, recvbuf, 1024, 0);
//		// 注)recvbufの末尾は改行コード
//		printf("受信データ[%d]: %s", dataCount + 1, unicode2str((WCHAR*)recvbuf));
//		dataCount++;
//	}
//
//
//
//}



/*******************************************************************************
* Function name  : update
* Function type  : public
* Input / Output : non / 0: 加速度・ジャイロ, 1: 地磁気, 2: 気圧, 3: バッテリー, -1: 更新なし
* Description    : update Accelation and Gyro infomation
*******************************************************************************/
int AtrTsnd121::update(){
	/* 更新データがない場合、-1を返す */
	if (checkRecvQueue() == false) return -1;
	
	memset(recvbuf, 0, sizeof(recvbuf));
	recv(dst_socket, recvbuf, 1024, 0);

	/* データ名の取得 */
	std::istringstream istr(unicode2str((WCHAR*)recvbuf));
	std::string data_name;
	std::getline(istr, data_name, ',');


	/* データごとに場合分け */
	if (data_name == "ags"){
		char c;
		istr >> time_accelgyro >> c >> accelation[0] >> c >> accelation[1] >> c >> accelation[2] >> c >> gyro[0] >> c >> gyro[1] >> c >> gyro[2];
		return 0;
	}
	if (data_name == "geo"){
		char c;
		istr >> time_geomagnetic >> c >> geomagnetic[0] >> c >> geomagnetic[1] >> c >> geomagnetic[2];
		return 1;
	}
	if (data_name == "pres"){
		char c;
		istr >> time_pressure >> c >> pressure[0] >> c >> pressure[1] >> c >> pressure[2];
		return 2;
	}
	if (data_name == "batt"){
		char c;
		istr >> time_battery >> c >> battery[0] >> c >> battery[1];
		return 3;
	}
	else{
		return -1;
	}
};



/*******************************************************************************
* Function name  : geoCalibration
* Function type  : public
* Input / Output : non / non
* Description    : Calibrate geomagnetic sensor
*******************************************************************************/
void AtrTsnd121::geoCalibration(){
	std::cout << "Geomagnetic calibration" << std::endl;
	sprintf_s(sendbuf, "geocalib\n");
	send(dst_socket, sendbuf, strlen(sendbuf), 0);
	waitOK();
}




/*******************************************************************************
* Function name  : getAccelation
* Function type  : public
* Input / Output : non / std::vector<double>
* Description    :
*******************************************************************************/
std::vector<double> AtrTsnd121::getAccelation(){
	std::vector<double> out = { accelation[0], accelation[1], accelation[2] };
	return out;
};
std::vector<double> AtrTsnd121::getGyro(){
	std::vector<double> out = { gyro[0], gyro[1], gyro[2] };
	return out;
};
std::vector<double> AtrTsnd121::getGeomagnetic(){
	std::vector<double> out = { geomagnetic[0], geomagnetic[1], geomagnetic[2] };
	return out;
};
std::vector<double> AtrTsnd121::getPressure(){
	std::vector<double> out = { pressure[0], pressure[1], pressure[2] };
	return out;
};
std::vector<double> AtrTsnd121::getBattery(){
	std::vector<double> out = { battery[0], battery[1] };
	return out;
};



/*******************************************************************************
* Function name  : getTimeAccelation
* Function type  : public
* Input / Output : non / measurement time of ags
* Description    :
*******************************************************************************/
int AtrTsnd121::getTimeAccelgyro(){
	return time_accelgyro;
};
int AtrTsnd121::getTimeGeomagnetic(){
	return time_geomagnetic;
};
int AtrTsnd121::getTimePressure(){
	return time_pressure;
};
int AtrTsnd121::getTimeBattery(){
	return time_battery;
};
