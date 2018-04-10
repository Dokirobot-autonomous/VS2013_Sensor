// Updated 2015/09/13 by Suyama

#pragma once
#ifndef _STDINT
#define _STDINT
#endif

#include <stdio.h>
#include <winsock2.h>	// windows socket version2 init (for communication)
#include "dsphal.h"		// dssphal header
#include "drive.h"
#include "stdafx.h"
#include "usonic_sensor.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <fstream>
#include <Windows.h>
#include <sys\types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <thread>
#include "atr_tsnd121.hpp"
//#include "TETRA_DS_IV.hpp"

/* urg library */
#include "urg_connection.h"
#include "urg_sensor.h"
#include "urg_utils.h"
#include "math_utilities.h"
/* ここまで */

/* OpenCV Library */
#include <opencv\highgui.h>
#include <opencv2\opencv.hpp>
#include <opencv2\legacy\legacy.hpp>
#include <opencv2\legacy\compat.hpp>
#include <opencv2\video\tracking.hpp>
/* ここまで */

/* Suyama Library */
#include "GPS.hpp"
#include "serial.h"
/* ここまで */

/* 保存するデータ */
#define GET_TIME 1
#define GET_ODOMETRY 1
#define GET_GPS 1
#define GET_LRF_L 1
#define GET_LRF_U 1
#define GET_OMNI 1
#define GET_ACCELATION 0
#define GET_GEOMAGNETIC 0
#define GET_GYRO 0

/* time,gps,accelation,gyro,geomagneticファイル保存時に分割するかどうか */
static bool separate_file = false;

/* データ取得前にtestを実施 */
static bool mode_test = true;
static bool finish_test_mode = false;
void finishTestMode(int signa){
	finish_test_mode = true;
	std::cout << "Finish Test Mode!" << std::endl;
}


// データ取得終了処理
static bool finish_getting_data = false;
void finalizeProcessing(int signa){
	finish_getting_data = true;
	std::cout << "Finish getting data!" << std::endl;
}

/* Ohashi function */
void readError(const std::string &filename){
	std::cout << "\n\nError!!" << std::endl;
	std::cout << filename << " do not exist!\n" << std::endl;
	finish_getting_data = true;
	return;
};
void writeError(const std::string &filename, bool fin = 1)
{
	std::cout << "\n\nError!!" << std::endl;
	std::cout << filename << " can not be created!\n" << std::endl;
	finish_getting_data = true;
	return;
};
/* ここまで */

// オドメトリの値取得
int get_odometry_value(std::vector<int> *datalist);





/******************** ここからパラメータ設定　********************/

/* GPS */
//8 -> 6 changed 2017.9.14
TCHAR buf1[1024] = _T("COM9");
LPCWSTR buf2 = buf1;
/* ここまで */

/* LRF */
#define LRF_UPPER_COM_PORT "COM6"
#define LRF_LOWER_COM_PORT "COM5"
/* ここまで */

#define UPDATE_TIME 200 // 更新周期 [ms]
#define NO 1 // 前回の続きから計測する場合は数字をインクリメント
#define MEASUREMENT_TIMES 1 // 一度の計測で何回データを取得するか

/******************** ここまでパラメータ設定　********************/


using namespace std;
using namespace cv;



/******************** ここからmain関数　********************/

/* Odometry, GPS, Omni camera, LRF, ATR */
int main()
{

	// ロボットにアクセス
	if (!winsock2_open()){
		std::cout << "No Valid Robot!" << std::endl;
		return -1;
	}
	std::cout << "Complete Robot Initialization! " << std::endl;

	
	
	int ret; // 接続，取得確認用

	/********** オドメトリの初期化 **********/
#if GET_ODOMETRY
	std::vector<int> odo_ini;
	ret = get_odometry_value(&odo_ini); // (0,0,0)を初期位置とする
	if (!ret){
		std::cout << "Odometry Initialization Failed!" << std::endl;
		return -1;
	}
	std::cout << "Complete Odometry Initialization! " << std::endl;
#endif
	/*********** ここまで **********/

	/********** GPSの初期化 **********/

#if GET_GPS
	SerialPort sp_gps;
	//sp_gps.start(buf2, 38400); // COM番号，ボーレート
	sp_gps.start(buf2, 19200); // COM番号，ボーレート
	if (sp_gps.ret < 0){
		std::cout << "No Valid GPS!" << std::endl;
		return -1;
	}
	std::cout << "Complete GPS Initialization! " << std::endl;
#endif
	/********** ここまで **********/


	/********** カメラの初期化 **********/
#if GET_OMNI
	cv::VideoCapture cap(1);//デバイスのオープン
	//cap.open(0);//こっちでも良い．
	cap.set(CV_CAP_PROP_FPS, 1.0 / (UPDATE_TIME / 1000.0));
	if (!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
	{
		//読み込みに失敗したときの処理
		std::cout << "No Valid Camera!" << std::endl;
		return -1;
	}
	std::cout << "Complete OmniCamera Initialization! " << std::endl;
#endif
	/********** ここまで **********/



	/********** LRFの初期化 **********/
#if GET_LRF_U
	// LRF_UPPER
	urg_t urg1;
	long *length_data1;
	int length_data_size1;
	const char connect_device1[] = LRF_UPPER_COM_PORT; // 要設定
	const long connect_baudrate1 = 115200;
	//const long connect_baudrate = 38400;
	ret = urg_open(&urg1, URG_SERIAL, connect_device1, connect_baudrate1); // センサに接続
	if (ret < 0){ // == 0：Success  < 0：Error
		std::cout << "No Valid LRF!" << std::endl;
		return -1;
	}
	length_data1 = (long *)malloc(sizeof(long)* urg_max_data_size(&urg1)); // データ受信領域を確保
	urg_set_scanning_parameter(&urg1, urg_deg2step(&urg1, -135), urg_deg2step(&urg1, 135), 0); //スキャンパラメータの設定 正面が0度
	ret = urg_start_measurement(&urg1, URG_DISTANCE, 1, 0); // LRFデータ取得（1回やっておいた方が良い）
	length_data_size1 = urg_get_distance(&urg1, length_data1, NULL); // データ更新と同時にデータサイズ取得
	std::cout << "Complete UPPER_LRF Initialization! " << std::endl;
#endif
#if GET_LRF_L
	// LRF_lower
	urg_t urg2;
	long *length_data2;
	int length_data_size2;
	const char connect_device2[] = LRF_LOWER_COM_PORT; // 要設定
	const long connect_baudrate2 = 115200;
	//const long connect_baudrate = 38400;
	ret = urg_open(&urg2, URG_SERIAL, connect_device2, connect_baudrate2); // センサに接続
	if (ret < 0){ // == 0：Success  < 0：Error
		std::cout << "No Valid LRF!" << std::endl;
		return -1;
	}
	length_data2 = (long *)malloc(sizeof(long)* urg_max_data_size(&urg2)); // データ受信領域を確保
	urg_set_scanning_parameter(&urg2, urg_deg2step(&urg2, -135), urg_deg2step(&urg2, 135), 0); //スキャンパラメータの設定 正面が0度
	ret = urg_start_measurement(&urg2, URG_DISTANCE, 1, 0); // LRFデータ取得（1回やっておいcた方が良い）
	length_data_size2 = urg_get_distance(&urg2, length_data2, NULL); // データ更新と同時にデータサイズ取得
	std::cout << "Complete LOWER_LRF Initialization! " << std::endl;
#endif
	/*********** ここまで **********/

	/********** ATRセンサの初期化 **********/
#if GET_ACCELATION || GET_GEOMAGNETIC ||GET_GYRO
	AtrTsnd121 atr("127.0.0.1", 10000);
	atr.setParamAccelgyro(UPDATE_TIME / 10, 10, 0);
	atr.setParamGeomagnetic(UPDATE_TIME / 10, 10, 0);
	atr.setParamPressure(0, 0, 0);
	atr.setParamBattery(0, 0);
	//atr.geoCalibration();
	std::cout << "Complete ATR Initialization! " << std::endl;
#endif
	/*********** ここまで **********/



	/********** 出力ファイルの初期化 **********/
	//// 取得時間
	//std::ofstream ofs_time_ini("./output/time.csv", ios_base::out);
	//ofs_time_ini.close();
	//// オドメトリ
	//std::ofstream ofs_odo_ini("./output/odometry.csv", ios_base::out);
	//ofs_odo_ini.close();
	/********** ここまで **********/


	/* ATRセンサのポジション調整 */
#if GET_ACCELATION || GET_GEOMAGNETIC ||GET_GYRO
	{
		bool flag_end = 0;
		std::thread th_cont([&flag_end]{
			cv::namedWindow("Press any key if the head of ATR is OK. ");
			cv::waitKey();
			flag_end = 1;
		});

		/* ATRセンサ計測 */
		std::thread th_atr([&flag_end, &atr]{
			atr.start();
			while (flag_end != 1)
			{
				if (atr.update() == 0)	// agsの更新があった場合
				{
					std::cout << atr.getAccelation()[0] << "," << atr.getAccelation()[1] << "," << atr.getAccelation()[2] << std::endl;
				};
			}
			atr.stop();
		});

		th_cont.join();
		th_atr.join();
	}
#endif
	/*********   ここまで   *********/


	/* 変数定義 */
	clock_t start, end;

	/* test mode */
//	if (mode_test)
//	{
//		while (true){
//			cv::namedWindow("Omni Image. Modefy the light of omni camera if necessary");
//			signal(SIGINT, finishTestMode);	// ctrl + Cで終了
//
//			start = clock();	// 開始時刻 
//			/* 時間計測 */
//#if GET_TIME
//			time_t now = time(NULL);
//			struct tm *pnow = localtime(&now);
//			char time_c[128];
//			sprintf_s(time_c, "%02d:%02d:%02d", pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
//			// 取得時間データをファイル出力
//			std::cout << "Time: " << time_c << std::endl;
//#endif
//			/* エンコーダ計測 */
//#if GET_ODOMETRY
//			std::vector<int> odo;
//			get_odometry_value(&odo);
//			/* ファイル出力 */
//			//char filename[256];
//			std::cout << "Odometry: " << odo[0] - odo_ini[0] << "," << odo[1] - odo_ini[1] << "," << odo[2] - odo_ini[2] << std::endl;
//#endif
//			//LRF_UPPER取得
//#if GET_LRF_U
//			{
//				urg_start_measurement(&urg1, URG_DISTANCE, 1, 0); // LRFデータ取得
//				int ret = urg_get_distance(&urg1, length_data1, NULL); // データ更新と同時にデータサイズ取得
//				// LRFデータをファイル出力
//				//char filename[256];
//				//for (int i = 0; i < ret; i++){
//				//	ofs_lrf << length_data1[i] << "," << urg_index2rad(&urg1, i) << std::endl;
//				//}
//				//std::cout << "LRF_U!" << std::endl;
//			}
//#endif
//			//LRF_LOWER取得
//#if GET_LRF_L
//			{
//				urg_start_measurement(&urg2, URG_DISTANCE, 1, 0); // LRFデータ取得
//				int ret = urg_get_distance(&urg2, length_data2, NULL); // データ更新と同時にデータサイズ取得
//				// LRFデータをファイル出力
//				std::cout << "Get LRF_L!" << std::endl;
//			}
//#endif
//			/* 全方位カメラ */
//#if GET_OMNI
//			//clock_t start = clock();
//			cv::Mat img;
//			cap >> img;
//			cv::imshow("Omni Image. Modefy the light of omni camera if necessary", img);
//			// カメラデータをファイル出力
//#endif
//			/* ATRセンサ */
//#if GET_ACCELATION || GET_GEOMAGNETIC || GET_GYRO
//			atr.update();
//			// 加速度の書き込み */
//			if (get_accelation)
//			{
//				sprintf_s(filename, "output/accelation/accelation_no%d_%dth.csv", NO, step);
//				std::ofstream ofs_acc(filename, std::ios_base::out);
//				if (ofs_acc.fail())
//				{
//					writeError(filename, false);
//					end = 'e';
//					break;
//				}
//				ofs_acc << atr.getAccelation()[0] << "," << atr.getAccelation()[1] << "," << atr.getAccelation()[2] << std::endl;
//			}
//			/* ジャイロの書き込み */
//			if (get_gyro){
//				sprintf_s(filename, "output/gyro/gyro_no%d_%dth.csv", NO, step);
//				std::ofstream ofs_gyr(filename, std::ios_base::out);
//				if (ofs_gyr.fail())
//				{
//					writeError(filename, false);
//					break;
//				}
//				ofs_gyr << atr.getGyro()[0] << "," << atr.getGyro()[1] << "," << atr.getGyro()[2] << std::endl;
//			}
//			// 地磁気の書き込み
//			if (get_geomagnetic){
//				sprintf_s(filename, "output/geomagnetic/geomagnetic_no%d_%dth.csv", NO, step);
//				std::ofstream ofs_geo(filename, std::ios_base::out);
//				if (ofs_geo.fail())
//				{
//					writeError(filename, false);
//					break;
//				}
//				ofs_geo << atr.getGeomagnetic()[0] << "," << atr.getGeomagnetic()[1] << "," << atr.getGeomagnetic()[2] << std::endl;
//			}
//#endif
//			/* GPS計測 */
//#if GET_GPS
//			//clock_t lap_gps_start = clock();
//			std::string str;
//			sp_gps.rxLen = 0;
//			while (1)
//			{
//				sp_gps.gets('$', '\n');
//				if (sp_gps.rxFlag == true){ // '\n'が含まれていたら終了
//					std::string str = sp_gps.rxData; // charなのでstringに変換 代入で自動変換らしい
//					std::cout << "GPS: " << str << std::endl;
//					break;
//				}
//				//clock_t lap3 = clock();
//				//if ((int)(lap3 - lap_gps_start) > UPDATE_TIME*0.8){
//				//	str = "No Data";
//				//	std::cout << str << std::endl;
//				//	break;	//	計測周期中にGPSを取得できなかったらbreak
//				//}
//			}
//			// GPSデータをファイル出力
//			//filename[256];
//#endif
//			end = clock();
//
//			int rest_time = UPDATE_TIME - (int)(end - start);
//			if (rest_time > 0){
//				std::this_thread::sleep_for(std::chrono::microseconds(rest_time));
//			}
//		}
//	}




	int step = 1;
	char filename[256];

	/*  計測開始 */
	cv::namedWindow("Press any key to start!");
	cv::waitKey();
	std::cout << "Get Started!" << std::endl;

	/* ATR開始 */
#if GET_ACCELATION || GET_GEOMAGNETIC ||GET_GYRO
	atr.start();
#endif


	/* 再起処理 */
	while (!finish_getting_data){
		signal(SIGINT, finalizeProcessing);	// ctrl + Cで終了

		start = clock();	// 開始時刻 
		/* 時間計測 */
#if GET_TIME
		time_t now = time(NULL);
		struct tm *pnow = localtime(&now);
		char time_c[128];
		sprintf_s(time_c, "%02d:%02d:%02d", pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
		// 取得時間データをファイル出力
		sprintf_s(filename, "./output/time/time_no%d_%dth.csv", NO, step);
		std::ofstream ofs_time(filename, ios_base::out);
		if (ofs_time.fail())
		{
			writeError(filename, false);
			break;
		}
		ofs_time << time_c << std::endl;
		std::cout << "Get Time!" << std::endl;
#endif
		/* エンコーダ計測 */
#if GET_ODOMETRY
		std::vector<int> odo;
		get_odometry_value(&odo);
		/* ファイル出力 */
		//char filename[256];
		sprintf_s(filename, "./output/odometry/odometry_no%d_%dth.csv", NO, step);
		std::ofstream ofs_odo(filename, ios_base::out);
		if (ofs_odo.fail())
		{
			writeError(filename, false);
			break;
		}
		ofs_odo << odo[0] - odo_ini[0] << "," << odo[1] - odo_ini[1] << "," << odo[2] - odo_ini[2] << std::endl;
		std::cout << "Get Odometry!" << std::endl;
#endif
		//LRF_UPPER取得
#if GET_LRF_U
		{
			urg_start_measurement(&urg1, URG_DISTANCE, 1, 0); // LRFデータ取得
			int ret = urg_get_distance(&urg1, length_data1, NULL); // データ更新と同時にデータサイズ取得
			// LRFデータをファイル出力
			//char filename[256];
			sprintf_s(filename, "./output/lrf_upper/lrf_no%d_%dth.csv", NO, step);
			std::ofstream ofs_lrf(filename, ios_base::out);
			if (ofs_lrf.fail())
			{
				writeError(filename, false);
				break;
			}
			for (int i = 0; i < ret; i++){
				ofs_lrf << length_data1[i] << "," << urg_index2rad(&urg1, i) << std::endl;
			}
			std::cout << "Get LRF_U!" << std::endl;
		}
#endif
		//LRF_LOWER取得
#if GET_LRF_L
		{
			urg_start_measurement(&urg2, URG_DISTANCE, 1, 0); // LRFデータ取得
			int ret = urg_get_distance(&urg2, length_data2, NULL); // データ更新と同時にデータサイズ取得
			// LRFデータをファイル出力
			sprintf_s(filename, "./output/lrf_lower/lrf_no%d_%dth.csv", NO, step);
			std::ofstream ofs_lrf(filename, ios_base::out);
			if (ofs_lrf.fail())
			{
				writeError(filename, false);
				break;
			}
			for (int i = 0; i < ret; i++){
				ofs_lrf << length_data2[i] << "," << urg_index2rad(&urg2, i) << std::endl;
			}
			std::cout << "Get LRF_L!" << std::endl;
		}
#endif
		/* 全方位カメラ */
#if GET_OMNI
		//clock_t start = clock();
		cv::Mat img;
		cap >> img;
		// カメラデータをファイル出力
		sprintf_s(filename, "./output/img/img_no%d_%dth.bmp", NO, step);
		cv::imwrite(filename, img);
		std::cout << "Get Omni!" << std::endl;
#endif
		/* ATRセンサ */
#if GET_ACCELATION || GET_GEOMAGNETIC || GET_GYRO
		atr.update();
		// 加速度の書き込み */
		if (get_accelation)
		{
			sprintf_s(filename, "output/accelation/accelation_no%d_%dth.csv", NO, step);
			std::ofstream ofs_acc(filename, std::ios_base::out);
			if (ofs_acc.fail())
			{
				writeError(filename, false);
				end = 'e';
				break;
			}
			ofs_acc << atr.getAccelation()[0] << "," << atr.getAccelation()[1] << "," << atr.getAccelation()[2] << std::endl;
		}
		/* ジャイロの書き込み */
		if (get_gyro){
			sprintf_s(filename, "output/gyro/gyro_no%d_%dth.csv", NO, step);
			std::ofstream ofs_gyr(filename, std::ios_base::out);
			if (ofs_gyr.fail())
			{
				writeError(filename, false);
				break;
			}
			ofs_gyr << atr.getGyro()[0] << "," << atr.getGyro()[1] << "," << atr.getGyro()[2] << std::endl;
		}
		// 地磁気の書き込み
		if (get_geomagnetic){
			sprintf_s(filename, "output/geomagnetic/geomagnetic_no%d_%dth.csv", NO, step);
			std::ofstream ofs_geo(filename, std::ios_base::out);
			if (ofs_geo.fail())
			{
				writeError(filename, false);
				break;
			}
			ofs_geo << atr.getGeomagnetic()[0] << "," << atr.getGeomagnetic()[1] << "," << atr.getGeomagnetic()[2] << std::endl;
		}
#endif
		/* GPS計測 */
#if GET_GPS
		//clock_t lap_gps_start = clock();
		std::string str;
		sp_gps.rxLen = 0;
		while (1)
		{
			sp_gps.gets('$', '\n');
			if (sp_gps.rxFlag == true){ // '\n'が含まれていたら終了
				std::string str = sp_gps.rxData; // charなのでstringに変換 代入で自動変換らしい
				std::cout << str << std::endl;
				sprintf_s(filename, "./output/gps/gps_no%d_%dth.txt", NO, step);
				std::ofstream ofs_gps(filename, ios_base::out);
				if (ofs_gps.fail())
				{
					writeError(filename, false);
					break;
				}
				ofs_gps << str << std::endl;
				break;
			}
			//clock_t lap3 = clock();
			//if ((int)(lap3 - lap_gps_start) > UPDATE_TIME*0.8){
			//	str = "No Data";
			//	std::cout << str << std::endl;
			//	break;	//	計測周期中にGPSを取得できなかったらbreak
			//}
		}
		std::cout << "Get GPS!" << std::endl;
		// GPSデータをファイル出力
		//filename[256];
#endif
		end = clock();
		
		int rest_time = UPDATE_TIME - (int)(end - start);
		if (rest_time > 0){
			std::this_thread::sleep_for(std::chrono::microseconds(rest_time));
		}

		step++;


	}

#if GET_ACCELATION || GET_GEOMAGNETIC || GET_GYRO
	atr.stop();
#endif

#if GET_GPS
	sp_gps.end(); // GPSへの接続を閉じる
#endif
#if GET_OMNI
	cv::destroyAllWindows();
#endif
#if GET_LRF_U
	urg_close(&urg1); // LRFへの接続を閉じる
#endif
#if GET_LRF_L
	urg_close(&urg2); // LRFへの接続を閉じる
#endif
	winsocke2_close(); // ロボットへの接続を閉じる

	return 0;
}

/* Odometry, GPS, Omni camera, LRF */
//int main()
//{
//	// ロボットにアクセス
//	if (!winsock2_open()){
//		std::cout << "No Valid Robot!" << std::endl;
//		return -1;
//	}
//
//
//	int ret; // 接続，取得確認用
//
//	/********** オドメトリの初期化 **********/
//
//	std::vector<int> odo_ini;
//	ret = get_odometry_value(&odo_ini); // (0,0,0)を初期位置とする
//	if (!ret){
//		std::cout << "Odometry Initialization Failed!" << std::endl;
//		return -1;
//	}
//
//
//	/*********** ここまで **********/
//
//
//	/********** GPSの初期化 **********/
//
//	SerialPort sp_gps;
//	sp_gps.start(buf2, 38400); // COM番号，ボーレート
//	if (sp_gps.ret < 0){
//		std::cout << "No Valid GPS!" << std::endl;
//		return -1;
//	}
//
//
//	/********** ここまで **********/
//
//
//	/********** カメラの初期化 **********/
//
//	cv::VideoCapture cap(1);//デバイスのオープン
//	//cap.open(0);//こっちでも良い．
//	cap.set(CV_CAP_PROP_FPS, 1.0 / (UPDATE_TIME / 1000.0));
//
//	if (!cap.isOpened())//カメラデバイスが正常にオープンしたか確認．
//	{
//		//読み込みに失敗したときの処理
//		std::cout << "No Valid Camera!" << std::endl;
//		return -1;
//	}
//
//
//	/********** ここまで **********/
//
//
//
//	/********** LRFの初期化 **********/
//
//	urg_t urg;
//	long *length_data;
//	int length_data_size;
//	const char connect_device[] = LRF_UPPER_COM_PORT; // 要設定
//	const long connect_baudrate = 115200;
//	//const long connect_baudrate = 38400;
//	ret = urg_open(&urg, URG_SERIAL, connect_device, connect_baudrate); // センサに接続
//	if (ret < 0){ // == 0：Success  < 0：Error
//		std::cout << "No Valid LRF!" << std::endl;
//		return -1;
//	}
//	length_data = (long *)malloc(sizeof(long)* urg_max_data_size(&urg)); // データ受信領域を確保
//	urg_set_scanning_parameter(&urg, urg_deg2step(&urg, -135), urg_deg2step(&urg, 135), 0); //スキャンパラメータの設定 正面が0度
//	ret = urg_start_measurement(&urg, URG_DISTANCE, 1, 0); // LRFデータ取得（1回やっておいた方が良い）
//	length_data_size = urg_get_distance(&urg, length_data, NULL); // データ更新と同時にデータサイズ取得
//
//
//	/*********** ここまで **********/
//
//
//	/********** ATRセンサの初期化 **********/
//	AtrTsnd121 atr("127.0.0.1", 10000);
//	atr.setParamAccelgyro(UPDATE_TIME / 10, 10, 0);
//	atr.setParamGeomagnetic(UPDATE_TIME / 10, 10, 0);
//	atr.setParamPressure(0, 0, 0);
//	atr.setParamBattery(0, 0);
//	//atr.geoCalibration();
//	/*********** ここまで **********/
//
//
//	/********** 出力ファイルの初期化 **********/
//
//	//// 取得時間
//	//std::ofstream ofs_time_ini("./output/time.csv", ios_base::out);
//	//ofs_time_ini.close();
//
//	//// オドメトリ
//	//std::ofstream ofs_odo_ini("./output/odometry.csv", ios_base::out);
//	//ofs_odo_ini.close();
//
//	/********** ここまで **********/
//
//	/*  計測開始 */
//	cv::namedWindow("Press any key to start!");
//	cv::waitKey();
//	std::cout << "Get Started!" << std::endl;
//
//
//	/* 制御コマンド変数定義 */
//	char end; // 計測終了
//	bool lrf_start_ = false;
//	bool cmr_start_ = false;
//	int step = 0;
//
//
//	/* コマンド制御 */
//	// 'e'を押した場合、計測終了
//	// UPDATE_TIMEミリ秒経過した場合、各計測器の値を取得
//	// 各センサの計測が終わったらstart_フラグにfalseを代入
//	std::thread th_cont([&end, &lrf_start_, &cmr_start_, odo_ini, &sp_gps, &step]{
//		cv::namedWindow("Press 'e' to end");
//		clock_t lap1 = clock();
//		clock_t lap2;
//		while (end != 'e'){
//			//if (lrf_start_ || cmr_start_ || atr_start_) continue;	// 他のセンサの計測が終わっていない場合、待つ
//			if (lrf_start_ || cmr_start_) continue;	// 他のセンサの計測が終わっていない場合、待つ
//			lap2 = clock();
//			int rest = UPDATE_TIME - (int)(lap2 - lap1);
//			if (rest > 0)	end = cv::waitKey(rest);
//			lap1 = lap2;
//			step++;
//			lrf_start_ = true;
//			cmr_start_ = true;
//
//			/* 時間計測 */
//			time_t now = time(NULL);
//			struct tm *pnow = localtime(&now);
//			char time_c[128];
//			sprintf_s(time_c, "%02d:%02d:%02d", pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
//			char filename[256];
//			// 取得時間データをファイル出力
//			sprintf_s(filename, "./output/time/time_no%d_%dth.csv", NO, step);
//			std::ofstream ofs_time(filename, ios_base::out);
//			ofs_time << time_c << std::endl;
//
//			/* エンコーダ計測 */
//			std::vector<int> odo;
//			get_odometry_value(&odo);
//			/* ファイル出力 */
//			//char filename[256];
//			sprintf_s(filename, "./output/odometry/odometry_no%d_%dth.csv", NO, step);
//			std::ofstream ofs_odo(filename, ios_base::out);
//			ofs_odo << odo[0] - odo_ini[0] << "," << odo[1] - odo_ini[1] << "," << odo[2] - odo_ini[2] << std::endl;
//
//			/* GPS計測 */
//			clock_t lap_gps_start = clock();
//			std::string str;
//			sp_gps.rxLen = 0;
//			while (1)
//			{
//				sp_gps.gets('$', '\n');
//				if (sp_gps.rxFlag == true){ // '\n'が含まれていたら終了
//					std::string str = sp_gps.rxData; // charなのでstringに変換 代入で自動変換らしい
//					std::cout << str << std::endl;
//					break;
//				}
//				clock_t lap3 = clock();
//				if ((int)(lap3 - lap_gps_start) > UPDATE_TIME*0.8){
//					str = "No Data";
//					std::cout << str << std::endl;
//					break;	//	計測周期中にGPSを取得できなかったらbreak
//				}
//			}
//			// GPSデータをファイル出力
//			filename[256];
//			sprintf_s(filename, "./output/gps/gps_no%d_%dth.txt", NO, step);
//			std::ofstream ofs_gps(filename, ios_base::out);
//			ofs_gps << str << std::endl;
//		}
//	});
//
//
//	/* 時間取得 */
//	//std::thread th_time([&end, &time_start_, &step]{
//	//	while (end != 'e')
//	//	{
//	//		if (!time_start_) continue;
//	//		time_t now = time(NULL);
//	//		struct tm *pnow = localtime(&now);
//	//		char time_c[128];
//	//		sprintf_s(time_c, "%02d:%02d:%02d", pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
//	//		char filename[256];
//	//		// 取得時間データをファイル出力
//	//		sprintf_s(filename, "./output/time/time_no%d_%dth.csv", NO, step);
//	//		std::ofstream ofs_time(filename, ios_base::out);
//	//		ofs_time << time_c << std::endl;
//	//		time_start_ = false;
//	//	}
//	//});
//
//
//	/* エンコーダの計測 */
//	//std::thread th_odo([&end, &odo_start_, &step, odo_ini]{
//	//	while (end != 'e'){
//	//		if (!odo_start_) continue;
//	//		std::vector<int> odo;
//	//		get_odometry_value(&odo);
//	//		/* ファイル出力 */
//	//		char filename[256];
//	//		sprintf_s(filename, "./output/odometry/odometry_no%d_%dth.csv", NO, step);
//	//		std::ofstream ofs_odo(filename, ios_base::out);
//	//		ofs_odo << odo[0] - odo_ini[0] << "," << odo[1] - odo_ini[1] << "," << odo[2] - odo_ini[2] << std::endl;
//	//		odo_start_ = false;
//	//	}
//	//});
//
//
//	/* LRFの計測 */
//	std::thread th_lrf1([&end, &lrf_start_, &step, &urg, &length_data]{
//		while (end != 'e'){
//			if (!lrf_start_) continue; // lrf_start_=tureとなるまで待つ
//
//			clock_t start = clock();
//
//			urg_start_measurement(&urg, URG_DISTANCE, 1, 0); // LRFデータ取得
//			int ret = urg_get_distance(&urg, length_data, NULL); // データ更新と同時にデータサイズ取得
//			// LRFデータをファイル出力
//			char filename[256];
//			sprintf_s(filename, "./output/lrf/lrf_no%d_%dth.csv", NO, step);
//			std::ofstream ofs_lrf(filename, ios_base::out);
//			for (int i = 0; i < ret; i++){
//				ofs_lrf << length_data[i] << "," << urg_index2rad(&urg, i) << std::endl;
//			}
//			lrf_start_ = false;
//
//			clock_t end = clock();
//			std::cout << "lrf: " << (int)(end - start) << std::endl;
//		}
//	});
//
//
//	/* 全方位カメラの計測 */
//	std::thread th_cmr([&end, &cmr_start_, &step, &cap]{
//		while (end != 'e')
//		{
//			if (!cmr_start_) continue;
//			clock_t start = clock();
//			cv::Mat img;
//			cap >> img;
//			// カメラデータをファイル出力
//			char filename[256];
//			sprintf_s(filename, "./output/img/img_no%d_%dth.bmp", NO, step);
//			cv::imwrite(filename, img);
//			cmr_start_ = false;
//
//			clock_t end = clock();
//			std::cout << "cmr: " << (int)(end - start) << std::endl;
//		}
//	});
//
//
//	/* GPSの計測 */
//	//std::thread th_gps([&end, &gps_start_, &step, &sp_gps]{
//	//	while (end != 'e')	{
//	//		if (!gps_start_) continue;
//	//		clock_t start = clock();
//	//		std::string str;
//	//		sp_gps.rxLen = 0;
//	//		clock_t clock1 = clock();
//	//		while (1)
//	//		{
//	//			sp_gps.gets('$', '\n');
//	//			if (sp_gps.rxFlag == true){ // '\n'が含まれていたら終了
//	//				std::string str = sp_gps.rxData; // charなのでstringに変換 代入で自動変換らしい
//	//				std::cout << str << std::endl;
//	//				break;
//	//			}
//	//			clock_t clock2 = clock();
//	//			if ((int)(clock2 - clock1) > UPDATE_TIME*0.8){
//	//				str = "No Data";
//	//				std::cout << str << std::endl;
//	//				break;	//	計測周期中にGPSを取得できなかったらbreak
//	//			}
//	//		}
//	//		// GPSデータをファイル出力
//	//		char filename[256];
//	//		sprintf_s(filename, "./output/gps/gps_no%d_%dth.txt", NO, step);
//	//		std::ofstream ofs_gps(filename, ios_base::out);
//	//		ofs_gps << str << std::endl;
//	//		gps_start_ = false;
//	//		clock_t end = clock();
//	//		std::cout << "gps: " << (int)(end - start) << std::endl;
//	//	}
//	//});
//
//
//	/* ATRセンサ計測 */
//	//std::thread th_atr([&end, &atr_start_, &step, &atr]{
//	//	atr.start();
//	//	while (end != 'e')
//	//	{
//	//		atr.update();
//	//		if (atr_start_ == true)
//	//		{
//	//			char filename[256];	// ファイル名
//	//			/* 加速度の書き込み */
//	//			sprintf_s(filename, "output/accelation/accelation_no%d_%dth.csv", NO, step);
//	//			std::ofstream ofs_acc(filename, std::ios_base::out);
//	//			ofs_acc << atr.getAccelation()[0] << "," << atr.getAccelation()[1] << "," << atr.getAccelation()[2] << std::endl;
//	//			/* ジャイロの書き込み */
//	//			sprintf_s(filename, "output/gyro/gyro_no%d_%dth.csv", NO, step);
//	//			std::ofstream ofs_gyr(filename, std::ios_base::out);
//	//			ofs_gyr << atr.getGyro()[0] << "," << atr.getGyro()[1] << "," << atr.getGyro()[2] << std::endl;
//	//			/* 地磁気の書き込み */
//	//			sprintf_s(filename, "output/geomagnetic/geomagnetic_no%d_%dth.csv", NO, step);
//	//			std::ofstream ofs_geo(filename, std::ios_base::out);
//	//			ofs_geo << atr.getGeomagnetic()[0] << "," << atr.getGeomagnetic()[1] << "," << atr.getGeomagnetic()[2] << std::endl;
//	//			atr_start_ = false;
//	//		}
//	//	}
//	//	atr.stop();
//	//});
//
//
//	/*  thread同期  */
//	th_cont.join();
//	//th_time.join();
//	//th_odo.join();
//	th_lrf1.join();
//	th_cmr.join();
//	//th_gps.join();
//	//th_atr.join();
//
//
//	sp_gps.end(); // GPSへの接続を閉じる
//	cv::destroyAllWindows();
//	urg_close(&urg); // LRFへの接続を閉じる
//	winsocke2_close(); // ロボットへの接続を閉じる
//
//	return 0;
//}

/* ATR */
//int main()
//{
//	/********** ATRセンサの初期化 **********/
//	AtrTsnd121 atr("127.0.0.1", 10000);
//	atr.setParamAccelgyro(10, 20, 0);
//	atr.setParamGeomagnetic(10, 20, 0);
//	atr.setParamPressure(0, 0, 0);
//	atr.setParamBattery(0, 0);
//	atr.geoCalibration();
//	/*********** ここまで **********/
//
//
//	/*  計測開始 */
//	cv::namedWindow("Press any key to start!");
//	cv::waitKey();
//	std::cout << "Get Started!" << std::endl;
//
//
//	/* 制御コマンド変数定義 */
//	char end; // 計測終了
//	bool time_start_ = false;
//	bool atr_start_ = false;
//	int step = 0;
//
//
//	/* コマンド制御 */
//	// 'e'を押した場合、計測終了
//	// UPDATE_TIMEミリ秒経過した場合、各計測器の値を取得
//	// 各センサの計測が終わったらstart_フラグにfalseを代入
//	std::thread th_cont([&end, &time_start_, &atr_start_, &step]{
//		cv::namedWindow("Press 'e' to end");
//		clock_t lap1 = clock();
//		clock_t lap2;
//		while (end != 'e'){
//			if (time_start_ || atr_start_) continue;	// 他のセンサの計測が終わっていない場合、待つ
//			lap2 = clock();
//			int rest = UPDATE_TIME - (int)(lap2 - lap1);
//			//std::cout << step << "," << rest << std::endl;
//			if (rest > 0)	end = cv::waitKey(rest);
//			lap1 = lap2;
//			step++;
//			time_start_ = true;
//			atr_start_ = true;
//
//			time_t now = time(NULL);
//			struct tm *pnow = localtime(&now);
//			//char time_c[128];
//			//sprintf_s(time_c, "%02d:%02d:%02d", pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
//			//char filename[256];
//			//// 取得時間データをファイル出力
//			//sprintf_s(filename, "./output/time/time_no%d_%dth.csv", NO, step);
//			//std::ofstream ofs_time(filename, ios_base::out);
//			//if (ofs_time.fail()){
//			//	writeError(filename, 0);
//			//	end = 'e';
//			//}
//			//ofs_time << time_c << std::endl;
//			time_start_ = false;
//		}
//	});
//
//
//	/* ATRセンサ計測 */
//	std::thread th_atr([&end, &atr_start_, &step, &atr]{
//		atr.start();
//		while (end != 'e')
//		{
//			atr.update();
//			if (atr_start_ == true)
//			{
//				//char filename[256];	// ファイル名
//				///* 加速度の書き込み */
//				//sprintf_s(filename, "output/accelation/accelation_no%d_%dth.csv", NO, step);
//				//std::ofstream ofs_acc(filename, std::ios_base::out);
//				//if (ofs_acc.fail()){
//				//	writeError(filename, 0);
//				//	end = 'e';
//				//}
//				//ofs_acc << atr.getAccelation()[0] << "," << atr.getAccelation()[1] << "," << atr.getAccelation()[2] << std::endl;
//				///* ジャイロの書き込み */
//				//sprintf_s(filename, "output/gyro/gyro_no%d_%dth.csv", NO, step);
//				//std::ofstream ofs_gyr(filename, std::ios_base::out);
//				//if (ofs_gyr.fail()){
//				//	writeError(filename, 0);
//				//	end = 'e';
//				//}
//				//ofs_gyr << atr.getGyro()[0] << "," << atr.getGyro()[1] << "," << atr.getGyro()[2] << std::endl;
//				///* 地磁気の書き込み */
//				//sprintf_s(filename, "output/geomagnetic/geomagnetic_no%d_%dth.csv", NO, step);
//				//std::ofstream ofs_geo(filename, std::ios_base::out);
//				//if (ofs_geo.fail()){
//				//	writeError(filename, 0);
//				//	end = 'e';
//				//}
//				//ofs_geo << atr.getGeomagnetic()[0] << "," << atr.getGeomagnetic()[1] << "," << atr.getGeomagnetic()[2] << std::endl;
//
//				std::vector<double> accelation = atr.getAccelation();
//				std::vector<double> geomagnetic = atr.getGeomagnetic();
//
//				std::cout << accelation[1] << std::endl;
//
//				///* roll角，pitch角 */
//				//double roll = atan2(accelation[1], accelation[2]);
//				//std::cout << "roll: " << roll*180.0 / M_PI << std::endl;
//
//				//double pitch = atan2(-accelation[0], accelation[1] * std::sin(roll) + accelation[2] * std::cos(roll));
//				//std::cout << "pitch: " << pitch*180.0 / M_PI << std::endl;
//
//				//double theta = atan2(geomagnetic[2] * std::sin(roll) - geomagnetic[1] * std::cos(roll), geomagnetic[0] * std::cos(pitch) + geomagnetic[1] * std::sin(pitch)*std::sin(roll) + geomagnetic[2] * std::sin(pitch)*std::cos(roll));
//				//std::cout << theta*180.0 / M_PI << std::endl;
//
//
//				atr_start_ = false;
//			}
//		}
//		atr.stop();
//	});
//
//
//	/*  thread同期  */
//	th_cont.join();
//	th_atr.join();
//
//	return 0;
//}

/******************** ここまでmain関数　********************/







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
	WSACleanup();
}

/*
*******************************************************************************
* Function name : get_encoder_handler
* Description   : ｱｸｵｿｺﾎﾀﾇ ｿ｣ﾄﾚｴ・ｰｪ ﾀﾐｱ・
* Arguments     : none
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/

bool get_encoder_handler()
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int connect_state;
	int encoder_l;
	int encoder_r;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, DRIVE_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false

	if (!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadEncoder", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}]",
				&encoder_l,
				&encoder_r);

			dsphal_datalist_destroy(datalist_ret);
			dsphal_tcp_client_destroy(tcp_client);
			printf("encoder left: %d \t right: %d \n", encoder_l, encoder_r);

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
* Function name : get_encoder_handler
* Description   : ｱｸｵｿｺﾎﾀﾇ ｿﾀｵｵｸﾞｵ蟶ｮ ｰｪ ﾀﾐｱ・
* Arguments     : none
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool get_odometry_handler()
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int connect_state;
	int x;
	int y;
	int theta;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, DRIVE_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false

	if (!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadPosition", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}]",
				&x,
				&y,
				&theta);

			dsphal_datalist_destroy(datalist_ret);
			dsphal_tcp_client_destroy(tcp_client);
			printf("odometry x: %d \t y: %d \t theta: %d \n", x, y, theta);

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

	return true;
}

int get_odometry_value(std::vector<int> *datalist)
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int connect_state;
	int x;
	int y;
	int theta;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, DRIVE_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false

	if (!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadPosition", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}]",
				&x,
				&y,
				&theta);

			dsphal_datalist_destroy(datalist_ret);
			dsphal_tcp_client_destroy(tcp_client);
			//printf("odometry x: %d \t y: %d \t theta: %d \n", x, y, theta);

			(*datalist).resize(3);
			(*datalist)[0] = x;
			(*datalist)[1] = y;
			(*datalist)[2] = theta;

			return 1;
		}
		else {
			return 0;
		}
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return 0;
	}
}

/*
*******************************************************************************
* Function name : get_encoder_handler
* Description   : ｱｸｵｿｺﾎｿ｡ ｼﾓｵｵ ｸ昞ﾉ ﾀ・ﾞ
* Arguments     : none
* Returns       : ｼｺｰ・true, ｽﾇﾆﾐ false
* Notes         : none
*******************************************************************************
*/
bool set_velocity_handler(int left_velocity, int right_velocity)
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_arg;
	dsphal_datalist_t *datalist_ret;

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, DRIVE_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);

	if (!connect_state) {
		datalist_arg = dsphal_build_root_datalist("[{i}{i}]", left_velocity, right_velocity);

		datalist_ret = dsphal_request_method_call(tcp_client, "VelocityControl", datalist_arg);

		if (datalist_arg) dsphal_datalist_destroy(datalist_arg);
		if (datalist_ret) dsphal_datalist_destroy(datalist_ret);

		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*Hand made function*/
void usonic_value(double *us_val)
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int i;
	int usonic_sensor_val[7];
	int reserve;
	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, USONIC_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);

	if (!connect_state) {
		//datalist_ret = dsphal_request_method_call(tcp_client, "ReadUltraSonicSensorArray", NULL);
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadSensors", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}{i}{i}{i}{i}{i}]",
				&usonic_sensor_val[0],
				&usonic_sensor_val[1],
				&usonic_sensor_val[2],
				&usonic_sensor_val[3],
				&usonic_sensor_val[4],
				&usonic_sensor_val[5],
				&usonic_sensor_val[6],
				&reserve);

			dsphal_datalist_destroy(datalist_ret);

			//printf("ultra sonic sensor test : ");
			for (i = 0; i < 7; i++){
				//printf("%4d", usonic_sensor_val[i]);
				us_val[i] = (double)usonic_sensor_val[i];
			}

			dsphal_tcp_client_destroy(tcp_client);
			return;
		}
		else {
			dsphal_tcp_client_destroy(tcp_client);
			printf("no return datalist\n");
			return;
		}
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return;
	}
}
