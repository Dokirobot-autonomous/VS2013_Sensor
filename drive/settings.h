#pragma once

/*
 * 設定パラメータなどを記述
 */

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

/* GPS */
//8 -> 6 changed 2017.9.14
TCHAR buf1[1024] = _T("COM9");
LPCWSTR buf2 = buf1;
/* ここまで */

/* OmniCamera */
#define OMNI_CAMERA_NO 1

/* LRF */
#define LRF_UPPER_COM_PORT "COM6"
#define LRF_LOWER_COM_PORT "COM5"
/* ここまで */

#define UPDATE_TIME 200 // 更新周期 [ms]
#define NO 1 // 前回の続きから計測する場合は数字をインクリメント
#define MEASUREMENT_TIMES 1 // 一度の計測で何回データを取得するか

#define _USE_MATH_DEFINES
