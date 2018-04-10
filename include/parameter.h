#pragma once

#include <string>
#include "myfun.h"
#include "mycv.h"

/**********************************************************/
// 実験条件
/**********************************************************/

/* 実験環境 */
#define LOCALIZATION_PARKING 0
#define LOCALIZATION_SQUARE 1
#define LOCALIZATION_B2 2
#define LOCALIZATION_AROUND_8GO 3
#define LOCALIZATION_AREA LOCALIZATION_SQUARE

/* 分布の底上げ */
#define ADD_BIAS true

/* Test */
#define TEST 0

/* INTEGRATE TIME */
enum IntegrateType
{
	AND = 0,
	OR = 1,
};
#define INTEGRATE_TYPE AND

/* Visualize After or Before Resampling*/
enum VisualizeAfterorBeforeResampling
{
	AFTER_RESAMPLING = 0,
	BEFORE_RESAMPLING = 1,
};
#define VISUALIZE_AFTER_OR_BEFORE_RESAMPLING BEFORE_RESAMPLING

/* Omni Camera BoF */
#define USE_BOF 1
#define BOF_ROTATION 0

/* 実行モード */
enum TrialType
{
	TRIAL_SIMULTANEOUS = 0,
	TRIAL_PEARSON = 1,
	TRIAL_PEARSON_NONSTAT = 2,
	TRIAL_NON_TIMESEQUENCE = 3,
	TRIAL_NON_TIMESEQUENCE_SIMUL = 4,
	TRIAL_SUYAMA_STAT=5,
	TRIAL_SUYAMA_NONSTAT = 6,
	TRIAL_3SENSORS_SIMULATNEOUS = 7,
	TRIAL_3SENSORS_PEARSON = 8,
	TRIAL_3SENSORS_PEARSON_NONSTAT = 9,
	TRIAL_3SENSORS_SUYAMA_NONSTAT = 10,
	TRIAL_3SENSORS_LRF_GPS=11,
};
#define TRIAL_TYPE TRIAL_3SENSORS_PEARSON

/* カメラ特徴量 */
enum OMNI_FEATURE_TYPE
{
	OMNI_FEATURE_SIFT = 0,
	OMNI_FEATURE_SURF = 1,

};
#define OMNI_FEATURE OMNI_FEATURE_SIFT

#define W_BIAS 1E-10

/* 試行回数 */
#define FIRST_OUTPUT 10
#define LAST_OUTPUT FIRST_OUTPUT

/*  開始ステップと終了ステップ  */
#define FIRST_STEP 1
#define FIRST_NO 1
#define LAST_STEP 10// 最後まで実行したい場合は10000にする
//#define LAST_STEP 1 // 最後まで実行したい場合は10000にする
#define LAST_NO 1
#define TRUE_IDX_INI 1
#define TRUE_IDX_LAST 100
#define SKIP_STEPS 1

/*  パーティクルフィルタ関連  */
#define SAMPLE_SIZE 100    // 散布するパーティクル数
//#define SAMPLE_SIZE 250    // 散布するパーティクル数
//#define SAMPLE_SIZE 500    // 散布するパーティクル数
// 初期パーティクルのサンプル範囲
//#define INI_SAMPLE_RADIUS_X 2000.0 // 初期パーティクルのサンプル半径[mm]
#define INI_SAMPLE_RADIUS_X 5000.0 // 初期パーティクルのサンプル半径[mm]
#define INI_SAMPLE_RADIUS_Y INI_SAMPLE_RADIUS_X
#define INI_SAMPLE_RADIUS_R M_PI/64.0
//#define INI_SAMPLE_RADIUS_R 0.0000001
// パーティクル遷移時に付加するシステムノイズ
#define TRANS_PAR_SYS_VAR_X 100.0  // xの誤差分散[mm]
//#define TRANS_PAR_SYS_VAR_X 200.0  // xの誤差分散[mm]
//#define TRANS_PAR_SYS_VAR_X 1000.0  // xの誤差分散[mm]
#define TRANS_PAR_SYS_VAR_Y TRANS_PAR_SYS_VAR_X  // yの誤差分散[mm]
#define TRANS_PAR_SYS_VAR_R M_PI/64.0 // rの誤差分散[rad]
//#define TRANS_PAR_SYS_VAR_X 500.0  // xの誤差分散[mm]
//#define TRANS_PAR_SYS_VAR_Y TRANS_PAR_SYS_VAR_X  // yの誤差分散[mm]
//#define TRANS_PAR_SYS_VAR_R M_PI/16.0 // rの誤差分散[rad]

/*  類似性評価用パーティクルのサンプルサイズ */
#define STAT_SAMPLE_SIZE 500
//#define STAT_SAMPLE_SIZE 250
#define STAT_SAMPLE_RADIUS_X 5000.0 // 検定用パーティクル半径[mm]
#define STAT_SAMPLE_RADIUS_Y STAT_SAMPLE_RADIUS_X
#define STAT_SAMPLE_RADIUS_R M_PI/64.0 // 角度のみ、位置推定パーティクルの最大・最小+STAT_SAMPLE_RADIUS_R
//#define STAT_SAMPLE_RADIUS_R M_PI/2.0

/* 方向成分はLRFのみで推定 */
#define ESTIMATE_THETA_ONLY_LRF 1

/*  環境データ関連  */
// parking
#if LOCALIZATION_AREA==LOCALIZATION_PARKING
//const std::string ENVIRONMENT_DATE = "171031"; // 入力ファイルのディレクトリ
//const std::string ENVIRONMENT_TIME = "1237";
//const std::string ENVIRONMENT_DATE_OMNI = "171209"; // 入力ファイルのディレクトリ
//const std::string ENVIRONMENT_TIME_OMNI = "1204";
//#define MAP_ORG_LAT 35110177 // 地図の原点の緯度（地図生成時のLeica位置）
//#define MAP_ORG_LON 137063850 // 地図の原点の経度（地図生成時のLeica位置）
//#define MAP_ORG_ELE 150 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define MAP_ORG_HEAD 0 // 地図の原点の初期角度（北を0°として時計回りを正，ほぼ全ての地図で0）
//#define MAP_IMG_ORG_X 0.2 // 地図画像の原点（正規化）
//#define MAP_IMG_ORG_Y 0.5 // 地図画像の原点（正規化）
//#define MAP_RES 50 // 地図画像の分解能[mm]
/* parking */
const std::string ENVIRONMENT_DATE = "171031"; // 入力ファイルのディレクトリ
const std::string ENVIRONMENT_TIME = "1237";
const std::string ENVIRONMENT_DATE_OMNI = "171213"; // 入力ファイルのディレクトリ
const std::string ENVIRONMENT_TIME_OMNI = "1519";
#define MAP_ORG_LAT 35110177 // 地図の原点の緯度（地図生成時のLeica位置）
#define MAP_ORG_LON 137063850 // 地図の原点の経度（地図生成時のLeica位置）
#define MAP_ORG_ELE 150 // 地図の原点の高度（環境内ではこの値で一定と見なす）
#define MAP_ORG_HEAD 0 // 地図の原点の初期角度（北を0°として時計回りを正，ほぼ全ての地図で0）
#define MAP_IMG_ORG_X 0.2 // 地図画像の原点（正規化）
#define MAP_IMG_ORG_Y 0.5 // 地図画像の原点（正規化）
#define MAP_RES 50 // 地図画像の分解能[mm]
///* parking */
//const std::string ENVIRONMENT_DATE = "171031"; // 入力ファイルのディレクトリ
//const std::string ENVIRONMENT_TIME = "1237";
//const std::string ENVIRONMENT_DATE_OMNI = "171031"; // 入力ファイルのディレクトリ
//const std::string ENVIRONMENT_TIME_OMNI = "1237";
//#define MAP_ORG_LAT 35110177 // 地図の原点の緯度（地図生成時のLeica位置）
//#define MAP_ORG_LON 137063850 // 地図の原点の経度（地図生成時のLeica位置）
//#define MAP_ORG_ELE 150 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define MAP_ORG_HEAD 0 // 地図の原点の初期角度（北を0°として時計回りを正，ほぼ全ての地図で0）
//#define MAP_IMG_ORG_X 0.5 // 地図画像の原点（正規化）
//#define MAP_IMG_ORG_Y 0.5 // 地図画像の原点（正規化）
//#define MAP_RES 100 // 地図画像の分解能[mm]
///* parking */
//const std::string ENVIRONMENT_DATE = "171031"; // 入力ファイルのディレクトリ
//const std::string ENVIRONMENT_TIME = "1237_100";
//const std::string ENVIRONMENT_DATE_OMNI = "171213"; // 入力ファイルのディレクトリ
//const std::string ENVIRONMENT_TIME_OMNI = "1519";
//#define MAP_ORG_LAT 35110177 // 地図の原点の緯度（地図生成時のLeica位置）
//#define MAP_ORG_LON 137063850 // 地図の原点の経度（地図生成時のLeica位置）
//#define MAP_ORG_ELE 150 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define MAP_ORG_HEAD 0 // 地図の原点の初期角度（北を0°として時計回りを正，ほぼ全ての地図で0）
//#define MAP_IMG_ORG_X 0.2 // 地図画像の原点（正規化）
//#define MAP_IMG_ORG_Y 0.5 // 地図画像の原点（正規化）
//#define MAP_RES 100 // 地図画像の分解能[mm]
#endif 
// square
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE
//const std::string ENVIRONMENT_DATE = "171115"; // 入力ファイルのディレクトリ
//const std::string ENVIRONMENT_TIME = "1536";
////const std::string ENVIRONMENT_DATE_OMNI = "171209"; // 入力ファイルのディレクトリ
////const std::string ENVIRONMENT_TIME_OMNI = "1145";
//#define BOF_A 1.816E+10 
//#define BOF_B -25.00
//#define MAP_ORG_LAT 35110390 // 地図の原点の緯度（地図生成時のLeica位置）
//#define MAP_ORG_LON 137064840 // 地図の原点の経度（地図生成時のLeica位置）
//#define MAP_ORG_ELE 170 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define MAP_ORG_HEAD 0 // 地図の原点の初期角度（北を0°として時計回りを正，ほぼ全ての地図で0）
//#define MAP_IMG_ORG_X 0.5 // 地図画像の原点（正規化）
//#define MAP_IMG_ORG_Y 0.5 // 地図画像の原点（正規化）
//#define MAP_RES 50 // 地図画像の分解能[mm]
const std::string ENVIRONMENT_DATE = "171115"; // 入力ファイルのディレクトリ
const std::string ENVIRONMENT_TIME = "1536";
const std::string ENVIRONMENT_DATE_OMNI = "180407"; // 入力ファイルのディレクトリ
const std::string ENVIRONMENT_TIME_OMNI = "1404";
#define BOF_A 3.8452E+13
#define BOF_B -44.27
//#define BOF_A 168423.8
//#define BOF_B -6.31
#define MAP_ORG_LAT 35110390 // 地図の原点の緯度（地図生成時のLeica位置）
#define MAP_ORG_LON 137064840 // 地図の原点の経度（地図生成時のLeica位置）
#define MAP_ORG_ELE 170 // 地図の原点の高度（環境内ではこの値で一定と見なす）
#define MAP_ORG_HEAD 0 // 地図の原点の初期角度（北を0°として時計回りを正，ほぼ全ての地図で0）
#define MAP_IMG_ORG_X 0.5 // 地図画像の原点（正規化）
#define MAP_IMG_ORG_Y 0.5 // 地図画像の原点（正規化）
#define MAP_RES 50 // 地図画像の分解能[mm]
#endif
// b2f 
#if LOCALIZATION_AREA==LOCALIZATION_B2
const std::string ENVIRONMENT_DATE = "171213"; // 入力ファイルのディレクトリ
const std::string ENVIRONMENT_TIME = "1700";
const std::string ENVIRONMENT_DATE_OMNI = "171213"; // 入力ファイルのディレクトリ
const std::string ENVIRONMENT_TIME_OMNI = "1700";
#define MAP_ORG_LAT 35110177 // 地図の原点の緯度（地図生成時のLeica位置）
#define MAP_ORG_LON 137063850 // 地図の原点の経度（地図生成時のLeica位置）
#define MAP_ORG_ELE 170 // 地図の原点の高度（環境内ではこの値で一定と見なす）
#define MAP_ORG_HEAD 0 // 地図の原点の初期角度（北を0°として時計回りを正，ほぼ全ての地図で0）
#define MAP_IMG_ORG_X 0.5 // 地図画像の原点（正規化）
#define MAP_IMG_ORG_Y 0.5 // 地図画像の原点（正規化）
#define MAP_RES 50 // 地図画像の分解能[mm]
#endif
// 8号館周辺
#if LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO
const std::string ENVIRONMENT_DATE = "180209"; // 入力ファイルのディレクトリ
const std::string ENVIRONMENT_TIME = "1404";
const std::string ENVIRONMENT_DATE_OMNI = "180209"; // 入力ファイルのディレクトリ
const std::string ENVIRONMENT_TIME_OMNI = "1404";
#define MAP_ORG_LAT 35110367 // 地図の原点の緯度（地図生成時のLeica位置）35110367
#define MAP_ORG_LON 137064898 // 地図の原点の経度（地図生成時のLeica位置）137064898
#define MAP_ORG_ELE 170 // 地図の原点の高度（環境内ではこの値で一定と見なす）
#define MAP_ORG_HEAD 0 // 地図の原点の初期角度（北を0°として時計回りを正，ほぼ全ての地図で0）
#define MAP_IMG_ORG_X 0.5 // 地図画像の原点（正規化）
#define MAP_IMG_ORG_Y 0.8 // 地図画像の原点（正規化）
#define MAP_RES 50 // 地図画像の分解能[mm]
#endif


/*  計測データ関連  */
// 地図データ作成時にleicaに方向バイアスを加えた場合，加えた分を引いておくこと
#if LOCALIZATION_AREA==LOCALIZATION_PARKING
/* parking */
const std::string MEASUREMENT_DATE = "171031"; // 入力ファイルのディレクトリ
const std::string MEASUREMENT_TIME = "1237";
#define LEICA_ORG_LAT 35110177 // 地図の原点の緯度
#define LEICA_ORG_LON 137063850// 地図の原点の経度
#define LEICA_ORG_ELE 150 // 地図の原点の高度（環境内ではこの値で一定と見なす）
#define LEICA_HORIZONTAL_ERROR -112+0.05/M_PI*180  // LEICAが0°を指すときの北からの角度
///* parking */
//const std::string MEASUREMENT_DATE = "171031"; // 入力ファイルのディレクトリ
//const std::string MEASUREMENT_TIME = "1223";
//#define LEICA_ORG_LAT 35110177 // 地図の原点の緯度
//#define LEICA_ORG_LON 137063850// 地図の原点の経度
//#define LEICA_ORG_ELE 150 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define LEICA_HORIZONTAL_ERROR -112 // LEICAが0°を指すときの北からの角度
//#define LEICA_HORIZONTAL_ERROR -260+0.05/M_PI*180.0 // LEICAが0°を指すときの北からの角度
/* parking */
//const std::string MEASUREMENT_DATE = "171217"; // 入力ファイルのディレクトリ
//const std::string MEASUREMENT_TIME = "0914";
//#define LEICA_ORG_LAT 35110177 // 地図の原点の緯度
//#define LEICA_ORG_LON 137063850// 地図の原点の経度
//#define LEICA_ORG_ELE 150 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define LEICA_HORIZONTAL_ERROR 288.439 - 323 // LEICAが0°を指すときの北からの角度
#endif
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE
/* square */
const std::string MEASUREMENT_DATE = "171031"; // 入力ファイルのディレクトリ
const std::string MEASUREMENT_TIME = "1200";
#define LEICA_ORG_LAT 35110360 // 地図の原点の緯度
#define LEICA_ORG_LON 137064845// 地図の原点の経度
#define LEICA_ORG_ELE 170 // 地図の原点の高度（環境内ではこの値で一定と見なす）
#define LEICA_HORIZONTAL_ERROR 68 // LEICAが0°を指すときの北からの角度
///* square */
//const std::string MEASUREMENT_DATE = "171209"; // 入力ファイルのディレクトリ
//const std::string MEASUREMENT_TIME = "1145";
//#define LEICA_ORG_LAT 35110390 // 地図の原点の緯度（地図生成時のLeica位置）
//#define LEICA_ORG_LON 137064840 // 地図の原点の経度（地図生成時のLeica位置）
//#define LEICA_ORG_ELE 170 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define LEICA_HORIZONTAL_ERROR 68 // LEICAが0°を指すときの北からの角度
///* square */
//const std::string MEASUREMENT_DATE = "171207"; // 入力ファイルのディレクトリ
//const std::string MEASUREMENT_TIME = "1310";
//#define LEICA_ORG_LAT 35110390 // 地図の原点の緯度（地図生成時のLeica位置）
//#define LEICA_ORG_LON 137064840 // 地図の原点の経度（地図生成時のLeica位置）
//#define LEICA_ORG_ELE 170 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define LEICA_HORIZONTAL_ERROR 150.554-169 // LEICAが0°を指すときの北からの角度
///* square */
//const std::string MEASUREMENT_DATE = "180407"; // 入力ファイルのディレクトリ
//const std::string MEASUREMENT_TIME = "1404";
//#define LEICA_ORG_LAT 35110390 // 地図の原点の緯度（地図生成時のLeica位置）
//#define LEICA_ORG_LON 137064840 // 地図の原点の経度（地図生成時のLeica位置）
//#define LEICA_ORG_ELE 170 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define LEICA_HORIZONTAL_ERROR 211.688-178 // LEICAが0°を指すときの北からの角度
#endif
#if LOCALIZATION_AREA==LOCALIZATION_B2
/* b2f */
const std::string MEASUREMENT_DATE = "171213"; // 入力ファイルのディレクトリ
const std::string MEASUREMENT_TIME = "1700";
#define LEICA_ORG_LAT 35110177 // 地図の原点の緯度
#define LEICA_ORG_LON 137063850// 地図の原点の経度
#define LEICA_ORG_ELE 150 // 地図の原点の高度（環境内ではこの値で一定と見なす）
#define LEICA_HORIZONTAL_ERROR -113 // LEICAが0°を指すときの北からの角度
//const std::string MEASUREMENT_DATE = "171216"; // 入力ファイルのディレクトリ
//const std::string MEASUREMENT_TIME = "2320";
//#define LEICA_ORG_LAT 35110177 // 地図の原点の緯度
//#define LEICA_ORG_LON 137063850// 地図の原点の経度
//#define LEICA_ORG_ELE 150 // 地図の原点の高度（環境内ではこの値で一定と見なす）
//#define LEICA_HORIZONTAL_ERROR -23 // LEICAが0°を指すときの北からの角度
#endif
// 8号館周辺
#if LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO
const std::string MEASUREMENT_DATE = "180209"; // 入力ファイルのディレクトリ
const std::string MEASUREMENT_TIME = "1724";
#define LEICA_ORG_LAT 35110367 // 地図の原点の緯度（地図生成時のLeica位置）
#define LEICA_ORG_LON 137064898 // 地図の原点の経度（地図生成時のLeica位置）
#define LEICA_ORG_ELE 170 // 地図の原点の高度（環境内ではこの値で一定と見なす）
#define LEICA_HORIZONTAL_ERROR  128.891-26 // LEICAが0°を指すときの北からの角度  キャリブレーション値-googlemap（leicaからロボットの角度)
#endif


/* ビデオ生成 */
#define MODE_LINIER_INTERPOLATION 1
#define SHOW_MOVIES 0
#define MOVIE_SCALE_W 1
#define MOVIE_SCALE_H 1
#define FPS 1
#define FOURCC CV_FOURCC('X', 'V', 'I', 'D')
#define MOVIE_CREATER_SLEEP_MILLISECONDS 100
#define MOVIE_CREATOR_TIMEOUT_MILLISECONDS 100000
// 計測データ
#define MEASUREMENT_DATA_VIDEO_FPS FPS
#define MEASUREMENT_DATA_VIDEO_FOURCC FOURCC
#define MEASUREMENT_DATA_VIDEO_SCAN_RADIUS 3
#define MEASUREMENT_DATA_VIDEO_SCAN_COLOR cv::Scalar(255,0,0)
#define MEASUREMENT_DATA_VIDEO_IMG_POS_RADIUS 3
#define MEASUREMENT_DATA_VIDEO_IMG_POS_COLOR cv::Scalar(255,0,0)
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_POINT cv::Point(25, 50)
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_FONT cv::FONT_HERSHEY_SIMPLEX
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_SCALE 0.25
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_COLOR cv::Scalar(255, 0, 0)
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_THIN 2
#define MEASUREMENT_DATA_VIDEO_GPS_RADIUS 5
#define MEASUREMENT_DATA_VIDEO_GPS_COLOR cv::Scalar(255,0,0)
// 推定位置
#define VISUALIZE_ESTI 1
#define IMAGE_ESTIPOSITION_RADIUS 5
#define IMAGE_ESTI_ARROW_LENGTH 8
#define IMAGE_ESTI_ARROW_THICKNESS 2
#define IMAGE_ESTIPOSITION_COLOR cv::Scalar(255, 0, 0)
// 事前推定位置
#define VISUALIZE_PRE_ESTI 0
#define IMAGE_PRE_ESTIPOSITION_RADIUS 5
#define IMAGE_PRE_ESTI_ARROW_LENGTH 8
#define IMAGE_PRE_ESTI_ARROW_THICKNESS 1
#define IMAGE_PRE_ESTIPOSITION_COLOR cv::Scalar(255, 255, 0)
// 真の位置
#define IMAGE_TRUEPOSITION_RADIUS 7
#define IMAGE_TPOS_ARROW_LENGTH 8
#define IMAGE_TPOS_ARROW_THICKNESS 3
#define IMAGE_TRUEPOSITION_COLOR cv::Scalar(0, 255, 0)
// 確率分布関係
#define IMAGE_PARTICLE_RADIUS 5
#define IMAGE_PARTICLE_MAX_LIKELIHOOD 0.02
#define IMAGE_PARTICLE_ARROW_LENGTH 8
#define IMAGE_PARTICLE_ARROW_THICKNESS 2.0
// 確率分布関係
#define IMAGE_STAT_PARTICLE_RADIUS 5
#define IMAGE_STAT_PARTICLE_MAX_LIKELIHOOD 0.05
#define IMAGE_STAT_PARTICLE_ARROW_LENGTH 8
#define IMAGE_STAT_PARTICLE_ARROW_THICKNESS 2.0

#define IMAGE_PARTICLE_LARGE_GRAY_CIRCLE_RADIUS 1000	// mm
#define IMAGE_PARTICLE_LARGE_GRAY_CIRCLE_COLOR cv::Scalar(128,128,128)
#define IMAGE_PARTICLE_LARGE_GRAY_CIRCLE_THICHNESS 1

/* ピアソン閾値 */
#define PEARSON_CORRCOEF_TH 0.2 // ピアソン積率相関係数の類似性の閾値


///*  尤度算出時分割数  */
//#define PARTICLE_THREAD_SIZE 10
//#define STAT_PARTICLE_THREAD_SIZE 10
//#define LIKELIHOOD_THREAD PARTICLE_THREAD_SIZE+STAT_PARTICLE_THREAD_SIZE
//#define READ_MEAS_THREAD_SIZE 5
//#define ENV_DESCRITOR_THREAD LIKELIHOOD_THREAD
////#define LIKELIHOOD_THREAD 1
//
///*  尤度算出時分割数（提案）  */
#define LIKELIHOOD_THREAD 20


/*  尤度生成関連  */
// Lidar2d
#define ICP_PAIR_DST_TH 3000 // ICPマッチングの対応点決定の閾値
#define ICP_PAIR_NUM_TH_L 0.3  // 対応点の最低数（閾値以下では尤度0）
#define ICP_PAIR_NUM_TH_U 0.3  // 対応点の最低数（閾値以下では尤度0）
//#define ICP_PARAMETER 1.0e-5   // ICPの尤度関数の正規分布のパラメータ
#define ICP_PARAMETER 4.0e-6   // ICPの尤度関数の正規分布のパラメータ
#define LASER_DIST_RANGE 20000 // レーザスキャナの計測可能距離[mm]
#define GRID_PARAMETER 7.0e-4
// Omni Camera
#define CMR_IMG_SIZE_X 1600 // 画像の横幅
#define CMR_IMG_SIZE_Y 1200 // 画像の縦幅
#define ROI_SIZE_X 1200
#define ROI_SIZE_Y 1200
#define ROI_ORG_X (CMR_IMG_SIZE_X - ROI_SIZE_X) * 0.5
#define ROI_ORG_Y (CMR_IMG_SIZE_Y - ROI_SIZE_Y) * 0.5
#define BOF_TH_DST 2.0 // BoFの撮像位置の閾値（*σ）
// PARKING
#if LOCALIZATION_AREA==LOCALIZATION_PARKING
#define BOF_A 3.982E+13 
#define BOF_B -36.15
#endif
// SQUARE
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE
#endif
#if LOCALIZATION_AREA==LOCALIZATION_B2
#define BOF_A 2.117997E+5
#define BOF_B -7.04353
#endif
#if LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO
#define BOF_A 2.24441E+5
#define BOF_B -5.37
#endif
//#define BOF_SGM(sim) 1.15E+08 * std::exp(-13.5598*sim) // y = 1E+10e-20.25x
#define BOF_SGM(sim) BOF_A* std::exp(BOF_B*sim) // y = BOF_Ae-BOF_Bx
#define SYS_X 7500 // 初期システムノイズ（x[mm]） // システムノイズはそんなに大きくしない方がよい
#define BOF_TH_SIM -1/BOF_B*log(SYS_X/BOF_A) // BoFの類似度の閾値
// Typeを指定 BruteForce（-L1, -SL2, -Hamming, -Hamming(2)）, FlannBased
// FlannBasedは回転ダメ？ L1かSL2が良さそう（L1のが緩い）
const std::string matching_type = "BruteForce-SL2";
#define CLUSTER_NUM 1000
#define SIFT_DIM 128
#define SURF_PARAM 400
// SIFT
#define SIFT_SIZE_USE_FUTURES 200 
#define SIFT_AMP_XY_SUNCTION(distance) 1/distance
#define SIFT_AMP_FUNCTION(dis_keys) 1/dis_keys	//	SURFにおいてkeyポイントの大きさを決定
// SURFとの直接比較
#define SURF_DMATCH_DIST_TH 0.008
#define ENV_IMG_TH 3000	//	現在のパーティクル位置から位置推定に使用する画像の閾値[mm]
#define SURF_SD_GMM 500	//	GMMにおける各正規分布の標準偏差．画像間の距離に応じて適切値を決定
#define SURF_SIZE_USE_FUTURES 200 
#define SURF_AMP_FUNCTION(dis_keys) 1/dis_keys	//	SURFにおいてkeyポイントの大きさを決定
//#define SURF_AMP_FUNCTION(dis_keys) std::exp(-dis_keys)	//	SURFにおいてkeyポイントの大きさを決定
#define SURF_AMP_DISTANCE(distance) 1/distance
#define SURF_SIM_XY(size_of_matches_good) size_of_matches_good/1000.0
#define SURF_XY_VARIANCE(sim_xy) 49168.83*std::exp(-21.6667*sim_xy)
//	GPS
#define GPS_UERE 500 // User Equivalent Range Error（利用者等価距離誤差）[mm]

/*  パーティクルの動画関係  */
#define PARTICLE_FPS 1	//	一秒間の画像数

/*  画像出力時、一定ステップごとに円を描画  */
#define IMG_CIRCLE_STEP 100

/*  画像切り出し半径  */
#define CUT_MAP_RADIUS_X 15000 // [mm]
#define CUT_MAP_RADIUS_Y 15000 // [mm]
/* Weighted particle の時間 */
cv::Point WEIGHTED_STAT_PAR_IMG_TEXT_POINT(25, 50);
#define WEIGHTED_STAT_PAR_IMG_TEXT_FONT cv::FONT_HERSHEY_SIMPLEX
#define WEIGHTED_STAT_PAR_IMG_TEXT_SCALE 1.0
cv::Scalar WEIGHTED_STAT_PAR_IMG_TEXT_COLOR(0, 0, 0);
#define WEIGHTED_STAT_PAR_IMG_TEXT_THIN 2

/* 拡大用 */
#define CUT_LARGE_MAP_RADIUS_X 3000
#define CUT_LARGE_MAP_RADIUS_Y CUT_LARGE_MAP_RADIUS_X

// filepath
/*  入力ファイルパス  */
//const std::string IFPATH = "\\\\Desktop-mt35ltg/f/Data/Localization/";
//const std::string IFPATH = "F://Data/Localization/";
const std::string IFPATH = "../input/";
//const std::string IFPATH_ENV = "C://Users/Robot/Documents/Visual Studio 2015/Projects/Localization.ver2/Localization.ver2/input/Environment/" + ENVIRONMENT_DATE + "/" + ENVIRONMENT_TIME + "/"; // 日付まで
//const std::string IFPATH_ENV = "C://Users/Robot/Documents/Visual Studio 2015/Projects/Localization.ver2/Localization.ver2/input/Environment/" + ENVIRONMENT_DATE + "/" + ENVIRONMENT_TIME + "/"; // 日付まで
const std::string IFPATH_ENV = IFPATH + "Environment/" + ENVIRONMENT_DATE + "/" + ENVIRONMENT_TIME + "/"; // 日付まで
#if LOCALIZATION_AREA==LOCALIZATION_PARKING || LOCALIZATION_AREA==LOCALIZATION_B2 || LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO
const std::string IFPATH_ENV_OMNI = IFPATH + "Environment/" + ENVIRONMENT_DATE_OMNI + "/" + ENVIRONMENT_TIME_OMNI + "/"; // 日付まで
#endif
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE
const std::string IFPATH_ENV_OMNI = IFPATH + "Environment/" + ENVIRONMENT_DATE_OMNI + "/" + ENVIRONMENT_TIME_OMNI + "/"; // 日付まで
//const std::string IFPATH_ENV_OMNI = IFPATH + "Environment/Omni/integrate1/"; // 日付まで
#endif
const std::string IFPATH_MEAS = IFPATH + "Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/"; // 日付まで
const std::string IFPATH_RMAP = IFPATH + "Result/Visual Studio/ReliabilityMap/pearson/Data/";

/*  出力ファイルパス  */
//const std::string OFPATH = "F://Data/Localization/Result/Visual Studio/Localization.ver2/map" + ENVIRONMENT_DATE + ENVIRONMENT_TIME + "/loc" + MEASUREMENT_DATE + MEASUREMENT_TIME + "/Pearson/01/";
//const std::string OFPATH_ALL = "\\\\Desktop-mt35ltg/f/Data/Localization/Result/Visual Studio/Localization.ver2/";
//const std::string OFPATH_ALL = "../output/";
const std::string OFPATH_ALL = "E://Data/Localization/Result/Visual Studio/Localization.ver2/";
//const std::string OFPATH_ALL = "//Desktop-mt35ltg/f/Data/Localization/Result/Visual Studio/Localization.ver2/";
// PARKING
#if LOCALIZATION_AREA==LOCALIZATION_PARKING && TEST==0
const std::string OFPATH_SIMUL = OFPATH_ALL + "parking/simultaneous";
const std::string OFPATH_PEAR = OFPATH_ALL + "parking/pearson";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "parking/pearson_nonstat";
const std::string OFPATH_NON_TIMESEQUENCE= OFPATH_ALL + "parking/nontimesequence_pear";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "parking/nontimesequence_simul";
const std::string OFPATH_SUYAMA_STAT = OFPATH_ALL + "parking/suyama_stat";
const std::string OFPATH_SUYAMA_NONSTAT = OFPATH_ALL + "parking/suyama_nonstat";
const std::string OFPATH_3SENSORS_SIMULTANEOUS = OFPATH_ALL + "parking/3sensors_simul";
const std::string OFPATH_3SENSORS_PEARSON = OFPATH_ALL + "parking/3sensors_pear";
const std::string OFPATH_3SENSORS_PEARSON_NONSTAT= OFPATH_ALL + "parking/3sensors_pear_nonstat";
const std::string OFPATH_3SENSORS_SUYAMA_NONSTAT = OFPATH_ALL + "parking/3sensors_suyama_nonstat";
const std::string OFPATH_3SENSORS_LRF_GPS = OFPATH_ALL + "parking/3sensors_lrf_gps";
#endif
// SQUARE
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE && TEST==0
const std::string OFPATH_SIMUL = OFPATH_ALL + "square/simultaneous";
//const std::string OFPATH_SIMUL = "output/";
const std::string OFPATH_PEAR = OFPATH_ALL + "square/pearson";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "square/pearson_nonstat";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "square/nontimesequence_simul";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE = OFPATH_ALL + "square/nontimesequence_pear";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PREKL = OFPATH_ALL + "square/PreviousKL/";
//const std::string OFPATH_PREKL = "output/";
const std::string OFPATH_SUYAMA_STAT = OFPATH_ALL + "square/suyama_stat";
const std::string OFPATH_SUYAMA_NONSTAT = OFPATH_ALL + "square/suyama_nonstat";
const std::string OFPATH_3SENSORS_SIMULTANEOUS = OFPATH_ALL + "square/3sensors_simul";
const std::string OFPATH_3SENSORS_PEARSON = OFPATH_ALL + "square/3sensors_pear";
const std::string OFPATH_3SENSORS_PEARSON_NONSTAT= OFPATH_ALL + "square/3sensors_pear_nonstat";
const std::string OFPATH_3SENSORS_SUYAMA_NONSTAT = OFPATH_ALL + "square/3sensors_suyama_nonstat";
const std::string OFPATH_3SENSORS_LRF_GPS = OFPATH_ALL + "square/3sensors_lrf_gps";
#endif
// B2
#if LOCALIZATION_AREA==LOCALIZATION_B2 && TEST==0
const std::string OFPATH_SIMUL = OFPATH_ALL + "B2/simultaneous";
//const std::string OFPATH_SIMUL = "output/";
const std::string OFPATH_PEAR = OFPATH_ALL + "B2/pearson";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "B2/pearson_nonstat";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "B2/nontimesequence_simul";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE = OFPATH_ALL + "B2/nontimesequence_pear";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PREKL = OFPATH_ALL + "B2/PreviousKL/";
//const std::string OFPATH_PREKL = "output/";
const std::string OFPATH_SUYAMA_STAT = OFPATH_ALL + "B2/suyama_stat";
const std::string OFPATH_SUYAMA_NONSTAT = OFPATH_ALL + "B2/suyama_nonstat";
const std::string OFPATH_3SENSORS_SIMULTANEOUS = OFPATH_ALL + "B2/3sensors_simul";
const std::string OFPATH_3SENSORS_PEARSON = OFPATH_ALL + "B2/3sensors_pear";
const std::string OFPATH_3SENSORS_PEARSON_NONSTAT= OFPATH_ALL + "B2/3sensors_pear_nonstat";
const std::string OFPATH_3SENSORS_SUYAMA_NONSTAT = OFPATH_ALL + "B2/3sensors_suyama_nonstat";
const std::string OFPATH_3SENSORS_LRF_GPS = OFPATH_ALL + "B2/3sensors_lrf_gps";
#endif
// 8号館周辺
#if LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO&& TEST==0
const std::string OFPATH_SIMUL = OFPATH_ALL + "around-8-building/simultaneous";
const std::string OFPATH_PEAR = OFPATH_ALL + "around-8-building/pearson";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "around-8-building/pearson_nonstat";
const std::string OFPATH_NON_TIMESEQUENCE= OFPATH_ALL + "around-8-building/nontimesequence_pear";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "around-8-building/nontimesequence_simul";
const std::string OFPATH_SUYAMA_STAT = OFPATH_ALL + "around-8-building/suyama_stat";
const std::string OFPATH_SUYAMA_NONSTAT = OFPATH_ALL + "around-8-building/suyama_nonstat";
const std::string OFPATH_3SENSORS_SIMULTANEOUS = OFPATH_ALL + "around-8-building/3sensors_simul";
const std::string OFPATH_3SENSORS_PEARSON = OFPATH_ALL + "around-8-building/3sensors_pear";
const std::string OFPATH_3SENSORS_PEARSON_NONSTAT= OFPATH_ALL + "around-8-building/3sensors_pear_nonstat";
const std::string OFPATH_3SENSORS_SUYAMA_NONSTAT = OFPATH_ALL + "around-8-building/3sensors_suyama_nonstat";
const std::string OFPATH_3SENSORS_LRF_GPS = OFPATH_ALL + "around-8-building/3sensors_lrf_gps";
#endif

// TEST_MODE
#if TEST==1
const std::string OFPATH_SIMUL = OFPATH_ALL + "simultaneous";
//const std::string OFPATH_SIMUL = "output/";
const std::string OFPATH_PEAR = OFPATH_ALL + "pearson";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "pearson_nonstat";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE = OFPATH_ALL + "non_timesequence";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "nontimesequence_simul";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PREKL = OFPATH_ALL + "PreviousKL/";
//const std::string OFPATH_PREKL = "output/";
#endif

/* 確率分布とパーティクルの保存 */
#define SAVE_PARTICLE_STATES 1
#define SAVE_PARTICLE_IMAGES 0
#define SAVE_STAT_PARTICLE_STATES 1
#define SAVE_STAT_PARTICLE_IMAGES 0

/* 確率分布とパーティクルvisualize */
#define SHOW_PARTICLE_IMAGES 0
#define SHOW_STAT_PARTICLE_IMAGES 0



