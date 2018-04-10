/*******************************************************************************
*
* File name     : TETRA_DS_IV.cpp
* Programmer(s) : Yuki FUNABORA
* Description   : 移動ロボット(TETRA-DS IV)のクラスファイル
*
*******************************************************************************/
#include "stdafx.h"
#include "TETRA_DS_IV.hpp"



/*******************************************************************************
* Function name  : TETRA_DS_IT
* Input / Output : IPアドレス,ポート / なし
* Description    : コンストラクタ
*******************************************************************************/
TETRA_DS_IV::TETRA_DS_IV(char* server_ip, int port) {
	// サーバ情報を格納
	server_ip_addr = server_ip;
	server_port = port;
	// 内部指令変数初期化
	velocity[0] = 0; velocity[1] = 0;

	// ソケットオープン(通信用)
	WSADATA wsad;
	if (WSAStartup(2, &wsad)) {
		std::cerr << "WSAStartup() Error in MS Windows" << std::endl;
		WSACleanup();
		exit(-1);	// ソケットオープンエラー
	}

	// デバイス電源オン

	// 初期状態の取得
	update();
	show();

	// サーボオン(別コマンドにすべきか)
	set_servo_on();

	return;	// 正常処理
}



/*******************************************************************************
* Function name  : ~TETRA_DS_IT
* Input / Output : なし / なし
* Description    : デストラクタ
*******************************************************************************/
TETRA_DS_IV::~TETRA_DS_IV() {
	// サーボオフ
	set_servo_off();

	// デバイス電源オフ

	// ソケットクローズ
	WSACleanup();

	return;	// 正常処理
}



/*******************************************************************************
* Function name  : set_velocity
* Input / Output : 左車輪速度指令, 右車輪速度指令 / なし
* Description    : 速度指令値の内部変更(実際のロボットへの指令はupdate実行時)
*******************************************************************************/
void TETRA_DS_IV::set_velocity(int velocity_left, int velocity_right) {
	velocity[0] = velocity_left;
	velocity[1] = velocity_right;
	return;
}



/*******************************************************************************
* Function name  : get_velocity
* Input / Output : なし / 車輪速度配列(0:左側, 1:右側)
* Description    : 内部速度指令値の取得
*******************************************************************************/
int* TETRA_DS_IV::get_velocity() {
	return velocity;
}



/*******************************************************************************
* Function name  : get_encoder
* Input / Output : なし / エンコーダ値配列(0:左側, 1:右側)
* Description    : エンコーダ値の取得(値は前回update時のもの)
*******************************************************************************/
int* TETRA_DS_IV::get_encoder() {
	return encoder;
}



/*******************************************************************************
* Function name  : get_odometry
* Input / Output : なし / ロボットの相対位置姿勢の配列(0:x, 1:y, 2:theta)
* Description    : オドメトリによる位置の取得(値は前回update時のもの)
*******************************************************************************/
int* TETRA_DS_IV::get_odometry() {
	return odometry;
}



/*******************************************************************************
* Function name  : update
* Input / Output : なし / なし
* Description    : 速度指令の更新とセンサ値の取得
*******************************************************************************/
void TETRA_DS_IV::update() {
	// ロボットとの情報交換
	if(!update_velocity()) std::cerr << "Err velocity_update:";
	if(!update_encoder()) std::cerr << "Err encoder_update:";
	if(!update_odometry()) std::cerr << "Err odometry_update:";

	// 正常終了
	return;
}



/*******************************************************************************
* Function name  : show
* Input / Output : なし / なし
* Description    : 速度指令値とセンサ値の標準出力への表示
*******************************************************************************/
void TETRA_DS_IV::show() {
	std::cout << velocity[0] << ", " << velocity[1] << " | ";
	std::cout << encoder[0] << ", " << encoder[1] << " | ";
	std::cout << odometry[0] << ", " << odometry[1] << ", " << odometry[2] << std::endl;
}



/*******************************************************************************
* Function name  : set_servo_on
* Input / Output : なし / 正誤
* Description    : 車輪のサーボオン
*******************************************************************************/
bool TETRA_DS_IV::set_servo_on() {
	// 変数定義
	dsphal_tcp_client_t *tcp_client;

	// ロボットとの通信路確立
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// 通信路確立失敗
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// サーボオン
	dsphal_request_method_call(tcp_client, "ServoOn", NULL);

	// 通信路の破棄
	dsphal_tcp_client_destroy(tcp_client);

	// 正常終了
	return true;
}



/*******************************************************************************
* Function name  : set_servo_off
* Input / Output : なし / 正誤
* Description    : 車輪のサーボオフ
*******************************************************************************/
bool TETRA_DS_IV::set_servo_off() {
	// 変数定義
	dsphal_tcp_client_t *tcp_client;

	// ロボットとの通信路確立
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// 通信路確立失敗
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// サーボオフ
	dsphal_request_method_call(tcp_client, "ServoOff", NULL);

	// 通信路の破棄
	dsphal_tcp_client_destroy(tcp_client);

	// 正常終了
	return true;
}



/*******************************************************************************
* Function name  : update_velocity
* Input / Output : なし / 正誤
* Description    : 内部変数に設定された速度指令値をロボットに反映
*******************************************************************************/
bool TETRA_DS_IV::update_velocity() {
	// 変数定義
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_arg;
	dsphal_datalist_t *datalist_ret;

	// ロボットとの通信路確立
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// 通信路確立失敗
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// 速度指令値の指令
	datalist_arg = dsphal_build_root_datalist("[{i}{i}]", velocity[0], velocity[1]);
	datalist_ret = dsphal_request_method_call(tcp_client, "VelocityControl", datalist_arg);
	if (datalist_arg) dsphal_datalist_destroy(datalist_arg);
	if (datalist_ret) dsphal_datalist_destroy(datalist_ret);

	// 通信路の破棄
	dsphal_tcp_client_destroy(tcp_client);

	// 正常終了
	return true;
}


/*******************************************************************************
* Function name  : update_encoder
* Input / Output : なし / 正誤
* Description    : ロボットからエンコーダ値を取得して内部変数に格納
*******************************************************************************/
bool TETRA_DS_IV::update_encoder() {
	// 変数定義
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	// ロボットとの通信路確立
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// 通信路確立失敗
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// エンコーダ値の取得
	if ((datalist_ret = dsphal_request_method_call(tcp_client, "ReadEncoder", NULL))){
		dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}]", &encoder[0], &encoder[1]);
		dsphal_datalist_destroy(datalist_ret);
	}
	else {
		std::cerr << "No encoder datalist" << std::endl;
		dsphal_datalist_destroy(datalist_ret);
	}

	// 通信路の破棄
	dsphal_tcp_client_destroy(tcp_client);

	// 正常終了
	return true;
}


/*******************************************************************************
* Function name  : update_odometry
* Input / Output : なし / 正誤
* Description    : ロボットからオドメトリ値を取得して内部変数に格納
*******************************************************************************/
bool TETRA_DS_IV::update_odometry() {
	// 変数定義
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	// ロボットとの通信路確立
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// 通信路確立失敗
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// オドメトリの取得
	if ((datalist_ret = dsphal_request_method_call(tcp_client, "ReadPosition", NULL))){
		dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}]", &odometry[0], &odometry[1], &odometry[2]);
		dsphal_datalist_destroy(datalist_ret);
	}
	else {
		std::cerr << "No odometry datalist" << std::endl;
		dsphal_datalist_destroy(datalist_ret);
	}

	// 通信路の破棄
	dsphal_tcp_client_destroy(tcp_client);

	// 正常終了
	return true;
}

