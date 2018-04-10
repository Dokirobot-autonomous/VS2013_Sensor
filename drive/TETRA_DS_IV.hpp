/*******************************************************************************
*
* File name     : TETRA_DS_IV.hpp
* Programmer(s) : Yuki FUNABORA
* Description   : 移動ロボット(TETRA-DS IV)のクラス用ヘッダ
*
*******************************************************************************/
#ifndef _INC_TETRA_DS_IV	// 読込済チェック
#define _INC_TETRA_DS_IV



/*******************************************************************************
* Windowsソケット通信ライブラリの利用設定
*******************************************************************************/
#include <WinSock2.h>
#pragma comment(lib,"wsock32.lib")



/*******************************************************************************
* TETRA-DS IV DSSP-HALライブラリの利用設定
*******************************************************************************/
#include "dsphal.h"
#pragma comment(lib,"../DSSMP_INFO/lib/libdsphal.lib")



/*******************************************************************************
* クラス内変数・関数定義
*******************************************************************************/
class TETRA_DS_IV
{
	/***** ここから:プライベート変数 *****/
	// 通信用
	char *server_ip_addr;
	int server_port;
	// 制御用
	int velocity[2];	// 0:left, 1:right
	// 観測用
	int encoder[2];		// 0:left, 1:right
	int odometry[3];	// 0:x, 1:y, 2:theta

	/***** ここまで:プライベート変数 *****/

	/***** ここから:プライベート関数 *****/
	bool set_servo_on();	// 車輪サーボのオン
	bool set_servo_off();	// 車輪サーボのオフ
	bool update_velocity();	// ロボットへの速度指令の反映
	bool update_encoder();	// ロボットからエンコーダ値を取得
	bool update_odometry();	// ロボットからオドメトリ値を取得
	/***** ここまで:プライベート関数 *****/

	/***** ここから:パブリック関数 *****/
public:
	TETRA_DS_IV(char* server_ip, int port); // コンストラクタ
	~TETRA_DS_IV();			// デストラクタ
	void set_velocity(int velocity_left, int velocity_right);	// 車輪速度の指令
	int* get_velocity();	// 現在の速度指令値取得(不要?)
	int* get_encoder();		// エンコーダ値の取得
	int* get_odometry();	// オドメトリ値の取得

	void update();	// 値の更新(ロボットとの情報交換)

	void show();	// ロボットの情報表示
	/***** ここまで:パブリック関数 *****/

};

#endif	// _INC_TETRA_DS_IV