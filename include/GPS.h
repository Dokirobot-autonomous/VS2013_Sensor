#pragma once

#include "myfun.h"
#include "parameter.h"

/*  尤度生成関連  */

class Gps
{
public:
	/*  コンストラクタ  */
	Gps() {};
	Gps(const Gps& obj)
	{
		/*  計測データ  */
		(this->signal) = (obj.signal);

		/*  尤度生成関連  */

		/*  正規分布モデル  */
		this->mean = obj.mean;
		this->variance = obj.variance;
	}
	Gps& operator=(const Gps& obj)
	{
		/*  計測データ  */
		(this->signal) = (obj.signal);

		/*  尤度生成関連  */

		/*  正規分布モデル  */
		this->mean = obj.mean;
		this->variance = obj.variance;

		return *this;
	}
	/*  デストラクタ*/
	~Gps()
	{
	}


	/**********************************************************/
	//	初期化
	/**********************************************************/

	/*  地図の原点をset  */
	template<typename T>
	void setMapOrgGL(T latitude, T longitude, T elevation)
	{
		signal.setIniPos(latitude, longitude, elevation);
	}


	/**********************************************************/
	//	ファイル読み込み
	/**********************************************************/

	/*  GPGGA信号の読み込み  */
	bool readMeasSignal(int no, int step)
	{
		std::string filename = IFPATH_MEAS[no] + "gps/gps_" + std::to_string(step) + "th.txt";
		std::ifstream ifs(filename);
		if (ifs.fail())	return false;
		std::string str;
		ifs >> str;
		signal.setData(str);
		return true;
	}
	bool readMeasSignal(std::string filename) {
		std::ifstream ifs(filename);
		if (ifs.fail())	return false;
		std::string str;
		ifs >> str;
		signal.setData(str);
		return true;
	}
	void setMeasSignal(std::string data) {
		this->signal.setData(data);
		Position<> position(signal.get_ENU()[0], signal.get_ENU()[1], 0.0);
		mean = TransMatrix(0.0, 0.0, (MAP_ORG_HEAD - 90)*M_PI / 180.0).Transform(position);
		variance = std::pow(GPS_UERE*signal.hdop, 2);
	}
	//bool readMeasSignal(int step)
	//{
	//	std::string filename = IFPATH_MEAS + "gps/gps_" + std::to_string(step) + "th.txt";
	//	std::ifstream ifs(filename);
	//	if (ifs.fail())	return false;
	//	std::string str;
	//	ifs >> str;
	//	signal.setData(str);
	//	return true;
	//}


	/**********************************************************/
	//	GPGGAによる正規分布モデル(ND)
	/**********************************************************/

	/*  正規分布モデルの平均と分散を決定  */
	void setMeasND()
	{
	}

	/*  各パーティクルの尤度算出  */
	double getLikelihoodND(const Position<> par_pos)
	{
		double dis = par_pos | mean;
		return (std::exp(-1 * std::pow(dis, 2) / (2.0 * variance)));
	}

	/**********************************************************/
	//	計測データのclear
	/**********************************************************/

	void clearMeasurement()
	{
		signal.clearData();
	}

	/**********************************************************/
	//  メンバ変数
	/**********************************************************/

	/*  計測データ  */
	GPGGA signal;										//	計測信号格納

														/*  尤度生成関連  */

														/*  正規分布モデル  */
	Position<> mean;
	double variance;
};

