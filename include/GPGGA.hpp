#pragma once

#include <sstream>
#include <string>

// blh：緯度，経度，高さ

namespace GPSParam{
	static const double F(1.0 / 298.257223563); // 偏平率
	static const double A(6378137); // 赤道面平均半径
	static const double E2(F*(2 - F)); // 離心率
};

struct GPGGA{
	int positioningTime;	//測位時刻 (UTC)
	double latitude;		//緯度 (北緯)
	double longitude;		//経度 (東経)
	int quality;			//GPSのクオリティ: 0=受信不能, 1=単独測位, 2=DGPS
	int satellitesNum;		//受信衛星数
	double hdop;			//HDOP
	double elevation;		//平均海水面からのアンテナ高度 (m)
	double geoidHeight;		//WGS-84楕円体から平均海水面の高度差 (m)
	double age;				//DGPSデータのエイジ(秒)
	//	int id;					//DGPS基準局のID
	//	std::string checksum;	//チェックサム
private:
	std::string fullDat;	//GPGGA形式そのまま
	bool empty_;			//データがないor不正だった
	static const int dataNum = 14;
	static const int METRE = 1000;

	double lat0 = 35091810;
	double lon0 = 136575865;
	double ele0 = 39;

	double x0;
	double y0;
	double z0;
	/* ここまで */

public:
	GPGGA() :
		positioningTime(0),
		latitude(0),
		longitude(0),
		quality(0),
		satellitesNum(0),
		hdop(0),
		elevation(0),
		geoidHeight(35.7), // 名大付近
		age(0),
		//		id(0),
		//		checksum("00"),
		empty_(true)

	{
	}

	GPGGA(const std::string &dat){
		setData(dat);
	}
	void setIniPos(double lat, double lon, double ele){
		lat0 = lat;
		lon0 = lon;
		ele0 = ele;
		setIniPos();
	}
	void setData(const std::string &dat){
		fullDat = dat;
		std::vector<std::string> v(Split(dat, ","));
		if (v.size() < dataNum){ empty_ = true; return; }
		positioningTime = string_cast<int>(v.at(1));
		latitude = string_cast<double>(v.at(2));
		longitude = string_cast<double>(v.at(4));
		quality = string_cast<int>(v.at(6));
		satellitesNum = string_cast<int>(v.at(7));
		hdop = string_cast<double>(v.at(8));
		elevation = string_cast<double>(v.at(9));
		geoidHeight = string_cast<double>(v.at(11));
		age = string_cast<double>(v.at(13));
		//		id = string_cast<int>(v.at(14));
		//		checksum = v.at(14).substr(5);
		empty_ = false;
	}
	void clearData()
	{
		fullDat = "";
		positioningTime = 0;
		latitude = 0;
		longitude = 0;
		quality = 0;
		satellitesNum = 0;
		hdop = 0;
		elevation = 0;
		geoidHeight = 0;
		age = 0;
		empty_ = true;
	}

	std::string str()const{ return fullDat; }
	bool empty()const{ return empty_; }

	std::vector<double> get_ENU(){
		convECEF();
		convENU();
		std::vector<double> pos;
		pos.resize(3);
		pos[0] = E*METRE;
		pos[1] = N*METRE;
		pos[2] = U*METRE;
		return pos;
	}
private:
	void setIniPos(){
		//std::cout << "setIniPos" << std::endl;
		using namespace GPSParam;
		double lat(convFormatGE(lat0)*M_PI / 180);
		double lon(convFormatGE(lon0)*M_PI / 180);
		double h = ele0 + geoidHeight;
		double N = A / sqrt(1.0 - E2*pow(sin(lat), 2));
		x0 = (N + h)*cos(lat)*cos(lon);
		y0 = (N + h)*cos(lat)*sin(lon);
		z0 = (N*(1.0 - E2) + h)*sin(lat);
	}
	template<typename T> T string_cast(std::string str){
		std::stringstream ss(str);
		T dat;
		ss >> dat;
		return dat;
	}
	double convFormatGE(double dat){ // dddmmssssをddddddに変換
		double d(static_cast<int>(dat*1e-6));
		double m(static_cast<int>(dat*1e-4 - d*1e2));
		double s(dat*1e-2 - d*1e4 - m*1e2);
		//std::cout << "d:" << d << " m:" << m << " s:" << s << std::endl;
		//std::cout << d + m / 60 + s / 3600 << std::endl;
		return d + m / 60 + s / 3600;
	}
	double convFormatGPGGA(double dat){
		double deg(static_cast<int>(dat / 100));
		double m(dat - deg * 100);
		dat = deg + m / 60;
		return dat;
	}
	//GPSのデータ(緯度,経度)を直交座標へ変換 参考:http://www.enri.go.jp/~fks442/K_MUSEN/
	double xe, ye, ze; //ECEF座標
	void convECEF(){
		using namespace GPSParam;
		double lat(convFormatGPGGA(latitude)*M_PI / 180);
		double lon(convFormatGPGGA(longitude)*M_PI / 180);
		double N = A / sqrt(1.0 - E2*pow(sin(lat), 2));
		double h = elevation + geoidHeight;
		xe = (N + h)*cos(lat)*cos(lon);
		ye = (N + h)*cos(lat)*sin(lon);
		ze = (N*(1.0 - E2) + h)*sin(lat);
	}
	//ENU座標(地平座標)変換
	double E, N, U;
	void convENU(){
		using namespace GPSParam;

		// 「原点も座標変換しているだけ」と解釈する．つまり本来はx'=Rx-x0としても問題ない（x0を予め計算しておく）．
		// そう考えると花井さんのプログラムは恐らく間違っている．ここでの緯度，経度は原点のもの．
		// そうでないと複数の計測値毎に異なる座標系に変換することになる．
		double SinLat(sin(convFormatGE(lat0)*M_PI / 180));
		double CosLat(cos(convFormatGE(lat0)*M_PI / 180));
		double SinLon(sin(convFormatGE(lon0)*M_PI / 180));
		double CosLon(cos(convFormatGE(lon0)*M_PI / 180));

		E = -SinLon*(xe - x0) + CosLon*(ye - y0); // East = x
		N = -SinLat*CosLon*(xe - x0) - SinLat*SinLon*(ye - y0) + CosLat*(ze - z0); // North = y
		U = CosLat*CosLon*(xe - x0) + CosLat*SinLon*(ye - y0) + SinLat*(ze - z0); // Up
	}
	std::vector<std::string> Split(const std::string &dat, const std::string delim){
		std::vector<std::string> res;
		res.resize(0);
		size_t current = 0, found;
		while ((found = dat.find_first_of(delim, current)) != std::string::npos){ // 一気に読み取り
			res.push_back(std::string(dat, current, found - current));
			current = found + 1;
		}
		// 文字列名，スタート位置，文字列長
		res.push_back(std::string(dat, current, dat.size() - current));
		return res;
	}
};