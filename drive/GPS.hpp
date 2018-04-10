#pragma once

#include <sstream>
#include <string>

namespace GPSParam{
	static const double F(1.0/298.257223563);
	static const double A(6378137);
	static const double E2(F*(2-F));
	//���W�̊ @���H�� �ܓx(35.183017)�o�x(137.112183)����(170.0+35.8)
	static const double X0(-3823839.880430);
	static const double Y0(3551816.409571);
	static const double Z0(3654599.184320);
};

struct GPGGA{
	int positioningTime;	//���ʎ��� (UTC)
	double latitude;		//�ܓx (�k��)
	double longitude;		//�o�x (���o)
	int quality;			//GPS�̃N�I���e�B: 0=��M�s�\, 1=�P�Ƒ���, 2=DGPS
	int satellitesNum;		//��M�q����
	double hdop;			//HDOP
	double elevation;		//���ϊC���ʂ���̃A���e�i���x (m)
	double geoidHeight;		//WGS-84�ȉ~�̂��畽�ϊC���ʂ̍��x�� (m)
	double age;				//DGPS�f�[�^�̃G�C�W(�b)
//	int id;					//DGPS��ǂ�ID
//	std::string checksum;	//�`�F�b�N�T��
private:
	std::string fullDat;	//GPGGA�`�����̂܂�
	bool empty_;			//�f�[�^���Ȃ�or�s��������
	static const int dataNum = 14;
	static const int METRE = 1000;

public:
	GPGGA():
		positioningTime(0),
		latitude(0),
		longitude(0),
		quality(0),
		satellitesNum(0),
		hdop(0),
		elevation(0),
		geoidHeight(0),
		age(0),
//		id(0),
//		checksum("00"),
		empty_(true)
	{
	}

	GPGGA(const std::string &dat)
	{
		setData(dat);
	}

	void setData(const std::string &dat){
		fullDat = dat;
		//std::vector<std::string> v(hutil::String::split(dat, ","));
		std::vector<std::string> v(Split(dat, ","));
		if(v.size() < dataNum){ empty_ = true; return; }
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

	std::string str()const{return fullDat;}
	bool empty()const{return empty_;}

	std::vector<double> get_XY(){
		convECEF();
		convENU();

		std::vector<double> pos_xy;
		pos_xy.resize(2);
		pos_xy[0] = E*METRE;
		pos_xy[1] = N*METRE;

		return pos_xy;
	}
	std::string get_XY_str(){
		convECEF();
		convENU();
		std::stringstream ss;
		ss<<E*METRE<<"\t"<<N*METRE<<"\t"<<U;
		return ss.str();
	}

private:
	template<typename T> T string_cast(std::string str){
		std::stringstream ss(str);
		T dat;
		ss >> dat;
		return dat;
	}

	double convFormat(double dat){
		double deg(static_cast<int>(dat/100));
		double m(dat-deg*100);
		dat = deg+m/60;
		return dat;
	}

	//GPS�̃f�[�^(�ܓx,�o�x)�𒼌����W�֕ϊ� �Q�l:http://www.enri.go.jp/~fks442/K_MUSEN/
	double xe,ye,ze; //ECEF���W
	void convECEF(){
		using namespace GPSParam;
		double lat(convFormat(latitude)*M_PI/180);
		double lon(convFormat(longitude)*M_PI/180);
		double R = A/sqrt(1.0-E2*pow(sin(lat),2));
		xe = (R+elevation+geoidHeight)*cos(lat)*cos(lon);
		ye = (R+elevation+geoidHeight)*cos(lat)*sin(lon);
		ze = (R*(1.0-E2)+elevation+geoidHeight)*sin(lat);
	}
	//ENU���W(�n�����W)�ϊ�
	double E,N,U;
	void convENU(){
		using namespace GPSParam;
		double SinLat(sin(convFormat(latitude)*M_PI/180));
		double CosLat(cos(convFormat(latitude)*M_PI/180));
		double SinLon(sin(convFormat(longitude)*M_PI/180));
		double CosLon(cos(convFormat(longitude)*M_PI/180));

		E = -SinLon*(xe-X0) + CosLon*(ye-Y0);
		N = -SinLat*CosLon*(xe-X0) - SinLat*SinLon*(ye-Y0) + CosLat*(ze-Z0);
		U = CosLat*CosLon*(xe-X0) + CosLat*SinLon*(ye-Y0) + SinLat*(ze-Z0);
	}
	std::vector<string> Split(const std::string &dat, const string delim){
		std::vector<string> res;
		res.resize(0);
		size_t current = 0, found;
		while ((found = dat.find_first_of(delim, current)) != string::npos){ // �����ň�C�ɓǂݎ��
			res.push_back(string(dat, current, found - current));
			current = found + 1;
		}
		// �����񖼁C�X�^�[�g�ʒu�C������
		res.push_back(string(dat, current, dat.size() - current));
		return res;
	}
};
