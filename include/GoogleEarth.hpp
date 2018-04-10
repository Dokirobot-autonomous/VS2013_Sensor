#pragma once

#include <sstream>
#include <string>
#include <cmath>
#include "Position.hpp"
#define M_PI 3.14159265359

// blh�F�ܓx�C�o�x�C����
// GoogleEarth�̉E���̍��x�̓r���[�̌����낵�ʒu�Ȃ̂Ŋ֌W�Ȃ�
// 60�i�@�̗��_�͉��ʂ�2���ŕ\����_

namespace GEParam{
	static const double F(1.0/298.257223563); // �Ε���
	static const double A(6378137.0); // �ԓ��ʕ��ϔ��a[m]
	static const double B(A*(1.0 - F));
	static const double E2(F*(2-F)); // ���S��
	static const double ED2(E2*A*A/(B*B));
	static const double geoidHeight(35.7); // WGS-84�ȉ~�̂��畽�ϊC���ʂ̍��x��[m] ����ł�35.7[m]
};

struct GoogleEarth{
	double latitude;		//�ܓx (�k��)
	double longitude;		//�o�x (���o)
	double elevation;		//���ϊC���ʂ���̃A���e�i���x (m)
private:
	static const int METRE = 1000;
	
	/* �����ʒu�֘A */
	// ����`�������W�����ʒu�iGoogleEarth�j
	//  35�� 9'18.10"N�C136��57'58.65"E �� 60�i�@
	// �@����ʁF26���i�k��0���Ƃ��Ď��v���𐳂Ƃ���p�x�j
	double lat0 =  35091810;
	double lon0 = 136575865;
	double ele0 = 39;

	double x0;
	double y0;
	double z0;
	/* �����܂� */

public:
	GoogleEarth():
		latitude(0),
		longitude(0),
		elevation(0)
	{
		setIniPos();
	}

	GoogleEarth(const double lat, const double lon, const double ele) 
	{
		setData(lat, lon, ele);
	}

	void setIniPos(double lat, double lon, double ele){
		lat0 = lat;	//	�
		lon0 = lon;
		ele0 = ele;
		setIniPos();
	}

	void setData(const double lat, const double lon, double ele){
		latitude = lat;
		longitude = lon;
		elevation = ele;
	}

	std::vector<double> get_ENU(){
		convECEF();
		convENU();
		std::vector<double> pos;
		pos.resize(3);
		pos[0] = E*METRE;	//	East
		pos[1] = N*METRE;	//	North
		pos[2] = U*METRE;
 		return pos;
	}

	std::vector<double> ENU2GE(double e, double n, double u){
		using namespace GEParam;
		std::vector<double> blh = convENU2BLH(e, n, u);
		std::vector<double> vec = { convFormatDeg2GE(blh[0]), convFormatDeg2GE(blh[1]), blh[2] - geoidHeight };
		return vec;
	}

	std::vector<double> ENU2GPGGA(double e, double n, double u){
		using namespace GEParam;
		std::vector<double> blh = convENU2BLH(e, n, u);
		std::vector<double> vec = { convFormatDeg2GPGGA(blh[0]), convFormatDeg2GPGGA(blh[1]), blh[2] - geoidHeight };
		return vec;
	}

private:
	void setIniPos(){
		//std::cout << "setIniPos" << std::endl;
		using namespace GEParam;
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
	double convFormatGPGGA(double dat){
		double d(static_cast<int>(dat / 100));
		double m(dat - d * 100);
		return d + m / 60;
	}
	double convFormatGE(double dat){ // dddmmssss��dddddd�ɕϊ�
		double d(static_cast<int>(dat*1e-6));
		double m(static_cast<int>(dat*1e-4 - d*1e2));
		double s(dat*1e-2 - d*1e4 - m*1e2);
		//std::cout << "d:" <<d << " m:" << m << " s:" << s << std::endl;
		//std::cout << d + m / 60 + s / 3600 << std::endl;
		return d + m / 60 + s / 3600;
	}
	double convFormatDeg2GE(double dat){ // dddddd��dddmmssss�ɕϊ�
		double d(static_cast<int>(dat));
		double m(static_cast<int>((dat - d) * 60));
		double s((dat - d - m / 60) * 3600);
		return d*1e6 + m*1e4 + s*1e2;
	}
	double convFormatDeg2GPGGA(double dat){ // dddddd��dddmm.mm�ɕϊ�
		double d(static_cast<int>(dat));
		double m((dat - d) * 60);
		return d*1e2 + m;
	}

	//GPS�̃f�[�^(�ܓx,�o�x)�𒼌����W�֕ϊ� �Q�l:http://www.enri.go.jp/~fks442/K_MUSEN/
	double xe,ye,ze; //ECEF���W
	void convECEF(){
		//std::cout << "convECEF" << std::endl;
		using namespace GEParam;
		double lat(convFormatGE(latitude)*M_PI / 180);
		double lon(convFormatGE(longitude)*M_PI / 180);
		double h = elevation + geoidHeight;
		double N = A / sqrt(1.0 - E2*pow(sin(lat), 2));
		xe = (N + h)*cos(lat)*cos(lon);
		ye = (N + h)*cos(lat)*sin(lon);
		ze = (N*(1.0 - E2) + h)*sin(lat);
	}
	//ENU���W(�n�����W)�ϊ�
	double E,N,U;
	void convENU(){
		//std::cout << "convENU" << std::endl;
		using namespace GEParam;

		// �u���_�����W�ϊ����Ă��邾���v�Ɖ��߂���D�܂�{����x'=Rx-x0�Ƃ��Ă����Ȃ��ix0��\�ߌv�Z���Ă����j�D
		// �����l����ƉԈ䂳��̃v���O�����͋��炭�Ԉ���Ă���D�����ł̈ܓx�C�o�x�͌��_�̂��́D
		// �����łȂ��ƕ����̌v���l���ɈقȂ���W�n�ɕϊ����邱�ƂɂȂ�D
		double SinLat(sin(convFormatGE(lat0)*M_PI / 180));
		double CosLat(cos(convFormatGE(lat0)*M_PI / 180));
		double SinLon(sin(convFormatGE(lon0)*M_PI / 180));
		double CosLon(cos(convFormatGE(lon0)*M_PI / 180));

		E = -SinLon*(xe - x0) + CosLon*(ye - y0); // East = x
		N = -SinLat*CosLon*(xe - x0) - SinLat*SinLon*(ye - y0) + CosLat*(ze - z0); // North = y
		U = CosLat*CosLon*(xe - x0) + CosLat*SinLon*(ye - y0) + SinLat*(ze - z0); // Up
	}

	std::vector<double> convENU2BLH(double east, double north, double up){
		using namespace GEParam;

		// �u���_�����W�ϊ����Ă��邾���v�Ɖ��߂���D�܂�{����x'=Rx-x0�Ƃ��Ă����Ȃ��ix0��\�ߌv�Z���Ă����j�D
		// �����l����ƉԈ䂳��̃v���O�����͋��炭�Ԉ���Ă���D�����ł̈ܓx�C�o�x�͌��_�̂��́D
		// �����łȂ��ƕ����̌v���l���ɈقȂ���W�n�ɕϊ����邱�ƂɂȂ�D
		double SinLat(sin(convFormatGE(lat0)*M_PI / 180));
		double CosLat(cos(convFormatGE(lat0)*M_PI / 180));
		double SinLon(sin(convFormatGE(lon0)*M_PI / 180));
		double CosLon(cos(convFormatGE(lon0)*M_PI / 180));

		double e = east / METRE - SinLon*x0 + CosLon*y0;
		double n = north / METRE - SinLat*CosLon*x0 - SinLat*SinLon*y0 + CosLat*z0;
		double u = up / METRE + CosLat*CosLon*x0 + CosLat*SinLon*y0 + SinLat*z0;

		double a = SinLat;
		double b = CosLat;
		double c = SinLon;
		double d = CosLon;

		double x = 1 / (c*c + d*d) * (-c*e - a*d / (a*a + b*b)*n + b*d / (a*a + b*b)*u);
		double y = 1 / (c*c + d*d) * (d*e - a*c / (a*a + b*b)*n + b*c / (a*a + b*b)*u);
		double z = (b*n + a*u) / (a*a + b*b);

		double p = sqrt(pow(x, 2) + pow(y, 2));
		double theta = 180 / M_PI * atan2(z*A, p*B);

		double lat = 180 / M_PI * atan2(z + ED2*B*pow(sin(theta*M_PI / 180.0), 3), p - E2*A*pow(cos(theta*M_PI / 180.0), 3));
		double lon = 180 / M_PI * atan2(y,x);
		double h = p / cos(lat*M_PI / 180) - A / sqrt(1.0 - E2*pow(sin(lat*M_PI / 180.0), 2));

		//std::cout << lat << " " << lon << " " << h << std::endl;

		std::vector<double> vec = { lat, lon, h };
		return vec;
	}


	/*double NN(double p){
		using namespace GEParam;
		return A/sqrt(1.0-E2*pow(sin(p*M_PI/180.0),2));
	}*/
};