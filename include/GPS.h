#pragma once

#include "myfun.h"
#include "parameter.h"

/*  �ޓx�����֘A  */

class Gps
{
public:
	/*  �R���X�g���N�^  */
	Gps() {};
	Gps(const Gps& obj)
	{
		/*  �v���f�[�^  */
		(this->signal) = (obj.signal);

		/*  �ޓx�����֘A  */

		/*  ���K���z���f��  */
		this->mean = obj.mean;
		this->variance = obj.variance;
	}
	Gps& operator=(const Gps& obj)
	{
		/*  �v���f�[�^  */
		(this->signal) = (obj.signal);

		/*  �ޓx�����֘A  */

		/*  ���K���z���f��  */
		this->mean = obj.mean;
		this->variance = obj.variance;

		return *this;
	}
	/*  �f�X�g���N�^*/
	~Gps()
	{
	}


	/**********************************************************/
	//	������
	/**********************************************************/

	/*  �n�}�̌��_��set  */
	template<typename T>
	void setMapOrgGL(T latitude, T longitude, T elevation)
	{
		signal.setIniPos(latitude, longitude, elevation);
	}


	/**********************************************************/
	//	�t�@�C���ǂݍ���
	/**********************************************************/

	/*  GPGGA�M���̓ǂݍ���  */
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
	//	GPGGA�ɂ�鐳�K���z���f��(ND)
	/**********************************************************/

	/*  ���K���z���f���̕��ςƕ��U������  */
	void setMeasND()
	{
	}

	/*  �e�p�[�e�B�N���̖ޓx�Z�o  */
	double getLikelihoodND(const Position<> par_pos)
	{
		double dis = par_pos | mean;
		return (std::exp(-1 * std::pow(dis, 2) / (2.0 * variance)));
	}

	/**********************************************************/
	//	�v���f�[�^��clear
	/**********************************************************/

	void clearMeasurement()
	{
		signal.clearData();
	}

	/**********************************************************/
	//  �����o�ϐ�
	/**********************************************************/

	/*  �v���f�[�^  */
	GPGGA signal;										//	�v���M���i�[

														/*  �ޓx�����֘A  */

														/*  ���K���z���f��  */
	Position<> mean;
	double variance;
};

