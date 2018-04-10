/*******************************************************************************
*
* File name     : atr_tsnd121.hpp
* Programmer(s) : Nozomu OHASHI
* Description   : ATR�Z���T(TsnD121)�̃N���X�p�w�b�_
*
*******************************************************************************/
#pragma once

#include <vector>
#include <iostream>
#include <sstream>
#include <string>

/*******************************************************************************
* Windows�\�P�b�g�ʐM���C�u�����̗��p�ݒ�
*******************************************************************************/
#include <Winsock2.h>
#include <ws2tcpip.h>
#pragma comment( lib, "ws2_32.lib" )
//#pragma comment(lib,"wsock32.lib")
//#pragma comment(lib,"Ws2_32.lib")
//#pragma_comment(lib,"Ws2_32.lib")

/*******************************************************************************
* �N���X���ϐ��E�֐���`
*******************************************************************************/
class AtrTsnd121
{
	/***** ��������:�v���C�x�[�g�ϐ� *****/
	/* �ʐM�p */
	struct sockaddr_in dst_addr;
	int dst_socket; // socket?
	/* ����p */
	int param_accelgyro[3];		// �����x�v&�W���C���Z���T�� 0:�v������, 1:�o�͎���, 2:�L�^����
	int param_geomagnetic[3];	// �n���C�Z���T�� 0:�v������, 1:�o�͎���, 2:�L�^����
	int param_pressure[3];		// �C���Z���T�� 0:�v������, 1:�o�͎���, 2:�L�^����
	bool param_battery[2];		// �o�b�e���d���� 0:�o�͗L��, 1:�L�^�L��
	/* �ϑ��p */
	int time_accelgyro;
	int time_geomagnetic;
	int time_pressure;
	int time_battery;
	double accelation[3];
	double gyro[3];
	double geomagnetic[3];
	double pressure[3];
	double battery[2];
	// �f�[�^�̑���M�p�z��
	char sendbuf[1024];
	char recvbuf[1024];
	/***** �����܂�:�v���C�x�[�g�ϐ� *****/

	/***** ��������:�v���C�x�[�g�֐� *****/
	char* unicode2str(const WCHAR* pszWchar);
	bool checkRecvQueue();
	std::string getDataName(char* str);
	void waitOK();
	/***** �����܂�:�v���C�x�[�g�֐� *****/



public:
	/***** ��������:�p�u���b�N�֐� *****/
	AtrTsnd121(); // �R���X�g���N�^
	AtrTsnd121(char* server_ip_addr, int ip_port_num); // �R���X�g���N�^
	~AtrTsnd121(); // �f�X�g���N�^
	void open(char* server_ip_addr, int ip_port_num);
	void start();
	void stop();
	void setParamAccelgyro(int meas_period, int out_ave_num, int save_ave_num);
	void setParamGeomagnetic(int meas_period, int out_ave_num, int save_ave_num);
	void setParamPressure(int meas_period, int out_ave_num, int save_ave_num);
	void setParamBattery(bool out_battery_, bool save_battery_);
	int update();				// [�Ԃ�l] 0:ags, 1:geo, 2:pres, 3:batt
	void waitRecv();
	void geoCalibration();
	std::vector<double> getAccelation();
	std::vector<double> getGyro();
	std::vector<double> getGeomagnetic();
	std::vector<double> getPressure();
	std::vector<double> getBattery();
	int getTimeAccelgyro();
	int getTimeGeomagnetic();
	int getTimePressure();
	int getTimeBattery();

	//void readMemdata(int entry_num);
	//void saveMemdata(int entry_num);

	/***** �����܂�:�p�u���b�N�֐� *****/



};

