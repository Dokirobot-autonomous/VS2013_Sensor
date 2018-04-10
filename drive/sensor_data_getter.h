#pragma once

#include "include.h"
#include "settings.h"

/*
*******************************************************************************
* Function name : winsock2_open
* Description   : winsock2 Ȱ��ȭ
* Arguments     : none
* Returns       : ����Etrue, ���� false
* Notes         : none
*******************************************************************************
*/
bool winsock2_open()
{
	WSADATA wsad;

	if (WSAStartup(2, &wsad)) {
		printf("WSAStartup() Error in MS Windows\n");
		WSACleanup();
		return false;
	}

	return true;
}

/*
*******************************************************************************
* Function name : winsocke2_close
* Description   : winsock2 ��Ȱ��ȭ
* Arguments     : none
* Returns       : none
* Notes         : none
*******************************************************************************
*/
void winsocke2_close()
{
	WSACleanup();
}

/*
*******************************************************************************
* Function name : get_encoder_handler
* Description   : �������� ���ڴ�E�� �б�E
* Arguments     : none
* Returns       : ����Etrue, ���� false
* Notes         : none
*******************************************************************************
*/

bool get_encoder_handler()
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int connect_state;
	int encoder_l;
	int encoder_r;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, DRIVE_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false

	if (!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadEncoder", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}]",
				&encoder_l,
				&encoder_r);

			dsphal_datalist_destroy(datalist_ret);
			dsphal_tcp_client_destroy(tcp_client);
			printf("encoder left: %d \t right: %d \n", encoder_l, encoder_r);

			return true;
		}
		else {
			dsphal_tcp_client_destroy(tcp_client);
			printf("no return datalist\n");
			return false;
		}
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*
*******************************************************************************
* Function name : get_encoder_handler
* Description   : �������� �����޵帮 �� �б�E
* Arguments     : none
* Returns       : ����Etrue, ���� false
* Notes         : none
*******************************************************************************
*/
bool get_odometry_handler()
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int connect_state;
	int x;
	int y;
	int theta;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, DRIVE_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false

	if (!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadPosition", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}]",
				&x,
				&y,
				&theta);

			dsphal_datalist_destroy(datalist_ret);
			dsphal_tcp_client_destroy(tcp_client);
			printf("odometry x: %d \t y: %d \t theta: %d \n", x, y, theta);

			return true;
		}
		else {
			dsphal_tcp_client_destroy(tcp_client);
			printf("no return datalist\n");
			return false;
		}
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}

	return true;
}

int get_odometry_value(std::vector<int> *datalist)
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int connect_state;
	int x;
	int y;
	int theta;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, DRIVE_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);	//network connect 0: sucess, -1: false

	if (!connect_state) {
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadPosition", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}]",
				&x,
				&y,
				&theta);

			dsphal_datalist_destroy(datalist_ret);
			dsphal_tcp_client_destroy(tcp_client);
			//printf("odometry x: %d \t y: %d \t theta: %d \n", x, y, theta);

			(*datalist).resize(3);
			(*datalist)[0] = x;
			(*datalist)[1] = y;
			(*datalist)[2] = theta;

			return 1;
		}
		else {
			return 0;
		}
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return 0;
	}
}

/*
*******************************************************************************
* Function name : get_encoder_handler
* Description   : �����ο� �ӵ� ���� ��E�
* Arguments     : none
* Returns       : ����Etrue, ���� false
* Notes         : none
*******************************************************************************
*/
bool set_velocity_handler(int left_velocity, int right_velocity)
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_arg;
	dsphal_datalist_t *datalist_ret;

	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, DRIVE_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);

	if (!connect_state) {
		datalist_arg = dsphal_build_root_datalist("[{i}{i}]", left_velocity, right_velocity);

		datalist_ret = dsphal_request_method_call(tcp_client, "VelocityControl", datalist_arg);

		if (datalist_arg) dsphal_datalist_destroy(datalist_arg);
		if (datalist_ret) dsphal_datalist_destroy(datalist_ret);

		dsphal_tcp_client_destroy(tcp_client);
		return true;
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return false;
	}
}

/*Hand made function*/
void usonic_value(double *us_val)
{
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	int i;
	int usonic_sensor_val[7];
	int reserve;
	int connect_state;

	tcp_client = dsphal_tcp_client_create(IP_ADDR_TETRA_DS, USONIC_PORT);
	connect_state = dsphal_tcp_client_connect(tcp_client);

	if (!connect_state) {
		//datalist_ret = dsphal_request_method_call(tcp_client, "ReadUltraSonicSensorArray", NULL);
		datalist_ret = dsphal_request_method_call(tcp_client, "ReadSensors", NULL);

		if (datalist_ret) {
			dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}{i}{i}{i}{i}{i}]",
				&usonic_sensor_val[0],
				&usonic_sensor_val[1],
				&usonic_sensor_val[2],
				&usonic_sensor_val[3],
				&usonic_sensor_val[4],
				&usonic_sensor_val[5],
				&usonic_sensor_val[6],
				&reserve);

			dsphal_datalist_destroy(datalist_ret);

			//printf("ultra sonic sensor test : ");
			for (i = 0; i < 7; i++){
				//printf("%4d", usonic_sensor_val[i]);
				us_val[i] = (double)usonic_sensor_val[i];
			}

			dsphal_tcp_client_destroy(tcp_client);
			return;
		}
		else {
			dsphal_tcp_client_destroy(tcp_client);
			printf("no return datalist\n");
			return;
		}
	}
	else {
		dsphal_tcp_client_destroy(tcp_client);
		printf("network connect false !!!\n");
		return;
	}
}


class SensorDataGetter
{
private:

	std::vector<int> odo_ini;
	std::vector<int> odo;
	char time_c[128];
	SerialPort sp_gps;
	std::string str;
	cv::VideoCapture cap;//�f�o�C�X�̃I�[�v��
	cv::Mat img;
	/* LRF */
	urg_t urg1, urg2;
	long *length_data1, *length_data2;
	int ret1, ret2;
	AtrTsnd121 atr;

public:

	SensorDataGetter()
	{
	}

	~SensorDataGetter()
	{
		finalize();
	}

	void finalize()
	{
#if GET_ACCELATION || GET_GEOMAGNETIC || GET_GYRO
		atr.stop();
#endif

#if GET_GPS
		sp_gps.end(); // GPS�ւ̐ڑ������
#endif
#if GET_OMNI
		cv::destroyAllWindows();
#endif
#if GET_LRF_U
		urg_close(&urg1); // LRF�ւ̐ڑ������
#endif
#if GET_LRF_L
		urg_close(&urg2); // LRF�ւ̐ڑ������
#endif
		winsocke2_close(); // ���{�b�g�ւ̐ڑ������

	}

	void initialize()
	{
		try{
			// ���{�b�g�ɃA�N�Z�X
			if (!winsock2_open()){
				std::string str = "No Valid Robot!";
				throw str;
			}
			std::cout << "Complete Robot Initialization! " << std::endl;

			/********** �I�h���g���̏����� **********/
#if GET_ODOMETRY
			int ret; // �ڑ��C�擾�m�F�p
			ret = get_odometry_value(&odo_ini); // (0,0,0)�������ʒu�Ƃ���
			if (!ret){
				std::string str = "Odometry Initialization Failed!";
				throw str;
			}
			std::cout << "Complete Odometry Initialization! " << std::endl;
#endif
			/********** LRF�̏����� **********/
#if GET_LRF_U
			// LRF_UPPER
			int length_data_size1;
			const char connect_device1[] = LRF_UPPER_COM_PORT; // �v�ݒ�
			const long connect_baudrate1 = 115200;
			//const long connect_baudrate = 38400;
			ret = urg_open(&urg1, URG_SERIAL, connect_device1, connect_baudrate1); // �Z���T�ɐڑ�
			if (ret < 0){ // == 0�FSuccess  < 0�FError
				std::string str = "No Valid LRF!";
				throw str;
			}
			length_data1 = (long *)malloc(sizeof(long)* urg_max_data_size(&urg1)); // �f�[�^��M�̈���m��
			urg_set_scanning_parameter(&urg1, urg_deg2step(&urg1, -135), urg_deg2step(&urg1, 135), 0); //�X�L�����p�����[�^�̐ݒ� ���ʂ�0�x
			ret = urg_start_measurement(&urg1, URG_DISTANCE, 1, 0); // LRF�f�[�^�擾�i1�����Ă����������ǂ��j
			length_data_size1 = urg_get_distance(&urg1, length_data1, NULL); // �f�[�^�X�V�Ɠ����Ƀf�[�^�T�C�Y�擾
			std::cout << "Complete UPPER_LRF Initialization! " << std::endl;
#endif
#if GET_LRF_L
			// LRF_lower
			int length_data_size2;
			const char connect_device2[] = LRF_LOWER_COM_PORT; // �v�ݒ�
			const long connect_baudrate2 = 115200;
			//const long connect_baudrate = 38400;
			ret = urg_open(&urg2, URG_SERIAL, connect_device2, connect_baudrate2); // �Z���T�ɐڑ�
			if (ret < 0){ // == 0�FSuccess  < 0�FError
				std::string str = "No Valid LRF!";
				throw str;
			}
			length_data2 = (long *)malloc(sizeof(long)* urg_max_data_size(&urg2)); // �f�[�^��M�̈���m��
			urg_set_scanning_parameter(&urg2, urg_deg2step(&urg2, -135), urg_deg2step(&urg2, 135), 0); //�X�L�����p�����[�^�̐ݒ� ���ʂ�0�x
			ret = urg_start_measurement(&urg2, URG_DISTANCE, 1, 0); // LRF�f�[�^�擾�i1�����Ă���c�������ǂ��j
			length_data_size2 = urg_get_distance(&urg2, length_data2, NULL); // �f�[�^�X�V�Ɠ����Ƀf�[�^�T�C�Y�擾
			std::cout << "Complete LOWER_LRF Initialization! " << std::endl;
#endif
			/********** GPS�̏����� **********/
#if GET_GPS
			//sp_gps.start(buf2, 38400); // COM�ԍ��C�{�[���[�g
			sp_gps.start(buf2, 19200); // COM�ԍ��C�{�[���[�g
			if (sp_gps.ret < 0){
				std::string str = "No Valid GPS!";
				throw str;
			}
			std::cout << "Complete GPS Initialization! " << std::endl;
#endif
			/********** �S���ʃJ�����̏����� **********/
#if GET_OMNI
			cap.open(OMNI_CAMERA_NO);//�������ł��ǂ��D
			cap.set(CV_CAP_PROP_FPS, 1.0 / (UPDATE_TIME / 1000.0));
			if (!cap.isOpened())//�J�����f�o�C�X������ɃI�[�v���������m�F�D
			{
				//�ǂݍ��݂Ɏ��s�����Ƃ��̏���
				std::string str = "No Valid Camera!";
				throw str;
			}
			std::cout << "Complete OmniCamera Initialization! " << std::endl;
#endif
			/********** ATR�Z���T�̏����� **********/
#if GET_ACCELATION || GET_GEOMAGNETIC ||GET_GYRO
			atr.open("127.0.0.1", 10000);
			atr.setParamAccelgyro(UPDATE_TIME / 10, 10, 0);
			atr.setParamGeomagnetic(UPDATE_TIME / 10, 10, 0);
			atr.setParamPressure(0, 0, 0);
			atr.setParamBattery(0, 0);
			//atr.geoCalibration();
			std::cout << "Complete ATR Initialization! " << std::endl;
#endif
		}
		catch (std::string error_message){
			std::cout << error_message << std::endl;
			finalize();
			exit(0);
		}
	}
	
	void update()
	{
		/* ���Ԍv�� */
#if GET_TIME
		time_t now = time(NULL);
		struct tm *pnow = localtime(&now);
		sprintf_s(time_c, "%02d:%02d:%02d", pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
#endif
		/* �G���R�[�_�v�� */
#if GET_ODOMETRY
		get_odometry_value(&odo);
#endif
		//LRF_UPPER�擾
#if GET_LRF_U
		urg_start_measurement(&urg1, URG_DISTANCE, 1, 0); // LRF�f�[�^�擾
		ret1 = urg_get_distance(&urg1, length_data1, NULL); // �f�[�^�X�V�Ɠ����Ƀf�[�^�T�C�Y�擾
#endif
		//LRF_LOWER�擾
#if GET_LRF_L
		urg_start_measurement(&urg2, URG_DISTANCE, 1, 0); // LRF�f�[�^�擾
		ret2 = urg_get_distance(&urg2, length_data2, NULL); // �f�[�^�X�V�Ɠ����Ƀf�[�^�T�C�Y�擾
#endif
		/* �S���ʃJ���� */
#if GET_OMNI
		cap >> img;
#endif
		/* ATR�Z���T */
#if GET_ACCELATION || GET_GEOMAGNETIC || GET_GYRO
		atr.update();
		// �����x�̏������� */
		if (get_accelation)
		{
			sprintf_s(filename, "output/accelation/accelation_no%d_%dth.csv", NO, step);
			std::ofstream ofs_acc(filename, std::ios_base::out);
			if (ofs_acc.fail())
			{
				writeError(filename, false);
				end = 'e';
				break;
			}
			ofs_acc << atr.getAccelation()[0] << "," << atr.getAccelation()[1] << "," << atr.getAccelation()[2] << std::endl;
		}
		/* �W���C���̏������� */
		if (get_gyro){
			sprintf_s(filename, "output/gyro/gyro_no%d_%dth.csv", NO, step);
			std::ofstream ofs_gyr(filename, std::ios_base::out);
			if (ofs_gyr.fail())
			{
				writeError(filename, false);
				break;
			}
			ofs_gyr << atr.getGyro()[0] << "," << atr.getGyro()[1] << "," << atr.getGyro()[2] << std::endl;
		}
		// �n���C�̏�������
		if (get_geomagnetic){
			sprintf_s(filename, "output/geomagnetic/geomagnetic_no%d_%dth.csv", NO, step);
			std::ofstream ofs_geo(filename, std::ios_base::out);
			if (ofs_geo.fail())
			{
				writeError(filename, false);
				break;
			}
			ofs_geo << atr.getGeomagnetic()[0] << "," << atr.getGeomagnetic()[1] << "," << atr.getGeomagnetic()[2] << std::endl;
		}
#endif
		/* GPS�v�� */
#if GET_GPS
		//clock_t lap_gps_start = clock();
		sp_gps.rxLen = 0;
		while (1)
		{
			sp_gps.gets('$', '\n');
			if (sp_gps.rxFlag == true){ // '\n'���܂܂�Ă�����I��
				std::string str = sp_gps.rxData; // char�Ȃ̂�string�ɕϊ� ����Ŏ����ϊ��炵��
				std::cout << str << std::endl;
				break;
			}
			//clock_t lap3 = clock();
			//if ((int)(lap3 - lap_gps_start) > UPDATE_TIME*0.8){
			//	str = "No Data";
			//	std::cout << str << std::endl;
			//	break;	//	�v����������GPS���擾�ł��Ȃ�������break
			//}
		}
		std::cout << "Get GPS!" << std::endl;
		// GPS�f�[�^���t�@�C���o��
		//filename[256];
#endif

	}
	
	void save(int no, int step){
		char filename[256];
		try{
			// �擾���ԃf�[�^���t�@�C���o��
#if GET_TIME
			sprintf_s(filename, "./output/time/time_no%d_%dth.csv", no, step);
			std::ofstream ofs_time(filename, ios_base::out);
			if (ofs_time.fail())
			{
				throw filename;
			}
			ofs_time << time_c << std::endl;
#endif
			/* �G���R�[�_�v�� */
#if GET_ODOMETRY
			sprintf_s(filename, "./output/odometry/odometry_no%d_%dth.csv", no, step);
			std::ofstream ofs_odo(filename, ios_base::out);
			if (ofs_odo.fail())
			{
				throw filename;
			}
			ofs_odo << odo[0] - odo_ini[0] << "," << odo[1] - odo_ini[1] << "," << odo[2] - odo_ini[2] << std::endl;
#endif
			//LRF_UPPER�擾
#if GET_LRF_U
			urg_start_measurement(&urg1, URG_DISTANCE, 1, 0); // LRF�f�[�^�擾
			ret1 = urg_get_distance(&urg1, length_data1, NULL); // �f�[�^�X�V�Ɠ����Ƀf�[�^�T�C�Y�擾
			// LRF�f�[�^���t�@�C���o��
			//char filename[256];
			sprintf_s(filename, "./output/lrf_upper/lrf_no%d_%dth.csv", NO, step);
			std::ofstream ofs_lrf1(filename, ios_base::out);
			if (ofs_lrf1.fail())
				throw filename;
			for (int i = 0; i < ret1; i++){
				ofs_lrf1 << length_data1[i] << "," << urg_index2rad(&urg1, i) << std::endl;
			}
#endif
			//LRF_LOWER�擾
#if GET_LRF_L
			urg_start_measurement(&urg2, URG_DISTANCE, 1, 0); // LRF�f�[�^�擾
			ret2 = urg_get_distance(&urg2, length_data2, NULL); // �f�[�^�X�V�Ɠ����Ƀf�[�^�T�C�Y�擾
			// LRF�f�[�^���t�@�C���o��
			sprintf_s(filename, "./output/lrf_lower/lrf_no%d_%dth.csv", NO, step);
			std::ofstream ofs_lrf2(filename, ios_base::out);
			if (ofs_lrf2.fail()) 
				throw filename;
			for (int i = 0; i < ret2; i++){
				ofs_lrf2 << length_data2[i] << "," << urg_index2rad(&urg2, i) << std::endl;
			}
#endif
		/* �S���ʃJ���� */
#if GET_OMNI
		//clock_t start = clock();
			// �J�����f�[�^���t�@�C���o��
			sprintf_s(filename, "./output/img/img_no%d_%dth.bmp", NO, step);
			cv::imwrite(filename, img);
#endif
			/* ATR�Z���T */
#if GET_ACCELATION || GET_GEOMAGNETIC || GET_GYRO
			atr.update();
			// �����x�̏������� */
			if (get_accelation)
			{
				sprintf_s(filename, "output/accelation/accelation_no%d_%dth.csv", NO, step);
				std::ofstream ofs_acc(filename, std::ios_base::out);
				if (ofs_acc.fail())
				{
					writeError(filename, false);
					end = 'e';
					break;
				}
				ofs_acc << atr.getAccelation()[0] << "," << atr.getAccelation()[1] << "," << atr.getAccelation()[2] << std::endl;
			}
			/* �W���C���̏������� */
			if (get_gyro){
				sprintf_s(filename, "output/gyro/gyro_no%d_%dth.csv", NO, step);
				std::ofstream ofs_gyr(filename, std::ios_base::out);
				if (ofs_gyr.fail())
				{
					writeError(filename, false);
					break;
				}
				ofs_gyr << atr.getGyro()[0] << "," << atr.getGyro()[1] << "," << atr.getGyro()[2] << std::endl;
			}
			// �n���C�̏�������
			if (get_geomagnetic){
				sprintf_s(filename, "output/geomagnetic/geomagnetic_no%d_%dth.csv", NO, step);
				std::ofstream ofs_geo(filename, std::ios_base::out);
				if (ofs_geo.fail())
				{
					writeError(filename, false);
					break;
				}
				ofs_geo << atr.getGeomagnetic()[0] << "," << atr.getGeomagnetic()[1] << "," << atr.getGeomagnetic()[2] << std::endl;
			}
#endif
			/* GPS�v�� */
#if GET_GPS
			sp_gps.rxLen = 0;
			sprintf_s(filename, "./output/gps/gps_no%d_%dth.txt", NO, step);
			std::ofstream ofs_gps(filename, ios_base::out);
			if (ofs_gps.fail())
				throw filename;
			ofs_gps << str << std::endl;
#endif
		}
		catch (std::string error_message){
			std::cout << error_message << "cannot be created!" << std::endl;
			finalize();
			exit(0);
		}
	}

	void waitUpdateTime(clock_t start){
		clock_t end = clock();

		int rest_time = UPDATE_TIME - (int)(end - start);
		if (rest_time > 0){
			std::this_thread::sleep_for(std::chrono::microseconds(rest_time));
		}

	}
};

