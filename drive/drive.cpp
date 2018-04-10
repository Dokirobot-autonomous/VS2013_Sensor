// Updated 2015/09/13 by Suyama

#pragma once
#ifndef _STDINT
#define _STDINT
#endif

#include "include.h"
#include "settings.h"
#include "sensor_data_getter.h"

// �f�[�^�擾�I������
static bool finish_getting_data = false;
void finalizeProcessing(int signa){
	finish_getting_data = true;
	std::cout << "Finish getting data!" << std::endl;
}

/******************** ��������main�֐��@********************/
int main()
{
	SensorDataGetter sdg;

	sdg.initialize();

	/*  �v���J�n */
	cv::namedWindow("Press any key to start!");
	cv::waitKey();
	std::cout << "Get Started!" << std::endl;

	int no = NO;
	int step = 1;
	while (!finish_getting_data){
		signal(SIGINT, finalizeProcessing);	// ctrl + C�ŏI��
		clock_t start = clock();
		sdg.update();
		sdg.save(no, step);
		sdg.waitUpdateTime(start);
		step++;
	}

	sdg.finalize();

	return 0;
}


//int main()
//{
//
//
//	/* ATR�Z���T�̃|�W�V�������� */
//#if GET_ACCELATION || GET_GEOMAGNETIC ||GET_GYRO
//	{
//		bool flag_end = 0;
//		std::thread th_cont([&flag_end]{
//			cv::namedWindow("Press any key if the head of ATR is OK. ");
//			cv::waitKey();
//			flag_end = 1;
//		});
//
//		/* ATR�Z���T�v�� */
//		std::thread th_atr([&flag_end, &atr]{
//			atr.start();
//			while (flag_end != 1)
//			{
//				if (atr.update() == 0)	// ags�̍X�V���������ꍇ
//				{
//					std::cout << atr.getAccelation()[0] << "," << atr.getAccelation()[1] << "," << atr.getAccelation()[2] << std::endl;
//				};
//			}
//			atr.stop();
//		});
//
//		th_cont.join();
//		th_atr.join();
//	}
//#endif
//	/*********   �����܂�   *********/
//
//
//	/* �ϐ���` */
//	clock_t start, end;
//
//
//	int step = 1;
//	char filename[256];
//
//	/*  �v���J�n */
//	cv::namedWindow("Press any key to start!");
//	cv::waitKey();
//	std::cout << "Get Started!" << std::endl;
//
//	/* ATR�J�n */
//#if GET_ACCELATION || GET_GEOMAGNETIC ||GET_GYRO
//	atr.start();
//#endif
//
//
//	/* �ċN���� */
//	while (!finish_getting_data){
//		signal(SIGINT, finalizeProcessing);	// ctrl + C�ŏI��
//
//		start = clock();	// �J�n���� 
//		/* ���Ԍv�� */
//#if GET_TIME
//		time_t now = time(NULL);
//		struct tm *pnow = localtime(&now);
//		char time_c[128];
//		sprintf_s(time_c, "%02d:%02d:%02d", pnow->tm_hour, pnow->tm_min, pnow->tm_sec);
//		// �擾���ԃf�[�^���t�@�C���o��
//		sprintf_s(filename, "./output/time/time_no%d_%dth.csv", NO, step);
//		std::ofstream ofs_time(filename, ios_base::out);
//		if (ofs_time.fail())
//		{
//			writeError(filename, false);
//			break;
//		}
//		ofs_time << time_c << std::endl;
//		std::cout << "Get Time!" << std::endl;
//#endif
//		/* �G���R�[�_�v�� */
//#if GET_ODOMETRY
//		std::vector<int> odo;
//		get_odometry_value(&odo);
//		/* �t�@�C���o�� */
//		//char filename[256];
//		sprintf_s(filename, "./output/odometry/odometry_no%d_%dth.csv", NO, step);
//		std::ofstream ofs_odo(filename, ios_base::out);
//		if (ofs_odo.fail())
//		{
//			writeError(filename, false);
//			break;
//		}
//		ofs_odo << odo[0] - odo_ini[0] << "," << odo[1] - odo_ini[1] << "," << odo[2] - odo_ini[2] << std::endl;
//		std::cout << "Get Odometry!" << std::endl;
//#endif
//		//LRF_UPPER�擾
//#if GET_LRF_U
//		{
//			urg_start_measurement(&urg1, URG_DISTANCE, 1, 0); // LRF�f�[�^�擾
//			int ret = urg_get_distance(&urg1, length_data1, NULL); // �f�[�^�X�V�Ɠ����Ƀf�[�^�T�C�Y�擾
//			// LRF�f�[�^���t�@�C���o��
//			//char filename[256];
//			sprintf_s(filename, "./output/lrf_upper/lrf_no%d_%dth.csv", NO, step);
//			std::ofstream ofs_lrf(filename, ios_base::out);
//			if (ofs_lrf.fail())
//			{
//				writeError(filename, false);
//				break;
//			}
//			for (int i = 0; i < ret; i++){
//				ofs_lrf << length_data1[i] << "," << urg_index2rad(&urg1, i) << std::endl;
//			}
//			std::cout << "Get LRF_U!" << std::endl;
//		}
//#endif
//		//LRF_LOWER�擾
//#if GET_LRF_L
//		{
//			urg_start_measurement(&urg2, URG_DISTANCE, 1, 0); // LRF�f�[�^�擾
//			int ret = urg_get_distance(&urg2, length_data2, NULL); // �f�[�^�X�V�Ɠ����Ƀf�[�^�T�C�Y�擾
//			// LRF�f�[�^���t�@�C���o��
//			sprintf_s(filename, "./output/lrf_lower/lrf_no%d_%dth.csv", NO, step);
//			std::ofstream ofs_lrf(filename, ios_base::out);
//			if (ofs_lrf.fail())
//			{
//				writeError(filename, false);
//				break;
//			}
//			for (int i = 0; i < ret; i++){
//				ofs_lrf << length_data2[i] << "," << urg_index2rad(&urg2, i) << std::endl;
//			}
//			std::cout << "Get LRF_L!" << std::endl;
//		}
//#endif
//		/* �S���ʃJ���� */
//#if GET_OMNI
//		//clock_t start = clock();
//		cv::Mat img;
//		cap >> img;
//		// �J�����f�[�^���t�@�C���o��
//		sprintf_s(filename, "./output/img/img_no%d_%dth.bmp", NO, step);
//		cv::imwrite(filename, img);
//		std::cout << "Get Omni!" << std::endl;
//#endif
//		/* ATR�Z���T */
//#if GET_ACCELATION || GET_GEOMAGNETIC || GET_GYRO
//		atr.update();
//		// �����x�̏������� */
//		if (get_accelation)
//		{
//			sprintf_s(filename, "output/accelation/accelation_no%d_%dth.csv", NO, step);
//			std::ofstream ofs_acc(filename, std::ios_base::out);
//			if (ofs_acc.fail())
//			{
//				writeError(filename, false);
//				end = 'e';
//				break;
//			}
//			ofs_acc << atr.getAccelation()[0] << "," << atr.getAccelation()[1] << "," << atr.getAccelation()[2] << std::endl;
//		}
//		/* �W���C���̏������� */
//		if (get_gyro){
//			sprintf_s(filename, "output/gyro/gyro_no%d_%dth.csv", NO, step);
//			std::ofstream ofs_gyr(filename, std::ios_base::out);
//			if (ofs_gyr.fail())
//			{
//				writeError(filename, false);
//				break;
//			}
//			ofs_gyr << atr.getGyro()[0] << "," << atr.getGyro()[1] << "," << atr.getGyro()[2] << std::endl;
//		}
//		// �n���C�̏�������
//		if (get_geomagnetic){
//			sprintf_s(filename, "output/geomagnetic/geomagnetic_no%d_%dth.csv", NO, step);
//			std::ofstream ofs_geo(filename, std::ios_base::out);
//			if (ofs_geo.fail())
//			{
//				writeError(filename, false);
//				break;
//			}
//			ofs_geo << atr.getGeomagnetic()[0] << "," << atr.getGeomagnetic()[1] << "," << atr.getGeomagnetic()[2] << std::endl;
//		}
//#endif
//		/* GPS�v�� */
//#if GET_GPS
//		//clock_t lap_gps_start = clock();
//		std::string str;
//		sp_gps.rxLen = 0;
//		while (1)
//		{
//			sp_gps.gets('$', '\n');
//			if (sp_gps.rxFlag == true){ // '\n'���܂܂�Ă�����I��
//				std::string str = sp_gps.rxData; // char�Ȃ̂�string�ɕϊ� ����Ŏ����ϊ��炵��
//				std::cout << str << std::endl;
//				sprintf_s(filename, "./output/gps/gps_no%d_%dth.txt", NO, step);
//				std::ofstream ofs_gps(filename, ios_base::out);
//				if (ofs_gps.fail())
//				{
//					writeError(filename, false);
//					break;
//				}
//				ofs_gps << str << std::endl;
//				break;
//			}
//			//clock_t lap3 = clock();
//			//if ((int)(lap3 - lap_gps_start) > UPDATE_TIME*0.8){
//			//	str = "No Data";
//			//	std::cout << str << std::endl;
//			//	break;	//	�v����������GPS���擾�ł��Ȃ�������break
//			//}
//		}
//		std::cout << "Get GPS!" << std::endl;
//		// GPS�f�[�^���t�@�C���o��
//		//filename[256];
//#endif
//		end = clock();
//		
//		int rest_time = UPDATE_TIME - (int)(end - start);
//		if (rest_time > 0){
//			std::this_thread::sleep_for(std::chrono::microseconds(rest_time));
//		}
//
//		step++;
//
//
//	}
//
//#if GET_ACCELATION || GET_GEOMAGNETIC || GET_GYRO
//	atr.stop();
//#endif
//
//#if GET_GPS
//	sp_gps.end(); // GPS�ւ̐ڑ������
//#endif
//#if GET_OMNI
//	cv::destroyAllWindows();
//#endif
//#if GET_LRF_U
//	urg_close(&urg1); // LRF�ւ̐ڑ������
//#endif
//#if GET_LRF_L
//	urg_close(&urg2); // LRF�ւ̐ڑ������
//#endif
//	winsocke2_close(); // ���{�b�g�ւ̐ڑ������
//
//	return 0;
//}

