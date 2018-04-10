
#pragma once

#include "myfun.h"
#include "parameter.h"
//#include "LocalizationPF.h"

/**********************************************************/
//  �v���v���Z�b�T
/**********************************************************/

/*  �ޓx�����֘A  */


class Lidar2d
{
public:
	/*  �R���X�g���N�^  */
	Lidar2d() {};
	Lidar2d(const Lidar2d& obj)
	{
		(this->point_cloud) = (obj.point_cloud);
		(this->scan) = (obj.scan);

		/*  ���f�[�^  */
		this->grid_map = obj.grid_map;								// ��Q���n�}

																	/*  �v���f�[�^  */
		this->scan_xy = obj.scan_xy;					//	scan�f�[�^���ɍ��W����f�J���g���W�ɕϊ�

		this->scan_grid = obj.scan_grid;

	}
	Lidar2d& operator=(const Lidar2d& obj)
	{
		(this->point_cloud) = (obj.point_cloud);
		(this->scan) = (obj.scan);


		/*  ���f�[�^  */
		cv::Mat mat = obj.grid_map.clone();
		this->grid_map = mat;

		/*  �v���f�[�^  */
		this->scan_xy = obj.scan_xy;					//	scan�f�[�^���ɍ��W����f�J���g���W�ɕϊ�

														/*  �ޓx�����֘A  */
		this->scan_grid = obj.scan_grid;

		return *this;
	}

	/*  �f�X�g���N�^  */
	~Lidar2d()
	{
		scan_grid.release();
	};


	/**********************************************************/
	//	�t�@�C���ǂݍ���
	/**********************************************************/

	/*  Lidar2D�_�Q�n�}�̓ǂݍ���  */
	void readPointCloud(std::string filename)
	{
		std::ifstream ifs(filename);
		if (ifs.fail())	readError(filename);							//	�G���[�o��
		ifs >> point_cloud;
	};
	void setPointCloud(std::vector<Coor<>> pointcloud)
	{
		this->point_cloud = pointcloud;
	};


	void setGMap(cv::Mat gmap)
	{
		grid_map = gmap;
	}

	/*  Lidar2D�X�L�����f�[�^�̓ǂݍ���  */
	bool readMeasScan(int no, int step)
	{
		std::string filename = IFPATH_MEAS + "lrf/lrf_" + std::to_string(step) + "th_no1.csv";
		std::ifstream ifs(filename);
		if (ifs.fail())	return false;	//	�ǂݍ��݂Ɏ��s�����ꍇ�Cfalse��Ԃ�
		ifs >> scan;
		convertScan();
		return true;
	};

	/*  Lidar2D�X�L�����f�[�^�̓ǂݍ���  */
	bool readMeasScan(std::string filename)
	{
		std::ifstream ifs(filename);
		if (ifs.fail())	return false;	//	�ǂݍ��݂Ɏ��s�����ꍇ�Cfalse��Ԃ�
		ifs >> scan;
		convertScan();
		return true;
	};
	bool setMeasScan(std::vector<Polar<>> scan) {
		this->scan = scan;
		convertScan();
		return true;
	}

	void convertScan() {
		//	�ɍ��W����f�J���g���W�ɕϊ�
		for (const auto tmp : scan)
		{
			if (tmp.r == 1 || tmp.theta>M_PI*2.0 / 3.0 || tmp.theta<-M_PI*2.0 / 3.0)	continue;	//	lrf�f�[�^�ُ̈�l��1�Ƃ��ďo�͂���邽�߁C�r���C90�x�ȏ�̓_�Q���r��
			Coor<> coor = polToCoor(tmp);			//	�ɍ��W����f�J���g���W�ɕϊ����ď�����
			scan_xy.push_back(coor);
		}
	}

	//bool readMeasScan(int step)
	//{

	//	std::string filename = IFPATH_MEAS + "lrf/lrf_" + std::to_string(step) + "th_no1.csv";
	//	std::ifstream ifs(filename);
	//	if (ifs.fail())	return false;	//	�ǂݍ��݂Ɏ��s�����ꍇ�Cfalse��Ԃ�
	//	ifs >> scan;
	//	return true;
	//};


	/**********************************************************/
	//	ICP�֘A
	/**********************************************************/

	/*  ICP�ɓ_�Q�n�}���Z�b�g  */
	void setEnvICP()
	{
		icp.setMapData(point_cloud);	//	�_�Q�n�}��set
		icp.setThreshold(ICP_PAIR_DST_TH);	//	�Ή��_�����臒l
	};

	/*  ICP�ɃX�L�����f�[�^���Z�b�g  */
	void setMeasICP()
	{
		icp.setInputDataOrg(scan_xy);	//	�v���f�[�^��set
	}

	/*  �p�[�e�B�N���̖ޓx�Z�o  */
	template<typename T>
	double getLikelihoodICP(const Position<T> pos, float bai)
	{
		icp.transInputDataOrg(pos);	//	�ʒu���ŕ��s�ړ�
		icp();

		if (icp.getPairNum() < scan_xy.size()*bai) return 0.0;		//	�Ή��_��臒l�ȉ��̏ꍇ�C�ޓx��0.0

		double out = std::exp(-(double)icp.getError()*ICP_PARAMETER);
		return out;	//	�ޓx�̎Z�o
	}


	/**********************************************************/
	//	�O���b�h�}�b�v�}�b�`���O
	/**********************************************************/
	void setScanGrid()
	{
		/* �摜�̐��� */
		size_t img_cols = LASER_DIST_RANGE / MAP_RES + 2;	// ���[�U�v���͈͂��班���]�T����������
		size_t img_rows = img_cols;

		cv::Mat img(cv::Size(img_cols * 2, img_rows * 2), CV_8U, cv::Scalar(128));	//	�O���[�摜�𐶐�

																					/* ���[�U�v���\�͈͂����h�� */
		cv::Point center = cv::Point(img_cols, img_rows);
		cv::Size laser_dist_radius(LASER_DIST_RANGE / MAP_RES, LASER_DIST_RANGE / MAP_RES);
		cv::ellipse(img, center, laser_dist_radius, 180, 55, 305, cv::Scalar(0), -1);

		/* ���[�U���ߔ͈͂𔒓h�� */
		for (int i = 0; i < scan.size(); i++)
		{
			Polar<> point = scan.at(i);

			if (point.theta<-M_PI*2.0 / 3.0 || point.theta>M_PI*2.0 / 3.0) {
				continue;
			}

			if (point.r == 1)
			{
				//if (point.theta > -M_PI / 4.0 && point.theta < M_PI / 4.0)
				//{
				//	point.r = 30000;	// point.r=1�̂Ƃ��C�r�[���͔��˂��Ȃ��������Ƃ��Ӗ����邽�߁C�v���\�͈͂�30000[mm]�ɐݒ�
				//}
				//else
				//{
				//	continue;
				//}
				point.r = LASER_DIST_RANGE;	// point.r=1�̂Ƃ��C�r�[���͔��˂��Ȃ��������Ƃ��Ӗ����邽�߁C�v���\�͈͂�30000[mm]�ɐݒ�
			}
			//if (point.r == 1)	point.r = 30000;	// point.r=1�̂Ƃ��C�r�[���͔��˂��Ȃ��������Ƃ��Ӗ����邽�߁C�v���\�͈͂�30000[mm]�ɐݒ�
			Coor<> point_xy = polToCoor(point);
			cv::Point point_pix = ToPixel(point_xy, img, MAP_RES, 0.5, 0.5);
			cv::line(img, center, point_pix, cv::Scalar(255), 2);
		}

		scan_grid = img;		//	�X�L�����f�[�^�����Q���n�}�𐶐�

								//cv::imshow("img", img);
								//cv::waitKey();
	}

	template<typename T>
	double getLikelihoodGrid(const Position<T> pos)
	{
		double angle_deg = pos.r / M_PI*180.0;	//	�X�L�����O���b�h��]�p�x
		cv::Point center(scan_grid.cols, scan_grid.rows);

		/* ��] */
		cv::Mat affine = cv::getRotationMatrix2D(center, angle_deg, 1.0);
		cv::Mat scan_grid_new;
		cv::warpAffine(scan_grid, scan_grid_new, affine, cv::Size(scan_grid.cols, scan_grid.rows), 1, 0, cv::Scalar(128));

		cv::Point pos_pix = ToPixel(pos, grid_map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);

		int pix_num_nonequal = 0;	// �Q�ƃf�[�^�ƌv���f�[�^�Ńs�N�Z������v���Ȃ�������

		for (int sy = 0; sy < scan_grid_new.rows; sy++)
		{
			for (int sx = 0; sx < scan_grid_new.cols; sx++)
			{
				if ((int)scan_grid_new.at<unsigned char>(sx, sy) == 128)
				{
					continue;	// �O���[�̈�̓X�L�b�v
				}

				cv::Point map_pix(-scan_grid_new.cols / 2 + sx + 1, -scan_grid_new.rows / 2 + sy + 1);
				map_pix = pos_pix + map_pix;

				if (scan_grid_new.at<unsigned char>(sx, sy) != grid_map.at<unsigned char>(map_pix))	pix_num_nonequal++;
			}
		}

		double out = std::exp(-(double)pix_num_nonequal*GRID_PARAMETER);

		//double out

		return out;	//	�ޓx�̎Z�o



		return out;	//	�ޓx�̎Z�o
	}



	/**********************************************************/
	//	�v���f�[�^��clear
	/**********************************************************/

	void clearMeasurement()
	{
		scan.clear();
		scan_xy.clear();
		scan_grid.release();
	}


	/**********************************************************/
	//  �����o�ϐ�
	/**********************************************************/

	/*  ���f�[�^  */
	std::vector<Coor<>> point_cloud;		//  ���[�U�X�L���i�F�_�Q�n�}
	cv::Mat grid_map;								// ��Q���n�}

													/*  �v���f�[�^  */
	std::vector<Polar<>> scan;			//  LRF�X�L�����f�[�^
	std::vector<Coor<>> scan_xy;					//	scan�f�[�^���ɍ��W����f�J���g���W�ɕϊ�

													/*  �ޓx�����֘A  */
	ICP icp;										//	ICP�A���S���Y��
	cv::Mat scan_grid;


};