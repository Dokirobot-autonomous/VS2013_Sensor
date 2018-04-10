
#pragma once

#include "myfun.h"
#include "mycv.h"
#include "ParticleFilter.h"
#include "parameter.h"
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/legacy/legacy.hpp>    //BruteForceMatche�ɕK�v�Bopencv2.4�ňړ������H

/**********************************************************/
//	�v���v���Z�b�T
/**********************************************************/



class OmniCamera
{
public:
	OmniCamera() {};
	OmniCamera(const OmniCamera& obj)
	{
		(this->env_centroid) = (obj.env_centroid);
		(this->env_histgram) = (obj.env_histgram);
		(this->env_img_position) = (obj.env_img_position);

		/*  ���f�[�^  */
		this->env_img_cap = obj.env_img_cap;
		this->env_img = obj.env_img;
		this->env_keypoints = obj.env_keypoints;
		this->env_descriptors = obj.env_descriptors;


		/*  �v���f�[�^  */
		this->img = obj.img.clone();					//	�v���摜
		this->keypoints = obj.keypoints;
		this->descriptors = obj.descriptors;		// ROWS:�����_�� COLS:128
		this->dot_product = obj.dot_product.clone();	//	���q�X�g�O�����ƌv���q�X�g�O�����̓���

														/*  �ޓx�����֘A  */
		this->sim_img_idx = obj.sim_img_idx;	//	�v���摜�Ɨގ�������摜�̃C���f�b�N�X

												/* SURF�p */
		this->matches_used_calc = obj.matches_used_calc;	//	[�摜�ԍ�][DMatch�ԍ�]
		this->imgForCalcLikeAll_positioin = obj.imgForCalcLikeAll_positioin;	//	[�|�W�V�����ԍ�]
		this->imgForCalcLikeAll_inImgAll_keypoint = obj.imgForCalcLikeAll_inImgAll_keypoint;	// [�摜�ԍ�][�L�[�|�C���g�ԍ�]
		this->imgForCalcLikeAll_descriptors = obj.imgForCalcLikeAll_descriptors;

	};

	OmniCamera& operator=(const OmniCamera &obj)
	{
		(this->env_centroid) = (obj.env_centroid);
		(this->env_histgram) = (obj.env_histgram);
		(this->env_img_position) = (obj.env_img_position);

		/*  ���f�[�^  */
		this->env_img_cap = obj.env_img_cap;
		this->env_img = obj.env_img;
		this->env_keypoints = obj.env_keypoints;
		this->env_descriptors = obj.env_descriptors;


		/*  �v���f�[�^  */
		this->img = obj.img.clone();					//	�v���摜
		this->keypoints = obj.keypoints;
		this->descriptors = obj.descriptors;		// ROWS:�����_�� COLS:128
		this->dot_product = obj.dot_product.clone();	//	���q�X�g�O�����ƌv���q�X�g�O�����̓���

														/*  �ޓx�����֘A  */
		this->sim_img_idx = obj.sim_img_idx;	//	�v���摜�Ɨގ�������摜�̃C���f�b�N�X

												/* SURF�p */
		this->matches_used_calc = obj.matches_used_calc;	//	[�摜�ԍ�][DMatch�ԍ�]
		this->imgForCalcLikeAll_positioin = obj.imgForCalcLikeAll_positioin;	//	[�|�W�V�����ԍ�]
		this->imgForCalcLikeAll_inImgAll_keypoint = obj.imgForCalcLikeAll_inImgAll_keypoint;	// [�摜�ԍ�][�L�[�|�C���g�ԍ�]
		this->imgForCalcLikeAll_descriptors = obj.imgForCalcLikeAll_descriptors;


		return *this;
	};
	~OmniCamera()
	{
		env_histgram.release();		//	���摜�̃q�X�g�O����
		env_centroid.release();		//	���摜�̃Z���g���C�h
		env_img_cap.release();
		env_img.clear();
		env_img_position.clear();				//	���n�}�摜�̎B�e�ʒu
		env_keypoints.clear();
		env_descriptors.clear();
		env_img.shrink_to_fit();
		env_img_position.shrink_to_fit();				//	���n�}�摜�̎B�e�ʒu
		env_keypoints.shrink_to_fit();
		env_descriptors.shrink_to_fit();


		/*  �v���f�[�^  */
		keypoints.clear();
		keypoints.shrink_to_fit();
		descriptors.release();		// ROWS:�����_�� COLS:128
		dot_product.release();	//	���q�X�g�O�����ƌv���q�X�g�O�����̓���


	};


	/**********************************************************/
	//	�t�@�C���̓ǂݍ���
	/**********************************************************/

public:

	/*  �q�X�g�O�����f�[�^�̓ǂݍ���  */
	void readEnvHistgram()
	{
		std::string filename = IFPATH_ENV + "histgram.csv";
		std::ifstream ifs(filename);
		if (ifs.fail()) readError(filename);
		ifs >> env_histgram;
	}
	void readEnvHistgram(std::string filename)
	{
		std::ifstream ifs(filename);
		if (ifs.fail()) readError(filename);
		ifs >> env_histgram;
	}

	/*  �Z���g���C�h�f�[�^�̓ǂݍ���  */
	void readEnvCentroid()
	{
		std::string filename = IFPATH_ENV + "centroid.csv";
		std::ifstream ifs(filename);
		if (ifs.fail()) readError(filename);
		ifs >> env_centroid;
	}
	void readEnvCentroid(std::string filename)
	{
		std::ifstream ifs(filename);
		if (ifs.fail()) readError(filename);
		ifs >> env_centroid;
	}
	void readImg(std::string filename){
		img = cv::imread(filename);
		if (img.empty()){
			readError(filename);
		}
	}

	/* ���摜keypoint�̓ǂݍ��� */
	void readEnvKeypoint(std::string filepath) {

		int fnum = 1;
		while (true) {
			std::string filename = filepath + "keypoint/keypoint_" + std::to_string(fnum) + "th.yml";
			std::vector<cv::KeyPoint> keypoints_tmp;
			cv::FileStorage fs(filename, cv::FileStorage::READ);
			cv::FileNode fn = fs["keypoints"];
			cv::read(fn, keypoints_tmp);
			if (keypoints_tmp.empty()) {
				break;
			}
			env_keypoints.push_back(keypoints_tmp);
			fnum++;
			//if (fnum > 2){
			//	break;
			//}
		}
		if (env_keypoints.empty()) {
			readError(filepath + "keypoints");
		}
	}

	/* ���摜descriptor�̓ǂݍ��� */
	void readEnvDescriptor(std::string filepath) {

		int fnum = 1;
		while (true) {
			std::string filename = filepath + "descriptor/desc_" + std::to_string(fnum) + "th.csv";
			cv::Mat descriptors_tmp;
			std::ifstream ifs(filename);
			if (ifs.fail()) {
				break;
			}
			
			std::vector<std::vector<float>> v;
			{
				v.reserve(1500);
				std::string str;
				while (std::getline(ifs, str))
				{
					std::vector<float> tmp = Split<float>(str, ",");
					v.push_back(tmp);
				}
			}
			clock_t lap4_7 = clock();
			cv::Mat descriptor_tmp;
			for (int i = 0; i < v.size(); i++){
				cv::Mat mat(v[i], true);
				mat = mat.t();
				descriptor_tmp.push_back(mat);
			}
			env_descriptors.push_back(descriptor_tmp);

			fnum++;

		}
		if (env_descriptors.empty()) {
			readError(filepath + "descriptor");
		}

	}
	void setEnvDescriptor(OMNI_FEATURE_TYPE feature) {

		cv::SiftDescriptorExtractor extractor_sift;
		cv::SurfDescriptorExtractor extractor_surf;

		switch (feature)
		{
		case OMNI_FEATURE_SIFT:
			//SIFT
			for (int i = 0; i < env_img.size(); i++) {
				//�摜�̓����_�ɂ���������ʂ𒊏o
				cv::Mat desc;
				extractor_sift.compute(env_img[i], env_keypoints[i], desc);
				env_descriptors.push_back(desc);
				std::cout << "Env Desc: " << i << std::endl;
			}
			break;
		case OMNI_FEATURE_SURF:
			//SURF
			for (int i = 0; i < env_img.size(); i++) {
				//�摜�̓����_�ɂ���������ʂ𒊏o
				cv::Mat desc;
				extractor_surf.compute(env_img[i], env_keypoints[i], desc);
				env_descriptors.push_back(desc);
				std::cout << "Env Desc: " << i << std::endl;
			}
			break;
		default:
			break;
		}
	}
	void setEnvDescriptorThread(int& th, std::mutex& mtx) {
		//SURF
		cv::SurfDescriptorExtractor extractor;

		while (true) {
			cv::Mat desc;
			int j;

			mtx.lock();
			if (th >= env_img.size()) {
				break;
			}
			j = th;
			th++;
			mtx.unlock();
			extractor.compute(env_img[j], env_keypoints[j], env_descriptors[j]);
			std::cout << "Env Descriptor: " << j << std::endl;
		}

	}
	void setEnvDescriptor(int thread_size) {

		std::vector<cv::Mat>& env_img_tmp = env_img;
		std::vector<std::vector<cv::KeyPoint>>& env_keypoints_tmp = env_keypoints;
		std::vector<cv::Mat>& env_descriptors_tmp = env_descriptors;

		std::vector<std::thread> threads(thread_size);
		std::mutex mtx;
		int i = 0;
		for (int th = 0; th < thread_size; th++) {
			threads[th] = std::thread([&i, &mtx, &env_img_tmp, &env_keypoints_tmp, &env_descriptors_tmp] {
				cv::SurfDescriptorExtractor extractor;
				std::cout << 1 << std::endl;
				int j;
				while (true) {
					cv::Mat desc;
					mtx.lock();
					std::cout << 3 << std::endl;
					if (i >= env_img_tmp.size()) {
						mtx.unlock();
						break;
					}
					j = i;
					i++;
					env_descriptors_tmp.push_back(desc);
					mtx.unlock();
					std::cout << 4 << std::endl;
					extractor.compute(env_img_tmp[j], env_keypoints_tmp[j], env_descriptors_tmp[j]);
					std::cout << "Env Descriptor: " << j << std::endl;
				}
			});
		}
		for (auto& thread : threads) {
			thread.join();
		}
	}
	void setKeypoints(const std::vector<cv::KeyPoint>& keys) {
		this->keypoints = keys;
	}
	void setDescriptors(cv::Mat desc) {
		this->descriptors = desc;
	}

	/*  ���摜�̎B�e�ʒu�̓ǂݍ���  */
	//	�t�@�C����Global���W�n�̏ꍇ�C�ϊ��p�N���X�������Ɏg�p
	void readEnvImgPosition(std::string filename)
	{
		std::ifstream ifs(filename);
		if (ifs.fail()) readError(filename);
		std::string str;
		std::getline(ifs, str);
		ifs >> env_img_position;
	}
	void setEnvImgPosition(std::vector<Position<>> env_img_position)
	{
		this->env_img_position = env_img_position;
	}

	void readEnvImgAvi(std::string filename) {
		env_img_cap.open(filename);
		if (!env_img_cap.isOpened()) {
			readError(filename);
		}
		while (true) {
			cv::Mat mat;
			env_img_cap >> mat;
			if (mat.empty()) {
				break;
			}
			env_img.push_back(mat);
		}
	}
	void readEnvImgSURF(std::string filepath)
	{
		int fnum = 1;
		while (true) {
			std::string filename = filepath + "img/img_" + std::to_string(fnum) + "th.bmp";

			cv::Mat mat = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
			if (mat.empty()) {
				break;
			}
			env_img.push_back(mat);

			//SURF
			cv::SurfFeatureDetector detector(1000);
			cv::SurfDescriptorExtractor extractor;

			//�摜��������_�����o
			std::vector<cv::KeyPoint> key;
			detector.detect(mat, key);
			//�摜�̓����_�ɂ���������ʂ𒊏o
			cv::Mat desc;
			extractor.compute(mat, key, desc);

			env_keypoints.push_back(key);
			env_descriptors.push_back(desc);
			std::cout << "Environment Image: " << fnum << std::endl;
			fnum++;
			if (fnum > 3) {
				break;
			}
		}
		if (env_keypoints.empty() || env_descriptors.empty()) {
			readError("Descriptor or Keypoints: " + filepath);
		}
	}

	void setEnvImgFromAvi() {
		while (true) {
			cv::Mat mat;
			env_img_cap >> mat;
			if (mat.empty()) {
				break;
			}
			env_img.push_back(mat);
		}
	}

	void clearEnvImg() {
		env_img.clear();
		env_img.shrink_to_fit();
		env_img_cap.release();
	}

	/*  �S���ʃJ�����B�e�摜�̓ǂݍ���  */
	bool readMeasImg(int no, int step)
	{
		std::string filename = IFPATH_MEAS[no] + "img/img_" + std::to_string(step) + "th_no1.bmp";
		img = cv::imread(filename, 0);
		if (img.empty()) return false;

		cv::Point center(img.cols / 1.9, img.rows / 2);

		///*  Operator�̓h��Ԃ�  */
		//cv::Size radius(img.rows / 3, img.rows / 3);
		//double start_angle = -105;	//	��n�̒��S�p�x
		//double angle = 35;	//	�p�x
		//cv::ellipse(img, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);

		//�@���Ԋu�̉摜�ɐ؂�o��
		cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
		cv::Mat input = img(roi_rect);

		//	�v���f�[�^����sift�����ʂ𒊏o
		std::vector<cv::KeyPoint> keypoint; // �����ʊi�[�p�z��
		cv::SiftDescriptorExtractor detector; // SIFT
		detector.detect(input, keypoint);
		detector.compute(input, keypoint, descriptors);
		return true;

	}

	bool readMeasImgSURF(std::string filename) {
		img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		if (img.empty()) {
			return false;
		}

		/*  Operator�̓h��Ԃ�  */
		cv::Point center(img.cols / 1.9, img.rows / 2);
		cv::Size radius(img.rows / 3, img.rows / 3);
		double start_angle = -105;	//	��n�̒��S�p�x
		double angle = 35;	//	�p�x
		cv::ellipse(img, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);

		/* �O�g�̓h��Ԃ� */
		cv::circle(img, center, 510, cv::Scalar(0, 0, 0), 150);

		//�@���Ԋu�̉摜�ɐ؂�o��
		cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
		img = img(roi_rect);

		//SURF
		cv::SurfFeatureDetector detector(1000);
		cv::SurfDescriptorExtractor extractor;

		//�摜��������_�����o
		detector.detect(img, keypoints);
		//�摜�̓����_�ɂ���������ʂ𒊏o
		extractor.compute(img, keypoints, descriptors);

		//cv::Mat mat;
		//cv::drawKeypoints(img, keypoints, mat, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		//cv::imshow("Keypoints", mat);
		//cv::waitKey();

		return true;
	}

	//bool readMeasImg(int step)
	//{
	//	std::string filename = IFPATH_MEAS + "img/img_" + std::to_string(step) + "th_no1.bmp";
	//	img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
	//	if (img.empty()) return false;

	//	/*  Operator�̓h��Ԃ�  */
	//	cv::Point center(img.cols / 1.9, img.rows / 2);
	//	cv::Size radius(img.rows / 3, img.rows / 3);
	//	double start_angle = -105;	//	��n�̒��S�p�x
	//	double angle = 35;	//	�p�x
	//	cv::ellipse(img, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);

	//	//�@���Ԋu�̉摜�ɐ؂�o��
	//	cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
	//	cv::Mat input = img(roi_rect);

	//	//	�v���f�[�^����sift�����ʂ𒊏o
	//	std::vector<cv::KeyPoint> keypoint; // �����ʊi�[�p�z��
	//	cv::SiftDescriptorExtractor detector; // SIFT
	//	detector.detect(input, keypoint);
	//	detector.compute(input, keypoint, descriptor);
	//	return true;

	//}

	bool readMeasSIFT(int no, int step)
	{
		std::string filename = IFPATH_MEAS[no] + "descriptor/desc_" + std::to_string(step) + ".csv";
		std::ifstream ifs(filename);
		if (ifs.fail()) return false;
		ifs >> descriptors;
		return true;
	}
	//bool readMeasSIFT(int step)
	//{
	//	std::string filename = IFPATH_MEAS + "descriptor/desc_" + std::to_string(step) + ".csv";
	//	std::ifstream ifs(filename);
	//	if (ifs.fail()) return false;
	//	ifs >> descriptor;
	//	return true;
	//}

	bool readDotProduct(int no, int step)
	{
		std::string filename = IFPATH_MEAS[no] + "dot_product/dot_product_" + std::to_string(step) + ".csv";
		std::ifstream ifs(filename);
		if (ifs.fail()) return false;
		ifs >> dot_product;
		return true;
	};
	bool readDotProduct(std::string filename)
	{
		std::ifstream ifs(filename);
		if (ifs.fail()) return false;
		ifs >> dot_product;
		return true;
	};

	//bool readDotProduct(int step)
	//{
	//	std::string filename = IFPATH_MEAS + "dot_product/dot_product_" + std::to_string(step) + ".csv";
	//	std::ifstream ifs(filename);
	//	if (ifs.fail()) return false;
	//	ifs >> dot_product;
	//	return true;
	//};

	/**********************************************************/
	//	BoF(SIFT)
	/**********************************************************/

	/*  BoF(SIFT)�̏���  */
	//void setEnvBofSift()
	//{

	//}

	/*  BoF(SIFT)�Ɍv���摜���Z�b�g  */
	template<typename T>
	void setMeasBofSift(Position<T> estimated_position)
	{
		/*  �v���f�[�^�̃}�b�`���O�H */
		// �q�X�g�O�����̌v�Z
		cv::Mat_<float> mes_histgram = cv::Mat_<float>::zeros(1, env_centroid.rows);
		cv::Mat masks;
		// Type���w�� BruteForce�i-L1, -SL2, -Hamming, -Hamming(2)�j, FlannBased
		cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type); // 2��a
		std::vector<cv::DMatch> dmatch;
		matcher->match(descriptors, env_centroid, dmatch, masks);
		for (int j = 0; j < dmatch.size(); j++) mes_histgram(0, dmatch[j].trainIdx)++;


		normalizeCvmat_Row(mes_histgram);


		// �q�X�g�O��������ގ��摜����
		//	���摜�̓����x�N�g���ƎB�e�摜�̓����x�N�g���Ƃ̓��ς��Ƃ�C�������x�N�g���͗ގ�
		dot_product = mes_histgram * env_histgram.t(); // �������͓���

		std::cout << dot_product << std::endl;

		//	臒l����
		for (int i = 0; i < dot_product.cols; i++)
		{
			double distance;
			distance = env_img_position.at(i) | estimated_position;	//	���݂̐���ʒu������摜�̋���
		
			if (dot_product(0, i) > BOF_TH_SIM && distance < ENV_IMG_TH)	//	�O�X�e�b�v�̐���ʒu��img_pos�ňړ����a���̉摜
			{
				sim_img_idx.push_back(i);
			}
		}

		/*  �ޓx�Z�o�̏����iCMR�j�F�����܂�  */
	}

	template<typename T>
	void setMeasBofSift1(Position<T> est_pos)
	{
		//	臒l����
		for (int i = 0; i < dot_product.cols; i++)
		{
			double distance;
			distance = env_img_position.at(i) | est_pos;	//	���݂̐���ʒu������摜�̋���
															//if (dot_product(0, i) > BOF_TH_SIM && distance < ENV_IMG_TH)	//	�O�X�e�b�v�̐���ʒu��img_pos�ňړ����a���̉摜
			if (dot_product(0, i) > BOF_TH_SIM && distance < ENV_IMG_TH)	//	�O�X�e�b�v�̐���ʒu��img_pos�ňړ����a���̉摜
			{
				sim_img_idx.push_back(i);
			}
		}
	}

	/* */
	template<typename T>
	void calcSimilarityBetweenImages(const std::vector<Particle<Position<T>>*>& particles) {

		imgForCalcAll_img.clear();
		imgForCalcAll_dmatch_size.clear();
		imgForCalcLikeAll_positioin.clear();
		imgForCalcLikeAll_descriptors.clear();
		imgForCalcLikeAll_inImgAll_keypoint.clear();
		matches_used_calc.clear();

		/* �p�[�e�B�N���̒[��T�� */
		int max_x, min_x, max_y, min_y;
		for (int i = 0; i < particles.size(); i++) {
			if (i == 0) {
				max_x = min_x = particles[i]->getState()->x;
				max_y = min_y = particles[i]->getState()->y;
				continue;
			}

			if (max_x < particles[i]->getState()->x) {
				max_x = particles[i]->getState()->x;
			}
			else if (min_x > particles[i]->getState()->x) {
				min_x = particles[i]->getState()->x;
			}

			if (max_y < particles[i]->getState()->y) {
				max_y = particles[i]->getState()->y;
			}
			else if (min_y > particles[i]->getState()->y) {
				min_y = particles[i]->getState()->y;
			}
		}

		/* �ގ����]���Ɏg���摜�̒��o */
		// �p�[�e�B�N���I�[���甼�aENV_ING_TH[mm]�͈͓̔��ɂ���摜���g�p
		int min_i = std::min({ env_img_position.size(), env_keypoints.size(), env_descriptors.size() });
		for (int i = 0; i < min_i; i++) {
			if (env_img_position.at(i).x < min_x - ENV_IMG_TH ||
				env_img_position.at(i).x > max_x + ENV_IMG_TH ||
				env_img_position.at(i).y < min_y - ENV_IMG_TH ||
				env_img_position.at(i).y > max_y + ENV_IMG_TH) {
				continue;
			}
			//imgForCalcAll_img.push_back(env_img[i].clone());
			imgForCalcLikeAll_positioin.push_back(env_img_position.at(i));
			imgForCalcLikeAll_inImgAll_keypoint.push_back(env_keypoints[i]);
			imgForCalcLikeAll_descriptors.push_back(env_descriptors[i]);
		}

		///* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
		//for (int i = 0; i < env_img_position.size();i++){			
		//	//�����_�̑Ή��t��
		//	std::vector<cv::DMatch> matches;
		//	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
		//	matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);
		//	/* 臒l���� */
		//	std::vector<cv::DMatch> matches_good;
		//	for (int j = 0; j < matches.size(); j++){
		//		if (matches[j].distance < SURF_DMATCH_DIST_TH){
		//			matches_good.push_back(matches[j]);
		//		}
		//	}
		//	matches_used_calc.push_back(matches_good);
		//}
		/* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
		for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
			//�����_�̑Ή��t��
			std::vector<cv::DMatch> matches;
			cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
			matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);

			///* �\�[�g����SURF_SIZE_USE_FUTURES�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g */
			//nth_element(matches.begin(), matches.begin() + SURF_SIZE_USE_FUTURES - 1, matches.end());
			//std::sort(matches.begin(), matches.end());
			//matches.erase(matches.begin() + SURF_SIZE_USE_FUTURES, matches.end());
			//matches_used_calc.push_back(matches);

			/* 臒l���� */
			std::vector<cv::DMatch> matches_good;
			for (int k = 0; k < matches.size(); k++) {
				if (matches[k].distance < SURF_DMATCH_DIST_TH) {
					matches_good.push_back(matches[k]);
				}
			}
			matches_used_calc.push_back(matches_good);
			imgForCalcAll_dmatch_size.push_back(matches_good.size());

		}

		assert(imgForCalcLikeAll_positioin.size() == imgForCalcLikeAll_descriptors.size());
		assert(imgForCalcLikeAll_positioin.size() == imgForCalcLikeAll_inImgAll_keypoint.size());
		assert(imgForCalcLikeAll_positioin.size() == imgForCalcAll_dmatch_size.size());
		assert(imgForCalcLikeAll_positioin.size() == matches_used_calc.size());


	}

	/* */
	template<typename T>
	void calcSimilarityBetweenImages(const std::vector<Position<T>>& particles,OMNI_FEATURE_TYPE feature) {

		imgForCalcAll_img.clear();
		imgForCalcAll_dmatch_size.clear();
		imgForCalcLikeAll_positioin.clear();
		imgForCalcLikeAll_descriptors.clear();
		imgForCalcLikeAll_inImgAll_keypoint.clear();
		matches_used_calc.clear();

		/* �p�[�e�B�N���̒[��T�� */
		int max_x, min_x, max_y, min_y;
		for (int i = 0; i < particles.size(); i++) {
			if (i == 0) {
				max_x = min_x = particles[i].x;
				max_y = min_y = particles[i].y;
				continue;
			}

			if (max_x < particles[i].x) {
				max_x = particles[i].x;
			}
			else if (min_x > particles[i].x) {
				min_x = particles[i].x;
			}

			if (max_y < particles[i].y) {
				max_y = particles[i].y;
			}
			else if (min_y > particles[i].y) {
				min_y = particles[i].y;
			}
		}
		/* �ގ����]���Ɏg���摜�̒��o */
		// �p�[�e�B�N���I�[���甼�aENV_ING_TH[mm]�͈͓̔��ɂ���摜���g�p
		int min_i = std::min({ env_img_position.size(), env_keypoints.size(), env_descriptors.size() });
		for (int i = 0; i < min_i; i++) {
			if (env_img_position.at(i).x < min_x - ENV_IMG_TH ||
				env_img_position.at(i).x > max_x + ENV_IMG_TH ||
				env_img_position.at(i).y < min_y - ENV_IMG_TH ||
				env_img_position.at(i).y > max_y + ENV_IMG_TH) {
				continue;
			}
			//imgForCalcAll_img.push_back(env_img[i].clone());
			imgForCalcLikeAll_positioin.push_back(env_img_position.at(i));
			imgForCalcLikeAll_inImgAll_keypoint.push_back(env_keypoints[i]);
			imgForCalcLikeAll_descriptors.push_back(env_descriptors[i].clone());
		}

		switch (feature)
		{
		case OMNI_FEATURE_SIFT:
			/* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				//�����_�̑Ή��t��
				std::vector<cv::DMatch> matches;
				cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
				matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);

				/* �\�[�g����SIFT_SIZE_USE_FUTURES�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g */
				nth_element(matches.begin(), matches.begin() + SIFT_SIZE_USE_FUTURES - 1, matches.end());
				std::sort(matches.begin(), matches.end());
				matches.erase(matches.begin() + SIFT_SIZE_USE_FUTURES, matches.end());
				matches_used_calc.push_back(matches);
				imgForCalcAll_dmatch_size.push_back(matches.size());

				///* 臒l���� */
				//std::vector<cv::DMatch> matches_good;
				//for (int k = 0; k < matches.size(); k++) {
				//	if (matches[k].distance < SIFT_DMATCH_DIST_TH) {
				//		matches_good.push_back(matches[k]);
				//	}
				//}
				//matches_used_calc.push_back(matches_good);
				//imgForCalcAll_dmatch_size.push_back(matches_good.size());

			}
			break;
		case OMNI_FEATURE_SURF:
			/* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				//�����_�̑Ή��t��
				std::vector<cv::DMatch> matches;
				cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
				matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);

				///* �\�[�g����SURF_SIZE_USE_FUTURES�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g */
				//nth_element(matches.begin(), matches.begin() + SURF_SIZE_USE_FUTURES - 1, matches.end());
				//std::sort(matches.begin(), matches.end());
				//matches.erase(matches.begin() + SURF_SIZE_USE_FUTURES, matches.end());
				//matches_used_calc.push_back(matches);
				//imgForCalcAll_dmatch_size.push_back(matches.size());

				/* 臒l���� */
				std::vector<cv::DMatch> matches_good;
				for (int k = 0; k < matches.size(); k++) {
					if (matches[k].distance < SURF_DMATCH_DIST_TH) {
						matches_good.push_back(matches[k]);
					}
				}
				matches_used_calc.push_back(matches_good);
				imgForCalcAll_dmatch_size.push_back(matches_good.size());
			}
			break;
		default:
			break;
		}

		///* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
		//for (int i = 0; i < env_img_position.size();i++){			
		//	//�����_�̑Ή��t��
		//	std::vector<cv::DMatch> matches;
		//	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
		//	matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);
		//	/* 臒l���� */
		//	std::vector<cv::DMatch> matches_good;
		//	for (int j = 0; j < matches.size(); j++){
		//		if (matches[j].distance < SURF_DMATCH_DIST_TH){
		//			matches_good.push_back(matches[j]);
		//		}
		//	}
		//	matches_used_calc.push_back(matches_good);
		//}


		assert(imgForCalcLikeAll_positioin.size() == imgForCalcLikeAll_descriptors.size());
		assert(imgForCalcLikeAll_positioin.size() == imgForCalcLikeAll_inImgAll_keypoint.size());
		assert(imgForCalcLikeAll_positioin.size() == imgForCalcAll_dmatch_size.size());
		assert(imgForCalcLikeAll_positioin.size() == matches_used_calc.size());


	}

	template<typename T>
	void calcSimilarityBetweenImages(const Position<T>& esti_positioin, OMNI_FEATURE_TYPE feature) {

		imgForCalcAll_img.clear();
		imgForCalcAll_dmatch_size.clear();
		imgForCalcLikeAll_positioin.clear();
		imgForCalcLikeAll_descriptors.clear();
		imgForCalcLikeAll_inImgAll_keypoint.clear();
		matches_used_calc.clear();

		/* �ގ����]���Ɏg���摜�̒��o */
		// �p�[�e�B�N���I�[���甼�aENV_ING_TH[mm]�͈͓̔��ɂ���摜���g�p
		int min_i = std::min({ env_img_position.size(), env_keypoints.size(), env_descriptors.size() });
		for (int i = 0; i < min_i; i++) {
			if (env_img_position.at(i).x < esti_positioin.x - ENV_IMG_TH ||
				env_img_position.at(i).x > esti_positioin.x + ENV_IMG_TH ||
				env_img_position.at(i).y < esti_positioin.y - ENV_IMG_TH ||
				env_img_position.at(i).y > esti_positioin.y + ENV_IMG_TH) {
				continue;
			}
			//imgForCalcAll_img.push_back(env_img[i].clone());
			imgForCalcLikeAll_positioin.push_back(env_img_position.at(i));
			imgForCalcLikeAll_inImgAll_keypoint.push_back(env_keypoints[i]);
			imgForCalcLikeAll_descriptors.push_back(env_descriptors[i].clone());
		}

		switch (feature)
		{
		case OMNI_FEATURE_SIFT:
			/* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				//�����_�̑Ή��t��
				std::vector<cv::DMatch> matches;
				cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
				matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);

				/* �\�[�g����SIFT_SIZE_USE_FUTURES�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g */
				nth_element(matches.begin(), matches.begin() + SIFT_SIZE_USE_FUTURES - 1, matches.end());
				std::sort(matches.begin(), matches.end());
				matches.erase(matches.begin() + SIFT_SIZE_USE_FUTURES, matches.end());
				matches_used_calc.push_back(matches);
				imgForCalcAll_dmatch_size.push_back(matches.size());

				///* 臒l���� */
				//std::vector<cv::DMatch> matches_good;
				//for (int k = 0; k < matches.size(); k++) {
				//	if (matches[k].distance < SIFT_DMATCH_DIST_TH) {
				//		matches_good.push_back(matches[k]);
				//	}
				//}
				//matches_used_calc.push_back(matches_good);
				//imgForCalcAll_dmatch_size.push_back(matches_good.size());

			}
			break;
		case OMNI_FEATURE_SURF:
			/* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				//�����_�̑Ή��t��
				std::vector<cv::DMatch> matches;
				cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
				matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);

				///* �\�[�g����SURF_SIZE_USE_FUTURES�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g */
				//nth_element(matches.begin(), matches.begin() + SURF_SIZE_USE_FUTURES - 1, matches.end());
				//std::sort(matches.begin(), matches.end());
				//matches.erase(matches.begin() + SURF_SIZE_USE_FUTURES, matches.end());
				//matches_used_calc.push_back(matches);
				//imgForCalcAll_dmatch_size.push_back(matches.size());

				/* 臒l���� */
				std::vector<cv::DMatch> matches_good;
				for (int k = 0; k < matches.size(); k++) {
					if (matches[k].distance < SURF_DMATCH_DIST_TH) {
						matches_good.push_back(matches[k]);
					}
				}
				matches_used_calc.push_back(matches_good);
				imgForCalcAll_dmatch_size.push_back(matches_good.size());
			}
			break;
		default:
			break;
		}

		///* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
		//for (int i = 0; i < env_img_position.size();i++){			
		//	//�����_�̑Ή��t��
		//	std::vector<cv::DMatch> matches;
		//	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
		//	matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);
		//	/* 臒l���� */
		//	std::vector<cv::DMatch> matches_good;
		//	for (int j = 0; j < matches.size(); j++){
		//		if (matches[j].distance < SURF_DMATCH_DIST_TH){
		//			matches_good.push_back(matches[j]);
		//		}
		//	}
		//	matches_used_calc.push_back(matches_good);
		//}


		assert(imgForCalcLikeAll_positioin.size() == imgForCalcLikeAll_descriptors.size());
		assert(imgForCalcLikeAll_positioin.size() == imgForCalcLikeAll_inImgAll_keypoint.size());
		assert(imgForCalcLikeAll_positioin.size() == imgForCalcAll_dmatch_size.size());
		assert(imgForCalcLikeAll_positioin.size() == matches_used_calc.size());


	}

	template<typename T>
	void calcSimilarityBetweenAllImages(const Position<T>& esti_positioin, OMNI_FEATURE_TYPE feature) {

		imgForCalcAll_img.clear();
		imgForCalcAll_dmatch_size.clear();
		imgForCalcLikeAll_positioin.clear();
		imgForCalcLikeAll_descriptors.clear();
		imgForCalcLikeAll_inImgAll_keypoint.clear();
		matches_used_calc.clear();

		/* �ގ����]���Ɏg���摜�̒��o */
		// �p�[�e�B�N���I�[���甼�aENV_ING_TH[mm]�͈͓̔��ɂ���摜���g�p
		int min_i = std::min({ env_img_position.size(), env_keypoints.size(), env_descriptors.size() });
		for (int i = 0; i < min_i; i++) {
			//if (env_img_position.at(i).x < esti_positioin.x - ENV_IMG_TH ||
			//	env_img_position.at(i).x > esti_positioin.x + ENV_IMG_TH ||
			//	env_img_position.at(i).y < esti_positioin.y - ENV_IMG_TH ||
			//	env_img_position.at(i).y > esti_positioin.y + ENV_IMG_TH) {
			//	continue;
			//}
			imgForCalcAll_img.push_back(env_img[i].clone());
			imgForCalcLikeAll_positioin.push_back(env_img_position.at(i));
			imgForCalcLikeAll_inImgAll_keypoint.push_back(env_keypoints[i]);
			imgForCalcLikeAll_descriptors.push_back(env_descriptors[i].clone());
		}

		switch (feature)
		{
		case OMNI_FEATURE_SIFT:
			/* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				//�����_�̑Ή��t��
				std::vector<cv::DMatch> matches;
				cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
				matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);

				/* �\�[�g����SIFT_SIZE_USE_FUTURES�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g */
				nth_element(matches.begin(), matches.begin() + SIFT_SIZE_USE_FUTURES - 1, matches.end());
				std::sort(matches.begin(), matches.end());
				matches.erase(matches.begin() + SIFT_SIZE_USE_FUTURES, matches.end());
				matches_used_calc.push_back(matches);
				imgForCalcAll_dmatch_size.push_back(matches.size());

				///* 臒l���� */
				//std::vector<cv::DMatch> matches_good;
				//for (int k = 0; k < matches.size(); k++) {
				//	if (matches[k].distance < SIFT_DMATCH_DIST_TH) {
				//		matches_good.push_back(matches[k]);
				//	}
				//}
				//matches_used_calc.push_back(matches_good);
				//imgForCalcAll_dmatch_size.push_back(matches_good.size());

			}
			break;
		case OMNI_FEATURE_SURF:
			/* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				//�����_�̑Ή��t��
				std::vector<cv::DMatch> matches;
				cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
				matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);

				///* �\�[�g����SURF_SIZE_USE_FUTURES�Ԗڂ܂ł̑Ή�����\��������Bnth_element�͗v�f����v�f������O�Ɉړ������邠���̃\�[�g */
				//nth_element(matches.begin(), matches.begin() + SURF_SIZE_USE_FUTURES - 1, matches.end());
				//std::sort(matches.begin(), matches.end());
				//matches.erase(matches.begin() + SURF_SIZE_USE_FUTURES, matches.end());
				//matches_used_calc.push_back(matches);
				//imgForCalcAll_dmatch_size.push_back(matches.size());

				/* 臒l���� */
				std::vector<cv::DMatch> matches_good;
				for (int k = 0; k < matches.size(); k++) {
					if (matches[k].distance < SURF_DMATCH_DIST_TH) {
						matches_good.push_back(matches[k]);
					}
				}
				matches_used_calc.push_back(matches_good);
				imgForCalcAll_dmatch_size.push_back(matches_good.size());
			}
			break;
		default:
			break;
		}

		///* matches_used_calc[i][j]: �g�p������摜i�ԖڂƊϑ��摜�Ԃ�j�Ԗڂ̑Ή��_ */
		//for (int i = 0; i < env_img_position.size();i++){			
		//	//�����_�̑Ή��t��
		//	std::vector<cv::DMatch> matches;
		//	cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(matching_type);;
		//	matcher->match(imgForCalcLikeAll_descriptors[i], descriptors, matches);
		//	/* 臒l���� */
		//	std::vector<cv::DMatch> matches_good;
		//	for (int j = 0; j < matches.size(); j++){
		//		if (matches[j].distance < SURF_DMATCH_DIST_TH){
		//			matches_good.push_back(matches[j]);
		//		}
		//	}
		//	matches_used_calc.push_back(matches_good);
		//}


		assert(imgForCalcLikeAll_positioin.size() == imgForCalcLikeAll_descriptors.size());
		assert(imgForCalcLikeAll_positioin.size() == imgForCalcLikeAll_inImgAll_keypoint.size());
		assert(imgForCalcLikeAll_positioin.size() == imgForCalcAll_dmatch_size.size());
		assert(imgForCalcLikeAll_positioin.size() == matches_used_calc.size());


	}

	/*  �p�[�e�B�N���̖ޓx�Z�o  */
	template<typename T>
	double getLikelihoodBofSift(const Position<T> particle)
	{
		assert(!img.empty());
		cv::Point center(img.cols / 2.0, img.rows / 2.0);

		double out = 1.0;
		for (int idx : sim_img_idx)
		{
			double xy_A = 1.0;
			double dis = env_img_position.at(idx) | particle;
			double sgm = BOF_SGM(dot_product(0, idx));
			xy_A = std::exp(-std::pow(dis, 2) / (2.0 * std::pow(sgm, 2)));	//	���K���z

			//std::cout << sgm << std::endl;
			double tmp = xy_A;
			out *= tmp;	//	���K���z
		}

		//double A = 1.0;
		//if (BOF_ROTATION){
		//	A = 0.0;
		//	for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
		//		//double dis = particle | imgForCalcLikeAll_positioin[i];
		//		//if (dis < ENV_IMG_TH) {

		//			for (const auto& match : matches_used_calc[i]) {
		//				/* p1�Ɖ�] */
		//				cv::Point env_p = imgForCalcLikeAll_inImgAll_keypoint[i][match.queryIdx].pt;
		//				cv::Point mea_p = keypoints[match.trainIdx].pt;

		//				Coor<> env_c(env_p.x - center.x, env_p.y - center.y);
		//				Coor<> mea_c(mea_p.x - center.x, mea_p.y - center.y);

		//				double rad_env_c = std::atan2(env_c.y, env_c.x);
		//				double rad_mea_c = std::atan2(mea_c.y, mea_c.x);

		//				double d_key_rad = rad_env_c - rad_mea_c;

		//				double d_pos_rad = imgForCalcLikeAll_positioin[i].r - particle.r;

		//				A += std::abs(SURF_AMP_FUNCTION(d_pos_rad - d_key_rad));

		//			//}
		//		}
		//	}
		//}

		//out /= A;
		//std::cout << out << std::endl;
		assert(out > 0);
		return out;
	}

	template<typename T>
	double getLikelihoodRotate(const Position<T> particle, OMNI_FEATURE_TYPE feature)
	{
		assert(!img.empty());
		switch (feature)
		{
		case OMNI_FEATURE_SIFT:{

			double out = 1.0;
			cv::Point center(img.cols / 2.0, img.rows / 2.0);

			/* particle�ߖ[�̉摜��T�� */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				double dis = particle | imgForCalcLikeAll_positioin[i];
				if (dis < ENV_IMG_TH) {

					//double sim_xy = SURF_SIM_XY(matches_used_calc[i].size());
					//double xy_variance = SURF_XY_VARIANCE(sim_xy);
					//out *= std::exp(-dis*dis / (2.0*xy_variance*xy_variance));
					//std::cout << sim_xy << "," << xy_variance << "," << std::exp(-dis*dis / (2.0*xy_variance*xy_variance)) << std::endl;;
					//std::cout << matches_used_calc[i].size()<< std::endl;;

					//double xy_A = 0.0;
					//for (const auto& matche : matches_used_calc[i]){
					//	xy_A += SIFT_AMP_XY_SUNCTION(matche.distance);
					//}

					//if (size == 0){
					//	continue;
					//}

					//double xy = xy_A*std::exp(-dis*dis / (2.0*SURF_SD_GMM*SURF_SD_GMM));
					//double xy = std::exp(-dis*dis / (2.0*xy_A*xy_A));

					double A = 0.0;
					for (const auto& match : matches_used_calc[i]) {
						/* p1�Ɖ�] */
						cv::Point env_p = imgForCalcLikeAll_inImgAll_keypoint[i][match.queryIdx].pt;
						cv::Point mea_p = keypoints[match.trainIdx].pt;

						Coor<> env_c(env_p.x - center.x, env_p.y - center.y);
						Coor<> mea_c(mea_p.x - center.x, mea_p.y - center.y);

						double rad_env_c = std::atan2(env_c.y, env_c.x);
						double rad_mea_c = std::atan2(mea_c.y, mea_c.x);

						double d_key_rad = rad_env_c - rad_mea_c;

						double d_pos_rad = imgForCalcLikeAll_positioin[i].r - particle.r;

						A += std::abs(SIFT_AMP_FUNCTION(d_pos_rad - d_key_rad));

						//if (size > 100){
						//	std::cout << imgForCalcLikeAll_positioin[i].r << "," << particle.r << std::endl;
						//	std::cout << d_key_rad <<","<< d_pos_rad << std::endl;
						//}



						//cv::Point p1_tmp = imgForCalcLikeAll_inImgAll_keypoint[i][match.queryIdx].pt;
						//cv::Point p1;
						//p1.x = std::cos(particle.r)*(p1_tmp.x - center.x) + std::sin(particle.r)*(p1_tmp.y - center.y) + center.x;
						//p1.y = -std::sin(particle.r)*(p1_tmp.x - center.x) + std::cos(particle.r)*(p1_tmp.y - center.y) + center.y;
						//cv::Point p2 = keypoints[match.trainIdx].pt;
						//double dx = p1.x - p2.x;
						//double dy = p1.y - p2.y;
						//double d_key = std::sqrt(dx*dx + dy*dy);
						//// �ޓx���Z
						//A += SURF_AMP_FUNCTION(d_key);
					}
					double tmp = A;
					out += tmp;
					//std::cout << size << " " << A << std::endl;
				}
			}
			//std::cout << out << std::endl;
			assert(out > 0);
			return out;
		}
		case OMNI_FEATURE_SURF:{

			double out = 1.0;
			cv::Point center(img.cols / 2.0, img.rows / 2.0);

			/* particle�ߖ[�̉摜��T�� */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				double dis = particle | imgForCalcLikeAll_positioin[i];
				if (dis < ENV_IMG_TH) {

					//double sim_xy = SURF_SIM_XY(matches_used_calc[i].size());
					//double xy_variance = SURF_XY_VARIANCE(sim_xy);
					//out *= std::exp(-dis*dis / (2.0*xy_variance*xy_variance));
					//std::cout << sim_xy << "," << xy_variance << "," << std::exp(-dis*dis / (2.0*xy_variance*xy_variance)) << std::endl;;
					//std::cout << matches_used_calc[i].size()<< std::endl;;
					double size = matches_used_calc[i].size();
					//if (size == 0){
					//	continue;
					//}
					double xy = size*std::exp(-dis*dis / (2.0*SURF_SD_GMM*SURF_SD_GMM));
					//double xy = std::exp(-dis*dis / (2.0*size*size));

					double A = 0.0;
					for (const auto& match : matches_used_calc[i]) {
						/* p1�Ɖ�] */
						cv::Point env_p = imgForCalcLikeAll_inImgAll_keypoint[i][match.queryIdx].pt;
						cv::Point mea_p = keypoints[match.trainIdx].pt;

						Coor<> env_c(env_p.x - center.x, env_p.y - center.y);
						Coor<> mea_c(mea_p.x - center.x, mea_p.y - center.y);

						double rad_env_c = std::atan2(env_c.y, env_c.x);
						double rad_mea_c = std::atan2(mea_c.y, mea_c.x);

						double d_key_rad = rad_env_c - rad_mea_c;

						double d_pos_rad = imgForCalcLikeAll_positioin[i].r - particle.r;

						A += std::abs(SURF_AMP_FUNCTION(d_pos_rad - d_key_rad));

						//if (size > 100){
						//	std::cout << imgForCalcLikeAll_positioin[i].r << "," << particle.r << std::endl;
						//	std::cout << d_key_rad <<","<< d_pos_rad << std::endl;
						//}



						//cv::Point p1_tmp = imgForCalcLikeAll_inImgAll_keypoint[i][match.queryIdx].pt;
						//cv::Point p1;
						//p1.x = std::cos(particle.r)*(p1_tmp.x - center.x) + std::sin(particle.r)*(p1_tmp.y - center.y) + center.x;
						//p1.y = -std::sin(particle.r)*(p1_tmp.x - center.x) + std::cos(particle.r)*(p1_tmp.y - center.y) + center.y;
						//cv::Point p2 = keypoints[match.trainIdx].pt;
						//double dx = p1.x - p2.x;
						//double dy = p1.y - p2.y;
						//double d_key = std::sqrt(dx*dx + dy*dy);
						//// �ޓx���Z
						//A += SURF_AMP_FUNCTION(d_key);
					}
					double tmp = A*xy;
					out += tmp;
					//std::cout << size << " " << A << std::endl;
				}
			}
			//std::cout << out << std::endl;
			assert(out > 0);
			return out;
		}
		default:
			assert(feature == OMNI_FEATURE_SIFT || feature == OMNI_FEATURE_SURF);
			break;
		}


	}


	template<typename T>
	double getLikelihood(const Position<T> particle, OMNI_FEATURE_TYPE feature)
	{
		assert(!img.empty());
		switch (feature)
		{
		case OMNI_FEATURE_SIFT:{

			double out = 1.0;
			cv::Point center(img.cols / 2.0, img.rows / 2.0);

			/* particle�ߖ[�̉摜��T�� */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				double dis = particle | imgForCalcLikeAll_positioin[i];
				if (dis < ENV_IMG_TH) {

					//double sim_xy = SURF_SIM_XY(matches_used_calc[i].size());
					//double xy_variance = SURF_XY_VARIANCE(sim_xy);
					//out *= std::exp(-dis*dis / (2.0*xy_variance*xy_variance));
					//std::cout << sim_xy << "," << xy_variance << "," << std::exp(-dis*dis / (2.0*xy_variance*xy_variance)) << std::endl;;
					//std::cout << matches_used_calc[i].size()<< std::endl;;
					double xy_A = 0.0;
					for (const auto& matche : matches_used_calc[i]){
						xy_A += SIFT_AMP_XY_SUNCTION(matche.distance);
					}

					//if (size == 0){
					//	continue;
					//}

					//double xy = xy_A*std::exp(-dis*dis / (2.0*SURF_SD_GMM*SURF_SD_GMM));
					double xy = std::exp(-dis*dis / (2.0*xy_A*xy_A));

					double A = 0.0;
					for (const auto& match : matches_used_calc[i]) {
						/* p1�Ɖ�] */
						cv::Point env_p = imgForCalcLikeAll_inImgAll_keypoint[i][match.queryIdx].pt;
						cv::Point mea_p = keypoints[match.trainIdx].pt;

						Coor<> env_c(env_p.x - center.x, env_p.y - center.y);
						Coor<> mea_c(mea_p.x - center.x, mea_p.y - center.y);

						double rad_env_c = std::atan2(env_c.y, env_c.x);
						double rad_mea_c = std::atan2(mea_c.y, mea_c.x);

						double d_key_rad = rad_env_c - rad_mea_c;

						double d_pos_rad = imgForCalcLikeAll_positioin[i].r - particle.r;

						A += std::abs(SURF_AMP_FUNCTION(d_pos_rad - d_key_rad));

						//if (size > 100){
						//	std::cout << imgForCalcLikeAll_positioin[i].r << "," << particle.r << std::endl;
						//	std::cout << d_key_rad <<","<< d_pos_rad << std::endl;
						//}



						//cv::Point p1_tmp = imgForCalcLikeAll_inImgAll_keypoint[i][match.queryIdx].pt;
						//cv::Point p1;
						//p1.x = std::cos(particle.r)*(p1_tmp.x - center.x) + std::sin(particle.r)*(p1_tmp.y - center.y) + center.x;
						//p1.y = -std::sin(particle.r)*(p1_tmp.x - center.x) + std::cos(particle.r)*(p1_tmp.y - center.y) + center.y;
						//cv::Point p2 = keypoints[match.trainIdx].pt;
						//double dx = p1.x - p2.x;
						//double dy = p1.y - p2.y;
						//double d_key = std::sqrt(dx*dx + dy*dy);
						//// �ޓx���Z
						//A += SURF_AMP_FUNCTION(d_key);
					}
					double tmp = A*xy;
					out += tmp;
					//std::cout << size << " " << A << std::endl;
				}
			}
			//std::cout << out << std::endl;
			assert(out > 0);
			return out;
		}
		case OMNI_FEATURE_SURF:{

			double out = 1.0;
			cv::Point center(img.cols / 2.0, img.rows / 2.0);

			/* particle�ߖ[�̉摜��T�� */
			for (int i = 0; i < imgForCalcLikeAll_positioin.size(); i++) {
				double dis = particle | imgForCalcLikeAll_positioin[i];
				if (dis < ENV_IMG_TH) {

					//double sim_xy = SURF_SIM_XY(matches_used_calc[i].size());
					//double xy_variance = SURF_XY_VARIANCE(sim_xy);
					//out *= std::exp(-dis*dis / (2.0*xy_variance*xy_variance));
					//std::cout << sim_xy << "," << xy_variance << "," << std::exp(-dis*dis / (2.0*xy_variance*xy_variance)) << std::endl;;
					//std::cout << matches_used_calc[i].size()<< std::endl;;
					double size = matches_used_calc[i].size();
					//if (size == 0){
					//	continue;
					//}
					double xy = size*std::exp(-dis*dis / (2.0*SURF_SD_GMM*SURF_SD_GMM));
					//double xy = std::exp(-dis*dis / (2.0*size*size));

					double A = 0.0;
					for (const auto& match : matches_used_calc[i]) {
						/* p1�Ɖ�] */
						cv::Point env_p = imgForCalcLikeAll_inImgAll_keypoint[i][match.queryIdx].pt;
						cv::Point mea_p = keypoints[match.trainIdx].pt;

						Coor<> env_c(env_p.x - center.x, env_p.y - center.y);
						Coor<> mea_c(mea_p.x - center.x, mea_p.y - center.y);

						double rad_env_c = std::atan2(env_c.y, env_c.x);
						double rad_mea_c = std::atan2(mea_c.y, mea_c.x);

						double d_key_rad = rad_env_c - rad_mea_c;

						double d_pos_rad = imgForCalcLikeAll_positioin[i].r - particle.r;

						A += std::abs(SURF_AMP_FUNCTION(d_pos_rad - d_key_rad));

						//if (size > 100){
						//	std::cout << imgForCalcLikeAll_positioin[i].r << "," << particle.r << std::endl;
						//	std::cout << d_key_rad <<","<< d_pos_rad << std::endl;
						//}



						//cv::Point p1_tmp = imgForCalcLikeAll_inImgAll_keypoint[i][match.queryIdx].pt;
						//cv::Point p1;
						//p1.x = std::cos(particle.r)*(p1_tmp.x - center.x) + std::sin(particle.r)*(p1_tmp.y - center.y) + center.x;
						//p1.y = -std::sin(particle.r)*(p1_tmp.x - center.x) + std::cos(particle.r)*(p1_tmp.y - center.y) + center.y;
						//cv::Point p2 = keypoints[match.trainIdx].pt;
						//double dx = p1.x - p2.x;
						//double dy = p1.y - p2.y;
						//double d_key = std::sqrt(dx*dx + dy*dy);
						//// �ޓx���Z
						//A += SURF_AMP_FUNCTION(d_key);
					}
					double tmp = A*xy;
					out += tmp;
					//std::cout << size << " " << A << std::endl;
				}
			}
			//std::cout << out << std::endl;
			assert(out > 0);
			return out;
		}
		default:
			assert(feature == OMNI_FEATURE_SIFT || feature == OMNI_FEATURE_SURF);
			break;
		}


	}

	/**********************************************************/
	//	�v���f�[�^��clear
	/**********************************************************/

	void clearMeasurement()
	{
		sim_img_idx.clear();
		imgForCalcLikeAll_positioin.clear();
		imgForCalcLikeAll_descriptors.clear();
		imgForCalcLikeAll_inImgAll_keypoint.clear();
		matches_used_calc.clear();
	}


	/**********************************************************/
	//	�����o�ϐ�
	/**********************************************************/

public:

	/*  ���f�[�^  */
	cv::Mat_<float> env_histgram;		//	���摜�̃q�X�g�O����
	cv::Mat_<float> env_centroid;		//	���摜�̃Z���g���C�h
	cv::VideoCapture env_img_cap;
	std::vector<cv::Mat> env_img;
	std::vector<Position<>> env_img_position;				//	���n�}�摜�̎B�e�ʒu
	std::vector<std::vector<cv::KeyPoint>> env_keypoints;
	std::vector<cv::Mat> env_descriptors;


	/*  �v���f�[�^  */
	cv::Mat img;					//	���S����p
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat_<float> descriptors;		// ROWS:�����_�� COLS:128
	cv::Mat_<double> dot_product;	//	���q�X�g�O�����ƌv���q�X�g�O�����̓���

									/*  �ޓx�����֘A  */
	std::vector<int> sim_img_idx;	//	�v���摜�Ɨގ�������摜�̃C���f�b�N�X

									/* SURF�p */
	std::vector<std::vector<cv::DMatch>> matches_used_calc;	//	[�摜�ԍ�][DMatch�ԍ�]
	std::vector<double> imgForCalcAll_dmatch_size;
	std::vector<cv::Mat> imgForCalcAll_img;
	std::vector<Position<>> imgForCalcLikeAll_positioin;	//	[�|�W�V�����ԍ�]
	std::vector<std::vector<cv::KeyPoint>> imgForCalcLikeAll_inImgAll_keypoint;	// [�摜�ԍ�][�L�[�|�C���g�ԍ�]
	std::vector<cv::Mat> imgForCalcLikeAll_descriptors;


};