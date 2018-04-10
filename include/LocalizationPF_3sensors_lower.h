
#pragma once

#include "parameter.h"
#include "ParticleFilter.h"
#include "leica.h"
#include "include/myfun.h"
#include "include/mycv.h"
#include "include/Lidar2d.h"
#include "include/OmniCamera.h"
#include "include/GPS.h"

#include <direct.h>
#include <signal.h>

bool fin = false;


/**********************************************************/
//	PFによる位置推定用のクラス
/**********************************************************/
//	遷移モデルのみ実装
//	計測モデルは各プログラムごとに実装
class LocalizationPF :public ParticleFilter<Position<>>
{
public:

	/**********************************************************/
	//  パラメータのファイル出力
	/**********************************************************/
	void outputParameter(std::string ofpath) {
		std::string filename = "C://Users/Robot/Documents/Visual Studio 2013/Projects/Localization.ver2/include/parameter.h";
		//std::string filename = "C://Users/robot/Desktop/Localization.ver2_2/Localization.ver2/include/parameter.h";
		//std::string filename = "C://Users/Nozomu Ohashi/Documents/Visual Studio 2013/Projects/Localization/Localization.ver2/include/parameter.h";
		std::ifstream ifs(filename);
		if (ifs.fail()) {
			readError(filename);
		}
		std::ofstream ofs1(ofpath + "parameter.txt");
		std::string str;
		while (std::getline(ifs, str)) {
			ofs1 << str << std::endl;
		}
	}

	/*  コンストラクタ  */
	LocalizationPF()
	{
		trial_type = TRIAL_TYPE;

		///* thread確保 */
		//lid2_l.resize(LIKELIHOOD_THREAD);
		//lid2_l.resize(LIKELIHOOD_THREAD);
		//omni.resize(LIKELIHOOD_THREAD);
		//gpgga.resize(LIKELIHOOD_THREAD);
		likelihood_threads.resize(LIKELIHOOD_THREAD);

		/*  自己位置推定の初期位置  */
		//	GL座標系からLC座標系に変換
		gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
		//ini_position = gl2lc.getPosition(INI_POS_LAT, INI_POS_LON, INI_POS_ELE, INI_POS_HEAD);


		/*  パーティクルフィルタのパラメータ  */
		sample_size = SAMPLE_SIZE;
		ini_sample_radius.set(INI_SAMPLE_RADIUS_X, INI_SAMPLE_RADIUS_Y, INI_SAMPLE_RADIUS_R);
		trans_par_sys_var.set(TRANS_PAR_SYS_VAR_X, TRANS_PAR_SYS_VAR_Y, TRANS_PAR_SYS_VAR_R);

		/*  オドメトリ誤差分散  */
		//odometry_system_noise.set(ODOMETRY_SYSTEM_NOISE_R, ODOMETRY_SYSTEM_NOISE_THETA);

		/*  オドメトリデータの初期値  */
		odometry1 = new Position<>(0.0, 0.0, 0.0);

		lid2_l.resize(LIKELIHOOD_THREAD);
		omni.resize(LIKELIHOOD_THREAD);
		gpgga.resize(LIKELIHOOD_THREAD);

	};

	/*  デストラクタ  */
	//	ポインタメンバ変数の領域解放"delete"を忘れずに
	~LocalizationPF()
	{
		delete odometry1, odometry2;
		delete true_position;
		map_img.release();
		map_img_color.release();
		map_img_clone.release();

		lid2_l.clear();
		omni.clear();
		gpgga.clear();

		lid2_l.shrink_to_fit();
		omni.shrink_to_fit();
		gpgga.shrink_to_fit();

		particle_video.release();
		particle_large_video.release();
		weighted_stat_particle_video.release();
		measurement_data_video.release();

		allClear();

	};

	void allClear() {

		estimated_position.clear();		//	重み付き平均による推定位置
		result_time.clear();
		all_particles.clear();
		all_particles_after_resampling.clear();
		all_stat_particles.clear();
		all_lid2_l_likelihood.clear();
		all_omni_likelihood.clear();
		all_gpgga_likelihood.clear();
		all_fusion_likelihood.clear();
		all_stat_lid2_l_likelihood.clear();
		all_stat_omni_likelihood.clear();
		all_stat_gpgga_likelihood.clear();
		all_omni_img_sim_pos.clear();
		all_omni_img_sim.clear();
		all_stock_tidx.clear();
		diff_time_ini_now.clear();
		all_error.clear();
		error_time.clear();
		all_omni_img_sim.clear();
		all_omni_img_sim_pos.clear();
		all_meas_gps_pos.clear();
		all_th.clear();

		estimated_position.shrink_to_fit();		//	重み付き平均による推定位置
		result_time.shrink_to_fit();
		all_particles.shrink_to_fit();
		all_particles_after_resampling.shrink_to_fit();
		all_stat_particles.shrink_to_fit();
		all_lid2_l_likelihood.shrink_to_fit();
		all_omni_likelihood.shrink_to_fit();
		all_gpgga_likelihood.shrink_to_fit();
		all_fusion_likelihood.shrink_to_fit();
		all_stat_lid2_l_likelihood.shrink_to_fit();
		all_stat_omni_likelihood.shrink_to_fit();
		all_stat_gpgga_likelihood.shrink_to_fit();
		all_omni_img_sim_pos.shrink_to_fit();
		all_omni_img_sim.shrink_to_fit();
		all_stock_tidx.shrink_to_fit();
		diff_time_ini_now.shrink_to_fit();
		all_error.shrink_to_fit();
		error_time.shrink_to_fit();
		all_omni_img_sim.shrink_to_fit();
		all_omni_img_sim_pos.shrink_to_fit();
		all_meas_gps_pos.shrink_to_fit();
		all_th.shrink_to_fit();

		esti_time = ini_time;
		read_meas_step = ini_step;
		tidx = ini_tidx;
		no = ini_no;
		finish = false;
		fin_movie_creator_ = false;
		read_all_measurement_ = false;
		now_step = 0;
		movie_step = 0;

		*odometry1 = Position<>(0.0, 0.0, 0.0);
		stat_particles.clear();
		stat_lid2_l_likelihood.clear();
		stat_omni_likelihood.clear();
		stat_gpgga_likelihood.clear();
		lid2_l_likelihood.clear();
		omni_likelihood.clear();
		gpgga_likelihood.clear();
		fusion_likelihood.clear();

		particle_video.release();
		particle_large_video.release();
		weighted_stat_particle_video.release();
		measurement_data_video.release();
		add_measurement_movie_step = 0;
		fin_add_measurement_movie_ = false;

		use_.clear();							// 	各ステップで選択されたセンサ
		similarity_table.clear();	//	センサから得られる確率分布間の類似度
		similar_table_.clear();		//	センサから得られる確率分布間を類似・非類似で閾値処理

		all_meas_lrf_l.clear();
		all_meas_keypoints.clear();
		all_meas_desriptor.clear();
		all_meas_gps.clear();
		all_meas_odometry.clear();
		all_meas_time.clear();


		error_time.clear();
	}

	void initOutput(std::string ofpath)
	{
		/* Make Folder */
		_mkdir(ofpath.c_str());
		std::string ofpath_data = ofpath + "Data/";
		_mkdir(ofpath_data.c_str());
		std::string ofpath_data_gitignore = ofpath + "Data/gitignore/";
		_mkdir(ofpath_data_gitignore.c_str());
		std::string ofpath_image = ofpath + "image/";
		_mkdir(ofpath_image.c_str());
		std::string ofpath_movie = ofpath + "movie/";
		_mkdir(ofpath_movie.c_str());

		///*  推定位置出力ファイルを初期化  */
		//initOutputEstiPosition(ofpath);

		///*  推定誤差の出力ファイルを初期化  */
		//initOutputEstiError(ofpath);

		//initEliminateSensorForAnalysis();

		outputParameter(ofpath);

		initLastInpugFileNumber(ofpath);

		/* 計測動画初期化 */
		initMeasurementDataVideo(ofpath);

		/*  パーティクル動画の初期化  */
		initParticleVideo(ofpath);
		initParticleLargeVideo(ofpath);

		initWeightedStatParVideo(ofpath);

		//initUseSensor();
		//initEliminateSensorForAnalysis();
		//initUseType();
		//initSimilarityTable();
		//initSimilarTable_();
	}

	/**********************************************************/
	//  ファイルの読み込み
	/**********************************************************/

public:

	void readGridMap(std::string filename)
	{
		/*  障害物地図の読み込み  */
		map_img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		map_img_color = cv::imread(filename, CV_LOAD_IMAGE_COLOR);	//	障害物地図をカラーで保存
		if (map_img.empty() || map_img_color.empty())	readError(filename);
	}

	/*  障害物地図の読み込み  */
	void readGridMapEach(std::string ifpath)
	{
		/*  障害物地図の読み込み  */
		std::string filename;
		filename = ifpath + "lrf_lower/gridmap.bmp";
		map_img_upper = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
		if (map_img_upper.empty()) {
			readError(filename);
		}
	}

	/*  オドメトリデータの読み込み  */
	bool readOdometry(int no, const int step)
	{
		/*  領域の確保  */
		if (!odometry2)	odometry2 = new Position<>;

		/*  オドメトリデータの読み込み  */
		std::string filename = IFPATH_MEAS + "odometry/odometry_no" + std::to_string(no) + "_" + std::to_string(step) + "th.csv";
		std::ifstream ifs_odo(filename);
		if (ifs_odo.fail())	return false;	//	読み込みに失敗した場合，falseを返す
		ifs_odo >> *odometry2;

		odometry2->r *= M_PI / 1800.0;

		return true;
	}


	/*  計測時刻の読み込み  */
	bool readTime(int no, const int step)
	{
		std::string filename = IFPATH_MEAS + "time/time_no" + std::to_string(no) + "_" + std::to_string(step) + "th.csv";
		std::ifstream ifs(filename);
		if (ifs.fail())	return false;

		MyTime tmp;
		ifs >> tmp;

		esti_time = tmp;
		return true;
	}



	/*  真の位置を読み込み  */
	void readTruePosition(int no)
	{
		/* leica 読み込み */
		gl2lc.setOriginal(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE, MAP_ORG_HEAD);
		Coor<> leica_coor = gl2lc.getCoor(LEICA_ORG_LAT, LEICA_ORG_LON, LEICA_ORG_ELE);
		leica::Param<> param_leica(leica_coor);
		param_leica.setHorizontalErrorDeg(LEICA_HORIZONTAL_ERROR);
		std::vector<leica::Data<>> dataset_leica;
		{
			std::string filename = IFPATH_MEAS + "true_position/true_position.csv";
			std::ifstream ifs(filename);
			if (ifs.fail())	readError(filename);
			leica::readDeg(ifs, param_leica, dataset_leica);
		}

		/*  領域の確保  */
		if (!true_position)	true_position = new std::vector<Position<>>;

		for (int i = 0; i < dataset_leica.size(); i++) {
			int tmp = i + 1;
			while (tmp < dataset_leica.size() && leica::distance(dataset_leica[i], dataset_leica[tmp]) < 3000) {
				tmp++;
			}
			if (tmp >= dataset_leica.size()) {
				tmp = i;
			}
			Position<> pos = dataset_leica[i].position(dataset_leica[tmp]);
			if (tmp >= dataset_leica.size() - 1) {
				pos.r = true_position->back().r;
			}
			true_position->push_back(pos);
			true_time.push_back(dataset_leica[i].time);
		}

		/* 初期位置がmap内になるようにする */
		while (true) {
			int tmp = tidx + 1;
			while (leica::distance(dataset_leica[tidx], dataset_leica[tmp]) < 3000) {
				tmp++;
			}
			ini_position = dataset_leica[tidx].position(dataset_leica[tmp]);
			cv::Point pixel = ToPixel(ini_position, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			if (pixel.x < 0 || map_img.cols < pixel.x ||
				pixel.y < 0 || map_img.rows < pixel.y) {
				std::cout << "Initial Position is out of Map!" << std::endl;
				exit(0);
			}
			if (!onObject_(ini_position)) {
				break;
			}
			tidx++;
		}
		/* 計測ステップも進める */
		while (true) {
			readTime(no, read_meas_step + 1);

			if (true_time[tidx] < esti_time) {
				read_meas_step++;
				bool o_ = LocalizationPF::readOdometry(read_meas_no, read_meas_step);
				assert(o_ == true);
				*odometry1 = *odometry2;

				break;
			}
			read_meas_step++;
		}

		ini_time = esti_time;
		ini_step = read_meas_step;
		ini_tidx = tidx;
		ini_no = no;

		std::cout << "Skip steps: 1-" << read_meas_step + 1 << std::endl;
		std::cout << "Initial position: " << ini_position << std::endl;
	}



	void readEnvironment() {

		/*  障害物地図を読み込み  */
		LocalizationPF::readGridMap(IFPATH_ENV + "lrf_lower/gridmap.bmp");
		LocalizationPF::readGridMapEach(IFPATH_ENV);


		/*  真の位置情報の読み込み  */
		LocalizationPF::readTruePosition(no);

		/*  センサデータの読み込み  */
		GlobalToLocal& gl2lc_tmp = gl2lc;

		std::string filename = IFPATH_ENV + "map.csv";

		lid2_l[0].readPointCloud(IFPATH_ENV + "lrf_lower/pointcloud.csv");
		lid2_l[0].setEnvICP();
		omni[0].readEnvImgPosition(IFPATH_ENV_OMNI + "omni/img_pos.csv");
		omni[0].readImg(IFPATH + "Environment/img.bmp");



		//omni[0].readEnvImgAvi(IFPATH_ENV_OMNI + "omni/img.avi");
		switch (OMNI_FEATURE)
		{
		case OMNI_FEATURE_SIFT:
			omni[0].readEnvKeypoint(IFPATH_ENV_OMNI + "omni/sift/");
			omni[0].readEnvDescriptor(IFPATH_ENV_OMNI + "omni/sift/");
			assert(omni[0].env_img_position.size() == omni[0].env_keypoints.size());
			assert(omni[0].env_img_position.size() == omni[0].env_descriptors.size());
			std::cout << omni[0].env_img_position.size() << std::endl;
			std::cout << omni[0].env_keypoints.size() << std::endl;
			if (USE_BOF){
				omni[0].readEnvCentroid(IFPATH_ENV_OMNI + "omni/sift/centroid.csv");
				omni[0].readEnvHistgram(IFPATH_ENV_OMNI + "omni/sift/histogram.csv");
			}
			break;
		case OMNI_FEATURE_SURF:
			omni[0].readEnvKeypoint(IFPATH_ENV_OMNI + "omni/surf/");
			omni[0].readEnvDescriptor(IFPATH_ENV_OMNI + "omni/surf/");
			if (USE_BOF){
				omni[0].readEnvCentroid(IFPATH_ENV_OMNI + "omni/surf/centroid.csv");
				omni[0].readEnvHistgram(IFPATH_ENV_OMNI + "omni/surf/histgram.csv");
			}
			break;
		default:
			std::cout << "OMNI_FEATURE: " << OMNI_FEATURE << std::endl;
			break;
		}
		//omni_tmp[th].readEnvDescriptor(ENVIRONMENT_DATE_OMNI + "omni/surf/");
		gpgga[0].setMapOrgGL(MAP_ORG_LAT, MAP_ORG_LON, MAP_ORG_ELE);

		std::vector<Coor<>> point_cloud_upper = lid2_l[0].point_cloud;





	}


	void setEnvironmentDetach() {

		//omni[0].setEnvImgFromAvi();
		//omni[0].setEnvDescriptor(OMNI_FEATURE);
		//omni[0].clearEnvImg();
		//omni.front().setEnvDescriptor(ENV_DESCRITOR_THREAD);

		GlobalToLocal& gl2lc_tmp = gl2lc;

		//for (int th = 0; th < LIKELIHOOD_THREAD; th++)
		//{
		//	Lidar2d& lid2_l_tmp = lid2_l[th];
		//	OmniCamera& omni_tmp = omni[th];
		//	Gps& gpgga_tmp = gpgga[th];
		//	likelihood_threads[th] = std::thread([th, &lid2_l_tmp, &lid2_l_tmp, &omni_tmp, &gpgga_tmp, gl2lc_tmp] {
		//		lid2_l_tmp = lid2_l_tmp.front();
		//		lid2_l_tmp.setEnvICP();
		//		omni_tmp = omni_tmp.front();
		//		gpgga_tmp = gpgga_tmp.front();
		//	});
		//}

		for (int th = 1; th < LIKELIHOOD_THREAD; th++)
		{
			lid2_l[th] = lid2_l[0];
			lid2_l[th].setEnvICP();
			omni[th] = omni[0];
			gpgga[th] = gpgga[0];
		}

		fin_read_env = true;
		std::cout << "Complete Environment" << std::endl;

	}
	void setEnvironment() {

		/* 読み込み */
		readEnvironment();

		set_environment_thread = std::thread(&LocalizationPF::setEnvironmentDetach, this);

		set_environment_thread.detach();


	}

	/* 信頼度マップ */
	void setReliabilityMap() {
		{
			std::string filename = IFPATH_RMAP + "reliability_map_gps.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()) {
				readError(filename);
			}
			ifs >> cvReadData<float>(reliabity_map_gps);
		}
		{
			std::string filename = IFPATH_RMAP + "reliability_map_omni.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()) {
				readError(filename);
			}
			ifs >> cvReadData<float>(reliabity_map_omni);
		}
		{
			std::string filename = IFPATH_RMAP + "reliability_map_lrf_l.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()) {
				readError(filename);
			}
			ifs >> cvReadData<float>(reliabity_map_lrf_l);
		}
	}


	/**********************************************************/
	//  途中スタート用
	/**********************************************************/

	/*  パーティクルの読み込み  */
	void readParticle(std::string ofpath)
	{
		/*  読み込み  */
		std::string filename = ofpath + "Data/particle_state_last.csv";
		std::ifstream ifs_par(filename);
		if (ifs_par.fail()) readError(filename);

		std::string str;
		while (std::getline(ifs_par, str))
		{
			std::istringstream istr(str);

			Particle<Position<>>* par = new Particle<Position<>>;
			Position<>* position = new Position<>;

			istr >> *position;
			par->setState(position);
			getParticles().push_back(par);
		}

	}

	/*  動画の読み込み  */
	void readParticleVideo()
	{
		//std::string filename = OFPATH + "Movie/particle.avi";
		//
		//cv::VideoCapture vcr(filename);
		//if (!vcr.isOpened()) readError(filename);


		///*  ビデオ関係  */
		//// ファイルをオープンし，ビデオライタを初期化
		//// filename - 出力ファイル名
		//// fourcc - コーデック
		//// fps - 1 秒あたりのフレーム数
		//// frameSize - ビデオフレームのサイズ
		//// isColor - ビデオストリームがカラーか，グレースケールかを指定
		//particle_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, cv::Size(1500, 1500));
		//if (!particle_video.isOpened())	writeError(filename);

		//while (1)
		//{
		//	cv::Mat mat;

		//	vcr >> mat;

		//	if (mat.empty()) break;

		//	particle_video << mat;

		//}
	}

	/*  推定位置の読み込み  */
	void readEstPos(std::string ofpath)
	{
		/*  読み込み  */
		std::string filename = ofpath + "Data/estimated_position.csv";
		std::ifstream ifs(filename);
		if (ifs.fail()) readError(filename);

		/*  1行スキップ  */
		std::string str;
		std::getline(ifs, str);

		ifs >> estimated_position;
	}


	/*  オドメトリデータの更新  */
	void swapOdometry()
	{
		*odometry1 = *odometry2;
	}

	/*  読み込みクラス  */
	/**********************************************************/
	//	出力ファイルの初期化

	/**********************************************************/

	/*  推定位置の出力ファイルを初期化  */
	void initOutputEstiPosition(std::string OFPATH)
	{
		{
			std::string filename = OFPATH + "Data/estimated_position.csv";
			std::ofstream ofs_est(filename, std::ios_base::out);
			if (ofs_est.fail())	writeError(filename);
			ofs_est << "time,s,x,y,rad" << std::endl;
			ofs_est.close();
		}
		//{
		//	std::string filename = "./output/Data/estimated_position.csv";
		//	std::ofstream ofs_est(filename, std::ios_base::out);
		//	if (ofs_est.fail())	writeError(filename);
		//	ofs_est << "time,x,y,rad" << std::endl;
		//	ofs_est.close();
		//}
	}

	/*  推定誤差の出力ファイルを初期化  */
	void initOutputEstiError(std::string OFPATH)
	{
		{
			std::string filename = OFPATH + "Data/error.csv";
			std::ofstream ofs_err(filename, std::ios_base::out);
			if (ofs_err.fail()) writeError(filename);
			ofs_err << "time,s,x,y,rad,abs" << std::endl;
			ofs_err.close();
		}
		//{
		//	std::string filename = "./output/Data/error.csv";
		//	std::ofstream ofs_err(filename, std::ios_base::out);
		//	if (ofs_err.fail()) writeError(filename);
		//	ofs_err << "time,x,y,rad" << std::endl;
		//	ofs_err.close();
		//}

	}

	void initLastInpugFileNumber(std::string OFPATH) {

		std::string filename = OFPATH + "Data/last_input_file_number.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		ofs << 0 << std::endl;
		ofs.close();
	}


	/*  パーティクル動画の初期化  */
	void initParticleVideo(std::string ofpath)
	{
		/*  ビデオ関係  */
		// ファイルをオープンし，ビデオライタを初期化
		// filename - 出力ファイル名
		// fourcc - コーデック
		// fps - 1 秒あたりのフレーム数
		// frameSize - ビデオフレームのサイズ
		// isColor - ビデオストリームがカラーか，グレースケールかを指定

		std::string filename = ofpath + "Movie/particle.avi";
		//std::string filename = "./output/Movie/particle.avi";
		Coor<> rect(CUT_MAP_RADIUS_X*6.0, CUT_MAP_RADIUS_Y*4.0);
		cv::Size rect_pix = ToPixelSize(rect, map_img_clone, MAP_RES);
		rect_pix.width *= MOVIE_SCALE_W;
		rect_pix.height *= MOVIE_SCALE_H;
		particle_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, rect_pix);
		if (!particle_video.isOpened())	writeError(filename);
	}

	void initParticleLargeVideo(std::string ofpath)
	{
		/*  ビデオ関係  */
		// ファイルをオープンし，ビデオライタを初期化
		// filename - 出力ファイル名
		// fourcc - コーデック
		// fps - 1 秒あたりのフレーム数
		// frameSize - ビデオフレームのサイズ
		// isColor - ビデオストリームがカラーか，グレースケールかを指定

		std::string filename = ofpath + "Movie/particle_large.avi";
		//std::string filename = "./output/Movie/particle.avi";
		Coor<> rect(CUT_MAP_RADIUS_X*6.0, CUT_MAP_RADIUS_Y*4.0);
		cv::Size rect_pix = ToPixelSize(rect, map_img_color, MAP_RES);
		rect_pix.width *= MOVIE_SCALE_W;
		rect_pix.height *= MOVIE_SCALE_H;
		particle_large_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, rect_pix);
		if (!particle_large_video.isOpened())	writeError(filename);
	}

	void initWeightedStatParVideo(std::string ofpath)
	{
		/*  ビデオ関係  */
		// ファイルをオープンし，ビデオライタを初期化
		// filename - 出力ファイル名
		// fourcc - コーデック
		// fps - 1 秒あたりのフレーム数
		// frameSize - ビデオフレームのサイズ
		// isColor - ビデオストリームがカラーか，グレースケールかを指定

		std::string filename = ofpath + "Movie/weighted_stat_particle.avi";
		//std::string filename = "./output/Movie/weighted_stat_particle.avi";
		Coor<> rect(CUT_MAP_RADIUS_X*4.0, CUT_MAP_RADIUS_Y*4.0);
		cv::Size rect_pix = ToPixelSize(rect, map_img_clone, MAP_RES);
		rect_pix.width *= MOVIE_SCALE_W;
		rect_pix.height *= MOVIE_SCALE_H;
		weighted_stat_particle_video.open(filename, CV_FOURCC('X', 'V', 'I', 'D'), PARTICLE_FPS, rect_pix);
		if (!weighted_stat_particle_video.isOpened())	writeError(filename);
	}

	void initMeasurementDataVideo(std::string ofpath)
	{
		/*  ビデオ関係  */
		// ファイルをオープンし，ビデオライタを初期化
		// filename - 出力ファイル名
		// fourcc - コーデック
		// fps - 1 秒あたりのフレーム数
		// frameSize - ビデオフレームのサイズ
		// isColor - ビデオストリームがカラーか，グレースケールかを指定

		std::string filename = ofpath + "Movie/measurement_data.avi";
		int fps = MEASUREMENT_DATA_VIDEO_FPS;
		Coor<> rect(CUT_MAP_RADIUS_X*4.0, CUT_MAP_RADIUS_Y*4.0);
		cv::Size rect_pix = ToPixelSize(rect, map_img_clone, MAP_RES);
		rect_pix.width *= MOVIE_SCALE_W;
		rect_pix.height *= MOVIE_SCALE_H;
		measurement_data_video.open(filename, MEASUREMENT_DATA_VIDEO_FOURCC, fps, rect_pix);
		if (!measurement_data_video.isOpened())	writeError(filename);
	}


	/**********************************************************/
	//  パーティクルフィルタ
	/**********************************************************/

	void Transition()
	{
		/*  状態遷移に付加する雑音の準備  */
		std::random_device rnd;     // 非決定的な乱数生成器を生成
		std::mt19937_64 mt(rnd());     //  メルセンヌ・ツイスタの64ビット版、引数は初期シード値

		//	パーティクルフィルタシステムノイズ
		Position<> par_mean(0.0, 0.0, 0.0);
		std::normal_distribution<double> par_sys_noise_x(par_mean.x, trans_par_sys_var.x);
		std::normal_distribution<double> par_sys_noise_y(par_mean.y, trans_par_sys_var.y);
		std::normal_distribution<double> par_sys_noise_r(par_mean.r, trans_par_sys_var.r);

		/*  ロボットの移動量をRobot座標系からLocal座標系に変換  */

		Polar<> delta;	//	移動半径と変化角度はRobot座標系とLocal座標系で共通

		//	前のステップからの移動量半径を算出
		delta.r = *odometry2 | *odometry1;
		delta.theta = odometry2->r - odometry1->r;

		//	オドメトリ
		Polar<> odo_mean(0.0, 0.0);	//  平均は全て0
		if (delta.theta < M_PI / 16){
			odometry_system_noise.set(std::abs(delta.r / 2.0) + 0.0001, std::abs(delta.theta) + 0.0001);
		}
		else{
			odometry_system_noise.set(std::abs(delta.r / 2.0) + 0.0001, std::abs(M_PI/16) + 0.0001);
		}

		std::normal_distribution<double> noise_r(odo_mean.r, odometry_system_noise.r);				//	ガウス雑音を生成
		std::normal_distribution<double> noise_theta(odo_mean.theta, odometry_system_noise.theta);	//	ガウス雑音を生成


		switch (trial_type)
		{
		case TRIAL_SIMULTANEOUS:
		case TRIAL_PEARSON:
		case TRIAL_PEARSON_NONSTAT:
		case TRIAL_SUYAMA_STAT:
		case TRIAL_SUYAMA_NONSTAT:
		case TRIAL_3SENSORS_SIMULATNEOUS:
		case TRIAL_3SENSORS_PEARSON:
		case TRIAL_3SENSORS_PEARSON_NONSTAT:
		case TRIAL_3SENSORS_SUYAMA_NONSTAT:
			/*  パーティクルの遷移  */
			for (const auto &par : getParticles())
			{
				Position<> par_tmp = *par->getState();
				Polar<> delta_tmp = delta;

				//	オドメトリのノイズ付加
				Polar<> noise(noise_r(mt), noise_theta(mt));
				delta_tmp += noise;
				//	パーティクルの遷移
				par_tmp.x += delta_tmp.r*std::cos(par_tmp.r);
				par_tmp.y += delta_tmp.r*std::sin(par_tmp.r);
				par_tmp.r += delta_tmp.theta;

				//	パーティクルフィルタのシステムノイズ付加
				Position<> par_noise(par_sys_noise_x(mt), par_sys_noise_y(mt), par_sys_noise_r(mt));
				par_tmp += par_noise;

				*par->getState() = par_tmp;

			}

			break;
		case TRIAL_NON_TIMESEQUENCE:
		case TRIAL_NON_TIMESEQUENCE_SIMUL:
			/*  パーティクルの遷移  */
			for (const auto &par : getParticles())
			{
				Position<> par_tmp = *par->getState();
				Polar<> delta_tmp = delta;

				//	オドメトリのノイズ付加
				Polar<> noise(noise_r(mt), noise_theta(mt));
				delta_tmp += noise;
				//	パーティクルの遷移
				par_tmp.x += delta_tmp.r*std::cos(par_tmp.r);
				par_tmp.y += delta_tmp.r*std::sin(par_tmp.r);
				par_tmp.r += delta_tmp.theta;

				*par->getState() = par_tmp;

			}

			break;
		default:
			std::cout << "Error 'TRIAL': " << TRIAL_TYPE << "FILE: " << __FILE__ << " LINE: " << __LINE__ << std::endl;
			exit(0);
			break;
		}
	


		std::cout << "Complete 'Transition' " << std::endl;
	};

	/*  計測データの読み込み  */
	//std::vector<cv::KeyPoint> setKeypoints(cv::Mat img) {
	//	std::vector<cv::KeyPoint> keypoints;
	//	/*  Operatorの塗りつぶし  */
	//	cv::Point center(img.cols / 1.9, img.rows / 2);
	//	cv::Size radius(img.rows / 3, img.rows / 3);
	//	double start_angle = -105;	//	扇系の中心角度
	//	double angle = 35;	//	角度
	//	cv::ellipse(img, center, radius, start_angle, 0, angle, cv::Scalar(0, 0, 0), -1, CV_AA);
	//	/* 外枠の塗りつぶし */
	//	cv::circle(img, center, 510, cv::Scalar(0, 0, 0), 150);
	//	//　等間隔の画像に切り出し
	//	cv::Rect roi_rect(ROI_ORG_X, ROI_ORG_Y, ROI_SIZE_X, ROI_SIZE_Y); // x,y,w,h
	//	img = img(roi_rect);
	//	//SURF
	//	cv::SurfFeatureDetector detector(1000);
	//	detector.detect(img, keypoints);
	//	return keypoints;
	//}

	//cv::Mat setDescriptor(cv::Mat img, std::vector<cv::KeyPoint> keypoints) {
	//	cv::SurfDescriptorExtractor extractor;
	//	//画像の特徴点における特徴量を抽出
	//	cv::Mat descriptors;
	//	extractor.compute(img, keypoints, descriptors);
	//	return descriptors;
	//}

	//
	void readMeasurementDetach() {
		{
			std::string filename = IFPATH_MEAS + "odometry/odometry_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step - 1) + "th.csv";
			std::ifstream ifs_odo(filename);
			if (ifs_odo.fail()) {
				readError(filename);
			}
			Position<> odo;
			ifs_odo >> odo;
			odo.r *= M_PI / 1800.0;
			*odometry1 = odo;
		}

		while (true) {
			clock_t lap1 = clock();
			std::string filename;
			filename = IFPATH_MEAS + "lrf_lower/lrf_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
			std::vector<Polar<>> scan_l;
			std::ifstream ifs_lrf_l(filename);
			if (ifs_lrf_l.fail()) {
				std::cout << filename << " dose not exist" << std::endl;
				break;
			}
			ifs_lrf_l >> scan_l;
			all_meas_lrf_l.push_back(scan_l);

			clock_t lap2 = clock();

			//filename = IFPATH_MEAS + "img/img_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.bmp";
			//cv::Mat img;
			//img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
			//if (img.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	break;
			//}
			////all_meas_img.push_back(img);

			clock_t lap3 = clock();
			filename = IFPATH_MEAS + "surf/keypoint/keypoint_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.yml";
			std::vector<cv::KeyPoint> keys;
			cv::FileStorage fs(filename, cv::FileStorage::READ);
			cv::FileNode fn = fs["keypoints"];
			cv::read(fn, keys);
			if (keys.empty()){
				std::cout << filename << " dose not exist" << std::endl;
				break;
			}
			all_meas_keypoints.push_back(keys);

			//std::vector<cv::KeyPoint> keys = setKeypoints(img);
			//if (keys.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	break;
			//}
			//all_meas_keypoints.push_back(keys);

			clock_t lap4 = clock();

			filename = IFPATH_MEAS + "surf/descriptor/desc_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()){
				break;
			}
			clock_t lap4_5 = clock();
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
			cv::Mat descriptor;
			for (int i = 0; i < v.size(); i++){
				cv::Mat mat(v[i], true);
				mat = mat.t();
				descriptor.push_back(mat);
			}
			all_meas_desriptor.push_back(descriptor);

			//cv::Mat desc = setDescriptor(img, keys);
			//if (desc.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	break;
			//}
			//all_meas_desriptor.push_back(desc);

			clock_t lap5 = clock();
			filename = IFPATH_MEAS + "gps/gps_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.txt";
			std::ifstream ifs_gps(filename);
			if (ifs_gps.fail()) {
				std::cout << filename << " dose not exist" << std::endl;
				break;
			}
			std::string str;
			ifs_gps >> str;
			all_meas_gps.push_back(str);

			clock_t lap6 = clock();
			filename = IFPATH_MEAS + "odometry/odometry_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
			std::ifstream ifs_odo(filename);
			if (ifs_odo.fail()) {
				std::cout << filename << " dose not exist" << std::endl;
				break;
			}
			Position<> odo;
			ifs_odo >> odo;
			odo.r *= M_PI / 1800.0;
			all_meas_odometry.push_back(odo);

			clock_t lap7 = clock();
			filename = IFPATH_MEAS + "time/time_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
			std::ifstream ifs_time(filename);
			if (ifs_time.fail()) {
				std::cout << filename << " dose not exist" << std::endl;
				break;
			}
			MyTime time;
			ifs_time >> time;
			all_meas_time.push_back(time);

			std::cout << "ReadMeasurement: " << read_meas_step << std::endl;
			//std::cout << lap1 << "," << lap2 << "," << lap3 << "," << lap4 << "," << lap4_5 << "," << lap4_7 << "," << lap5 << "," << lap6 << "," << lap7 << std::endl;

			read_meas_step += SKIP_STEPS;
			//if (read_meas_step >= 10){
			//	break;
			//}
		}
		read_all_measurement_ = true;
	}

	//
	void readMeasurement() {

		read_measurement_thread = std::thread(&LocalizationPF::readMeasurementDetach, this);
		read_measurement_thread.detach();
		//read_measurement_thread.join();

	}

	//
	void setMeasurement() {
		std::cout << 1 << std::endl;
		while (true) {
			int min = std::min({ all_meas_odometry.size(), all_meas_time.size(), all_meas_lrf_l.size(), all_meas_keypoints.size(), all_meas_desriptor.size(), all_meas_gps.size() });
			if (now_step < min) {
				break;
			}
			if (read_all_measurement_) {
				finish = true;
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(MOVIE_CREATER_SLEEP_MILLISECONDS));
		}

		if (odometry2 == nullptr) {
			odometry2 = new Position<>;
		}
		*odometry2 = all_meas_odometry[now_step];
		esti_time = all_meas_time[now_step];

		for (int i = 0; i < LIKELIHOOD_THREAD; i++) {

			lid2_l[i].setMeasScan(all_meas_lrf_l[now_step]);
			omni[i].setKeypoints(all_meas_keypoints[now_step]);
			omni[i].setDescriptors(all_meas_desriptor[now_step]);
			gpgga[i].setMeasSignal(all_meas_gps[now_step]);
		}
		all_meas_gps_pos.push_back(gpgga[0].mean);

		std::cout << "Complete setMeasurement: " << now_step << std::endl;

		now_step++;

	}

	void readMeasurement1(){


		clock_t lap1 = clock();
		std::string filename;
		filename = IFPATH_MEAS + "lrf_lower/lrf_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
		std::vector<Polar<>> scan_l;
		std::ifstream ifs_lrf_l(filename);
		if (ifs_lrf_l.fail()) {
			std::cout << filename << " dose not exist" << std::endl;
			read_all_measurement_ = true;
		}
		ifs_lrf_l >> scan_l;
		all_meas_lrf_l.push_back(scan_l);

		clock_t lap2 = clock();
		//filename = IFPATH_MEAS + "img/img_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.bmp";
		//cv::Mat img;
		//img = cv::imread(filename, CV_LOAD_IMAGE_GRAYSCALE);
		//if (img.empty()) {
		//	std::cout << filename << " dose not exist" << std::endl;
		//	read_all_measurement_=true;
		//}
		////all_meas_img.push_back(img);

		switch (OMNI_FEATURE)
		{
		case OMNI_FEATURE_SIFT:{
			clock_t lap3 = clock();
			filename = IFPATH_MEAS + "sift/keypoint/keypoint_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.yml";
			std::vector<cv::KeyPoint> keys;
			cv::FileStorage fs(filename, cv::FileStorage::READ);
			cv::FileNode fn = fs["keypoints"];
			cv::read(fn, keys);
			if (keys.empty()){
				std::cout << filename << " dose not exist" << std::endl;
				read_all_measurement_ = true;
			}
			all_meas_keypoints.push_back(keys);

			//std::vector<cv::KeyPoint> keys = setKeypoints(img);
			//if (keys.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	read_all_measurement_=true;
			//}
			//all_meas_keypoints.push_back(keys);

			clock_t lap4 = clock();

			filename = IFPATH_MEAS + "sift/descriptor/desc_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()){
				read_all_measurement_ = true;
			}
			clock_t lap4_5 = clock();
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
			cv::Mat descriptor;
			for (int i = 0; i < v.size(); i++){
				cv::Mat mat(v[i], true);
				mat = mat.t();
				descriptor.push_back(mat);
			}
			all_meas_desriptor.push_back(descriptor);

			//cv::Mat desc = setDescriptor(img, keys);
			//if (desc.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	read_all_measurement_=true;
			//}
			//all_meas_desriptor.push_back(desc);
			break;
		}
		case OMNI_FEATURE_SURF:{
			clock_t lap3 = clock();
			filename = IFPATH_MEAS + "surf/keypoint/keypoint_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.yml";
			std::vector<cv::KeyPoint> keys;
			cv::FileStorage fs(filename, cv::FileStorage::READ);
			cv::FileNode fn = fs["keypoints"];
			cv::read(fn, keys);
			if (keys.empty()){
				std::cout << filename << " dose not exist" << std::endl;
				read_all_measurement_ = true;
			}
			all_meas_keypoints.push_back(keys);

			//std::vector<cv::KeyPoint> keys = setKeypoints(img);
			//if (keys.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	read_all_measurement_=true;
			//}
			//all_meas_keypoints.push_back(keys);

			clock_t lap4 = clock();

			filename = IFPATH_MEAS + "surf/descriptor/desc_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
			std::ifstream ifs(filename);
			if (ifs.fail()){
				read_all_measurement_ = true;
			}
			clock_t lap4_5 = clock();
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
			cv::Mat descriptor;
			for (int i = 0; i < v.size(); i++){
				cv::Mat mat(v[i], true);
				mat = mat.t();
				descriptor.push_back(mat);
			}
			all_meas_desriptor.push_back(descriptor);

			//cv::Mat desc = setDescriptor(img, keys);
			//if (desc.empty()) {
			//	std::cout << filename << " dose not exist" << std::endl;
			//	read_all_measurement_=true;
			//}
			//all_meas_desriptor.push_back(desc);
			break;
		}
		default:
			std::cout << "OMNI_FEATURE: " << OMNI_FEATURE << std::endl;
			exit(0);
			break;
		}

		clock_t lap5 = clock();
		filename = IFPATH_MEAS + "gps/gps_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.txt";
		std::ifstream ifs_gps(filename);
		if (ifs_gps.fail()) {
			std::cout << filename << " dose not exist" << std::endl;
			read_all_measurement_ = true;
		}
		std::string str;
		ifs_gps >> str;
		all_meas_gps.push_back(str);

		clock_t lap6 = clock();
		filename = IFPATH_MEAS + "odometry/odometry_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
		std::ifstream ifs_odo(filename);
		if (ifs_odo.fail()) {
			std::cout << filename << " dose not exist" << std::endl;
			read_all_measurement_ = true;
		}
		Position<> odo;
		ifs_odo >> odo;
		odo.r *= M_PI / 1800.0;
		all_meas_odometry.push_back(odo);

		clock_t lap7 = clock();
		filename = IFPATH_MEAS + "time/time_no" + std::to_string(read_meas_no) + "_" + std::to_string(read_meas_step) + "th.csv";
		std::ifstream ifs_time(filename);
		if (ifs_time.fail()) {
			std::cout << filename << " dose not exist" << std::endl;
			read_all_measurement_ = true;
		}
		MyTime time;
		ifs_time >> time;
		all_meas_time.push_back(time);

		if (read_all_measurement_) {
			finish = true;
			return;
		}



		std::cout << "ReadMeasurement: " << read_meas_step << std::endl;
		//std::cout << lap1 << "," << lap2 << "," << lap3 << "," << lap4 << "," << lap4_5 << "," << lap4_7 << "," << lap5 << "," << lap6 << "," << lap7 << std::endl;

		read_meas_step += SKIP_STEPS;


		if (odometry2 == nullptr) {
			odometry2 = new Position<>;
		}
		*odometry2 = all_meas_odometry.front();
		esti_time = all_meas_time.front();

		for (int i = 0; i < LIKELIHOOD_THREAD; i++) {

			lid2_l[i].setMeasScan(all_meas_lrf_l.front());
			omni[i].setKeypoints(all_meas_keypoints.front());
			omni[i].setDescriptors(all_meas_desriptor.front());
			gpgga[i].setMeasSignal(all_meas_gps.front());
		}
		all_meas_gps_pos.push_back(gpgga[0].mean);

		std::cout << "Complete setMeasurement: " << now_step << std::endl;

		now_step++;

	}

	/* Movie Creater*/


	//
	void visualizeScanL(const std::vector<Polar<>>& scan, const Position<> esti_pos, cv::Mat& out, int step) {

		map_img_clone2 = map_img_upper.clone();

		std::vector<Polar<>> scan_role;
		for (const auto& s : scan) {
			if (s.r == 1)	continue;
			Polar<> p = s;
			p.theta += esti_pos.r;
			scan_role.push_back(p);
		}

		std::vector<Coor<>> scan_xy = polToCoor(scan_role);

		std::vector<Coor<>> scan_from_robot;
		for (const auto& s : scan_xy) {
			Coor<> coor;
			coor.x = s.x + esti_pos.x;
			coor.y = s.y + esti_pos.y;
			scan_from_robot.push_back(coor);
		}


		for (const auto&s : scan_from_robot) {
			cv::Point pixel = ToPixel(s, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			cv::circle(map_img_clone2, pixel, MEASUREMENT_DATA_VIDEO_SCAN_RADIUS, MEASUREMENT_DATA_VIDEO_SCAN_COLOR, -1);
		}

		/* 推定位置の出力*/
		cv::Point esti_pixel = ToPixel(esti_pos, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::circle(map_img_clone2, esti_pixel, IMAGE_ESTIPOSITION_RADIUS, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS, 8, 0);
		// 矢印描画
		double l_e = IMAGE_ESTI_ARROW_LENGTH*MAP_RES;
		Coor<> coor2_e;
		coor2_e.x = l_e*std::cos(esti_pos.r) + esti_pos.x;
		coor2_e.y = l_e*std::sin(esti_pos.r) + esti_pos.y;
		cv::Point p2_e = ToPixel(coor2_e, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		cv::line(map_img_clone2, esti_pixel, p2_e, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS);

		/* 真の位置のみ描画 */
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone2, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
			coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
			cv::Point p2 = ToPixel(coor2, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone2, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
		}
		else {
#if MODE_LINIER_INTERPOLATION
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], esti_time);
			Position<> tpos_tmp = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			tpos_tmp.r = true_position->at(all_stock_tidx[step] - 1).r;
			cv::Point true_pixel = ToPixel(tpos_tmp, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone2, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(tpos_tmp.r) + tpos_tmp.x;
			coor2.y = l*std::sin(tpos_tmp.r) + tpos_tmp.y;
			cv::Point p2 = ToPixel(coor2, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone2, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
#endif
		}


		//cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//cv::circle(map_img_clone2, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
		//// 矢印描画
		//double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
		//Coor<> coor2;
		//coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
		//coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
		//cv::Point p2 = ToPixel(coor2, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//cv::line(map_img_clone2, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);

		/*  画像の切り出し  */
		Coor<> upleft(true_position->at(all_stock_tidx[step]).x - CUT_MAP_RADIUS_X, true_position->at(all_stock_tidx[step]).y - CUT_MAP_RADIUS_Y);
		//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
		Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);

		cv::Point upleft_pix = ToPixel(upleft, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
		cv::Size rect_pix = ToPixelSize(rect, map_img_clone2, MAP_RES);

		if (upleft_pix.x < 0)	upleft_pix.x = 0;
		if (upleft_pix.y < 0)	upleft_pix.y = 0;

		if (upleft_pix.x >= map_img_clone2.cols - rect_pix.width)	upleft_pix.x = map_img_clone2.cols - rect_pix.width;
		if (upleft_pix.y >= map_img_clone2.rows - rect_pix.height)	upleft_pix.y = map_img_clone2.rows - rect_pix.height;

		map_img_clone2 = cv::Mat(map_img_clone2, cv::Rect(upleft_pix, rect_pix));

		cv::flip(map_img_clone2, map_img_clone2, 0);

		out = map_img_clone2.clone();



	}

	//
	void visualizeOmniImg(const std::vector<Position<>>& img_pos, const std::vector<double> img_sim, const Position<> esti_pos, cv::Mat& out, int step) {

		map_img_clone2 = map_img_color.clone();

		for (int i = 0; i < img_pos.size(); i++) {
			cv::Point pixel = ToPixel(img_pos[i], map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
			cv::circle(map_img_clone2, pixel, MEASUREMENT_DATA_VIDEO_IMG_POS_RADIUS, MEASUREMENT_DATA_VIDEO_IMG_POS_COLOR, -1);
			cv::putText(map_img_clone2, std::to_string(img_sim[i]), MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_POINT, MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_FONT, MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_SCALE, MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_COLOR, MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_THIN, CV_AA);
		}

		cv::Point esti_pixel = ToPixel(esti_pos, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::circle(map_img_clone2, esti_pixel, IMAGE_ESTIPOSITION_RADIUS, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS, 8, 0);
		// 矢印描画
		double l_e = IMAGE_ESTI_ARROW_LENGTH*MAP_RES;
		Coor<> coor2_e;
		coor2_e.x = l_e*std::cos(esti_pos.r) + esti_pos.x;
		coor2_e.y = l_e*std::sin(esti_pos.r) + esti_pos.y;
		cv::Point p2_e = ToPixel(coor2_e, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		cv::line(map_img_clone2, esti_pixel, p2_e, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS);

		/* 真の位置のみ描画 */
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone2, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
			coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
			cv::Point p2 = ToPixel(coor2, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone2, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
		}
		else {
#if MODE_LINIER_INTERPOLATION
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], esti_time);
			Position<> tpos_tmp = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			tpos_tmp.r = true_position->at(all_stock_tidx[step] - 1).r;
			cv::Point true_pixel = ToPixel(tpos_tmp, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone2, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(tpos_tmp.r) + tpos_tmp.x;
			coor2.y = l*std::sin(tpos_tmp.r) + tpos_tmp.y;
			cv::Point p2 = ToPixel(coor2, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone2, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
#endif
		}

		//cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//cv::circle(map_img_clone2, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
		//// 矢印描画
		//double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
		//Coor<> coor2;
		//coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
		//coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
		//cv::Point p2 = ToPixel(coor2, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//cv::line(map_img_clone2, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);

		/*  画像の切り出し  */
		Coor<> upleft(true_position->at(all_stock_tidx[step]).x - CUT_MAP_RADIUS_X, true_position->at(all_stock_tidx[step]).y - CUT_MAP_RADIUS_Y);
		//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
		Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);

		cv::Point upleft_pix = ToPixel(upleft, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
		cv::Size rect_pix = ToPixelSize(rect, map_img_clone2, MAP_RES);

		if (upleft_pix.x < 0)	upleft_pix.x = 0;
		if (upleft_pix.y < 0)	upleft_pix.y = 0;

		if (upleft_pix.x >= map_img_clone2.cols - rect_pix.width)	upleft_pix.x = map_img_clone2.cols - rect_pix.width;
		if (upleft_pix.y >= map_img_clone2.rows - rect_pix.height)	upleft_pix.y = map_img_clone2.rows - rect_pix.height;

		map_img_clone2 = cv::Mat(map_img_clone2, cv::Rect(upleft_pix, rect_pix));

		cv::flip(map_img_clone2, map_img_clone2, 0);

		out = map_img_clone2.clone();

	}

	//
	void visualizeGPS(const Position<> gps, const Position<> esti_pos, cv::Mat& out, int step) {
		map_img_clone2 = map_img_color.clone();
		cv::Point pix_gps = ToPixel(gps, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
		cv::circle(map_img_clone2, pix_gps, MEASUREMENT_DATA_VIDEO_GPS_RADIUS, MEASUREMENT_DATA_VIDEO_GPS_COLOR, -1);

		cv::Point esti_pixel = ToPixel(esti_pos, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::circle(map_img_clone2, esti_pixel, IMAGE_ESTIPOSITION_RADIUS, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS, 8, 0);
		// 矢印描画
		double l_e = IMAGE_ESTI_ARROW_LENGTH*MAP_RES;
		Coor<> coor2_e;
		coor2_e.x = l_e*std::cos(esti_pos.r) + esti_pos.x;
		coor2_e.y = l_e*std::sin(esti_pos.r) + esti_pos.y;
		cv::Point p2_e = ToPixel(coor2_e, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		cv::line(map_img_clone2, esti_pixel, p2_e, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS);

		/* 真の位置のみ描画 */
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone2, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
			coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
			cv::Point p2 = ToPixel(coor2, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone2, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
		}
		else {
#if MODE_LINIER_INTERPOLATION
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], esti_time);
			Position<> tpos_tmp = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			tpos_tmp.r = true_position->at(all_stock_tidx[step] - 1).r;
			cv::Point true_pixel = ToPixel(tpos_tmp, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone2, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(tpos_tmp.r) + tpos_tmp.x;
			coor2.y = l*std::sin(tpos_tmp.r) + tpos_tmp.y;
			cv::Point p2 = ToPixel(coor2, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone2, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
#endif
		}


		//cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//cv::circle(map_img_clone2, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
		//// 矢印描画
		//double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
		//Coor<> coor2;
		//coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
		//coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
		//cv::Point p2 = ToPixel(coor2, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//cv::line(map_img_clone2, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);

		/*  画像の切り出し  */
		Coor<> upleft(true_position->at(all_stock_tidx[step]).x - CUT_MAP_RADIUS_X, true_position->at(all_stock_tidx[step]).y - CUT_MAP_RADIUS_Y);
		//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
		Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);

		cv::Point upleft_pix = ToPixel(upleft, map_img_clone2, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
		cv::Size rect_pix = ToPixelSize(rect, map_img_clone2, MAP_RES);

		if (upleft_pix.x < 0)	upleft_pix.x = 0;
		if (upleft_pix.y < 0)	upleft_pix.y = 0;

		if (upleft_pix.x >= map_img_clone2.cols - rect_pix.width)	upleft_pix.x = map_img_clone2.cols - rect_pix.width;
		if (upleft_pix.y >= map_img_clone2.rows - rect_pix.height)	upleft_pix.y = map_img_clone2.rows - rect_pix.height;

		map_img_clone2 = cv::Mat(map_img_clone2, cv::Rect(upleft_pix, rect_pix));

		cv::flip(map_img_clone2, map_img_clone2, 0);

		out = map_img_clone2.clone();





	}

	/*  重み付きパーティクルを障害物地図上に表示  */
	void visualizeParticleWeight(const std::vector<double> &likelihood, int step)
	{
		cv::cvtColor(map_img_clone, map_img_clone, CV_BGR2HSV); // RGB→HSVに変換
		std::vector<int> up_idx = sortUpIdx(likelihood);		//	パーティクルを昇順にソート

		for (const auto &idx : up_idx)
		{
			cv::Point particle_pixel = ToPixel(*getParticles()[idx]->getState(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			if (likelihood[idx] != 0.0) {
				cv::Scalar color;
				//color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
				color = cv::Scalar(180, (int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
				if ((int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5) < 5){
					color = cv::Scalar(180, 5, 255);
				}
				cv::circle(map_img_clone, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
				// 矢印の描画
				//double l = likelihood[idx] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
				double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
				Coor<> coor2;
				coor2.x = l*std::cos(getParticles()[idx]->getState()->r) + getParticles()[idx]->getState()->x;
				coor2.y = l*std::sin(getParticles()[idx]->getState()->r) + getParticles()[idx]->getState()->y;
				cv::Point p2 = ToPixel(coor2, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
				cv::line(map_img_clone, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
			}
		}

		//for (const auto &idx : up_idx)
		//{
		//	cv::Point particle_pixel = ToPixel(all_particles[step][idx], map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//	if (likelihood[idx] != 0.0) {
		//		cv::Scalar color;
		//		//color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
		//		color = cv::Scalar(180, (int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
		//		cv::circle(map_img_clone, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
		//		// 矢印の描画
		//		//double l = likelihood[idx] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//		double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//		Coor<> coor2;
		//		coor2.x = l*std::cos(all_particles[step][idx].r) + all_particles[step][idx].x;
		//		coor2.y = l*std::sin(all_particles[step][idx].r) + all_particles[step][idx].y;
		//		cv::Point p2 = ToPixel(coor2, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//		cv::line(map_img_clone, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
		//	}
		//}

		//for (int th = 0; th < LIKELIHOOD_THREAD; th++){
		//	std::vector<std::vector<Position<>>>& all_particles_tmp = all_particles;
		//	cv::Mat& map_img_clone_tmp = map_img_clone;
		//	likelihood_threads[th] = std::thread([all_particles_tmp, &map_img_clone_tmp, likelihood, up_idx, step, th]{
		//		for (int i = th; i < up_idx.size(); i += LIKELIHOOD_THREAD)
		//		{
		//			cv::Point particle_pixel = ToPixel(all_particles_tmp[step][up_idx[i]], map_img_clone_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//			if (likelihood[up_idx[i]] != 0.0) {
		//				cv::Scalar color;
		//				//color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
		//				color = cv::Scalar(180, (int)(likelihood[up_idx[i]] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
		//				cv::circle(map_img_clone_tmp, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
		//				// 矢印の描画
		//				//double l = likelihood[idx] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//				double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//				Coor<> coor2;
		//				coor2.x = l*std::cos(all_particles_tmp[step][up_idx[i]].r) + all_particles_tmp[step][up_idx[i]].x;
		//				coor2.y = l*std::sin(all_particles_tmp[step][up_idx[i]].r) + all_particles_tmp[step][up_idx[i]].y;
		//				cv::Point p2 = ToPixel(coor2, map_img_clone_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//				cv::line(map_img_clone_tmp, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
		//			}
		//		}
		//	});
		//}
		//for (auto& thread : likelihood_threads){
		//	thread.join();
		//}


		cv::cvtColor(map_img_clone, map_img_clone, CV_HSV2BGR); // HSV→RGBに変換

		/* 真の位置のみ描画 */
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
			coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
			cv::Point p2 = ToPixel(coor2, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
		}
		else {
#if MODE_LINIER_INTERPOLATION
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], esti_time);
			Position<> tpos_tmp = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			tpos_tmp.r = true_position->at(all_stock_tidx[step] - 1).r;
			cv::Point true_pixel = ToPixel(tpos_tmp, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(tpos_tmp.r) + tpos_tmp.x;
			coor2.y = l*std::sin(tpos_tmp.r) + tpos_tmp.y;
			cv::Point p2 = ToPixel(coor2, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
#endif
		}
	}
	void visualizeParticleWeight(cv::Mat& img, const std::vector<double> &likelihood, int step)
	{
		cv::cvtColor(img, img, CV_BGR2HSV); // RGB→HSVに変換
		std::vector<int> up_idx = sortUpIdx(likelihood);		//	パーティクルを昇順にソート

		//for (const auto &idx : up_idx)
		//{
		//	cv::Point particle_pixel = ToPixel(*getParticles()[idx]->getState(), img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//	if (likelihood[idx] != 0.0) {
		//		cv::Scalar color;
		//		//color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
		//		color = cv::Scalar(180, (int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
		//		if ((int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5)<5){
		//			color = cv::Scalar(180, 5, 255);
		//		}
		//		cv::circle(img, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
		//		// 矢印の描画
		//		//double l = likelihood[idx] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//		double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//		Coor<> coor2;
		//		coor2.x = l*std::cos(getParticles()[idx]->getState()->r) + getParticles()[idx]->getState()->x;
		//		coor2.y = l*std::sin(getParticles()[idx]->getState()->r) + getParticles()[idx]->getState()->y;
		//		cv::Point p2 = ToPixel(coor2, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//		cv::line(img, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
		//	}
		//}

		for (const auto &idx : up_idx)
		{
			cv::Point particle_pixel = ToPixel(all_particles[step][idx], img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			if (likelihood[idx] != 0.0) {
				cv::Scalar color;
				//color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
				color = cv::Scalar(180, (int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
				cv::circle(img, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
				// 矢印の描画
				//double l = likelihood[idx] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
				double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
				Coor<> coor2;
				coor2.x = l*std::cos(all_particles[step][idx].r) + all_particles[step][idx].x;
				coor2.y = l*std::sin(all_particles[step][idx].r) + all_particles[step][idx].y;
				cv::Point p2 = ToPixel(coor2, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
				cv::line(img, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
			}
		}

		//for (int th = 0; th < LIKELIHOOD_THREAD; th++){
		//	std::vector<std::vector<Position<>>>& all_particles_tmp = all_particles;
		//	cv::Mat& img_tmp = img;
		//	likelihood_threads[th] = std::thread([all_particles_tmp, &img_tmp, likelihood, up_idx, step, th]{
		//		for (int i = th; i < up_idx.size(); i += LIKELIHOOD_THREAD)
		//		{
		//			cv::Point particle_pixel = ToPixel(all_particles_tmp[step][up_idx[i]], img_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//			if (likelihood[up_idx[i]] != 0.0) {
		//				cv::Scalar color;
		//				//color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
		//				color = cv::Scalar(180, (int)(likelihood[up_idx[i]] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
		//				cv::circle(img_tmp, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
		//				// 矢印の描画
		//				//double l = likelihood[idx] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//				double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//				Coor<> coor2;
		//				coor2.x = l*std::cos(all_particles_tmp[step][up_idx[i]].r) + all_particles_tmp[step][up_idx[i]].x;
		//				coor2.y = l*std::sin(all_particles_tmp[step][up_idx[i]].r) + all_particles_tmp[step][up_idx[i]].y;
		//				cv::Point p2 = ToPixel(coor2, img_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//				cv::line(img_tmp, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
		//			}
		//		}
		//	});
		//}
		//for (auto& thread : likelihood_threads){
		//	thread.join();
		//}


		cv::cvtColor(img, img, CV_HSV2BGR); // HSV→RGBに変換

		/* 真の位置のみ描画 */
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(img, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
			coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
			cv::Point p2 = ToPixel(coor2, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(img, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
		}
		else {
#if MODE_LINIER_INTERPOLATION
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], esti_time);
			Position<> tpos_tmp = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			tpos_tmp.r = true_position->at(all_stock_tidx[step] - 1).r;
			cv::Point true_pixel = ToPixel(tpos_tmp, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(img, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(tpos_tmp.r) + tpos_tmp.x;
			coor2.y = l*std::sin(tpos_tmp.r) + tpos_tmp.y;
			cv::Point p2 = ToPixel(coor2, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(img, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
#endif
		}
	}


	/*  重み付きパーティクルを障害物地図上に表示  */
	void visualizeParticleWeight(const std::vector<Position<>>& particle, const std::vector<double> &likelihood, int step)
	{

		cv::cvtColor(map_img_clone, map_img_clone, CV_BGR2HSV); // RGB→HSVに変換
		std::vector<int> up_idx = sortUpIdx(likelihood);		//	パーティクルを昇順にソート
		//cv::cvtColor(map_img_clone, map_img_clone, cv::COLOR_RGB2HSV);	// RGB→HSVに変換
		for (const auto &idx : up_idx)
		{
			cv::Point particle_pixel = ToPixel(particle[idx], map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			if (likelihood[idx] != 0.0) {
				cv::Scalar color;
				//color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
				color = cv::Scalar(180, (int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
				if ((int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5) < 5){
					color = cv::Scalar(180, 5, 255);
				}
				cv::circle(map_img_clone, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
				// 矢印の描画
				//double l = likelihood[idx] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
				double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
				Coor<> coor2;
				coor2.x = l*std::cos(particle[idx].r) + particle[idx].x;
				coor2.y = l*std::sin(particle[idx].r) + particle[idx].y;
				cv::Point p2 = ToPixel(coor2, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
				cv::line(map_img_clone, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
			}
		}

		//cv::cvtColor(map_img_clone, map_img_clone, CV_BGR2HSV); // RGB→HSVに変換
		//std::vector<int> up_idx = sortUpIdx(likelihood);		//	パーティクルを昇順にソート
		////cv::cvtColor(map_img_clone, map_img_clone, cv::COLOR_RGB2HSV);	// RGB→HSVに変換
		//for (int th = 0; th < LIKELIHOOD_THREAD; th++){
		//	cv::Mat& map_img_clone_tmp = map_img_clone;
		//	likelihood_threads[th] = std::thread([particle, &map_img_clone_tmp, likelihood, up_idx, step, th]{
		//		for (int i = th; i < up_idx.size(); i += LIKELIHOOD_THREAD){
		//			cv::Point particle_pixel = ToPixel(particle[up_idx[i]], map_img_clone_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//			if (likelihood[up_idx[i]] != 0.0) {
		//				cv::Scalar color;
		//				//color = cv::Scalar(150, 255 - (int)(likelihood[up_idx[i]] / max*255.0 + 0.5), 255);
		//				color = cv::Scalar(180, (int)(likelihood[up_idx[i]] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
		//				cv::circle(map_img_clone_tmp, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
		//				// 矢印の描画
		//				//double l = likelihood[up_idx[i]] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//				double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//				Coor<> coor2;
		//				coor2.x = l*std::cos(particle[up_idx[i]].r) + particle[up_idx[i]].x;
		//				coor2.y = l*std::sin(particle[up_idx[i]].r) + particle[up_idx[i]].y;
		//				cv::Point p2 = ToPixel(coor2, map_img_clone_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//				cv::line(map_img_clone_tmp, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
		//			}

		//		}
		//	});
		//}
		//for (auto& thread : likelihood_threads){
		//	thread.join();
		//}


		cv::cvtColor(map_img_clone, map_img_clone, CV_HSV2BGR); // HSV→RGBに変換

		/* 推定位置の出力*/
		cv::Point esti_pixel = ToPixel(estimated_position[step], map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::circle(map_img_clone, esti_pixel, IMAGE_ESTIPOSITION_RADIUS, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS, 8, 0);
		// 矢印描画
		double l_e = IMAGE_ESTI_ARROW_LENGTH*MAP_RES;
		Coor<> coor2_e;
		coor2_e.x = l_e*std::cos(estimated_position[step].r) + estimated_position[step].x;
		coor2_e.y = l_e*std::sin(estimated_position[step].r) + estimated_position[step].y;
		cv::Point p2_e = ToPixel(coor2_e, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		cv::line(map_img_clone, esti_pixel, p2_e, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS);

		/* 真の位置のみ描画 */
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
			coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
			cv::Point p2 = ToPixel(coor2, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
		}
		else {
#if MODE_LINIER_INTERPOLATION
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], esti_time);
			Position<> tpos_tmp = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			tpos_tmp.r = true_position->at(all_stock_tidx[step] - 1).r;
			cv::Point true_pixel = ToPixel(tpos_tmp, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(tpos_tmp.r) + tpos_tmp.x;
			coor2.y = l*std::sin(tpos_tmp.r) + tpos_tmp.y;
			cv::Point p2 = ToPixel(coor2, map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(map_img_clone, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
#endif
		}

		//cv::Point esti_pixel = ToPixel(estimated_position.back(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//cv::circle(map_img_clone, esti_pixel, 5, cv::Scalar(255, 0, 0), -1, 8, 0);

	}
	void visualizeParticleWeight(cv::Mat& img, const std::vector<Position<>>& particle, const std::vector<double> &likelihood, int step)
	{
		cv::cvtColor(img, img, CV_BGR2HSV); // RGB→HSVに変換
		std::vector<int> up_idx = sortUpIdx(likelihood);		//	パーティクルを昇順にソート
		//cv::cvtColor(img, img, cv::COLOR_RGB2HSV);	// RGB→HSVに変換
		for (const auto &idx : up_idx)
		{
			cv::Point particle_pixel = ToPixel(particle[idx], img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			if (likelihood[idx] != 0.0) {
				cv::Scalar color;
				//color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
				color = cv::Scalar(180, (int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
				if ((int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5) < 5){
					color = cv::Scalar(180, 5, 255);
				}
				cv::circle(img, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
				// 矢印の描画
				//double l = likelihood[idx] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
				double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
				Coor<> coor2;
				coor2.x = l*std::cos(particle[idx].r) + particle[idx].x;
				coor2.y = l*std::sin(particle[idx].r) + particle[idx].y;
				cv::Point p2 = ToPixel(coor2, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
				cv::line(img, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
			}
		}

		//cv::cvtColor(img, img, CV_BGR2HSV); // RGB→HSVに変換
		//std::vector<int> up_idx = sortUpIdx(likelihood);		//	パーティクルを昇順にソート
		////cv::cvtColor(img, img, cv::COLOR_RGB2HSV);	// RGB→HSVに変換
		//for (int th = 0; th < LIKELIHOOD_THREAD; th++){
		//	cv::Mat& img_tmp = img;
		//	likelihood_threads[th] = std::thread([particle, &img_tmp, likelihood, up_idx, step, th]{
		//		for (int i = th; i < up_idx.size(); i += LIKELIHOOD_THREAD){
		//			cv::Point particle_pixel = ToPixel(particle[up_idx[i]], img_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//			if (likelihood[up_idx[i]] != 0.0) {
		//				cv::Scalar color;
		//				//color = cv::Scalar(150, 255 - (int)(likelihood[up_idx[i]] / max*255.0 + 0.5), 255);
		//				color = cv::Scalar(180, (int)(likelihood[up_idx[i]] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
		//				cv::circle(img_tmp, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
		//				// 矢印の描画
		//				//double l = likelihood[up_idx[i]] * IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//				double l = IMAGE_PARTICLE_ARROW_LENGTH*MAP_RES;
		//				Coor<> coor2;
		//				coor2.x = l*std::cos(particle[up_idx[i]].r) + particle[up_idx[i]].x;
		//				coor2.y = l*std::sin(particle[up_idx[i]].r) + particle[up_idx[i]].y;
		//				cv::Point p2 = ToPixel(coor2, img_tmp, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		//				cv::line(img_tmp, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
		//			}

		//		}
		//	});
		//}
		//for (auto& thread : likelihood_threads){
		//	thread.join();
		//}


		cv::cvtColor(img, img, CV_HSV2BGR); // HSV→RGBに変換

#if VISUALIZE_PRE_ESTI
		/* 事前推定位置の出力*/
		if (!init){
			cv::Point esti_pixel = ToPixel(estimated_position[step-1], img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(img, esti_pixel, IMAGE_PRE_ESTIPOSITION_RADIUS, IMAGE_PRE_ESTIPOSITION_COLOR, IMAGE_PRE_ESTI_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l_e = IMAGE_PRE_ESTI_ARROW_LENGTH*MAP_RES;
			Coor<> coor2_e;
			coor2_e.x = l_e*std::cos(estimated_position[step-1].r) + estimated_position[step-1].x;
			coor2_e.y = l_e*std::sin(estimated_position[step-1].r) + estimated_position[step-1].y;
			cv::Point p2_e = ToPixel(coor2_e, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(img, esti_pixel, p2_e, IMAGE_PRE_ESTIPOSITION_COLOR, IMAGE_PRE_ESTI_ARROW_THICKNESS);
		}
#endif

		/* 推定位置の出力*/
		cv::Point esti_pixel = ToPixel(estimated_position[step], img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::circle(img, esti_pixel, IMAGE_ESTIPOSITION_RADIUS, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS, 8, 0);
		// 矢印描画
		double l_e = IMAGE_ESTI_ARROW_LENGTH*MAP_RES;
		Coor<> coor2_e;
		coor2_e.x = l_e*std::cos(estimated_position[step].r) + estimated_position[step].x;
		coor2_e.y = l_e*std::sin(estimated_position[step].r) + estimated_position[step].y;
		cv::Point p2_e = ToPixel(coor2_e, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		cv::line(img, esti_pixel, p2_e, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS);

		/* 真の位置のみ描画 */
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(img, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
			coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
			cv::Point p2 = ToPixel(coor2, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(img, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
		}
		else {
#if MODE_LINIER_INTERPOLATION
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], esti_time);
			Position<> tpos_tmp = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			tpos_tmp.r = true_position->at(all_stock_tidx[step] - 1).r;
			cv::Point true_pixel = ToPixel(tpos_tmp, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(img, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l = IMAGE_TPOS_ARROW_LENGTH*MAP_RES;
			Coor<> coor2;
			coor2.x = l*std::cos(tpos_tmp.r) + tpos_tmp.x;
			coor2.y = l*std::sin(tpos_tmp.r) + tpos_tmp.y;
			cv::Point p2 = ToPixel(coor2, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			cv::line(img, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
#endif
		}

		//cv::Point esti_pixel = ToPixel(estimated_position.back(), img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//cv::circle(img, esti_pixel, 5, cv::Scalar(255, 0, 0), -1, 8, 0);

	}
	void visualizeParticleWeightLarge(cv::Mat& img, const std::vector<Position<>>& particle, const std::vector<double> &likelihood, int step, int cols, int rows, double map_res, cv::Point upleft_pix)
	{
		cv::cvtColor(img, img, CV_BGR2HSV); // RGB→HSVに変換
		std::vector<int> up_idx = sortUpIdx(likelihood);		//	パーティクルを昇順にソート
		//cv::cvtColor(img, img, cv::COLOR_RGB2HSV);	// RGB→HSVに変換
		for (const auto &idx : up_idx)
		{
			//if (!onObject_(particle[idx])){
				cv::Point particle_pixel = ToPixel(particle[idx], cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
				particle_pixel -= upleft_pix;
				if (particle_pixel.x > 0 && particle_pixel.x < img.cols &&
					particle_pixel.y>0 && particle_pixel.y < img.rows){
					if (likelihood[idx] != 0.0) {
						cv::Scalar color;
						//color = cv::Scalar(150, 255 - (int)(likelihood[idx] / max*255.0 + 0.5), 255);
						color = cv::Scalar(180, (int)(likelihood[idx] / IMAGE_PARTICLE_MAX_LIKELIHOOD*255.0 + 0.5), 255);
						cv::circle(img, particle_pixel, IMAGE_PARTICLE_RADIUS, color, IMAGE_PARTICLE_ARROW_THICKNESS, 8, 0);	//	パーティクルの描画
						// 矢印の描画
						//double l = likelihood[idx] * IMAGE_PARTICLE_ARROW_LENGTH*map_res;
						double l = IMAGE_PARTICLE_ARROW_LENGTH*map_res;
						Coor<> coor2;
						coor2.x = l*std::cos(particle[idx].r) + particle[idx].x;
						coor2.y = l*std::sin(particle[idx].r) + particle[idx].y;
						cv::Point p2 = ToPixel(coor2, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
						p2 -= upleft_pix;
						cv::line(img, particle_pixel, p2, color, IMAGE_PARTICLE_ARROW_THICKNESS);
					}
				}
			//}
		}




		cv::cvtColor(img, img, CV_HSV2BGR); // HSV→RGBに変換

		/* 前ステップの推定位置の出力*/
#if VISUALIZE_PRE_ESTI
		if (!init){
			cv::Point esti_pixel = ToPixel(estimated_position[step - 1], cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			esti_pixel -= upleft_pix;
			cv::circle(img, esti_pixel, IMAGE_PRE_ESTIPOSITION_RADIUS, IMAGE_PRE_ESTIPOSITION_COLOR, IMAGE_PRE_ESTI_ARROW_THICKNESS, 8, 0);
			// 矢印描画
			double l_e = IMAGE_PRE_ESTI_ARROW_LENGTH*map_res;
			Coor<> coor2_e;
			coor2_e.x = l_e*std::cos(estimated_position[step - 1].r) + estimated_position[step - 1].x;
			coor2_e.y = l_e*std::sin(estimated_position[step - 1].r) + estimated_position[step - 1].y;
			cv::Point p2_e = ToPixel(coor2_e, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			p2_e -= upleft_pix;
			cv::line(img, esti_pixel, p2_e, IMAGE_PRE_ESTIPOSITION_COLOR, IMAGE_PRE_ESTI_ARROW_THICKNESS);
		}
#endif
		/* 推定位置の出力*/
		cv::Point esti_pixel = ToPixel(estimated_position[step], cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		esti_pixel -= upleft_pix;
		cv::circle(img, esti_pixel, IMAGE_ESTIPOSITION_RADIUS, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS, 8, 0);
		// 矢印描画
		double l_e = IMAGE_ESTI_ARROW_LENGTH*map_res;
		Coor<> coor2_e;
		coor2_e.x = l_e*std::cos(estimated_position[step].r) + estimated_position[step].x;
		coor2_e.y = l_e*std::sin(estimated_position[step].r) + estimated_position[step].y;
		cv::Point p2_e = ToPixel(coor2_e, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
		p2_e -= upleft_pix;
		cv::line(img, esti_pixel, p2_e, IMAGE_ESTIPOSITION_COLOR, IMAGE_ESTI_ARROW_THICKNESS);

		/* 真の位置のみ描画 */
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			cv::Point true_pixel = ToPixel(true_position->at(all_stock_tidx[step]), cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			true_pixel -= upleft_pix;
			if (true_pixel.x > 0 && true_pixel.x < img.cols &&
				true_pixel.y>0 && true_pixel.y < img.rows){
				cv::circle(img, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
				// 矢印描画
				double l = IMAGE_TPOS_ARROW_LENGTH*map_res;
				Coor<> coor2;
				coor2.x = l*std::cos(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).x;
				coor2.y = l*std::sin(true_position->at(all_stock_tidx[step]).r) + true_position->at(all_stock_tidx[step]).y;
				cv::Point p2 = ToPixel(coor2, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
				p2 -= upleft_pix;
				cv::line(img, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
			}
		}
		else {
#if MODE_LINIER_INTERPOLATION
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], result_time[step]);
			Position<> tpos_tmp = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			tpos_tmp.r = true_position->at(all_stock_tidx[step] - 1).r;
			cv::Point true_pixel = ToPixel(tpos_tmp, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			true_pixel -= upleft_pix;
			if (true_pixel.x > 0 && true_pixel.x < img.cols &&
				true_pixel.y>0 && true_pixel.y < img.rows){
				cv::circle(img, true_pixel, IMAGE_TRUEPOSITION_RADIUS, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS, 8, 0);
				// 矢印描画
				double l = IMAGE_TPOS_ARROW_LENGTH*map_res;
				Coor<> coor2;
				coor2.x = l*std::cos(tpos_tmp.r) + tpos_tmp.x;
				coor2.y = l*std::sin(tpos_tmp.r) + tpos_tmp.y;
				cv::Point p2 = ToPixel(coor2, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
				p2 -= upleft_pix;
				cv::line(img, true_pixel, p2, IMAGE_TRUEPOSITION_COLOR, IMAGE_TPOS_ARROW_THICKNESS);
			}
#endif
		}

		//cv::Point esti_pixel = ToPixel(estimated_position.back(), img, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//cv::circle(img, esti_pixel, 5, cv::Scalar(255, 0, 0), -1, 8, 0);

	}

	//
	/*  確率分布出力  */
	cv::Mat createWeightedParImg(const std::vector<Position<>>& particle, const std::vector<double>& likelihood, int step)
	{
		cv::Mat img = map_img_color.clone();
		visualizeParticleWeight(img, particle, likelihood, step);

		/*  画像の切り出し  */
		Coor<> upleft(true_position->at(tidx).x - CUT_MAP_RADIUS_X, true_position->at(tidx).y - CUT_MAP_RADIUS_Y);
		//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
		Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);

		cv::Point upleft_pix = ToPixel(upleft, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
		cv::Size rect_pix = ToPixelSize(rect, img, MAP_RES);

		if (upleft_pix.x < 0)	upleft_pix.x = 0;
		if (upleft_pix.y < 0)	upleft_pix.y = 0;

		if (upleft_pix.x >= img.cols - rect_pix.width)	upleft_pix.x = img.cols - rect_pix.width;
		if (upleft_pix.y >= img.rows - rect_pix.height)	upleft_pix.y = img.rows - rect_pix.height;

		img = cv::Mat(img, cv::Rect(upleft_pix, rect_pix));

		cv::flip(img, img, 0);

		return img;

	}
	cv::Mat createParImg(const std::vector<double>& likelihood, int step)
	{
		cv::Mat img = map_img_color.clone();



		visualizeParticleWeight(img, likelihood, step);

		/*  画像の切り出し  */
		Coor<> upleft(true_position->at(tidx).x - CUT_MAP_RADIUS_X, true_position->at(tidx).y - CUT_MAP_RADIUS_Y);
		//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
		Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);

		cv::Point upleft_pix = ToPixel(upleft, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
		cv::Size rect_pix = ToPixelSize(rect, img, MAP_RES);

		if (upleft_pix.x < 0)	upleft_pix.x = 0;
		if (upleft_pix.y < 0)	upleft_pix.y = 0;

		if (upleft_pix.x >= img.cols - rect_pix.width)	upleft_pix.x = img.cols - rect_pix.width;
		if (upleft_pix.y >= img.rows - rect_pix.height)	upleft_pix.y = img.rows - rect_pix.height;

		img = cv::Mat(img, cv::Rect(upleft_pix, rect_pix));

		cv::flip(img, img, 0);

		return img;

	}
	cv::Mat createParImgAfterResampling(int step)
	{
		cv::Mat img = map_img_color.clone();
		std::vector<double> likelihood = std::vector<double>(SAMPLE_SIZE, 1.0);
		visualizeParticleWeight(img, likelihood, step);

		/*  画像の切り出し  */
		Coor<> upleft(true_position->at(tidx).x - CUT_MAP_RADIUS_X, true_position->at(tidx).y - CUT_MAP_RADIUS_Y);
		//Coor<> upleft(estimated_position.back().x - CUT_MAP_RADIUS_X, estimated_position.back().y - CUT_MAP_RADIUS_Y);
		Coor<> rect(CUT_MAP_RADIUS_X*2.0, CUT_MAP_RADIUS_Y*2.0);

		cv::Point upleft_pix = ToPixel(upleft, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
		cv::Size rect_pix = ToPixelSize(rect, img, MAP_RES);

		if (upleft_pix.x < 0)	upleft_pix.x = 0;
		if (upleft_pix.y < 0)	upleft_pix.y = 0;

		if (upleft_pix.x >= img.cols - rect_pix.width)	upleft_pix.x = img.cols - rect_pix.width;
		if (upleft_pix.y >= img.rows - rect_pix.height)	upleft_pix.y = img.rows - rect_pix.height;

		img = cv::Mat(img, cv::Rect(upleft_pix, rect_pix));

		cv::flip(img, img, 0);

		return img;

	}
	cv::Mat createWeightedParImgLarge(const std::vector<Position<>>& particle, const std::vector<double>& likelihood, int step)
	{
		cv::Mat img = map_img_color.clone();

		Position<> tpos_tmp;
		/* 真の位置のみ描画 */
		if (true_time[all_stock_tidx[step]] == esti_time || all_stock_tidx[step] - 1 < 0) {
			tpos_tmp = true_position->at(all_stock_tidx[step]);
		}
		else {
			//std::cout << true_time[all_stock_tidx[step]] << "," << esti_time << std::endl;
			Position<> diff_tpos = true_position->at(all_stock_tidx[step]) - true_position->at(all_stock_tidx[step] - 1);
			double diff_ttime = MyTime::diff(true_time[all_stock_tidx[step] - 1], true_time[all_stock_tidx[step]]);
			double diff_etime = MyTime::diff(true_time[all_stock_tidx[step] - 1], result_time[step]);
			tpos_tmp = true_position->at(all_stock_tidx[step] - 1) + diff_tpos / diff_ttime*diff_etime;
			tpos_tmp.r = true_position->at(all_stock_tidx[step] - 1).r;
		}




		/*  画像の切り出し  */
		Coor<> upleft(tpos_tmp.x - CUT_LARGE_MAP_RADIUS_X, tpos_tmp.y - CUT_LARGE_MAP_RADIUS_Y);
		//Coor<> upleft(true_position->at(tidx).x - CUT_LARGE_MAP_RADIUS_X, true_position->at(tidx).y - CUT_LARGE_MAP_RADIUS_Y);
		//Coor<> upleft(estimated_position[step].x - CUT_LARGE_MAP_RADIUS_X, estimated_position[step].y - CUT_LARGE_MAP_RADIUS_Y);
		//Coor<> upleft(estimated_position.back().x - CUT_LARGE_MAP_RADIUS_X, estimated_position.back().y - CUT_LARGE_MAP_RADIUS_Y);
		Coor<> rect(CUT_LARGE_MAP_RADIUS_X*2.0, CUT_LARGE_MAP_RADIUS_Y*2.0);

		cv::Point upleft_pix = ToPixel(upleft, img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//
		cv::Size rect_pix = ToPixelSize(rect, img, MAP_RES);

		if (upleft_pix.x < 0)	upleft_pix.x = 0;
		if (upleft_pix.y < 0)	upleft_pix.y = 0;

		if (upleft_pix.x >= img.cols - rect_pix.width)	upleft_pix.x = img.cols - rect_pix.width;
		if (upleft_pix.y >= img.rows - rect_pix.height)	upleft_pix.y = img.rows - rect_pix.height;

		img = cv::Mat(img, cv::Rect(upleft_pix, rect_pix));

		double scale = CUT_MAP_RADIUS_X / CUT_LARGE_MAP_RADIUS_X;
		cv::resize(img, img, cv::Size(), scale, scale);
		upleft_pix.x *= scale;
		upleft_pix.y *= scale;
		int cols = map_img_color.cols*scale;
		int rows = map_img_color.rows*scale;

		double map_res = MAP_RES / scale;

		int radius_pix = IMAGE_PARTICLE_LARGE_GRAY_CIRCLE_RADIUS / map_res;

		/* 真の位置のみ描画 */
		cv::Point true_pixel = ToPixel(tpos_tmp, cols, rows, map_res, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		true_pixel -= upleft_pix;
		if (true_pixel.x > 0 && true_pixel.x < img.cols &&
			true_pixel.y>0 && true_pixel.y < img.rows){
			cv::circle(img, true_pixel, radius_pix, IMAGE_PARTICLE_LARGE_GRAY_CIRCLE_COLOR, IMAGE_PARTICLE_LARGE_GRAY_CIRCLE_THICHNESS);
		}


		// 真の位置を中心に円を描画
		visualizeParticleWeightLarge(img, particle, likelihood, step, cols, rows, map_res, upleft_pix);

		cv::flip(img, img, 0);

		return img;

	}


	/*  パーティクル動画にフレームを追加  */

	void addMeasurementVideo2() {
		cv::Mat img_lrf_l, img_lrf_u, img_omni, img_gps;
		visualizeScanL(all_meas_lrf_l.front(), estimated_position[movie_step], img_lrf_l, movie_step);
		visualizeOmniImg(all_omni_img_sim_pos.front(), all_omni_img_sim.front(), estimated_position[movie_step], img_omni, movie_step);
		visualizeGPS(all_meas_gps_pos.front(), estimated_position[movie_step], img_gps, movie_step);

		img_lrf_u = cv::Mat::zeros(cv::Size(img_lrf_l.cols, img_lrf_l.rows), CV_8UC3);

		cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		cv::putText(img_omni, "c" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		cv::putText(img_gps, "g" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);

		int w = img_lrf_u.cols;
		int h = img_lrf_u.rows;

		cv::Mat img(cv::Size(w*2.0, h*2.0), CV_8UC3, cv::Scalar(0, 0, 0));

		assert(img_lrf_l.channels() == img.channels());
		assert(img_lrf_u.channels() == img.channels());
		assert(img_omni.channels() == img.channels());
		assert(img_gps.channels() == img.channels());

		img_lrf_l.copyTo(img(cv::Rect(cv::Point(0, 0), cv::Size(w, h))));
		img_lrf_u.copyTo(img(cv::Rect(cv::Point(w, 0), cv::Size(w, h))));
		img_omni.copyTo(img(cv::Rect(cv::Point(0, h), cv::Size(w, h))));
		img_gps.copyTo(img(cv::Rect(cv::Point(w, h), cv::Size(w, h))));

		cv::resize(img, img, cv::Size(), MOVIE_SCALE_W, MOVIE_SCALE_H);

		measurement_data_video << img;

#if SHOW_MOVIES
		cv::imshow("Measurements", img);
		cv::waitKey(1);
#endif


	}
	void addFlameWeightedStatParImg2()
	{
		cv::Mat img_lrf_l, img_lrf_u, img_omni, img_gpgga, img_fusion;
		std::thread thread_lrf_l = std::thread([&]{
			img_lrf_l = createWeightedParImg(all_stat_particles[movie_step], all_stat_lid2_l_likelihood[movie_step], movie_step);
		});
		std::thread thread_omni = std::thread([&]{
			img_omni = createWeightedParImg(all_stat_particles[movie_step], all_stat_omni_likelihood[movie_step], movie_step);
		});
		std::thread thread_gpgga = std::thread([&]{
			img_gpgga = createWeightedParImg(all_stat_particles[movie_step], all_stat_gpgga_likelihood[movie_step], movie_step);
		});

		thread_lrf_l.join();
		thread_omni.join();
		thread_gpgga.join();
		img_lrf_u = cv::Mat::zeros(cv::Size(img_lrf_l.cols, img_lrf_l.rows), CV_8UC3);


		//cv::putText(img_lrf_u, "l_l" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_lrf_l, "l_u" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_omni, "c" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_gpgga, "g" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);

		if (use_sim_[movie_step][0] == true){
			cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		if (use_sim_[movie_step][1] == true){
			cv::putText(img_omni, "c" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_omni, "c" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		if (use_sim_[movie_step][2] == true){
			cv::putText(img_gpgga, "g" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_gpgga, "g" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}


		int w = img_lrf_l.cols;
		int h = img_lrf_l.rows;

		cv::Mat img(cv::Size(w*2.0, h*2.0), CV_8UC3, cv::Scalar(0, 0, 0));

		img_lrf_l.copyTo(img(cv::Rect(cv::Point(0, 0), cv::Size(w, h))));
		img_lrf_u.copyTo(img(cv::Rect(cv::Point(w, 0), cv::Size(w, h))));
		img_omni.copyTo(img(cv::Rect(cv::Point(0, h), cv::Size(w, h))));
		img_gpgga.copyTo(img(cv::Rect(cv::Point(w, h), cv::Size(w, h))));

		cv::resize(img, img, cv::Size(), MOVIE_SCALE_W, MOVIE_SCALE_H);

		//writeWeightedStatParAllImg(img);

		weighted_stat_particle_video << img;

#if SHOW_MOVIES
		cv::imshow("Stat Particles", img);
		cv::waitKey(1);
#endif


	}
	void addFlameParticleVideo2()
	{
		cv::Mat img_lrf_l, img_lrf_u, img_omni, img_gpgga, img_fusion;
		std::thread thread_lrf_l = std::thread([&]{
			img_lrf_l = createWeightedParImg(all_particles[movie_step], all_lid2_l_likelihood[movie_step], movie_step);
		});
		std::thread thread_omni = std::thread([&]{
			img_omni = createWeightedParImg(all_particles[movie_step], all_omni_likelihood[movie_step], movie_step);
		});
		std::thread thread_gpgga = std::thread([&]{
			img_gpgga = createWeightedParImg(all_particles[movie_step], all_gpgga_likelihood[movie_step], movie_step);
		});
		std::thread thread_fusion = std::thread([&]{
			img_fusion = createWeightedParImg(all_particles[movie_step], all_fusion_likelihood[movie_step], movie_step);
		});

		thread_lrf_l.join();
		thread_omni.join();
		thread_gpgga.join();
		thread_fusion.join();
		img_lrf_u = cv::Mat::zeros(cv::Size(img_lrf_l.cols, img_lrf_l.rows), CV_8UC3);


		//cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_lrf_l, "l_u" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_omni, "c" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_gpgga, "g" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_fusion, "Fu" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);

		if (use_sim_[movie_step][0] == true){
			cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		if (use_sim_[movie_step][1] == true){
			cv::putText(img_omni, "c" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_omni, "c" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		if (use_sim_[movie_step][2] == true){
			cv::putText(img_gpgga, "g" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_gpgga, "g" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		if (std::find(use_sim_[movie_step].begin(), use_sim_[movie_step].end(), true) == use_sim_[movie_step].end()) {
			cv::putText(img_fusion, "Fu" + result_time[movie_step].str() + " Use All", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_fusion, "Fu" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}

		VisualizeAfterorBeforeResampling vabr = VISUALIZE_AFTER_OR_BEFORE_RESAMPLING;
		std::vector<double> likelihood;
		cv::Mat img_resampling;
		switch (vabr)
		{
		case AFTER_RESAMPLING:
			likelihood = std::vector<double>(SAMPLE_SIZE, 1.0);
			img_resampling = createWeightedParImg(all_particles_after_resampling[movie_step], likelihood, movie_step);
			cv::putText(img_resampling, "Re" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			break;
		case BEFORE_RESAMPLING:
			likelihood = std::vector<double>(SAMPLE_SIZE, 1.0);
			img_resampling = createWeightedParImg(all_particles[movie_step], likelihood, movie_step);
			cv::putText(img_resampling, "All" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			break;
		default:
			std::cout << "NON SAMPLING VISUALIZE_AFTER_OR_BEFORE_RESAMPLING: " << vabr << std::endl;
			exit(0);
			break;
		}


		int w = img_lrf_u.cols;
		int h = img_lrf_u.rows;

		cv::Mat img(cv::Size(w*3.0, h*2.0), CV_8UC3, cv::Scalar(0, 0, 0));

		img_lrf_l.copyTo(img(cv::Rect(cv::Point(0, 0), cv::Size(w, h))));
		img_lrf_u.copyTo(img(cv::Rect(cv::Point(w, 0), cv::Size(w, h))));
		img_omni.copyTo(img(cv::Rect(cv::Point(0, h), cv::Size(w, h))));
		img_gpgga.copyTo(img(cv::Rect(cv::Point(w, h), cv::Size(w, h))));
		img_fusion.copyTo(img(cv::Rect(cv::Point(2 * w, 0), cv::Size(w, h))));
		img_resampling.copyTo(img(cv::Rect(cv::Point(2 * w, h), cv::Size(w, h))));

		cv::resize(img, img, cv::Size(), MOVIE_SCALE_W, MOVIE_SCALE_H);

		particle_video << img;

#if SHOW_MOVIES
		cv::imshow("Particles", img);
		cv::waitKey(1);
#endif


	}
	void addFlameParticleLargeVideo2()
	{

		cv::Mat img_lrf_l= createWeightedParImgLarge(all_particles[movie_step], all_lid2_l_likelihood[movie_step], movie_step);
		cv::Mat img_lrf_u = cv::Mat(img_lrf_l.rows, img_lrf_l.cols, CV_8UC3, cv::Scalar(255, 255, 255));
		cv::Mat img_omni = createWeightedParImgLarge(all_particles[movie_step], all_omni_likelihood[movie_step], movie_step);
		cv::Mat img_gpgga = createWeightedParImgLarge(all_particles[movie_step], all_gpgga_likelihood[movie_step], movie_step);
		cv::Mat img_fusion = createWeightedParImgLarge(all_particles[movie_step], all_fusion_likelihood[movie_step], movie_step);

		//cv::putText(img_lrf_u, "l_l" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_lrf_l, "l_u" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_omni, "c" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_gpgga, "g" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		//cv::putText(img_fusion, "Fu" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);

		if (use_sim_[movie_step][0] == true){
			cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_lrf_l, "l_l" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		if (use_sim_[movie_step][1] == true){
			cv::putText(img_omni, "c" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_omni, "c" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		if (use_sim_[movie_step][2] == true){
			cv::putText(img_gpgga, "g" + result_time[movie_step].str() + " Use", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 255, 0), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_gpgga, "g" + result_time[movie_step].str() + " Eli", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, cv::Scalar(0, 0, 255), WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		if (std::find(use_sim_[movie_step].begin(), use_sim_[movie_step].end(), true) == use_sim_[movie_step].end()) {
			cv::putText(img_fusion, "Fu" + result_time[movie_step].str() + " Use All", WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}
		else{
			cv::putText(img_fusion, "Fu" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
		}



		VisualizeAfterorBeforeResampling vabr = VISUALIZE_AFTER_OR_BEFORE_RESAMPLING;
		std::vector<double> likelihood;
		cv::Mat img_resampling;
		switch (vabr)
		{
		case AFTER_RESAMPLING:
			likelihood = std::vector<double>(SAMPLE_SIZE, 1.0);
			img_resampling = createWeightedParImgLarge(all_particles_after_resampling[movie_step], likelihood, movie_step);
			cv::putText(img_resampling, "Re" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			break;
		case BEFORE_RESAMPLING:
			likelihood = std::vector<double>(SAMPLE_SIZE, 1.0);
			img_resampling = createWeightedParImgLarge(all_particles[movie_step], likelihood, movie_step);
			cv::putText(img_resampling, "All" + result_time[movie_step].str(), WEIGHTED_STAT_PAR_IMG_TEXT_POINT, WEIGHTED_STAT_PAR_IMG_TEXT_FONT, WEIGHTED_STAT_PAR_IMG_TEXT_SCALE, WEIGHTED_STAT_PAR_IMG_TEXT_COLOR, WEIGHTED_STAT_PAR_IMG_TEXT_THIN, CV_AA);
			break;
		default:
			std::cout << "NON SAMPLING VISUALIZE_AFTER_OR_BEFORE_RESAMPLING: " << vabr << std::endl;
			exit(0);
			break;
		}


		int w = img_lrf_u.cols;
		int h = img_lrf_u.rows;

		cv::Mat img(cv::Size(w*3.0, h*2.0), CV_8UC3, cv::Scalar(0, 0, 0));

		img_lrf_l.copyTo(img(cv::Rect(cv::Point(0, 0), cv::Size(w, h))));
		img_lrf_u.copyTo(img(cv::Rect(cv::Point(w, 0), cv::Size(w, h))));
		img_omni.copyTo(img(cv::Rect(cv::Point(0, h), cv::Size(w, h))));
		img_gpgga.copyTo(img(cv::Rect(cv::Point(w, h), cv::Size(w, h))));
		img_fusion.copyTo(img(cv::Rect(cv::Point(2 * w, 0), cv::Size(w, h))));
		img_resampling.copyTo(img(cv::Rect(cv::Point(2 * w, h), cv::Size(w, h))));

		cv::resize(img, img, cv::Size(), MOVIE_SCALE_W, MOVIE_SCALE_H);

		particle_large_video << img;

#if SHOW_MOVIES
		cv::imshow("ParticlesLarge", img);
		cv::waitKey(1);
#endif


	}


	//
	inline int minAllMeasurementData() {
		int min = std::min({
			all_meas_odometry.size(), all_meas_time.size(), all_meas_lrf_l.size(),
			all_meas_keypoints.size(), all_meas_desriptor.size(), all_meas_gps.size(), all_meas_gps_pos.size(), estimated_position.size(),
			all_omni_img_sim_pos.size(), all_omni_img_sim.size(),
			all_stock_tidx.size(), diff_time_ini_now.size(),
		});

		return min;

	}

	//
	inline int minAllParticleLikelihood() {
		int min = std::min({
			estimated_position.size(),
			all_particles.size(), all_lid2_l_likelihood.size(), all_omni_likelihood.size(),
			all_gpgga_likelihood.size(), all_fusion_likelihood.size(),
			all_stock_tidx.size(), diff_time_ini_now.size(),
		});

		return min;
	}


	//
	inline int minAllDataSize() {
		int min = std::min({
			all_meas_odometry.size(), all_meas_time.size(), all_meas_lrf_l.size(), 
			all_meas_keypoints.size(), all_meas_desriptor.size(), all_meas_gps.size(), all_meas_gps_pos.size(), estimated_position.size(),
			all_particles.size(), all_stat_particles.size(), all_lid2_l_likelihood.size(), all_omni_likelihood.size(),
			all_gpgga_likelihood.size(), all_fusion_likelihood.size(), all_stat_lid2_l_likelihood.size(),
			all_stat_omni_likelihood.size(), all_stat_gpgga_likelihood.size(),
			all_omni_img_sim_pos.size(), all_omni_img_sim.size(),
			all_stock_tidx.size(), diff_time_ini_now.size(),
		});

		return min;
	}





public:

	/*  パーティクルを初期化  */
	//	sample_size数のパーティクルを生成
	//	ini_positionの半径sample_range/2の範囲にパーティクルを散布
	void initPF()
	{
		getParticles().clear();

		Position<>	lowerbound = ini_position - ini_sample_radius;
		Position<>	upperbound = ini_position + ini_sample_radius;

		std::cout << lowerbound << std::endl;
		std::cout << upperbound << std::endl;

		std::random_device rnd;     // 非決定的な乱数生成器を生成
		std::mt19937_64 mt(rnd());     //  メルセンヌ・ツイスタの64ビット版、引数は初期シード値
		std::uniform_real_distribution<double> noise_x(lowerbound.x, upperbound.x);	//パーティクルの散布範囲
		std::uniform_real_distribution<double> noise_y(lowerbound.y, upperbound.y);	//パーティクルの散布範囲
		std::uniform_real_distribution<double> noise_r(lowerbound.r, upperbound.r);	//パーティクルの散布範囲

		for (int si = 0; si < sample_size; si++)
		{
			Particle<Position<>>* par = new Particle<Position<>>;	//	パーティクルの領域確保
			Position<>* position = new Position<>;	//	Stateの領域確保

			while (1)
			{
				position->set(noise_x(mt), noise_y(mt), noise_r(mt));	//	指定された範囲にパーティクルを散布
				//	存在可能領域に散布された場合，break
				cv::Point pixel = ToPixel(*position, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
				if ((int)map_img.at<unsigned char>(pixel) != 0)	break;
			}

			par->setState(position);
			getParticles().push_back(par);	//	パーティクルを追加
		}

		getParticles().shrink_to_fit();	//	領域を最適化

		std::cout << "Complete 'Initialization' " << std::endl;
	};

	void sampling(Position<> center) {

		getParticles().clear();

		Position<>	lowerbound = center - ini_sample_radius;
		Position<>	upperbound = center + ini_sample_radius;

		std::random_device rnd;     // 非決定的な乱数生成器を生成
		std::mt19937_64 mt(rnd());     //  メルセンヌ・ツイスタの64ビット版、引数は初期シード値
		std::uniform_real_distribution<double> noise_x(lowerbound.x, upperbound.x);	//パーティクルの散布範囲
		std::uniform_real_distribution<double> noise_y(lowerbound.y, upperbound.y);	//パーティクルの散布範囲
		std::uniform_real_distribution<double> noise_r(lowerbound.r, upperbound.r);	//パーティクルの散布範囲

		for (int si = 0; si < sample_size; si++)
		{
			Particle<Position<>>* par = new Particle<Position<>>;	//	パーティクルの領域確保
			Position<>* position = new Position<>;	//	Stateの領域確保

			while (1)
			{
				position->set(noise_x(mt), noise_y(mt), noise_r(mt));	//	指定された範囲にパーティクルを散布
				//	存在可能領域に散布された場合，break
				cv::Point pixel = ToPixel(*position, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
				if ((int)map_img.at<unsigned char>(pixel) != 0)	break;
			}

			par->setState(position);
			getParticles().push_back(par);	//	パーティクルを追加
		}

		getParticles().shrink_to_fit();	//	領域を最適化

		std::cout << "Complete 'Sampling' " << std::endl;

	}



	/*  推定位置を算出  */
	//	それぞれのパラメータの重み付き平均値
	void calcEPos()
	{
		Position<> epos(0.0, 0.0, 0.0);

		for (const auto& par : getParticles())
		{
			epos += *(par->getState())*par->getWeight();
		}

		estimated_position.push_back(epos);
		std::cout << "Complete calcEpos" << std::endl;
	}
	void calcError() {

		while (esti_time > true_time[tidx]) {
			tidx++;
		}

		std::cout << "esti_time: " << esti_time << ", true_time: " << true_time[tidx] << std::endl;
		if (esti_time != true_time[tidx]){
			std::cout << "Skip calcError" << std::endl;
			return;
		}
		Position<> error = true_position->at(tidx) - estimated_position.back();
		all_error.push_back(error);
		error_time.push_back(true_time[tidx]);

		std::cout << "Time: " << true_time[tidx] << ", Error: " << error << std::endl;

		if (tidx + 1 >= true_position->size()) {
			fin = true;
		}
		std::cout << "Complete calcError" << std::endl;
	}

	Position<> maxParameters(std::vector<Particle<Position<>>*> particles) {
		Position<> max = *particles.front()->getState();
		for (int i = 1; i < particles.size(); i++) {
			if (max.x < particles[i]->getState()->x) {
				max.x = particles[i]->getState()->x;
			}
			if (max.y < particles[i]->getState()->y) {
				max.y = particles[i]->getState()->y;
			}
			if (max.r < particles[i]->getState()->r) {
				max.r = particles[i]->getState()->r;
			}
		}
		return max;
	}

	Position<> minParameters(std::vector<Particle<Position<>>*> particles) {
		Position<> min = *particles.front()->getState();
		for (int i = 1; i < particles.size(); i++) {
			if (min.x > particles[i]->getState()->x) {
				min.x = particles[i]->getState()->x;
			}
			if (min.y > particles[i]->getState()->y) {
				min.y = particles[i]->getState()->y;
			}
			if (min.r > particles[i]->getState()->r) {
				min.r = particles[i]->getState()->r;
			}
		}
		return min;
	}


	/*  パーティクルが障害物map上で存在不可能領域にあるときtrue  */
	template <typename T> bool onObject_(T tmp)
	{
		cv::Point pixel = ToPixel(tmp, map_img, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	障害物地図上のパーティクル存在するピクセル

		if (pixel.x < 0 || pixel.x >= map_img.cols ||
			pixel.y < 0 || pixel.y >= map_img.rows ||
			(int)map_img.at<unsigned char>(pixel) != 255)	//	存在不可能領域の条件式
		{
			return true;
		}
		return false;
	}


	/**********************************************************/
	//	Grid Map 上に描画
	/**********************************************************/

protected:

	/*  重みなしパーティクルを障害物地図上に表示  */
	virtual void visualizeParticle()
	{
		cv::cvtColor(map_img_clone, map_img_clone, CV_BGR2HSV);
		for (const auto &particle : getParticles())
		{
			cv::Point particle_pixel = ToPixel(*particle->getState(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);	//	パーティクルピクセル
			if (particle_pixel.x<0 || particle_pixel.x>map_img.cols ||
				particle_pixel.y<0 || particle_pixel.y>map_img.rows ||
				map_img.at<unsigned char>(particle_pixel) == 0)
			{
				continue;
			}
			cv::Scalar color = cv::Scalar(180, (int)(0.015 / 0.02*255.0 + 0.5), 255);
			cv::circle(map_img_clone, particle_pixel, 2, color, -1, 8, 0);	//	パーティクルの描画
		}
		cv::cvtColor(map_img_clone, map_img_clone, CV_HSV2BGR); // 

	}


	/*  軌跡を描画  */
	void visualizeEstiRoute()
	{
		/*  推定位置の軌跡を描画  */
		cv::Point estimated_pixel1 = ToPixel(estimated_position.front(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::Point estimated_pixel2;
		for (int si = 0; si < estimated_position.size() - 1; si++)
		{
			estimated_pixel2 = ToPixel(estimated_position[si + 1], map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::line(map_img_clone, estimated_pixel1, estimated_pixel2, cv::Scalar(255, 0, 0), 2);
			estimated_pixel1 = estimated_pixel2;
		}

	};

	//void visualizeTrueRoute()
	//{
	//	/*  真の位置の軌跡を描画  */
	//	cv::Point true_pixel1 = ToPixel(true_position->front(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
	//	cv::Point true_pixel2;
	//	for (int si = 0; si < true_position->size() - 1; si++)
	//	//for (int si = 0; si < true_position->size() - 20; si++)
	//		{
	//		true_pixel2 = ToPixel(true_position->at(si + 1), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
	//		cv::circle(map_img_clone, true_pixel2, 3, cv::Scalar(0, 255, 0), -1, 8, 0);	//	パーティクルの描画
	//		//cv::line(map_img_clone, true_pixel1, true_pixel2, cv::Scalar(0, 255, 0), 2);
	//		true_pixel1 = true_pixel2;
	//	}
	//}

	/*  円を描画  */
	void visualizeCercle()
	{
		//for (int i = 0; i < estimated_position.size(); i = i + IMG_CIRCLE_STEP)
		//{
		//	cv::Point epix = ToPixel(estimated_position[i], map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		//	cv::circle(map_img_clone, epix, 5, cv::Scalar(0, 255, 0), -1);
		//}
	}

	/*  真の軌跡を描画  */
	void visualizeTrueRoute()
	{
		/*  真の位置の軌跡を描画  */
		cv::Point true_pixel1 = ToPixel(true_position->front(), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
		cv::Point true_pixel2;
		for (int si = TRUE_IDX_INI - 1; si < tidx; si++)
			//for (int si = 0; si < true_position->size()-1; si++)
		{
			true_pixel2 = ToPixel(true_position->at(si + 1), map_img_clone, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);
			cv::circle(map_img_clone, true_pixel2, 3, cv::Scalar(0, 255, 0), -1, 8, 0);	//	パーティクルの描画
			//cv::line(map_img_clone, true_pixel1, true_pixel2, cv::Scalar(0, 255, 0), 2);
			true_pixel1 = true_pixel2;
		}
	}

	/* 矢印の描画 */
	void cvArrow(cv::Mat img, cv::Point pt1, cv::Point pt2, cv::Scalar color, int thickness = 1, int lineType = 8, int shift = 0) {
		cv::line(img, pt1, pt2, color, thickness, lineType, shift);
		//float vx = (float)(pt2.x - pt1.x);
		//float vy = (float)(pt2.y - pt1.y);
		//float v = sqrt(vx*vx + vy*vy);
		//float ux = vx / v;
		//float uy = vy / v;
		////矢印の幅の部分
		//float w = 5, h = 10;
		//cv::Point ptl, ptr;
		//ptl.x = (int)((float)pt2.x - uy*w - ux*h);
		//ptl.y = (int)((float)pt2.y + ux*w - uy*h);
		//ptr.x = (int)((float)pt2.x + uy*w - ux*h);
		//ptr.y = (int)((float)pt2.y - ux*w - uy*h);
		////矢印の先端を描画する
		//cv::line(img, pt2, ptl, color, thickness, lineType, shift);
		//cv::line(img, pt2, ptr, color, thickness, lineType, shift);
	}

	/**********************************************************/
	//	ファイル出力
	/**********************************************************/

public:

	void writeParState(std::string OFPATH)
	{
		std::string filename = OFPATH + "Data/particle_state_last.csv";
		std::ofstream ofs_par(filename, std::ios_base::out);
		if (ofs_par.fail()) {
			writeError(filename);
		}
		for (const auto& par : getParticles())
		{
			ofs_par << *par->getState() << std::endl;
		}

		ofs_par.close();

		filename = OFPATH + "Data/size_of_esti_position.csv";
		std::ofstream ofs_step(filename, std::ios_base::out);
		ofs_step << std::to_string(estimated_position.size()) << std::endl;
		ofs_step.close();
	}

	void writeLastParState(std::string OFPATH) {
		std::string filename = OFPATH + "Data/particle_state_last.csv";
		std::ofstream ofs_par(filename, std::ios_base::out);
		if (ofs_par.fail()) {
			writeError(filename);
		}
		for (const auto& par : getParticles())
		{
			ofs_par << *par->getState() << std::endl;
		}

		ofs_par.close();

		filename = OFPATH + "Data/size_of_esti_position.csv";
		std::ofstream ofs_step(filename, std::ios_base::out);
		ofs_step << std::to_string(estimated_position.size()) << std::endl;
		ofs_step.close();
	}


	void writeSizeOfEstiPosition(std::string OFPATH) {
		std::string filename = OFPATH + "Data/size_of_esti_position.csv";
		std::ofstream ofs_step(filename, std::ios_base::out);
		ofs_step << std::to_string(estimated_position.size()) << std::endl;
		ofs_step.close();
	}

	/*  ロボットの軌跡を出力  */
	void writeRoute(std::string OFPATH)
	{
		map_img_clone = map_img_color.clone();	//	描画用のグリッドマップを初期化
		visualizeEstiRoute();	//	ロボットの通った軌跡を描画
		//visualizeCercle();
		visualizeTrueRoute();	//	ロボットの通った真の軌跡を描画
		cv::flip(map_img_clone, map_img_clone, 0);	//	上下反転

		/*  ファイル出力  */
		std::string filename = OFPATH + "Image/route.bmp";
		cv::imwrite(filename, map_img_clone);
		cv::imwrite("./output/route.bmp", map_img_clone);
	}

	/* 読み込みファイルナンバーの出力 */
	void writeLastInpugFileNumber(std::string OFPATH) {
		{
			std::string filename = OFPATH + "Data/last_input_file_number.csv";
			std::ofstream ofs(filename, std::ios_base::out);
			if (ofs.fail()) {
				writeError(filename);
			}
			ofs << "no,step" << std::endl;
			ofs << read_meas_no << "," << read_meas_step << std::endl;
			ofs.close();
		}
	}

	/*  推定位置をファイル出力  */
	void writeOutputEstiPosition(std::string OFPATH)
	{
		{
			std::string filename = OFPATH + "Data/estimated_position.csv";
			std::ofstream ofs_est(filename, std::ios_base::app);
			if (ofs_est.fail()) {
				writeError(filename);
			}
			ofs_est << esti_time << "," << MyTime::diff(ini_time, esti_time) << "," << estimated_position.back() << std::endl;
			ofs_est.close();
		}
		//{
		//	std::string filename = "./output/Data/estimated_position.csv";
		//	std::ofstream ofs_est(filename, std::ios_base::app);
		//	ofs_est << esti_time << "," << estimated_position.back() << "," << std::endl;
		//	ofs_est.close();
		//}
	}
	void writeOutputEstiPositionAll(std::string ofpath)
	{
		std::string filename = ofpath + "Data/estimated_position.csv";
		std::ofstream ofs_est(filename, std::ios_base::out);
		if (ofs_est.fail()) {
			writeError(filename);
		}
		if (estimated_position.size() != result_time.size() || estimated_position.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'estimated_position' are not same!" << std::endl;
			exit(0);
		}
		ofs_est << "step,time,s,x,y,rad" << std::endl;
		for (int i = 0; i < estimated_position.size(); i++) {
			ofs_est << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << estimated_position[i] << std::endl;
		}
		ofs_est.close();
		std::cout << "Complete Output Esti Position" << std::endl;
	}

	/*  */
	void writeUseSensor(std::string ofpath)
	{
		std::string filename = ofpath + "Data/use_sensor.csv";
		std::ofstream ofs(filename, std::ios_base::app);
		ofs << esti_time << "," << MyTime::diff(ini_time, esti_time) << "," << use_.back() << std::endl;
		ofs.close();
	}
	void writeUseSensorAll(std::string ofpath)
	{
		std::string filename = ofpath + "Data/use_sensor.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (use_.size() != result_time.size() || use_.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'use_' are not same!" << std::endl;
			exit(0);
		}
		ofs << "step,time,s,lrf_l,cmr,gps" << std::endl;
		for (int i = 0; i < use_.size(); i++) {
			ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << use_[i] << std::endl;
		}
		ofs.close();
		std::cout << "Complete Output Use Sensor" << std::endl;
		
		{
			std::string filename = ofpath + "Data/use_sensor_sim_.csv";
			std::ofstream ofs(filename, std::ios_base::out);
			if (use_.size() != result_time.size() || use_.size() != diff_time_ini_now.size()) {
				std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'use_' are not same!" << std::endl;
				exit(0);
			}
			ofs << "step,time,s,lrf_l,cmr,gps" << std::endl;
			for (int i = 0; i < use_.size(); i++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << use_sim_[i] << std::endl;
			}
			ofs.close();
			std::cout << "Complete Output Use Sensor" << std::endl;
		}

	}


	void writeEliminateSensorForAnalysis(std::string ofpath)
	{
		std::string filename = ofpath + "Data/eliminate_sensor_for_analysis.csv";
		std::ofstream ofs(filename, std::ios_base::app);
		ofs << esti_time << "," << MyTime::diff(ini_time, esti_time) << ",";
		for (int i = 0; i < SENSOR_NUM; i++) {
			if (use_.back()[i] == false) {
				ofs << std::to_string(SENSOR_NUM - i);
			}
			else {
				ofs << "-1";
			}
			if (i != SENSOR_NUM - 1) {
				ofs << ",";
			}
		}
		ofs << std::endl;
		ofs.close();
	}
	void writeEliminateSensorForAnalysisAll(std::string ofpath)
	{
		std::string filename = ofpath + "Data/eliminate_sensor_for_analysis.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		if (use_.size() != result_time.size() || use_.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'use_' are not same!" << std::endl;
			exit(0);
		}
		ofs << "step,time,s,lrf_l,cmr,gps" << std::endl;
		for (int i = 0; i < use_.size(); i++) {

			ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << ",";
			for (int j = 0; j < SENSOR_NUM; j++) {
				if (use_[i][j] == false) {
					ofs << std::to_string(SENSOR_NUM - j);
				}
				else {
					ofs << "-1";
				}
				if (j != SENSOR_NUM - 1) {
					ofs << ",";
				}
			}
			ofs << std::endl;
		}
		ofs.close();
		std::cout << "Complete writeEliminateSensorForAnalysisAll" << std::endl;

	}



	void writeSimilarityTable(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/similarity_table/similarity_table_" + std::to_string(estimated_position.size()) + "_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << esti_time << std::endl;
		ofs << similarity_table.back();
		ofs << std::endl;
		ofs.close();
	}
	void writeSimilarityTableAll(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/similarity_table.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		if (similarity_table.size() != result_time.size() || similarity_table.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'similarity_table' are not same!" << std::endl;
			exit(0);
		}
		ofs << "step,time,s,lrf_l,cmr,gps" << std::endl;
		for (int i = 0; i < similarity_table.size(); i++) {
			for (int j = 0; j < similarity_table[i].size(); j++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << similarity_table[i][j] << std::endl;
			}
		}
		ofs.close();
		std::cout << "Complete writeSimilarityTableAll" << std::endl;
	}

	void writeSimilarTable_(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/similar_table_/similar_table_" + std::to_string(estimated_position.size()) + +"_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << similar_table_.back();
		ofs << std::endl;
		ofs.close();
	}
	void writeSimilarTableAll_(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/similar_table_.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		if (similar_table_.size() != result_time.size() || similar_table_.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'similar_table_' are not same!" << std::endl;
			exit(0);
		}
		ofs << "step,time,s,lrf_l,cmr,gps" << std::endl;
		for (int i = 0; i < similar_table_.size(); i++) {
			for (int j = 0; j < similar_table_[i].size(); j++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << similar_table_[i][j] << std::endl;
			}
		}
		ofs.close();
		std::cout << "Complete writeSimilarTableAll_" << std::endl;
	}

	void writeOutputParticleAll(std::string ofpath) {
		std::string filename = ofpath + "Data/gitignore/particle.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		if (all_particles.size() != result_time.size() || all_particles.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'all_particles' are not same!" << std::endl;
			exit(0);
		}
		else if (all_particles.size() != all_fusion_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'fusion_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_particles.size() != all_lid2_l_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'lid2_l_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_particles.size() != all_omni_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'omni_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_particles.size() != all_gpgga_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'gpgga_likelihood' are not same!" << std::endl;
			exit(0);
		}

		ofs << "step,time,s,x,y,rad,fusion,lrf_l,cmr,gps" << std::endl;
		for (int i = 0; i < all_particles.size(); i++) {
			for (int j = 0; j < all_particles[i].size(); j++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << all_particles[i][j] << "," << all_fusion_likelihood[i][j] << "," << all_lid2_l_likelihood[i][j] << "," << all_omni_likelihood[i][j] << "," << all_gpgga_likelihood[i][j] << std::endl;
			}
		}
		ofs.close();
		std::cout << "Complete writeOutputParticleAll" << std::endl;
	}
	void writeOutputParticleAllAfterResampling(std::string ofpath) {
		std::string filename = ofpath + "Data/gitignore/particle_after_resampling.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		if (all_particles_after_resampling.size() != result_time.size() || all_particles_after_resampling.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'all_particles_after_resampling' are not same!" << std::endl;
			exit(0);
		}

		ofs << "step,time,s,x,y,rad" << std::endl;
		for (int i = 0; i < all_particles_after_resampling.size(); i++) {
			for (int j = 0; j < all_particles_after_resampling[i].size(); j++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << all_particles_after_resampling[i][j] << std::endl;
			}
		}
		ofs.close();
		std::cout << "Complete writeOutputParticleAllAfterResampling" << std::endl;
	}


	void writeOutputStatParticleAll(std::string ofpath) {
		std::string filename = ofpath + "Data/gitignore/stat_particle.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}

		if (all_particles.size() != result_time.size() || all_stat_particles.size() != diff_time_ini_now.size()) {
			std::cout << "Size of 'result_time', 'diff_time_ini_now' and 'all_particles' are not same!" << std::endl;
			exit(0);
		}
		else if (all_stat_particles.size() != all_stat_lid2_l_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'stat_lid2_l_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_stat_particles.size() != all_stat_omni_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'stat_omni_likelihood' are not same!" << std::endl;
			exit(0);
		}
		else if (all_stat_particles.size() != all_stat_gpgga_likelihood.size()) {
			std::cout << "Size of 'all_particles' and 'stat_gpgga_likelihood' are not same!" << std::endl;
			exit(0);
		}


		ofs << "step,time,s,x,y,rad,lrf_l,cmr,gps" << std::endl;
		for (int i = 0; i < all_stat_particles.size(); i++) {
			for (int j = 0; j < all_stat_particles[i].size(); j++) {
				ofs << i << "," << result_time[i] << "," << diff_time_ini_now[i] << "," << all_stat_particles[i][j] << "," << all_stat_lid2_l_likelihood[i][j] << "," << all_stat_omni_likelihood[i][j] << "," << all_stat_gpgga_likelihood[i][j] << std::endl;
			}
		}

		ofs.close();
		std::cout << "Complete writeOutputStatParticleAll" << std::endl;
	}

	void writeOutputParticleLrfL(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/particle/lrf_l/particle_" + std::to_string(estimated_position.size()) + "_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		ofs << "x,y,rad,weight" << std::endl;
		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			ofs << *getParticles()[i]->getState() << "," << lid2_l_likelihood[i] << std::endl;
		}
		ofs.close();
	}

	void writeOutputParticleCmr(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/particle/cmr/particle_" + std::to_string(estimated_position.size()) + "_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << "x,y,rad,weight" << std::endl;
		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			ofs << *getParticles()[i]->getState() << "," << omni_likelihood[i] << std::endl;
		}
		ofs.close();
	}

	void writeOutputParticleGps(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/particle/gps/particle_" + std::to_string(estimated_position.size()) + "_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << "x,y,rad,weight" << std::endl;
		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			ofs << *getParticles()[i]->getState() << "," << gpgga_likelihood[i] << std::endl;
		}
		ofs.close();
	}

	void writeOutputParticleFusion(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/particle/fusion/particle_" + std::to_string(estimated_position.size()) + "_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << "x,y,rad,weight" << std::endl;
		for (int i = 0; i < SAMPLE_SIZE; i++)
		{
			ofs << *getParticles()[i]->getState() << "," << fusion_likelihood[i] << std::endl;
		}
		ofs.close();
	}

	void writeOutputStatParticleLrfL(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/stat_particle/lrf_l/particle_" + std::to_string(estimated_position.size()) + "_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << "x,y,rad,weight" << std::endl;
		for (int i = 0; i < stat_particles.size(); i++)
		{
			ofs << stat_particles[i] << "," << stat_lid2_l_likelihood[i] << std::endl;
		}
		ofs.close();
	}

	void writeOutputStatParticleCmr(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/stat_particle/cmr/particle_" + std::to_string(estimated_position.size()) + "_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << "x,y,rad,weight" << std::endl;
		for (int i = 0; i < stat_particles.size(); i++)
		{
			ofs << stat_particles[i] << "," << stat_omni_likelihood[i] << std::endl;
		}
		ofs.close();
	}

	void writeOutputStatParticleGps(std::string ofpath)
	{
		std::string filename = ofpath + "Data/gitignore/stat_particle/gps/particle_" + std::to_string(estimated_position.size()) + "_" + esti_time.filenameStr() + ".csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) writeError(filename);
		ofs << "x,y,rad,weight" << std::endl;
		for (int i = 0; i < stat_particles.size(); i++)
		{
			ofs << stat_particles[i] << "," << stat_gpgga_likelihood[i] << std::endl;
		}
		ofs.close();
	}



	/*  推定誤差の出力  */
	void writeOutputEstiError(std::string OFPATH)
	{
		while (esti_time > true_time[tidx]) {
			tidx++;
		}

		std::cout << "esti_time: " << esti_time << ", true_time: " << true_time[tidx] << std::endl;
		if (esti_time != true_time[tidx])	return;

		Position<> error = true_position->at(tidx) - estimated_position.back();
		all_error.push_back(error);
		error_time.push_back(true_time[tidx]);

		if (tidx + 1 >= true_position->size()) {
			fin = true;
		}

	}
	void writeOutputEstiErrorAll(std::string OFPATH)
	{
		std::string filename = OFPATH + "Data/error.csv";
		std::ofstream ofs_est(filename, std::ios_base::out);
		if (ofs_est.fail()) {
			writeError(filename);
		}
		if (all_error.size() != error_time.size()) {
			std::cout << "Size of 'error_time' and 'all_error' are not same!" << std::endl;
			exit(0);
		}

		Position<> error_sum(0.0, 0.0, 0.0);
		int sum_num = 0;

		ofs_est << "time,x,y,rad,abs" << std::endl;
		for (int i = 0; i < all_error.size(); i++) {
			error_sum += all_error[i];
			sum_num++;
			if (i == all_error.size() || error_time[i] != error_time[i + 1]){
				double abs = std::sqrt(std::pow(error_sum.x, 2) + std::pow(error_sum.y, 2)) / sum_num;
				ofs_est << error_time[i] << "," << error_sum / sum_num << "," << abs << std::endl;
				error_sum = Position<>(0.0, 0.0, 0.0);
				sum_num = 0;
			}
		}
		ofs_est.close();
		std::cout << "Complete writeOutputEstiErrorAll" << std::endl;

	}

	void writeOutputThAll(std::string ofpath){
		std::string filename = ofpath + "Data/similarity_th.csv";
		std::ofstream ofs(filename, std::ios_base::out);
		if (ofs.fail()) {
			writeError(filename);
		}
		if (all_th.size() != result_time.size()) {
			std::cout << "Size of 'result_time' and 'all_th' are not same!" << std::endl;
			exit(0);
		}

		for (int i = 0; i < all_th.size(); i++){
			ofs << all_th[i] << std::endl;
		}
		std::cout << "Complete writeOutputThAll" << std::endl;


	}

	void setOutputValue(std::string ofpath) {
		result_time.push_back(esti_time);
		std::vector<Position<>> par;
		for (int i = 0; i < getParticles().size(); i++) {
			par.push_back(*getParticles()[i]->getState());
		}
		all_particles.push_back(par);
		all_stat_particles.push_back(stat_particles);
		all_lid2_l_likelihood.push_back(lid2_l_likelihood);
		all_omni_likelihood.push_back(omni_likelihood);
		all_gpgga_likelihood.push_back(gpgga_likelihood);
		all_fusion_likelihood.push_back(fusion_likelihood);
		all_stat_lid2_l_likelihood.push_back(stat_lid2_l_likelihood);
		all_stat_omni_likelihood.push_back(stat_omni_likelihood);
		all_stat_gpgga_likelihood.push_back(stat_gpgga_likelihood);
		all_stock_tidx.push_back(tidx);
		diff_time_ini_now.push_back(MyTime::diff(ini_time, esti_time));

		std::cout << "Complete 'setOutputValue' " << std::endl;

	}

	void setOutputAllParticlesAfterResampling(){
		std::vector<Position<>> par;
		for (int i = 0; i < getParticles().size(); i++) {
			par.push_back(*getParticles()[i]->getState());
		}
		all_particles_after_resampling.push_back(par);
		std::cout << "Complete setOutputAllParticlesAfterResampling" << std::endl;
	}



	/*  計測データの初期化  */
	void clearMeasurement()
	{
		for (int i = 0; i < LIKELIHOOD_THREAD; i++) {
			lid2_l[i].clearMeasurement();
			omni[i].clearMeasurement();
			gpgga[i].clearMeasurement();
		}
		all_meas_lrf_l.erase(all_meas_lrf_l.begin());
		all_meas_keypoints.erase(all_meas_keypoints.begin());
		all_meas_desriptor.erase(all_meas_desriptor.begin());
		all_meas_gps.erase(all_meas_gps.begin());
		all_meas_odometry.erase(all_meas_odometry.begin());
		all_meas_time.erase(all_meas_time.begin());
		LocalizationPF::swapOdometry();
		stat_particles.clear();
		stat_lid2_l_likelihood.clear();
		stat_omni_likelihood.clear();
		stat_gpgga_likelihood.clear();
		lid2_l_likelihood.clear();
		omni_likelihood.clear();
		gpgga_likelihood.clear();
		fusion_likelihood.clear();
	}




	/**********************************************************/
	//  メンバ変数
	/**********************************************************/

	TrialType trial_type;

	/*  環境データ  */
	GlobalToLocal gl2lc;							//	IniPosには地図の原点GL座標を代入
	cv::Mat map_img;							//  障害物地図（白黒）
	cv::Mat map_img_color;						//  障害物地図（カラー）
	cv::Mat map_img_clone;							//	描画用
	cv::Mat map_img_pointcloud;

	cv::Mat map_img_upper;
	cv::Mat map_img_clone2;
	cv::Mat map_img_clone3;

	/* Reliability Map */
	cv::Mat_<float> reliabity_map_lrf_l;
	cv::Mat_<float> reliabity_map_omni;
	cv::Mat_<float> reliabity_map_gps;


	bool finish = false;
	bool read_all_measurement_ = false;
	bool fin_read_env = false;

	int meas_step = FIRST_STEP - 1;
	int read_meas_no = FIRST_NO;
	int read_meas_step = FIRST_STEP;


	/* スレッド */
	std::thread set_environment_thread;
	std::thread read_measurement_thread;
	std::thread create_movies_thread;
	bool fin_movie_creator_ = false;


	/*  計測データ */
	Position<>* odometry1 = nullptr;				//	オドメトリデータ
	Position<>* odometry2 = nullptr;

	/* 計測データの保管 */
	std::vector<Position<>> all_meas_odometry;
	std::deque<MyTime> all_meas_time;
	std::vector<std::vector<Polar<>>> all_meas_lrf_l;
	std::vector<cv::Mat> all_meas_img;
	std::vector<std::vector<cv::KeyPoint>> all_meas_keypoints;
	std::vector<cv::Mat> all_meas_desriptor;
	std::vector<std::string> all_meas_gps;
	std::vector<Position<>> all_meas_gps_pos;
	std::vector<Position<>> estimated_position;		//	重み付き平均による推定位置
	std::vector<MyTime> result_time;
	std::vector<std::vector<Position<>>> all_particles;
	std::vector<std::vector<Position<>>> all_particles_after_resampling;
	std::vector<std::vector<Position<>>> all_stat_particles;
	std::vector<std::vector<double>> all_lid2_l_likelihood;
	std::vector<std::vector<double>> all_omni_likelihood;
	std::vector<std::vector<double>> all_gpgga_likelihood;
	std::vector<std::vector<double>> all_fusion_likelihood;
	std::vector<std::vector<double>> all_stat_lid2_l_likelihood;
	std::vector<std::vector<double>> all_stat_omni_likelihood;
	std::vector<std::vector<double>> all_stat_gpgga_likelihood;
	std::vector<std::vector<Position<>>> all_omni_img_sim_pos;
	std::vector<std::vector<double>> all_omni_img_sim;
	std::vector<double> all_th;
	std::vector<int> all_stock_tidx;
	std::vector<double> diff_time_ini_now;
	std::vector<Position<>> all_error;
	std::vector<MyTime> error_time;


	/*  ロボットの真の位置  */
	std::vector<Position<>>* true_position = nullptr;	//	真の位置
	std::vector<MyTime> true_time;
	int tidx = TRUE_IDX_INI - 1;	//	真の位置のindex
	int ini_tidx;	// 最初の真の位置
	Position<> ini_position;						//	自己位置推定のスタート（真の位置を与える）

	int movie_step = 0;
	/*  ロボットの推定位置  */

	MyTime esti_time;

	bool init = true;;

	/*  推定位置と真の位置の誤差  */
	//std::vector<Position<>> error;

	/*  パーティクルフィルタのparameter  */
	int sample_size;
	Position<> ini_sample_radius;					//	初期パーティクルのサンプル半径
	Position<> trans_par_sys_var;					//	パーティクル遷移時に誤差を付加
	Polar<> odometry_system_noise;					//	オドメトリ誤差分散

	/*  パーティクルの動画  */
	cv::VideoWriter particle_video;
	cv::VideoWriter weighted_stat_particle_video;
	cv::VideoWriter measurement_data_video;
	cv::VideoWriter particle_large_video;

	/*  現在のステップ  */
	int now_step = 0;
	int add_measurement_movie_step = 0;
	bool fin_add_measurement_movie_ = false;
	int ini_step;

	/*  何回目の計測か */
	int no = FIRST_NO;
	int ini_no;

	/* 出力用 */


	/* 位置推定開始時刻 */
	MyTime ini_time;

	std::vector<std::thread> likelihood_threads;

	/*  尤度格納  */
	std::vector<double> lid2_l_likelihood;
	std::vector<double> omni_likelihood;
	std::vector<double> gpgga_likelihood;
	std::vector<double> fusion_likelihood;

	/*  センサ統合関係  */
	std::vector<Position<>> stat_particles;
	Position<> stat_sample_radius;
	std::vector<double> stat_lid2_l_likelihood;
	std::vector<double> stat_omni_likelihood;
	std::vector<double> stat_gpgga_likelihood;

	/*  各センサのクラス  */
	std::vector<Lidar2d> lid2_l;
	std::vector<OmniCamera> omni;
	std::vector<Gps> gpgga;


	std::vector<std::vector<bool>> use_;							// 	各ステップで選択されたセンサ
	std::vector<std::vector<std::vector<double>>> similarity_table;	//	センサから得られる確率分布間の類似度
	std::vector<std::vector<std::vector<bool>>> similar_table_;		//	センサから得られる確率分布間を類似・非類似で閾値処理

	std::vector<std::vector<bool>> use_sim_;

	cv::Mat ParticleVideo;
	cv::Mat WeightedStatParImg;

	std::string ofpath_i;
};