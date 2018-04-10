
#pragma once

#include "myfun.h"
#include "parameter.h"
//#include "LocalizationPF.h"

/**********************************************************/
//  プリプロセッサ
/**********************************************************/

/*  尤度生成関連  */


class Lidar2d
{
public:
	/*  コンストラクタ  */
	Lidar2d() {};
	Lidar2d(const Lidar2d& obj)
	{
		(this->point_cloud) = (obj.point_cloud);
		(this->scan) = (obj.scan);

		/*  環境データ  */
		this->grid_map = obj.grid_map;								// 障害物地図

																	/*  計測データ  */
		this->scan_xy = obj.scan_xy;					//	scanデータを極座標からデカルト座標に変換

		this->scan_grid = obj.scan_grid;

	}
	Lidar2d& operator=(const Lidar2d& obj)
	{
		(this->point_cloud) = (obj.point_cloud);
		(this->scan) = (obj.scan);


		/*  環境データ  */
		cv::Mat mat = obj.grid_map.clone();
		this->grid_map = mat;

		/*  計測データ  */
		this->scan_xy = obj.scan_xy;					//	scanデータを極座標からデカルト座標に変換

														/*  尤度生成関連  */
		this->scan_grid = obj.scan_grid;

		return *this;
	}

	/*  デストラクタ  */
	~Lidar2d()
	{
		scan_grid.release();
	};


	/**********************************************************/
	//	ファイル読み込み
	/**********************************************************/

	/*  Lidar2D点群地図の読み込み  */
	void readPointCloud(std::string filename)
	{
		std::ifstream ifs(filename);
		if (ifs.fail())	readError(filename);							//	エラー出力
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

	/*  Lidar2Dスキャンデータの読み込み  */
	bool readMeasScan(int no, int step)
	{
		std::string filename = IFPATH_MEAS + "lrf/lrf_" + std::to_string(step) + "th_no1.csv";
		std::ifstream ifs(filename);
		if (ifs.fail())	return false;	//	読み込みに失敗した場合，falseを返す
		ifs >> scan;
		convertScan();
		return true;
	};

	/*  Lidar2Dスキャンデータの読み込み  */
	bool readMeasScan(std::string filename)
	{
		std::ifstream ifs(filename);
		if (ifs.fail())	return false;	//	読み込みに失敗した場合，falseを返す
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
		//	極座標からデカルト座標に変換
		for (const auto tmp : scan)
		{
			if (tmp.r == 1 || tmp.theta>M_PI*2.0 / 3.0 || tmp.theta<-M_PI*2.0 / 3.0)	continue;	//	lrfデータの異常値は1として出力されるため，排除，90度以上の点群も排除
			Coor<> coor = polToCoor(tmp);			//	極座標からデカルト座標に変換して初期化
			scan_xy.push_back(coor);
		}
	}

	//bool readMeasScan(int step)
	//{

	//	std::string filename = IFPATH_MEAS + "lrf/lrf_" + std::to_string(step) + "th_no1.csv";
	//	std::ifstream ifs(filename);
	//	if (ifs.fail())	return false;	//	読み込みに失敗した場合，falseを返す
	//	ifs >> scan;
	//	return true;
	//};


	/**********************************************************/
	//	ICP関連
	/**********************************************************/

	/*  ICPに点群地図をセット  */
	void setEnvICP()
	{
		icp.setMapData(point_cloud);	//	点群地図をset
		icp.setThreshold(ICP_PAIR_DST_TH);	//	対応点決定の閾値
	};

	/*  ICPにスキャンデータをセット  */
	void setMeasICP()
	{
		icp.setInputDataOrg(scan_xy);	//	計測データのset
	}

	/*  パーティクルの尤度算出  */
	template<typename T>
	double getLikelihoodICP(const Position<T> pos, float bai)
	{
		icp.transInputDataOrg(pos);	//	位置情報で平行移動
		icp();

		if (icp.getPairNum() < scan_xy.size()*bai) return 0.0;		//	対応点が閾値以下の場合，尤度は0.0

		double out = std::exp(-(double)icp.getError()*ICP_PARAMETER);
		return out;	//	尤度の算出
	}


	/**********************************************************/
	//	グリッドマップマッチング
	/**********************************************************/
	void setScanGrid()
	{
		/* 画像の生成 */
		size_t img_cols = LASER_DIST_RANGE / MAP_RES + 2;	// レーザ計測範囲から少し余裕を持たせる
		size_t img_rows = img_cols;

		cv::Mat img(cv::Size(img_cols * 2, img_rows * 2), CV_8U, cv::Scalar(128));	//	グレー画像を生成

																					/* レーザ計測可能範囲を黒塗り */
		cv::Point center = cv::Point(img_cols, img_rows);
		cv::Size laser_dist_radius(LASER_DIST_RANGE / MAP_RES, LASER_DIST_RANGE / MAP_RES);
		cv::ellipse(img, center, laser_dist_radius, 180, 55, 305, cv::Scalar(0), -1);

		/* レーザ透過範囲を白塗り */
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
				//	point.r = 30000;	// point.r=1のとき，ビームは反射しなかったことを意味するため，計測可能範囲の30000[mm]に設定
				//}
				//else
				//{
				//	continue;
				//}
				point.r = LASER_DIST_RANGE;	// point.r=1のとき，ビームは反射しなかったことを意味するため，計測可能範囲の30000[mm]に設定
			}
			//if (point.r == 1)	point.r = 30000;	// point.r=1のとき，ビームは反射しなかったことを意味するため，計測可能範囲の30000[mm]に設定
			Coor<> point_xy = polToCoor(point);
			cv::Point point_pix = ToPixel(point_xy, img, MAP_RES, 0.5, 0.5);
			cv::line(img, center, point_pix, cv::Scalar(255), 2);
		}

		scan_grid = img;		//	スキャンデータから障害物地図を生成

								//cv::imshow("img", img);
								//cv::waitKey();
	}

	template<typename T>
	double getLikelihoodGrid(const Position<T> pos)
	{
		double angle_deg = pos.r / M_PI*180.0;	//	スキャングリッド回転角度
		cv::Point center(scan_grid.cols, scan_grid.rows);

		/* 回転 */
		cv::Mat affine = cv::getRotationMatrix2D(center, angle_deg, 1.0);
		cv::Mat scan_grid_new;
		cv::warpAffine(scan_grid, scan_grid_new, affine, cv::Size(scan_grid.cols, scan_grid.rows), 1, 0, cv::Scalar(128));

		cv::Point pos_pix = ToPixel(pos, grid_map, MAP_RES, MAP_IMG_ORG_X, MAP_IMG_ORG_Y);

		int pix_num_nonequal = 0;	// 参照データと計測データでピクセルが一致しなかった数

		for (int sy = 0; sy < scan_grid_new.rows; sy++)
		{
			for (int sx = 0; sx < scan_grid_new.cols; sx++)
			{
				if ((int)scan_grid_new.at<unsigned char>(sx, sy) == 128)
				{
					continue;	// グレー領域はスキップ
				}

				cv::Point map_pix(-scan_grid_new.cols / 2 + sx + 1, -scan_grid_new.rows / 2 + sy + 1);
				map_pix = pos_pix + map_pix;

				if (scan_grid_new.at<unsigned char>(sx, sy) != grid_map.at<unsigned char>(map_pix))	pix_num_nonequal++;
			}
		}

		double out = std::exp(-(double)pix_num_nonequal*GRID_PARAMETER);

		//double out

		return out;	//	尤度の算出



		return out;	//	尤度の算出
	}



	/**********************************************************/
	//	計測データのclear
	/**********************************************************/

	void clearMeasurement()
	{
		scan.clear();
		scan_xy.clear();
		scan_grid.release();
	}


	/**********************************************************/
	//  メンバ変数
	/**********************************************************/

	/*  環境データ  */
	std::vector<Coor<>> point_cloud;		//  レーザスキャナ：点群地図
	cv::Mat grid_map;								// 障害物地図

													/*  計測データ  */
	std::vector<Polar<>> scan;			//  LRFスキャンデータ
	std::vector<Coor<>> scan_xy;					//	scanデータを極座標からデカルト座標に変換

													/*  尤度生成関連  */
	ICP icp;										//	ICPアルゴリズム
	cv::Mat scan_grid;


};