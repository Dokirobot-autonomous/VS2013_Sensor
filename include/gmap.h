#pragma once

#include "myfun.h"
#include "mycv.h"

/**********************************************************/
//	 グリッドマップ
/**********************************************************/
class GMap
{

	/*  変数  */
public:
	cv::Mat img;				// 地図画像
private:
	cv::Mat img_gray;	// 白黒画像
	double org_lat;		// 地図の原点の緯度
	double org_lon;		// 地図の原点の経度
	double org_ele;		// 地図の原点の高度（環境内ではこの値で一定と見なす）
	double org_head;		// 地図の原点の初期角度（北を0°として時計回りを正）
	double res;			// 地図画像の分解能[mm]
	double img_org_x;	// 地図画像の原点（正規化）
	double img_org_y;	// 地図画像の原点（正規化）


	/*  コンストラクタ  */
public:
	GMap(){};
	GMap(std::string filename, double org_lat, double org_lon, double org_ele, double org_head, double res, double img_org_x, double img_org_y)
		:org_lat(org_lat), org_lon(org_lon), org_ele(org_ele), org_head(org_head), res(res), img_org_x(img_org_x), img_org_y(img_org_y)
	{
		img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
		if (img.empty()) readError(filename);
		cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);
	};
	GMap(const GMap& obj)
	{
		img = obj.img.clone();
		img_gray = obj.img_gray.clone();
		org_lat = obj.org_lat;
		org_lon = obj.org_lon;
		org_ele = obj.org_ele;
		org_head = obj.org_head;
		res = obj.res;
		img_org_x = obj.img_org_x;
		img_org_y = obj.img_org_y;
	}
	//inline GMap& operator=(const GMap& obj){
	//	if (this == &obj){
	//		return *this;
	//	}
	//	img = obj.img.clone();
	//	img_gray = obj.img_gray.clone();
	//	org_lat = obj.org_lat;
	//	org_lon = obj.org_lon;
	//	org_ele = obj.org_ele;
	//	org_head = obj.org_head;
	//	res = obj.res;
	//	img_org_x = obj.img_org_x;
	//	img_org_y = obj.img_org_y;
	//	return *this;
	//}
	~GMap(){};

	/*  read image  */
	inline void read(std::string& filename)
	{
		img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
		cv::cvtColor(img, img_gray, cv::COLOR_RGB2GRAY);
	};

	/*  セッター*/
	inline void set(double org_lat, double org_lon, double org_ele, double org_head, double res, double img_org_x, double img_org_y)
	{
		this->org_lat = org_lat;
		this->org_lon = org_lon;
		this->org_ele = org_ele;
		this->org_head = org_head;
		this->res = res;
		this->img_org_x = img_org_x;
		this->img_org_y = img_org_y;
	}

	/*  クローンを出力  */
	GMap clone() const
	{
		GMap out;
		out.img = this->img.clone();
		out.img_gray = this->img_gray.clone();
		out.org_lat = this->org_lat;
		out.org_lon = this->org_lon;
		out.org_ele = this->org_ele;
		out.org_head = this->org_head;
		out.res = this->res;
		out.img_org_x = this->img_org_x;
		out.img_org_y = this->img_org_y;
		return out;
	}

	/*  障害物上にある場合，true  */
	template <class C> bool isOnObstacle(const C& obj)
	{
		cv::Point pixel = toPixel(*this, obj);	//	障害物地図上のパーティクル存在するピクセル

		if (pixel.x < 0 || pixel.x >= img_gray.cols ||
			pixel.y < 0 || pixel.y >= img_gray.rows ||
			(int)img_gray.at<unsigned char>(pixel) == 0)	//	存在不可能領域の条件式
		{
			return true;
		}
		return false;
	};

	/*  画像の切り出し  */
	template<typename T> void cutImg(Coor<T> center, Coor<T> radius)
	{
		Coor<T> upleft = center - radius;	//	画像の左端
		Coor<T> rect = radius*2.0;

		cv::Point upleft_pix = toPixel(*this, upleft);
		cv::Size rect_pix = toPixelSize(*this, rect);


		if (upleft_pix.x < 0)	upleft_pix.x = 0;
		if (upleft_pix.y < 0)	upleft_pix.y = 0;

		if (upleft_pix.x >=img.cols - rect_pix.width)	upleft_pix.x = img.cols - rect_pix.width;
		if (upleft_pix.y >=img.rows - rect_pix.height)	upleft_pix.y = img.rows - rect_pix.height;

		img = cv::Mat(img, cv::Rect(upleft_pix, rect_pix));
		img_gray = cv::Mat(img_gray, cv::Rect(upleft_pix, rect_pix));

	}

	/*  塗りつぶし円の描画(friend)  */
	template<class C> friend void drawCircle(GMap& gmap, const C& point, int radius, cv::Scalar color, int thickenss = -1);
	/* Xの描画(friend) */
	template<class C> friend void drawCross(GMap& gmap, const C& point, int radius, cv::Scalar color, int thickness);
	/*  線の描画(friend)  */
	template<class C> friend void drawLine(GMap& gmap, C point1, C point2, cv::Scalar color, int thickness = 2);
	/*  ピクセル出力(Point)(friend)  */
	template<class C> friend cv::Point toPixel(const GMap& gmap, const C& obj);
	/*  ピクセル出力(Size)(friend)  */
	template<class C> friend cv::Size toPixelSize(const GMap& gmap, const C& obj);

};

/*  塗りつぶし円の描画  */
template<class C>	void drawCircle(GMap& gmap, const C& point, int radius, cv::Scalar color, int thickness)
{
	cv::Point pix = toPixel(gmap, point);

	//if (pix.x<0 || pix.x >= gmap.img_gray.cols ||
	//	pix.y<0 || pix.y >= gmap.img_gray.rows ||
	//	gmap.img_gray.at<unsigned char>(pix) == 0)
	//{
	//	return;
	//}

	cv::circle(gmap.img, pix, radius, color, thickness, 8, 0);	//	パーティクルの描画

}

/*  線の描画  */
template<class C>	void drawLine(GMap& gmap, C point1, C point2, cv::Scalar color, int thickness)
{
	cv::Point pix1 = toPixel(gmap, point1);
	cv::Point pix2 = toPixel(gmap, point2);

	if (pix1.x<0 || pix1.x>gmap.img_gray.cols ||
		pix1.y<0 || pix1.y>gmap.img_gray.rows ||
		gmap.img_gray.at<unsigned char>(pix1) == 0 ||
		pix2.x<0 || pix2.x>gmap.img_gray.cols ||
		pix2.y<0 || pix2.y>gmap.img_gray.rows ||
		gmap.img_gray.at<unsigned char>(pix2) == 0
		)
	{
		return;
	}
	cv::line(gmap.img, pix1, pix2, color, thickness);

}

/* Xの描画 */
template<class C>	void drawCross(GMap& gmap, const C& point, int radius, cv::Scalar color, int thickness)
{
	cv::Point pix = toPixel(gmap, point);
	cv::Point pix1 = pix + cv::Point(radius, radius);
	cv::Point pix2 = pix + cv::Point(-radius, -radius);
	cv::Point pix3 = pix + cv::Point(-radius, radius);
	cv::Point pix4 = pix + cv::Point(radius, -radius);

	if (pix1.x<0 || pix1.x>gmap.img_gray.cols ||
		pix1.y<0 || pix1.y>gmap.img_gray.rows ||
		gmap.img_gray.at<unsigned char>(pix1) == 0 ||
		pix2.x<0 || pix2.x>gmap.img_gray.cols ||
		pix2.y<0 || pix2.y>gmap.img_gray.rows ||
		gmap.img_gray.at<unsigned char>(pix2) == 0	||
		pix3.x<0 || pix3.x>gmap.img_gray.cols ||
		pix3.y<0 || pix3.y>gmap.img_gray.rows ||
		gmap.img_gray.at<unsigned char>(pix2) == 0	||
		pix4.x<0 || pix4.x>gmap.img_gray.cols ||
		pix4.y<0 || pix4.y>gmap.img_gray.rows ||
		gmap.img_gray.at<unsigned char>(pix2) == 0
		)
	{
		return;
	}
	cv::line(gmap.img, pix1, pix2, color, thickness);
	cv::line(gmap.img, pix3, pix4, color, thickness);
}





/*  ピクセル出力  */
template<class C>	cv::Point toPixel(const GMap& gmap,const C& obj)
{
	int x = (int)(obj.x + gmap.res*0.5) / gmap.res + gmap.img.cols*gmap.img_org_x;
	int y = (int)(obj.y + gmap.res*0.5) / gmap.res + gmap.img.rows*gmap.img_org_y;
	return cv::Point(x, y);
}

/*  ピクセル出力(Size)  */
template<class C>	cv::Size toPixelSize(const GMap& gmap, const C& obj)
{
	int w = (int)(obj.x + gmap.res*0.5) / gmap.res;
	int h = (int)(obj.y + gmap.res*0.5) / gmap.res;
	return cv::Size(w, h);
}


///*  重み付きパーティクルの描画  */
//template<typename T> void drawWeghtedParicle(GMap& gmap,Position<T> par,)

/*  パーティクルをScalar変換  */
cv::Scalar weightToScalar(double weight, double max, double min)
{
	min = 0.001;
	double tmp = weight - min;
	cv::Scalar color = cv::Scalar(180, (int)(tmp / (max - min) * 255.0 + 0.5), 255);
	return color;
}
cv::Scalar weightToScalar(double weight, double max)
{
	
	cv::Scalar color = cv::Scalar(180, (int)(weight / max *255.0 + 0.5), 255);
	//cv::Scalar color = cv::Scalar(0, 255, 0);
	return color;
}
cv::Scalar weightToScalar(double weight)
{
	//cv::Scalar color = cv::Scalar(150, 255 - (int)((weight) / 0.008*255.0 + 0.5), 255);
	//cv::Scalar color = cv::Scalar(120, (int)(weight/ 0.008*255.0 + 0.5), 255);
	//cv::Scalar color = cv::Scalar(180, (int)((weight) / 0.02*255.0 + 0.5), 255);
	cv::Scalar color = cv::Scalar(150, (int)(weight / 0.02*255.0 + 0.5), 255);
	return color;
}


