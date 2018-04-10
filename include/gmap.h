#pragma once

#include "myfun.h"
#include "mycv.h"

/**********************************************************/
//	 �O���b�h�}�b�v
/**********************************************************/
class GMap
{

	/*  �ϐ�  */
public:
	cv::Mat img;				// �n�}�摜
private:
	cv::Mat img_gray;	// �����摜
	double org_lat;		// �n�}�̌��_�̈ܓx
	double org_lon;		// �n�}�̌��_�̌o�x
	double org_ele;		// �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
	double org_head;		// �n�}�̌��_�̏����p�x�i�k��0���Ƃ��Ď��v���𐳁j
	double res;			// �n�}�摜�̕���\[mm]
	double img_org_x;	// �n�}�摜�̌��_�i���K���j
	double img_org_y;	// �n�}�摜�̌��_�i���K���j


	/*  �R���X�g���N�^  */
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

	/*  �Z�b�^�[*/
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

	/*  �N���[�����o��  */
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

	/*  ��Q����ɂ���ꍇ�Ctrue  */
	template <class C> bool isOnObstacle(const C& obj)
	{
		cv::Point pixel = toPixel(*this, obj);	//	��Q���n�}��̃p�[�e�B�N�����݂���s�N�Z��

		if (pixel.x < 0 || pixel.x >= img_gray.cols ||
			pixel.y < 0 || pixel.y >= img_gray.rows ||
			(int)img_gray.at<unsigned char>(pixel) == 0)	//	���ݕs�\�̈�̏�����
		{
			return true;
		}
		return false;
	};

	/*  �摜�̐؂�o��  */
	template<typename T> void cutImg(Coor<T> center, Coor<T> radius)
	{
		Coor<T> upleft = center - radius;	//	�摜�̍��[
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

	/*  �h��Ԃ��~�̕`��(friend)  */
	template<class C> friend void drawCircle(GMap& gmap, const C& point, int radius, cv::Scalar color, int thickenss = -1);
	/* X�̕`��(friend) */
	template<class C> friend void drawCross(GMap& gmap, const C& point, int radius, cv::Scalar color, int thickness);
	/*  ���̕`��(friend)  */
	template<class C> friend void drawLine(GMap& gmap, C point1, C point2, cv::Scalar color, int thickness = 2);
	/*  �s�N�Z���o��(Point)(friend)  */
	template<class C> friend cv::Point toPixel(const GMap& gmap, const C& obj);
	/*  �s�N�Z���o��(Size)(friend)  */
	template<class C> friend cv::Size toPixelSize(const GMap& gmap, const C& obj);

};

/*  �h��Ԃ��~�̕`��  */
template<class C>	void drawCircle(GMap& gmap, const C& point, int radius, cv::Scalar color, int thickness)
{
	cv::Point pix = toPixel(gmap, point);

	//if (pix.x<0 || pix.x >= gmap.img_gray.cols ||
	//	pix.y<0 || pix.y >= gmap.img_gray.rows ||
	//	gmap.img_gray.at<unsigned char>(pix) == 0)
	//{
	//	return;
	//}

	cv::circle(gmap.img, pix, radius, color, thickness, 8, 0);	//	�p�[�e�B�N���̕`��

}

/*  ���̕`��  */
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

/* X�̕`�� */
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





/*  �s�N�Z���o��  */
template<class C>	cv::Point toPixel(const GMap& gmap,const C& obj)
{
	int x = (int)(obj.x + gmap.res*0.5) / gmap.res + gmap.img.cols*gmap.img_org_x;
	int y = (int)(obj.y + gmap.res*0.5) / gmap.res + gmap.img.rows*gmap.img_org_y;
	return cv::Point(x, y);
}

/*  �s�N�Z���o��(Size)  */
template<class C>	cv::Size toPixelSize(const GMap& gmap, const C& obj)
{
	int w = (int)(obj.x + gmap.res*0.5) / gmap.res;
	int h = (int)(obj.y + gmap.res*0.5) / gmap.res;
	return cv::Size(w, h);
}


///*  �d�ݕt���p�[�e�B�N���̕`��  */
//template<typename T> void drawWeghtedParicle(GMap& gmap,Position<T> par,)

/*  �p�[�e�B�N����Scalar�ϊ�  */
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


