
#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <random>
#include <numeric>
#include <cmath>

#include "Coor.hpp"
#include "Polar.h"
#include "Position.hpp"
#include "GlobalToLocal.h"
#include "TransMatrix.hpp"
#include <opencv2\opencv.hpp>
#include <opencv\highgui.h>

/**********************************************************/
//	自作クラス
/**********************************************************/

/*  csvファイルをcv::Matに読み込み  */
template<typename T>
class cvReadData
{
public:
	cvReadData(cv::Mat& mat, char delimitor = ',') :mat(mat), delimitor(delimitor){};
	~cvReadData(){};

	template <typename T>
	friend std::istream& operator>>(std::istream& is, cvReadData<T>& obj);

private:
	cv::Mat mat;
	char delimitor;

};
template<typename T>
std::istream& operator>>(std::istream& is, cvReadData<T>& obj)
{
	std::string str;
	while (std::getline(is, str))
	{
		std::istringstream istr(str);
		std::string token;
		std::vector<T> vec;

		while (std::getline(istr, token, obj.delimitor))
		{
			std::istringstream istr2(token);
			T tmp;
			istr2 >> tmp;
			vec.push_back(tmp);
		}
		cv::Mat mat(vec);
		mat = mat.t();
		obj.mat.push_back(mat);
	}

	return is;
}

template<typename T>
std::istream& operator>>(std::istream& is, cv::Mat_<T>& mat_)
{
	mat_.release();
	std::string str;
	while (std::getline(is, str))
	{
		std::istringstream istr(str);
		std::string token;
		std::vector<T> vec;

		while (std::getline(istr, token, ','))
		{
			std::istringstream istr2(token);
			T tmp;
			istr2 >> tmp;
			vec.push_back(tmp);
		}
		cv::Mat mat(vec);
		mat = mat.t();
		mat_.push_back(mat);
	}

	return is;
}

/**********************************************************/
//	自作関数の宣言 5
/**********************************************************/

template<typename T, int SIZE>
cv::Matx<T, SIZE, 1> vecToMatx(const std::vector<T> &vector);
template<typename T, int ROW, int COL >
cv::Matx<T, ROW, COL> vec2ToMatx(const std::vector<std::vector<T>> &vector);
template<typename T = double> void normalizeCvmat_Row(cv::Mat_<T> &mat);
template<class C, typename T, typename S>
cv::Point ToPixel(const C &obj, const cv::Mat &map, T resolution, S original_x, S original_y);
template<class C, typename T>
cv::Size ToPixelSize(const C &obj, const cv::Mat &map, T resolution);


/**********************************************************/
//	ユーザ定義関数 5
/**********************************************************/

/*  1次元配列をMatxの縦ベクトルに変換  */
template<typename T, int SIZE>
cv::Matx<T, SIZE, 1> vecToMatx(const std::vector<T> &vector)
{
	cv::Matx<T, SIZE, 1> out;

	for (int i = 0; i < SIZE; i++)
	{
		out(i, 0) = vector[i];
	}

	return out;

};

/*  2次元配列をMatxに変換*/
template<typename T, int ROW, int COL >
cv::Matx<T, ROW, COL> vec2ToMatx(const std::vector<std::vector<T>> &vector)
{
	cv::Matx<T, ROW, COL> out;

	for (int ri = 0; ri < ROW; ri++)
	{
		for (int ci = 0; ci < COL; ci++)
		{
			out(ri, ci) = vector[ri][ci];
		}
	}

	return out;

}

/*  cv::Matの行の合計を1に正規化*/
template<typename T>
void normalizeCvmat_Row(cv::Mat_<T> &mat){

	for (int ri = 0; ri < mat.rows; ri++)
	{
		T sum = 0.0;
		for (int ci = 0; ci < mat.cols; ci++)
		{
			sum += std::pow(mat(ri, ci), 2);	//	二乗和
		}
		sum = std::sqrt(sum);
		for (int ci = 0; ci < mat.cols; ci++)
		{
			if (sum == 0)
			{
				mat(ri, ci) = (T)1 / mat.cols;
			}
			else
			{
				mat(ri, ci) /= sum;
			}
		}
	}
};

/*  Local座標をPixelに変換  */
template<class C, typename T, typename S>
cv::Point ToPixel(const C &obj, const cv::Mat &map, T resolution, S original_x, S original_y)
{
	cv::Point out;
	out.x = (int)(obj.x + resolution*0.5) / resolution + map.cols*original_x;
	out.y = (int)(obj.y + resolution*0.5) / resolution + map.rows*original_y;
		
	return out;
}

template<class C, typename T, typename S>
cv::Point ToPixel(const C &obj, int cols, int rows, T resolution, S original_x, S original_y)
{
	cv::Point out;
	out.x = (int)(obj.x + resolution*0.5) / resolution + cols*original_x;
	out.y = (int)(obj.y + resolution*0.5) / resolution + rows*original_y;

	return out;
}

template<class C, typename T>
cv::Size ToPixelSize(const C &obj, const cv::Mat &map, T resolution)
{
	cv::Size out;
	out.width = (int)(obj.x + resolution*0.5) / resolution;
	out.height = (int)(obj.y + resolution*0.5) / resolution;
	return out;
}
