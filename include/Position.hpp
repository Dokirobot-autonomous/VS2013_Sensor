//位置(X,Y,θ)

#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <sstream>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>

template<typename T = double>class Position{
public:
	T x;	//座標 x
	T y;	//座標 y
	T r;	//角度θ[rad]
	const int par_num = 3;	//	パラメータの数(x,y,θ)

	//コンストラクタ
	Position() :x(0), y(0), r(0){}
	Position(T x, T y, T r) :x(x), y(y), r(r){}
	//コピーコンストラクタ
	Position(const Position<T> &obj) :x(obj.x), y(obj.y), r(obj.r){}
	//代入演算子
	Position<T>& operator=(const Position<T> &obj){
		if (this == &obj){
			return *this;
		}
		x = obj.x;
		y = obj.y;
		r = obj.r;
		return *this;
	}
	//加算演算子
	Position<T> operator+(const Position<T> &obj)const{
		Position<T> tmp;
		tmp.x = this->x + obj.x;
		tmp.y = this->y + obj.y;
		tmp.r = this->r + obj.r;
		return tmp;
	}
	Position<T>& operator+=(const Position<T> &obj){
		x += obj.x;
		y += obj.y;
		r += obj.r;
		return *this;
	}
	//減算演算子
	Position<T> operator-(const Position<T> &obj)const{
		Position<T> tmp;
		tmp.x = x - obj.x;
		tmp.y = y - obj.y;
		tmp.r = r - obj.r;
		return tmp;
	}
	Position<T>& operator-=(const Position<T> &obj){
		x -= obj.x;
		y -= obj.y;
		r -= obj.r;
		return *this;
	}
	//積演算子
	Position<T> operator*(const T &obj) const{
		Position<T> tmp;
		tmp.x = x * obj;
		tmp.y = y * obj;
		tmp.r = r * obj;
		return tmp;
	}
	Position<T>& operator*=(const T &obj){
		x *= obj;
		y *= obj;
		r *= obj;
		return *this;
	}
	Position<T> operator*(const Position<T> &obj)const{
		Position<T> tmp;
		tmp.x = x*obj.x;
		tmp.y = y*obj.y;
		tmp.r = r*obj.r;
		return tmp;
	}
	//商演算子
	Position<T> operator/(const T &obj) const{
		Position<T> tmp;
		tmp.x = x / obj;
		tmp.y = y / obj;
		tmp.r = r / obj;
		return tmp;
	}
	Position<T>& operator/=(const T &obj){
		x /= obj;
		y /= obj;
		r /= obj;
		return *this;
	}
	//値変更
	void set(T x, T y, T radian = 0.0){
		this->x = x;
		this->y = y;
		this->r = radian;
	}

	void clear(){
		x = 0;
		y = 0;
		r = 0;
	}
	//角度の取得
	double degree()const{ return (r * 180.0 / M_PI); }

	// Max
	Position<T> max(const std::vector<Position<T>>& positions){
		Position<> max=positions.front();
		for (int i = 1; i < positions.size(); i++){
			if (max.x < positions[i].x){
				max.x = positions[i].x;
			}
			if (max.y < positions[i].y){
				max.y = positions[i].y;
			}
			if (max.r < positions[i].r){
				max.r = positions[i].r;
			}
		}
		return max;
	}
	// Max
	Position<T> min(const std::vector<Position<T>>& positions){
		Position<> min = positions.front();
		for (int i = 1; i < positions.size(); i++){
			if (min.x > positions[i].x){
				min.x = positions[i].x;
			}
			if (min.y > positions[i].y){
				min.y = positions[i].y;
			}
			if (min.r > positions[i].r){
				min.r = positions[i].r;
			}
		}
		return min;
	}


	//2つのデータの距離の算出
	template<typename T> double operator|(const T &obj) const{
		return std::sqrt(std::pow((static_cast<double>(x)-static_cast<double>(obj.x)), 2) + std::pow((static_cast<double>(y)-static_cast<double>(obj.y)), 2));
	}

	T & operator [](int n)
	{
		switch (n)
		{
		case 0: return this->x;
		case 1: return this->y;
		case 2: return this->r;
		default:
			std::cerr << "No Value! " << std::endl;
			exit;
		}
	}

	/*
	//文字列変換 ※(rad→deg)
	std::string ToString() const{
	std::stringstream ss;
	//ss.setf(std::ios::fixed);
	ss<< x << '\t' << y << '\t' << (r * 180.0 / M_PI);
	return ss.str();
	}
	*/
	std::string ToString() const{
		std::stringstream ss;
		ss << x << '\t' << y << '\t' << r;
		return ss.str();
	}



	/*
	//read
	template<typename T> static void Read(const boost::filesystem::path &filePath, std::vector< Position<T> > &obj){
	boost::filesystem::ifstream fin(filePath);
	if(fin.fail()){fin.close(); throw"OpenFile \""+filePath.string()+"\" NotFound";}
	T x, y, th;
	obj.clear();
	fin >> x >> y >> th;
	while(!fin.eof()){
	th *= (M_PI/180.0);
	obj.push_back(std::vector< Position<T> >::value_type(x, y, th));
	fin >> x >> y >> th;
	}
	}
	//write
	template<typename T> static void Write(const boost::filesystem::path &filePath, const std::vector< Position<T> > &obj, const std::ios_base::openmode &mode = std::ios::out){
	boost::filesystem::ofstream fout(filePath, mode);
	fout.setf(std::ios::fixed);
	for(std::vector< Position<T> >::const_iterator itr(obj.begin()); itr != obj.end(); ++itr){
	fout << itr->x << '\t' << itr->y << '\t' << itr->degree() << '\n';
	}
	}
	*/
};

// 入力用 演算子オーバーロード
template<typename T> inline std::istream& operator>>(std::istream &is, Position<T> &obj){
	char split;
	is >> obj.x >> split >> obj.y >> split >> obj.r;
	return is;
}
template<typename T> inline std::istream& operator>>(std::istream &is, std::vector<Position<T>> &obj){
	std::string str;
	while (std::getline(is,str))
	{
		std::istringstream istr(str);
		Position<T> tmp;
		istr >> tmp;
		obj.push_back(tmp);
	}
	return is;
}

// 出力用 演算子オーバーロード
template<typename T> inline std::ostream& operator<<(std::ostream &os, const Position<T> &obj){
	return (os << obj.x << ',' << obj.y << ',' << obj.r);
}
template<typename T> inline std::ostream& operator<<(std::ostream &os, const std::vector<Position<T>> &obj){
	for (const auto &tmp : obj)
	{
		os << tmp << std::endl;
	}
	return os;
}