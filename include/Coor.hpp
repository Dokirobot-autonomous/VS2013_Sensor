//直交座標クラス

#pragma once

#include <cmath>
#include <sstream>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include "Polar.h"

//#include <boost/filesystem/fstream.hpp>

template<typename T = double>class Coor{
public:
	T x;	//座標 x
	T y;	//座標 y

	//コンストラクタ
	Coor() :x(0), y(0){}
	Coor(T x, T y) :x(x), y(y){}
	//コピーコンストラクタ
	Coor(const Coor<T> &obj) :x(obj.x), y(obj.y){}
	//極座標系を直交座標系に変換するコンストラクタ
	Coor(const Polar<T> &obj) :x(obj.r*std::cos(obj.theta)), y(obj.r*std::sin(obj.theta)){};
	//代入演算子
	Coor<T>& operator=(const Coor<T> &obj){
		if (this == &obj){
			return *this;
		}
		x = obj.x;
		y = obj.y;
		return *this;
	}
	//演算子
	Coor<T> operator+(const Coor<T> &obj) const{
		Coor<T> tmp;
		tmp.x = x + obj.x;
		tmp.y = y + obj.y;
		return tmp;
	}
	Coor<T>& operator+=(const Coor<T> &obj){
		x += obj.x;
		y += obj.y;
		return *this;
	}
	Coor<T> operator-(const Coor<T> &obj) const{
		Coor<T> tmp;
		tmp.x = x - obj.x;
		tmp.y = y - obj.y;
		return tmp;
	}
	Coor<T>& operator-=(const Coor<T> &obj){
		x -= obj.x;
		y -= obj.y;
		return *this;
	}
	Coor<T> operator*(const T &obj) const{
		Coor<T> tmp;
		tmp.x = x * obj;
		tmp.y = y * obj;
		return tmp;
	}
	Coor<T>& operator*=(const T &obj){
		x *= obj;
		y *= obj;
		return *this;
	}
	Coor<T> operator/(const T &obj) const{
		Coor<T> tmp;
		tmp.x = x / obj;
		tmp.y = y / obj;
		return tmp;
	}
	Coor<T>& operator/=(const T &obj){
		x /= obj;
		y /= obj;
		return *this;
	}
	bool operator==(const Coor<T> &obj) const{
		if ((x == obj.x) && (y == obj.y)) return true;
		else return false;
	}
	bool operator!=(const Coor<T> &obj) const{
		if ((x == obj.x) && (y == obj.y)) return false;
		else return true;
	}
	template<typename T> double operator|(const T &obj) const{//距離の算出
		return std::sqrt(std::pow((static_cast<double>(x)-static_cast<double>(obj.x)), 2) + std::pow((static_cast<double>(y)-static_cast<double>(obj.y)), 2));
	}
	//比較 std::map,std::setでソートを使うときの都合
	bool operator<(const Coor<T>& obj) const{
		double a, b;
		a = std::sqrt(std::pow((x), 2) + std::pow((y), 2));
		b = std::sqrt(std::pow((obj.x), 2) + std::pow((obj.y), 2));
		return (a<b);
	}
	bool operator>(const Coor<T>& obj) const{
		double a, b;
		a = std::sqrt(std::pow((x), 2) + std::pow((y), 2));
		b = std::sqrt(std::pow((obj.x), 2) + std::pow((obj.y), 2));
		return (a>b);
	}
	//値変更
	void set(T x, T y){
		this->x = x;
		this->y = y;
	}
	void clear(){
		x = 0;
		y = 0;
	}
	//文字列変換 ※(rad→deg)
	std::string ToString() const{
		std::stringstream ss;
		//ss.setf(std::ios::fixed);
		ss << x << '\t' << y;
		return ss.str();
	}

};

//スカラー倍
template<typename T, typename U> Coor<T> operator*(const U &scal, const Coor<T> &vect){
	Coor<T> tmp;
	tmp.x = scal * vect.x;
	tmp.y = scal * vect.y;
	return tmp;
}

// 入力用 演算子オーバーロード
template<typename T> inline std::istream& operator>>(std::istream &is, Coor<T> &obj){
	char split;
	is >> obj.x >> split >> obj.y;
	return is;
}
template<typename T> inline std::istream& operator>>(std::istream &is, std::vector<Coor<T>> &obj){
	std::string str;
	while (std::getline(is, str))
	{
		std::istringstream istr(str);
		Coor<T> tmp;
		istr >> tmp;
		obj.push_back(tmp);
	}
	return is;
}

// 出力用 演算子オーバーロード
template<typename T> inline std::ostream& operator<<(std::ostream &os, const Coor<T> &obj){
	return (os << obj.x << ',' << obj.y);
}
template<typename T> inline std::ostream& operator<<(std::ostream &os, const std::vector<Coor<T>> &obj){
	for (const auto &tmp : obj)
	{
		os << tmp << std::endl;
	}
	return os;
}