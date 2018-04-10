#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>

template<typename T=double> class Polar
{
public:
	T r;		//座標 r
	T theta;	//座標 theta

	//コンストラクタ
	Polar() :r(0), theta(0){}
	Polar(T r, T theta) :r(r), theta(theta){}
	//コピーコンストラクタ
	Polar(const Polar<T> &obj) :r(obj.r), theta(obj.theta){}
	//代入演算子
	Polar<T>& operator=(const Polar<T> &obj){
		if (this == &obj){
			return *this;
		}
		r = obj.r;
		theta = obj.theta;
		return *this;
	}
	//演算子
	Polar<T> operator+(const Polar<T> &obj) const{
		Polar<T> tmp;
		tmp.r = r + obj.r;
		tmp.theta = theta + obj.theta;
		return tmp;
	}
	Polar<T>& operator+=(const Polar<T> &obj){
		r += obj.r;
		theta += obj.theta;
		return *this;
	}
	Polar<T> operator-(const Polar<T> &obj) const{
		Polar<T> tmp;
		tmp.r		= r - obj.r;
		tmp.theta = theta - obj.theta;
		return tmp;
	}
	Polar<T>& operator-=(const Polar<T> &obj){
		r -= obj.r;
		theta -= obj.theta;
		return *this;
	}
	Polar<T> operator*(const T &obj) const{
		Polar<T> tmp;
		tmp.r = r * obj;
		tmp.theta = theta * obj;
		return tmp;
	}
	Polar<T>& operator*=(const T &obj){
		r *= obj;
		theta *= obj;
		return *this;
	}
	Polar<T> operator/(const T &obj) const{
		Polar<T> tmp;
		tmp.r = r / obj;
		tmp.theta = theta / obj;
		return tmp;
	}
	Polar<T>& operator/=(const T &obj){
		r /= obj;
		theta /= obj;
		return *this;
	}
	bool operator==(const Polar<T> &obj) const{
		if ((r == obj.r) && (theta == obj.theta)) return true;
		else return false;
	}
	bool operator!=(const Polar<T> &obj) const{
		if ((r == obj.r) && (theta == obj.theta)) return false;
		else return true;
	}
	//値変更
	void set(T r, T theta){
		this->r = r;
		this->theta = theta;
	}
	void clear(){
		r = 0;
		theta = 0;
	}
	//文字列変換 ※(rad→deg)
	std::string ToString() const{
		std::stringstream ss;
		//ss.setf(std::ios::fixed);
		ss << r << '\t' << theta;
		return ss.str();
	}
};

//スカラー倍
template<typename T, typename U> Polar<T> operator*(const U &scal, const Polar<T> &vect){
	Coor<T> tmp;
	tmp.r = scal * vect.r;
	tmp.theta = scal * vect.theta;
	return tmp;
}

// 入力用 演算子オーバーロード
template<typename T> inline std::istream& operator>>(std::istream &is, Polar<T> &obj){
	char split;
	is >> obj.r >> split >> obj.theta;
	return is;
}
template<typename T> inline std::istream& operator>>(std::istream &is, std::vector<Polar<T>> &obj){
	std::string str;
	while (std::getline(is, str))
	{
		std::istringstream istr(str);
		Polar<T> tmp;
		istr >> tmp;
		obj.push_back(tmp);
	}
	return is;
}

// 出力用 演算子オーバーロード
template<typename T> inline std::ostream& operator<<(std::ostream &os, const Polar<T> &obj){
	return (os << obj.r << ',' << obj.theta);
}
template<typename T> inline std::ostream& operator<<(std::ostream &os, const std::vector<Polar<T>> &obj){
	for (const auto &tmp : obj)
	{
		os << tmp << std::endl;
	}
	return os;
}
