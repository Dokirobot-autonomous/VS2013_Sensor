#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include <fstream>
#include <iostream>

template<typename T=double> class Polar
{
public:
	T r;		//���W r
	T theta;	//���W theta

	//�R���X�g���N�^
	Polar() :r(0), theta(0){}
	Polar(T r, T theta) :r(r), theta(theta){}
	//�R�s�[�R���X�g���N�^
	Polar(const Polar<T> &obj) :r(obj.r), theta(obj.theta){}
	//������Z�q
	Polar<T>& operator=(const Polar<T> &obj){
		if (this == &obj){
			return *this;
		}
		r = obj.r;
		theta = obj.theta;
		return *this;
	}
	//���Z�q
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
	//�l�ύX
	void set(T r, T theta){
		this->r = r;
		this->theta = theta;
	}
	void clear(){
		r = 0;
		theta = 0;
	}
	//������ϊ� ��(rad��deg)
	std::string ToString() const{
		std::stringstream ss;
		//ss.setf(std::ios::fixed);
		ss << r << '\t' << theta;
		return ss.str();
	}
};

//�X�J���[�{
template<typename T, typename U> Polar<T> operator*(const U &scal, const Polar<T> &vect){
	Coor<T> tmp;
	tmp.r = scal * vect.r;
	tmp.theta = scal * vect.theta;
	return tmp;
}

// ���͗p ���Z�q�I�[�o�[���[�h
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

// �o�͗p ���Z�q�I�[�o�[���[�h
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
