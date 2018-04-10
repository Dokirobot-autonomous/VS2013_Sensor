
#pragma once

#include <string>
#include <iostream>
#include <fstream>
#include <random>
#include <numeric>
#include <cmath>
#include <thread>
#include <time.h>
#include <mutex>
#include <deque>

#include "Coor.hpp"
#include "Polar.h"
#include "Position.hpp"
#include "mytime.h"
#include "GlobalToLocal.h"
#include "TransMatrix.hpp"
#include "Statistics.h"
#include "ICPal2.hpp"
#include "GPGGA.hpp"
#include "ParticleFilter.h"


/**********************************************************/
//	�v���v���Z�b�T
/**********************************************************/

/*  ���񏈗���  */
//#define THREAD_NUM std::thread::hardware_concurrency()
#define THREAD_NUM 2


/**********************************************************/
//	�f�[�^���o�͗p�X�g���[��
/**********************************************************/

/*  ���o�̓f�[�^�̋�؂蕶����C�ӂŎw��  */
template<typename T>
class stdStreamData
{
public:
	stdStreamData(std::vector<T> &vec, char delimitor) :vec(&vec), delimitor(delimitor) { flag = 0; };
	stdStreamData(std::vector<std::vector<T>> &vvec, char delimitor) :vvec(&vvec), delimitor(delimitor) { flag = 1; };
	~stdStreamData() {};

	template<typename T>
	friend std::istream& operator >> (std::istream& is, stdStreamData<T> &obj);
	template<typename T>
	friend std::ostream& operator<<(std::ostream& os, const stdStreamData<T> &obj);

private:

	/*  std::vector�p  */
	std::vector<T> *vec = nullptr;
	std::vector<std::vector<T>> *vvec = nullptr;

	/*  FLAG  */
	int flag;

	/*  ��؂蕶��  */
	char delimitor;

};
template<typename T>	//	friend�֐�
std::istream& operator >> (std::istream& is, stdStreamData<T> &obj)
{
	std::string str;
	switch (obj.flag)
	{
		/*  std::vector<T> �̏ꍇ  */
	case 0:
		while (std::getline(is, str, obj.delimitor))
		{
			std::istringstream istr(str);
			T tmp;
			istr >> tmp;
			obj.vec->push_back(tmp);
		}
		return is;

		/*  std::vector<std::vector<T>> �̏ꍇ  */
	case 1:
		while (std::getline(is, str))
		{
			std::istringstream istr(str);
			std::vector<T> tmp;
			istr >> stdStreamData<T>(tmp, obj.delimitor);
			obj.vvec->push_back(tmp);
		}
		return is;

		/*  ���s�����Ƃ��̂��߂ɃG���[�o��  */
	default:
		std::cout << "Stream Error!" << std::endl;
		exit(0);
	}
}
template<typename T>	//	friend�֐�
std::ostream& operator<<(std::ostream& os, const stdStreamData<T> &obj)
{
	switch (obj.flag)
	{
		/*  std::vector<T> �̏ꍇ  */
	case 0:
		for (auto &tmp : *obj.vec)
		{
			os << tmp << obj.delimitor;
		}
		return os;

		/*  std::vector<std::vector<T>> �̏ꍇ  */
	case 1:
		for (auto &tmp : *obj.vvec)
		{
			os << stdStreamData<T>(tmp, obj.delimitor) << std::endl;
		}
		return os;

		/*  ���s�����Ƃ��̂��߂ɃG���[�o��  */
	default:
		std::cout << "Stream Error!" << std::endl;
		exit(0);
	}
}

/*  csv���͗p�X�g���[��(std::vector<T>)  */
template<typename T> std::istream& operator >> (std::istream &is, std::vector<T> &obj)
{
	std::string str;
	while (std::getline(is, str, ','))
	{
		std::istringstream istr(str);
		T tmp;
		istr >> tmp;
		obj.push_back(tmp);
	}
	return is;
}

/*  csv���͗p�X�g���[��(std::vector<std::vector<T>>)  */
template<typename T> std::istream& operator >> (std::istream &is, std::vector<std::vector<T>> &obj)
{
	std::string str;
	while (std::getline(is, str))
	{
		std::istringstream istr(str);
		std::vector<T> tmp;
		istr >> tmp;
		obj.push_back(tmp);
	}
	return is;
}

/*  csv�o�͗p�X�g���[��(std::vector<T>)  */
template<typename T> std::ostream& operator<<(std::ostream &os, const std::vector<T> &obj)
{
	for (int i = 0; i < obj.size() - 1; i++) {
		os << obj[i] << ",";
	}
	os << obj.back();

	return os;
}

/*  csv�o�͗p�X�g���[��(std::vector<std::vector<T>>)  */
template<typename T> std::ostream& operator<<(std::ostream &os, const std::vector<std::vector<T>> &obj)
{
	for (const auto &vec : obj)
	{
		os << vec << std::endl;
	}
	return os;
}



/**********************************************************/
//	���W�ϊ��X�g���[��
/**********************************************************/

/*  Position <-> Coor  */
template<typename T, typename U> Position<T> coorToPos(const Coor<U>& coor)
{
	Position<T> pos;
	pos.x = coor.x;
	pos.y = coor.y;
	return pos;
};
template<typename T, typename U> Coor<T> posToCoor(const Position<U>& pos)
{
	Coor<T> coor;
	coor.x = pos.x;
	coor.y = pos.y;
	return coor;
}

/*  Position <-> Polar  */
template<typename T, typename U> Position<T> posToPol(const Polar<U>& pol)
{
	Position<T> pos;
	pos.x = pol.r*std::cos(pol.theta);
	pos.y = pol.r*std::sin(pol.theta);
	return pos;
}
template<typename T, typename U> Polar<T> polToPos(const Position<U>& pos)
{
	Polar<T> pol;
	pol.r = std::sqrt(pos.x*pos.x + pos.y*pos.y);
	pol.theta = std::atan2(pos.y, pos.x);
	return pol;
}

/*  Coor <-> Polar  */
template<typename T> Coor<T> polToCoor(const Polar<T>& pol)
{
	Coor<T> coor;
	coor.x = pol.r*std::cos(pol.theta);
	coor.y = pol.r*std::sin(pol.theta);
	return coor;
}
template<typename T> std::vector<Coor<T>> polToCoor(const std::vector<Polar<T>>& pol_v) {
	std::vector<Coor<T>> out;
	for (const auto& pol : pol_v) {
		Coor<T> coor = polToCoor(pol);
		out.push_back(coor);
	}
	return out;
}
template<typename T, typename U> Polar<T> coorToPol(const Coor<U>& coor)
{
	Polar<T> pol;
	pol.r = std::sqrt(coor.x*coor.x + coor.y*coor.y);
	pol.theta = std::atan2(coor.y, coor.x);
	return pol;
}



/**********************************************************/
//	���[�U��`operator
/**********************************************************/

/*  Coor��Position�̋���  */
template<typename T, typename U>
double operator|(Coor<T> coor, Position<U> position)
{
	double x = coor.x - position.x;
	double y = coor.y - position.y;
	return std::sqrt(x*x + y*y);
}


/**********************************************************/
//	���Ȋ�������֐��̒�`
/**********************************************************/

/*  std�z��𕪊����C�C�e���[�^��Ԋ�  */
template<typename Iterator>
std::vector<Iterator> divide(Iterator begin, Iterator end, int num)
{
	std::vector<Iterator> out;

	const int length = std::distance(begin, end);
	const int quotient = length / num;
	int remainder = length%num;

	int partial_length = 0;

	for (Iterator itr = begin; itr != end; std::advance(itr, partial_length))
	{
		out.push_back(itr);
		partial_length = quotient;

		if (remainder>0)
		{
			remainder--;
			partial_length++;
		}
	}
	out.push_back(end);

	return out;
}



/**********************************************************/
//	����֐��̐錾
/**********************************************************/

std::vector<Position<>>* readGlobalToLocal(const std::string &filename, GlobalToLocal &gl2lc, const bool error_flag = true);

template<typename T> void Normalize(std::vector<T> &vec);

void readError(const std::string &filename);
void writeError(const std::string &filename);

template<typename T> std::vector<T> Split(const std::string &dat, const std::string delimiter);
template<typename T>
std::vector<std::vector<bool>> ThresholdProcessing(const std::vector<std::vector<T>> &in, const T th);
std::vector<bool> UseIndex(const std::vector<std::vector<bool>> &in, float th);
template <typename T>
std::vector<int> sortUpIdx(std::vector<T> data);

/*  �~���\�[�g���s���C�C���f�b�N�X��ϊ�  */
template <typename T>
std::vector<int> sortDownIdx(std::vector<T> data);


/**********************************************************/
//	����֐��̒�`
/**********************************************************/



/*  �O���[�o�����W�n�f�[�^��ǂݍ��݁C���[�J�����W�n�ɕϊ�  */
//	��1�v�f��radian��0.0�ŏ���������Ă���̂Œ���
//	��2�v�f�ȍ~��radian�͑O�X�e�b�v�̗v�f�Ƃ�p���ĎZ�o
std::vector<Position<>>* readGlobalToLocal(const std::string &filename, GlobalToLocal &gl2lc, const bool error_flag)
{
	/*  �o�̓t�@�C���̗̈�m��  */
	std::vector<Position<>>* out = new std::vector<Position<>>;

	/*  �t�@�C���̓ǂݍ���  */
	std::ifstream ifs(filename);
	if (ifs.fail()) {
		if (error_flag)	readError(filename);
		return out;
	}

	std::string row;
	while (std::getline(ifs, row)) {
		std::istringstream stream(row);
		std::vector<double> tmp(3);
		char split;
		stream >> tmp[0] >> split >> tmp[1] >> split >> tmp[2];
		Coor<> true_coor = gl2lc.getCoor(tmp[0], tmp[1], tmp[2]);
		Position<> position;
		if (out->empty()) {
			position.set(true_coor.x, true_coor.y);	//	radian�ȊO��set
		}
		else {
			double radian = atan2(true_coor.y - out->back().y, true_coor.x - out->back().x);
			position.set(true_coor.x, true_coor.y, radian);
		}
		out->push_back(position);
	}

	out->shrink_to_fit();

	std::cout << "Read: " << "'" << filename << "'" << std::endl;

	return out;
}


/*  std::vector��1�ɐ��K��  */
template<typename T>
void Normalize(std::vector<T> &vec)
{
	T sum = std::accumulate(vec.begin(), vec.end(), (T)0);

	if (sum != 0.0) {
		for (auto &x : vec) {
			x /= sum;
		}
		return;
	}

	for (auto &x : vec) {
		x = 1.0 / (T)vec.size();
	}
	return;
};

/*  �ǂݍ��݃G���[�\��  */
void readError(const std::string &filename) {
	std::cout << "\n\nError!!" << std::endl;
	std::cout << filename << " do not exist!\n" << std::endl;
	exit(0);
};

/*  �o�͗p�f�B���N�g�������݂��Ȃ��ꍇ  */
void writeError(const std::string &filename)
{
	std::cout << "\n\nError!!" << std::endl;
	std::cout << filename << " can not be created!\n" << std::endl;
	exit(0);
};

/*  �������delimiter�ŋ�؂�  */
template<typename T>
std::vector<T> Split(const std::string &dat, const std::string delimiter)
{
	std::vector<T> out;
	out.reserve(0);

	size_t current = 0, found;
	std::string str;
	T tmp;
	while ((found = dat.find_first_of(delimiter, current)) != std::string::npos) {
		str = std::string(dat, current, found - current);
		tmp = (T)std::stod(str);
		out.push_back(tmp);
		current = found + 1;
	}
	str = std::string(dat, current, dat.size() - current);
	tmp = std::stof(str);
	out.push_back(tmp);

	return out;
}

/*  臒l����  */
//	臒l�𒴂��Ă���F1
//	臒l�𒴂��Ă��Ȃ�:0
template<typename T>
std::vector<std::vector<bool>> ThresholdProcessing(const std::vector<std::vector<T>> &in, const T th)
{
	std::vector<std::vector<bool>> out;

	for (const auto &row : in)
	{
		std::vector<bool> vec;

		for (const auto &x : row)
		{
			/*  臒l����  */
			bool tmp;
			if (x > th)
			{
				tmp = 1;
			}
			else
			{
				tmp = 0;
			}

			vec.push_back(tmp);
		}
		vec.shrink_to_fit();
		out.push_back(vec);
	}

	out.shrink_to_fit();
	return out;
}
template<typename T>
std::vector<bool> ThresholdProcessing(const std::vector<T> &in, const T th)
{
	std::vector<bool> out;

	for (const auto& x : in)
	{
		/*  臒l����  */
		bool tmp;
		if (x > th)
		{
			tmp = 1;
		}
		else
		{
			tmp = 0;
		}
		out.push_back(tmp);
	}
	return out;
}





/*  �ߔ����ɑ΂��Ĕ�ގ��ƂȂ镪�z��r��  */
//	���͂͗ގ����e�[�u��
std::vector<bool> UseIndex(const std::vector<std::vector<bool>> &in, float th)
{
	std::vector<bool> out;

	for (int i = 0; i<in.size(); i++)
	{
		int count = std::count(in[i].begin(), in[i].end(), true);
		if (count >= th)
			out.push_back(true);
		else
			out.push_back(false);
	}

	return out;
}

/*  �����\�[�g���s���C�C���f�b�N�X��ϊ�  */
template <typename T>
std::vector<int> sortUpIdx(std::vector<T> data)
{
	std::vector<int> idx;
	idx.reserve(data.size());
	for (int i = 0; i < data.size(); i++)	idx.push_back(i);

	for (int i = 0; i < data.size() - 1; i++)
	{
		for (int j = i + 1; j<data.size(); j++)
		{
			if (data[i]>data[j])
			{
				T dtmp = data[i];
				data[i] = data[j];
				data[j] = dtmp;

				int itmp = idx[i];
				idx[i] = idx[j];
				idx[j] = itmp;
			}
		}
	}

	return idx;
}

/*  �~���\�[�g���s���C�C���f�b�N�X��ϊ�  */
template <typename T>
std::vector<int> sortDownIdx(std::vector<T> data)
{
	std::vector<int> idx;
	idx.reserve(data.size());
	for (int i = 0; i < data.size(); i++)	idx.push_back(i);

	for (int i = 0; i < data.size() - 1; i++)
	{
		for (int j = i + 1; j<data.size(); j++)
		{
			if (data[i]<data[j])
			{
				T dtmp = data[i];
				data[i] = data[j];
				data[j] = dtmp;

				int itmp = idx[i];
				idx[i] = idx[j];
				idx[j] = itmp;
			}
		}
	}

	return idx;
}

/*  �����l��Ԋ�  */
template<typename T>
T median(std::vector<T> vec)
{
	size_t c = vec.size() / 2;
	std::sort(vec.begin(), vec.end());
	return vec[c];
}