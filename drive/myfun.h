
#pragma once

#include "stdafx.h"

#include "Coor.h"
#include "Polar.h"
#include "Position.h"
#include "mytime.h"
//#include "leica.h"
#include "GlobalToLocal.h"
#include "gpgga.h"


/**********************************************************/
//	�v���v���Z�b�T
/**********************************************************/

/*  ���񏈗���  */
#define THREAD_NUM std::thread::hardware_concurrency()




/**********************************************************/
//	�f�[�^���o�͗p�X�g���[��
/**********************************************************/

/*  ���o�̓f�[�^�̋�؂蕶����C�ӂŎw��  */
template<typename T>
void readData(std::istream& is, std::vector<std::vector<T>>& obj, const int skip_rows = 0, const int skip_cols = 0, const char delimiter = ',')
{
	obj.clear();
	std::string str;

	/*  �s�̃X�L�b�v  */
	for (int i = 0; i < skip_rows; i++)	std::getline(is, str);

	/*  �s�̓ǂݍ���  */
	while (std::getline(is, str))
	{
		std::istringstream istr(str);
		std::string token;

		/*  ��̃X�L�b�v  */
		for (int i = 0; i < skip_cols; i++)	std::getline(istr, token, delimiter);
		
		/*  �l�̓ǂݍ���  */
		std::vector<T> vec;
		while (std::getline(istr, token, delimiter))
		{
			std::istringstream iistr(token);
			T tmp;
			iistr >> tmp;
			vec.push_back(tmp);
		}

		obj.push_back(vec);
	}
}

/*  csv���͗p�X�g���[��(std::vector<T>)  */
template<typename T> std::istream& operator>>(std::istream &is, std::vector<T> &obj)
{
	obj.clear();
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
template<typename T> std::istream& operator>>(std::istream &is, std::vector<std::vector<T>> &obj)
{
	obj.clear();
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
	for (const auto &x : obj)
	{
		os << x << ",";
	}

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
//	���W�ϊ�
/**********************************************************/

/*  Position <-> Coor  */
template<typename T = double, typename U> Position<T> coor2pos(const Coor<U>& coor)
{
	Position<T> pos;
	pos.x = coor.x;
	pos.y = coor.y;
	return pos;
};
template<typename T = double, typename U> Coor<T> pos2coor(const Position<U>& pos)
{
	Coor<T> coor;
	coor.x = pos.x;
	coor.y = pos.y;
	return coor;
}

/*  Position <-> Polar  */
template<typename T = double, typename U> Position<T> pos2pol(const Polar<U>& pol)
{
	Position<T> pos;
	pos.x = pol.r*std::cos(pol.rad);
	pos.y = pol.r*std::sin(pol.rad);
	return pos;
}
template<typename T = double, typename U> Polar<T> pol2pos(const Position<U>& pos)
{
	Polar<T> pol;
	pol.r = std::sqrt(pos.x*pos.x + pos.y*pos.y);
	pol.rad = std::atan2(pos.y, pos.x);
	return pol;
}

/*  Coor <-> Polar  */
template<typename T = double, typename U> Coor<T> pol2coor(const Polar<U>& pol)
{
	Coor<T> coor;
	coor.x = pol.r*std::cos(pol.rad);
	coor.y = pol.r*std::sin(pol.rad);
	return coor;
}
template<typename T = double, typename U> Polar<T> coor2pol(const Coor<U>& coor)
{
	Polar<T> pol;
	pol.r = std::sqrt(coor.x*coor.x + coor.y*coor.y);
	pol.rad = std::atan2(coor.y, coor.x);
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

	Iterator itr = begin;

	int partial_length = quotient + 1;
	for (int i = 0; i < remainder; i++)
	{
		out.push_back(itr);
		std::advance(itr, partial_length);
	}

	partial_length--;
	for (int i = 0; i < num - remainder; i++)
	{
		out.push_back(itr);
		std::advance(itr, partial_length);
	}

	out.push_back(end);

	return out;
}

/*  �����s�̕�������s���ŕ������C�z��̕�����Ɋi�[  */
void split(const std::string& str, std::vector<std::string>& out)
{
	std::stringstream istr(str);
	std::string row;

	while (std::getline(istr, row))
	{
		out.push_back(row);
	}
}


/*  �������delimiter�ŕ������C�z��̕�����Ɋi�[  */
void split(const std::string& str, std::vector<std::string>& out, const char delimiter)
{
	std::stringstream istr(str);
	std::string token;

	while (std::getline(istr, token, delimiter))
	{
		out.push_back(token);
	}
}


/**********************************************************/
//	����֐��̒�`
/**********************************************************/

/*  �ǂݍ��݃G���[�\��  */
template<typename T = int>
void readError(const std::string &filename){
	std::cout << "\n\nError!!" << std::endl;
	std::cout << filename << " do not exist!\n" << std::endl;
	exit(0);
};

/*  �o�͗p�f�B���N�g�������݂��Ȃ��ꍇ  */
template<typename T = int>
void writeError(const std::string &filename)
{
	std::cout << "\n\nError!!" << std::endl;
	std::cout << filename << " can not be created!\n" << std::endl;
	exit(0);
};

/*  �O���[�o�����W�n�f�[�^��ǂݍ��݁C���[�J�����W�n�ɕϊ�  */
//	��1�v�f��radian��0.0�ŏ���������Ă���̂Œ���
//	��2�v�f�ȍ~��radian�͑O�X�e�b�v�̗v�f�Ƃ�p���ĎZ�o
//template<typename T = int>
//std::vector<Position<>>* readGlobalToLocal(const std::string &filename, GlobalToLocal &gl2lc, const bool error_flag = true)
//{
//	/*  �o�̓t�@�C���̗̈�m��  */
//	std::vector<Position<>>* out = new std::vector<Position<>>;
//
//	/*  �t�@�C���̓ǂݍ���  */
//	std::ifstream ifs(filename);
//	if (ifs.fail()){
//		if (error_flag)	readError(filename);
//		return out;
//	}
//
//	std::string row;
//	while (std::getline(ifs, row)){
//		std::istringstream stream(row);
//		std::vector<double> tmp(3);
//		char split;
//		stream >> tmp[0] >> split >> tmp[1] >> split >> tmp[2];
//		Coor<> true_coor = gl2lc.getCoor(tmp[0], tmp[1], tmp[2]);
//		Position<> position;
//		if (out->empty()){
//			position.set(true_coor.x, true_coor.y);	//	radian�ȊO��set
//		}
//		else{
//			double radian = atan2(true_coor.y - out->back().y, true_coor.x - out->back().x);
//			position.set(true_coor.x, true_coor.y, radian);
//		}
//		out->push_back(position);
//	}
//
//	out->shrink_to_fit();
//
//	std::cout << "Read: " << "'" << filename << "'" << std::endl;
//
//	return out;
//}


/*  std::vector��1�ɐ��K��  */
template<typename T>
void Normalize(std::vector<T> &vec)
{
	T sum = std::accumulate(vec.begin(), vec.end(), (T)0);

	if (sum != 0.0){
		for (auto &x : vec){
			x /= sum;
		}
		return;
	}

	for (auto &x : vec){
		x = 1.0 / (T)vec.size();
	}
	return;
};


/*  臒l����  */
//	臒l�𒴂��Ă���F1
//	臒l�𒴂��Ă��Ȃ�:0
template<typename T>
std::vector<std::vector<bool>> thresholdProcessing(const std::vector<std::vector<T>> &in, const T th)
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

/*  �ߔ����ɑ΂��Ĕ�ގ��ƂȂ镪�z��r��  */
//	���͂͗ގ����e�[�u��
template<typename T = int>
std::vector<int> useIndex(const std::vector<std::vector<bool>> &in)
{
	std::vector<int> out;

	for (int i = 0; i < in.size(); i++)
	{
		int count = std::count(in[i].begin(), in[i].end(), 0);
		if (count <= (float)in.size() / 2.0)
			out.push_back(i);
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

/*  �����l��Ԋ�  */
template<typename T>
T median(std::vector<T> vec)
{
	size_t c = vec.size() / 2;
	std::sort(vec.begin(), vec.end());
	return vec[c];
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
		for (int j = i + 1; j < data.size(); j++)
		{
			if (data[i] < data[j])
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

