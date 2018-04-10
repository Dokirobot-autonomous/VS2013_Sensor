#pragma once

// �k��x
// ����y
// �k���玞�v���̊p�x����
// �^�̊p�x - �v���p�x��horizontal error

#include <vector>
#include <string>
#include <sstream>
#include "myfun.h"
#include "mytime.h"
//#include <math.h>
#define M_PI 3.1415

/**********************************************************/
//	���[�U�����v(Leica Disto)�pclass
/**********************************************************/
namespace leica
{
	/**********************************************************/
	//	 �N���X�̐錾
	/**********************************************************/
	template <typename T = double, typename S = double>
	class Data;


	/**********************************************************/
	//	 �v���@��̃p�����[�^�N���X
	/**********************************************************/
	template <typename T = double>
	class Param
	{
	public:
		Param(){};
		Param(Coor<T> origin) :origin(origin){};
		~Param(){};


		/**********************************************************/
		//  �A�N�Z�b�T
		/**********************************************************/
		inline void setOrigin(Coor<T> origin){
			this->origin = origin;
		};
		inline void setHorizontalErrorDeg(double horizontal_error){ 
			this->horizontal_error = horizontal_error;
			this->horizontal_error = this->horizontal_error*M_PI / 180.0;
		};
		inline void setHorizontalErrorRad(double horizontal_error){ 
			this->horizontal_error = horizontal_error; 
		};
		//inline Position<T> getOrigin() const { return origin; };


		/**********************************************************/
		//	Data�N���X�̃t�����h�錾
		/**********************************************************/
		friend Data<T>;


		/**********************************************************/
		//	�o�̓X�g���[��(friend)
		/**********************************************************/
		template <typename T>
		inline friend std::ostream& operator<<(std::ostream& os, const Param<T>& obj);


		/**********************************************************/
		//	 Param�o�͗plabel
		/**********************************************************/
		inline static std::string label()
		{
			return Position<T>::label();
		};


		/**********************************************************/
		//	 �v���@��̃p�����[�^
		/**********************************************************/
	private:
		Coor<T> origin;	//	�v���@��̐ݒu�ꏊ
		double horizontal_error;	// �k���玞�v���̂����[rad]
		// �v���œ���ꂽ�p�x+horizontal error�Ő^�̊p�x
	};


	/**********************************************************/
	//	�o�̓X�g���[��
	/**********************************************************/
	template <typename T>
	inline std::ostream& operator<<(std::ostream& os, const Param<T>& obj)
	{
		os << obj.origin;
		return os;
	};



	/**********************************************************/
	//	 �v���f�[�^�N���X
	/**********************************************************/
	template <typename T, typename S>
	class Data
	{
	public:
		Data(const leica::Param<S>& param) :param(param){};
		~Data(){};


		/**********************************************************/
		//  �o��
		/**********************************************************/
		//	Coor<>(x[mm],y[mm])
		Coor<> coor() const
		{
			double rad = horizontal + param.horizontal_error;

			double x = (range * 1000.0)*std::cos(rad) + param.origin.x;
			double y = (range * 1000.0)*std::sin(rad) + param.origin.y;

			return Coor<>(x, y);

		}

		Position<> position(leica::Data<T> data2) const {
			
			Coor<> coor1 = this->coor();
			Coor<> coor2 = data2.coor();

			double theta1 = std::atan2(coor2.y - coor1.y, coor2.x - coor1.x);
			return Position<>(coor1.x, coor1.y, theta1);

		}


		/**********************************************************/
		//  �A�N�Z�b�T
		/**********************************************************/
		//inline void setTime(MyTime& time){ this->time = time; };
		//inline void setImage_(bool image_){ this->image_ = image_; };
		//inline void setRange(double range){ this->range = range; };
		//inline void setInclimination(double inclimination){ this->inclimination = inclimination; };


		/**********************************************************/
		//	�o�̓X�g���[��(friend)
		/**********************************************************/
		template <typename T>
		inline friend std::istream& operator>>(std::istream& is, Data<T>& obj);
		template <typename T>
		inline friend std::ostream& operator<<(std::ostream& os, const Data<T>& obj);
		template<typename T>
		friend void deg2rad(leica::Data<T>& obj);

		/**********************************************************/
		//	 �f�[�^�̓ǂݍ���
		/**********************************************************/


		/**********************************************************/
		//	�t�����h�֐�
		/**********************************************************/
		//	�f�[�^�̓ǂݍ���
		template <typename T, typename S>
		friend void readDeg(std::istream& is, const leica::Param<S>& param, std::vector<leica::Data<T>>& data_set);
		template <typename T, typename S>
		friend void readRad(std::istream& is, const leica::Param<S>& param, std::vector<leica::Data<T>>& data_set);


		/**********************************************************/
		//	 �f�[�^�Z�b�g�o�͗plabel
		/**********************************************************/
		inline static std::string label()
		{
			return MyTime::label() + ",image,range[m],unknown[rad],vertical[rad],horizontal[rad],x[m],y[m],z[m],accuracy[mm]";
		};


		/**********************************************************/
		//  �v���f�[�^�ϐ� 
		/**********************************************************/
	public:
		double num;			//	�ԍ�
		MyTime time;		//	�v������
		bool image_;		//	�v�����̉摜�̗L��
		double range;	//	����[m]
		double inclimination;		//	�킩��Ȃ��p�x
		double vertical;	//	�����p�x[rad]
		double horizontal;	//	�����p�x[rad]
		double x;			//	������x�ړ���[m]
		double y;			//	������y�ړ���[m]
		double z;			//	������z�ړ���[m]
		double accuracy = 0;	//	���x[mm]

		//private:
		const Param<S>& param;	//	�v���@��̃p�����[�^

	};


	/**********************************************************/
	//	�f�[�^�̓ǂݍ���
	/**********************************************************/
	template <typename T, typename S>
	void readDeg(std::istream& is, const leica::Param<S>& param, std::vector<leica::Data<T>>& data_set)
	{
		std::string str;

		std::getline(is, str);	//	�P�s�ڂ��X�L�b�v

		/*  �f�[�^�̓ǂݍ���  */
		while (std::getline(is, str))
		{
			std::istringstream istr(str);
			leica::Data<T> tmp(param);
			istr >> tmp;
			data_set.push_back(tmp);
			data_set.back().inclimination = data_set.back().inclimination*M_PI / 180.0;
			data_set.back().vertical = data_set.back().vertical*M_PI / 180.0;
			data_set.back().horizontal = -data_set.back().horizontal*M_PI / 180.0;
		}
	}
	template <typename T, typename S>
	void readRad(std::istream& is, const leica::Param<S>& param, std::vector<leica::Data<T>>& data_set)
	{
		std::string str;

		std::getline(is, str);	//	�P�s�ڂ��X�L�b�v

		/*  �f�[�^�̓ǂݍ���  */
		while (std::getline(is, str))
		{
			std::istringstream istr(str);
			leica::Data<T> tmp(param);
			istr >> tmp;
			data_set.push_back(tmp);
			dataset_leica.back().horizontal = -dataset_leica.back().horizontal;
		}
	}

	template<typename T>
	double distance(const leica::Data<T>& data1, const leica::Data<T>& data2){
		double dx = data1.coor().x - data2.coor().x;
		double dy = data1.coor().y - data2.coor().y;
		return std::sqrt(dx*dx + dy*dy);
	}

	/**********************************************************/
	//	���o�̓X�g���[��
	/**********************************************************/
	template<typename T>
	inline std::istream& operator>>(std::istream& is, Data<T> & obj)
	{
		char c;
		is >> obj.num >> c
			>> obj.time >> c
			>> obj.image_ >> c
			>> obj.range >> c
			>> obj.inclimination >> c
			>> obj.vertical >> c
			>> obj.horizontal >> c
			>> obj.x >> c
			>> obj.y >> c
			>> obj.z >> c
			>> obj.accuracy;
		return is;
	};
	template <typename T>
	inline std::ostream& operator<<(std::ostream& os, const Data<T>& obj)
	{
		os << obj.num << ","
			<< obj.time << ","
			<< obj.image_ << ","
			<< obj.range << ","
			<< obj.inclimination << ","
			<< obj.vertical << ","
			<< obj.horizontal << ","
			<< obj.x << ","
			<< obj.y << ","
			<< obj.z << ","
			<< obj.accuracy << ","
			<< std::endl;
		return os;
	};
	template<typename T>
	void deg2rad(std::vector<leica::Data<T>>& obj)
	{
		for (auto& data : obj)
		{
			data.inclimination = data.inclimination*M_PI / 180.0;
			data.vertical = data.vertical*M_PI / 180.0;
			data.horizontal = data.horizontal*M_PI / 180.0;
		}
	}
};
