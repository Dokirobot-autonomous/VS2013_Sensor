#pragma once

// 北をx
// 東をy
// 北から時計回りの角度をθ
// 真の角度 - 計測角度でhorizontal error

#include <vector>
#include <string>
#include <sstream>
#include "myfun.h"
#include "mytime.h"
//#include <math.h>
#define M_PI 3.1415

/**********************************************************/
//	レーザ距離計(Leica Disto)用class
/**********************************************************/
namespace leica
{
	/**********************************************************/
	//	 クラスの宣言
	/**********************************************************/
	template <typename T = double, typename S = double>
	class Data;


	/**********************************************************/
	//	 計測機器のパラメータクラス
	/**********************************************************/
	template <typename T = double>
	class Param
	{
	public:
		Param(){};
		Param(Coor<T> origin) :origin(origin){};
		~Param(){};


		/**********************************************************/
		//  アクセッサ
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
		//	Dataクラスのフレンド宣言
		/**********************************************************/
		friend Data<T>;


		/**********************************************************/
		//	出力ストリーム(friend)
		/**********************************************************/
		template <typename T>
		inline friend std::ostream& operator<<(std::ostream& os, const Param<T>& obj);


		/**********************************************************/
		//	 Param出力用label
		/**********************************************************/
		inline static std::string label()
		{
			return Position<T>::label();
		};


		/**********************************************************/
		//	 計測機器のパラメータ
		/**********************************************************/
	private:
		Coor<T> origin;	//	計測機器の設置場所
		double horizontal_error;	// 北から時計回りのずれを正[rad]
		// 計測で得られた角度+horizontal errorで真の角度
	};


	/**********************************************************/
	//	出力ストリーム
	/**********************************************************/
	template <typename T>
	inline std::ostream& operator<<(std::ostream& os, const Param<T>& obj)
	{
		os << obj.origin;
		return os;
	};



	/**********************************************************/
	//	 計測データクラス
	/**********************************************************/
	template <typename T, typename S>
	class Data
	{
	public:
		Data(const leica::Param<S>& param) :param(param){};
		~Data(){};


		/**********************************************************/
		//  出力
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
		//  アクセッサ
		/**********************************************************/
		//inline void setTime(MyTime& time){ this->time = time; };
		//inline void setImage_(bool image_){ this->image_ = image_; };
		//inline void setRange(double range){ this->range = range; };
		//inline void setInclimination(double inclimination){ this->inclimination = inclimination; };


		/**********************************************************/
		//	出力ストリーム(friend)
		/**********************************************************/
		template <typename T>
		inline friend std::istream& operator>>(std::istream& is, Data<T>& obj);
		template <typename T>
		inline friend std::ostream& operator<<(std::ostream& os, const Data<T>& obj);
		template<typename T>
		friend void deg2rad(leica::Data<T>& obj);

		/**********************************************************/
		//	 データの読み込み
		/**********************************************************/


		/**********************************************************/
		//	フレンド関数
		/**********************************************************/
		//	データの読み込み
		template <typename T, typename S>
		friend void readDeg(std::istream& is, const leica::Param<S>& param, std::vector<leica::Data<T>>& data_set);
		template <typename T, typename S>
		friend void readRad(std::istream& is, const leica::Param<S>& param, std::vector<leica::Data<T>>& data_set);


		/**********************************************************/
		//	 データセット出力用label
		/**********************************************************/
		inline static std::string label()
		{
			return MyTime::label() + ",image,range[m],unknown[rad],vertical[rad],horizontal[rad],x[m],y[m],z[m],accuracy[mm]";
		};


		/**********************************************************/
		//  計測データ変数 
		/**********************************************************/
	public:
		double num;			//	番号
		MyTime time;		//	計測時間
		bool image_;		//	計測時の画像の有無
		double range;	//	距離[m]
		double inclimination;		//	わからない角度
		double vertical;	//	垂直角度[rad]
		double horizontal;	//	水平角度[rad]
		double x;			//	基準からのx移動量[m]
		double y;			//	基準からのy移動量[m]
		double z;			//	基準からのz移動量[m]
		double accuracy = 0;	//	精度[mm]

		//private:
		const Param<S>& param;	//	計測機器のパラメータ

	};


	/**********************************************************/
	//	データの読み込み
	/**********************************************************/
	template <typename T, typename S>
	void readDeg(std::istream& is, const leica::Param<S>& param, std::vector<leica::Data<T>>& data_set)
	{
		std::string str;

		std::getline(is, str);	//	１行目をスキップ

		/*  データの読み込み  */
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

		std::getline(is, str);	//	１行目をスキップ

		/*  データの読み込み  */
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
	//	入出力ストリーム
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
