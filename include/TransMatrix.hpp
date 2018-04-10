//座標変換クラス
#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
#include <sstream>
#include <vector>
//#include <boost/filesystem/fstream.hpp>

class TransMatrix{
private:
	double tx_;		//平行移動量
	double ty_;		//平行移動量
	double r[2][2]; //回転行列
	double angle_;	//回転角 ↑と同一 [rad]
public:
	/*コンストラクタ*/
	TransMatrix()
	:tx_(0.0), ty_(0.0), angle_(0.0)
	{
		r[0][0] = 1.0; r[0][1] = 0.0;
		r[1][0] = 0.0; r[1][1] = 1.0;
	}
	TransMatrix(double x, double y, double radian)
	:tx_(x), ty_(y), angle_(radian){
		r[0][0] = std::cos(angle_); r[0][1] = -std::sin(angle_);
		r[1][0] = std::sin(angle_); r[1][1] =  std::cos(angle_);
	}
	template<typename T> TransMatrix(const T &dat)
	:tx_(dat.x), ty_(dat.y), angle_(dat.r){
		r[0][0] = std::cos(angle_); r[0][1] = -std::sin(angle_);
		r[1][0] = std::sin(angle_); r[1][1] =  std::cos(angle_);
	}
	TransMatrix(double x, double y, const double role[2][2])
	:tx_(x),ty_(y){
		angle_ = std::atan2(role[1][0], role[0][0]);
		r[0][0] = role[0][0]; r[0][1] = role[0][1];
		r[1][0] = role[1][0]; r[1][1] = role[1][1];
	}
//コピーコンストラクタ
	TransMatrix(const TransMatrix &obj){
		tx_ = obj.tx_; ty_ = obj.ty_; angle_ = obj.angle_;
		r[0][0] = obj.r[0][0]; r[0][1] = obj.r[0][1];
		r[1][0] = obj.r[1][0]; r[1][1] = obj.r[1][1];
	}
//代入演算子
	TransMatrix& operator=(const TransMatrix &obj){
		if(this == &obj){return *this;}
		tx_ = obj.tx_; ty_ = obj.ty_; angle_ = obj.angle_;
		r[0][0] = obj.r[0][0]; r[0][1] = obj.r[0][1];
		r[1][0] = obj.r[1][0]; r[1][1] = obj.r[1][1];
		return *this;
	}
//代入
	void set(double x, double y, double radian){
		//double rad(theta*M_PI/180.0);
		tx_ = x, ty_ = y; angle_ = radian;
		double c(std::cos(radian)),s(std::sin(radian));
		r[0][0] = c; r[0][1] = -s;
		r[1][0] = s; r[1][1] =	c;
	}
	void set(double x, double y, const double role[2][2]){
		tx_ = x, ty_ = y;
		angle_ = std::atan2(role[1][0], role[0][0]);
		r[0][0] = role[0][0]; r[0][1] = role[0][1];
		r[1][0] = role[1][0]; r[1][1] = role[1][1];
	}
//アクセッサ
	void tx(double x){ tx_ = x; }
	double tx()const{ return tx_; }
	void ty(double y){ ty_ = y; }
	double ty()const{ return ty_; }
	double radian(double radian){
		angle_ = radian;
		double c(std::cos(radian)),s(std::sin(radian));
		r[0][0] = c; r[0][1] = -s;
		r[1][0] = s; r[1][1] =	c;
	}
	void setRMatrix(const double role[2][2])
	{
		angle_ = std::atan2(role[1][0], role[0][0]);
		r[0][0] = role[0][0]; r[0][1] = role[0][1];
		r[1][0] = role[1][0]; r[1][1] = role[1][1];
	}
	template<int row, int column> double getRMatrix()const{ return r[row][column]; }
//角度の取得
	double degree()const{ return (angle_ * 180 / M_PI); }
	double radian()const{ return angle_; }
//座標変換
	template<typename T> T Transform(const T &obj, double scale = 1.0) const {
		T tr;
		tr.x = scale * (r[0][0] * obj.x + r[0][1] * obj.y) + tx_;
		tr.y = scale * (r[1][0] * obj.x + r[1][1] * obj.y) + ty_;
		tr.r = obj.r + angle_;
		return tr;
	}
	template<typename T> Coor<T> Transform(const Coor<T> &obj, double scale = 1.0) const {
		Coor<T> tr;
		tr.x = static_cast<T>(scale * (r[0][0] * obj.x + r[0][1] * obj.y) + tx_);
		tr.y = static_cast<T>(scale * (r[1][0] * obj.x + r[1][1] * obj.y) + ty_);
		return tr;
	}
	template<typename T> std::vector<T> Transform(const std::vector<T> &obj, double scale = 1.0) const {
		std::vector<T> tr;
		tr.reserve(obj.size());
		for(std::vector<T>::const_iterator i(obj.begin()); i != obj.end(); ++i){
			tr.push_back(Transform(*i,scale));
		}
		return tr;
	}
/*
//文字列変換 ※(rad→deg)
	std::string ToString() const {
		std::stringstream ss;
		//ss.setf(std::ios::fixed);
		ss<< tx_ << '\t' << ty_ << '\t' << (angle_ * 180 / M_PI);
		return ss.str();
	}
*/
	std::string ToString() const {
		std::stringstream ss;
		ss<< tx_ << '\t' << ty_ << '\t' << angle_;
		return ss.str();
	}
/*
//readTransMatrix
	static void Read(const boost::filesystem::path &filePath, std::vector<TransMatrix> &obj){
		boost::filesystem::ifstream fin(filePath);
		if(fin.fail()){fin.close(); throw"OpenFile \""+filePath.string()+"\" NotFound";}
		double x, y, th;
		obj.clear();
		fin >> x >> y >> th;
		while(!fin.eof()){
			th *= (M_PI/180.0);
			obj.push_back(std::vector<TransMatrix>::value_type(x, y, th));
			fin >> x >> y >> th;
		}
	}
//writeTransMatrix
	static void Write(const boost::filesystem::path &filePath,const std::vector<TransMatrix> &obj, const std::ios_base::openmode &mode = std::ios::out){
		boost::filesystem::ofstream fout(filePath, mode);
		fout.setf(std::ios::fixed);
		for(std::vector<TransMatrix>::const_iterator itr(obj.begin()); itr != obj.end(); ++itr){
			fout << itr.tx_ << '\t' << itr.ty_ << "\t" << itr.degree() << '\n';
		}
		fout  << std::flush;
	}
*/
};

// 入力用 演算子オーバーロード
inline std::istream& operator>>(std::istream &is, TransMatrix &obj){
	double x, y, r;
	is >> x >> y >> r;
	//obj.set(x,y,r*(M_PI/180.0));
	obj.set(x,y,r);
	return is;
}

// 出力用 演算子オーバーロード
inline std::ostream& operator<<(std::ostream &os, const TransMatrix &obj){
	//return ( os << obj.tx() << '\t' << obj.ty() << '\t' << obj.degree() );
	return ( os << obj.tx() << '\t' << obj.ty() << '\t' << obj.radian() );
}
