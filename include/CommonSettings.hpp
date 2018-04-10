//---共通設定 Ver.5.0---//
//説明:
//      各ヘッダで共通して使われるもの
//      以下の物はこのヘッダで読み込むので#include"～"はいらない
//内容:
//      乱数用のクラス名称の再定義
//      パラメータの定義
//      直交座標クラス(X,Y)
//      位置・向きクラス(X,Y,θ)
//      座標変換クラス
//前回からの変更:
//      クラス・メンバ名の変更
//      関数の追加・削除
//      Ver.4.0: クラスを分割→各クラスのヘッダを読み込む形式に
//      Ver.4.1: utility.hppの方に入出力関数を書いたので各クラスの入出力関数は廃止 ReadLRFだけは特殊なのでこちらに移動 (2012:11:22)
//               ↑の関係で各クラスが ">>","<<"演算子を使った入出力に対応 (2012:11:22)
//      Ver.5.0: バグの原因になりやすかった入出力時の角度の(deg⇔rad)変換をやめる テキストにはradianで角度表記 (2012:11:26)

#pragma once

#define _USE_MATH_DEFINES

#include <boost/random.hpp>
typedef boost::variate_generator< boost::mt19937&,boost::uniform_int<> > RandomUniformInt;
typedef boost::variate_generator< boost::mt19937&,boost::uniform_real<> > RandomUniformReal;
typedef boost::variate_generator< boost::mt19937&,boost::normal_distribution<> > RandomNormalDist;
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>

#include"Coor.hpp"
#include"Position.hpp"
#include"TransMatrix.hpp"

//共通のパラメータいくつか
//Top-URGのパラメータ
namespace URGParam{
static const int MINSTEP(0);			//LRFの計測ステップの最小値
static const int CENTERSTEP(540);		//角度0となるTop-URGの計測ステップ
static const int MAXSTEP(1080);			//LRFの計測ステップの最大値
static const double STEPDEG(0.25);		//LRFの計測ステップ角度[deg]
static const double MINRANGE(100.0);	//LRFの計測範囲の最小値[mm]
static const double MAXRANGE(30000.0);	//LRFの計測範囲の最大値[mm]
}
namespace hfile{                        //名前の区別
inline void ReadLRF(const boost::filesystem::path &filePath, std::vector< Coor<> > &obj, Coor<> ofset = Coor<>(0,0))
{
	boost::filesystem::ifstream fin(filePath);
	if(fin.fail()){fin.close(); throw"OpenFile \""+filePath.string()+"\" NotFound";}
	Coor<> pos;
	int n, l;
	double rad;
	obj.clear();
	fin >> l >> n;
	while(!fin.eof()){
		if((l>=URGParam::MINRANGE) && (l<=URGParam::MAXRANGE)){
			rad = static_cast<double>(URGParam::MINSTEP - URGParam::CENTERSTEP + n) * URGParam::STEPDEG * M_PI / 180.0;
			pos.x = static_cast<double>(l) * cos(rad) + ofset.x;
			pos.y = static_cast<double>(l) * sin(rad) + ofset.y;
			obj.push_back(pos);
		}
		fin >> l >> n;
	}
}
}

namespace hmath{
template<typename T, typename U> inline double Distance(T a, U b){
	return std::sqrt(std::pow((static_cast<double>(a.x) - static_cast<double>(b.x)), 2) + std::pow((static_cast<double>(a.y) - static_cast<double>(b.y)), 2));
}
}
//const int LATTICESIZE(10);		//格子サイズ

template<typename T, typename U> inline T cast_XY(const U &obj){
	T ret;
	ret.x = obj.x;
	ret.y = obj.y;
	return ret;
}
template<typename T, typename U> inline T cast_XYR(const U &obj){
	T ret;
	ret.x = obj.x;
	ret.y = obj.y;
	ret.r = obj.r;
	return ret;
}
