//---ICPalgorithm用ヘッダVer.5.0---//
//説明:
//      ICPアルゴリズムを実行
//内容:
//      MapDataを基準としInputDataとのICPアルゴリズムを実行
//      以下使用時の流れ
//      (1) setMapDataとsetInputDataでデータをセット
//      (2) ()で実行
//      (3) getErrorで誤差, getTransで移動量を得る
//前回からの変更:
//      Ver.4.0: 説明の追加, 微調整
//      Ver.4.1: 関数の整理, 微調整 (2012:11:14)
//      Ver.5.0: 大規模な改造のためファイル名をICPal2にして前のものは残す 以下内容
//               マッチング時のオプションは廃止 対応点は固定閾値で除外
//               対応の取り方をpoint to point から point to line へ (点と近傍点二点を結ぶ直線との距離) (2012:11:27)

#pragma once
//#include "CommonSettings.hpp"
#include "svd.h"
#include "kd-tree.hpp"
#include "TransMatrix.hpp"
#include <cmath>
#include <vector>

class ICP{
private:
	//入力データ関係
	std::vector<Coor<> > Input;		//計測点群
	std::vector<Coor<> > inputOrg;	//計測点群 オリジナル 座標変換したものをInputに代入(パーティクルの都合)

	//対応点探索
	KDtree kdtree;
	struct Pair{ //構造体 対応点
		Coor<> input;
		Coor<> base;
		double distance;
		Pair():input(0,0),base(0,0),distance(0){}
		Pair(Coor<> input_,Coor<> base_,double distance_):input(input_),base(base_),distance(distance_){}
		Pair(const Pair &obj):input(obj.input),base(obj.base),distance(obj.distance){}
	};
	std::vector<Pair> pairPoint;  //入力に対する対応点の組み合わせ全て
	std::vector<Pair> pairReject; //対応点の組み合わせ 条件に合わないものは除外
	double Thr; //固定閾値
	int pairNum; //リジェクト後のペアの数

	//出力データ関係
	TransMatrix trans;		  //現在の座標変換パラメータ(現在のステップでの変化量)
	TransMatrix currentTrans; //現在の座標変換パラメータ(現在までの変化量の合計)
	TransMatrix leastTrans;	  //誤差が最小となる座標変換パラメータ⇒最終結果
	double currentError;	  //現在の二乗誤差
	double leastError;		  //最小二乗誤差
	int increaseCount;		  //繰り返しの際に二乗誤差が増加した回数を記録
	int decreaseCount;		  //繰り返しの際に二乗誤差が減少した回数を記録
	double lastError;		  //前回の二乗誤差

	//回転・移動量計算
	Coor<> inSum,baSum; //対応点に選ばれた座標値の総和
	double inputMyu[2],baseMyu[2]; //重心
	double covMatrix[2][2]; //共分散行列

	std::vector<double> vectorW; //特異値分解の結果の内、対角行列D(対角成分が特異値になる)を格納する配列
	std::vector< std::vector<double> > arrayA; //SDV計算用配列arrayA
	std::vector< std::vector<double> > arrayV; //SVDで求まるの直交行列Vの転置行列格納用配列

	const double scale;	   //拡縮用変
	double trDS;		   //trDsは平均二乗誤差の計算に使用。2つの対角行列SとD(Vector_W)を掛けたものの対角成分の和
	double S1,S2;		   //対角行列S
	bool calcError;		   //特異値分解成否判定
	bool svdComp;		   //特異値分解の成功・不成功(svdcmpが失敗したときfalse 対応点の組が0のときなどに発生)
	static const int X=0,Y=1;
public:
	//コンストラクタ
	ICP()
	:scale(1.0)
	,trDS(0.0)
	,S1(0.0),S2(0.0)
	,increaseCount(0)
	,decreaseCount(0)
	,svdComp(true)
	,Thr(500)
	,pairNum(0)
	{
		Input.reserve(1081);
		pairPoint.reserve(1081);
		pairReject.reserve(1081);
	}

	ICP(const ICP& obj)
		:scale(obj.scale)
		, trDS(obj.trDS)
		, S1(obj.S1), S2(obj.S2)
		, increaseCount(obj.increaseCount)
		, decreaseCount(obj.decreaseCount)
		, svdComp(obj.svdComp)
		, Thr(obj.Thr)
		, pairNum(obj.pairNum)
		, kdtree(obj.kdtree)
		, inputOrg(obj.inputOrg)
	{
		Input.reserve(1081);
		pairPoint.reserve(1081);
		pairReject.reserve(1081);
	};

	ICP& operator=(const ICP& obj)
	{
		this->trDS = obj.trDS;
		this->S1 = obj.S1;
		this->S2 = obj.S2;
		this->increaseCount = obj.increaseCount;
		this->decreaseCount = obj.decreaseCount;
		this->svdComp = obj.svdComp;
		this->Thr = obj.Thr;
		this->pairNum = obj.pairNum;
		this->kdtree = obj.kdtree;
		this->inputOrg = obj.inputOrg;
		this->Input.reserve(1081);
		this->pairPoint.reserve(1081);
		this->pairReject.reserve(1081);

		return *this;
	}

	//デストラクタ
	//~ICP()
	//{
	//}
/*-------------------------------------------------------------------------------------------------------*/

private:
	//--最近傍点探索--//
	//BaseのInputとの近傍点二点を結ぶ直線との対応を取る (そうなるように仮想的にBase側の対応点を設定)
	//閾値固定 最近傍点との距離が閾値より大きいものは除外する
	Coor<> nearest(Coor<> point, const std::pair<Coor<>,Coor<> > &nearPoint){
		if(nearPoint.first==nearPoint.second)return nearPoint.first;
		Coor<> a(nearPoint.second - nearPoint.first);
		Coor<> b(point - nearPoint.first);
		double r((a.x*b.x + a.y*b.y) / (a.x*a.x + a.y*a.y));
		//if( r<=0 ) point = nearPoint.first;
		//else if( r>=1 ) point = nearPoint.second;
		//else point = nearPoint.first + (r*a);
		//return point;
		return nearPoint.first + (r*a);
	}
	void serchClosest(){
		std::pair<Coor<>,Coor<> > base;
		std::pair<double,double> dist;
		Coor<> nr; // nearだとマクロ定義されるのでnrに変更
		pairPoint.clear();
		pairReject.clear();
		inSum.clear();baSum.clear();
		for(std::vector<Coor<> >::iterator i(Input.begin()); i!=Input.end(); ++i){
			base = kdtree.searchNeighborPoint2(*i);
			dist = kdtree.getDistance2();
			nr = nearest(*i,base);
			Pair ppair(*i, nr, *i | nr);
			pairPoint.push_back(ppair);
			if(dist.first < Thr){
				inSum += ppair.input;
				baSum += ppair.base;
				pairReject.push_back(ppair);
			}
		}
		pairNum = static_cast<int>(pairReject.size());
	}

	//--各パラメータの計算--//
	void calcParameter(){
		//重心計算
		//inputMyu[0]=0.0; inputMyu[1]=0.0;
		//baseMyu[0]=0.0; baseMyu[1]=0.0;
		//    //対応点に選ばれた座標値の総和をとる
		//for(std::vector<Pair>::iterator itr(pairReject.begin()); itr != pairReject.end(); ++itr){
		//	inputMyu[0] += itr->input.x;
		//	inputMyu[1] += itr->input.y;
		//	baseMyu[0]	+= itr->base.x;
		//	baseMyu[1]	+= itr->base.y;
		//}
		//    //座標値の総和を点の数で割って平均を出す
		//inputMyu[0] = inputMyu[0] / pairNum;
		//inputMyu[1] = inputMyu[1] / pairNum;
		//baseMyu[0]	= baseMyu[0]  / pairNum;
		//baseMyu[1]	= baseMyu[1]  / pairNum;
		inputMyu[X] = inSum.x / pairNum;
		inputMyu[Y] = inSum.y / pairNum;
		baseMyu[X]	= baSum.x  / pairNum;
		baseMyu[Y]	= baSum.y  / pairNum;

		//分散計算
//		  double inputVar[2]={0.0,0.0},baseVar[2]={0.0,0.0};/*分散*/
//		  for(int i(0); i<pairNum; i++){
//			  inputVar[0] += pow(inputPair[i].x_-inputMyu[0], 2.0);
//			  inputVar[1] += pow(inputPair[i].y_-inputMyu[1], 2.0);
//			  baseVar[0] += pow(basePair[i].x_-baseMyu[0], 2.0);
//			  baseVar[1] += pow(basePair[i].y_-baseMyu[1], 2.0);
//		  }
//		  inputVar[0] /= (double)pairNum;
//		  inputVar[1] /= (double)pairNum;
//		  baseVar[0] /= (double)pairNum;
//		  baseVar[1] /= (double)pairNum;

		//共分散行列計算
		covMatrix[0][0] = 0.0;
		covMatrix[0][1] = 0.0;
		covMatrix[1][0] = 0.0;
		covMatrix[1][1] = 0.0;
		for(std::vector<Pair>::iterator itr(pairReject.begin()); itr != pairReject.end(); ++itr){
			covMatrix[0][0] += (itr->base.x - baseMyu[X]) * (itr->input.x - inputMyu[X]);
			covMatrix[0][1] += (itr->base.x - baseMyu[X]) * (itr->input.y - inputMyu[Y]);
			covMatrix[1][0] += (itr->base.y - baseMyu[Y]) * (itr->input.x - inputMyu[X]);
			covMatrix[1][1] += (itr->base.y - baseMyu[Y]) * (itr->input.y - inputMyu[Y]);
		}
		covMatrix[0][0] = covMatrix[0][0] / pairNum;
		covMatrix[0][1] = covMatrix[0][1] / pairNum;
		covMatrix[1][0] = covMatrix[1][0] / pairNum;
		covMatrix[1][1] = covMatrix[1][1] / pairNum;

		double detCovMatrix(0.0);
		//共分散行列の行列式の計算
		detCovMatrix = covMatrix[0][0] * covMatrix[1][1] - covMatrix[0][1] * covMatrix[1][0];
		//対角行列S(回転行列と平均二乗誤差を求める時に使う)の判定
		if(detCovMatrix >= 0){
			S1 = 1.0;
			S2 = 1.0;
		}else{
			S1 =  1.0;
			S2 = -1.0;
		}
	}

	//特異値分解(SVD)実行
	//特異値分解の結果を(CoMa)=(Array_A)(Vector_W)(Array_V)として格納する
	void singulerValuDecomposition(){
		const int m= 2, n= 2; //今回2×2の行列を考えているのでm=n=2(m×n行列)

		vectorW.resize((m<n)?(m+1):(n+1)); //特異値分解の結果の内、対角行列D(対角成分が特異値になる)を格納する配列
                                           //特異値分解を実行する関数svdcmpの性質上Vector_W[1]とVector_W[2]を使う為Vector_W[3]と宣言

		arrayA.resize(m+1); //SDV計算用配列arrayAの確保
		for(int i(0); i<m+1; i++){
			arrayA[i].resize(n+1);
		}

		for(int i(0); i<m; i++){ for(int j(0); j<n; j++){
			arrayA[i+1][j+1] = covMatrix[i][j]; //arrayAに共分散行列covMatrixの値を代入
		}}

		arrayV.resize(n+1); //SVDで求まるの直交行列Vの転置行列格納用配列
		for(int i(0); i<n+1; i++){
			arrayV[i].resize(m+1);
		}

		svdComp = nrC::svdcmp(arrayA,m,n,vectorW,arrayV); //SVD実行用関数(ソースはsvd.hに在ります)
		if(!svdComp) {return;}

		//SVDの結果はVector_W[1]>=Vector_W[2]でなければならない為、そうでなかった場合の修正行程
		//ここから-------
		if(vectorW[1] < vectorW[2])
		{
			double change(0.0);
			change = vectorW[1];
			vectorW[1] = vectorW[2];
			vectorW[2] = change;

			change = arrayA[1][1];
			arrayA[1][1] = arrayA[2][2];
			arrayA[2][2] = change;

			change = arrayA[1][2];
			arrayA[1][2] = arrayA[2][1];
			arrayA[2][1] = change;

			change = arrayV[1][1];
			arrayV[1][1] = arrayV[2][2];
			arrayV[2][2] = change;

			change = arrayV[1][2];
			arrayV[1][2] = arrayV[2][1];
			arrayV[2][1] = change;
		}
		//-------ここまで

		trDS = vectorW[1] * S1 + vectorW[2] * S2; //対角行列D(Vector_W)に右から対角行列S(s1,s2)を掛けたものの対角成分の総和

		//拡縮の計算。拡縮を考えない場合はscale=1.0で固定
//		  double scale1 = trDS / (inputVar[0] + inputVar[1]); //拡縮を考える場合、scaleはこの式より求める
//		  double ss = ss * scale;                             //繰り返し計算を行って求めた最終的な拡縮の値を求める為の計算

		//万が一特異値分解に失敗した時のフェイルセイフ
		//ICPアルゴリズムは、重ね合わせる2つの点群同士のx軸、y軸方向の分散共分散により、
		//分散共分散行列の特異値分解を行うことで回転移動量を求めるが、
		//対応点の取り方が不適切で、共分散の関係が適切に得られない場合、特異値分解に失敗する場合がある。
		//(例：点群Aと点群Bが離れた位置にあり、点群Aの対応点が全て、点群B中のある1つの点座標となってしまった場合)
		//このような場合は回転移動を行わず並進移動のみ行う
		//以下はそのための判定処理。
		//特異値分解に失敗した場合は変数calcErrorの値がfalseになる(正常はtrue)
		calcError=true;
		for(int i(1);i<=m;i++){
			for(int j(1);j<=n;j++){
				if(_finite(arrayA[i][j])!=1 || _finite(arrayV[i][j])!=1){calcError = false;}
			}
		}
	}

	//--回転行列計算行程--//
	void calcRotation(){
		double Role[2][2]; //計算用のバッファ
		//現在の回転行列の計算
		if(calcError){
			for(int i(0); i<=1; i++){for(int j(0); j<=1; j++){
				Role[i][j] = (arrayA[i+1][1] * S1 * arrayV[j+1][1]) + (arrayA[i+1][2] * S2 * arrayV[j+1][2]);
				trans.setRMatrix(Role);
			}}
		}else{
			Role[0][0] = 0; Role[0][1] = -1;
			Role[1][0] = 1; Role[1][1] =  0;
			trans.setRMatrix(Role);
		}

		if(trans.getRMatrix<0,0>()<0 && trans.getRMatrix<1,1>()>0 || trans.getRMatrix<0,0>()>0 && trans.getRMatrix<1,1>()<0){
			calcError = false;
			Role[0][0] = 1; Role[0][1] = 0;
			Role[1][0] = 0; Role[1][1] = 1;
			trans.setRMatrix(Role);
		}
		//処理開始から現在までの合計の回転移動量の計算
		Role[0][0] = (currentTrans.getRMatrix<0,0>() * trans.getRMatrix<0,0>())+(currentTrans.getRMatrix<0,1>() * trans.getRMatrix<1,0>());
		Role[0][1] = (currentTrans.getRMatrix<0,0>() * trans.getRMatrix<0,1>())+(currentTrans.getRMatrix<0,1>() * trans.getRMatrix<1,1>());
		Role[1][0] = (currentTrans.getRMatrix<1,0>() * trans.getRMatrix<0,0>())+(currentTrans.getRMatrix<1,1>() * trans.getRMatrix<1,0>());
		Role[1][1] = (currentTrans.getRMatrix<1,0>() * trans.getRMatrix<0,1>())+(currentTrans.getRMatrix<1,1>() * trans.getRMatrix<1,1>());

		currentTrans.setRMatrix(Role);
	}

	//--並進移動量の計算--//
	void calcTranslation(){
		trans.tx(baseMyu[0] - scale * (trans.getRMatrix<0,0>() * inputMyu[0] + trans.getRMatrix<0,1>() * inputMyu[1])); //現在の並進移動量の計算(x軸側)
		trans.ty(baseMyu[1] - scale * (trans.getRMatrix<1,0>() * inputMyu[0] + trans.getRMatrix<1,1>() * inputMyu[1])); //現在の並進移動量の計算(y軸側)

		double bufX(0.0),bufY(0.0); //計算用のバッファ

		bufX = (trans.getRMatrix<0,0>() * currentTrans.tx() + trans.getRMatrix<0,1>() * currentTrans.ty());
		bufY = (trans.getRMatrix<1,0>() * currentTrans.tx() + trans.getRMatrix<1,1>() * currentTrans.ty());

		currentTrans.tx(bufX + trans.tx());	//これまでの合計の並進移動量の計算(x軸側)
		currentTrans.ty(bufY + trans.ty()); //これまでの合計の並進移動量の計算(y軸側)
	}

	//--求めた移動パラメータ(Role,Trans,scale)を元にInputの点座標を変換する--//
	//void transformation(){
	//	for(std::vector< Coor<> >::iterator itr(Input.begin()); itr != Input.end(); ++itr){
	//		*itr = trans.Transform(*itr, scale);
	//	}
	//}

	//--最小二乗誤差の計算--//
	void calculationError()
	{
		//最小二乗誤差はtrDSとInputとStockの分散を用いて求める事が出来る
		Coor<> pos;
		currentError = 0.0;
		for(std::vector<Pair>::iterator itr(pairReject.begin()); itr != pairReject.end(); ++itr){
			pos = trans.Transform(itr->input, scale);
			currentError += pow((itr->base.x - pos.x), 2) + pow((itr->base.y - pos.y), 2);
		}
		//cout << "pairNum:" << pairNum << endl;
		currentError = currentError / pairNum;
	}

	//--繰り返し計算終了の判定--//
	bool checkIteration()
	{
		const int increaseMax(10); //誤差の増加回数の閾値
		const int decreaseMax(20); //誤差の減少回数の閾値
		const double diffMax(1.0); //前回と今回の差の絶対値の閾値
		double diffError;          //前回と今回の差の絶対値
		bool check(true);          //ICPの繰り返し終了→false

		if(leastError > currentError){
			leastTrans = currentTrans;
			leastError = currentError;
			increaseCount = 0;
			++decreaseCount;
			//誤差の減少回数が閾値を超えた
			if(decreaseCount > decreaseMax){
				check = false;
				return check;
			}
		}else{
			decreaseCount = 0;
			++increaseCount;
			//誤差の増加回数が閾値を超えた
			if(increaseCount > increaseMax){
				check = false;
				return check;
			}
		}

		diffError = fabs(currentError - lastError);
		if(diffError < diffMax){
			check = false;
			return check;
		}
		lastError = currentError;

		return check;
	}

	//--初期化--//
	void reset()
	{
		currentTrans.set(0.0,0.0,0.0);
		leastTrans.set(0.0,0.0,0.0);
		increaseCount = 0;
		decreaseCount = 0;
		leastError = HUGE_VAL;
		lastError = HUGE_VAL;
	}

	bool icpRoutine(){
		serchClosest();
		calcParameter();
		singulerValuDecomposition();
		if(!svdComp) {reset(); return false;}
		calcRotation();
		calcTranslation();
		Input = trans.Transform(Input, scale);
		//transformation();
		calculationError();
		return checkIteration();
	}

public:
	//ICP一回 計測点群は別途指定
	void operator()(){
		reset();
		icpRoutine();
	}
	//ICP複数回 計測点群は別途指定
	void operator()(const int &iterate){
		bool ite(true);
		reset();
		for(int i(0); (i<iterate) && ite; ++i){
			ite = icpRoutine();
		}
	}
	//計測点群を指定しICP(一回)
	void operator()(const std::vector<Coor<>> &object){
		setInputData(object);
		reset();
		icpRoutine();
	}
	//計測点群を指定し平行・回転移動後ICP(一回)
	void operator()(const std::vector<Coor<>> &object, const Position<> &pos){
		setInputData(object,pos);
		reset();
		icpRoutine();
	}

	//除外閾値の設定
	void setThreshold(const double thr){ Thr = thr; }

	//比較の基準の設定
	void setMapData(const std::vector<Coor<>> &object){ kdtree.makeTree(object); }
	void setMapData(const std::vector <std::vector<Coor<> > > &object){ kdtree.makeTree(object); }

	//比較対象の設定
	void setInputData(const std::vector<Coor<>> &object){
		Input.clear();
		Input = object;
	}
	//比較対象の設定 平行・回転移動させたものを設定
	void setInputData(const std::vector<Coor<>> &object, const Position<> &pos){
		Input.clear();
		Input = TransMatrix(pos).Transform(object);
	}

	//比較対象の設定 キープしておく transInputDataOrgとセットで使う
	void setInputDataOrg(const std::vector<Coor<>> &object){
		inputOrg.clear();
		inputOrg = object;
	}
	//inputOrgを平行・回転移動させて比較対象として設定
	void transInputDataOrg(const Position<> &pos){
		setInputData(inputOrg, pos);
	}

	//ペアの数を返す
	int getPairNum() const { return pairNum; }
	//誤差を返す
	double getError() const { return leastError; }
	//移動量を返す
	TransMatrix getTrans() const { return leastTrans; }
	//平行・回転移動した座標を返す
	std::vector<Coor<>> getTransInputData() const { return Input; };
};
