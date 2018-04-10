//---ICPalgorithm�p�w�b�_Ver.5.0---//
//����:
//      ICP�A���S���Y�������s
//���e:
//      MapData����Ƃ�InputData�Ƃ�ICP�A���S���Y�������s
//      �ȉ��g�p���̗���
//      (1) setMapData��setInputData�Ńf�[�^���Z�b�g
//      (2) ()�Ŏ��s
//      (3) getError�Ō덷, getTrans�ňړ��ʂ𓾂�
//�O�񂩂�̕ύX:
//      Ver.4.0: �����̒ǉ�, ������
//      Ver.4.1: �֐��̐���, ������ (2012:11:14)
//      Ver.5.0: ��K�͂ȉ����̂��߃t�@�C������ICPal2�ɂ��đO�̂��͎̂c�� �ȉ����e
//               �}�b�`���O���̃I�v�V�����͔p�~ �Ή��_�͌Œ�臒l�ŏ��O
//               �Ή��̎�����point to point ���� point to line �� (�_�ƋߖT�_��_�����Ԓ����Ƃ̋���) (2012:11:27)

#pragma once
//#include "CommonSettings.hpp"
#include "svd.h"
#include "kd-tree.hpp"
#include "TransMatrix.hpp"
#include <cmath>
#include <vector>

class ICP{
private:
	//���̓f�[�^�֌W
	std::vector<Coor<> > Input;		//�v���_�Q
	std::vector<Coor<> > inputOrg;	//�v���_�Q �I���W�i�� ���W�ϊ��������̂�Input�ɑ��(�p�[�e�B�N���̓s��)

	//�Ή��_�T��
	KDtree kdtree;
	struct Pair{ //�\���� �Ή��_
		Coor<> input;
		Coor<> base;
		double distance;
		Pair():input(0,0),base(0,0),distance(0){}
		Pair(Coor<> input_,Coor<> base_,double distance_):input(input_),base(base_),distance(distance_){}
		Pair(const Pair &obj):input(obj.input),base(obj.base),distance(obj.distance){}
	};
	std::vector<Pair> pairPoint;  //���͂ɑ΂���Ή��_�̑g�ݍ��킹�S��
	std::vector<Pair> pairReject; //�Ή��_�̑g�ݍ��킹 �����ɍ���Ȃ����̂͏��O
	double Thr; //�Œ�臒l
	int pairNum; //���W�F�N�g��̃y�A�̐�

	//�o�̓f�[�^�֌W
	TransMatrix trans;		  //���݂̍��W�ϊ��p�����[�^(���݂̃X�e�b�v�ł̕ω���)
	TransMatrix currentTrans; //���݂̍��W�ϊ��p�����[�^(���݂܂ł̕ω��ʂ̍��v)
	TransMatrix leastTrans;	  //�덷���ŏ��ƂȂ���W�ϊ��p�����[�^�ˍŏI����
	double currentError;	  //���݂̓��덷
	double leastError;		  //�ŏ����덷
	int increaseCount;		  //�J��Ԃ��̍ۂɓ��덷�����������񐔂��L�^
	int decreaseCount;		  //�J��Ԃ��̍ۂɓ��덷�����������񐔂��L�^
	double lastError;		  //�O��̓��덷

	//��]�E�ړ��ʌv�Z
	Coor<> inSum,baSum; //�Ή��_�ɑI�΂ꂽ���W�l�̑��a
	double inputMyu[2],baseMyu[2]; //�d�S
	double covMatrix[2][2]; //�����U�s��

	std::vector<double> vectorW; //���ْl�����̌��ʂ̓��A�Ίp�s��D(�Ίp���������ْl�ɂȂ�)���i�[����z��
	std::vector< std::vector<double> > arrayA; //SDV�v�Z�p�z��arrayA
	std::vector< std::vector<double> > arrayV; //SVD�ŋ��܂�̒����s��V�̓]�u�s��i�[�p�z��

	const double scale;	   //�g�k�p��
	double trDS;		   //trDs�͕��ϓ��덷�̌v�Z�Ɏg�p�B2�̑Ίp�s��S��D(Vector_W)���|�������̂̑Ίp�����̘a
	double S1,S2;		   //�Ίp�s��S
	bool calcError;		   //���ْl���𐬔۔���
	bool svdComp;		   //���ْl�����̐����E�s����(svdcmp�����s�����Ƃ�false �Ή��_�̑g��0�̂Ƃ��Ȃǂɔ���)
	static const int X=0,Y=1;
public:
	//�R���X�g���N�^
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

	//�f�X�g���N�^
	//~ICP()
	//{
	//}
/*-------------------------------------------------------------------------------------------------------*/

private:
	//--�ŋߖT�_�T��--//
	//Base��Input�Ƃ̋ߖT�_��_�����Ԓ����Ƃ̑Ή������ (�����Ȃ�悤�ɉ��z�I��Base���̑Ή��_��ݒ�)
	//臒l�Œ� �ŋߖT�_�Ƃ̋�����臒l���傫�����̂͏��O����
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
		Coor<> nr; // near���ƃ}�N����`�����̂�nr�ɕύX
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

	//--�e�p�����[�^�̌v�Z--//
	void calcParameter(){
		//�d�S�v�Z
		//inputMyu[0]=0.0; inputMyu[1]=0.0;
		//baseMyu[0]=0.0; baseMyu[1]=0.0;
		//    //�Ή��_�ɑI�΂ꂽ���W�l�̑��a���Ƃ�
		//for(std::vector<Pair>::iterator itr(pairReject.begin()); itr != pairReject.end(); ++itr){
		//	inputMyu[0] += itr->input.x;
		//	inputMyu[1] += itr->input.y;
		//	baseMyu[0]	+= itr->base.x;
		//	baseMyu[1]	+= itr->base.y;
		//}
		//    //���W�l�̑��a��_�̐��Ŋ����ĕ��ς��o��
		//inputMyu[0] = inputMyu[0] / pairNum;
		//inputMyu[1] = inputMyu[1] / pairNum;
		//baseMyu[0]	= baseMyu[0]  / pairNum;
		//baseMyu[1]	= baseMyu[1]  / pairNum;
		inputMyu[X] = inSum.x / pairNum;
		inputMyu[Y] = inSum.y / pairNum;
		baseMyu[X]	= baSum.x  / pairNum;
		baseMyu[Y]	= baSum.y  / pairNum;

		//���U�v�Z
//		  double inputVar[2]={0.0,0.0},baseVar[2]={0.0,0.0};/*���U*/
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

		//�����U�s��v�Z
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
		//�����U�s��̍s�񎮂̌v�Z
		detCovMatrix = covMatrix[0][0] * covMatrix[1][1] - covMatrix[0][1] * covMatrix[1][0];
		//�Ίp�s��S(��]�s��ƕ��ϓ��덷�����߂鎞�Ɏg��)�̔���
		if(detCovMatrix >= 0){
			S1 = 1.0;
			S2 = 1.0;
		}else{
			S1 =  1.0;
			S2 = -1.0;
		}
	}

	//���ْl����(SVD)���s
	//���ْl�����̌��ʂ�(CoMa)=(Array_A)(Vector_W)(Array_V)�Ƃ��Ċi�[����
	void singulerValuDecomposition(){
		const int m= 2, n= 2; //����2�~2�̍s����l���Ă���̂�m=n=2(m�~n�s��)

		vectorW.resize((m<n)?(m+1):(n+1)); //���ْl�����̌��ʂ̓��A�Ίp�s��D(�Ίp���������ْl�ɂȂ�)���i�[����z��
                                           //���ْl���������s����֐�svdcmp�̐�����Vector_W[1]��Vector_W[2]���g����Vector_W[3]�Ɛ錾

		arrayA.resize(m+1); //SDV�v�Z�p�z��arrayA�̊m��
		for(int i(0); i<m+1; i++){
			arrayA[i].resize(n+1);
		}

		for(int i(0); i<m; i++){ for(int j(0); j<n; j++){
			arrayA[i+1][j+1] = covMatrix[i][j]; //arrayA�ɋ����U�s��covMatrix�̒l����
		}}

		arrayV.resize(n+1); //SVD�ŋ��܂�̒����s��V�̓]�u�s��i�[�p�z��
		for(int i(0); i<n+1; i++){
			arrayV[i].resize(m+1);
		}

		svdComp = nrC::svdcmp(arrayA,m,n,vectorW,arrayV); //SVD���s�p�֐�(�\�[�X��svd.h�ɍ݂�܂�)
		if(!svdComp) {return;}

		//SVD�̌��ʂ�Vector_W[1]>=Vector_W[2]�łȂ���΂Ȃ�Ȃ��ׁA�����łȂ������ꍇ�̏C���s��
		//��������-------
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
		//-------�����܂�

		trDS = vectorW[1] * S1 + vectorW[2] * S2; //�Ίp�s��D(Vector_W)�ɉE����Ίp�s��S(s1,s2)���|�������̂̑Ίp�����̑��a

		//�g�k�̌v�Z�B�g�k���l���Ȃ��ꍇ��scale=1.0�ŌŒ�
//		  double scale1 = trDS / (inputVar[0] + inputVar[1]); //�g�k���l����ꍇ�Ascale�͂��̎���苁�߂�
//		  double ss = ss * scale;                             //�J��Ԃ��v�Z���s���ċ��߂��ŏI�I�Ȋg�k�̒l�����߂�ׂ̌v�Z

		//��������ْl�����Ɏ��s�������̃t�F�C���Z�C�t
		//ICP�A���S���Y���́A�d�ˍ��킹��2�̓_�Q���m��x���Ay�������̕��U�����U�ɂ��A
		//���U�����U�s��̓��ْl�������s�����Ƃŉ�]�ړ��ʂ����߂邪�A
		//�Ή��_�̎������s�K�؂ŁA�����U�̊֌W���K�؂ɓ����Ȃ��ꍇ�A���ْl�����Ɏ��s����ꍇ������B
		//(��F�_�QA�Ɠ_�QB�����ꂽ�ʒu�ɂ���A�_�QA�̑Ή��_���S�āA�_�QB���̂���1�̓_���W�ƂȂ��Ă��܂����ꍇ)
		//���̂悤�ȏꍇ�͉�]�ړ����s�킸���i�ړ��̂ݍs��
		//�ȉ��͂��̂��߂̔��菈���B
		//���ْl�����Ɏ��s�����ꍇ�͕ϐ�calcError�̒l��false�ɂȂ�(�����true)
		calcError=true;
		for(int i(1);i<=m;i++){
			for(int j(1);j<=n;j++){
				if(_finite(arrayA[i][j])!=1 || _finite(arrayV[i][j])!=1){calcError = false;}
			}
		}
	}

	//--��]�s��v�Z�s��--//
	void calcRotation(){
		double Role[2][2]; //�v�Z�p�̃o�b�t�@
		//���݂̉�]�s��̌v�Z
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
		//�����J�n���猻�݂܂ł̍��v�̉�]�ړ��ʂ̌v�Z
		Role[0][0] = (currentTrans.getRMatrix<0,0>() * trans.getRMatrix<0,0>())+(currentTrans.getRMatrix<0,1>() * trans.getRMatrix<1,0>());
		Role[0][1] = (currentTrans.getRMatrix<0,0>() * trans.getRMatrix<0,1>())+(currentTrans.getRMatrix<0,1>() * trans.getRMatrix<1,1>());
		Role[1][0] = (currentTrans.getRMatrix<1,0>() * trans.getRMatrix<0,0>())+(currentTrans.getRMatrix<1,1>() * trans.getRMatrix<1,0>());
		Role[1][1] = (currentTrans.getRMatrix<1,0>() * trans.getRMatrix<0,1>())+(currentTrans.getRMatrix<1,1>() * trans.getRMatrix<1,1>());

		currentTrans.setRMatrix(Role);
	}

	//--���i�ړ��ʂ̌v�Z--//
	void calcTranslation(){
		trans.tx(baseMyu[0] - scale * (trans.getRMatrix<0,0>() * inputMyu[0] + trans.getRMatrix<0,1>() * inputMyu[1])); //���݂̕��i�ړ��ʂ̌v�Z(x����)
		trans.ty(baseMyu[1] - scale * (trans.getRMatrix<1,0>() * inputMyu[0] + trans.getRMatrix<1,1>() * inputMyu[1])); //���݂̕��i�ړ��ʂ̌v�Z(y����)

		double bufX(0.0),bufY(0.0); //�v�Z�p�̃o�b�t�@

		bufX = (trans.getRMatrix<0,0>() * currentTrans.tx() + trans.getRMatrix<0,1>() * currentTrans.ty());
		bufY = (trans.getRMatrix<1,0>() * currentTrans.tx() + trans.getRMatrix<1,1>() * currentTrans.ty());

		currentTrans.tx(bufX + trans.tx());	//����܂ł̍��v�̕��i�ړ��ʂ̌v�Z(x����)
		currentTrans.ty(bufY + trans.ty()); //����܂ł̍��v�̕��i�ړ��ʂ̌v�Z(y����)
	}

	//--���߂��ړ��p�����[�^(Role,Trans,scale)������Input�̓_���W��ϊ�����--//
	//void transformation(){
	//	for(std::vector< Coor<> >::iterator itr(Input.begin()); itr != Input.end(); ++itr){
	//		*itr = trans.Transform(*itr, scale);
	//	}
	//}

	//--�ŏ����덷�̌v�Z--//
	void calculationError()
	{
		//�ŏ����덷��trDS��Input��Stock�̕��U��p���ċ��߂鎖���o����
		Coor<> pos;
		currentError = 0.0;
		for(std::vector<Pair>::iterator itr(pairReject.begin()); itr != pairReject.end(); ++itr){
			pos = trans.Transform(itr->input, scale);
			currentError += pow((itr->base.x - pos.x), 2) + pow((itr->base.y - pos.y), 2);
		}
		//cout << "pairNum:" << pairNum << endl;
		currentError = currentError / pairNum;
	}

	//--�J��Ԃ��v�Z�I���̔���--//
	bool checkIteration()
	{
		const int increaseMax(10); //�덷�̑����񐔂�臒l
		const int decreaseMax(20); //�덷�̌����񐔂�臒l
		const double diffMax(1.0); //�O��ƍ���̍��̐�Βl��臒l
		double diffError;          //�O��ƍ���̍��̐�Βl
		bool check(true);          //ICP�̌J��Ԃ��I����false

		if(leastError > currentError){
			leastTrans = currentTrans;
			leastError = currentError;
			increaseCount = 0;
			++decreaseCount;
			//�덷�̌����񐔂�臒l�𒴂���
			if(decreaseCount > decreaseMax){
				check = false;
				return check;
			}
		}else{
			decreaseCount = 0;
			++increaseCount;
			//�덷�̑����񐔂�臒l�𒴂���
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

	//--������--//
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
	//ICP��� �v���_�Q�͕ʓr�w��
	void operator()(){
		reset();
		icpRoutine();
	}
	//ICP������ �v���_�Q�͕ʓr�w��
	void operator()(const int &iterate){
		bool ite(true);
		reset();
		for(int i(0); (i<iterate) && ite; ++i){
			ite = icpRoutine();
		}
	}
	//�v���_�Q���w�肵ICP(���)
	void operator()(const std::vector<Coor<>> &object){
		setInputData(object);
		reset();
		icpRoutine();
	}
	//�v���_�Q���w�肵���s�E��]�ړ���ICP(���)
	void operator()(const std::vector<Coor<>> &object, const Position<> &pos){
		setInputData(object,pos);
		reset();
		icpRoutine();
	}

	//���O臒l�̐ݒ�
	void setThreshold(const double thr){ Thr = thr; }

	//��r�̊�̐ݒ�
	void setMapData(const std::vector<Coor<>> &object){ kdtree.makeTree(object); }
	void setMapData(const std::vector <std::vector<Coor<> > > &object){ kdtree.makeTree(object); }

	//��r�Ώۂ̐ݒ�
	void setInputData(const std::vector<Coor<>> &object){
		Input.clear();
		Input = object;
	}
	//��r�Ώۂ̐ݒ� ���s�E��]�ړ����������̂�ݒ�
	void setInputData(const std::vector<Coor<>> &object, const Position<> &pos){
		Input.clear();
		Input = TransMatrix(pos).Transform(object);
	}

	//��r�Ώۂ̐ݒ� �L�[�v���Ă��� transInputDataOrg�ƃZ�b�g�Ŏg��
	void setInputDataOrg(const std::vector<Coor<>> &object){
		inputOrg.clear();
		inputOrg = object;
	}
	//inputOrg�𕽍s�E��]�ړ������Ĕ�r�ΏۂƂ��Đݒ�
	void transInputDataOrg(const Position<> &pos){
		setInputData(inputOrg, pos);
	}

	//�y�A�̐���Ԃ�
	int getPairNum() const { return pairNum; }
	//�덷��Ԃ�
	double getError() const { return leastError; }
	//�ړ��ʂ�Ԃ�
	TransMatrix getTrans() const { return leastTrans; }
	//���s�E��]�ړ��������W��Ԃ�
	std::vector<Coor<>> getTransInputData() const { return Input; };
};
