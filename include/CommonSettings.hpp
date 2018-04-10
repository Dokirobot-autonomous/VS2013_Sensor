//---���ʐݒ� Ver.5.0---//
//����:
//      �e�w�b�_�ŋ��ʂ��Ďg�������
//      �ȉ��̕��͂��̃w�b�_�œǂݍ��ނ̂�#include"�`"�͂���Ȃ�
//���e:
//      �����p�̃N���X���̂̍Ē�`
//      �p�����[�^�̒�`
//      �������W�N���X(X,Y)
//      �ʒu�E�����N���X(X,Y,��)
//      ���W�ϊ��N���X
//�O�񂩂�̕ύX:
//      �N���X�E�����o���̕ύX
//      �֐��̒ǉ��E�폜
//      Ver.4.0: �N���X�𕪊����e�N���X�̃w�b�_��ǂݍ��ތ`����
//      Ver.4.1: utility.hpp�̕��ɓ��o�͊֐����������̂Ŋe�N���X�̓��o�͊֐��͔p�~ ReadLRF�����͓���Ȃ̂ł�����Ɉړ� (2012:11:22)
//               ���̊֌W�Ŋe�N���X�� ">>","<<"���Z�q���g�������o�͂ɑΉ� (2012:11:22)
//      Ver.5.0: �o�O�̌����ɂȂ�₷���������o�͎��̊p�x��(deg��rad)�ϊ�����߂� �e�L�X�g�ɂ�radian�Ŋp�x�\�L (2012:11:26)

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

//���ʂ̃p�����[�^������
//Top-URG�̃p�����[�^
namespace URGParam{
static const int MINSTEP(0);			//LRF�̌v���X�e�b�v�̍ŏ��l
static const int CENTERSTEP(540);		//�p�x0�ƂȂ�Top-URG�̌v���X�e�b�v
static const int MAXSTEP(1080);			//LRF�̌v���X�e�b�v�̍ő�l
static const double STEPDEG(0.25);		//LRF�̌v���X�e�b�v�p�x[deg]
static const double MINRANGE(100.0);	//LRF�̌v���͈͂̍ŏ��l[mm]
static const double MAXRANGE(30000.0);	//LRF�̌v���͈͂̍ő�l[mm]
}
namespace hfile{                        //���O�̋��
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
//const int LATTICESIZE(10);		//�i�q�T�C�Y

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
