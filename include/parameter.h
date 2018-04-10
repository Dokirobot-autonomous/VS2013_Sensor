#pragma once

#include <string>
#include "myfun.h"
#include "mycv.h"

/**********************************************************/
// ��������
/**********************************************************/

/* ������ */
#define LOCALIZATION_PARKING 0
#define LOCALIZATION_SQUARE 1
#define LOCALIZATION_B2 2
#define LOCALIZATION_AROUND_8GO 3
#define LOCALIZATION_AREA LOCALIZATION_SQUARE

/* ���z�̒�グ */
#define ADD_BIAS true

/* Test */
#define TEST 0

/* INTEGRATE TIME */
enum IntegrateType
{
	AND = 0,
	OR = 1,
};
#define INTEGRATE_TYPE AND

/* Visualize After or Before Resampling*/
enum VisualizeAfterorBeforeResampling
{
	AFTER_RESAMPLING = 0,
	BEFORE_RESAMPLING = 1,
};
#define VISUALIZE_AFTER_OR_BEFORE_RESAMPLING BEFORE_RESAMPLING

/* Omni Camera BoF */
#define USE_BOF 1
#define BOF_ROTATION 0

/* ���s���[�h */
enum TrialType
{
	TRIAL_SIMULTANEOUS = 0,
	TRIAL_PEARSON = 1,
	TRIAL_PEARSON_NONSTAT = 2,
	TRIAL_NON_TIMESEQUENCE = 3,
	TRIAL_NON_TIMESEQUENCE_SIMUL = 4,
	TRIAL_SUYAMA_STAT=5,
	TRIAL_SUYAMA_NONSTAT = 6,
	TRIAL_3SENSORS_SIMULATNEOUS = 7,
	TRIAL_3SENSORS_PEARSON = 8,
	TRIAL_3SENSORS_PEARSON_NONSTAT = 9,
	TRIAL_3SENSORS_SUYAMA_NONSTAT = 10,
	TRIAL_3SENSORS_LRF_GPS=11,
};
#define TRIAL_TYPE TRIAL_3SENSORS_PEARSON

/* �J���������� */
enum OMNI_FEATURE_TYPE
{
	OMNI_FEATURE_SIFT = 0,
	OMNI_FEATURE_SURF = 1,

};
#define OMNI_FEATURE OMNI_FEATURE_SIFT

#define W_BIAS 1E-10

/* ���s�� */
#define FIRST_OUTPUT 10
#define LAST_OUTPUT FIRST_OUTPUT

/*  �J�n�X�e�b�v�ƏI���X�e�b�v  */
#define FIRST_STEP 1
#define FIRST_NO 1
#define LAST_STEP 10// �Ō�܂Ŏ��s�������ꍇ��10000�ɂ���
//#define LAST_STEP 1 // �Ō�܂Ŏ��s�������ꍇ��10000�ɂ���
#define LAST_NO 1
#define TRUE_IDX_INI 1
#define TRUE_IDX_LAST 100
#define SKIP_STEPS 1

/*  �p�[�e�B�N���t�B���^�֘A  */
#define SAMPLE_SIZE 100    // �U�z����p�[�e�B�N����
//#define SAMPLE_SIZE 250    // �U�z����p�[�e�B�N����
//#define SAMPLE_SIZE 500    // �U�z����p�[�e�B�N����
// �����p�[�e�B�N���̃T���v���͈�
//#define INI_SAMPLE_RADIUS_X 2000.0 // �����p�[�e�B�N���̃T���v�����a[mm]
#define INI_SAMPLE_RADIUS_X 5000.0 // �����p�[�e�B�N���̃T���v�����a[mm]
#define INI_SAMPLE_RADIUS_Y INI_SAMPLE_RADIUS_X
#define INI_SAMPLE_RADIUS_R M_PI/64.0
//#define INI_SAMPLE_RADIUS_R 0.0000001
// �p�[�e�B�N���J�ڎ��ɕt������V�X�e���m�C�Y
#define TRANS_PAR_SYS_VAR_X 100.0  // x�̌덷���U[mm]
//#define TRANS_PAR_SYS_VAR_X 200.0  // x�̌덷���U[mm]
//#define TRANS_PAR_SYS_VAR_X 1000.0  // x�̌덷���U[mm]
#define TRANS_PAR_SYS_VAR_Y TRANS_PAR_SYS_VAR_X  // y�̌덷���U[mm]
#define TRANS_PAR_SYS_VAR_R M_PI/64.0 // r�̌덷���U[rad]
//#define TRANS_PAR_SYS_VAR_X 500.0  // x�̌덷���U[mm]
//#define TRANS_PAR_SYS_VAR_Y TRANS_PAR_SYS_VAR_X  // y�̌덷���U[mm]
//#define TRANS_PAR_SYS_VAR_R M_PI/16.0 // r�̌덷���U[rad]

/*  �ގ����]���p�p�[�e�B�N���̃T���v���T�C�Y */
#define STAT_SAMPLE_SIZE 500
//#define STAT_SAMPLE_SIZE 250
#define STAT_SAMPLE_RADIUS_X 5000.0 // ����p�p�[�e�B�N�����a[mm]
#define STAT_SAMPLE_RADIUS_Y STAT_SAMPLE_RADIUS_X
#define STAT_SAMPLE_RADIUS_R M_PI/64.0 // �p�x�̂݁A�ʒu����p�[�e�B�N���̍ő�E�ŏ�+STAT_SAMPLE_RADIUS_R
//#define STAT_SAMPLE_RADIUS_R M_PI/2.0

/* ����������LRF�݂̂Ő��� */
#define ESTIMATE_THETA_ONLY_LRF 1

/*  ���f�[�^�֘A  */
// parking
#if LOCALIZATION_AREA==LOCALIZATION_PARKING
//const std::string ENVIRONMENT_DATE = "171031"; // ���̓t�@�C���̃f�B���N�g��
//const std::string ENVIRONMENT_TIME = "1237";
//const std::string ENVIRONMENT_DATE_OMNI = "171209"; // ���̓t�@�C���̃f�B���N�g��
//const std::string ENVIRONMENT_TIME_OMNI = "1204";
//#define MAP_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
//#define MAP_ORG_LON 137063850 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
//#define MAP_ORG_ELE 150 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define MAP_ORG_HEAD 0 // �n�}�̌��_�̏����p�x�i�k��0���Ƃ��Ď��v���𐳁C�قڑS�Ă̒n�}��0�j
//#define MAP_IMG_ORG_X 0.2 // �n�}�摜�̌��_�i���K���j
//#define MAP_IMG_ORG_Y 0.5 // �n�}�摜�̌��_�i���K���j
//#define MAP_RES 50 // �n�}�摜�̕���\[mm]
/* parking */
const std::string ENVIRONMENT_DATE = "171031"; // ���̓t�@�C���̃f�B���N�g��
const std::string ENVIRONMENT_TIME = "1237";
const std::string ENVIRONMENT_DATE_OMNI = "171213"; // ���̓t�@�C���̃f�B���N�g��
const std::string ENVIRONMENT_TIME_OMNI = "1519";
#define MAP_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
#define MAP_ORG_LON 137063850 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
#define MAP_ORG_ELE 150 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
#define MAP_ORG_HEAD 0 // �n�}�̌��_�̏����p�x�i�k��0���Ƃ��Ď��v���𐳁C�قڑS�Ă̒n�}��0�j
#define MAP_IMG_ORG_X 0.2 // �n�}�摜�̌��_�i���K���j
#define MAP_IMG_ORG_Y 0.5 // �n�}�摜�̌��_�i���K���j
#define MAP_RES 50 // �n�}�摜�̕���\[mm]
///* parking */
//const std::string ENVIRONMENT_DATE = "171031"; // ���̓t�@�C���̃f�B���N�g��
//const std::string ENVIRONMENT_TIME = "1237";
//const std::string ENVIRONMENT_DATE_OMNI = "171031"; // ���̓t�@�C���̃f�B���N�g��
//const std::string ENVIRONMENT_TIME_OMNI = "1237";
//#define MAP_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
//#define MAP_ORG_LON 137063850 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
//#define MAP_ORG_ELE 150 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define MAP_ORG_HEAD 0 // �n�}�̌��_�̏����p�x�i�k��0���Ƃ��Ď��v���𐳁C�قڑS�Ă̒n�}��0�j
//#define MAP_IMG_ORG_X 0.5 // �n�}�摜�̌��_�i���K���j
//#define MAP_IMG_ORG_Y 0.5 // �n�}�摜�̌��_�i���K���j
//#define MAP_RES 100 // �n�}�摜�̕���\[mm]
///* parking */
//const std::string ENVIRONMENT_DATE = "171031"; // ���̓t�@�C���̃f�B���N�g��
//const std::string ENVIRONMENT_TIME = "1237_100";
//const std::string ENVIRONMENT_DATE_OMNI = "171213"; // ���̓t�@�C���̃f�B���N�g��
//const std::string ENVIRONMENT_TIME_OMNI = "1519";
//#define MAP_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
//#define MAP_ORG_LON 137063850 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
//#define MAP_ORG_ELE 150 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define MAP_ORG_HEAD 0 // �n�}�̌��_�̏����p�x�i�k��0���Ƃ��Ď��v���𐳁C�قڑS�Ă̒n�}��0�j
//#define MAP_IMG_ORG_X 0.2 // �n�}�摜�̌��_�i���K���j
//#define MAP_IMG_ORG_Y 0.5 // �n�}�摜�̌��_�i���K���j
//#define MAP_RES 100 // �n�}�摜�̕���\[mm]
#endif 
// square
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE
//const std::string ENVIRONMENT_DATE = "171115"; // ���̓t�@�C���̃f�B���N�g��
//const std::string ENVIRONMENT_TIME = "1536";
////const std::string ENVIRONMENT_DATE_OMNI = "171209"; // ���̓t�@�C���̃f�B���N�g��
////const std::string ENVIRONMENT_TIME_OMNI = "1145";
//#define BOF_A 1.816E+10 
//#define BOF_B -25.00
//#define MAP_ORG_LAT 35110390 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
//#define MAP_ORG_LON 137064840 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
//#define MAP_ORG_ELE 170 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define MAP_ORG_HEAD 0 // �n�}�̌��_�̏����p�x�i�k��0���Ƃ��Ď��v���𐳁C�قڑS�Ă̒n�}��0�j
//#define MAP_IMG_ORG_X 0.5 // �n�}�摜�̌��_�i���K���j
//#define MAP_IMG_ORG_Y 0.5 // �n�}�摜�̌��_�i���K���j
//#define MAP_RES 50 // �n�}�摜�̕���\[mm]
const std::string ENVIRONMENT_DATE = "171115"; // ���̓t�@�C���̃f�B���N�g��
const std::string ENVIRONMENT_TIME = "1536";
const std::string ENVIRONMENT_DATE_OMNI = "180407"; // ���̓t�@�C���̃f�B���N�g��
const std::string ENVIRONMENT_TIME_OMNI = "1404";
#define BOF_A 3.8452E+13
#define BOF_B -44.27
//#define BOF_A 168423.8
//#define BOF_B -6.31
#define MAP_ORG_LAT 35110390 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
#define MAP_ORG_LON 137064840 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
#define MAP_ORG_ELE 170 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
#define MAP_ORG_HEAD 0 // �n�}�̌��_�̏����p�x�i�k��0���Ƃ��Ď��v���𐳁C�قڑS�Ă̒n�}��0�j
#define MAP_IMG_ORG_X 0.5 // �n�}�摜�̌��_�i���K���j
#define MAP_IMG_ORG_Y 0.5 // �n�}�摜�̌��_�i���K���j
#define MAP_RES 50 // �n�}�摜�̕���\[mm]
#endif
// b2f 
#if LOCALIZATION_AREA==LOCALIZATION_B2
const std::string ENVIRONMENT_DATE = "171213"; // ���̓t�@�C���̃f�B���N�g��
const std::string ENVIRONMENT_TIME = "1700";
const std::string ENVIRONMENT_DATE_OMNI = "171213"; // ���̓t�@�C���̃f�B���N�g��
const std::string ENVIRONMENT_TIME_OMNI = "1700";
#define MAP_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
#define MAP_ORG_LON 137063850 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
#define MAP_ORG_ELE 170 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
#define MAP_ORG_HEAD 0 // �n�}�̌��_�̏����p�x�i�k��0���Ƃ��Ď��v���𐳁C�قڑS�Ă̒n�}��0�j
#define MAP_IMG_ORG_X 0.5 // �n�}�摜�̌��_�i���K���j
#define MAP_IMG_ORG_Y 0.5 // �n�}�摜�̌��_�i���K���j
#define MAP_RES 50 // �n�}�摜�̕���\[mm]
#endif
// 8���َ���
#if LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO
const std::string ENVIRONMENT_DATE = "180209"; // ���̓t�@�C���̃f�B���N�g��
const std::string ENVIRONMENT_TIME = "1404";
const std::string ENVIRONMENT_DATE_OMNI = "180209"; // ���̓t�@�C���̃f�B���N�g��
const std::string ENVIRONMENT_TIME_OMNI = "1404";
#define MAP_ORG_LAT 35110367 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j35110367
#define MAP_ORG_LON 137064898 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j137064898
#define MAP_ORG_ELE 170 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
#define MAP_ORG_HEAD 0 // �n�}�̌��_�̏����p�x�i�k��0���Ƃ��Ď��v���𐳁C�قڑS�Ă̒n�}��0�j
#define MAP_IMG_ORG_X 0.5 // �n�}�摜�̌��_�i���K���j
#define MAP_IMG_ORG_Y 0.8 // �n�}�摜�̌��_�i���K���j
#define MAP_RES 50 // �n�}�摜�̕���\[mm]
#endif


/*  �v���f�[�^�֘A  */
// �n�}�f�[�^�쐬����leica�ɕ����o�C�A�X���������ꍇ�C���������������Ă�������
#if LOCALIZATION_AREA==LOCALIZATION_PARKING
/* parking */
const std::string MEASUREMENT_DATE = "171031"; // ���̓t�@�C���̃f�B���N�g��
const std::string MEASUREMENT_TIME = "1237";
#define LEICA_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx
#define LEICA_ORG_LON 137063850// �n�}�̌��_�̌o�x
#define LEICA_ORG_ELE 150 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
#define LEICA_HORIZONTAL_ERROR -112+0.05/M_PI*180  // LEICA��0�����w���Ƃ��̖k����̊p�x
///* parking */
//const std::string MEASUREMENT_DATE = "171031"; // ���̓t�@�C���̃f�B���N�g��
//const std::string MEASUREMENT_TIME = "1223";
//#define LEICA_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx
//#define LEICA_ORG_LON 137063850// �n�}�̌��_�̌o�x
//#define LEICA_ORG_ELE 150 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define LEICA_HORIZONTAL_ERROR -112 // LEICA��0�����w���Ƃ��̖k����̊p�x
//#define LEICA_HORIZONTAL_ERROR -260+0.05/M_PI*180.0 // LEICA��0�����w���Ƃ��̖k����̊p�x
/* parking */
//const std::string MEASUREMENT_DATE = "171217"; // ���̓t�@�C���̃f�B���N�g��
//const std::string MEASUREMENT_TIME = "0914";
//#define LEICA_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx
//#define LEICA_ORG_LON 137063850// �n�}�̌��_�̌o�x
//#define LEICA_ORG_ELE 150 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define LEICA_HORIZONTAL_ERROR 288.439 - 323 // LEICA��0�����w���Ƃ��̖k����̊p�x
#endif
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE
/* square */
const std::string MEASUREMENT_DATE = "171031"; // ���̓t�@�C���̃f�B���N�g��
const std::string MEASUREMENT_TIME = "1200";
#define LEICA_ORG_LAT 35110360 // �n�}�̌��_�̈ܓx
#define LEICA_ORG_LON 137064845// �n�}�̌��_�̌o�x
#define LEICA_ORG_ELE 170 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
#define LEICA_HORIZONTAL_ERROR 68 // LEICA��0�����w���Ƃ��̖k����̊p�x
///* square */
//const std::string MEASUREMENT_DATE = "171209"; // ���̓t�@�C���̃f�B���N�g��
//const std::string MEASUREMENT_TIME = "1145";
//#define LEICA_ORG_LAT 35110390 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
//#define LEICA_ORG_LON 137064840 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
//#define LEICA_ORG_ELE 170 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define LEICA_HORIZONTAL_ERROR 68 // LEICA��0�����w���Ƃ��̖k����̊p�x
///* square */
//const std::string MEASUREMENT_DATE = "171207"; // ���̓t�@�C���̃f�B���N�g��
//const std::string MEASUREMENT_TIME = "1310";
//#define LEICA_ORG_LAT 35110390 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
//#define LEICA_ORG_LON 137064840 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
//#define LEICA_ORG_ELE 170 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define LEICA_HORIZONTAL_ERROR 150.554-169 // LEICA��0�����w���Ƃ��̖k����̊p�x
///* square */
//const std::string MEASUREMENT_DATE = "180407"; // ���̓t�@�C���̃f�B���N�g��
//const std::string MEASUREMENT_TIME = "1404";
//#define LEICA_ORG_LAT 35110390 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
//#define LEICA_ORG_LON 137064840 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
//#define LEICA_ORG_ELE 170 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define LEICA_HORIZONTAL_ERROR 211.688-178 // LEICA��0�����w���Ƃ��̖k����̊p�x
#endif
#if LOCALIZATION_AREA==LOCALIZATION_B2
/* b2f */
const std::string MEASUREMENT_DATE = "171213"; // ���̓t�@�C���̃f�B���N�g��
const std::string MEASUREMENT_TIME = "1700";
#define LEICA_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx
#define LEICA_ORG_LON 137063850// �n�}�̌��_�̌o�x
#define LEICA_ORG_ELE 150 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
#define LEICA_HORIZONTAL_ERROR -113 // LEICA��0�����w���Ƃ��̖k����̊p�x
//const std::string MEASUREMENT_DATE = "171216"; // ���̓t�@�C���̃f�B���N�g��
//const std::string MEASUREMENT_TIME = "2320";
//#define LEICA_ORG_LAT 35110177 // �n�}�̌��_�̈ܓx
//#define LEICA_ORG_LON 137063850// �n�}�̌��_�̌o�x
//#define LEICA_ORG_ELE 150 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
//#define LEICA_HORIZONTAL_ERROR -23 // LEICA��0�����w���Ƃ��̖k����̊p�x
#endif
// 8���َ���
#if LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO
const std::string MEASUREMENT_DATE = "180209"; // ���̓t�@�C���̃f�B���N�g��
const std::string MEASUREMENT_TIME = "1724";
#define LEICA_ORG_LAT 35110367 // �n�}�̌��_�̈ܓx�i�n�}��������Leica�ʒu�j
#define LEICA_ORG_LON 137064898 // �n�}�̌��_�̌o�x�i�n�}��������Leica�ʒu�j
#define LEICA_ORG_ELE 170 // �n�}�̌��_�̍��x�i�����ł͂��̒l�ň��ƌ��Ȃ��j
#define LEICA_HORIZONTAL_ERROR  128.891-26 // LEICA��0�����w���Ƃ��̖k����̊p�x  �L�����u���[�V�����l-googlemap�ileica���烍�{�b�g�̊p�x)
#endif


/* �r�f�I���� */
#define MODE_LINIER_INTERPOLATION 1
#define SHOW_MOVIES 0
#define MOVIE_SCALE_W 1
#define MOVIE_SCALE_H 1
#define FPS 1
#define FOURCC CV_FOURCC('X', 'V', 'I', 'D')
#define MOVIE_CREATER_SLEEP_MILLISECONDS 100
#define MOVIE_CREATOR_TIMEOUT_MILLISECONDS 100000
// �v���f�[�^
#define MEASUREMENT_DATA_VIDEO_FPS FPS
#define MEASUREMENT_DATA_VIDEO_FOURCC FOURCC
#define MEASUREMENT_DATA_VIDEO_SCAN_RADIUS 3
#define MEASUREMENT_DATA_VIDEO_SCAN_COLOR cv::Scalar(255,0,0)
#define MEASUREMENT_DATA_VIDEO_IMG_POS_RADIUS 3
#define MEASUREMENT_DATA_VIDEO_IMG_POS_COLOR cv::Scalar(255,0,0)
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_POINT cv::Point(25, 50)
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_FONT cv::FONT_HERSHEY_SIMPLEX
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_SCALE 0.25
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_COLOR cv::Scalar(255, 0, 0)
#define MEASUREMENT_DATA_VIDEO_IMG_SIM_TEXT_THIN 2
#define MEASUREMENT_DATA_VIDEO_GPS_RADIUS 5
#define MEASUREMENT_DATA_VIDEO_GPS_COLOR cv::Scalar(255,0,0)
// ����ʒu
#define VISUALIZE_ESTI 1
#define IMAGE_ESTIPOSITION_RADIUS 5
#define IMAGE_ESTI_ARROW_LENGTH 8
#define IMAGE_ESTI_ARROW_THICKNESS 2
#define IMAGE_ESTIPOSITION_COLOR cv::Scalar(255, 0, 0)
// ���O����ʒu
#define VISUALIZE_PRE_ESTI 0
#define IMAGE_PRE_ESTIPOSITION_RADIUS 5
#define IMAGE_PRE_ESTI_ARROW_LENGTH 8
#define IMAGE_PRE_ESTI_ARROW_THICKNESS 1
#define IMAGE_PRE_ESTIPOSITION_COLOR cv::Scalar(255, 255, 0)
// �^�̈ʒu
#define IMAGE_TRUEPOSITION_RADIUS 7
#define IMAGE_TPOS_ARROW_LENGTH 8
#define IMAGE_TPOS_ARROW_THICKNESS 3
#define IMAGE_TRUEPOSITION_COLOR cv::Scalar(0, 255, 0)
// �m�����z�֌W
#define IMAGE_PARTICLE_RADIUS 5
#define IMAGE_PARTICLE_MAX_LIKELIHOOD 0.02
#define IMAGE_PARTICLE_ARROW_LENGTH 8
#define IMAGE_PARTICLE_ARROW_THICKNESS 2.0
// �m�����z�֌W
#define IMAGE_STAT_PARTICLE_RADIUS 5
#define IMAGE_STAT_PARTICLE_MAX_LIKELIHOOD 0.05
#define IMAGE_STAT_PARTICLE_ARROW_LENGTH 8
#define IMAGE_STAT_PARTICLE_ARROW_THICKNESS 2.0

#define IMAGE_PARTICLE_LARGE_GRAY_CIRCLE_RADIUS 1000	// mm
#define IMAGE_PARTICLE_LARGE_GRAY_CIRCLE_COLOR cv::Scalar(128,128,128)
#define IMAGE_PARTICLE_LARGE_GRAY_CIRCLE_THICHNESS 1

/* �s�A�\��臒l */
#define PEARSON_CORRCOEF_TH 0.2 // �s�A�\���ϗ����֌W���̗ގ�����臒l


///*  �ޓx�Z�o��������  */
//#define PARTICLE_THREAD_SIZE 10
//#define STAT_PARTICLE_THREAD_SIZE 10
//#define LIKELIHOOD_THREAD PARTICLE_THREAD_SIZE+STAT_PARTICLE_THREAD_SIZE
//#define READ_MEAS_THREAD_SIZE 5
//#define ENV_DESCRITOR_THREAD LIKELIHOOD_THREAD
////#define LIKELIHOOD_THREAD 1
//
///*  �ޓx�Z�o���������i��āj  */
#define LIKELIHOOD_THREAD 20


/*  �ޓx�����֘A  */
// Lidar2d
#define ICP_PAIR_DST_TH 3000 // ICP�}�b�`���O�̑Ή��_�����臒l
#define ICP_PAIR_NUM_TH_L 0.3  // �Ή��_�̍Œᐔ�i臒l�ȉ��ł͖ޓx0�j
#define ICP_PAIR_NUM_TH_U 0.3  // �Ή��_�̍Œᐔ�i臒l�ȉ��ł͖ޓx0�j
//#define ICP_PARAMETER 1.0e-5   // ICP�̖ޓx�֐��̐��K���z�̃p�����[�^
#define ICP_PARAMETER 4.0e-6   // ICP�̖ޓx�֐��̐��K���z�̃p�����[�^
#define LASER_DIST_RANGE 20000 // ���[�U�X�L���i�̌v���\����[mm]
#define GRID_PARAMETER 7.0e-4
// Omni Camera
#define CMR_IMG_SIZE_X 1600 // �摜�̉���
#define CMR_IMG_SIZE_Y 1200 // �摜�̏c��
#define ROI_SIZE_X 1200
#define ROI_SIZE_Y 1200
#define ROI_ORG_X (CMR_IMG_SIZE_X - ROI_SIZE_X) * 0.5
#define ROI_ORG_Y (CMR_IMG_SIZE_Y - ROI_SIZE_Y) * 0.5
#define BOF_TH_DST 2.0 // BoF�̎B���ʒu��臒l�i*�Ёj
// PARKING
#if LOCALIZATION_AREA==LOCALIZATION_PARKING
#define BOF_A 3.982E+13 
#define BOF_B -36.15
#endif
// SQUARE
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE
#endif
#if LOCALIZATION_AREA==LOCALIZATION_B2
#define BOF_A 2.117997E+5
#define BOF_B -7.04353
#endif
#if LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO
#define BOF_A 2.24441E+5
#define BOF_B -5.37
#endif
//#define BOF_SGM(sim) 1.15E+08 * std::exp(-13.5598*sim) // y = 1E+10e-20.25x
#define BOF_SGM(sim) BOF_A* std::exp(BOF_B*sim) // y = BOF_Ae-BOF_Bx
#define SYS_X 7500 // �����V�X�e���m�C�Y�ix[mm]�j // �V�X�e���m�C�Y�͂���Ȃɑ傫�����Ȃ������悢
#define BOF_TH_SIM -1/BOF_B*log(SYS_X/BOF_A) // BoF�̗ގ��x��臒l
// Type���w�� BruteForce�i-L1, -SL2, -Hamming, -Hamming(2)�j, FlannBased
// FlannBased�͉�]�_���H L1��SL2���ǂ������iL1�̂��ɂ��j
const std::string matching_type = "BruteForce-SL2";
#define CLUSTER_NUM 1000
#define SIFT_DIM 128
#define SURF_PARAM 400
// SIFT
#define SIFT_SIZE_USE_FUTURES 200 
#define SIFT_AMP_XY_SUNCTION(distance) 1/distance
#define SIFT_AMP_FUNCTION(dis_keys) 1/dis_keys	//	SURF�ɂ�����key�|�C���g�̑傫��������
// SURF�Ƃ̒��ڔ�r
#define SURF_DMATCH_DIST_TH 0.008
#define ENV_IMG_TH 3000	//	���݂̃p�[�e�B�N���ʒu����ʒu����Ɏg�p����摜��臒l[mm]
#define SURF_SD_GMM 500	//	GMM�ɂ�����e���K���z�̕W���΍��D�摜�Ԃ̋����ɉ����ēK�ؒl������
#define SURF_SIZE_USE_FUTURES 200 
#define SURF_AMP_FUNCTION(dis_keys) 1/dis_keys	//	SURF�ɂ�����key�|�C���g�̑傫��������
//#define SURF_AMP_FUNCTION(dis_keys) std::exp(-dis_keys)	//	SURF�ɂ�����key�|�C���g�̑傫��������
#define SURF_AMP_DISTANCE(distance) 1/distance
#define SURF_SIM_XY(size_of_matches_good) size_of_matches_good/1000.0
#define SURF_XY_VARIANCE(sim_xy) 49168.83*std::exp(-21.6667*sim_xy)
//	GPS
#define GPS_UERE 500 // User Equivalent Range Error�i���p�ғ��������덷�j[mm]

/*  �p�[�e�B�N���̓���֌W  */
#define PARTICLE_FPS 1	//	��b�Ԃ̉摜��

/*  �摜�o�͎��A���X�e�b�v���Ƃɉ~��`��  */
#define IMG_CIRCLE_STEP 100

/*  �摜�؂�o�����a  */
#define CUT_MAP_RADIUS_X 15000 // [mm]
#define CUT_MAP_RADIUS_Y 15000 // [mm]
/* Weighted particle �̎��� */
cv::Point WEIGHTED_STAT_PAR_IMG_TEXT_POINT(25, 50);
#define WEIGHTED_STAT_PAR_IMG_TEXT_FONT cv::FONT_HERSHEY_SIMPLEX
#define WEIGHTED_STAT_PAR_IMG_TEXT_SCALE 1.0
cv::Scalar WEIGHTED_STAT_PAR_IMG_TEXT_COLOR(0, 0, 0);
#define WEIGHTED_STAT_PAR_IMG_TEXT_THIN 2

/* �g��p */
#define CUT_LARGE_MAP_RADIUS_X 3000
#define CUT_LARGE_MAP_RADIUS_Y CUT_LARGE_MAP_RADIUS_X

// filepath
/*  ���̓t�@�C���p�X  */
//const std::string IFPATH = "\\\\Desktop-mt35ltg/f/Data/Localization/";
//const std::string IFPATH = "F://Data/Localization/";
const std::string IFPATH = "../input/";
//const std::string IFPATH_ENV = "C://Users/Robot/Documents/Visual Studio 2015/Projects/Localization.ver2/Localization.ver2/input/Environment/" + ENVIRONMENT_DATE + "/" + ENVIRONMENT_TIME + "/"; // ���t�܂�
//const std::string IFPATH_ENV = "C://Users/Robot/Documents/Visual Studio 2015/Projects/Localization.ver2/Localization.ver2/input/Environment/" + ENVIRONMENT_DATE + "/" + ENVIRONMENT_TIME + "/"; // ���t�܂�
const std::string IFPATH_ENV = IFPATH + "Environment/" + ENVIRONMENT_DATE + "/" + ENVIRONMENT_TIME + "/"; // ���t�܂�
#if LOCALIZATION_AREA==LOCALIZATION_PARKING || LOCALIZATION_AREA==LOCALIZATION_B2 || LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO
const std::string IFPATH_ENV_OMNI = IFPATH + "Environment/" + ENVIRONMENT_DATE_OMNI + "/" + ENVIRONMENT_TIME_OMNI + "/"; // ���t�܂�
#endif
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE
const std::string IFPATH_ENV_OMNI = IFPATH + "Environment/" + ENVIRONMENT_DATE_OMNI + "/" + ENVIRONMENT_TIME_OMNI + "/"; // ���t�܂�
//const std::string IFPATH_ENV_OMNI = IFPATH + "Environment/Omni/integrate1/"; // ���t�܂�
#endif
const std::string IFPATH_MEAS = IFPATH + "Measurement/" + MEASUREMENT_DATE + "/" + MEASUREMENT_TIME + "/"; // ���t�܂�
const std::string IFPATH_RMAP = IFPATH + "Result/Visual Studio/ReliabilityMap/pearson/Data/";

/*  �o�̓t�@�C���p�X  */
//const std::string OFPATH = "F://Data/Localization/Result/Visual Studio/Localization.ver2/map" + ENVIRONMENT_DATE + ENVIRONMENT_TIME + "/loc" + MEASUREMENT_DATE + MEASUREMENT_TIME + "/Pearson/01/";
//const std::string OFPATH_ALL = "\\\\Desktop-mt35ltg/f/Data/Localization/Result/Visual Studio/Localization.ver2/";
//const std::string OFPATH_ALL = "../output/";
const std::string OFPATH_ALL = "E://Data/Localization/Result/Visual Studio/Localization.ver2/";
//const std::string OFPATH_ALL = "//Desktop-mt35ltg/f/Data/Localization/Result/Visual Studio/Localization.ver2/";
// PARKING
#if LOCALIZATION_AREA==LOCALIZATION_PARKING && TEST==0
const std::string OFPATH_SIMUL = OFPATH_ALL + "parking/simultaneous";
const std::string OFPATH_PEAR = OFPATH_ALL + "parking/pearson";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "parking/pearson_nonstat";
const std::string OFPATH_NON_TIMESEQUENCE= OFPATH_ALL + "parking/nontimesequence_pear";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "parking/nontimesequence_simul";
const std::string OFPATH_SUYAMA_STAT = OFPATH_ALL + "parking/suyama_stat";
const std::string OFPATH_SUYAMA_NONSTAT = OFPATH_ALL + "parking/suyama_nonstat";
const std::string OFPATH_3SENSORS_SIMULTANEOUS = OFPATH_ALL + "parking/3sensors_simul";
const std::string OFPATH_3SENSORS_PEARSON = OFPATH_ALL + "parking/3sensors_pear";
const std::string OFPATH_3SENSORS_PEARSON_NONSTAT= OFPATH_ALL + "parking/3sensors_pear_nonstat";
const std::string OFPATH_3SENSORS_SUYAMA_NONSTAT = OFPATH_ALL + "parking/3sensors_suyama_nonstat";
const std::string OFPATH_3SENSORS_LRF_GPS = OFPATH_ALL + "parking/3sensors_lrf_gps";
#endif
// SQUARE
#if LOCALIZATION_AREA==LOCALIZATION_SQUARE && TEST==0
const std::string OFPATH_SIMUL = OFPATH_ALL + "square/simultaneous";
//const std::string OFPATH_SIMUL = "output/";
const std::string OFPATH_PEAR = OFPATH_ALL + "square/pearson";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "square/pearson_nonstat";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "square/nontimesequence_simul";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE = OFPATH_ALL + "square/nontimesequence_pear";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PREKL = OFPATH_ALL + "square/PreviousKL/";
//const std::string OFPATH_PREKL = "output/";
const std::string OFPATH_SUYAMA_STAT = OFPATH_ALL + "square/suyama_stat";
const std::string OFPATH_SUYAMA_NONSTAT = OFPATH_ALL + "square/suyama_nonstat";
const std::string OFPATH_3SENSORS_SIMULTANEOUS = OFPATH_ALL + "square/3sensors_simul";
const std::string OFPATH_3SENSORS_PEARSON = OFPATH_ALL + "square/3sensors_pear";
const std::string OFPATH_3SENSORS_PEARSON_NONSTAT= OFPATH_ALL + "square/3sensors_pear_nonstat";
const std::string OFPATH_3SENSORS_SUYAMA_NONSTAT = OFPATH_ALL + "square/3sensors_suyama_nonstat";
const std::string OFPATH_3SENSORS_LRF_GPS = OFPATH_ALL + "square/3sensors_lrf_gps";
#endif
// B2
#if LOCALIZATION_AREA==LOCALIZATION_B2 && TEST==0
const std::string OFPATH_SIMUL = OFPATH_ALL + "B2/simultaneous";
//const std::string OFPATH_SIMUL = "output/";
const std::string OFPATH_PEAR = OFPATH_ALL + "B2/pearson";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "B2/pearson_nonstat";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "B2/nontimesequence_simul";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE = OFPATH_ALL + "B2/nontimesequence_pear";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PREKL = OFPATH_ALL + "B2/PreviousKL/";
//const std::string OFPATH_PREKL = "output/";
const std::string OFPATH_SUYAMA_STAT = OFPATH_ALL + "B2/suyama_stat";
const std::string OFPATH_SUYAMA_NONSTAT = OFPATH_ALL + "B2/suyama_nonstat";
const std::string OFPATH_3SENSORS_SIMULTANEOUS = OFPATH_ALL + "B2/3sensors_simul";
const std::string OFPATH_3SENSORS_PEARSON = OFPATH_ALL + "B2/3sensors_pear";
const std::string OFPATH_3SENSORS_PEARSON_NONSTAT= OFPATH_ALL + "B2/3sensors_pear_nonstat";
const std::string OFPATH_3SENSORS_SUYAMA_NONSTAT = OFPATH_ALL + "B2/3sensors_suyama_nonstat";
const std::string OFPATH_3SENSORS_LRF_GPS = OFPATH_ALL + "B2/3sensors_lrf_gps";
#endif
// 8���َ���
#if LOCALIZATION_AREA==LOCALIZATION_AROUND_8GO&& TEST==0
const std::string OFPATH_SIMUL = OFPATH_ALL + "around-8-building/simultaneous";
const std::string OFPATH_PEAR = OFPATH_ALL + "around-8-building/pearson";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "around-8-building/pearson_nonstat";
const std::string OFPATH_NON_TIMESEQUENCE= OFPATH_ALL + "around-8-building/nontimesequence_pear";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "around-8-building/nontimesequence_simul";
const std::string OFPATH_SUYAMA_STAT = OFPATH_ALL + "around-8-building/suyama_stat";
const std::string OFPATH_SUYAMA_NONSTAT = OFPATH_ALL + "around-8-building/suyama_nonstat";
const std::string OFPATH_3SENSORS_SIMULTANEOUS = OFPATH_ALL + "around-8-building/3sensors_simul";
const std::string OFPATH_3SENSORS_PEARSON = OFPATH_ALL + "around-8-building/3sensors_pear";
const std::string OFPATH_3SENSORS_PEARSON_NONSTAT= OFPATH_ALL + "around-8-building/3sensors_pear_nonstat";
const std::string OFPATH_3SENSORS_SUYAMA_NONSTAT = OFPATH_ALL + "around-8-building/3sensors_suyama_nonstat";
const std::string OFPATH_3SENSORS_LRF_GPS = OFPATH_ALL + "around-8-building/3sensors_lrf_gps";
#endif

// TEST_MODE
#if TEST==1
const std::string OFPATH_SIMUL = OFPATH_ALL + "simultaneous";
//const std::string OFPATH_SIMUL = "output/";
const std::string OFPATH_PEAR = OFPATH_ALL + "pearson";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PEAR_NONSTAT = OFPATH_ALL + "pearson_nonstat";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE = OFPATH_ALL + "non_timesequence";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_NON_TIMESEQUENCE_SIMUL = OFPATH_ALL + "nontimesequence_simul";
//const std::string OFPATH_PEAR = "output/";
const std::string OFPATH_PREKL = OFPATH_ALL + "PreviousKL/";
//const std::string OFPATH_PREKL = "output/";
#endif

/* �m�����z�ƃp�[�e�B�N���̕ۑ� */
#define SAVE_PARTICLE_STATES 1
#define SAVE_PARTICLE_IMAGES 0
#define SAVE_STAT_PARTICLE_STATES 1
#define SAVE_STAT_PARTICLE_IMAGES 0

/* �m�����z�ƃp�[�e�B�N��visualize */
#define SHOW_PARTICLE_IMAGES 0
#define SHOW_STAT_PARTICLE_IMAGES 0



