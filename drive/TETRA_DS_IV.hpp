/*******************************************************************************
*
* File name     : TETRA_DS_IV.hpp
* Programmer(s) : Yuki FUNABORA
* Description   : �ړ����{�b�g(TETRA-DS IV)�̃N���X�p�w�b�_
*
*******************************************************************************/
#ifndef _INC_TETRA_DS_IV	// �Ǎ��σ`�F�b�N
#define _INC_TETRA_DS_IV



/*******************************************************************************
* Windows�\�P�b�g�ʐM���C�u�����̗��p�ݒ�
*******************************************************************************/
#include <WinSock2.h>
#pragma comment(lib,"wsock32.lib")



/*******************************************************************************
* TETRA-DS IV DSSP-HAL���C�u�����̗��p�ݒ�
*******************************************************************************/
#include "dsphal.h"
#pragma comment(lib,"../DSSMP_INFO/lib/libdsphal.lib")



/*******************************************************************************
* �N���X���ϐ��E�֐���`
*******************************************************************************/
class TETRA_DS_IV
{
	/***** ��������:�v���C�x�[�g�ϐ� *****/
	// �ʐM�p
	char *server_ip_addr;
	int server_port;
	// ����p
	int velocity[2];	// 0:left, 1:right
	// �ϑ��p
	int encoder[2];		// 0:left, 1:right
	int odometry[3];	// 0:x, 1:y, 2:theta

	/***** �����܂�:�v���C�x�[�g�ϐ� *****/

	/***** ��������:�v���C�x�[�g�֐� *****/
	bool set_servo_on();	// �ԗփT�[�{�̃I��
	bool set_servo_off();	// �ԗփT�[�{�̃I�t
	bool update_velocity();	// ���{�b�g�ւ̑��x�w�߂̔��f
	bool update_encoder();	// ���{�b�g����G���R�[�_�l���擾
	bool update_odometry();	// ���{�b�g����I�h���g���l���擾
	/***** �����܂�:�v���C�x�[�g�֐� *****/

	/***** ��������:�p�u���b�N�֐� *****/
public:
	TETRA_DS_IV(char* server_ip, int port); // �R���X�g���N�^
	~TETRA_DS_IV();			// �f�X�g���N�^
	void set_velocity(int velocity_left, int velocity_right);	// �ԗ֑��x�̎w��
	int* get_velocity();	// ���݂̑��x�w�ߒl�擾(�s�v?)
	int* get_encoder();		// �G���R�[�_�l�̎擾
	int* get_odometry();	// �I�h���g���l�̎擾

	void update();	// �l�̍X�V(���{�b�g�Ƃ̏�����)

	void show();	// ���{�b�g�̏��\��
	/***** �����܂�:�p�u���b�N�֐� *****/

};

#endif	// _INC_TETRA_DS_IV