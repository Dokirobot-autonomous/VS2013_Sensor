/*******************************************************************************
*
* File name     : TETRA_DS_IV.cpp
* Programmer(s) : Yuki FUNABORA
* Description   : �ړ����{�b�g(TETRA-DS IV)�̃N���X�t�@�C��
*
*******************************************************************************/
#include "stdafx.h"
#include "TETRA_DS_IV.hpp"



/*******************************************************************************
* Function name  : TETRA_DS_IT
* Input / Output : IP�A�h���X,�|�[�g / �Ȃ�
* Description    : �R���X�g���N�^
*******************************************************************************/
TETRA_DS_IV::TETRA_DS_IV(char* server_ip, int port) {
	// �T�[�o�����i�[
	server_ip_addr = server_ip;
	server_port = port;
	// �����w�ߕϐ�������
	velocity[0] = 0; velocity[1] = 0;

	// �\�P�b�g�I�[�v��(�ʐM�p)
	WSADATA wsad;
	if (WSAStartup(2, &wsad)) {
		std::cerr << "WSAStartup() Error in MS Windows" << std::endl;
		WSACleanup();
		exit(-1);	// �\�P�b�g�I�[�v���G���[
	}

	// �f�o�C�X�d���I��

	// ������Ԃ̎擾
	update();
	show();

	// �T�[�{�I��(�ʃR�}���h�ɂ��ׂ���)
	set_servo_on();

	return;	// ���폈��
}



/*******************************************************************************
* Function name  : ~TETRA_DS_IT
* Input / Output : �Ȃ� / �Ȃ�
* Description    : �f�X�g���N�^
*******************************************************************************/
TETRA_DS_IV::~TETRA_DS_IV() {
	// �T�[�{�I�t
	set_servo_off();

	// �f�o�C�X�d���I�t

	// �\�P�b�g�N���[�Y
	WSACleanup();

	return;	// ���폈��
}



/*******************************************************************************
* Function name  : set_velocity
* Input / Output : ���ԗ֑��x�w��, �E�ԗ֑��x�w�� / �Ȃ�
* Description    : ���x�w�ߒl�̓����ύX(���ۂ̃��{�b�g�ւ̎w�߂�update���s��)
*******************************************************************************/
void TETRA_DS_IV::set_velocity(int velocity_left, int velocity_right) {
	velocity[0] = velocity_left;
	velocity[1] = velocity_right;
	return;
}



/*******************************************************************************
* Function name  : get_velocity
* Input / Output : �Ȃ� / �ԗ֑��x�z��(0:����, 1:�E��)
* Description    : �������x�w�ߒl�̎擾
*******************************************************************************/
int* TETRA_DS_IV::get_velocity() {
	return velocity;
}



/*******************************************************************************
* Function name  : get_encoder
* Input / Output : �Ȃ� / �G���R�[�_�l�z��(0:����, 1:�E��)
* Description    : �G���R�[�_�l�̎擾(�l�͑O��update���̂���)
*******************************************************************************/
int* TETRA_DS_IV::get_encoder() {
	return encoder;
}



/*******************************************************************************
* Function name  : get_odometry
* Input / Output : �Ȃ� / ���{�b�g�̑��Έʒu�p���̔z��(0:x, 1:y, 2:theta)
* Description    : �I�h���g���ɂ��ʒu�̎擾(�l�͑O��update���̂���)
*******************************************************************************/
int* TETRA_DS_IV::get_odometry() {
	return odometry;
}



/*******************************************************************************
* Function name  : update
* Input / Output : �Ȃ� / �Ȃ�
* Description    : ���x�w�߂̍X�V�ƃZ���T�l�̎擾
*******************************************************************************/
void TETRA_DS_IV::update() {
	// ���{�b�g�Ƃ̏�����
	if(!update_velocity()) std::cerr << "Err velocity_update:";
	if(!update_encoder()) std::cerr << "Err encoder_update:";
	if(!update_odometry()) std::cerr << "Err odometry_update:";

	// ����I��
	return;
}



/*******************************************************************************
* Function name  : show
* Input / Output : �Ȃ� / �Ȃ�
* Description    : ���x�w�ߒl�ƃZ���T�l�̕W���o�͂ւ̕\��
*******************************************************************************/
void TETRA_DS_IV::show() {
	std::cout << velocity[0] << ", " << velocity[1] << " | ";
	std::cout << encoder[0] << ", " << encoder[1] << " | ";
	std::cout << odometry[0] << ", " << odometry[1] << ", " << odometry[2] << std::endl;
}



/*******************************************************************************
* Function name  : set_servo_on
* Input / Output : �Ȃ� / ����
* Description    : �ԗւ̃T�[�{�I��
*******************************************************************************/
bool TETRA_DS_IV::set_servo_on() {
	// �ϐ���`
	dsphal_tcp_client_t *tcp_client;

	// ���{�b�g�Ƃ̒ʐM�H�m��
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// �ʐM�H�m�����s
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// �T�[�{�I��
	dsphal_request_method_call(tcp_client, "ServoOn", NULL);

	// �ʐM�H�̔j��
	dsphal_tcp_client_destroy(tcp_client);

	// ����I��
	return true;
}



/*******************************************************************************
* Function name  : set_servo_off
* Input / Output : �Ȃ� / ����
* Description    : �ԗւ̃T�[�{�I�t
*******************************************************************************/
bool TETRA_DS_IV::set_servo_off() {
	// �ϐ���`
	dsphal_tcp_client_t *tcp_client;

	// ���{�b�g�Ƃ̒ʐM�H�m��
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// �ʐM�H�m�����s
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// �T�[�{�I�t
	dsphal_request_method_call(tcp_client, "ServoOff", NULL);

	// �ʐM�H�̔j��
	dsphal_tcp_client_destroy(tcp_client);

	// ����I��
	return true;
}



/*******************************************************************************
* Function name  : update_velocity
* Input / Output : �Ȃ� / ����
* Description    : �����ϐ��ɐݒ肳�ꂽ���x�w�ߒl�����{�b�g�ɔ��f
*******************************************************************************/
bool TETRA_DS_IV::update_velocity() {
	// �ϐ���`
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_arg;
	dsphal_datalist_t *datalist_ret;

	// ���{�b�g�Ƃ̒ʐM�H�m��
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// �ʐM�H�m�����s
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// ���x�w�ߒl�̎w��
	datalist_arg = dsphal_build_root_datalist("[{i}{i}]", velocity[0], velocity[1]);
	datalist_ret = dsphal_request_method_call(tcp_client, "VelocityControl", datalist_arg);
	if (datalist_arg) dsphal_datalist_destroy(datalist_arg);
	if (datalist_ret) dsphal_datalist_destroy(datalist_ret);

	// �ʐM�H�̔j��
	dsphal_tcp_client_destroy(tcp_client);

	// ����I��
	return true;
}


/*******************************************************************************
* Function name  : update_encoder
* Input / Output : �Ȃ� / ����
* Description    : ���{�b�g����G���R�[�_�l���擾���ē����ϐ��Ɋi�[
*******************************************************************************/
bool TETRA_DS_IV::update_encoder() {
	// �ϐ���`
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	// ���{�b�g�Ƃ̒ʐM�H�m��
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// �ʐM�H�m�����s
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// �G���R�[�_�l�̎擾
	if ((datalist_ret = dsphal_request_method_call(tcp_client, "ReadEncoder", NULL))){
		dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}]", &encoder[0], &encoder[1]);
		dsphal_datalist_destroy(datalist_ret);
	}
	else {
		std::cerr << "No encoder datalist" << std::endl;
		dsphal_datalist_destroy(datalist_ret);
	}

	// �ʐM�H�̔j��
	dsphal_tcp_client_destroy(tcp_client);

	// ����I��
	return true;
}


/*******************************************************************************
* Function name  : update_odometry
* Input / Output : �Ȃ� / ����
* Description    : ���{�b�g����I�h���g���l���擾���ē����ϐ��Ɋi�[
*******************************************************************************/
bool TETRA_DS_IV::update_odometry() {
	// �ϐ���`
	dsphal_tcp_client_t *tcp_client;
	dsphal_datalist_t *datalist_ret;

	// ���{�b�g�Ƃ̒ʐM�H�m��
	tcp_client = dsphal_tcp_client_create(server_ip_addr, server_port);
	if (dsphal_tcp_client_connect(tcp_client)) {	// 0:sucess, -1:failure
		// �ʐM�H�m�����s
		std::cerr << "Connection failed..." << std::endl;
		dsphal_tcp_client_destroy(tcp_client);
		return false;
	}

	// �I�h���g���̎擾
	if ((datalist_ret = dsphal_request_method_call(tcp_client, "ReadPosition", NULL))){
		dsphal_decompose_root_datalist(datalist_ret, "[{i}{i}{i}]", &odometry[0], &odometry[1], &odometry[2]);
		dsphal_datalist_destroy(datalist_ret);
	}
	else {
		std::cerr << "No odometry datalist" << std::endl;
		dsphal_datalist_destroy(datalist_ret);
	}

	// �ʐM�H�̔j��
	dsphal_tcp_client_destroy(tcp_client);

	// ����I��
	return true;
}

