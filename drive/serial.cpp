// serial.cpp
// Windows API���g�p���� �V���A���ʐM�v���O����

#include "serial.h"
#include <iostream>
#include <string.h>
#include <locale.h>
using namespace std;

// �V���A���|�[�g�̃R���X�g���N�^
SerialPort::SerialPort(void)
{
	rxLen = 0;
	rxCnt = 0;
	rxFlag = 0;
	dwWritten = 0;
	dwErrors = 0;
	dwCount = 0;
	dwRead = 0;
	ret = 0;
}

// �V���A���|�[�g�̏����ݒ�
//void SerialPort::start(char *modem, unsigned int baudRate)
void SerialPort::start(LPCWSTR modem, unsigned int baudRate)
{
	/*size_t wLen = 0;
	errno_t err = 0;
	WCHAR wStrW[50];
	setlocale(LC_ALL, "japanese");
	err = mbstowcs_s(&wLen, wStrW, 20, modem, _TRUNCATE);*/

	// �V���A���|�[�g���J����
	hComm = CreateFile(
		modem,                              // �V���A���|�[�g�̕�����
		GENERIC_READ | GENERIC_WRITE,       // �A�N�Z�X���[�h�F�ǂݏ���
		0,                                  // ���L���[�h�F������̓A�N�Z�X�s��
		NULL,                               // �Z�L�����e�B�����F�n���h���p������
		OPEN_EXISTING,                      // �쐬�t���O�F
		FILE_ATTRIBUTE_NORMAL,              // �����F
		NULL                                // �e���v���[�g�̃n���h���F
		);

	if (hComm == INVALID_HANDLE_VALUE){
		std::cout << "�V���A���|�[�g���J���܂���ł���" << std::endl;
		ret = -1;
	}
	
	// �ʐM������ݒ肷��
	DCB dcb;
	GetCommState(hComm, &dcb);              // DCB ���擾
	dcb.BaudRate = baudRate;
	dcb.ByteSize = BYTE_SIZE;
	dcb.Parity = PARITY;
	dcb.fParity = STOP_BIT;
	dcb.StopBits = F_PARITY;
	SetCommState(hComm, &dcb);            // DCB ��ݒ�
}

// �V���A���|�[�g�̏I��
void SerialPort::end(void)
{
	CloseHandle(hComm);                     // �n���h�������
}

// �V���A���|�[�g���當����𑗐M����
void SerialPort::send(char *str)
{
	WriteFile(hComm, str, (DWORD)strlen(str), &dwWritten, NULL);        // ���M����֐�
}

// �V���A���|�[�g�Ŏ�M�����������ǂݍ���
bool SerialPort::gets(char start, char end)
{
	ClearCommError(hComm, &dwErrors, &ComStat);
	dwCount = ComStat.cbInQue;
	if (dwCount > 0)
	{
		ReadFile(hComm, buf, dwCount, &dwRead, NULL);     // �ǂݍ��ފ֐��@�ǂݍ��񂾕������ buf�ɓ���
		// �L���ȕ�����̂ݔ������
		for (rxCnt = 0; rxCnt < dwRead; rxCnt++)         // dwRead (�|�[�g����ǂݏo�����o�C�g��)���������
		{
			if (buf[rxCnt] == start)                      // buf�̒���$������ΗL��������̎n��
			{
				rxFlag = 0; // Flag��������
				rxLen = 0;                                  // $:��M�X�^�[�g
			}
			else if (buf[rxCnt] == end && rxFlag == 0)  // buf�̒���\n������ΗL��������̏I���
			{
				rxData[rxLen] = '\0';
				rxLen = 0;
				rxFlag = 1;
				return true;
			}
			else                                            // $�ł�\r�ł��Ȃ��ꍇ
			{
				if (rxLen < RXDATA_SIZE)
				{
					rxData[rxLen] = buf[rxCnt];         // ������̊i�[ ����Ȃ̂ŏ㏑��
					rxLen++;
				}
			}
		}
		return false;
	}
	else
	{
		return false;
	}
}