#ifndef	__serial_h
#define	__serial_h

#include <windows.h>

#define		MORPHOLOGY_OPEN			3
#define		PARITY					NOPARITY
#define		BYTE_SIZE				8
#define		STOP_BIT				FALSE
#define		F_PARITY				ONESTOPBIT
#define		RXDATA_SIZE				100

// �V���A���|�[�g�̃N���X
class SerialPort{
public:

	int key, ret;
	unsigned int rxCnt, rxLen;
	char buf[4096], rxData[RXDATA_SIZE];
	char rxFlag;

	HANDLE hComm;				// �V���A���|�[�g�Ƃ̒ʐM�n���h��

	DWORD dwWritten;			// �|�[�g�֏������񂾃o�C�g��
	DWORD dwErrors;
	COMSTAT ComStat;
	DWORD dwCount;
	DWORD dwRead;				// �|�[�g����ǂݏo�����o�C�g��

	SerialPort(void);
	//void start(char *modem, unsigned int baudRate);
	void start(LPCWSTR modem, unsigned int baudRate);
	void send(char *str);
	bool gets(char start, char end);
	void end(void);
};

#endif