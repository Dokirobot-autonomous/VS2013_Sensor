// serial.cpp
// Windows APIを使用した シリアル通信プログラム

#include "serial.h"
#include <iostream>
#include <string.h>
#include <locale.h>
using namespace std;

// シリアルポートのコンストラクタ
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

// シリアルポートの初期設定
//void SerialPort::start(char *modem, unsigned int baudRate)
void SerialPort::start(LPCWSTR modem, unsigned int baudRate)
{
	/*size_t wLen = 0;
	errno_t err = 0;
	WCHAR wStrW[50];
	setlocale(LC_ALL, "japanese");
	err = mbstowcs_s(&wLen, wStrW, 20, modem, _TRUNCATE);*/

	// シリアルポートを開ける
	hComm = CreateFile(
		modem,                              // シリアルポートの文字列
		GENERIC_READ | GENERIC_WRITE,       // アクセスモード：読み書き
		0,                                  // 共有モード：他からはアクセス不可
		NULL,                               // セキュリティ属性：ハンドル継承せず
		OPEN_EXISTING,                      // 作成フラグ：
		FILE_ATTRIBUTE_NORMAL,              // 属性：
		NULL                                // テンプレートのハンドル：
		);

	if (hComm == INVALID_HANDLE_VALUE){
		std::cout << "シリアルポートを開けませんでした" << std::endl;
		ret = -1;
	}
	
	// 通信属性を設定する
	DCB dcb;
	GetCommState(hComm, &dcb);              // DCB を取得
	dcb.BaudRate = baudRate;
	dcb.ByteSize = BYTE_SIZE;
	dcb.Parity = PARITY;
	dcb.fParity = STOP_BIT;
	dcb.StopBits = F_PARITY;
	SetCommState(hComm, &dcb);            // DCB を設定
}

// シリアルポートの終了
void SerialPort::end(void)
{
	CloseHandle(hComm);                     // ハンドルを閉じる
}

// シリアルポートから文字列を送信する
void SerialPort::send(char *str)
{
	WriteFile(hComm, str, (DWORD)strlen(str), &dwWritten, NULL);        // 送信する関数
}

// シリアルポートで受信した文字列を読み込む
bool SerialPort::gets(char start, char end)
{
	ClearCommError(hComm, &dwErrors, &ComStat);
	dwCount = ComStat.cbInQue;
	if (dwCount > 0)
	{
		ReadFile(hComm, buf, dwCount, &dwRead, NULL);     // 読み込む関数　読み込んだ文字列は bufに入る
		// 有効な文字列のみ抜き取る
		for (rxCnt = 0; rxCnt < dwRead; rxCnt++)         // dwRead (ポートから読み出したバイト数)分抜き取る
		{
			if (buf[rxCnt] == start)                      // bufの中に$があれば有効文字列の始め
			{
				rxFlag = 0; // Flagを初期化
				rxLen = 0;                                  // $:受信スタート
			}
			else if (buf[rxCnt] == end && rxFlag == 0)  // bufの中に\nがあれば有効文字列の終わり
			{
				rxData[rxLen] = '\0';
				rxLen = 0;
				rxFlag = 1;
				return true;
			}
			else                                            // $でも\rでもない場合
			{
				if (rxLen < RXDATA_SIZE)
				{
					rxData[rxLen] = buf[rxCnt];         // 文字列の格納 代入なので上書き
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