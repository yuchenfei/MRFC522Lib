/*
 Name:		MFRC522Lib.cpp
 Created:	2016/3/24 17:00:51
 Author:	ycf
 Editor:	http://www.visualmicro.com
*/

#include "MFRC522Lib.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
//���캯��
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* ��    �ܣ����캯��
*/
MFRC522::MFRC522() {

}

/*
* ��    �ܣ����캯����Ӳ��SPI��
* ����˵����chipSelectPin[����]��SPI�ӻ�ʹ������
*           resetPin[����]����λ����
*/

MFRC522::MFRC522(byte chipSelectPin, byte resetPin) {
	_chipSelectPin = chipSelectPin;
	_resetPin = resetPin;
	//_hardwareSPI = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//SPI���ú���
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* ��    �ܣ�SPI���ú���
*/
void MFRC522::SPI_Config() {
	SPI.begin();
	spisetting = SPISettings(10000000, MSBFIRST, SPI_MODE0);  //MFRC522SPI�������Ϊ10Mbit/s����λ�ȷ��͡������ֲ��8.1.2���֡�
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//MFRC522�ײ㺯�����Ĵ�����д��
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* ��    �ܣ�д�Ĵ���
* ����˵����reg[����]��Ҫд��ļĴ�����ַ��MFRC522_Registerö��ֵ�е�һ����
*           value[����]��Ҫд���ֵ
*/
void MFRC522::WriteReg(byte reg, byte value) {
	SPI.beginTransaction(spisetting);
	digitalWrite(_chipSelectPin, LOW);
	SPI.transfer((reg << 1) & 0x7E);            //д�Ĵ�����ַ��ʽ��0XXXXXXX0����λ0��ʾд����λ0�����ڵ�ַ��[�����ֲ��8.1.2.3����]
	SPI.transfer(value);
	digitalWrite(_chipSelectPin, HIGH);
	SPI.endTransaction();
}

/*
* ��    �ܣ����Ĵ���
* ����˵����reg[����]��Ҫ��ȡ�ļĴ�����ַ��MFRC522_Registerö��ֵ�е�һ����
* ��    �أ���ȡ����ֵ
*/
byte MFRC522::ReadReg(byte reg) {
	byte value;
	SPI.beginTransaction(spisetting);
	digitalWrite(_chipSelectPin, LOW);
	SPI.transfer(((reg << 1) & 0x7E) | 0x80);   //���Ĵ�����ַ��ʽ��1XXXXXXX0����λ1��ʾ������λ0�����ڵ�ַ��[�����ֲ��8.1.2.3����]
	value = SPI.transfer(0x00);
	digitalWrite(_chipSelectPin, HIGH);
	SPI.endTransaction();
	return value;
}

/*
* ��    �ܣ��üĴ���λ
* ����˵����reg[����]��Ҫ��λ�ļĴ�����ַ��MFRC522_Registerö��ֵ�е�һ����
* ��    �أ�mask[����]����λֵ
*/
void MFRC522::SetRegBitMask(byte reg, byte mask) {
	byte temp;
	temp = ReadReg(reg);
	WriteReg(reg, temp | mask);
}

/*
* ��    �ܣ���Ĵ���λ
* ����˵����reg[����]��Ҫ��λ�ļĴ�����ַ��MFRC522_Registerö��ֵ�е�һ����
* ��    �أ�mask[����]����λֵ
*/
void MFRC522::ClearRegBitMask(byte reg, byte mask) {
	byte temp;
	temp = ReadReg(reg);
	WriteReg(reg, temp | (~mask));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//MFRC522���ܺ���
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* ��    �ܣ���ʼ��MFRC522
*/
void MFRC522::RC522_Init() {
	pinMode(_chipSelectPin, OUTPUT);      //����chipSelectPinΪ�������
	digitalWrite(_chipSelectPin, HIGH);   //��ʱ�����ù���
	pinMode(_resetPin, OUTPUT);           //����resetPinΪ�������

	SPI_Config();                         //��ʼ��SPI����

	if (LOW == digitalRead(_resetPin)) {   //оƬ��Ӳ����ģʽ��
		digitalWrite(_resetPin, HIGH);      //�˳�Ӳ����ģʽ������Ӳ������
		delay(50);                          //�ȴ�����������ȶ���������Ҫt_startup(����ʱ��)+t_delay(37.74us),�����50ms�㹻��[�����ֲ�8.8.2����]
	}
	else {
		RC522_Reset();                    //�����������
	}

	//������RFID��ǩͨѶʱ�ĳ�ʱʱ��
	//��ʱ��Ƶ�� = 13.56MHz/(TPrescaler*2+1),TPrescaler��ֵ��TModeReg�ĵ�4λ��TPrescalerReg���
	WriteReg(TModeReg, 0x80);             //TAuto=1����ʱ��������ͨѶģʽ�������ʵķ��ͽ���ʱ�Զ��������ڽ��յ���һ������λ��ʱ������ֹͣ
	WriteReg(TPrescalerReg, 0xA9);        //TPrescaler=TModeReg[3..0]:TPrescalerReg[7..0],����Ϊ0x0A9 = 168 => ��ʱ��Ƶ��=40kHz,����Ϊ25us
	WriteReg(TReloadRegH, 0x03);          //��ʱ����װֵ��0x3E8 = 1000,����ʱʱ��Ϊ25ms
	WriteReg(TReloadRegL, 0xE8);

	WriteReg(TxASKReg, 0x40);             //Ĭ��Ϊ0x00,ǿ��100%ASK����
	WriteReg(ModeReg, 0x3D);              //Ĭ��Ϊ0x3F,����CRCЭ��������Ԥ��ֵΪ0x6363[ISO 14443-3 part 6.2.4]
	RC522_AntennaOn();                  //ʹ����������������TX1��TX2(���û��Զ��ر�)
}

/*
* ��    �ܣ�����MFRC522
*/
void MFRC522::RC522_Reset() {
	WriteReg(CommandReg, RC522_SoftReset);  //��������������
	delay(50);                            //�ȴ�����������ȶ���������Ҫt_startup(����ʱ��)+t_delay(37.74us),�����50ms�㹻��[�����ֲ�8.8.2����]
	while (ReadReg(CommandReg) & (1 << 4));
}

/*
* ��    �ܣ�����MFRC522����
*/
void MFRC522::RC522_AntennaOn() {
	byte temp;
	temp = ReadReg(TxControlReg);
	if (!(temp & 0x03)) {
		SetRegBitMask(TxControlReg, 0x03);
	}
}

/*
* ��    �ܣ��ر�MFRC522����
*/
void MFRC522::RC522_AntennaOff() {
	ClearRegBitMask(TxControlReg, 0x03);
}

/*
* ��    �ܣ���ȡ��ǰMFRC522����������ֵ
* ��    �أ�RxGain��ֵ<<4 [�����ֲ�9.3.3.6����]
*           0x00  18dB
*           0x10  23dB
*           0x20  18dB
*           0x30  23dB
*           0x40  33dB
*           0x50  38dB
*           0x60  43dB
*           0x70  48dB
*/
byte MFRC522::RC522_GetAntennaGain() {
	return ReadReg(RFCfgReg) & (0x07 << 4);
}

/*
* ��    �ܣ���ȡ��ǰMFRC522����������ֵ
* ����˵����mask[����]��RxGain��ֵ<<4 [�����ֲ�9.3.3.6����]
*                       0x00  18dB
*                       0x10  23dB
*                       0x20  18dB
*                       0x30  23dB
*                       0x40  33dB
*                       0x50  38dB
*                       0x60  43dB
*                       0x70  48dB
*/
void MFRC522::RC522_SetAntennaGain(byte mask) {
	if (RC522_GetAntennaGain() != mask) {
		ClearRegBitMask(RFCfgReg, (0x07 << 4));
		SetRegBitMask(RFCfgReg, mask & (0x07 << 4));
	}
}

