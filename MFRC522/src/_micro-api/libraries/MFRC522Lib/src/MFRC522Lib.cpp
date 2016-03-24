/*
 Name:		MFRC522Lib.cpp
 Created:	2016/3/24 17:00:51
 Author:	ycf
 Editor:	http://www.visualmicro.com
*/

#include "MFRC522Lib.h"

////////////////////////////////////////////////////////////////////////////////////////////////////
//构造函数
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 功    能：构造函数
*/
MFRC522::MFRC522() {

}

/*
* 功    能：构造函数（硬件SPI）
* 参数说明：chipSelectPin[输入]：SPI从机使能引脚
*           resetPin[输入]：复位引脚
*/

MFRC522::MFRC522(byte chipSelectPin, byte resetPin) {
	_chipSelectPin = chipSelectPin;
	_resetPin = resetPin;
	//_hardwareSPI = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//SPI配置函数
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 功    能：SPI配置函数
*/
void MFRC522::SPI_Config() {
	SPI.begin();
	spisetting = SPISettings(10000000, MSBFIRST, SPI_MODE0);  //MFRC522SPI最高速率为10Mbit/s，高位先发送。数据手册第8.1.2部分。
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//MFRC522底层函数（寄存器读写）
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 功    能：写寄存器
* 参数说明：reg[输入]：要写入的寄存器地址。MFRC522_Register枚举值中的一个。
*           value[输入]：要写入的值
*/
void MFRC522::WriteReg(byte reg, byte value) {
	SPI.beginTransaction(spisetting);
	digitalWrite(_chipSelectPin, LOW);
	SPI.transfer((reg << 1) & 0x7E);            //写寄存器地址格式：0XXXXXXX0，高位0表示写，低位0不用于地址。[数据手册第8.1.2.3部分]
	SPI.transfer(value);
	digitalWrite(_chipSelectPin, HIGH);
	SPI.endTransaction();
}

/*
* 功    能：读寄存器
* 参数说明：reg[输入]：要读取的寄存器地址。MFRC522_Register枚举值中的一个。
* 返    回：读取到的值
*/
byte MFRC522::ReadReg(byte reg) {
	byte value;
	SPI.beginTransaction(spisetting);
	digitalWrite(_chipSelectPin, LOW);
	SPI.transfer(((reg << 1) & 0x7E) | 0x80);   //读寄存器地址格式：1XXXXXXX0，高位1表示读，低位0不用于地址。[数据手册第8.1.2.3部分]
	value = SPI.transfer(0x00);
	digitalWrite(_chipSelectPin, HIGH);
	SPI.endTransaction();
	return value;
}

/*
* 功    能：置寄存器位
* 参数说明：reg[输入]：要置位的寄存器地址。MFRC522_Register枚举值中的一个。
* 返    回：mask[输入]：置位值
*/
void MFRC522::SetRegBitMask(byte reg, byte mask) {
	byte temp;
	temp = ReadReg(reg);
	WriteReg(reg, temp | mask);
}

/*
* 功    能：清寄存器位
* 参数说明：reg[输入]：要清位的寄存器地址。MFRC522_Register枚举值中的一个。
* 返    回：mask[输入]：清位值
*/
void MFRC522::ClearRegBitMask(byte reg, byte mask) {
	byte temp;
	temp = ReadReg(reg);
	WriteReg(reg, temp | (~mask));
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//MFRC522功能函数
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 功    能：初始化MFRC522
*/
void MFRC522::RC522_Init() {
	pinMode(_chipSelectPin, OUTPUT);      //设置chipSelectPin为数字输出
	digitalWrite(_chipSelectPin, HIGH);   //暂时还不用工作
	pinMode(_resetPin, OUTPUT);           //设置resetPin为数字输出

	SPI_Config();                         //初始化SPI总线

	if (LOW == digitalRead(_resetPin)) {   //芯片在硬掉电模式下
		digitalWrite(_resetPin, HIGH);      //退出硬掉电模式，触发硬件重置
		delay(50);                          //等待晶振起振和稳定，至少需要t_startup(启动时间)+t_delay(37.74us),这里给50ms足够了[数据手册8.8.2部分]
	}
	else {
		RC522_Reset();                    //进行软件重置
	}

	//设置与RFID标签通讯时的超时时间
	//定时器频率 = 13.56MHz/(TPrescaler*2+1),TPrescaler的值由TModeReg的低4位和TPrescalerReg组成
	WriteReg(TModeReg, 0x80);             //TAuto=1；定时器在所有通讯模式所有速率的发送结束时自动启动，在接收到第一个数据位后定时器立即停止
	WriteReg(TPrescalerReg, 0xA9);        //TPrescaler=TModeReg[3..0]:TPrescalerReg[7..0],这里为0x0A9 = 168 => 定时器频率=40kHz,周期为25us
	WriteReg(TReloadRegH, 0x03);          //定时器重装值：0x3E8 = 1000,即超时时间为25ms
	WriteReg(TReloadRegL, 0xE8);

	WriteReg(TxASKReg, 0x40);             //默认为0x00,强制100%ASK调制
	WriteReg(ModeReg, 0x3D);              //默认为0x3F,设置CRC协处理器的预设值为0x6363[ISO 14443-3 part 6.2.4]
	RC522_AntennaOn();                  //使能天线驱动器引脚TX1和TX2(重置会自动关闭)
}

/*
* 功    能：重置MFRC522
*/
void MFRC522::RC522_Reset() {
	WriteReg(CommandReg, RC522_SoftReset);  //发送软重置命令
	delay(50);                            //等待晶振起振和稳定，至少需要t_startup(启动时间)+t_delay(37.74us),这里给50ms足够了[数据手册8.8.2部分]
	while (ReadReg(CommandReg) & (1 << 4));
}

/*
* 功    能：开启MFRC522天线
*/
void MFRC522::RC522_AntennaOn() {
	byte temp;
	temp = ReadReg(TxControlReg);
	if (!(temp & 0x03)) {
		SetRegBitMask(TxControlReg, 0x03);
	}
}

/*
* 功    能：关闭MFRC522天线
*/
void MFRC522::RC522_AntennaOff() {
	ClearRegBitMask(TxControlReg, 0x03);
}

/*
* 功    能：获取当前MFRC522接收器增益值
* 返    回：RxGain的值<<4 [数据手册9.3.3.6部分]
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
* 功    能：获取当前MFRC522接收器增益值
* 参数说明：mask[输入]：RxGain的值<<4 [数据手册9.3.3.6部分]
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

