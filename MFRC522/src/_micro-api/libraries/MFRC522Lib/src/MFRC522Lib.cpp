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
	spisetting = SPISettings(10000000, MSBFIRST, SPI_MODE0);  // MFRC522SPI最高速率为10Mbit/s，高位先发送 [数据手册第8.1.2部分]
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//MFRC522底层函数
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 功    能：写寄存器
* 参数说明：reg[输入]：要写入的寄存器的地址,MFRC522_Register枚举中的成员
*           value[输入]：要写入的数据
*/
void MFRC522::WriteReg(byte reg, byte value) {
	SPI.beginTransaction(spisetting);
	digitalWrite(_chipSelectPin, LOW);
	SPI.transfer((reg << 1) & 0x7E);	// 写寄存器地址格式：0XXXXXXX0，高位0表示写，低位0不用于地址 [数据手册第8.1.2.3部分]
	SPI.transfer(value);
	digitalWrite(_chipSelectPin, HIGH);
	SPI.endTransaction();
}

/*
* 功    能：写寄存器
* 参数说明：reg[输入]：要写入的寄存器的地址，MFRC522_Register枚举中的成员
*			count[输入]：要写入数据的字节数
*           * value[输入]：要写入的数据，byte数组
*/
void MFRC522::WriteReg(byte reg, byte count, byte * values) {
	SPI.beginTransaction(spisetting);
	digitalWrite(_chipSelectPin, LOW);
	SPI.transfer((reg << 1) & 0x7E);				// 写寄存器地址格式：0XXXXXXX0，高位0表示写，低位0不用于地址 [数据手册第8.1.2.3部分]
	for (byte index = 0; index < count; index++) {
		SPI.transfer(values[index]);
	}
	digitalWrite(_chipSelectPin, HIGH);
	SPI.endTransaction();
}

/*
* 功    能：读寄存器
* 参数说明：reg[输入]：要读取的寄存器的地址，MFRC522_Register枚举中的成员
* 返    回：读取到的数据
*/
byte MFRC522::ReadReg(byte reg) {
	byte value;
	SPI.beginTransaction(spisetting);
	digitalWrite(_chipSelectPin, LOW);
	SPI.transfer(((reg << 1) & 0x7E) | 0x80);	// 读寄存器地址格式：1XXXXXXX0，高位1表示读，低位0不用于地址 [数据手册第8.1.2.3部分]
	value = SPI.transfer(0x00);
	digitalWrite(_chipSelectPin, HIGH);
	SPI.endTransaction();
	return value;
}

/*
* 功    能：读寄存器
* 参数说明：reg[输入]：要读取的寄存器的地址，MFRC522_Register枚举中的成员
*			count[输入]：要读取数据的字节数
*           * value[输出]：存储读取数据的缓冲区，byte数组
*/
void MFRC522::ReadReg(byte reg, byte count, byte * values) {
	byte index = 0;
	byte address = ((reg << 1) & 0x7E) | 0x80;		// 读寄存器地址格式：1XXXXXXX0，高位1表示读，低位0不用于地址 [数据手册第8.1.2.3部分]
	SPI.beginTransaction(spisetting);
	digitalWrite(_chipSelectPin, LOW);
	SPI.transfer(address);							
	for (index = 0; index < count - 1; index++) {
		values[index] = SPI.transfer(address);
	}
	values[index] = SPI.transfer(0);
	digitalWrite(_chipSelectPin, HIGH);
	SPI.endTransaction();
}

/*
* 功    能：置寄存器位
* 参数说明：reg		[输入]：要置位的寄存器的地址，MFRC522_Register枚举中的成员
*			mask	[输入]：置位值
*/
void MFRC522::SetRegBitMask(byte reg, byte mask) {
	byte temp;
	temp = ReadReg(reg);
	WriteReg(reg, temp | mask);
}

/*
* 功    能：清寄存器位
* 参数说明：reg		[输入]：要清位的寄存器的地址，MFRC522_Register枚举中的成员
*			mask	[输入]：清位值
*/
void MFRC522::ClearRegBitMask(byte reg, byte mask) {
	byte temp;
	temp = ReadReg(reg);
	WriteReg(reg, temp | (~mask));
}

/*
* 功    能：使用MFRC522的CRC协处理器计算CRC
* 参数说明：* data		[输入]：指向要传至FIFO进行CRC计算的值
*			length		[输入]：传输的字节数
*			* result	[输出]：指向结果缓存区，结果写入result[0..1]，低字节在前
* 返    回：STATUS_OK表示成功，其他结果参照状态码
*/
MFRC522::StatusCode MFRC522::CalculateCRC(byte * data, byte length, byte * result) {
	WriteReg(CommandReg, RC522_CMD_Idle);		// 停止任何在活动的命令
	WriteReg(DivIrqReg, 0x04);					// 清除CRCIRq中断请求位
	SetRegBitMask(FIFOLevelReg, 0x80);			// FlushBuffer置位，FIFO初始化
	WriteReg(FIFODataReg, length, data);		// 数据写入FIFO
	WriteReg(CommandReg, RC522_CMD_CalcCRC);	// 开始CRC计算
												
	word i = 5000;								// 等待CRC计算完成，该while循环的每次迭代预计需要17.73us
	byte n;
	while (1) {
		n = ReadReg(DivIrqReg);					// DivIrqReg[7..0]的内容为: Set2 保留 保留 MfinActIRq 保留 CRCIRq 保留 保留
		if (n & 0x04) {							// CRCIRq置位，计算完成
			break;
		}
		if (--i == 0) {							// 89ms内没完成，超时终止，与RC522的通讯可能断开了
			return STATUS_TIMEOUT;
		}
	}
	WriteReg(CommandReg, RC522_CMD_Idle);		// 停止CRC计算，新内容可以写入FIFO

	result[0] = ReadReg(CRCResultRegL);			// 将结果从寄存器传至结果缓存区
	result[1] = ReadReg(CRCResultRegH);
	return STATUS_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//MFRC522功能函数
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
* 功    能：初始化MFRC522
*/
void MFRC522::RC522_Init() {
	pinMode(_chipSelectPin, OUTPUT);		// 设置chipSelectPin为数字输出
	digitalWrite(_chipSelectPin, HIGH);		// 暂时还不用工作
	pinMode(_resetPin, OUTPUT);				// 设置resetPin为数字输出

	SPI_Config();							// 初始化SPI总线

	if (LOW == digitalRead(_resetPin)) {	// 芯片在硬掉电模式下
		digitalWrite(_resetPin, HIGH);		// 退出硬掉电模式，触发硬件重置
		delay(50);							// 等待晶振起振和稳定，至少需要t_startup(启动时间)+t_delay(37.74us)，这里给50ms足够了 [数据手册8.8.2部分]
	}
	else {
		RC522_Reset();						// 进行软重置
	}

	// 设置与RFID标签通讯时的超时时间
	// 定时器频率 = 13.56MHz/(TPrescaler*2+1)，TPrescaler的值由TModeReg的低4位和TPrescalerReg组成
	WriteReg(TModeReg, 0x80);				// TAuto置位，定时器在所有通讯模式所有速率的发送结束时自动启动，在接收到第一个数据位后定时器立即停止
	WriteReg(TPrescalerReg, 0xA9);			// TPrescaler=TModeReg[3..0]:TPrescalerReg[7..0]，这里为0x00A9 = 168 => 定时器频率=40kHz，周期为25us
	WriteReg(TReloadRegH, 0x03);			// 定时器重装值：0x3E8 = 1000，即超时时间为25ms
	WriteReg(TReloadRegL, 0xE8);

	WriteReg(TxASKReg, 0x40);				// 默认为0x00，强制100%ASK调制
	WriteReg(ModeReg, 0x3D);				// 默认为0x3F，设置CRC协处理器的预设值为0x6363 [ISO 14443-3 section 6.2.4]
	RC522_AntennaOn();						// 使能天线驱动器引脚TX1和TX2(重置会自动关闭)
}

/*
* 功    能：重置MFRC522
*/
void MFRC522::RC522_Reset() {
	WriteReg(CommandReg, RC522_CMD_SoftReset);	// 发送软重置命令
	delay(50);									// 等待晶振起振和稳定，至少需要t_startup(启动时间)+t_delay(37.74us)，这里给50ms足够了[数据手册8.8.2部分]
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
* 参数列表：
*			参数	增益
* 			0x00	18dB
* 			0x10	23dB
* 			0x20	18dB
* 			0x30	23dB
*			0x40	33dB
*			0x50	38dB
*			0x60	43dB
*			0x70	48dB
*/
byte MFRC522::RC522_GetAntennaGain() {
	return ReadReg(RFCfgReg) & (0x07 << 4);
}

/*
* 功    能：获取当前MFRC522接收器增益值
* 参数说明：mask[输入]：RxGain的值<<4 [数据手册9.3.3.6部分]
* 参数列表：	
*			参数	增益
* 			0x00	18dB
* 			0x10	23dB
* 			0x20	18dB
* 			0x30	23dB
*			0x40	33dB
*			0x50	38dB
*			0x60	43dB
*			0x70	48dB
*/
void MFRC522::RC522_SetAntennaGain(byte mask) {
	if (RC522_GetAntennaGain() != mask) {
		ClearRegBitMask(RFCfgReg, (0x07 << 4));
		SetRegBitMask(RFCfgReg, mask & (0x07 << 4));
	}
}

/*
| 功    能：传送数据至FIFO缓冲区，执行命令，等待完成并接收FIFO内收到的新数据
| 参数说明：command			[输入]：		要执行的命令，RC522_Command枚举中的成员
|			* sendData		[输入]：		指向要发送至FIFO的数据
|			sendLen			[输入]：		要发送至FIFO的数据的字节数
|			* receiveData	[输出]：		无（默认） 或 指向用于接收FIFO内新数据的缓冲区
|			* receiveLen	[输入/输出]：	无（默认） 或 [输入]：receiveData缓冲区的最大字节数；[输出]：实际接收到的字节数
|			* validBits		[输入/输出]：	无（默认） 或 最后一字节中有效位的位数，[输入]：对于发送数据的最后一字节中有效位的位数（既sendData[0]）；[输出]：对于接收到数据的最后一字节（既receiveData[0]）
|			rxAlign			[输入]：		定义receiveData[0]中收到第一个位的存储位置
*/
MFRC522::StatusCode MFRC522::RC522_CommunicateWithPICC(byte command, byte * sendData, byte sendLen, byte * receiveData , byte * receiveLen , byte * validBits , byte rxAlign) {
	byte n, waitIRq, _validBits;
	unsigned int i;

	byte txLastBits = validBits ? *validBits : 0;
	byte bitFraming = (rxAlign << 4) + txLastBits;	// RxAlign=BitFramingReg[6..4]，TxLastBits=BitFramingReg[2..0] [数据手册9.3.1.14部分]

	switch (command) {								// 不同的命令需要等待不同的中断信号
		case RC522_CMD_Transceive:
			waitIRq = 0x30;							// 接收RxIRq、IdleIRq中断
			break;
		default:
			break;
	}

	WriteReg(CommandReg, RC522_CMD_Idle);			// 停止任何在活动的命令
	ClearRegBitMask(ComIrqReg, 0x80);				// 清除ComIrqReg中所有的中断请求标志位
	SetRegBitMask(FIFOLevelReg, 0x80);				// FlushBuffer置位，FIFO初始化

	WriteReg(FIFODataReg, sendLen, sendData);		// 发送数据写入FIFO
	WriteReg(BitFramingReg, bitFraming);			// 调整帧格式
	WriteReg(CommandReg, command);					// 执行命令
	if (command == RC522_CMD_Transceive) {
		SetRegBitMask(BitFramingReg, 0x80);			// StartSend置位，开始传送数据
	}

	i = 2000;										// 等待命令执行完成，该while循环的每次迭代估计需要17.86us
	while (1) {
		n = ReadReg(ComIrqReg);						// ComIrqReg[7..0]的内容为：Set1 TxIRq RxIRq IdleIRq HiAlertIRq LoAlertIRq ErrIRq TimerIRq
		if (n & waitIRq) {							// 一个中断信号被成功设置
			break;
		}
		if (n & 0x01) {								// 定时器中断，25ms内未收到任何数据（在PCD_Init()已将TModeReg寄存器的TAuto置位，RC522发送完毕后自动启动定时器）
			return STATUS_NO_TAG;					// 未检测到标签
		}
		if (--i == 0) {								// 35.7ms内没完成，超时终止，与RC522的通讯可能断开了
			return STATUS_TIMEOUT;
		}
	}

	byte errorRegValue = ReadReg(ErrorReg);
	if (errorRegValue & 0x13) {						// 出现以下错误中断：BufferOvfl ParityErr ProtocolErr
		return STATUS_ERROR;
	}

	if (receiveData && receiveLen) {				// 判断是否需要接收数据
		n = ReadReg(FIFOLevelReg);					// FIFO内数据的字节数
		if (n > *receiveLen) {
			return STATUS_NO_ROOM;
		}
		*receiveLen = n;							// 接收到的数据的字节数
		ReadReg(FIFODataReg, n, receiveData);		// 从FIFO中获取接收到的数据
		_validBits = ReadReg(ControlReg) & 0x07;	// RxLastBits[2:0]表示最后接收到的字节中有效的位数，如果全零，整个字节有效 [数据手册9.3.1.13部分]
		if (validBits) {
			*validBits = _validBits;
		}
	}

	if (errorRegValue & 0x08) {						// 检测到碰撞，CollErr被置位
		return STATUS_COLLISION;
	}

	return STATUS_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//标签操作函数
////////////////////////////////////////////////////////////////////////////////////////////////////
/*
| 功    能：发送REQA或WUPA命令，可用于检测天线范围内是否存在标签
| 参数说明：requsetCommand	[输入]：请求命令，PICC_CMD_REQA或PICC_CMD_WUPA中的一个
|			* bufferATQA	[输出]：指向用于接收ATGA的缓冲区（2字节）
| 返    回：STATUS_OK表示成功，其他结果参照状态码
*/
MFRC522::StatusCode MFRC522::PICC_Request(byte requsetCommand, byte * bufferATQA) {
	byte validBits;
	MFRC522::StatusCode status;

	byte bufferSize = sizeof(bufferATQA);		// 读取ATQA缓冲区的大小
	if (bufferSize < 2) {						// ATQA的长度应为2字节
		return STATUS_NO_ROOM;
	}
	ClearRegBitMask(CollReg, 0x80);
	validBits = 7;
	status = RC522_CommunicateWithPICC(RC522_CMD_Transceive, &requsetCommand, 1, bufferATQA, &bufferSize, &validBits);
	if (status != STATUS_OK) {
		return status;
	}
	if (bufferSize != 2 || validBits != 0) {	// ATQA需正好2字节
		return STATUS_ERROR;
	}
	return STATUS_OK;
}

/*
| 功    能：发送防碰撞命令，获取卡号
| 参数说明：* bufferUID	[输出]：指向用于接收UID的缓冲区
| 返    回：STATUS_OK表示成功，其他结果参照状态码
*/
MFRC522::StatusCode MFRC522::PICC_Anticollision(byte *bufferUID) {
	byte i, sendData[2], uidCheck = 0;
	MFRC522::StatusCode status;

	ClearRegBitMask(CollReg, 0x80);

	byte buffersize = 5;					// 目前只能读取S50卡序列号
	sendData[0] = PICC_CMD_SEL_CL1;
	sendData[1] = 0x20;
	status = RC522_CommunicateWithPICC(RC522_CMD_Transceive, sendData, 2, bufferUID, &buffersize);
	if (status != STATUS_OK) {
		return status;
	}

	for (i = 0; i < 4; ++i) {				// 校验序列号
		uidCheck ^= bufferUID[i];
	}
	if (uidCheck != bufferUID[i]) {
		return STATUS_ERROR;
	}

	return STATUS_OK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//辅助函数
////////////////////////////////////////////////////////////////////////////////////////////////////
String MFRC522::GetStatusCodeName(MFRC522::StatusCode code) {
	switch (code) {
		case STATUS_OK:				return "OK";
		case STATUS_ERROR:			return "Error in communication.";
		case STATUS_TIMEOUT:		return "Timeout in communication.";
		case STATUS_NO_TAG:			return "No tag detected.";
		case STATUS_NO_ROOM:		return "A buffer is not big enough.";
		case STATUS_COLLISION:		return "Collission detected.";
		default:					return "Unknown error.";
	}
	return;
}