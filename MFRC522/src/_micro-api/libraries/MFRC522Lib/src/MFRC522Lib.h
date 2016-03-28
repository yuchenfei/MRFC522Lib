/*
 Name:		MFRC522Lib.h
 Created:	2016/3/24 17:00:51
 Author:	ycf
 Editor:	http://www.visualmicro.com
*/

#ifndef _MFRC522Lib_h
#define _MFRC522Lib_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <SPI.h>

class MFRC522 {
public:
	////////////////////////////////////////////////////////////////////////////////////////////////////
	//MFRC522寄存器
	////////////////////////////////////////////////////////////////////////////////////////////////////
	enum RC522_Register {
		// Page 0: Command and status
		Reserved00		= 0x00,		// 保留
		CommandReg		= 0x01,		// 启动和停止命令的执行
		ComIEnReg		= 0x02,		// 中断请求使能控制位
		DivIEnReg		= 0x03,		// 中断请求使能控制位
		ComIrqReg		= 0x04,		// 包含中断请求标志
		DivIrqReg		= 0x05,		// 包含中断请求标志
		ErrorReg		= 0x06,		// 错误标志，指示执行的上个命令的错误状态
		Status1Reg		= 0x07,		// 包含通讯状态标志
		Status2Reg		= 0x08,		// 包含发送器和接收器的状态标志
		FIFODataReg		= 0x09,		// 64字节FIFO缓冲区的输入和输出
		FIFOLevelReg	= 0x0A,		// FIFO中存储的字节数
		WaterLevelReg	= 0x0B,		// 定义FIFO下溢和上溢警告的阈值
		ControlReg		= 0x0C,		// 多种控制寄存器
		BitFramingReg	= 0x0D,		// 面向位帧的调整
		CollReg			= 0x0E,		// RF接口上检测到的第一个位冲突的位置
		Reserved0F		= 0x0F,		// 保留

		// Page 1: Command
		Reserved10		= 0x10,		// 保留
		ModeReg			= 0x11,		// 定义发送和接收的常用模式
		TxModeReg		= 0x12,		// defines transmission data rate and framing
		RxModeReg		= 0x13,		// defines reception data rate and framing
		TxControlReg	= 0x14,		// 控制天线驱动引脚TX1和TX2的逻辑行为
		TxASKReg		= 0x15,		// 控制传输调制的设置
		TxSelReg		= 0x16,		// selects the internal sources for the antenna driver
		RxSelReg		= 0x17,		// selects internal receiver settings
		RxThresholdReg	= 0x18,		// selects thresholds for the bit decoder
		DemodReg		= 0x19,		// defines demodulator settings
		Reserved1A		= 0x1A,		// reserved for future use
		Reserved1B		= 0x1B,		// reserved for future use
		MfTxReg			= 0x1C,		// controls some MIFARE communication transmit parameters
		MfRxReg			= 0x1D,		// controls some MIFARE communication receive parameters
		Reserved1E		= 0x1E,		// 保留
		SerialSpeedReg	= 0x1F,		// selects the speed of the serial UART interface

		// Page 2: Configuration
		Reserved20		= 0x20,		// 保留
		CRCResultRegH	= 0x21,		// shows the MSB and LSB values of the CRC calculation
		CRCResultRegL	= 0x22,
		Reserved23		= 0x23,		// 保留
		ModWidthReg		= 0x24,		// controls the ModWidth setting?
		Reserved25		= 0x25,		// 保留
		RFCfgReg		= 0x26,		// 配置接收器增益
		GsNReg			= 0x27,		// selects the conductance of the antenna driver pins TX1 and TX2 for modulation
		CWGsPReg		= 0x28,		// defines the conductance of the p-driver output during periods of no modulation
		ModGsPReg		= 0x29,		// defines the conductance of the p-driver output during periods of modulation
		TModeReg		= 0x2A,		// 定义内部定时器的设置
		TPrescalerReg	= 0x2B,		// 预分频器重装值的低8位，高4位在TModeReg中
		TReloadRegH		= 0x2C,		// 定义了16位定时器的重装值
		TReloadRegL		= 0x2D,
		TCounterValueRegH = 0x2E,	// 显示16位定时器的当前值
		TCounterValueRegL = 0x2F,

		// Page 3: Test Registers
		Reserved30		= 0x30,		// 保留
		TestSel1Reg		= 0x31,		// general test signal configuration
		TestSel2Reg		= 0x32,		// general test signal configuration
		TestPinEnReg	= 0x33,		// enables pin output driver on pins D1 to D7
		TestPinValueReg = 0x34,		// defines the values for D1 to D7 when it is used as an I/O bus
		TestBusReg		= 0x35,		// shows the status of the internal test bus
		AutoTestReg		= 0x36,		// controls the digital self test
		VersionReg		= 0x37,		// shows the software version
		AnalogTestReg	= 0x38,		// controls the pins AUX1 and AUX2
		TestDAC1Reg		= 0x39,		// defines the test value for TestDAC1
		TestDAC2Reg		= 0x3A,		// defines the test value for TestDAC2
		TestADCReg		= 0x3B,		// shows the value of ADC I and Q channels
		Reserved3C		= 0x3C,		// reserved for production tests
		Reserved3D		= 0x3D,		// reserved for production tests
		Reserved3E		= 0x3E,		// reserved for production tests
		Reserved3F		= 0x3F		// reserved for production tests
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//MFRC522命令
	////////////////////////////////////////////////////////////////////////////////////////////////////
	enum RC522_Command {
		RC522_CMD_Idle				= 0x00,	// 无动作，取消当前命令的执行
		RC522_CMD_Mem				= 0x01,	// stores 25 bytes into the internal buffer
		RC522_CMD_GenerateRandomID	= 0x02,	// generates a 10-byte random ID number
		RC522_CMD_CalcCRC			= 0x03,	// 激活CRC协处理器或执行自检
		RC522_CMD_Transmit			= 0x04,	// transmits data from the FIFO buffer
		RC522_CMD_NoCmdChange		= 0x07,	// no command change, can be used to modify the CommandReg register bits without affecting the command, for example, the PowerDown bit
		RC522_CMD_Receive			= 0x08,	// activates the receiver circuits
		RC522_CMD_Transceive		= 0x0C,	// transmits data from FIFO buffer to antenna and automatically activates the receiver after transmission
		RC522_CMD_MFAuthent			= 0x0E,	// performs the MIFARE standard authentication as a reader
		RC522_CMD_SoftReset			= 0x0F	// 软复位MFRC522
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//标签相关指令
	////////////////////////////////////////////////////////////////////////////////////////////////////
	enum PICC_Command {
		// 读写器与几种标签通信的管理命令有：REQA、WUPA、ANTICOLLISION、SELECT、HLTA [ISO 14443-3 section 6.2.4]
		PICC_CMD_REQA			= 0x26,	// 寻找天线范围内未睡眠的卡，标签由“空闲”状态进入“准备”状态，准备进行防碰撞或选择，命令长度为7位
		PICC_CMD_WUPA			= 0x52,	// 寻找天线范围内所有的卡，标签由“空闲”、“睡眠”状态进入“准备”状态，准备进行防碰撞或选择，命令长度为7位
		PICC_CMD_CT				= 0x88,	// Cascade Tag. Not really a command, but used during anti collision.
		PICC_CMD_SEL_CL1		= 0x93,	// Anti collision/Select, Cascade Level 1
		PICC_CMD_SEL_CL2		= 0x95,	// Anti collision/Select, Cascade Level 2
		PICC_CMD_SEL_CL3		= 0x97,	// Anti collision/Select, Cascade Level 3
		PICC_CMD_HLTA			= 0x50,	// HaLT command, Type A. Instructs an ACTIVE PICC to go to state HALT.
										// The commands used for MIFARE Classic (from http://www.mouser.com/ds/2/302/MF1S503x-89574.pdf, Section 9)
										// Use PCD_MFAuthent to authenticate access to a sector, then use these commands to read/write/modify the blocks on the sector.
										// The read/write commands can also be used for MIFARE Ultralight.
		PICC_CMD_MF_AUTH_KEY_A	= 0x60,	// Perform authentication with Key A
		PICC_CMD_MF_AUTH_KEY_B	= 0x61,	// Perform authentication with Key B
		PICC_CMD_MF_READ		= 0x30,	// Reads one 16 byte block from the authenticated sector of the PICC. Also used for MIFARE Ultralight.
		PICC_CMD_MF_WRITE		= 0xA0,	// Writes one 16 byte block to the authenticated sector of the PICC. Called "COMPATIBILITY WRITE" for MIFARE Ultralight.
		PICC_CMD_MF_DECREMENT	= 0xC0,	// Decrements the contents of a block and stores the result in the internal data register.
		PICC_CMD_MF_INCREMENT	= 0xC1,	// Increments the contents of a block and stores the result in the internal data register.
		PICC_CMD_MF_RESTORE		= 0xC2,	// Reads the contents of a block into the internal data register.
		PICC_CMD_MF_TRANSFER	= 0xB0,	// Writes the contents of the internal data register to a block.
										// The commands used for MIFARE Ultralight (from http://www.nxp.com/documents/data_sheet/MF0ICU1.pdf, Section 8.6)
										// The PICC_CMD_MF_READ and PICC_CMD_MF_WRITE can also be used for MIFARE Ultralight.
		PICC_CMD_UL_WRITE		= 0xA2	// Writes one 4 byte page to the PICC.
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//返回状态码
	////////////////////////////////////////////////////////////////////////////////////////////////////
	enum StatusCode {
		STATUS_OK			= 1,	// 成功
		STATUS_ERROR		= 2,	// 通讯错误
		STATUS_TIMEOUT		= 3,	// 通讯超时
		STATUS_NO_ROOM		= 4,	// 存储空间不足
		STATUS_COLLISION	= 5		// 发生碰撞
	};

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//FIFO缓冲区大小
	////////////////////////////////////////////////////////////////////////////////////////////////////
	static const byte FIFO_SIZE = 64;

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//构造函数
	////////////////////////////////////////////////////////////////////////////////////////////////////
	MFRC522();
	MFRC522(byte chipSelectPin, byte resetPin); //硬件SPI

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//SPI配置函数
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void SPI_Config();
	//void SPI_Write(byte data);
	//byte SPI_Read(void);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//MFRC522底层函数
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void WriteReg(byte reg, byte value);
	void WriteReg(byte reg, byte count, byte *values);
	byte ReadReg(byte reg);
	void ReadReg(byte reg, byte count, byte *values);
	void SetRegBitMask(byte reg, byte mask);
	void ClearRegBitMask(byte reg, byte mask);
	MFRC522::StatusCode CalculateCRC(byte *data, byte length, byte *result);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//MFRC522功能函数
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void RC522_Init();
	void RC522_Reset();
	void RC522_AntennaOn();
	void RC522_AntennaOff();
	byte RC522_GetAntennaGain();
	void RC522_SetAntennaGain(byte mask);
	MFRC522::StatusCode RC522_CommunicateWithPICC(byte command, byte *sendData, byte sendLen, byte *receiveData = NULL, byte *receiveLen = NULL, byte *validBits = NULL, byte rxAlign = 0);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//标签操作函数
	////////////////////////////////////////////////////////////////////////////////////////////////////
	MFRC522::StatusCode PICC_Request(byte reqCommand, byte *bufferATQA, byte *bufferSize);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//辅助函数
	////////////////////////////////////////////////////////////////////////////////////////////////////
	String GetStatusCodeName(MFRC522::StatusCode code);

private:
	byte _chipSelectPin;    //SPI从机使能引脚（Pin 24, NSS,低电平使能）
	byte _resetPin;         //复位引脚（Pin 6, NRSTPD，低电平使能）
	//bool _hardwareSPI;      //真表示使用硬件SPI，假表示使用软件SPI
	SPISettings spisetting; //SPI设置
};
#endif

