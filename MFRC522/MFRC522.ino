/*
 Name:		MFRC522.ino
 Created:	2016/3/24 17:00:51
 Author:	ycf
 Editor:	http://www.visualmicro.com
*/

#include "MFRC522Lib.h"

MFRC522 rc522(10, 9);

void setup() {
	Serial.begin(115200);
	rc522.RC522_Init();
	Serial.println("Init:OK!");
}

void loop() {
	byte bufferATQA[2];
	byte bufferSize = sizeof(bufferATQA);
	MFRC522::StatusCode status = rc522.PICC_Request(rc522.PICC_CMD_REQA, bufferATQA, &bufferSize);
	if (status != rc522.STATUS_TIMEOUT) {
		Serial.println(rc522.GetStatusCodeName(status));
	}
	if (status == rc522.STATUS_OK) {
		Serial.print(bufferATQA[0],HEX);
		Serial.print(",");
		Serial.println(bufferATQA[1],HEX);
	}
	delay(100);
}
