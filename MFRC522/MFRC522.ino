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
	byte bufferATQA[2],bufferUID[5];
	MFRC522::StatusCode status = rc522.PICC_Request(rc522.PICC_CMD_REQA, bufferATQA);
	if (status != rc522.STATUS_NO_TAG) {
		Serial.println(rc522.GetStatusCodeName(status));
	}
	if (status == rc522.STATUS_OK) {
		status = rc522.PICC_Anticollision(bufferUID);
		Serial.print("ATQA:");
		Serial.print(bufferATQA[0],HEX);
		Serial.print(",");
		Serial.println(bufferATQA[1],HEX);
		if (status != rc522.STATUS_OK) {
			Serial.println(rc522.GetStatusCodeName(status));
		}
		else {
			Serial.print("UID:");
			for (int i = 0; i < 4; ++i) {
				Serial.print(bufferUID[i], HEX);
				Serial.print(" ");
			}
			Serial.println("");
		}
	}
	delay(100);
}
