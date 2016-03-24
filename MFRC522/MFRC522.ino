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
  
}
