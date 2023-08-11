/*!
 * @file Master.ino
 * @brief LoRa Radio Module-433MHZ.
 * @n [Get the module here]()
 * @n This example is a Master of the application.
 * @n [Connection and Diagram](https://wiki.dfrobot.com.cn/index.php?title=(SKU:TEL0116)LoRa_Radio_Module-433MHZ)
 *
 * @copyright  [DFRobot](https://www.dfrobot.com), 2016
 * @copyright GNU Lesser General Public License
 *
 * @author [lijun](ju.li@dfrobot.com)
 * @version  V1.0
 * @date  2017-01-20
 */

#include <Wire.h>
#include "DFRobot_RGBLCD.h"
#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX
DFRobot_RGBLCD lcd(0x7c >> 1, 0xc0 >> 1, 16, 2); //parameter1:LCD address; parameter2:RGB address; 16 characters and 2 lines of show


const char EN = 4;  //
const char AUX = 5; //


unsigned int buffer_RTT[64] = {}; //


void setup()
{
  pinMode(EN, OUTPUT);
  //pinMode(AUX, INPUT);
  Serial.begin(9600);
  mySerial.begin(9600);
  lcd.init();
  digitalWrite(EN, LOW);    //低电平无线模块工作，高电平模块休�
}

void loop()
{
  while (Serial.available() < 1); //等待PC串口数据
  while (Serial.available() > 0)
  {
    char data;
    data = Serial.read();
    delay(2);
    mySerial.write(data);  //转发串口数据给无线模块
  }
  Serial.flush();
  mySerial.flush();
  while (mySerial.available() < 1);  //等待无线模块数据
  while (mySerial.available() > 0)
  {
    char data;
    for (int i = 0; i < 64; i++)
    {
      data = mySerial.read();
      buffer_RTT[i] = (char)data;  //接收无线模块数据
      Serial.write(data);  //转发无线模块数据给串口
      delay(2);
    }
    mySerial.flush();

  }

  lcd.setCursor(0, 0 );
  for (int i = 0; i < 16; i++) lcd.write(buffer_RTT[i]);  //显示前32个接收的数据
  lcd.setCursor(0, 1 );
  for (int i = 16; i < 32; i++) lcd.write(buffer_RTT[i]);  //
  delay(1000);
}