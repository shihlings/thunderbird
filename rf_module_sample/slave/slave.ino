/*!
 * @file Slave.ino
 * @brief LoRa Radio Module-433MHZ.
 * @n [Get the module here]()
 * @n This example is a Slave of the application.
 * @n [Connection and Diagram](https://wiki.dfrobot.com.cn/index.php?title=(SKU:TEL0116)LoRa_Radio_Module-433MHZ)
 *
 * @copyright  [DFRobot](https://www.dfrobot.com), 2016
 * @copyright GNU Lesser General Public License
 *
 * @author [lijun](ju.li@dfrobot.com)
 * @version  V1.0
 * @date  2017-01-20
 */

#include <SoftwareSerial.h>

SoftwareSerial mySerial(10, 11); // RX, TX



const char EN = 4;  //
const char AUX = 5; //
const char LED = 13; //


unsigned int buffer_receive[40] = {};
unsigned int buffer_RTT[40] = {};
int flag = HIGH;

void setup()
{
  pinMode(EN, OUTPUT); //
  pinMode(AUX, INPUT); //
  pinMode(LED, OUTPUT);
  Serial.begin(9600);
  mySerial.begin(9600);

}

void loop()
{
  digitalWrite(EN, HIGH);  //低电平无线模块工作，高电平模块休�
  while (digitalRead(AUX) != 0);  //检测是否接收到信息，并输出低电平给单片机，用于唤醒无线模块
  digitalWrite(EN, LOW);  //
  while (mySerial.available() < 1);  //等待接收无线数据
  while (mySerial.available() > 0)
  {
    char data;
    for (int i = 0; i < 40; i++)
    {
      data = mySerial.read();
      buffer_RTT[i] = (char)data;  //接收无线数据
      delay(2);
    }
    mySerial.flush();
  }

  while (buffer_RTT[0] == 0x11 && buffer_RTT[1] == 0x22) //判断数据命令是否符合
  {
    digitalWrite(EN, LOW);  //
    digitalWrite(LED, flag);
    flag = !flag;
    Serial.print('!');  //发送!给PC串口
    delay(2);
    while (Serial.available() < 1);  //等待PC串口数据
    while (Serial.available() > 0)
    {
      char data;
      data = Serial.read();
      delay(2);
      mySerial.write(data);  //转发串口数据给无线模块
    }
    Serial.flush();
    mySerial.flush();
    buffer_RTT[40] = {0};
    break;
  }
}