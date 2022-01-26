/*

  PrintUTF8.ino
  
  Use the (Arduino compatible) u8g2 function "print"  to draw a text.

  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)

  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  

*/

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/5, /* data=*/4, /* reset=*/U8X8_PIN_NONE);

int default_bullet = 9;
int is_first = 1;
int pinInterrupt = D3; //接中断信号的脚

int current_bullet = 9;
int max_bullet = 9;
int pullup_lock = 0;
void setup(void)
{
    Serial.begin(9600);                  //打开串口
    pinMode(pinInterrupt, INPUT_PULLUP); //设置管脚为输入
    pinMode(D5, OUTPUT);
    digitalWrite(D5, LOW); //先保证拉低
    ServoControlOne(D5, 0);

    attachInterrupt(digitalPinToInterrupt(pinInterrupt), onChange, FALLING);
    u8g2.begin();
    u8g2.enableUTF8Print(); // enable UTF8 support for the Arduino print() function
    u8g2.setFontDirection(0);
}

void loop(void)
{
    //timer();
    if (is_first)
    {
        //首次启动，随便显示一些什么
        welcome();
        is_first = 0;
        ServoControlOne(D5, 0);

    }
}

ICACHE_RAM_ATTR void onChange()
{
    //esp8266需要加 ICACHE_RAM_ATTR
    if (digitalRead(pinInterrupt) == LOW)
    {
        if (pullup_lock == 0)
        {
            pullup_lock = 1;
            launch();
            delay(500);
            pullup_lock = 0;
        }
    }
}
void welcome(void)
{

    u8g2.clearBuffer();
    u8g2.setFontDirection(0);
    for (int i = 0; i < 100; i += 10)
    {
        u8g2.clearBuffer();
        progressBar(u8g2, 15, 30, 90, 10, i);
        u8g2.sendBuffer(); // transfer internal memory to the display
        delay(100);
    }
    u8g2.clearBuffer();
    u8g2.setFontDirection(0);
    u8g2.setFont(u8g2_font_open_iconic_other_8x_t);
    u8g2.drawGlyph(30, 65, 71);
    u8g2.sendBuffer(); // transfer internal memory to the display
}

void launch()
{
    if (current_bullet == 0)
    {
        current_bullet = max_bullet;
    }
    else
    {
        u8g2.setFont(u8g2_font_7Segments_26x42_mn); // use chinese2 for all the glyphs of "你好世界"
        u8g2.setFontDirection(0);
        u8g2.clearBuffer();
        char buff[8];
        sprintf_P(buff, PSTR("%d"), current_bullet);
        u8g2.drawStr(50, 50, buff);

        drawBattery(u8g2, 100, 8, 22, 50, 10, current_bullet);
        u8g2.sendBuffer();
        current_bullet--;
        ServoControlOne(D5, 180);

    }
}

void progressBar(U8G2 u8g2, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t percent)
{
    // can't draw it smaller than 10x8
    height = height < 8 ? 8 : height;
    width = width < 10 ? 10 : width;

    // draw percentage
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawStr(x + width + 2, y + height / 2 + 2, (String(percent) + String("%")).c_str());

    // draw it
    u8g2.drawRFrame(x, y, width, height, 4);
    u8g2.drawBox(x + 2, y + 2, (width - 4) * (percent / 100.0), height - 4);
}
void timer(void)
{
    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_7Segments_26x42_mn); // use chinese2 for all the glyphs of "你好世界"
        u8g2.setFontDirection(0);
        u8g2.clearBuffer();
        for (int i = max_bullet; i > 0; i--)
        {
            char buff[8];
            sprintf_P(buff, PSTR("%d"), i);
            u8g2.drawStr(50, 50, buff);
            u8g2.sendBuffer();
            delay(1000);
        }

    } while (u8g2.nextPage());
}

void drawBattery(U8G2 u8g2, int x, int y, int w, int h, int segments, int lvl)
{
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_5x8_tf);
    u8g2.drawFrame(x, y, w, h);
    //u8g2.drawFrame(x + w / 3, y - 2, w / 3, 2);

    for (int i = 0; i < segments; i++)
    {
        if ((segments - i) > lvl)
        {
            u8g2.drawFrame(x + 2, y + i * h / segments + 2, w - 4, h / (segments + 1) - 1);
        }
        else
        {
            u8g2.drawBox(x + 2, y + i * h / segments + 2, w - 3, h / (segments + 1) - 1);
        }
    }
}

void ServoControlOne(uint8_t ServoPin, int servoAngle)
{

    double thisAngle = map(servoAngle, 0, 180, 500, 2500); //等比例角度值范围转换高电平持续时间范围
    unsigned char i = 50;                                  //50Hz 每秒的周期次数(周期/秒) 即1S 50 个周期 每个周期20ms
    while (i--)
    {
        digitalWrite(ServoPin, HIGH);
        delayMicroseconds(thisAngle); //高电平时间
        digitalWrite(ServoPin, LOW);
        delayMicroseconds(20000 - thisAngle); //每个周期20ms减去高电平持续时间
    }
}
