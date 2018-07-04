/*
The MIT License (MIT)

Copyright (c) 2016-2017 winlin

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "SimpleDHT.h"
#include <limits.h>

int SimpleDHT::read(int pin, byte* ptemperature, byte* phumidity, byte pdata[40]) {
    int ret = SimpleDHTErrSuccess;

    float temperature = 0;
    float humidity = 0;
    if ((ret = read2(pin, &temperature, &humidity, pdata)) != SimpleDHTErrSuccess) {
        return ret;
    }

    if (ptemperature) {
        *ptemperature = (byte)(int)temperature;
    }

    if (phumidity) {
        *phumidity = (byte)(int)humidity;
    }

    return ret;
}

int SimpleDHT::confirm(int pin, int us, byte level) {
    int cnt = us / 10;
    if ((us % 10) > 0) {
        cnt++;
    }

    bool ok = false;
    for (int i = 0; i < cnt; i++) {
        delayMicroseconds(10);

        if (digitalRead(pin) != level) {
            ok = true;
            break;
        }
    }

    if (!ok) {
        return -1;
    }
    return SimpleDHTErrSuccess;
}


int SimpleDHT::levelTime(int pin, byte level)
{
/*
    int time = 0;

  // On AVR platforms use direct GPIO port access as it's much faster and better
  // for catching pulses that are 10's of microseconds in length:
  #ifdef __AVR
    uint8_t portState = level ? _bit : 0;
    while ( ( *portInputRegister( _port ) & _bit ) == portState )
    {
        delayMicroseconds(10);
	time += 10;
    }
  // Otherwise fall back to using digitalRead (this seems to be necessary on ESP8266
  // right now, perhaps bugs in direct port access functions?).
#else
    while ( digitalRead(pin) == level )
    {
        delayMicroseconds(10);
        time += 20;
    };
#endif
*/
    unsigned long time_start = micros();
    unsigned long time_end;

  #ifdef __AVR
    uint8_t portState = level ? _bit : 0;
    while ( ( *portInputRegister( _port ) & _bit ) == portState );
  #else
    while ( digitalRead( pin ) == level );
  #endif
    
    time_end = micros();
    unsigned long time;
    if ( time_end < time_start )
    {   // clock overflow (went back through 0)
             time = time_end + ULONG_MAX - time_start;        
    } else { time = time_end - time_start; }

    return time;
}


byte SimpleDHT::bits2byte(byte data[8]) {
    byte v = 0;
    for (int i = 0; i < 8; i++) {
        v += data[i] << (7 - i);
    }
    return v;
}

int SimpleDHT::parse(byte data[40], short* ptemperature, short* phumidity) {
    short humidity = bits2byte(data);
    short humidity2 = bits2byte(data + 8);
    short temperature = bits2byte(data + 16);
    short temperature2 = bits2byte(data + 24);
    byte check = bits2byte(data + 32);
    byte expect = (byte)humidity + (byte)humidity2 + (byte)temperature + (byte)temperature2;
    if (check != expect) {
        return SimpleDHTErrDataChecksum;
    }

    *ptemperature = temperature<<8 | temperature2;
    *phumidity = humidity<<8 | humidity2;

    return SimpleDHTErrSuccess;
}

int SimpleDHT11::read2(int pin, float* ptemperature, float* phumidity, byte pdata[40]) {
    int ret = SimpleDHTErrSuccess;

    byte data[40] = {0};
    if ((ret = sample(pin, data)) != SimpleDHTErrSuccess) {
        return ret;
    }

    short temperature = 0;
    short humidity = 0;
    if ((ret = parse(data, &temperature, &humidity)) != SimpleDHTErrSuccess) {
        return ret;
    }

    if (pdata) {
        memcpy(pdata, data, 40);
    }
    if (ptemperature) {
        *ptemperature = (int)(temperature>>8);
    }
    if (phumidity) {
        *phumidity = (int)(humidity>>8);
    }

    // For example, when remove the data line, it will be success with zero data.
    if (temperature == 0 && humidity == 0) {
        return SimpleDHTErrZeroSamples;
    }

    return ret;
}

int SimpleDHT11::sample(int pin, byte data[40]) {
    // empty output data.
    memset(data, 0, 40);

    // According to protocol: https://akizukidenshi.com/download/ds/aosong/DHT11.pdf
    // notify DHT11 to start:
    //    1. PULL LOW 20ms.
    //    2. PULL HIGH 20-40us.
    //    3. SET TO INPUT.
    pinMode(pin, OUTPUT);
    digitalWrite(pin, HIGH);
    delay(500);
    //delay(250);

    digitalWrite(pin, LOW);
    delay(20);

    digitalWrite(pin, HIGH);
    delayMicroseconds(25);

    pinMode(pin, INPUT);
    //delayMicroseconds(10);

    // DHT11 starting:
    //    1. PULL LOW 80us
    //    2. PULL HIGH 80us
    //delayMicroseconds(10);
    int t = levelTime( pin, LOW );
    //if (confirm(pin, 80, LOW)) {
    if (t < 60) {
        return SimpleDHTErrStartLow;
        //return (t == 0 ? 1234 : t);
	//return t;
    }
    //delayMicroseconds(20);
//    if (confirm(pin, 80, HIGH)) {
    t = levelTime( pin, HIGH );
    if ( t < 70 ) {  // specs says: 80
        return SimpleDHTErrStartHigh;
        //return (t == 0 ? 1234 : t);
        //return t;
    }

    // DHT11 data transmite:
    //    1. 1bit start, PULL LOW 50us
    //    2. PULL HIGH 26-28us, bit(0)
    //    3. PULL HIGH 70us, bit(1)
    for (int j = 0; j < 40; j++)
    {
//        if (confirm(pin, 50, LOW)) {
//            return SimpleDHTErrDataLow;
//        }
//        delayMicroseconds(10);

        t = levelTime( pin, LOW );
        if ( t < 35 ) { // specs says: 50us
            //return SimpleDHTErrDataLow;
            //return (t == 0 ? 1234 : t);
	    //return t;
        }
/*
        // read a bit, should never call method,
        // for the method call use more than 20us,
        // so it maybe failed to detect the bit0.
        bool ok = false;
        int tick = 0;
        for (int i = 0; i < 8; i++, tick++) {
            if (digitalRead(pin) != HIGH) {
                ok = true;
                break;
            }
            delayMicroseconds(10);
        }
        if (!ok) {
            return SimpleDHTErrDataRead;
        }
        data[j] = ( tick > 3 ? 1 : 0 );
*/
        //delayMicroseconds(10);

	t = levelTime( pin, HIGH );
        if (t < 12 )  // specs say: 20us
            //return SimpleDHTErrDataRead;
            return (t == 0 ? 1234 : t);
            //return t;
	data[ j ] = ( t > 40 ? 1 : 0 );
    }

    // DHT11 EOF:
    //    1. PULL LOW 50us.
    if (confirm(pin, 50, LOW)) {
        return SimpleDHTErrDataEOF;
    }

    return SimpleDHTErrSuccess;
}

int SimpleDHT22::read2(int pin, float* ptemperature, float* phumidity, byte pdata[40]) {
    int ret = SimpleDHTErrSuccess;

    byte data[40] = {0};
    if ((ret = sample(pin, data)) != SimpleDHTErrSuccess) {
        return ret;
    }

    short temperature = 0;
    short humidity = 0;
    if ((ret = parse(data, &temperature, &humidity)) != SimpleDHTErrSuccess) {
        return ret;
    }

    if (pdata) {
        memcpy(pdata, data, 40);
    }
    if (ptemperature) {
        *ptemperature = (float)temperature / 10.0;
    }
    if (phumidity) {
        *phumidity = (float)humidity / 10.0;
    }

    return ret;
}

int SimpleDHT22::sample(int pin, byte data[40]) {
    // empty output data.
    memset(data, 0, 40);

    // According to protocol: http://akizukidenshi.com/download/ds/aosong/AM2302.pdf
    // notify DHT11 to start: 
    //    1. T(be), PULL LOW 1ms(0.8-20ms).
    //    2. T(go), PULL HIGH 30us(20-200us), use 40us.
    //    3. SET TO INPUT.
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(1000);
    digitalWrite(pin, HIGH);
    pinMode(pin, INPUT);
    delayMicroseconds(40);

    // DHT11 starting:
    //    1. T(rel), PULL LOW 80us(75-85us), use 90us.
    //    2. T(reh), PULL HIGH 80us(75-85us), use 90us.
    if (confirm(pin, 90, LOW)) {
        return SimpleDHTErrStartLow;
    }
    if (confirm(pin, 90, HIGH)) {
        return SimpleDHTErrStartHigh;
    }

    // DHT11 data transmite:
    //    1. T(LOW), 1bit start, PULL LOW 50us(48-55us), use 60us.
    //    2. T(H0), PULL HIGH 26us(22-30us), bit(0)
    //    3. T(H1), PULL HIGH 70us(68-75us), bit(1)
    for (int j = 0; j < 40; j++) {
        if (confirm(pin, 60, LOW)) {
            return SimpleDHTErrDataLow;
        }

        // read a bit, should never call method,
        // for the method call use more than 20us,
        // so it maybe failed to detect the bit0.
        bool ok = false;
        int tick = 0;
        for (int i = 0; i < 8; i++, tick++) {
            if (digitalRead(pin) != HIGH) {
                ok = true;
                break;
            }
            delayMicroseconds(10);
        }
        if (!ok) {
            return SimpleDHTErrDataRead;
        }
        data[j] = (tick > 3? 1:0);
    }

    // DHT11 EOF:
    //    1. T(en), PULL LOW 50us(45-55us), use 60us.
    if (confirm(pin, 60, LOW)) {
        return SimpleDHTErrDataEOF;
    }

    return SimpleDHTErrSuccess;
}

