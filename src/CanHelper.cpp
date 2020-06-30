/*
  CanHelper.cpp - Helper library to construct and parse can messages from a dbc like format
  ATTENTION: This class is not optimized and uses per-bit operations. Each set/parsed bit takes ~7.5 microseconds!


  The MIT License (MIT)

  Copyright (c) 2020 Florian Brede

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include "CanHelper.h"
#include "globals.h"


uint32_t CanHelper::parseParameterBigEndianUnsignedLong(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t returnValue=0;

    if (size>32) {
        Serial.print("CANHelper Error: size is "); Serial.print(size); Serial.println(" but maximum supported size is 32 bit!");
        return 0;
    }
    unsigned char i;

    int16_t currentByte = mostSignifcantBit/8;
    int16_t currentBit = mostSignifcantBit%8-(size-1);
    currentByte+=abs(currentBit)/8;
    currentBit=abs(currentBit)%8;

    unsigned char bit;
    for (i = 0; i<size; i++) {
        bit = buffer[currentByte] >> (currentBit) & 0b00000001;
        returnValue |= bit << i;
        currentBit++;
        if (currentBit>7) {
            currentBit=0;
            currentByte--;
        }
    }
    return returnValue;
}

int32_t CanHelper::parseParameterBigEndianLong(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t rawValue = parseParameterBigEndianUnsignedLong(buffer, 0, 1, mostSignifcantBit, size);
    int32_t value = rawValue*paramScale+paramOffset;
    return value;
}
float CanHelper::parseParameterBigEndianFloat(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t rawValue = parseParameterBigEndianUnsignedLong(buffer, 0, 1, mostSignifcantBit, size);
    float value = rawValue*paramScale+paramOffset;
    return value;
}
uint8_t CanHelper::parseParameterBigEndianByte(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t rawValue = parseParameterBigEndianUnsignedLong(buffer, 0, 1, mostSignifcantBit, size);
    uint8_t value = rawValue*paramScale+paramOffset;
    return value;
}
int16_t CanHelper::parseParameterBigEndianInt(uint8_t *buffer, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t rawValue = parseParameterBigEndianUnsignedLong(buffer, 0, 1, mostSignifcantBit, size);
    int16_t value = rawValue*paramScale+paramOffset;
    return value;
}

void CanHelper::putParameterBigEndianUnsignedLong(uint8_t *buffer, uint32_t value, uint8_t mostSignifcantBit, uint8_t size) {
    if (size>32) {
        Serial.print("CANHelper Error: size is "); Serial.print(size); Serial.println(" but maximum supported size is 32 bit!");
        return;
    }
    unsigned char i;

    int16_t currentByte = mostSignifcantBit/8;
    int16_t currentBit = mostSignifcantBit%8-(size-1);
    currentByte+=abs(currentBit)/8;
    currentBit=abs(currentBit)%8;

    unsigned char bit;
    for (i = 0; i<size; i++) {
        bit = value >> (i) & 0b00000001;
        buffer[currentByte] |= bit << currentBit;
        currentBit++;
        if (currentBit>7) {
            currentBit=0;
            currentByte--;
        }
    }
}

void CanHelper::putParameterBigEndian(uint8_t *buffer, uint8_t value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterBigEndianUnsignedLong(buffer, adjustedValue, mostSignifcantBit, size);
}
void CanHelper::putParameterBigEndian(uint8_t *buffer, int16_t value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterBigEndianUnsignedLong(buffer, adjustedValue, mostSignifcantBit, size);
}
void CanHelper::putParameterBigEndian(uint8_t *buffer, int32_t value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterBigEndianUnsignedLong(buffer, adjustedValue, mostSignifcantBit, size);
}
void CanHelper::putParameterBigEndian(uint8_t *buffer, uint32_t value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterBigEndianUnsignedLong(buffer, adjustedValue, mostSignifcantBit, size);
}
void CanHelper::putParameterBigEndian(uint8_t *buffer, float value, float paramOffset, float paramScale, uint8_t mostSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterBigEndianUnsignedLong(buffer, adjustedValue, mostSignifcantBit, size);
}








uint32_t CanHelper::parseParameterLittleEndianUnsignedLong(uint8_t *buffer, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t returnValue=0;

    if (size>32) {
        Serial.print("CANHelper Error: size is "); Serial.print(size); Serial.println(" but maximum supported size is 32 bit!");
        return 0;
    }
    unsigned char i;

    leastSignifcantBit = leastSignifcantBit+size-1;

    int16_t currentByte = leastSignifcantBit/8-(size-1)/8;
    int16_t currentBit = leastSignifcantBit%8-(size-1);
    currentBit=abs(currentBit)%8;

    unsigned char bit;
    for (i = 0; i<size; i++) {
        bit = buffer[currentByte] >> (currentBit) & 0b00000001;
        returnValue |= bit << i;
        currentBit++;
        if (currentBit>7) {
            currentBit=0;
            currentByte++;
        }
    }
    return returnValue;
}

int32_t CanHelper::parseParameterLittleEndianLong(uint8_t *buffer, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t rawValue = parseParameterLittleEndianUnsignedLong(buffer, 0, 1, leastSignifcantBit, size);
    int32_t value = rawValue*paramScale+paramOffset;
    return value;
}
float CanHelper::parseParameterLittleEndianFloat(uint8_t *buffer, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t rawValue = parseParameterLittleEndianUnsignedLong(buffer, 0, 1, leastSignifcantBit, size);
    float value = rawValue*paramScale+paramOffset;
    return value;
}
uint8_t CanHelper::parseParameterLittleEndianByte(uint8_t *buffer, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t rawValue = parseParameterLittleEndianUnsignedLong(buffer, 0, 1, leastSignifcantBit, size);
    uint8_t value = rawValue*paramScale+paramOffset;
    return value;
}
int16_t CanHelper::parseParameterLittleEndianInt(uint8_t *buffer, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t rawValue = parseParameterLittleEndianUnsignedLong(buffer, 0, 1, leastSignifcantBit, size);
    int16_t value = rawValue*paramScale+paramOffset;
    return value;
}


void CanHelper::putParameterLittleEndianUnsignedLong(uint8_t *buffer, uint32_t value, uint8_t leastSignifcantBit, uint8_t size) {
    if (size>32) {
        Serial.print("CANHelper Error: size is "); Serial.print(size); Serial.println(" but maximum supported size is 32 bit!");
        return;
    }
    unsigned char i;

    leastSignifcantBit = leastSignifcantBit+size-1;

    int16_t currentByte = leastSignifcantBit/8-(size-1)/8;
    int16_t currentBit = leastSignifcantBit%8-(size-1);
    currentBit=abs(currentBit)%8;

    unsigned char bit;
    for (i = 0; i<size; i++) {
        bit = value >> (i) & 0b00000001;
        buffer[currentByte] |= bit << currentBit;
        currentBit++;
        if (currentBit>7) {
            currentBit=0;
            currentByte++;
        }
    }
}

void CanHelper::putParameterLittleEndian(uint8_t *buffer, uint8_t value, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterLittleEndianUnsignedLong(buffer, adjustedValue, leastSignifcantBit, size);
}
void CanHelper::putParameterLittleEndian(uint8_t *buffer, int16_t value, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterLittleEndianUnsignedLong(buffer, adjustedValue, leastSignifcantBit, size);
}
void CanHelper::putParameterLittleEndian(uint8_t *buffer, int32_t value, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterLittleEndianUnsignedLong(buffer, adjustedValue, leastSignifcantBit, size);
}
void CanHelper::putParameterLittleEndian(uint8_t *buffer, uint32_t value, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterLittleEndianUnsignedLong(buffer, adjustedValue, leastSignifcantBit, size);
}
void CanHelper::putParameterLittleEndian(uint8_t *buffer, float value, float paramOffset, float paramScale, uint8_t leastSignifcantBit, uint8_t size) {
    uint32_t adjustedValue = (value-paramOffset)/paramScale;
    return putParameterLittleEndianUnsignedLong(buffer, adjustedValue, leastSignifcantBit, size);
}


void CanHelper::printBitmask(uint8_t *buffer) {
    int16_t i, by, bi;
    Serial.println("printBitmask:");
    for (by=7; by>=0; by--) {
        Serial.println("");
        for (bi = 0; bi<8; bi++) {
            i = by*8+bi;
            uint8_t bit = (buffer[7 - ((i+0) / 8)] >> (7 - ((i+0)%8))) & 0b00000001;
            Serial.print(bit); Serial.print(" ");
        }
    }
    Serial.println("");
}


void CanHelper::resetBuffer(uint8_t *buffer) {
    buffer[0]=buffer[1]=buffer[2]=buffer[3]=buffer[4]=buffer[5]=buffer[6]=buffer[7]=0;
}


void CanHelper::bitRepresentation(uint32_t value, int8_t length) {
    for(byte mask = 1<<(length-1); mask; mask >>= 1){
        if(mask  & value)
            Serial.print('1');
        else
            Serial.print('0');
    }
    Serial.println("");
}