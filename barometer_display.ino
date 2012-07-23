#include <Wire.h>

#define I2C_ADDRESS 0x77 //77?

const unsigned char oversampling_setting = 3; //oversamplig (передискретизация)
const unsigned char pressure_waittime[4] = { 5, 8, 14, 26 };

//взято с даташита BMP085 
int ac1;
int ac2; 
int ac3; 
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1; 
int b2;
int mb;
int mc;
int md;

#define ZTSEG8B4_DADDR 0x51
#define ZTSEG8B4_ADDR  0x37
#define SET_ADDR       0x61
#define WRITE_CODE     0xAA
#define WRITE_CMD      0x55
#include <Wire.h>
uint8_t Nopdelay;
uint8_t buff[4];
unsigned short testnum=0;
unsigned char codetable[] = 
{
         0x3F,0x06,0x5B,0x4F,0x66,0x6D,0x7D,0x07,0x7F,0x6F, 0x77, 0x7C,0x39,0x5E,0x79,0x71,0x00
};


void DisplayDec(uint8_t addr,unsigned short val)
{
  uint8_t i;
  uint8_t segnum[5];
  segnum[0] = 0xAA;
  segnum[1] = val%10;
  segnum[2] = (val%100)/10;
  segnum[3] = (val/100)%10;
  segnum[4] = val/1000;
  Wire.beginTransmission(addr); // transmit to device 0x51
  Wire.write(WRITE_CODE);
  for (i=1; i<5; i++)
  {
    Wire.write(codetable[segnum[i]]);         // sends one byte  
  }
  Wire.endTransmission();              // stop transmitting
}

void Display(uint8_t addr,uint8_t *buf)
{
   uint8_t i;
  Wire.beginTransmission(addr); // transmit to device 0x51
  Wire.write(WRITE_CODE);
  for (i=0; i<4; i++)
  {
    Wire.write(buf[i]);         // sends one byte  
  }
  Wire.endTransmission();              // stop transmitting
}

void SetLumin(uint8_t address, uint8_t OnDelay, uint8_t OffDelay)
{
    Wire.beginTransmission(address); // transmit to device 0x51
    Wire.write(0x55);
    Wire.write(OnDelay);
    Wire.write(~OnDelay); 
    Wire.write(OffDelay);
    Wire.write(~OffDelay);
    Wire.endTransmission();              // stop transmitting
}
void SetAddress(uint8_t val)
{
      Wire.beginTransmission(ZTSEG8B4_DADDR); // transmit to device 0x51
      Wire.write(SET_ADDR);
      Wire.write(val);
      Wire.write(~val);
      Wire.endTransmission();              // stop transmitting
}

void Test()
{
    uint8_t i;
    Nopdelay = 1000;
    for (i=0 ;i<8; i++)
    {
        buff[0]=buff[1]=buff[2]=buff[3] = (1<<i);
        Display(ZTSEG8B4_ADDR, buff);
        delay(Nopdelay);
    }
    Nopdelay = 100;
    for (i=3 ;i>0; i--)
    {
        buff[0]=buff[1]=buff[2]=buff[3] = 0;
        buff[i] = (1<<4);
        Display(ZTSEG8B4_ADDR, buff);
        delay(Nopdelay);
        buff[i] = (1<<5);
        Display(ZTSEG8B4_ADDR, buff);
        delay(Nopdelay);
        buff[i] = (1<<0);
        Display(ZTSEG8B4_ADDR, buff);
        delay(Nopdelay);
        buff[i] = (1<<1);
        Display(ZTSEG8B4_ADDR, buff);
        delay(Nopdelay);
        buff[i] = (1<<2);
        Display(ZTSEG8B4_ADDR, buff);
        delay(Nopdelay);
    }
     buff[0]=buff[1]=buff[2]=buff[3] = 0xFF;
     Display(ZTSEG8B4_ADDR, buff);
     delay(1000);
}

void setup()
{  
  delay(1000);
  Serial.begin(9600);  // start serial for output
    pinMode(13, OUTPUT); 
  pinMode(13, INPUT); 
  
     SetAddress(ZTSEG8B4_ADDR);
  Wire.requestFrom(ZTSEG8B4_ADDR, 15);    // request 6 bytes from slave device #2
  while(Wire.available())    // slave may send less than requested
  { 
    char c = Wire.read(); // receive a byte as character
    Serial.print(c);         // print the character
  }
  Serial.print('\n'); 
  Test();
  Nopdelay = 2000;
  
  Serial.println("Setting up BMP085");
  Wire.begin();
  bmp085_get_cal_data();
}
void bmp085_read_temperature_and_pressure(int& temperature, long& pressure);

unsigned char ttp=0;;
bool isPress = true;

void loop()
{
  int  temperature = 0;
  long pressure = 0;
  bmp085_read_temperature_and_pressure(&temperature,&pressure);
  Serial.print(temperature,DEC);
  Serial.print(" ");
  Serial.print(pressure/133.3,DEC);
  if(isPress)
  {
    DisplayDec(ZTSEG8B4_ADDR,pressure/133.3);
  }
  else
  {
    DisplayDec(ZTSEG8B4_ADDR,temperature*0.1);
  }
  isPress = !isPress;
  Serial.println();
  delay(2000);
}

void bmp085_read_temperature_and_pressure(int* temperature, long* pressure) {
   //int ut = bmp085_read_ut();
   long ut = bmp085_read_ut();
   long up = bmp085_read_up();
   long x1, x2, x3, b3, b5, b6, p;
   unsigned long b4, b7;

   //расчет температуры 
   x1 = ((long) ut - ac6) * ac5 >> 15;
   x2 = ((long) mc << 11) / (x1 + md);
   b5 = x1 + x2;
   *temperature = (b5 + 8) >> 4;
     
   //расчет давления
   b6 = b5 - 4000;
   x1 = (b2 * (b6 * b6 >> 12)) >> 11;
   x2 = ac2 * b6 >> 11;
   x3 = x1 + x2;
   //b3 = (((int32_t) ac1 * 4 + x3)<> 2;
   
   if (oversampling_setting == 3) b3 = ((int32_t) ac1 * 4 + x3 + 2) << 1;
   if (oversampling_setting == 2) b3 = ((int32_t) ac1 * 4 + x3 + 2);
   if (oversampling_setting == 1) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 1;
   if (oversampling_setting == 0) b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;

   x1 = ac3 * b6 >> 13;
   x2 = (b1 * (b6 * b6 >> 12)) >> 16;
   x3 = ((x1 + x2) + 2) >> 2;
   b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
   b7 = ((uint32_t) up - b3) * (50000 >> oversampling_setting);
   p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

   x1 = (p >> 8) * (p >> 8);
   x1 = (x1 * 3038) >> 16;
   x2 = (-7357 * p) >> 16;
   *pressure = p + ((x1 + x2 + 3791) >> 4);
}


unsigned int bmp085_read_ut() {
  write_register(0xf4,0x2e);
  delay(5); //дольше чем 4.5 мс 
  return read_int_register(0xf6);
}

void  bmp085_get_cal_data() {
  Serial.println("Reading Calibration Data");
  ac1 = read_int_register(0xAA);
  Serial.print("AC1: ");
  Serial.println(ac1,DEC);
  ac2 = read_int_register(0xAC);
  Serial.print("AC2: ");
  Serial.println(ac2,DEC);
  ac3 = read_int_register(0xAE);
  Serial.print("AC3: ");
  Serial.println(ac3,DEC);
  ac4 = read_int_register(0xB0);
  Serial.print("AC4: ");
  Serial.println(ac4,DEC);
  ac5 = read_int_register(0xB2);
  Serial.print("AC5: ");
  Serial.println(ac5,DEC);
  ac6 = read_int_register(0xB4);
  Serial.print("AC6: ");
  Serial.println(ac6,DEC);
  b1 = read_int_register(0xB6);
  Serial.print("B1: ");
  Serial.println(b1,DEC);
  b2 = read_int_register(0xB8);
  Serial.print("B2: ");
  Serial.println(b1,DEC);
  mb = read_int_register(0xBA);
  Serial.print("MB: ");
  Serial.println(mb,DEC);
  mc = read_int_register(0xBC);
  Serial.print("MC: ");
  Serial.println(mc,DEC);
  md = read_int_register(0xBE);
  Serial.print("MD: ");
  Serial.println(md,DEC);
}


long bmp085_read_up() {
  write_register(0xf4,0x34+(oversampling_setting<<6));
  delay(pressure_waittime[oversampling_setting]);
  
  unsigned char msb, lsb, xlsb;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(0xf6);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 3); // read a byte
  while(!Wire.available()) {
    // ожидание 
  }
  msb = Wire.read();
  while(!Wire.available()) {
    // ожидание
  }
  lsb |= Wire.read();
  while(!Wire.available()) {
    // ожидание
  }
  xlsb |= Wire.read();
  return (((long)msb<<16) | ((long)lsb<<8) | ((long)xlsb)) >>(8-oversampling_setting);
}

void write_register(unsigned char r, unsigned char v)
{
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r);
  Wire.write(v);
  Wire.endTransmission();
}

char read_register(unsigned char r)
{
  unsigned char v;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 1); // read a byte
  while(!Wire.available()) {
    // ожидание
  }
  v = Wire.read();
  return v;
}

int read_int_register(unsigned char r)
{
  unsigned char msb, lsb;
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(r);  // register to read
  Wire.endTransmission();

  Wire.requestFrom(I2C_ADDRESS, 2); // read a byte
  while(!Wire.available()) {
    // ожидание
  }
  msb = Wire.read();
  while(!Wire.available()) {
    // ожидание
  }
  lsb = Wire.read();
  return (((int)msb<<8) | ((int)lsb));
}



