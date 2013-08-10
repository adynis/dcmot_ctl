/*
  
    dcmot_serial.ino
    
    commandline interface to the dcmot module

*/



#include <Wire.h>

#define MON_BUF_LEN 128

char mon_buf[MON_BUF_LEN];
uint8_t mon_buf_cnt = 0;
char *mon_curr;

uint8_t adr = 15;
uint8_t cnt = 0;
uint8_t w = 0x07;


uint8_t dcmot_read(uint8_t idx)
{
    Wire.beginTransmission(adr);
    Wire.write(idx);
    Wire.endTransmission();
    Wire.requestFrom(adr, (uint8_t)1, (uint8_t)1);
    while(Wire.available()) 
    {
      return Wire.read();  
    }
    return 255;
}

void dcmot_write(uint8_t idx, uint8_t val)
{
  Wire.beginTransmission(adr);
  Wire.write(idx);  
  Wire.write(val);  
  Wire.endTransmission();
}


void mon_skip_space(void)
{
  for(;;)
  {
    if ( *mon_curr == '\0' )
      break;
    if ( *mon_curr > ' ')
      break;    
    mon_curr++;
  }
}

uint16_t mon_get_value(void)
{
  uint16_t v = 0;
  uint8_t c;
  for(;;)
  {
    if ( *mon_curr >= '0' && *mon_curr <= '9' )
    {  
      c = *mon_curr;
      c -= '0';
      v *= 10;
      v += c;
      mon_curr++;
    }
    else
      break;
  }
  mon_skip_space();
  return v;
}

uint8_t mon_check(char c)
{
  if ( *mon_curr != c ) 
    return 0;
  mon_curr++;
  mon_skip_space();
  return 1;
}

void mon_help(void)
{
  Serial.println("Available commands:");
  Serial.println("i          show attiny84 register set");
  Serial.println("r          revert direction (modify register 1)");
  Serial.println("m <a> <v>  write value <v> to location <a>");
  Serial.println("h          this help");
}

void line(const char *name, uint8_t idx)
{
    uint8_t err, val = 255;
    Serial.print(name);
    Serial.print(" idx=");
    Serial.print((int)idx);
    Wire.beginTransmission(adr);
    Wire.write(idx);
    err = Wire.endTransmission();
    Serial.print(" err=");
    Serial.print((int)err);
    Wire.requestFrom(adr, (uint8_t)1, (uint8_t)1);
    while(Wire.available()) 
    {
      val = Wire.read();  
    }

    Serial.print(" val=");
    Serial.print((int)val);
    Serial.println("");
}

void info(void)
{
  uint8_t val;
  line("speed", 0);
  line("reverse", 1);
  line("drive ticks", 2);
  line("measure ticks", 3);
  line("measure ctl", 4);
  line("measure 1.1V ref", 5);
  line("speed with rpm control", 6);
  line("max speed with rpm control", 7);
  line("adc low pwm val", 8);
  line("measure cnt", 9);
  line("measure average", 10);
  line("dc motor present (1=yes)", 11);
  //line("Back-EMF", 12);
  
  Serial.print("BEMF: ");
  Wire.beginTransmission(adr);
  Wire.write(16);
  Wire.endTransmission();
  Wire.requestFrom(adr, (uint8_t)16, (uint8_t)1);
  while(Wire.available()) 
  {
    val = Wire.read();  
    Serial.print(" ");
    Serial.print((int)val);
  }
  Serial.println("");

  Serial.print("High Values: ");
  Wire.beginTransmission(adr);
  Wire.write(32);
  Wire.endTransmission();
  Wire.requestFrom(adr, (uint8_t)8, (uint8_t)1);
  while(Wire.available()) 
  {
    val = Wire.read();  
    Serial.print(" ");
    Serial.print((int)val);
  }
  Serial.println("");
  
  Serial.print("High Delta: ");
  Wire.beginTransmission(adr);
  Wire.write(40);
  Wire.endTransmission();
  Wire.requestFrom(adr, (uint8_t)8, (uint8_t)1);
  while(Wire.available()) 
  {
    val = Wire.read();  
    Serial.print(" ");
    Serial.print((int)val);
  }
  Serial.println("");
  
}


void mem_memory(void)
{
  uint8_t a, m, err;
  a = mon_get_value();
  m = mon_get_value();
  Serial.print("Write ");
  Serial.print(m, DEC);
  Serial.print(" to index ");
  Serial.print(a, DEC);

  Wire.beginTransmission(adr);
  Wire.write(a);  
  Wire.write(m);  
  err = Wire.endTransmission();

  Serial.print(" err=");
  Serial.print((int)err);
  
  Wire.beginTransmission(adr);
  Wire.write(a);
  err = Wire.endTransmission();
  Wire.requestFrom(adr, (uint8_t)1, (uint8_t)1);
  while(Wire.available()) 
  {
    m = Wire.read();  
  }

  Serial.print(" m=");
  Serial.print((int)m);
  Serial.println("");
}

void mon_reverse(void)
{
  if ( dcmot_read(1) == 0 )
    dcmot_write(1,1);
  else
    dcmot_write(1,0);

}


void mon_cmd(void)
{
  mon_skip_space();
  if ( *mon_curr == '\0' )
    return;
  if ( mon_check('?') )
    mon_help();
  else if ( mon_check('h') )
    mon_help();
  else if ( mon_check('i') )
    info();
  else if ( mon_check('m') )
    mem_memory();
  else if ( mon_check('r') )
    mon_reverse();
  else
    ;
    
}



void setup() {
  // put your setup code here, to run once:
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  Serial.println("DCMOT CTL Monitor (enable LF/CR, type 'h' for help)");
  analogWrite(3, 127);
}

void xloop() {
  uint8_t err, val;
  if ( cnt < 10 )
  {
    if ( cnt == 3 )
      adr = 0;
    else
      adr = 15;
    
    // put your main code here, to run repeatedly: 
    Serial.print("Start cnt=");
    Serial.print((int)cnt);
    Serial.print(" adr=");
    Serial.print((int)adr);
    Serial.print(" w=");
    Serial.print((int)w);
    
    Wire.beginTransmission(adr);
    Wire.write(0x000);  
    Wire.write(w);  
    err = Wire.endTransmission();

    Serial.print(" err=");
    Serial.print((int)err);

    Wire.beginTransmission(adr);
    if ( cnt == 8 ) 
      Wire.write(16);
    else if ( cnt == 9 )
      Wire.write(17);
    else
      Wire.write(0x000);
    
    err = Wire.endTransmission();

    Serial.print(" err=");
    Serial.print((int)err);

    Wire.requestFrom(adr, (uint8_t)1, (uint8_t)1);
    while(Wire.available()) 
    {
      val = Wire.read();  
    }

    Serial.print("  Read Value: ");
    Serial.print((int)val);
    
    Serial.println("");
    delay(500);
    //adr++;
    cnt++;
    w *= 2;
  }
  else if ( cnt == 10 )
  {
    info();
    cnt++;
  }
}

void exec(void)
{
  Serial.println(mon_buf);
  mon_curr = mon_buf;
  mon_cmd();
}

void loop()
{  
  
  if ( Serial.available() )
  {
    char c;
    c = Serial.read();
    if ( mon_buf_cnt >= MON_BUF_LEN-1 || c == '\n' || c == '\r'  )
    {
      exec();
      mon_buf_cnt = 0;
      mon_buf[mon_buf_cnt] = '\0';
    }
    else
    {
      mon_buf[mon_buf_cnt] = c;
      mon_buf_cnt++;
      mon_buf[mon_buf_cnt] = '\0';
    }
  }
}


