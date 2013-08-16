/*
  
    dcmot_i2c.ino
    
    commandline interface to the dcmot module


    dsscircuits.com/articles/arduino-i2c-master-library.html


*/



#include <I2C.h>

#define MON_BUF_LEN 128

char mon_buf[MON_BUF_LEN];
uint8_t mon_buf_cnt = 0;
char *mon_curr;

uint8_t adr = 64;
uint8_t cnt = 0;
uint8_t w = 0x07;


uint8_t dcmot_read(uint8_t idx)
{
    I2c.write(adr, idx);
    I2c.read(adr, (uint8_t)1);
    if ( I2c.available() > 0 )
      return I2c.receive();
    return 255;

/*    
    Wire.beginTransmission(adr);
    Wire.write(idx);
    Wire.endTransmission();
    Wire.requestFrom(adr, (uint8_t)1, (uint8_t)1);
    while(Wire.available()) 
    {
      return Wire.read();  
    }
    return 255;
*/
}

void dcmot_write(uint8_t idx, uint8_t val)
{
  I2c.write(adr, idx, val);
  /*
  Wire.beginTransmission(adr);
  Wire.write(idx);  
  Wire.write(val);  
  Wire.endTransmission();
  */
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
  Serial.println("a <dip>    set t2i address (DIP switch)");
  Serial.println("h          this help");
}

void line(const char *name, uint8_t idx)
{
    uint8_t err, val = 255;
    Serial.print(name);
    Serial.print(" idx=");
    Serial.print((int)idx);
    
    val = dcmot_read(idx);

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
  line("mosfet0", 8);
  line("mosfet1", 9);
  line("adc low pwm val", 10);
  line("measure cnt", 11);
  line("measure average (BEMF)", 12);
  line("dc motor present (1=yes)", 13);
  line("DIP ADC", 14);

  Serial.print("BEMF: ");
  I2c.read(adr, (uint8_t)16, (uint8_t)16);
  while(I2c.available()) 
  {
    val = I2c.receive();  
    Serial.print(" ");
    Serial.print((int)val);
  }
  Serial.println("");


  Serial.print("Rising Edge Abs: ");
  I2c.read(adr, (uint8_t)32, (uint8_t)8);
  while(I2c.available()) 
  {
    val = I2c.receive();  
    Serial.print(" ");
    Serial.print((int)val);
  }
  Serial.println("");

  Serial.print("Rising Edge Delta: ");
  I2c.read(adr, (uint8_t)40, (uint8_t)8);
  while(I2c.available()) 
  {
    val = I2c.receive();  
    Serial.print(" ");
    Serial.print((int)val);
  }
  Serial.println("");

/*  
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
  */
}

void mon_twi_adr(void)
{
  uint8_t a;
  a = mon_get_value();
  adr = a + 64;
  Serial.print("Using twi device ");
  Serial.print(adr, DEC);
  Serial.print(" (DIP=");
  Serial.print(a, DEC);
  Serial.println(")");
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
  dcmot_write(a,m);

  m = dcmot_read(a);

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
  else if ( mon_check('a') )
    mon_twi_adr();
  else
    ;
    
}



void setup() {
  // put your setup code here, to run once:
  // Wire.begin();        // join i2c bus (address optional for master)
  I2c.begin();
  I2c.timeOut(10);      // timeout to 10ms */
  Serial.begin(9600);  // start serial for output
  Serial.println("DCMOT CTL Monitor (enable LF/CR, type 'h' for help)");
  Serial.print("TWI address=");
  Serial.println(adr, DEC);
  //analogWrite(3, 127);
}

void exec(void)
{
  Serial.println(mon_buf);
  mon_curr = mon_buf;
  mon_cmd();
}

void loop()
{  
  dcmot_write(8, 1);
  
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


