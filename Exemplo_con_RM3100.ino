#include <Wire.h>
//#include <Adafruit_GFX.h> //Solo si usas graficos
#include <Adafruit_SSD1306.h>
#include <nmea.h>

#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (ou -1 si comparte o pin reset de Arduino)
#define SCREEN_ADDRESS 0x3C  ///< Direccions no datasheet; 0x3D para 128x64, 0x3C para 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define RM3100_direccion 0x20  // Direcc. escrava hexadec. para o RM3100 cos Pins configurables en BAIXO
#define PIN_DRDY 19            //STM32F103C8 -> Set pin PB1 para ser Data Ready Pin

//valores de registro interno sin bit R/W
#define RM3100_REVID_REG 0x36   // Direcc. hexadec. para Revid internal register
#define RM3100_POLL_REG 0x00    // Direcc. hexadec. para Sondeo internal register
#define RM3100_CMM_REG 0x01     // Direcc. hexadec. para CMM internal register
#define RM3100_STATUS_REG 0x34  // Direcc. hexadec. para Status internal register
#define RM3100_CCX1_REG 0x04    // Direcc. hexadec. para Cycle Count X1 internal register
#define RM3100_CCX0_REG 0x05    // Direcc. hexadec. para Cycle Count X0 internal register

//opcions
#define initialCC 200  //Establece o reconto de ciclos
#define singleMode 0   //0 = utilizar modo de medición continua; 1 = usar modo de medición única
#define useDRDYPin 1   //0 = non usar DRDYPin; 1 = usando DRDYPin para agardar datos


float declinacion_magnetica = -1.33;  //++IMPORTANTE++
float variacion_magnet = 0.0;         //++IMPORTANTE++
float gain;
float mx;
float my;
float mz;
float rumbo;
float rumbo_m;
uint8_t revid;
uint16_t cycleCount;
int32_t x = 0;
int32_t y = 0;
int32_t z = 0;

void setup() {
  pinMode(PIN_DRDY, INPUT);
  Wire.begin();          // Inicia a libreria Wire
  Serial.begin(9600);    //set baud rate a 9600 para depuracion
  Serial1.begin(38400);  //Para NMEA183 HS
  delay(100);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();             // Clear the buffer
  revid = readReg(RM3100_REVID_REG);  // 0x36
  Serial.print("REVID ID = 0x");      //REVID ID deberia ser 0x22
  Serial.println(revid, HEX);

  changeCycleCount(initialCC);  //Troca cycle count; por defecto = 200
  //menor reconto de ciclos = velocidades de datos mais altas pero menor resolución

  cycleCount = readReg(RM3100_CCX1_REG);                      //0x04
  cycleCount = (cycleCount << 8) | readReg(RM3100_CCX0_REG);  //0x05
  Serial.print("Reconto de ciclos = ");
  Serial.println(cycleCount);

  gain = (0.3671 * (float)cycleCount) + 1.5;  //ecuación lineal para calcular ganancia do reconto de ciclos

  Serial.print("Ganancia = ");  //visualizar a ganancia
  Serial.println(gain);         //a ganancia debe ser de arredor de 75 para reconto de ciclos predeterminado de 200

  if (singleMode) {  //configura modo de medición única
    writeReg(RM3100_CMM_REG, 0);
    writeReg(RM3100_POLL_REG, 0x70);
  } else {
    writeReg(RM3100_CMM_REG, 0x79);  //Habilita a transmisión para realizar medicions continuas cas funcions de alarma desactivadas
  }
}

void loop() {
  uint8_t x2, x1, x0, y2, y1, y0, z2, z1, z0;

  //espera ata que os datos estean listos usando calquera das opcions
  if (useDRDYPin) {
    while (digitalRead(PIN_DRDY) == LOW)
      ;  //check RDRY pin
  } else {
    while ((readReg(RM3100_STATUS_REG) & 0x80) != 0x80)
      ;  //lee o rexistro de estado interno 0x34
  }

  Wire.beginTransmission(RM3100_direccion);
  Wire.write(0x24);  //solicitude do primeiro rexistro da medición
  Wire.endTransmission();

  // Solicita 9 bytes dos rexistros de medición.
  Wire.requestFrom(RM3100_direccion, 9);
  if (Wire.available() == 9) {
    x2 = Wire.read();
    x1 = Wire.read();
    x0 = Wire.read();

    y2 = Wire.read();
    y1 = Wire.read();
    y0 = Wire.read();

    z2 = Wire.read();
    z1 = Wire.read();
    z0 = Wire.read();
  }

  x = ((uint32_t)x2 << 24) | ((uint32_t)x1 << 16) | ((uint32_t)x0 << 8);
  y = ((uint32_t)y2 << 24) | ((uint32_t)y1 << 16) | ((uint32_t)y0 << 8);
  z = ((uint32_t)z2 << 24) | ((uint32_t)z1 << 16) | ((uint32_t)z0 << 8);

  x >>= 8;
  y >>= 8;
  z >>= 8;

  imprime();  //comenta esta linea si non desexas imprimir
  compas_rumbo();
  OLEDdisplay();
  HDG();
  HDM();
  HDT();
  HSC();
}

//O checksum para as sentencias NMEA183
byte getChecksum(char* str) {
  byte cs = 0;
  for (unsigned int n = 1; n < strlen(str) - 1; n++) {
    cs ^= str[n];
  }
  return cs;
}

uint8_t readReg(uint8_t addr) {
  uint8_t data = 0;
  Wire.beginTransmission(RM3100_direccion);
  Wire.write(addr);
  Wire.endTransmission();
  delay(100);
  Wire.requestFrom(RM3100_direccion, 1);
  if (Wire.available() == 1) {
    data = Wire.read();
  }
  return data;
}

void writeReg(uint8_t addr, uint8_t data) {
  Wire.beginTransmission(RM3100_direccion);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

void changeCycleCount(uint16_t newCC) {
  uint8_t CCMSB = (newCC & 0xFF00) >> 8;
  uint8_t CCLSB = newCC & 0xFF;

  Wire.beginTransmission(RM3100_direccion);
  Wire.write(RM3100_CCX1_REG);
  Wire.write(CCMSB);  //escribir un novo reconto de ciclos en ccx1
  Wire.write(CCLSB);
  Wire.write(CCMSB);
  Wire.write(CCLSB);
  Wire.write(CCMSB);
  Wire.write(CCLSB);
  Wire.endTransmission();
}
void imprime() {
  Serial.print("Datos en recontos:");
  Serial.print("   X:");
  Serial.print(x);
  Serial.print("   Y:");
  Serial.print(y);
  Serial.print("   Z:");
  Serial.println(z);
  mx = (float)(x) / gain;
  my = (float)(y) / gain;
  mz = (float)(z) / gain;
  Serial.print("Datos en microTesla(uT): ");
  Serial.print(" X: ");
  Serial.print(mx);
  Serial.print("   Y: ");
  Serial.print(my);
  Serial.print("   Z: ");
  Serial.println(mz);

  double uT = sqrt(pow(((float)(mx) / gain), 2) + pow(((float)(my) / gain), 2) + pow(((float)(mz) / gain), 2));
  Serial.print("Intensidade do campo magnético(uT): ");
  Serial.println(uT);  //Ria de Arousa:Magnetic field strength: 45.657 uT
  Serial.println();
}
void compas_rumbo() {                                      //Calcular ángulo do eixe X respecto ao norte  //Declinación magnética Ria de Arousa: -1º 19´
  rumbo_m = atan2(my, mx);                                 //https://www.magnetic-declination.com/
  rumbo = rumbo_m + (declinacion_magnetica / (180 / PI));  // Rv = Rm + D;
  if (rumbo < 0) {
    rumbo += 2 * PI;
  }
  if (rumbo > 2 * PI) {
    rumbo -= 2 * PI;
  }
  rumbo = rumbo * (180 / PI);  //convertimos de Radians a grados
  Serial.print("Rumbo   ");
  Serial.println(rumbo);
}
void OLEDdisplay() {
  display.clearDisplay();
  display.setCursor(0, 0);  // Start at top-left corner
  display.setTextSize(1);   // Normal 1:1 pixel scale
  display.print(rumbo);
  display.display();
}

void HDG() {              //Fabricamos sentecia $XXVWR de NMEA183
  char SentenciaHDG[83];  //reservamos un mínimo de caracteres para esta cadea $XXVWR
  byte cs;                //para o checksum
  PString str(SentenciaHDG, sizeof(SentenciaHDG));
  str.print("$WIHDG,");
  str.print(rumbo_m, 1);
  str.print(",");
  float dec_mag_abs = abs(declinacion_magnetica);
  str.print(dec_mag_abs, 2);
  str.print(",W,");
  str.print(variacion_magnet);
  str.print(",W,*");
  cs = getChecksum(SentenciaHDG);  // calcula o checksum
  //caracteristica de arduino: imprime 0x02 como 2, 0x3E como 3E, necesita dun apaño
  if (cs < 0x10) str.print('0');  //  este e o apaño
  str.print(cs, HEX);             //  terminase de ensamblar toda a sentencia e enviase ao porto
  Serial.println(SentenciaHDG);   //
  Serial2.println(SentenciaHDG);  //
}

void HDM() {              //Fabricamos sentecia $XXVWR de NMEA183
  char SentenciaHDM[83];  //reservamos un mínimo de caracteres para esta cadea $XXVWR
  byte cs;                //para o checksum
  PString str(SentenciaHDM, sizeof(SentenciaHDM));
  str.print("$WIHDM,");
  str.print(rumbo_m, 1);
  str.print(",M,*");
  cs = getChecksum(SentenciaHDM);  // calcula o checksum
  //caracteristica de arduino: imprime 0x02 como 2, 0x3E como 3E, necesita dun apaño
  if (cs < 0x10) str.print('0');  //  este e o apaño
  str.print(cs, HEX);             //  terminase de ensamblar toda a sentencia e enviase ao porto
  Serial.println(SentenciaHDM);   //
  Serial2.println(SentenciaHDM);  //
}
void HDT() {              //Fabricamos sentecia $XXVWR de NMEA183
  char SentenciaHDT[83];  //reservamos un mínimo de caracteres para esta cadea $XXVWR
  byte cs;                //para o checksum
  PString str(SentenciaHDT, sizeof(SentenciaHDT));
  str.print("$WIHDT,");
  str.print(rumbo, 1);
  str.print(",T,*");
  cs = getChecksum(SentenciaHDT);  // calcula o checksum
  //caracteristica de arduino: imprime 0x02 como 2, 0x3E como 3E, necesita dun apaño
  if (cs < 0x10) str.print('0');  //  este e o apaño
  str.print(cs, HEX);             //  terminase de ensamblar toda a sentencia e enviase ao porto
  Serial.println(SentenciaHDT);   //
  Serial2.println(SentenciaHDT);  //
}
void HSC() {              //Fabricamos sentecia $XXVWR de NMEA183
  char SentenciaHSC[83];  //reservamos un mínimo de caracteres para esta cadea $XXVWR
  byte cs;                //para o checksum
  PString str(SentenciaHSC, sizeof(SentenciaHSC));
  str.print("$WIHSC,");
  str.print(rumbo, 1);
  str.print(",T,*");
  str.print(rumbo_m, 1);
  str.print(",M,*");
  cs = getChecksum(SentenciaHSC);  // calcula o checksum
  if (cs < 0x10) str.print('0');
  str.print(cs, HEX);             //  terminase de ensamblar toda a sentencia e enviase ao porto
  Serial.println(SentenciaHSC);   //
  Serial2.println(SentenciaHSC);  //
}
