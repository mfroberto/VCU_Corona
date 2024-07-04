/*ErrorLCD y despues Reset*/
#include <Arduino.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "stdio.h"
#include "stdlib.h"
#include "driver/twai.h"
#include <WString.h>
#include <String.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
byte Cell_Voltage_Maxinc;
byte charge_connecte;
byte session;
byte Charge_Complete;
byte Charge_Enable;
byte System_State = 0;
unsigned int incSystem_State = 0;
twai_message_t EV_control_connected = {.extd = 0, .identifier = 0x500, .data_length_code = 8, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}}; // 00 00 00 f5 0a 00 00 min 20 =1000
twai_message_t EV_control_ready = {.extd = 0, .identifier = 0x500, .data_length_code = 8, .data = {0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};     // 00 00 00 f5 0a 00 00 min 20 =1000
twai_message_t EV_control_fault = {.extd = 0, .identifier = 0x500, .data_length_code = 8, .data = {0x02, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};     // 00 00 00 f5 0a 00 00 min 20 =1000
twai_message_t EV_control_100 = {.extd = 0, .identifier = 0x500, .data_length_code = 8, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};       // 00 00 00 f5 0a 00 00 min 20 =1000
twai_message_t VCU_507 = {.extd = 0, .identifier = 0x507, .data_length_code = 8, .data = {0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};              // 00 00 00 f5 0a 00 00 min 20 =1000
twai_message_t VCU_507_7 = {.extd = 0, .identifier = 0x507, .data_length_code = 8, .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};            // 00 00 00 f5 0a 00 00 min 20 =1000
twai_message_t VCU_508_SOC100 = {.extd = 0, .identifier = 0x508, .data_length_code = 8, .data = {0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};       // 00 00 00 f5 0a 00 00 min 20 =1000
twai_message_t ledClusterON = {.extd = 1, .identifier = 0x18FEF100, .data_length_code = 8, .data = {0x03, 0x00, 0x00, 0x45, 0x00, 0xFE, 0x1C, 0xCF}};
twai_message_t rx_frame;
twai_message_t RPM = {.extd = 1, .identifier = 0xCF00400, .data_length_code = 8, .data = {0x00, 0x00, 0x00, 0xf5, 0x0b, 0x00, 0x00, 0x00}};    // 00 00 00 f5 0a 00 00 min 20 =1000 0xf5, 0x0b,inicio
twai_message_t HVES1D1 = {.extd = 1, .identifier = 0xf09001, .data_length_code = 8, .data = {0x80, 0x0c, 0x20, 0x03, 0x00, 0x00, 0x00, 0x00}}; // 00 00 00 f5 0a 00 00 min 20 =1000 0xf5, 0x0b,inicio
twai_message_t HSS1 = {.extd = 1, .identifier = 0x0FCC201, .data_length_code = 8, .data = {0x00, 0x00, 0x00, 0xf5, 0x0b, 0x00, 0x00, 0x00}};   // 00 00 00 f5 0a 00 00 min 20 =1000 0xf5, 0x0b,inicio
int cmp_asc(const void *c1, const void *c2)
{
  return *((int *)c1) - *((int *)c2);
}
uint64_t event = 0; //[5] = {0, 0, 0, 0, 0};
union wordchar64
{
  uint8_t x[8]; // Cambie char a byte
  uint64_t event;
};
union wordchar64 q1;
union wordchar64 *q2 = &q1;
byte rpmfiltro = 0;
byte rpmsInc = 0;
unsigned int rpms[5] = {0, 0, 0, 0, 0};
union wordchar
{
  byte x[2]; // Cambie char a byte
  word rpms;
};
union wordchar p1;
union wordchar *p2 = &p1;
byte SOC = 102;
byte arrSOC[101];
byte SOCinc;
unsigned int incTiempoCluster = 0;
unsigned int errorCont, resetCont = 0;
byte trpmzero = 0;
bool fRPM = false;
unsigned long tRPM;
bool fRPMmax = false;
unsigned long tRPMmax;
unsigned long tbatbaja;
void setup_twai_driver()
{
  twai_general_config_t g_config = {
      .mode = TWAI_MODE_NORMAL,
      .tx_io = GPIO_NUM_5, // 5
      .rx_io = GPIO_NUM_4, // 4
      .clkout_io = TWAI_IO_UNUSED,
      .bus_off_io = TWAI_IO_UNUSED,
      .tx_queue_len = 5,
      .rx_queue_len = 5,
      .alerts_enabled = TWAI_ALERT_NONE,
      .clkout_divider = 0};
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    printf("Driver installed\n");
  }
  else
  {
    printf("Failed to install driver\n");
    return;
  }
  // Start TWAI driver
  if (twai_start() == ESP_OK)
  {
    printf("Driver started\n");
  }
  else
  {
    printf("Failed to start driver\n");
    return;
  }
}
void reset_twai_driver()
{
  esp_err_t error;
  error = twai_driver_uninstall();
  setup_twai_driver();
}
LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
void LCD_inicio()
{
  lcd.setCursor(0, 0);
  lcd.print("Carga Bat.     %");
  lcd.setCursor(0, 1);
  lcd.print("RangoAprox    Km");
}
void setup_LCD_driver()
{
  lcd.init(); // initialize the lcd
  lcd.init();
  lcd.backlight();
  LCD_inicio();
}
void LCD_conectado()
{
  lcd.setCursor(0, 0);
  lcd.print("    INICIANDO   ");
  lcd.setCursor(0, 1);
  lcd.print("    CARGADOR    ");
}
void LCD_cargando()
{
  lcd.setCursor(0, 0);
  lcd.print("    CARGANDO... ");
}
void LCD_desconecte()
{
  lcd.setCursor(0, 0);
  lcd.print("   DESCONECTE   ");
  lcd.setCursor(0, 1);
  lcd.print("    CARGADOR    ");
}
void LCD_error()
{
  lcd.setCursor(0, 0);
  lcd.print("ERROR DESCONECTE");
  lcd.setCursor(0, 1);
  lcd.print(q2->event);
}
void RX_CAN()
{
  if (twai_receive(&rx_frame, pdMS_TO_TICKS(100)) == ESP_OK)
  {
    resetCont = 0;

    if (rx_frame.identifier == 0x503)
    {
      if (rx_frame.data[1] != System_State)
      {
        incSystem_State++;
        if (incSystem_State > 3)
          System_State = rx_frame.data[1];
      }
      else
      {
        incSystem_State = 0;
      }
      if (session == 5)
      {
        Cell_Voltage_Maxinc++;
        if (Cell_Voltage_Maxinc == 51)
        {
          Cell_Voltage_Maxinc = 0;
          if (SOC > 98)
            twai_transmit(&VCU_508_SOC100, pdMS_TO_TICKS(100));
        }
      }
    }
    if (rx_frame.identifier == 0x504)
    {
      if (rx_frame.data[1] < 101)
      {
        arrSOC[SOCinc] = rx_frame.data[1];
        SOCinc++;

        if (SOCinc == 51)
        {
          SOCinc = 0;
          qsort(arrSOC, 51, sizeof(byte), cmp_asc);
          SOC = arrSOC[25];
        }
        Charge_Complete = rx_frame.data[2] & 0b00100000;
        Charge_Enable = rx_frame.data[4];
      }
    }
    if (rx_frame.identifier == 0x540)
    {
      charge_connecte = rx_frame.data[1] & 0b00000010;
      session = rx_frame.data[0] & 0b00001111;
    }
    if (charge_connecte != 2)
    {
      if (rx_frame.identifier == 0x267)
      {
        p1.x[0] = rx_frame.data[4];
        p1.x[1] = rx_frame.data[5];
        word x = p2->rpms;
        if (x > 0 && x < 3000)
        {
          rpms[1] = x;
          rpms[1] = rpms[1] << 3;
          p2->rpms = rpms[1];
          RPM.data[4] = p1.x[1];
          RPM.data[3] = p1.x[0];
          if (RPM.data[4] > 0x38 && RPM.data[4] < 0x50) // Filto rpm
          {
            twai_transmit(&ledClusterON, pdMS_TO_TICKS(100));
            // fRPMmax = true;
            RPM.data[4] = 0x38;
          }
          else
          {
            trpmzero = 0;
          }
          if (RPM.data[4] < 0x0b)
            RPM.data[4] = 0x0b;
        }
      }
      if (rx_frame.identifier == 0x501)
      {
        VCU_507.data[7] = rx_frame.data[0];
        VCU_507.data[6] = rx_frame.data[1];
        p1.x[0] = rx_frame.data[0];
        p1.x[1] = rx_frame.data[1];
        word HVES1_Voltage_Level = p2->rpms << 1;
        p2->rpms = HVES1_Voltage_Level;
        HVES1D1.data[4] = p1.x[0];
        HVES1D1.data[5] = p1.x[1];
      }
    }
    else if (rx_frame.identifier == 0x549) // Matriz
    {
      if (charge_connecte == 2)
      {
        q1.x[0] = rx_frame.data[0];
        q1.x[1] = rx_frame.data[1];
        q1.x[2] = rx_frame.data[2];
        q1.x[3] = rx_frame.data[3];
        q1.x[4] = rx_frame.data[4];
        q1.x[5] = rx_frame.data[5] & 0xFB;
        q1.x[6] = 0;
        q1.x[7] = 0;
        if (q2->event != 0)
        {
          errorCont++;
          LCD_error();
          if (errorCont > 30)
            ESP.restart();
        }
        else
          errorCont = 0;
      }
    }
  }
  else
  {
    resetCont++;
    if (resetCont > 250)
    {
      resetCont = 0;
      reset_twai_driver();
      LCD_inicio();
    }
  }
}
char antSOC = 102;
unsigned short tbrillo;
byte tsonido = 0;
bool fsonido = false;
bool fbatbaja = false;
void LCD_SOC()
{
  if (fbatbaja == true)
  {
    lcd.setCursor(0, 1);
    if (tbatbaja + 6000 < millis())
    {
      tbatbaja = millis();
      lcd.print("  BATERIA BAJA! ");
    }
    else if (millis() < tbatbaja + 3000)
    {
      lcd.print("  BATERIA BAJA! ");
    }
    else
    {
      lcd.print("RangoAprox ");
      long rango = ((SOC - 10) * 1.333); // 135km
      if (rango < 10)
        lcd.print(" ");
      if (rango < 100)
        lcd.print(" ");
      lcd.print(rango);
      lcd.print("km");
    }
  }
  if (antSOC != SOC || SOC <= 20)
    if (SOC > 20 && SOC < 101)
    {
      tbrillo = 0;
      lcd.backlight();
      lcd.setCursor(0, 0);
      lcd.print("Carga Bat.  ");
      lcd.setCursor(12, 0);
      if (SOC < 100)
        lcd.print(" ");
      lcd.print(SOC);
      lcd.print("%            ");
      lcd.setCursor(0, 1);
      if (SOC > 30)
      {
        fbatbaja = 0;
        digitalWrite(13, LOW);
        lcd.print("RangoAprox ");
        long rango = ((SOC - 10) * 1.333); // 135km
        if (rango < 10)
          lcd.print(" ");
        if (rango < 100)
          lcd.print(" ");
        lcd.print(rango);
        lcd.print("km");
      }
      else
      {
        fbatbaja = 0;
        digitalWrite(13, LOW);
        lcd.print("  BATERIA BAJA! ");
        fRPM = true;
        fbatbaja = true;
      }
    }
    else
    {
      if (SOC < 21)
      {
        fbatbaja = 0;
        tbrillo++;
        if (tbrillo < 2)
        {
          lcd.setCursor(0, 0);
          lcd.print("RESERVA  BATERIA");
          lcd.setCursor(0, 1);
          lcd.print("  !!!CARGAR!!!  ");
        }
        else
        {
          lcd.noBacklight(); // lcd.noBacklight();
          digitalWrite(13, LOW);
        }
        if (tbrillo > 2)
        {
          tbrillo = 0;
          lcd.backlight();
          digitalWrite(13, HIGH);
        }
      }
    }
  antSOC = SOC;
}
String Id_canbus(twai_message_t rx_frame)
{
  String x = "";
  char buffer[100];
  sprintf(buffer, "%X,%d", rx_frame.identifier, rx_frame.data_length_code);
  // printf(buffer);
  x += buffer;
  for (int i = 0; i < rx_frame.data_length_code; i++)
  {
    // printf(" 0x%02X ", rx_frame.data[i]);
    sprintf(buffer, ",%02X ", rx_frame.data[i]);
    // printf(buffer);
    x += buffer;
  }
  return x + ",";
}
void charger(void)
{
reiniciar:
  antSOC = 102;
  Serial.println(session);
  Serial.println("charge_connecte: " + String(charge_connecte));
  if (session < 3)
  {
    LCD_conectado();
    if (twai_transmit(&EV_control_connected, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
      EV_control_connected.data[6] = EV_control_connected.data[6] + 1;
      Serial.println("C_500->" + String(EV_control_connected.data[6]));
    }
    else
    {
      session = 7;
      goto reiniciar;
    }
    if (twai_transmit(&VCU_507, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
      Serial.println("C_507->" + String(VCU_507.data[6]));
    }
    else
    {
      session = 7;
      goto reiniciar;
    }
  }
  if (session > 2 && session < 6)
  {
    byte LCD_cargado = 0;
    digitalWrite(26, HIGH); // SEVCOM NO PERMITE ACELERAR releSONMOT
    for (;;)
    {
      RX_CAN();
      if (charge_connecte != 2)
      {
        session = 7;
        goto reiniciar;
      }
      if (LCD_cargado == 0 && session == 5)
      {
        LCD_cargado = 1;
        LCD_cargando();
      }
      Serial.print("3_5_Cargando Sesion ->");
      Serial.println(session);
      Serial.print("SOC ->");
      Serial.println(SOC);
      delay(10);
      if (antSOC != SOC)
      {
        lcd.setCursor(0, 1);
        lcd.print("Carga Bat.  ");
        lcd.setCursor(12, 1);
        if (SOC < 100)
          lcd.print(" ");
        lcd.print(SOC);
        lcd.print("%            ");
        antSOC = SOC;
      }
      if (twai_transmit(&EV_control_ready, pdMS_TO_TICKS(1000)) == ESP_OK)
      {
        EV_control_ready.data[6] = EV_control_ready.data[6] + 1;
        Serial.println("C_500->" + String(EV_control_ready.data[6]));
      }
      else
      {
        session = 7;
        goto reiniciar;
      }
      if (twai_transmit(&VCU_507, pdMS_TO_TICKS(1000)) == ESP_OK)
      {
        Serial.println("C_507->" + String(VCU_507.data[6]));
      }
      else
      {
        session = 7;
        goto reiniciar;
      }
      if (session == 7)
      {
        LCD_desconecte();
        for (;;)
        {
          RX_CAN();
          if (twai_transmit(&EV_control_connected, pdMS_TO_TICKS(1000)) == ESP_OK)
          {
            EV_control_100.data[6] = EV_control_100.data[6] + 1;
            Serial.println("7_500->" + String(EV_control_ready.data[6]));
          }
          if (twai_transmit(&VCU_507, pdMS_TO_TICKS(1000)) == ESP_OK)
          {
            Serial.println("7_507->" + String(VCU_507.data[6]));
          }
          if (charge_connecte != 2)
          {
            if (twai_transmit(&EV_control_connected, pdMS_TO_TICKS(1000)) == ESP_OK)
            {
              EV_control_100.data[6] = EV_control_100.data[6] + 1;
              Serial.println("7_500->" + String(EV_control_ready.data[6]));
            }
            if (twai_transmit(&VCU_507_7, pdMS_TO_TICKS(1000)) == ESP_OK)
            {
              Serial.println("7_507->" + String(VCU_507.data[6]));
            }
            Serial.println("Desconecte -> 7 Reset");
            delay(1000);
            if (twai_transmit(&EV_control_connected, pdMS_TO_TICKS(1000)) == ESP_OK)
            {
              EV_control_100.data[6] = EV_control_100.data[6] + 1;
              Serial.println("7_500->" + String(EV_control_ready.data[6]));
            }
            if (twai_transmit(&VCU_507_7, pdMS_TO_TICKS(1000)) == ESP_OK)
            {
              Serial.println("7_507->" + String(VCU_507.data[6]));
            }
            delay(1000);
            antSOC = 120;
            LCD_SOC();
            goto reiniciar;
          }
        }
      }
    }
  }
}
unsigned long TiempoCluster, TiempoLCD, TiempoSonido = 0;
void setup()
{
  pinMode(26, OUTPUT);   // SEVCOM releSONMOT
  digitalWrite(26, LOW); // PERMITE ACELERAR(LOW)//NO PERMITE ACELERAR releSONMOT(HIGH)
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW); // BOCINA
  pinMode(14, OUTPUT);   // ARDUINO
  digitalWrite(14, LOW); // ARDUINO
  Serial.begin(115200);
  Serial.println("Ver:10_2 Oscilacion SOC cuando carga y SOC 99 solo con SOC 99");
  setup_LCD_driver();
  setup_twai_driver();
}
void loop()
{
  RX_CAN();
  if (charge_connecte == 2 || session == 5)
  {
    tbrillo = 0;
    lcd.backlight();
    digitalWrite(13, LOW);
    digitalWrite(14, LOW);  // Arduino OFF
    digitalWrite(26, HIGH); // SEVCOM NO PERMITE ACELERAR releSONMOT
    charger();
    if (session > 5)
    {
      Serial.println("SESION->6 o 7");
      reset_twai_driver();
      LCD_inicio();
    }
  }
  else
  {
    if (System_State == 1)
    {
      digitalWrite(14, HIGH); // j8c
      Serial.println("System_State==1");
      digitalWrite(26, LOW); // PERMITE ACELERAR(LOW)//NO PERMITE ACELERAR releSONMOT(HIGH)
    }
    else
    {
      digitalWrite(14, LOW); // j8c
    }
    if (millis() > TiempoLCD)
    {
      TiempoLCD = millis() + 500;
      LCD_SOC();
    }
    if (fRPM == true)
    {
      if (tRPM + 3000 < millis())
      {
        tRPM = millis();
        digitalWrite(14, HIGH);
      }
      else if (millis() < tRPM + 100)
        digitalWrite(13, HIGH);
      else if (millis() < tRPM + 200)
        digitalWrite(13, LOW);
      else if (millis() < tRPM + 300)
        digitalWrite(13, HIGH);
      else if (millis() < tRPM + 400)
        digitalWrite(13, LOW);
      else if (millis() > tRPM + 2000)
        fRPM = false;
    }
    if (fRPMmax == true)
    {
      if (tRPMmax + 3000 < millis())
      {
        tRPMmax = millis();
        digitalWrite(14, HIGH);
      }
      else if (millis() < tRPMmax + 50)
        digitalWrite(13, HIGH);
      else if (millis() < tRPMmax + 100)
        digitalWrite(13, LOW);
      else if (millis() < tRPMmax + 150)
        digitalWrite(13, HIGH);
      else if (millis() < tRPMmax + 200)
        digitalWrite(13, LOW);
      else if (millis() < tRPMmax + 250)
        digitalWrite(13, HIGH);
      else if (millis() < tRPMmax + 300)
        digitalWrite(13, LOW);
      else if (millis() > tRPMmax + 2000)
        fRPMmax = false;
    }
    if (millis() > TiempoCluster)
    {
      TiempoCluster = millis() + 20;
      if (twai_transmit(&RPM, pdMS_TO_TICKS(1000)) == ESP_OK)
      {
        incTiempoCluster = 0;
      }
      else
      {
        incTiempoCluster++;
        if (incTiempoCluster > 3)
        {
          reset_twai_driver();
        }
      }
      if (twai_transmit(&HVES1D1, pdMS_TO_TICKS(1000)) == ESP_OK)
      {
        // Serial.println("HVES1D1->" + String(RPM.data[5]));
        incTiempoCluster = 0;
      }
      if (twai_transmit(&HSS1, pdMS_TO_TICKS(1000)) == ESP_OK)
      {
        // Serial.println("HSS1->" + String(RPM.data[5]));
        incTiempoCluster = 0;
      }
    }
  }
}
