#include <ArduinoJson.h>
#include "src/SimpleTimer/SimpleTimer.h"
#include <avr/wdt.h>

#define SEGMENT_DATA PORTC

#define LED_LEVEL3 0
#define LED_LEVEL2 3
#define LED_LEVEL1 12
#define LED_MODE 2

#define MUTE 1
#define MODE_ali 4
#define POWER 11

#define COL_1_MOSI 5
#define COL_2_MISO 6
#define COL_3_SCK 7

#define LED_POWER_ON 10
#define RELAY 31
#define LM324 A6 // 30

#define MOC_1 14 // untuk define plate
#define MOC_2 13 // untuk define heating wire
#define LED_ERROR_ALARM 24
#define LED_SILENCE 25

#define TEMP_PROBE_1 A2
#define TEMP_PROBE_2 A3
#define TEMP_PROBE_3 A4
#define TEMP_PLATE A5

#define BUZZER 15

#define DO 262
#define RE 294
#define MI 330
#define FA 349
#define SOL 395
#define LA 440
#define SI 494
#define DOO 523

uint8_t kondisi_power = 0;
uint8_t pwm_heat = 0;
uint8_t pwm_plate = 0;
uint8_t pwm_mode = 0;
uint8_t segment_ke = 0;
uint8_t x = 0;
uint8_t y = 0;
uint8_t z = 0;

uint8_t awal = 0;

uint8_t val_mode = 0;
uint8_t val_power = 0;
uint8_t val_silent = 0;

uint8_t p = 10;
uint8_t loop_numberx = 0;
uint16_t loop_json = 0;
uint8_t p_old;

uint8_t mode_state = LOW;
uint8_t last_mode;
uint8_t current_mode;

uint8_t power_state = LOW;
uint8_t last_power;
uint8_t current_power;

uint8_t status_mode = 0;

uint8_t silent_state = LOW;
uint8_t last_silent;
uint8_t current_silent;
uint16_t speaker_loop = 0;
uint16_t adc_merah;
uint16_t adc_putih;
uint16_t adc_hijau;
uint16_t adc_plate;
uint16_t adc_termistor;
uint16_t loop_number = 0;
uint16_t out_plate = 0;

float suhu_hijau;
float suhu_putih;
float suhu_merah;
float suhu_plate;
float suhu_termistor;

uint16_t avg_putih[50] = {0};
uint8_t i_putih = 0;
long wholesome_putih = 0;

uint16_t avg_merah[100] = {0};
uint8_t i_merah = 0;
long wholesome_merah = 0;

uint16_t avg_hijau[100] = {0};
uint8_t i_hijau = 0;
long wholesome_hijau = 0;

uint16_t avg_plate[100] = {0};
uint8_t i_plate = 0;
long wholesome_plate = 0;

uint16_t avg_termistor[100] = {0};
uint8_t i_termistor = 0;
long wholesome_termistor = 0;

StaticJsonDocument<70> out;
StaticJsonDocument<30> in;
float TEMPData;
int noTEMPData;
int level;
int MODE, SILENT, ONOFF;
uint8_t alarm = 0;

int CMD = 0;
int FLO = 0;
int command_rcvd;
int command_rcvd_flo;

int error; // for debug
int done, status, t;

unsigned long startcmd;

unsigned char state_pwm = 0;
unsigned long pwm_millis_start;

unsigned char state_pwm_heat = 0;
unsigned long pwm_millis_start_heat;

unsigned char state_pwm2 = 0;
unsigned long pwm_millis_start2;

unsigned int deteksi_wire;
float suhu_tampil;

uint8_t suhu_set_hfnc;
uint8_t flow_set_hfnc;

unsigned long buttonPushedMillis;
unsigned long ledTurnedOnAt;
unsigned long turnOnDelay = 2000;
unsigned long turnOffDelay = 5000;
bool ledReady = false;
bool ledState = false;

static char userInput[30];
static unsigned char iX;

boolean LED1State = false;
boolean LED2State = false;
boolean buttonActive = false;
boolean longPressActive = false;
long buttonTimer = 0;
long longPressTime = 250;

SimpleTimer scheduler;

uint8_t segment(uint8_t data)
{
  switch (data)
  {
    case 0:
      return PORTC = B11111010; // 0
    case 1:
      return PORTC = B01010000; // 1
    case 2:
      return PORTC = B10111001; // 2
    case 3:
      return PORTC = B10101011; // 3
    case 4:
      return PORTC = B11000011; // 4
    case 5:
      return PORTC = B01101011; // 5
    case 6:
      return PORTC = B01111011; // 6
    case 7:
      return PORTC = B10100010; // 7
    case 8:
      return PORTC = B11111011; // 8
    case 9:
      return PORTC = B11101011; // 9
    case 10:
      return PORTC = B01111001; // huruf E
    case 11:
      return PORTC = B01110000; // huruf r
    case 12:
      return PORTC = B11110001; // huruf P
    case 13:
      return PORTC = B11011010; // huruf U
    case 14:
      return PORTC = B01010100; // huruf 1.
    case 15:
      return PORTC = B11110011; // huruf A
    case 16:
      return PORTC = B01011000; // huruf L
    case 17:
      return PORTC = B00000100; //,
    case 18:
      return PORTC = B00000000; //,mati
    case 19:
      return PORTC = B11010011; //,H
  }
  return PORTC = B11111111;
}

void error2_segment()
{
  x = 10;
  y = 11;
  z = 2;
}

void error3_segment()
{
  x = 10;
  y = 11;
  z = 3;
}

void low_segment()
{
  x = 18;
  y = 16;
  z = 18;
}

void mati_segment()
{
  x = 18;
  y = 18;
  z = 18;
}

void high_segment()
{
  x = 18;
  y = 19;
  z = 18;
}

void model_segment()
{
  x = 12;
  y = 10;
  z = 2;
}

void version_segment()
{
  x = 13;
  y = 14;
  z = 0;
}

void button_silent()
{
  if (kondisi_power == 1)
  {
    last_silent = current_silent;
    current_silent = digitalRead(MUTE);
    if (last_silent == HIGH && current_silent == LOW)
    {
      val_silent++;
      silent_state = !silent_state;
    }
    if (val_silent > 1)
    {
      val_silent = 0;
    }
    switch (val_silent)
    {
      case 0:
        SILENT = 0;
        if (adc_putih > 1000)
        {
          error2_segment();
          tampil_segment_x_error(x, y, z);
          speaker_function();
          digitalWrite(MOC_1, LOW);
          pwm_controller_heat(1000, 100);
          alarm = 2;
          digitalWrite(LED_SILENCE, HIGH);
        }
        else if (adc_plate > 1000)
        {
          error3_segment();
          tampil_segment_x_error(x, y, z);
          speaker_function();
          digitalWrite(MOC_1, LOW);
          pwm_controller_heat(1000, 100);

          alarm = 3;
          digitalWrite(LED_SILENCE, HIGH);
        }
        else if (suhu_putih >= 41)
        {
          if (loop_number == 1000)
          {
            suhu_segment_warna(suhu_putih);
          }
          tampil_segment_x(x, y, z);
          speaker_function();
          digitalWrite(MOC_1, LOW);
          pwm_controller_heat(1000, 100);
          digitalWrite(LED_SILENCE, HIGH);
        }

        else if (suhu_putih >= 50)
        {
          high_segment();
          tampil_segment_x_error(x, y, z);
          speaker_function();
          digitalWrite(MOC_1, LOW);
          pwm_controller_heat(1000, 100);
          digitalWrite(LED_SILENCE, HIGH);
        }
        else if (suhu_putih <= 10)
        {
          low_segment();
          tampil_segment_x_error(x, y, z);
          speaker_function();
          digitalWrite(MOC_1, HIGH);
          pwm_controller_heat(1000, 100);
          digitalWrite(LED_SILENCE, HIGH);
        }
        else
        {

          noTone(BUZZER);
          alarm = 0;
          digitalWrite(LED_SILENCE, HIGH);
          digitalWrite(LED_ERROR_ALARM, HIGH);
          if (loop_number == 1000)
          {
            suhu_segment_warna(suhu_putih);
          }
          tampil_segment_x(x, y, z);
        }

        break;

      case 1:
        SILENT = 1;
        if (adc_putih > 1000)
        {
          noTone(BUZZER);
          error2_segment();
          tampil_segment_x_error(x, y, z);
          alarm = 2;
          digitalWrite(LED_ERROR_ALARM, LOW);
          digitalWrite(LED_SILENCE, LOW);
          digitalWrite(MOC_1, LOW);
          pwm_controller_heat(1000, 100);
        }
        else if (adc_plate > 1000)
        {
          noTone(BUZZER);
          error3_segment();
          tampil_segment_x_error(x, y, z);
          alarm = 3;
          digitalWrite(LED_ERROR_ALARM, LOW);
          digitalWrite(LED_SILENCE, LOW);
          digitalWrite(MOC_1, LOW);
          pwm_controller_heat(1000, 100);
        }
        else if (suhu_putih >= 41)
        {
          (LED_ERROR_ALARM, LOW);
          digitalWrite(LED_SILENCE, LOW);
          digitalWrite(MOC_1, LOW);
          if (loop_number == 1000)
          {
            suhu_segment_warna(suhu_putih);
          }
          tampil_segment_x(x, y, z);
          noTone(BUZZER);
          digitalWrite(MOC_1, LOW);
          pwm_controller_heat(1000, 100);
          digitalWrite(LED_SILENCE, LOW);
          digitalWrite(LED_ERROR_ALARM, LOW);
        }
        else if (suhu_putih >= 50)
        {
          high_segment();
          tampil_segment_x_error(x, y, z);
          noTone(BUZZER);
          digitalWrite(MOC_1, LOW);
          pwm_controller_heat(1000, 100);
          digitalWrite(LED_SILENCE, LOW);
        }
        else if (suhu_putih <= 10) {
          low_segment();
          tampil_segment_x_error(x, y, z);
          noTone(BUZZER);
          digitalWrite(MOC_1, HIGH);
          pwm_controller_heat(1000, 100);
          digitalWrite(LED_SILENCE, LOW);
        }
        else
        {
          digitalWrite(LED_ERROR_ALARM, HIGH);
          alarm = 0;
          noTone(BUZZER);
          digitalWrite(LED_SILENCE, HIGH);
          if (loop_number == 1000)
          {
            suhu_segment_warna(suhu_putih);
          }
          tampil_segment_x(x, y, z);
        }
        break;
    }
  }
  else if (kondisi_power == 0)
  {
    noTone(BUZZER);
    alarm = 0;
  }
}
void button_mode()
{
  if (kondisi_power == 1)
  {
    last_mode = current_mode;
    current_mode = digitalRead(MODE_ali);
    if (last_mode == HIGH && current_mode == LOW)
    {
      val_mode++;
      mode_state = !mode_state;
    }

    if (val_mode > 2)
    {
      val_mode = 0;
    }
    switch (val_mode)
    {
      case 0:
        digitalWrite(LED_LEVEL1, LOW);
        digitalWrite(LED_LEVEL2, HIGH);
        digitalWrite(LED_LEVEL3, HIGH);
        mode_suhu(suhu_merah, 35, 20);
        mode_suhu_v2(suhu_putih, suhu_merah);
        break;
      case 1:
        digitalWrite(LED_LEVEL1, LOW);
		digitalWrite(LED_LEVEL2, LOW);
        digitalWrite(LED_LEVEL3, HIGH);
        mode_suhu(suhu_merah, 37, 20);
        mode_suhu_v2(suhu_putih, suhu_merah);
        break;
      case 2:
        digitalWrite(LED_LEVEL3, LOW);
        digitalWrite(LED_LEVEL2, LOW);
        digitalWrite(LED_LEVEL1, LOW);
        mode_suhu(suhu_merah, 40, 20);
        mode_suhu_v2(suhu_putih, suhu_merah);
        break;
    }
  }
}

void tampil_segment_x(uint8_t x, uint8_t y, uint8_t z)
{

  digitalWrite(COL_1_MOSI, LOW);
  digitalWrite(COL_3_SCK, LOW);
  digitalWrite(COL_2_MISO, LOW);
  if (segment_ke == 1)
  {
    SEGMENT_DATA = segment(x);
    digitalWrite(COL_1_MOSI, HIGH);
    digitalWrite(COL_3_SCK, LOW);
    digitalWrite(COL_2_MISO, LOW);
  }
  else if (segment_ke == 2)
  {
    SEGMENT_DATA = segment(y);
    digitalWrite(COL_1_MOSI, LOW);
    digitalWrite(COL_3_SCK, HIGH);
    digitalWrite(COL_2_MISO, LOW);
  }
  else if (segment_ke == 3)
  {
    SEGMENT_DATA = segment(17);
    digitalWrite(COL_1_MOSI, LOW);
    digitalWrite(COL_3_SCK, HIGH);
    digitalWrite(COL_2_MISO, LOW);
  }
  else if (segment_ke == 4)
  {
    SEGMENT_DATA = segment(z);
    digitalWrite(COL_1_MOSI, LOW);
    digitalWrite(COL_3_SCK, LOW);
    digitalWrite(COL_2_MISO, HIGH);
    segment_ke = 0;
  }
  segment_ke++;
}

void tampil_segment_x_error(uint8_t x, uint8_t y, uint8_t z)
{
  digitalWrite(COL_1_MOSI, LOW);
  digitalWrite(COL_3_SCK, LOW);
  digitalWrite(COL_2_MISO, LOW);
  if (segment_ke == 1)
  {
    SEGMENT_DATA = segment(x);
    digitalWrite(COL_1_MOSI, HIGH);
    digitalWrite(COL_3_SCK, LOW);
    digitalWrite(COL_2_MISO, LOW);
  }
  else if (segment_ke == 2)
  {
    SEGMENT_DATA = segment(y);
    digitalWrite(COL_1_MOSI, LOW);
    digitalWrite(COL_3_SCK, HIGH);
    digitalWrite(COL_2_MISO, LOW);
  }
  else if (segment_ke == 3)
  {
    SEGMENT_DATA = segment(z);
    digitalWrite(COL_1_MOSI, LOW);
    digitalWrite(COL_3_SCK, LOW);
    digitalWrite(COL_2_MISO, HIGH);
    segment_ke = 0;
  }
  segment_ke++;
}
void baca_adc_merah()
{ // y=(-8.7176 * x) + 857.1;

  // y = -0.2278x + 99.995
  avg_merah[i_merah] = analogRead(TEMP_PROBE_1);
  wholesome_merah = 0;
  for (uint8_t i = 0; i < 50; i++)
  {
    wholesome_merah += avg_merah[i_merah];
  }
  adc_merah = wholesome_merah / 100;
  suhu_merah = (adc_merah - 438.59) / -4.38;
}

void baca_adc_putih()
{

  // y = -0.0001x^2 + 0.0182x + 60.537
  //suhu_putih = (-0.1129*adc_putih) + 99.455;
//eastern = -0,111x + 99,574
  avg_putih[i_putih] = analogRead(TEMP_PROBE_2);
  wholesome_putih = 0;
  for (uint8_t i = 0; i < 50; i++)
  {
    wholesome_putih += avg_putih[i_putih];
  }

  adc_putih = wholesome_putih / 50;

  //suhu_putih = (-0.1015*adc_putih) + 93.275;
  suhu_putih = (-0.1111 * adc_putih) + 99.574;
  //suhu_putih = 1.0697*suhu_putih - 2.4851;
  i_putih++;
  // suhu_segment_warna(suhu_putih);
  if (i_putih == 50)
  {
    i_putih = 0;
  }
}

void baca_adc_hijau()
{
  // y=(-0.4471 * x) + 652.4;
  // y = -1.2369x + 672.99
  avg_hijau[i_hijau] = analogRead(TEMP_PROBE_3);
  wholesome_hijau = 0;
  for (uint8_t i = 0; i < 50; i++)
  {
    wholesome_hijau += avg_hijau[i_hijau];
  }

  adc_hijau = wholesome_hijau / 50;
  suhu_hijau = (adc_hijau - 672.99) / -1.2369;
}

void baca_adc_plate()
{
  // y= 8.8358 *x - 62.083;
  //  y = -8.339 x + 970.13
  avg_plate[i_plate] = analogRead(TEMP_PLATE);
  wholesome_plate = 0;
  for (uint8_t i = 0; i < 50; i++)
  {
    wholesome_plate += avg_plate[i_plate];
  }
  adc_plate = wholesome_plate / 50;

  suhu_plate = (adc_plate - 1088.3) / -8.2583;
  if (suhu_plate >= 85)
  {
    digitalWrite(MOC_1, LOW);
  }
}

void mode_suhu_v2(uint8_t suhu_warna, uint8_t set_point_suhu)
{
  wdt_reset();
  float suhu_lebih_nol_lima = set_point_suhu + 0.5;
  float suhu_kurang_nol_lima = set_point_suhu - 0.5; // kasih 15
  float suhu_kurang_satu = set_point_suhu - 1;       // kasih 25%
  float suhu_kurang_dua = set_point_suhu - 2;        // kasih 50%
  float suhu_kurang_tiga = set_point_suhu - 3;       // kasih 75%
  if (suhu_warna < suhu_kurang_tiga)
  {
    pwm_controller_heat(1000, 1000);
  }
  else if (suhu_warna >= suhu_kurang_tiga && suhu_warna <= suhu_kurang_dua)
  {
    pwm_controller_heat(1000, 1000);
  }
  else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
  {
    pwm_controller_heat(1000, 1000);
  }
  else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
  {
    pwm_controller_heat(1000, 800);
  }
  else if (suhu_warna >= suhu_kurang_satu && suhu_warna <= suhu_kurang_nol_lima)
  {
    pwm_controller_heat(1000, 800);
  }
  else if (suhu_warna >= set_point_suhu && suhu_warna <= suhu_lebih_nol_lima)
  {
    pwm_controller_heat(1000, 500);
  }
  else if (suhu_warna >= suhu_lebih_nol_lima)
  {
    pwm_controller_heat(1000, 100);
  }
  else if (suhu_warna >= suhu_merah)
  {
    pwm_controller_heat(1000, 100);
  }
}
void mode_suhu(uint8_t suhu_warna, uint8_t set_point_suhu, uint8_t nilai_flow)
{
  wdt_reset();
  float suhu_lebih_nol_lima = set_point_suhu + 0.5;
  float suhu_kurang_nol_lima = set_point_suhu - 0.5; // kasih 15
  float suhu_kurang_satu = set_point_suhu - 1;       // kasih 25%
  float suhu_kurang_dua = set_point_suhu - 2;        // kasih 50%
  float suhu_kurang_tiga = set_point_suhu - 3;       // kasih 75%

  if (nilai_flow >= 20 && nilai_flow <= 30)
  {
    if (suhu_warna < suhu_kurang_tiga)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_tiga && suhu_warna <= suhu_kurang_dua)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 800);
    }
    else if (suhu_warna >= suhu_kurang_satu && suhu_warna <= suhu_kurang_nol_lima)
    {
      pwm_controller(1000, 500);
    }
    else if (suhu_warna >= set_point_suhu && suhu_warna <= suhu_lebih_nol_lima)
    {
      pwm_controller(1000, 250);
    }
    else if (suhu_warna >= suhu_lebih_nol_lima)
    {
      digitalWrite(MOC_1, LOW);
    }
  }
  else if (nilai_flow >= 30 && nilai_flow <= 40)
  {
    if (suhu_warna < suhu_kurang_tiga)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_tiga && suhu_warna <= suhu_kurang_dua)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_satu && suhu_warna <= suhu_kurang_nol_lima)
    {
      pwm_controller(1000, 800);
    }
    else if (suhu_warna >= set_point_suhu && suhu_warna <= suhu_lebih_nol_lima)
    {
      pwm_controller(1000, 800);
    }
    else if (suhu_warna >= suhu_lebih_nol_lima)
    {
      digitalWrite(MOC_1, LOW);
    }
  }
  else if (nilai_flow >= 40 && nilai_flow <= 50)
  {
    if (suhu_warna < suhu_kurang_tiga)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_tiga && suhu_warna <= suhu_kurang_dua)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 800);
    }
    else if (suhu_warna >= suhu_kurang_satu && suhu_warna <= suhu_kurang_nol_lima)
    {
      pwm_controller(1000, 800);
    }
    else if (suhu_warna >= set_point_suhu && suhu_warna <= suhu_lebih_nol_lima)
    {
      pwm_controller(1000, 500);
    }
    else if (suhu_warna >= suhu_lebih_nol_lima)
    {
      digitalWrite(MOC_1, LOW);
    }
  }
  else if (nilai_flow >= 50 && nilai_flow <= 60)
  {
    if (suhu_warna < suhu_kurang_tiga)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_tiga && suhu_warna <= suhu_kurang_dua)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_satu && suhu_warna <= suhu_kurang_nol_lima)
    {
      pwm_controller(1000, 800);
    }
    else if (suhu_warna >= set_point_suhu && suhu_warna <= suhu_lebih_nol_lima)
    {
      pwm_controller(1000, 800);
    }
    else if (suhu_warna >= suhu_lebih_nol_lima)
    {
      digitalWrite(MOC_1, LOW);
    }
  }
  else if (nilai_flow >= 60 && nilai_flow <= 70)
  {

    if (suhu_warna < suhu_kurang_tiga)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_tiga && suhu_warna <= suhu_kurang_dua)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_dua && suhu_warna <= suhu_kurang_satu)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= suhu_kurang_satu && suhu_warna <= suhu_kurang_nol_lima)
    {
      pwm_controller(1000, 1000);
    }
    else if (suhu_warna >= set_point_suhu && suhu_warna <= suhu_lebih_nol_lima)
    {
      pwm_controller(1000, 800);
    }
    else if (suhu_warna >= suhu_lebih_nol_lima)
    {
      digitalWrite(MOC_1, LOW);
    }
  }
  else
  {
    digitalWrite(MOC_1, LOW);
  }
}
void suhu_segment_warna(float adc_warna)
{
  x = adc_warna / 10;
  y = adc_warna - (x * 10);
  z = (adc_warna * 10) - ((x * 100) + (y * 10));
}



void speaker_function()
{
  if (kondisi_power == 1)
  {

    if (speaker_loop == 250)
    {
      tone(BUZZER, LA);
      digitalWrite(LED_ERROR_ALARM, HIGH);
    }

    else if (speaker_loop == 500)
    {
      tone(BUZZER, DOO);
      digitalWrite(LED_ERROR_ALARM, LOW);

      speaker_loop = 0;
    }
    speaker_loop++;

    if (kondisi_power == 0)
    {
      noTone(BUZZER);
    }
  }
}

void print_debug_output()
{
  // Serial.println("============================================================");
  // Serial.print("SM = ");
  // Serial.println(suhu_merah);
  // Serial.print("SP = ");
   //.println(suhu_putih);
  // Serial.print("SH = ");
  // Serial.println(suhu_hijau);
  // Serial.print("ST = ");
  // Serial.println(suhu_termistor);
  // Serial.print("SPL = ");
  // Serial.println(suhu_plate);
  // Serial.print("ADC_merah = ");
  // Serial.println(adc_merah);
   Serial.print("ADC_putih = ");
   Serial.println(adc_putih);
  // Serial.print("ADC_hijau = ");
  // Serial.println(adc_hijau);
  // Serial.print("ADC_plate = ");
  // Serial.println(adc_plate);
  // Serial.print("ADC_termistor = ");
  // Serial.println(adc_termistor);
  // Serial.print("p= ");
  // Serial.println(p);
  // Serial.print("po= ");
  // Serial.println(p_old);
  // Serial.print("val_mode= ");
  // Serial.println(val_mode);
  // Serial.print("heat_wire= ");
  // Serial.println(deteksi_wire);
  // Serial.print("suhu_tampil= ");
  // Serial.println(suhu_tampil);
  // Serial.print("suhu yang di set di hfnc= ");
  // Serial.println(suhu_set_hfnc);
}
void pinmode_setup()
{

  pinMode(LED_LEVEL3, OUTPUT);
  pinMode(MUTE, INPUT);
  pinMode(LED_MODE, OUTPUT);
  pinMode(LED_LEVEL2, OUTPUT);
  pinMode(MODE_ali, INPUT);
  pinMode(COL_1_MOSI, OUTPUT);
  pinMode(COL_2_MISO, OUTPUT);
  pinMode(COL_3_SCK, OUTPUT);

  pinMode(LED_POWER_ON, OUTPUT);
  pinMode(POWER, INPUT);
  pinMode(LED_LEVEL1, OUTPUT);
  pinMode(MOC_2, OUTPUT);
  pinMode(MOC_1, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  pinMode(LED_ERROR_ALARM, OUTPUT);
  pinMode(LED_SILENCE, OUTPUT);
  pinMode(TEMP_PROBE_1, INPUT);
  pinMode(TEMP_PROBE_2, INPUT);
  pinMode(TEMP_PROBE_3, INPUT);
  pinMode(LM324, INPUT);
  pinMode(TEMP_PLATE, INPUT);
  // pinMode(TEMP_TERMISTOR, INPUT);
  pinMode(RELAY, OUTPUT);
  DDRC = 0xFF;
}

void send_json()
{
  // float putih_baru = (suhu_putih, 1);

  float xq = suhu_putih;
  xq = xq + 0.05;              // 123.506
  xq = xq * 10.0;              // 1235.06
  int yq = (int)xq;            // 1235
  float zq = (float)yq / 10.0; // 123.5
  out["PW"] = kondisi_power;   // status humidifier on=1 / off=0
  out["T"] = zq;               // data temperature (float)
  out["TT"] = suhu_set_hfnc;   // display tidak menampilkan temperature. nilai temp data obsolete
  out["LV"] = level;           // level =1-3, 0 apabila led off semua
  out["A"] = alarm;            // alarm =
  out["S"] = SILENT;           // alarm di-mute=1 atau tidak=0
  // out["MO"] = MODE;       // heating wire=1, non heating wire=0

  serializeJson(out, Serial);
  out.clear();
  //Serial.println("");
}
void data_json()
{
  ONOFF = kondisi_power; // status humidifier on=1 / off=0
  TEMPData = suhu_putih; // data temperature (float)
  noTEMPData = 1;        // display tidak menampilkan temperature. nilai temp data obsolete
  // level =1-3, 0 apabila led off semua
  alarm = 2;             // alarm =
  SILENT = 1;
}

void pwm_controller(uint16_t interval, uint16_t duty_cycle)
{
  if (suhu_plate <= 85)
  {
    if (state_pwm == 0)
    {
      pwm_millis_start = millis();
      digitalWrite(MOC_1, HIGH);
      state_pwm = 1;
    }
    else
    { // state_pwm == 1
      if ((millis() - pwm_millis_start) > interval)
      {
        state_pwm = 0;
      }
      else if ((millis() - pwm_millis_start) == duty_cycle)
      {
        digitalWrite(MOC_1, LOW);
      }
      else
      {
      }
    }
  }
}

void pwm_controller_heat(uint16_t interval_heat, uint16_t duty_cycle_heat)
{
  if (state_pwm_heat == 0)
  {
    pwm_millis_start_heat = millis();
    digitalWrite(MOC_2, HIGH);
    state_pwm_heat = 1;
  }
  else
  { // state_pwm == 1
    if ((millis() - pwm_millis_start_heat) > interval_heat)
    {
      state_pwm_heat = 0;
    }
    else if ((millis() - pwm_millis_start_heat) == duty_cycle_heat)
    {

      digitalWrite(MOC_2, LOW);
    }

    else
    {
    }
  }
}

void tampil_loop()
{
  if (kondisi_power == 1)
  {
    if (loop_number == 500)
    {
      // suhu_segment_warna(suhu_putih);
      loop_number = 0;
    }
  }
  else
  {
  }
}

void tampilan_awal()
{
  if (awal == 1)
  {
    for (int i = 0; i < 20000; i++) // program loop untuk menampilkan model dari humidifier
    {
      wdt_reset();
      model_segment();
      tampil_segment_x_error(x, y, z);
    }

    for (int i = 0; i < 20000; i++) // program loop untuk menampilkan version dari humidifier
    {
      wdt_reset();
      version_segment();
      tampil_segment_x_error(x, y, z);
    }

    tampil_segment_x_error(x, y, z);
  }
}

void button_power()
{
  last_power = current_power;
  current_power = digitalRead(POWER);

  if (last_power == HIGH && current_power == LOW)
  {
    val_power++;
    power_state = !power_state;
  }

  if (val_power > 1)
  {
    val_power = 0;
  }
}
void val_power_humi()
{
  switch (val_power)
  {
    case 0:
      awal = 1;
      kondisi_power = 0;

      digitalWrite(RELAY, LOW);

      digitalWrite(COL_1_MOSI, LOW);
      digitalWrite(COL_3_SCK, LOW);
      digitalWrite(COL_2_MISO, LOW);

      digitalWrite(LED_LEVEL1, HIGH);
      digitalWrite(LED_LEVEL2, HIGH);
      digitalWrite(LED_LEVEL3, HIGH);
      digitalWrite(LED_SILENCE, HIGH);

      digitalWrite(LED_POWER_ON, HIGH);
      digitalWrite(LED_ERROR_ALARM, HIGH);
      digitalWrite(LED_MODE, HIGH);
      digitalWrite(MOC_1, LOW);
      digitalWrite(MOC_2, LOW);
      break;

    case 1:
      // pwm_controller_heat(1000, 100);
      //baca_heating_wire();

      tampilan_awal();
      scheduler.run();
      // baca_heating_wire();
      if (awal == 1)
      {
        val_silent = 0;
        val_mode = 0;
      }
      kondisi_power = 1;
      awal = 0;

      digitalWrite(RELAY, HIGH);
      digitalWrite(LED_POWER_ON, LOW);
      if (loop_number == 1000)
      {
        loop_number = 0;
      }
      loop_number++;

      break;
  }
}
void rcvCommand()
{
  while (Serial.available() > 0)
  {
    userInput[iX] = Serial.read();
    // Serial.println(userInput[iX]);
    iX++;
    if (userInput[iX - 1] == '}')
    {

      deserializeJson(in, userInput);
      command_rcvd = in["CMD"];
      command_rcvd_flo = in["FLO"];

      for (unsigned char xo = 0; xo < 30; xo++)
      {
        userInput[xo] = 0;
      }
      iX = 0;

      if (command_rcvd != 0)
      {
        CMD = command_rcvd;
        command_rcvd = 0;
        preparation();
      }

      if (command_rcvd_flo != 0)
      {
        FLO = command_rcvd_flo;
        flow_set_hfnc = FLO;
        command_rcvd_flo = 0;
      }
    }
  }
}

void silent_ditekan()
{


  if (digitalRead(MUTE) == LOW) {
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
    if ((millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      longPressActive = true;
      LED1State = !LED1State;
    }

  } else {

    if (buttonActive == true) {

      if (longPressActive == true) {
		  
		  for (int i = 0; i <= 10000; i++)
        {
          wdt_reset();
		  mati_segment();
          //tampil_segment_x_error(x, y, z);
          //suhu_segment_warna(suhu_merah);
          tampil_segment_x_error(x, y, z);
          switch (val_mode)
          {
            case 0:
              mode_suhu(suhu_merah, 28, 20);
              break;
            case 1:
              mode_suhu(suhu_merah, 33, 50);
              break;
            case 2:
              mode_suhu(suhu_merah, 38, 70);
              break;
          }
        }
		  
		 for (int i = 0; i <= 30000; i++)
        {
          wdt_reset();
		  //error2_segment();
          //tampil_segment_x_error(x, y, z);
          suhu_segment_warna(suhu_merah);
          tampil_segment_x(x, y, z);
          switch (val_mode)
          {
            case 0:
              mode_suhu(suhu_merah, 28, 20);
              break;
            case 1:
              mode_suhu(suhu_merah, 33, 50);
              break;
            case 2:
              mode_suhu(suhu_merah, 38, 70);
              break;
          }
        }
		
		for (int i = 0; i <= 10000; i++)
        {
          wdt_reset();
		  mati_segment();
          //tampil_segment_x_error(x, y, z);
          //suhu_segment_warna(suhu_merah);
          tampil_segment_x_error(x, y, z);
          switch (val_mode)
          {
            case 0:
              mode_suhu(suhu_merah, 28, 20);
              break;
            case 1:
              mode_suhu(suhu_merah, 33, 50);
              break;
            case 2:
              mode_suhu(suhu_merah, 38, 70);
              break;
          }
        }

        longPressActive = false;

      } else {

        LED2State = !LED2State;
      }

      buttonActive = false;

    }

  }

}
void baca_heating_wire()
{
  analogRead(LM324);
  deteksi_wire = analogRead(LM324);

  if (deteksi_wire >= 50)
  {
    digitalWrite(LED_MODE, LOW);
  }
  else if (deteksi_wire <= 50)
  {
    digitalWrite(LED_MODE, HIGH);
  }
}
void preparation()
{
  error = 0;
  switch (CMD)
  {
    case 1:
      val_power = 1;
      break;
    case 2:
      val_power = 0;
      break;
    case 6:
      val_silent = 1;
      break;
    case 7:
      val_silent = 0;
      break;
  }

  if (CMD >= 8)
  {
    suhu_set_hfnc = CMD;
  }
}
void setup()
{
  //Serial.begin(38400);
  pinmode_setup();
  analogReference(EXTERNAL);
  scheduler.setInterval(1000, baca_heating_wire);
  //scheduler.setInterval(1000, print_debug_output);
  wdt_enable(WDTO_8S);
}
void loop()
{
  wdt_reset();
  button_power();
  val_power_humi();
  baca_adc_putih();
  baca_adc_merah();
  baca_adc_hijau();
  baca_adc_plate();
  button_mode();
  button_silent();
  silent_ditekan();
}