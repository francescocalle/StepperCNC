//AUTHORS: Callegaro Francesco - Masiero Elia - 17/05/2026
//Version: 1.0 Stable
#include <SD.h>
#include <SPI.h>
#include <math.h>
#include <Nextion.h>
#include <StepperCNC.h>
#include <Preferences.h>
//dimensioni del piatto di lavoro e valori azzeramento
#define max_x        255  //mm
#define max_y        295  //mm
#define max_z        195  //mm
#define v_reset        2  //mm/s
#define l_reset        3  //mm
#define v_punta        1  //mm/2
#define l_punta        2  //mm
//coordinate per calibrazioni
#define x_punta        0  //mm
#define y_punta      9.5  //mm
#define x_sicur       70  //mm
#define y_sicur       70  //mm
#define z_sicur      100  //mm
#define altezza_sens   5  //mm
#define laser_off_x   -3  //mm
#define laser_off_y  -28  //mm
#define z_pausa       20  //mm
//creazione coordinate punti di calibrazione
#define punto_1_x      0  //mm
#define punto_1_y    295  //mm
#define punto_2_x    255  //mm
#define punto_2_y    295  //mm
#define punto_3_x      0  //mm
#define punto_3_y     70  //mm
#define punto_4_x    255  //mm
#define punto_4_y      0  //mm
//inizializzazione pin e tempi ventole
#define sonda         39
#define ventole        4
#define T_TEMP_OFF    45
#define T_TEMP_ON     15
#define SOGLIA_TEMP 1400
//inizializzazione pin microsd
#define sck           18
#define miso          19
#define mosi          14
#define cs             5
//inizializzazione pin dei motori
#define sda           23
#define scl           22
//inizializzazione pin nextion
#define rx_nex        16
#define tx_nex        17
//inizializzazione pin di output
#define fresa         25
#define luci_p        13
#define luci_pp       27
#define laser_p       26
//inizializzazione pin di flag/interrrupt
#define check12       34
#define schermo       35
#define flag          21
#define flag2         32
#define poweron       33
//inizializzazione indirizzi di trasmissione
#define adress      0x20
//offset da agguingere per tempo di stampa
#define t_offset     6.5  //ms
//inizializzo gli oggetti dello schermo
NexVariable tempos1 = NexVariable(0, 0, "sp1");
NexVariable tempos2 = NexVariable(0, 0, "sp2");
NexVariable tempos3 = NexVariable(0, 0, "sp3");
NexVariable tempos4 = NexVariable(0, 0, "sp4");
NexVariable tempos5 = NexVariable(0, 0, "sp5");
NexVariable vel_mil = NexVariable(0, 0, "velocity1");
NexVariable vel_vel = NexVariable(0, 0, "velocity2");
NexVariable x_val   = NexVariable(0, 0, "init0.x");
NexVariable y_val   = NexVariable(0, 0, "init0.y");
NexVariable z_val   = NexVariable(0, 0, "init0.z");
NexVariable valpwm  = NexVariable(0, 0, "init0.valpwm");
NexVariable valmlk  = NexVariable(0, 0, "init0.valmlk");
NexVariable pagine  = NexVariable(0, 0, "pagine");
NexVariable b_press = NexVariable(0, 0, "bpress");
NexVariable sel1    = NexVariable(0, 0, "sel1");
NexVariable sel2    = NexVariable(0, 0, "sel2");
NexVariable sel3    = NexVariable(0, 0, "sel3");
NexVariable sel4    = NexVariable(0, 0, "sel4");
NexVariable sel5    = NexVariable(0, 0, "sel5");
NexVariable sel6    = NexVariable(0, 0, "sel6");
NexVariable stampa1 = NexVariable(0, 0, "st1");
NexVariable stampa2 = NexVariable(0, 0, "st2");
NexVariable stampa3 = NexVariable(0, 0, "st3");
NexVariable stampa4 = NexVariable(0, 0, "st4");
NexVariable stampa5 = NexVariable(0, 0, "st5");
NexVariable pb_blkm = NexVariable(0, 0, "bll");
NexVariable ul      = NexVariable(0, 0, "ul");
NexVariable hl      = NexVariable(0, 0, "hl");
NexVariable hll     = NexVariable(0, 0, "hll");
NexVariable mm      = NexVariable(0, 0, "mm");
NexVariable nomesel = NexVariable(0, 0, "nomesel");
NexVariable tempsel = NexVariable(0, 0, "tempsel");
NexVariable ric_p   = NexVariable(0, 0, "ricp");
NexVariable prtname = NexVariable(0, 0, "printname");
NexVariable tempopr = NexVariable(0, 0, "tempo");
NexVariable percepr = NexVariable(0, 0, "percentuale");
NexVariable cropx   = NexVariable(0, 0, "setstampa.cropx");
NexVariable cropy   = NexVariable(0, 0, "setstampa.cropy");
NexVariable cor_x   = NexVariable(0, 0, "setstampa.cor_x");
NexVariable cor_y   = NexVariable(0, 0, "setstampa.cor_y");
NexVariable initx   = NexVariable(0, 0, "init0.x_sicura");
NexVariable inity   = NexVariable(0, 0, "init0.y_sicura");
NexVariable stm     = NexVariable(0, 0, "stampa1.stm");
NexVariable bprestm = NexVariable(0, 0, "process.bpressstm");
NexPage home        = NexPage(0, 0, "schermatahome");
NexPage stampe      = NexPage(0, 0, "menustampe");
NexPage homexyz     = NexPage(0, 0, "menuhome");
NexPage menumov     = NexPage(0, 0, "menumovimento");
NexPage menucal     = NexPage(0, 0, "menucalibr");
NexPage out         = NexPage(0, 0, "menusetoutput");
NexPage sicuro      = NexPage(0, 0, "scsicuro");
NexPage preprint    = NexPage(0, 0, "setstampa");
NexPage print       = NexPage(0, 0, "stampa1");
NexPage loading     = NexPage(0, 0, "loading");
NexPage process     = NexPage(0, 0, "process");
NexPage c_punta     = NexPage(0, 0, "cpunta");
NexPage c_bed       = NexPage(0, 0, "bedheight");
NexPage c_angles    = NexPage(0, 0, "anglesheight");
NexPage endprint    = NexPage(0, 0, "endprint");
NexPage setprint    = NexPage(0, 0, "setprint");
NexPage errore4     = NexPage(0, 0, "errore4");
NexPage pausa       = NexPage(0, 0, "stampa2");

//inizializzazione eeprom interna
Preferences preferences;
// Handle della task
TaskHandle_t taskIOHandle = NULL;
File apribile;

//variabili per il programma
volatile int limiter_milling = 100, stop_pressed = 0, last_p = 0, task = 1, s = 0;
volatile float coord_x_min = 0, coord_y_min = 0, feedrate = 0, last_z = 0, limiter_stepper = 10, assixyz[3] = { -1, -1, -1 };
volatile unsigned long line = 0, tempo_stampa = 0, split = 0;
int i = 0, stato_percentuale_pwm = 50;
float lunghezza = 0.1, velocita_motori = 6, distanza_punta = 0, distanza_punta_base = 0, prev_distanza_punta = 0, prev_distanza_punta_base = 0, z_minima = 0;
float stampa_x_min = 0, stampa_x_max = 0, stampa_y_min = 0, stampa_y_max = 0, stampa_x = 0, stampa_y = 0, pre_print_x = 0, pre_print_y = 0;
bool blkm = 0, luci = 0, luci_testa = 0, laser = 0, milling_motor = 0, segno = 1, card = 0, test = 0;
unsigned long line_max = 0, line_intestazione = 0, er = 0, check = millis();
char nomifile[5][101];
uint8_t tempofile[5][2];

void setup() {
  set_fans(sonda, ventole, T_TEMP_OFF, T_TEMP_ON, SOGLIA_TEMP);
  set_pin(fresa, luci_p, luci_pp, laser_p);
  att_luci(1);
  att_luci_testa(1);
  digitalWrite(ventole, 1);
  Serial1.begin(9600, SERIAL_8N1, rx_nex, tx_nex);
  nexInit();
  preinit(flag, flag2, poweron);
  Serial.begin(115200);
  set_interrupt(schermo, check12);
  setexp(sda, scl, adress);
  restore();
  SPI.begin(sck, miso, mosi, cs);
  memset(nomifile, 0, sizeof(nomifile));
  memset(tempofile, 0, sizeof(tempofile));
  preferences.begin("cnc", false);
  distanza_punta = preferences.getFloat("punta", 0.0);
  distanza_punta_base = preferences.getFloat("base", 0.0);
  pre_print_x = preferences.getFloat("pos_x", 0.0);
  pre_print_y = preferences.getFloat("pos_y", 0.0);
  initx.setValue((300 * x_sicur) / max_x);
  inity.setValue((300 * y_sicur) / max_y);
  if (distanza_punta == 0) {
    distanza_punta = 1;
  }
  blkmot(0);
  att_luci(0);
  att_luci_testa(0);
  digitalWrite(ventole, 0);
}
void loop() {
  inviodati();
  check_fans();
}
void inviodati() {
  if (digitalRead(check12) == 0) {
    delay(2);
    if (digitalRead(check12) == 0) {
      delay(2);
      if (digitalRead(check12) == 0) {
        blocco(3);
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
      }
    }
  }
  if (digitalRead(schermo) == 1 && milling_motor == 1) {
    milling_motor = 0;
    pwm_mot(0);
  }
  if (restore() == 1) {
    blkm = 0;
    blkmot(blkm);
    milling_motor = 0;
    pwm_mot(0);
    assixyz[0] = -1;
    assixyz[1] = -1;
    assixyz[2] = -1;
    home.show();
  }
  float v = 0;
  String bufff;
  uint32_t val = 0, pag = 0;
  pagine.getValue(&pag);
  switch (pag) {
    case 2:
      b_press.getValue(&val);
      if (val == 1) {
        b_press.setValue(0);
        loading.show();
        card = lettura_carta(card);
        if (card == 1) {
          stampe.show();
        } else {
          home.show();
        }
      }
      if (assixyz[0] < 0) {
        x_val.setText("?");
      } else {
        x_val.setText(String(assixyz[0], 2).c_str());
      }
      if (assixyz[1] < 0) {
        y_val.setText("?");
      } else {
        y_val.setText(String(assixyz[1], 2).c_str());
      }
      if (assixyz[2] < 0) {
        z_val.setText("?");
      } else {
        z_val.setText(String(assixyz[2], 2).c_str());
      }
      valmlk.setText(String(velocita_motori, 2).c_str());
      if (milling_motor == 1) {
        valpwm.setText(String(stato_percentuale_pwm).c_str());
      } else {
        valpwm.setText(String(0).c_str());
      }
      break;
    case 3:
      card = lettura_carta(card);
      if (card == 0) {
        errore_carta();
      }
      b_press.getValue(&val);
      delay(10);
      if (val > 0) {
        b_press.setValue(0);
        if ((tempofile[val - 1][0] + tempofile[val - 1][1]) > 0) {
          STAMPA(val - 1);
          val = 0;
        }
      }
      char buf[10];
      stampa1.setText(nomifile[0]);
      sprintf(buf, "%dh %dm", tempofile[0][0], tempofile[0][1]);
      tempos1.setText(buf);
      stampa2.setText(nomifile[1]);
      sprintf(buf, "%dh %dm", tempofile[1][0], tempofile[1][1]);
      tempos2.setText(buf);
      stampa3.setText(nomifile[2]);
      sprintf(buf, "%dh %dm", tempofile[2][0], tempofile[2][1]);
      tempos3.setText(buf);
      stampa4.setText(nomifile[3]);
      sprintf(buf, "%dh %dm", tempofile[3][0], tempofile[3][1]);
      tempos4.setText(buf);
      stampa5.setText(nomifile[4]);
      sprintf(buf, "%dh %dm", tempofile[4][0], tempofile[4][1]);
      tempos5.setText(buf);
      break;
    case 4:
      if (assixyz[0] < 0) {
        x_val.setText("?");
      } else {
        x_val.setText(String(assixyz[0], 2).c_str());
      }
      if (assixyz[1] < 0) {
        y_val.setText("?");
      } else {
        y_val.setText(String(assixyz[1], 2).c_str());
      }
      if (assixyz[2] < 0) {
        z_val.setText("?");
      } else {
        z_val.setText(String(assixyz[2], 2).c_str());
      }
      b_press.getValue(&val);
      delay(10);
      if (val == 1) {
        b_press.setValue(0);
        val = 0;
        process.show();
        blkm = !blkm;
        blkmot(blkm);
        if (blkm == 0) {
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        homexyz.show();
      }
      if (val == 2) {
        b_press.setValue(0);
        val = 0;
        process.show();
        blkm = 1;
        blkmot(blkm);
        milling_motor = 0;
        pwm_mot(0);
        s0zz(velocita_motori);
        movxyz(v_reset, 0, 0, l_reset);
        s0zz(v_reset);
        assixyz[2] = 0;
        if (assixyz[0] >= 0 && assixyz[1] >= 0) {
          muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
        }
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        homexyz.show();
      }
      if (val == 3) {
        b_press.setValue(0);
        val = 0;
        process.show();
        blkm = 1;
        blkmot(blkm);
        milling_motor = 0;
        pwm_mot(0);
        if (assixyz[2] >= 0) {
          if (assixyz[2] > z_sicur) {
            movxyz(velocita_motori, 0, 0, -(assixyz[2] - z_sicur));
            assixyz[2] = z_sicur;
          }
          s0xy(velocita_motori);
          movxyz(v_reset, l_reset, l_reset, 0);
          s0xy(v_reset);
        } else {
          s0zz(velocita_motori);
          movxyz(v_reset, 0, 0, l_reset);
          s0zz(v_reset);
          s0xy(velocita_motori);
          movxyz(v_reset, l_reset, l_reset, 0);
          s0xy(v_reset);
          assixyz[2] = 0;
        }
        assixyz[0] = 0;
        assixyz[1] = 0;
        muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        homexyz.show();
      }
      if (val == 4) {
        b_press.setValue(0);
        val = 0;
        process.show();
        blkm = 1;
        blkmot(blkm);
        milling_motor = 0;
        pwm_mot(0);
        s0zz(velocita_motori);
        movxyz(v_reset, 0, 0, l_reset);
        s0zz(v_reset);
        s0xy(velocita_motori);
        movxyz(v_reset, l_reset, l_reset, 0);
        s0xy(v_reset);
        assixyz[0] = 0;
        assixyz[1] = 0;
        assixyz[2] = 0;
        muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        homexyz.show();
      }
      pb_blkm.setValue(blkm * 100);
      break;
    case 5:
      if (assixyz[0] < 0) {
        x_val.setText("?");
      } else {
        x_val.setText(String(assixyz[0], 2).c_str());
      }
      if (assixyz[1] < 0) {
        y_val.setText("?");
      } else {
        y_val.setText(String(assixyz[1], 2).c_str());
      }
      if (assixyz[2] < 0) {
        z_val.setText("?");
      } else {
        z_val.setText(String(assixyz[2], 2).c_str());
      }
      b_press.getValue(&val);
      if (val == 1) {
        b_press.setValue(0);
        segno = 1;
      }
      if (val == 2) {
        b_press.setValue(0);
        segno = 0;
      }
      if (val == 3) {
        b_press.setValue(0);
        lunghezza = 0.1;
      }
      if (val == 4) {
        b_press.setValue(0);
        lunghezza = 10;
      }
      if (val == 5) {
        b_press.setValue(0);
        lunghezza = 1;
      }
      if (val == 6) {
        b_press.setValue(0);
        lunghezza = 100;
      }
      if (val == 7 && blkm == 1 && assixyz[0] >= 0) {
        b_press.setValue(0);
        process.show();
        if (segno == 0) {
          if (assixyz[1] < y_sicur && assixyz[2] > z_sicur && assixyz[0] - lunghezza < x_sicur) {
            muovi_pos(velocita_motori, x_sicur, assixyz[1], assixyz[2]);
          } else {
            muovi_pos(velocita_motori, assixyz[0] - lunghezza, assixyz[1], assixyz[2]);
          }
        } else {
          muovi_pos(velocita_motori, assixyz[0] + lunghezza, assixyz[1], assixyz[2]);
        }
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        menumov.show();
      }
      if (val == 8 && blkm == 1 && assixyz[1] >= 0) {
        b_press.setValue(0);
        process.show();
        if (segno == 0) {
          if (assixyz[0] < x_sicur && assixyz[2] > z_sicur && assixyz[1] - lunghezza < y_sicur) {
            muovi_pos(velocita_motori, assixyz[0], y_sicur, assixyz[2]);
          } else {
            muovi_pos(velocita_motori, assixyz[0], assixyz[1] - lunghezza, assixyz[2]);
          }
        } else {
          muovi_pos(velocita_motori, assixyz[0], assixyz[1] + lunghezza, assixyz[2]);
        }
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        menumov.show();
      }
      if (val == 9 && blkm == 1 && assixyz[2] >= 0) {
        b_press.setValue(0);
        process.show();
        if (segno == 0) {
          muovi_pos(velocita_motori, assixyz[0], assixyz[1], assixyz[2] - lunghezza);
        } else {
          if (assixyz[0] < x_sicur && assixyz[1] < y_sicur && assixyz[2] + lunghezza > z_sicur) {
            muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
          } else {
            if (assixyz[2] + lunghezza > distanza_punta_base) {
              muovi_pos(velocita_motori, assixyz[0], assixyz[1], distanza_punta_base);
            } else {
              muovi_pos(velocita_motori, assixyz[0], assixyz[1], assixyz[2] + lunghezza);
            }
          }
        }
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        menumov.show();
      }
      if (segno == 0) {
        sel1.setValue(0);
        sel2.setValue(1);
      } else {
        sel1.setValue(1);
        sel2.setValue(0);
      }
      if (lunghezza * 10 == 1) {
        sel3.setValue(1);
        sel4.setValue(0);
        sel5.setValue(0);
        sel6.setValue(0);
      } else if (lunghezza * 10 == 100) {
        sel3.setValue(0);
        sel4.setValue(1);
        sel5.setValue(0);
        sel6.setValue(0);
      } else if (lunghezza * 10 == 10) {
        sel3.setValue(0);
        sel4.setValue(0);
        sel5.setValue(1);
        sel6.setValue(0);
      } else {
        sel3.setValue(0);
        sel4.setValue(0);
        sel5.setValue(0);
        sel6.setValue(1);
      }
      break;
    case 6:
      if (assixyz[0] < 0) {
        x_val.setText("?");
      } else {
        x_val.setText(String(assixyz[0], 2).c_str());
      }
      if (assixyz[1] < 0) {
        y_val.setText("?");
      } else {
        y_val.setText(String(assixyz[1], 2).c_str());
      }
      if (assixyz[2] < 0) {
        z_val.setText("?");
      } else {
        z_val.setText(String(assixyz[2], 2).c_str());
      }
      b_press.getValue(&val);
      if (val == 1) {
        b_press.setValue(0);
        process.show();
        blkm = !blkm;
        blkmot(blkm);
        if (blkm == 0) {
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        menucal.show();
      }
      if (val == 2) {
        b_press.setValue(0);
        prev_distanza_punta = distanza_punta;
        prev_distanza_punta_base = distanza_punta_base;
        process.show();
        cambio_punta(0);
        muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
        if (restore() == 1 || distanza_punta_base == 0) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
          distanza_punta = prev_distanza_punta;
          distanza_punta_base = prev_distanza_punta_base;
        }
        preferences.putFloat("punta", distanza_punta);
        preferences.putFloat("base", distanza_punta_base);
        menucal.show();
      }
      if (val == 3) {
        b_press.setValue(0);
        prev_distanza_punta = distanza_punta;
        prev_distanza_punta_base = distanza_punta_base;
        process.show();
        cambio_punta(0);
        muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
        altezza_piano();
        if (restore() == 1 || distanza_punta_base == 0) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
          distanza_punta = prev_distanza_punta;
          distanza_punta_base = prev_distanza_punta_base;
        }
        preferences.putFloat("punta", distanza_punta);
        preferences.putFloat("base", distanza_punta_base);
        menucal.show();
      }
      if (val == 4) {
        b_press.setValue(0);
        prev_distanza_punta = distanza_punta;
        prev_distanza_punta_base = distanza_punta_base;
        process.show();
        cambio_punta(0);
        muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
        altezza_angoli();
        if (restore() == 1 || distanza_punta_base == 0) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
          distanza_punta = prev_distanza_punta;
          distanza_punta_base = prev_distanza_punta_base;
        }
        preferences.putFloat("punta", distanza_punta);
        preferences.putFloat("base", distanza_punta_base);
        menucal.show();
      }
      pb_blkm.setValue(blkm * 100);
      break;
    case 7:
      b_press.getValue(&val);
      if (val == 1) {
        b_press.setValue(0);
        luci = !luci;
        att_luci(luci);
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
      }
      if (val == 2) {
        b_press.setValue(0);
        luci_testa = !luci_testa;
        att_luci_testa(luci_testa);
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
      }
      if (val == 3) {
        b_press.setValue(0);
        laser = !laser;
        att_laser(laser);
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
      }
      if (val == 4) {
        b_press.setValue(0);
        process.show();
        milling_motor = !milling_motor;
        if (blkm == 0 || assixyz[0] < 0 || assixyz[1] < 0 || assixyz[2] < 0) {
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        } else {
          if (milling_motor == 1) {
            pwm_mot(stato_percentuale_pwm);
          } else {
            pwm_mot(0);
          }
        }
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
        }
        out.show();
      }
      ul.setValue(luci * 100);
      hl.setValue(luci_testa * 100);
      hll.setValue(laser * 100);
      mm.setValue(milling_motor * 100);
      break;
    case 9:
      milling_motor = 0;
      pwm_mot(0);
      b_press.getValue(&val);
      v = val;
      if (val > 0) {
        velocita_motori = v / 100;
      }
      bufff = String(velocita_motori, 2);
      bufff += "mm/s";
      vel_vel.setText(bufff.c_str());
      break;
    case 10:
      milling_motor = 0;
      pwm_mot(0);
      b_press.getValue(&val);
      if (val >= 5) {
        stato_percentuale_pwm = val;
      }
      char buff[10];
      sprintf(buff, "%d%%", stato_percentuale_pwm);
      vel_mil.setText(buff);
      break;
  }
  delay(30);
}
bool lettura_carta(bool prev_card) {
  bool controllo = 0;
  if (SD.begin(cs)) {
    if (prev_card == 0) {
      File root = SD.open("/");
      if (root) {
        File file = root.openNextFile();
        memset(nomifile, 0, sizeof(nomifile));
        memset(tempofile, 0, sizeof(tempofile));
        int cursore = 0;
        while (file) {
          check_fans();
          if (!file.isDirectory()) {
            const char* nome = file.name();
            const char* est = strrchr(nome, '.');
            if (est && strcasecmp(est, ".nc") == 0 && cursore < 5) {
              strncpy(nomifile[cursore], file.name(), 100);
              nomifile[cursore][100] = '\0';
              file.seek(0);
              float tempo = calcola_tempo(file);
              int ore = int(tempo) / 60;
              int minuti = int(tempo) % 60;
              tempofile[cursore][0] = ore;
              tempofile[cursore][1] = minuti;
              cursore++;
              if (cursore >= 5) {
                break;
              }
            }
          }
          file.close();
          file = root.openNextFile();
        }
      }
    }
    SD.end();
    controllo = 1;
  } else {
    memset(nomifile, 0, sizeof(nomifile));
    memset(tempofile, 0, sizeof(tempofile));
    controllo = 0;
  }
  return controllo;
}
float calcola_tempo(File file) {
  int len = 0;
  char line[200];
  float x = 0, y = 0, z = 0, f = 0, pre_x = 0, pre_y = 0, pre_z = 0, tempo_tot = 0;
  while(file.available()){
    len = file.readBytesUntil('\n', line, sizeof(line) - 1);
    line[len] = '\0';
    switch (line[0]) {
      case 'G':
        switch (line[1]) {
          case '0':
            switch (line[2]) {
              case '0':
                switch (line[4]) {
                  case 'X':
                    sscanf(line, "%*s X%f Y%f", &x, &y);
                    break;
                  case 'Z':
                    sscanf(line, "%*s Z%f", &z);
                    break;
                  case 'F':
                    sscanf(line, "%*s F%f", &f);
                    break;
                  default:
                    continue;
                }
                break;
              case '1':
                switch (line[4]) {
                  case 'X':
                    sscanf(line, "%*s X%f Y%f", &x, &y);
                    break;
                  case 'Z':
                    sscanf(line, "%*s Z%f", &z);
                    break;
                  case 'F':
                    sscanf(line, "%*s F%f", &f);
                    break;
                  default:
                    continue;
                }
                break;
              default:
                continue;
            }
            break;
          default:
            continue;
        }
        break;
      default:
        continue;
    }
    tempo_tot += (float(t_offset)/60000.0) + hypot(hypot(x - pre_x, y - pre_y), z - pre_z) / fabs(f);
    pre_x = x;
    pre_y = y;
    pre_z = z;
  }
  return tempo_tot + 0.5;
}
bool lettura_range(bool prev_card, int indice) {
  bool controllo = 1;
  if (SD.begin(cs)) {
    File root = SD.open("/");
    if (root) {
      File file = root.openNextFile();
      int cursore = 0;
      while (file) {
        check_fans();
        if (!file.isDirectory()) {
          const char* nome = file.name();
          const char* est = strrchr(nome, '.');
          if (est && strcasecmp(est, ".nc") == 0) {
            if (cursore == indice) {
              file.seek(0);
              char line[120];
              bool trovato_x = false;
              bool trovato_y = false;
              while (file.available()) {
                check_fans();
                int len = file.readBytesUntil('\n', line, sizeof(line) - 1);
                line[len] = '\0';
                if (line[0] != '(') continue;
                if (!trovato_x && strstr(line, "X range")) {
                  if (sscanf(line, "(X range: %f ... %f", &stampa_x_min, &stampa_x_max) == 2) {
                    stampa_x = stampa_x_max - stampa_x_min;
                    trovato_x = true;
                  }
                }
                if (!trovato_y && strstr(line, "Y range")) {
                  if (sscanf(line, "(Y range: %f ... %f", &stampa_y_min, &stampa_y_max) == 2) {
                    stampa_y = stampa_y_max - stampa_y_min;
                    trovato_y = true;
                  }
                }
                if (trovato_x && trovato_y) break;
              }
              file.seek(0);
              line_intestazione = 0;
              while (file.available()) {
                check_fans();
                int len = file.readBytesUntil('\n', line, sizeof(line) - 1);
                line[len] = '\0';
                if (line[0] == 'G' || line[0] == 'M' || line[0] == 'T') {
                  break;
                }
                line_intestazione++;
              }
              file.seek(0);
              line_max = 0;
              while (file.available()) {
                check_fans();
                int len = file.readBytesUntil('\n', line, sizeof(line) - 1);
                line[len] = '\0';
                line_max++;
              }
              file.close();
              break;
            }
            cursore++;
          }
        }
        file.close();
        file = root.openNextFile();
      }
    }
    SD.end();
  } else {
    controllo = 0;
  }
  return controllo;
}
void errore_carta(){
  uint32_t val = 0;
  card = 0;
  errore4.show();
  while(val == 0){
    b_press.getValue(&val);
    delay(300);
    check_fans();
  }
  home.show();
}
void muovi_pos(float vel, float posx, float posy, float posz) {
  if (vel < 0 || blkm == 0 || assixyz[2] < 0) {
    return;
  }
  if (posz < 0) {
    posz = 0;
  }
  if (assixyz[0] < 0 || assixyz[1] < 0) {
    if (posz > z_sicur) {
      posz = z_sicur;
    }
    movxyz(vel, 0, 0, posz - assixyz[2]);
  } else {
    if (posx < 0) {
      posx = 0;
    }
    if (posy < 0) {
      posy = 0;
    }
    if (posx > max_x) {
      posx = max_x;
    }
    if (posy > max_y) {
      posy = max_y;
    }
    if (posz > max_z) {
      posz = max_z;
    }
    movxyz(vel, posx - assixyz[0], posy - assixyz[1], posz - assixyz[2]);
    assixyz[0] = posx;
    assixyz[1] = posy;
  }
  assixyz[2] = posz;
}
void azz_tot() {
  milling_motor = 0;
  pwm_mot(0);
  blkm = 1;
  blkmot(blkm);
  s0zz(velocita_motori);
  movxyz(v_reset, 0, 0, l_reset);
  s0zz(v_reset);
  s0xy(velocita_motori);
  movxyz(v_reset, l_reset, l_reset, 0);
  s0xy(v_reset);
  assixyz[0] = 0;
  assixyz[1] = 0;
  assixyz[2] = 0;
  muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
}
void cambio_punta(int n_punta) {
  milling_motor = 0;
  pwm_mot(0);
  if (blkm == 0 || assixyz[0] < 0 || assixyz[1] < 0 || assixyz[2] < 0) {
    milling_motor = 0;
    pwm_mot(0);
    blkm = 1;
    blkmot(blkm);
    s0zz(velocita_motori);
    movxyz(v_reset, 0, 0, l_reset);
    s0zz(v_reset);
    s0xy(velocita_motori);
    movxyz(v_reset, l_reset, l_reset, 0);
    s0xy(v_reset);
    assixyz[0] = 0;
    assixyz[1] = 0;
    assixyz[2] = 0;
  }
  muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
  muovi_pos(velocita_motori, x_punta, y_punta, z_sicur);
  if (restore() == 1) {
    distanza_punta = 0;
    distanza_punta_base = 0;
    return;
  }
  c_punta.show();
  delayS(500);
  uint32_t val = 0;
  while (val == 0) {
    vTaskDelay(1);
    check_fans();
    if (n_punta > 0) {
      ric_p.setText(String(n_punta).c_str());
    } else {
      ric_p.setText("TEST TIP");
    }
    b_press.getValue(&val);
    delayS(10);
    if (restore() == 1) {
      distanza_punta = 0;
      distanza_punta_base = 0;
      return;
    }
  }
  b_press.setValue(0);
  process.show();
  while (1) {
    vTaskDelay(1);
    assixyz[2] += s0pz(velocita_motori);
    muovi_pos(v_punta, assixyz[0], assixyz[1], assixyz[2] - l_punta);
    assixyz[2] += s0pz(v_punta);
    if (distanza_punta > 0) {
      distanza_punta_base = distanza_punta_base - (distanza_punta - assixyz[2]);
    } else {
      distanza_punta_base = 1;
    }
    distanza_punta = assixyz[2];
    if (assixyz[2] <= float(z_sicur - l_punta) || restore() == 1) {
      distanza_punta = 0;
      distanza_punta_base = 0;
      return;
    }
    check = millis();
    muovi_pos(velocita_motori, x_punta, y_punta, z_sicur);
    if ((millis() - check) >= 50) {
      break;
    } else {
      s0zz(velocita_motori);
      movxyz(v_reset, 0, 0, l_reset);
      s0zz(v_reset);
      assixyz[2] = 0;
    }
  }
}
void altezza_piano() {
  if (distanza_punta == 0 || distanza_punta_base == 0) {
    return;
  }
  muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, distanza_punta + altezza_sens);
  uint32_t val = 0;
  uint32_t pag = 0;
  float selezione = 0.01;
  float offset = distanza_punta + altezza_sens;
  c_bed.show();
  delayS(500);
  while (1) {
    pagine.getValue(&pag);
    if (pag != 21) {
      c_bed.show();
      delayS(500);
    }
    b_press.getValue(&val);
    if (val == 1) {
      b_press.setValue(0);
      selezione = 0.01;
    }
    if (val == 2) {
      b_press.setValue(0);
      selezione = 0.05;
    }
    if (val == 3) {
      b_press.setValue(0);
      selezione = 0.1;
    }
    if (val == 4) {
      b_press.setValue(0);
      selezione = 0.5;
    }
    if (val == 5) {
      b_press.setValue(0);
      selezione = 1;
    }
    if (val == 6) {
      b_press.setValue(0);
      selezione = 5;
    }
    if (val == 7) {
      b_press.setValue(0);
      if (offset - selezione < distanza_punta + altezza_sens) {
        movxyz(v_punta, 0, 0, (distanza_punta + altezza_sens) - offset);
        offset = distanza_punta + altezza_sens;
      } else {
        movxyz(v_punta, 0, 0, -selezione);
        offset = offset - selezione;
      }
    }
    if (val == 8) {
      b_press.setValue(0);
      if (offset - selezione > max_z) {
        movxyz(v_punta, 0, 0, max_z - offset);
        offset = max_z;
      } else {
        movxyz(v_punta, 0, 0, selezione);
        offset = offset + selezione;
      }
    }
    if (val == 9) {
      b_press.setValue(0);
      process.show();
      muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
      return;
    }
    if (val == 10) {
      b_press.setValue(0);
      process.show();
      break;
    }
    delayS(10);
    if (restore() == 1) {
      distanza_punta_base = 0;
      return;
    }
    switch (int(selezione * 100)) {
      case 1:
        sel1.setValue(1);
        sel2.setValue(0);
        sel3.setValue(0);
        sel4.setValue(0);
        sel5.setValue(0);
        sel6.setValue(0);
        break;
      case 5:
        sel1.setValue(0);
        sel2.setValue(1);
        sel3.setValue(0);
        sel4.setValue(0);
        sel5.setValue(0);
        sel6.setValue(0);
        break;
      case 10:
        sel1.setValue(0);
        sel2.setValue(0);
        sel3.setValue(1);
        sel4.setValue(0);
        sel5.setValue(0);
        sel6.setValue(0);
        break;
      case 50:
        sel1.setValue(0);
        sel2.setValue(0);
        sel3.setValue(0);
        sel4.setValue(1);
        sel5.setValue(0);
        sel6.setValue(0);
        break;
      case 100:
        sel1.setValue(0);
        sel2.setValue(0);
        sel3.setValue(0);
        sel4.setValue(0);
        sel5.setValue(1);
        sel6.setValue(0);
        break;
      case 500:
        sel1.setValue(0);
        sel2.setValue(0);
        sel3.setValue(0);
        sel4.setValue(0);
        sel5.setValue(0);
        sel6.setValue(1);
        break;
      default:
        sel1.setValue(0);
        sel2.setValue(0);
        sel3.setValue(0);
        sel4.setValue(0);
        sel5.setValue(0);
        sel6.setValue(0);
        break;
    }
    z_val.setText(String(offset, 2).c_str());
    check_fans();
  }
  distanza_punta_base = offset;
  assixyz[2] = offset;
  preferences.putFloat("base", distanza_punta_base);
  muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
}
void altezza_angoli() {
  if (distanza_punta == 0 || distanza_punta_base == 0) {
    return;
  }
  muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
  uint32_t val = 0;
  uint32_t pag = 0;
  c_angles.show();
  delayS(500);
  while (1) {
    pagine.getValue(&pag);
    if (pag != 22) {
      c_angles.show();
      delayS(500);
    }
    b_press.getValue(&val);
    if (val == 1) {
      b_press.setValue(0);
      process.show();
      muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
      muovi_pos(velocita_motori, punto_1_x, punto_1_y, z_sicur);
      muovi_pos(velocita_motori, assixyz[0], assixyz[1], distanza_punta_base);
    }
    if (val == 2) {
      b_press.setValue(0);
      process.show();
      muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
      muovi_pos(velocita_motori, punto_2_x, punto_2_y, z_sicur);
      muovi_pos(velocita_motori, assixyz[0], assixyz[1], distanza_punta_base);
    }
    if (val == 3) {
      b_press.setValue(0);
      process.show();
      muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
      muovi_pos(velocita_motori, punto_3_x, punto_3_y, z_sicur);
      muovi_pos(velocita_motori, assixyz[0], assixyz[1], distanza_punta_base);
    }
    if (val == 4) {
      b_press.setValue(0);
      process.show();
      muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
      muovi_pos(velocita_motori, punto_4_x, punto_4_y, z_sicur);
      muovi_pos(velocita_motori, assixyz[0], assixyz[1], distanza_punta_base);
    }
    if (val == 5) {
      b_press.setValue(0);
      process.show();
      break;
    }
    delayS(10);
    if (restore() == 1) {
      distanza_punta_base = 0;
      return;
    }
    check_fans();
  }
  muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
  muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
}
void STAMPA(int n_file) {
  loading.show();
  milling_motor = 0;
  pwm_mot(0);
  laser = 0;
  att_laser(laser);
  last_z = 0;
  last_p = 0;
  coord_x_min = 0;
  coord_y_min = 0;
  stop_pressed = 0;
  limiter_stepper = 10;
  limiter_milling = 100;
  uint32_t px_x = 0, px_y = 0, val = 0, vall = 0, pag = 0;
  int velocita_limiter_pwm = 100, fase = 1;
  float v = 0, velocita_limiter = 10;
  char buf[15], buf1[10], buff[10];
  bool stop = 0, mode = 0;
  card = lettura_range(card, n_file);
  if (card == 0 || stampa_x > max_x || stampa_y > max_y || distanza_punta_base < z_sicur) {
    home.show();
    card = 0;
    return;
  }
  if (stampa_x > max_x - x_sicur && stampa_y > max_y - y_sicur) {
    home.show();
    card = 0;
    return;
  }
  cropx.setValue(int((stampa_x * 300) / max_x));
  cropy.setValue(int((stampa_y * 300) / max_y));
  preprint.show();
  delay(500);
  while (stop == 0) {
    check_fans();
    pagine.getValue(&pag);
    switch (fase) {
      case 1:
        if (pag != 24) {
          preprint.show();
          delayS(500);
        }
        b_press.getValue(&val);
        if (val == 1) {
          b_press.setValue(0);
          home.show();
          card = 0;
          return;
        }
        if (val == 2) {
          b_press.setValue(0);
          cor_x.getValue(&px_x);
          cor_y.getValue(&px_y);
          coord_x_min = (float(px_x) * float(max_x)) / 300.0;
          coord_y_min = (float(px_y) * float(max_y)) / 300.0;
          sicuro.show();
          nomesel.setText(nomifile[n_file]);
          sprintf(buf1, "%dh %dm", tempofile[n_file][0], tempofile[n_file][1]);
          tempsel.setText(buf1);
          fase = 2;
        }
        if (val == 3) {
          pre_print_x = preferences.getFloat("pos_x", 0.0);
          pre_print_y = preferences.getFloat("pos_y", 0.0);
          cor_x.setValue(pre_print_x);
          cor_y.setValue(pre_print_y);
          b_press.setValue(4);
        }
        break;
      case 2:
        if (pag != 14) {
          sicuro.show();
          nomesel.setText(nomifile[n_file]);
          sprintf(buf1, "%dh %dm", tempofile[n_file][0], tempofile[n_file][1]);
          tempsel.setText(buf1);
          delayS(500);
        }
        b_press.getValue(&val);
        if (val == 1) {
          b_press.setValue(0);
          preprint.show();
          fase = 1;
        }
        if (val == 2) {
          b_press.setValue(0);
          process.show();
          if (blkm == 0) {
            milling_motor = 0;
            pwm_mot(0);
            blkm = 1;
            blkmot(blkm);
            s0zz(velocita_motori);
            movxyz(v_reset, 0, 0, l_reset);
            s0zz(v_reset);
            s0xy(velocita_motori);
            movxyz(v_reset, l_reset, l_reset, 0);
            s0xy(v_reset);
            assixyz[0] = 0;
            assixyz[1] = 0;
            assixyz[2] = 0;
          }
          muovi_pos(velocita_motori, coord_x_min + laser_off_x, max_y - (coord_y_min - laser_off_y), z_sicur);
          att_laser(1);
          muovi_pos(velocita_motori, coord_x_min + laser_off_x, max_y - (coord_y_min + stampa_y - laser_off_y), z_sicur);
          muovi_pos(velocita_motori, coord_x_min + stampa_x + laser_off_x, max_y - (coord_y_min + stampa_y - laser_off_y), z_sicur);
          muovi_pos(velocita_motori, coord_x_min + stampa_x + laser_off_x, max_y - (coord_y_min - laser_off_y), z_sicur);
          muovi_pos(velocita_motori, coord_x_min + laser_off_x, max_y - (coord_y_min - laser_off_y), z_sicur);
          att_laser(0);
          muovi_pos(velocita_motori, float(max_x) / 2.0, float(max_y) / 2.0, z_sicur);
          if (restore() == 1) {
            blkm = 0;
            blkmot(blkm);
            milling_motor = 0;
            pwm_mot(0);
            assixyz[0] = -1;
            assixyz[1] = -1;
            assixyz[2] = -1;
            home.show();
            return;
          }
          sicuro.show();
          nomesel.setText(nomifile[n_file]);
          sprintf(buf1, "%dh %dm", tempofile[n_file][0], tempofile[n_file][1]);
          tempsel.setText(buf1);
          delay(500);
        }
        if (val == 3) {
          b_press.setValue(0);
          preferences.putFloat("pos_x", px_x);
          preferences.putFloat("pos_y", px_y);
          process.show();
          stop = 1;
        }
        break;
      default:
        val = 0;
        break;
    }
    if (restore() == 1) {
      blkm = 0;
      blkmot(blkm);
      milling_motor = 0;
      pwm_mot(0);
      laser = 0;
      att_laser(laser);
      assixyz[0] = -1;
      assixyz[1] = -1;
      assixyz[2] = -1;
      home.show();
      card = 0;
      return;
    }
    delayS(30);
  }
  if (!SD.begin(cs)) {
    errore_carta();
    return;
  }
  File root = SD.open("/");
  if (!root) {
    errore_carta();
    return;
  }
  File file = root.openNextFile();
  unsigned long t_card = millis();
  while (file) {
    if(millis() - t_card >= 2500){
      file.close();
      root.close();
      SD.end();
      errore_carta();
      return;
    }
    if (!file.isDirectory()) {
      const char* nome = file.name();
      if (strncmp(nome, nomifile[n_file], strlen(nomifile[n_file])) == 0) {
        char path[120];
        snprintf(path, sizeof(path), "/%s", nome);
        apribile = SD.open(path);
        if (!apribile) {
          file.close();
          root.close();
          SD.end();
          errore_carta();
          return;
        }
        break;
      }
    }
    file.close();
    file = root.openNextFile();
  }
  root.close();
  if (!apribile) {
    SD.end();
    errore_carta();
    return;
  }
  if (strstr(nomifile[n_file], "drl")) {
    mode = 1;
  }
  if (strstr(nomifile[n_file], "Top-Cop")) {
    mode = 1;
  }
  stop = 0;
  blkm = 1;
  blkmot(blkm);
  s0zz(velocita_motori);
  movxyz(v_reset, 0, 0, l_reset);
  s0zz(v_reset);
  s0xy(velocita_motori);
  movxyz(v_reset, l_reset, l_reset, 0);
  s0xy(v_reset);
  assixyz[0] = 0;
  assixyz[1] = 0;
  assixyz[2] = 0;
  if (restore() == 1) {
    apribile.close();
    SD.end();
    blkm = 0;
    blkmot(blkm);
    milling_motor = 0;
    pwm_mot(0);
    laser = 0;
    att_laser(laser);
    assixyz[0] = -1;
    assixyz[1] = -1;
    assixyz[2] = -1;
    home.show();
    card = 0;
    return;
  }
  apribile.seek(0);
  task = 1;
  bprestm.setValue(4);
  delayS(100);
  tempo_stampa = millis();
  xTaskCreatePinnedToCore(task_stampa, "task_stampa", 16384, NULL, 1, &taskIOHandle, 0);
  stm.setValue(mode);
  val = 0;
  while (stop == 0) {
    check_fans();
    pagine.getValue(&pag);
    switch (pag) {
      case 0:
        delay(100);
        break;
      case 16:
        pag = 0;
        bprestm.getValue(&val);
        switch (val) {
          case 0:
            delayS(100);
            bprestm.getValue(&val);
            if (val == 0) {
              if(stop_pressed == 2){
                pausa.show();
              }else{
                print.show();
              }
              delayS(400);
            }
            val = 0;
            delayS(100);
            break;
          case 1:
            bprestm.setValue(0);
            val = 0;
            stop_pressed = 2;
            while(task != 3){
              vTaskDelay(pdMS_TO_TICKS(50));
            }
            if (velocita_motori > limiter_stepper) {
              muovi_pos(limiter_stepper, assixyz[0], assixyz[1], z_sicur);
            } else {
              muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
            }
            milling_motor = 0;
            pwm_mot(0);
            pausa.show();
            delayS(300);
            break;
          case 2:
            bprestm.setValue(0);
            val = 0;
            luci = !luci;
            att_luci(luci);
            setprint.show();
            delayS(500);
            break;
          case 3:
            bprestm.setValue(0);
            val = 0;
            luci_testa = !luci_testa;
            att_luci_testa(luci_testa);
            setprint.show();
            delayS(500);
            break;
          case 4:
            delayS(200);
            break;
          case 5:
            bprestm.setValue(0);
            val = 0;
            if (s > limiter_milling) {
              pwm_mot(limiter_milling);
            } else {
              pwm_mot(s);
            }
            if (velocita_motori > limiter_stepper) {
              muovi_pos(limiter_stepper, assixyz[0], assixyz[1], last_z);
            } else {
              muovi_pos(velocita_motori, assixyz[0], assixyz[1], last_z);
            }
            stop_pressed = 0;
            task = 1;
            tempo_stampa = tempo_stampa + (millis() - split);
            sprintf(buf, "%dh %dm ", (millis() - tempo_stampa) / 3600000, ((millis() - tempo_stampa) % 3600000) / 60000);
            print.show();
            delayS(30);
            tempopr.setText(buf);
            break;
        }
        break;
      case 18:
        delay(100);
        break;
      case 19:
        delay(100);
        break;
      case 20:
        delay(100);
        break;
      case 23:
        pag = 0;
        limiter_stepper = velocita_limiter;
        limiter_milling = velocita_limiter_pwm;
        prtname.setText(String(nomifile[n_file]).c_str());
        sprintf(buf, "%dh %dm ", (millis() - tempo_stampa) / 3600000, ((millis() - tempo_stampa) % 3600000) / 60000);
        tempopr.setText(buf);
        percepr.setValue(map(line - (line_intestazione - 1), 0, (line_max - 2) - (line_intestazione - 1), 0, 100));
        x_val.setText(String(assixyz[0], 2).c_str());
        y_val.setText(String(assixyz[1], 2).c_str());
        z_val.setText(String(assixyz[2], 2).c_str());
        if (feedrate > limiter_stepper) {
          valmlk.setText(String(limiter_stepper, 2).c_str());
        } else {
          valmlk.setText(String(feedrate, 2).c_str());
        }
        if (s > limiter_milling) {
          valpwm.setText(String(limiter_milling).c_str());
        } else {
          valpwm.setText(String(s).c_str());
        }
        break;
      case 27:
        pag = 0;
        limiter_stepper = velocita_limiter;
        limiter_milling = velocita_limiter_pwm;
        ul.setValue(luci * 100);
        hl.setValue(luci_testa * 100);
        break;
      case 28:
        pag = 0;
        b_press.getValue(&val);
        v = val;
        if (val > 0) {
          velocita_limiter = v / 100;
        }
        dtostrf(velocita_limiter, 1, 2, buff);
        strcat(buff, "mm/s");
        vel_vel.setText(buff);
        break;
      case 29:
        pag = 0;
        b_press.getValue(&val);
        if (val >= 5) {
          velocita_limiter_pwm = val;
        }
        sprintf(buff, "%d%%", velocita_limiter_pwm);
        vel_mil.setText(buff);
        break;
      case 30:
        if(stop_pressed < 2){
          stop_pressed = 1;
        }else{
          vTaskDelete(taskIOHandle);
          vTaskDelay(pdMS_TO_TICKS(200));
          taskIOHandle = NULL;
          apribile.close();
          SD.end();
          milling_motor = 0;
          pwm_mot(0);
          laser = 0;
          att_laser(laser);
          process.show();
          muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
          muovi_pos(velocita_motori, float(max_x) / 2.0, max_y, z_sicur);
          if (restore() == 1) {
            blkm = 0;
            blkmot(blkm);
            milling_motor = 0;
            pwm_mot(0);
            laser = 0;
            att_laser(laser);
            assixyz[0] = -1;
            assixyz[1] = -1;
            assixyz[2] = -1;
            home.show();
            card = 0;
            return;
          }
          home.show();
          card = 0;
          return;
        }
        delayS(100);
        break;
      case 32:
        pag = 0;
        b_press.getValue(&val);
        if(val == 1){
          process.show();
          float s_x = assixyz[0];
          float s_y = assixyz[1];
          prev_distanza_punta = distanza_punta;
          prev_distanza_punta_base = distanza_punta_base;
          cambio_punta(last_p);
          if(distanza_punta_base == 0){
            distanza_punta = prev_distanza_punta;
            distanza_punta_base = prev_distanza_punta_base;
            preferences.putFloat("punta", distanza_punta);
            preferences.putFloat("base", distanza_punta_base);
            vTaskDelete(taskIOHandle);
            vTaskDelay(pdMS_TO_TICKS(200));
            taskIOHandle = NULL;
            apribile.close();
            SD.end();
            blkm = 0;
            blkmot(blkm);
            milling_motor = 0;
            pwm_mot(0);
            laser = 0;
            att_laser(laser);
            assixyz[0] = -1;
            assixyz[1] = -1;
            assixyz[2] = -1;
            home.show();
            card = 0;
            return;
          }
          preferences.putFloat("punta", distanza_punta);
          preferences.putFloat("base", distanza_punta_base);
          vTaskDelay(1);
          muovi_pos(velocita_motori, s_x, s_y, z_sicur);
          pausa.show();
          delayS(300);
        }
        limiter_stepper = velocita_limiter;
        limiter_milling = velocita_limiter_pwm;
        prtname.setText(String(nomifile[n_file]).c_str());
        x_val.setText(String(assixyz[0], 2).c_str());
        y_val.setText(String(assixyz[1], 2).c_str());
        z_val.setText(String(assixyz[2], 2).c_str());
        if (feedrate > limiter_stepper) {
          valmlk.setText(String(limiter_stepper, 2).c_str());
        } else {
          valmlk.setText(String(feedrate, 2).c_str());
        }
        if (s > limiter_milling) {
          valpwm.setText(String(limiter_milling).c_str());
        } else {
          valpwm.setText(String(s).c_str());
        }
        delayS(50);
        if(restore() == 1){
          vTaskDelete(taskIOHandle);
          vTaskDelay(pdMS_TO_TICKS(200));
          taskIOHandle = NULL;
          apribile.close();
          SD.end();
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          laser = 0;
          att_laser(laser);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
          home.show();
          card = 0;
          return;
        }
        break;
    }
    switch (task) {
      case -2:
        vTaskDelete(taskIOHandle);
        vTaskDelay(pdMS_TO_TICKS(200));
        taskIOHandle = NULL;
        apribile.close();
        SD.end();
        blkm = 0;
        blkmot(blkm);
        milling_motor = 0;
        pwm_mot(0);
        laser = 0;
        att_laser(laser);
        assixyz[0] = -1;
        assixyz[1] = -1;
        assixyz[2] = -1;
        errore_carta();
        return;
      case -1:
        vTaskDelete(taskIOHandle);
        vTaskDelay(pdMS_TO_TICKS(200));
        taskIOHandle = NULL;
        apribile.close();
        SD.end();
        milling_motor = 0;
        pwm_mot(0);
        laser = 0;
        att_laser(laser);
        process.show();
        muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
        muovi_pos(velocita_motori, float(max_x) / 2.0, max_y, z_sicur);
        if (restore() == 1) {
          blkm = 0;
          blkmot(blkm);
          milling_motor = 0;
          pwm_mot(0);
          laser = 0;
          att_laser(laser);
          assixyz[0] = -1;
          assixyz[1] = -1;
          assixyz[2] = -1;
          home.show();
          card = 0;
          return;
        }
        home.show();
        card = 0;
        return;
      case 0:
        vTaskDelete(taskIOHandle);
        vTaskDelay(pdMS_TO_TICKS(200));
        taskIOHandle = NULL;
        apribile.close();
        SD.end();
        blkm = 0;
        blkmot(blkm);
        milling_motor = 0;
        pwm_mot(0);
        laser = 0;
        att_laser(laser);
        assixyz[0] = -1;
        assixyz[1] = -1;
        assixyz[2] = -1;
        home.show();
        card = 0;
        return;
      case 2:
        stop = 1;
        break;
    }
    vTaskDelay(1);
  }
  vTaskDelete(taskIOHandle);
  vTaskDelay(pdMS_TO_TICKS(200));
  taskIOHandle = NULL;
  milling_motor = 0;
  pwm_mot(0);
  apribile.close();
  SD.end();
  process.show();
  tempo_stampa = millis() - tempo_stampa;
  muovi_pos(velocita_motori, assixyz[0], assixyz[1], z_sicur);
  muovi_pos(velocita_motori, float(max_x) / 2.0, max_y, z_sicur);
  if (restore() == 1) {
    blkm = 0;
    blkmot(blkm);
    milling_motor = 0;
    pwm_mot(0);
    laser = 0;
    att_laser(laser);
    assixyz[0] = -1;
    assixyz[1] = -1;
    assixyz[2] = -1;
    home.show();
    card = 0;
    return;
  }
  val = 0;
  endprint.show();
  sprintf(buf, "%dh %dm %ds", tempo_stampa / 3600000, (tempo_stampa % 3600000) / 60000, (tempo_stampa % 60000) / 1000);
  ric_p.setText(buf);
  while (val == 0) {
    check_fans();
    b_press.getValue(&val);
    delay(100);
  }
  card = 0;
  home.show();
}
void task_stampa(void* parameter) {
  s = 0;
  feedrate = 0;
  line = line_intestazione - 1;
  float x = 0, y = 0, z = 0, f = 0;
  bool tip = 0;
  char buf[15], riga[200];
  int len = 0, p = 0;
  while (1) {
    if (apribile.available()) {
      len = apribile.readBytesUntil('\n', riga, sizeof(riga) - 1);
      if (len <= 0) {
        task = -2;
        while (1) {
          vTaskDelay(pdMS_TO_TICKS(200));
        }
      }
      riga[len] = '\0';
      switch (riga[0]) {
        case 'M':
          switch (riga[1]) {
            case '0':
              if (riga[2] == '3') {
                sscanf(riga, "%*[^S]S%d", &s);
                if (s > 100) {
                  s = 100;
                }
                if (s < 0) {
                  s = 0;
                }
              }
              if (riga[2] == '5') {
                s = 0;
              }
              break;
          }
          break;
        case 'G':
          switch (riga[1]) {
            case '2':
              switch (riga[2]) {
                case '1':
                  break;
                default:
                  task = -1;
                  while (1) {
                    vTaskDelay(pdMS_TO_TICKS(200));
                  }
              }
              break;
            case '9':
              switch (riga[2]) {
                case '0':
                  break;
                case '4':
                  break;
                default:
                  task = -1;
                  while (1) {
                    vTaskDelay(pdMS_TO_TICKS(200));
                  }
              }
              break;
            case '0':
              switch (riga[2]) {
                case '0':
                  switch (riga[4]) {
                    case 'X':
                      sscanf(riga, "%*s X%f Y%f", &x, &y);
                      if (tip == 0) {
                        break;
                      }
                      if (feedrate > limiter_stepper) {
                        muovi_pos(limiter_stepper, x + coord_x_min, max_y - ((stampa_y - y) + coord_y_min), assixyz[2]);
                      } else {
                        muovi_pos(feedrate, x + coord_x_min, max_y - ((stampa_y - y) + coord_y_min), assixyz[2]);
                      }
                      break;
                    case 'Z':
                      sscanf(riga, "%*s Z%f", &z);
                      if (z < 0) {
                        z = -z;
                      }
                      if (tip == 0) {
                        break;
                      }
                      if (feedrate > limiter_stepper) {
                        muovi_pos(limiter_stepper, assixyz[0], assixyz[1], distanza_punta_base - z);
                      } else {
                        muovi_pos(feedrate, assixyz[0], assixyz[1], distanza_punta_base - z);
                      }
                      break;
                    case 'F':
                      sscanf(riga, "%*s F%f", &f);
                      feedrate = f / 60.0;
                      if (feedrate > 10) {
                        feedrate = 10;
                      }
                      if (feedrate < 0.01) {
                        feedrate = 0.01;
                      }
                      break;
                  }
                  break;
                case '1':
                  switch (riga[4]) {
                    case 'X':
                      sscanf(riga, "%*s X%f Y%f", &x, &y);
                      if (tip == 0) {
                        break;
                      }
                      if (feedrate > limiter_stepper) {
                        muovi_pos(limiter_stepper, x + coord_x_min, max_y - ((stampa_y - y) + coord_y_min), assixyz[2]);
                      } else {
                        muovi_pos(feedrate, x + coord_x_min, max_y - ((stampa_y - y) + coord_y_min), assixyz[2]);
                      }
                      break;
                    case 'Z':
                      sscanf(riga, "%*s Z%f", &z);
                      if (z < 0) {
                        z = -z;
                      }
                      if (tip == 0) {
                        break;
                      }
                      if (feedrate > limiter_stepper) {
                        muovi_pos(limiter_stepper, assixyz[0], assixyz[1], distanza_punta_base - z);
                      } else {
                        muovi_pos(feedrate, assixyz[0], assixyz[1], distanza_punta_base - z);
                      }
                      break;
                    case 'F':
                      sscanf(riga, "%*s F%f", &f);
                      feedrate = f / 60.0;
                      if (feedrate > 10) {
                        feedrate = 10;
                      }
                      if (feedrate < 0.01) {
                        feedrate = 0.01;
                      }
                      break;
                  }
                  break;
              }
              break;
          }
          break;
        case 'T':
          bprestm.setValue(4);
          delayS(100);
          if(tip == 1){
            process.show();
          }
          pwm_mot(0);
          split = millis();
          sscanf(riga, "T%d", &p);
          last_p = p;
          prev_distanza_punta = distanza_punta;
          prev_distanza_punta_base = distanza_punta_base;
          cambio_punta(p);
          if(distanza_punta_base == 0){
            distanza_punta = prev_distanza_punta;
            distanza_punta_base = prev_distanza_punta_base;
            preferences.putFloat("punta", distanza_punta);
            preferences.putFloat("base", distanza_punta_base);
            task = 0;
            while (1) {
              vTaskDelay(pdMS_TO_TICKS(200));
            }
          }
          preferences.putFloat("punta", distanza_punta);
          preferences.putFloat("base", distanza_punta_base);
          vTaskDelay(1);
          tip = 1;
          muovi_pos(velocita_motori, x + coord_x_min, max_y - ((stampa_y - y) + coord_y_min), assixyz[2]);
          muovi_pos(velocita_motori, assixyz[0], assixyz[1], distanza_punta_base - z);
          tempo_stampa = tempo_stampa + (millis() - split);
          sprintf(buf, "%dh %dm ", (millis() - tempo_stampa) / 3600000, ((millis() - tempo_stampa) % 3600000) / 60000);
          print.show();
          delayS(30);
          tempopr.setText(buf);
          bprestm.setValue(0);
          break;
      }
      if (s > limiter_milling) {
        pwm_mot(limiter_milling);
      } else {
        pwm_mot(s);
      }
      line++;
    }
    if (line >= line_max - 2) {
      task = 2;
      while (1) {
        vTaskDelay(pdMS_TO_TICKS(200));
      }
    }
    switch (restore()) {
      case 1:
        task = 0;
        while (1) {
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        break;
    }
    switch (stop_pressed) {
      case 1:
        task = -1;
        while (1) {
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        break;
      case 2:
        last_z = distanza_punta_base - z;
        split = millis();
        task = 3;
        while (task != 1) {
          vTaskDelay(pdMS_TO_TICKS(200));
        }
        break;
    }
    vTaskDelay(1);
  }
}