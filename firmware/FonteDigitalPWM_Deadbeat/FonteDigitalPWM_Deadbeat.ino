// Fonte digital 0 a 10 V com controle deadbeat (ESP32 na placa PAT-DAELN v2.1)
//
// GPIO18: PWM de 2 kHz e 8 bits para o modulo LC-LM358-PWM2V, que gera a saida de 0 a 10 V.
// I2C (GPIO21/22): ADS1015 (0x48), que le a saida por um divisor de 2,7 k / 1,2 k, e OLED SSD1306 (0x3C).
//
// Modelo de 1a ordem discretizado: a = exp(-Ts/tau), b = K(1 - a), com K = 1.
// Lei deadbeat: u[k] = (r - a y[k]) / b, mais um trim integrador com anti-windup
// que corrige o erro de regime.
//
// Ciclo de Ts = 250 ms: 10 leituras do ADS1015 a cada 25 ms, media, decisao e espera.
// O degrau e considerado estabilizado apos 4 ciclos seguidos com erro de ate 0,025 V
// sem troca de sinal; a partir dai o comando fica congelado ate o proximo setpoint.
//
// Serial a 115200 baud:
//   numero de 0 a 10  -> novo setpoint em volts
//   T=valor           -> tau em ms
//   STATUS            -> parametros atuais
//   RUN               -> reinicia a sequencia automatica 0, 1, 0, 2 ... 0, 10 V (10 s por patamar),
//                        que tambem comeca sozinha cerca de 2 s apos ligar
//   STOP              -> interrompe a sequencia
// Saida em CSV, uma linha por ciclo: t_ms,passo,SP,y,u,bias,estavel
//
// OLED: titulo (ou AUTO n/20), setpoint, tempo de estabelecimento do ultimo degrau,
// valor medido em destaque e barra de 0 a 10 V com marcador do setpoint.

#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_ADS1015 ads;
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

// Saida PWM
#define PWM_PIN   18
#define PWM_FREQ  2000
#define PWM_RES   8

// Divisor do ADS1015: 1,2 / (2,7 + 1,2)
#define RATIO_ADS  0.3077f

// Transistores auxiliares da placa, mantidos desligados
#define TSEG1 27
#define TSEG2 14

// Barra de progresso no OLED
#define BAR_X       4
#define BAR_Y       42
#define BAR_W       120
#define BAR_H       10
#define BAR_MAXV    10.0f

// Modelo da planta e periodo de amostragem
float tau_ms = 30.0f;
float Ts_ms  = 250.0f;
float Kgain  = 1.0f;
float a_param, b_param;

float setpoint = 0.0f;
float yMedido  = 0.0f;
float uAtual   = 0.0f;

// Trim integrador e deteccao de regime
float biasIntegral = 0.0f;
bool  jaEstabilizou = false;
unsigned long tInicioDegrau = 0;
long teUltimo_ms = -1;
int   ciclosEstaveis = 0;
#define TOL_ESTAVEL      0.025f
#define CICLOS_P_ESTAVEL 4
float erroAnterior = 0.0f;
#define KI_TRIM       0.15f
#define BIAS_LIMITE   2.0f
#define ERRO_TRIM_MAX 0.5f
#define ERRO_DEADZONE 0.02f

String uartBuf = "";
bool   entradaInvalida = false;

// Sequencia automatica de degraus, sempre voltando a 0 V entre eles
struct DegrauStep { float valor; unsigned long duracao_ms; };
DegrauStep sequenciaDegraus[] = {
  {0.0f,10000},{1.0f,10000},
  {0.0f,10000},{2.0f,10000},
  {0.0f,10000},{3.0f,10000},
  {0.0f,10000},{4.0f,10000},
  {0.0f,10000},{5.0f,10000},
  {0.0f,10000},{6.0f,10000},
  {0.0f,10000},{7.0f,10000},
  {0.0f,10000},{8.0f,10000},
  {0.0f,10000},{9.0f,10000},
  {0.0f,10000},{10.0f,10000}
};
const int NUM_DEGRAUS = sizeof(sequenciaDegraus) / sizeof(sequenciaDegraus[0]);

bool autoRodando = false;
int  autoIndice  = 0;
unsigned long autoTStep     = 0;
unsigned long autoTInicioAbs = 0;

void iniciaAuto() {
  autoRodando   = true;
  autoIndice    = 0;
  autoTStep     = millis();
  autoTInicioAbs = millis();
  setpoint      = sequenciaDegraus[0].valor;
  reiniciaCiclo();
  Serial.println("=== INICIANDO SEQUENCIA AUTOMATICA DE DEGRAUS (RUN) ===");
  Serial.print("Total de passos: "); Serial.println(NUM_DEGRAUS);
  Serial.print("Passo 1/"); Serial.print(NUM_DEGRAUS);
  Serial.print(" -> SP="); Serial.print(sequenciaDegraus[0].valor, 2);
  Serial.print("V  duracao_ms="); Serial.println(sequenciaDegraus[0].duracao_ms);
  Serial.println("Log CSV continuo (formato: t_ms,passo,SP,y,u,bias,estavel)");
  Serial.println("t_ms,passo,SP,y,u,bias,estavel");
}

void atualizaAuto() {
  if (!autoRodando) return;
  unsigned long ag = millis();
  if (ag - autoTStep >= sequenciaDegraus[autoIndice].duracao_ms) {
    autoIndice++;
    if (autoIndice >= NUM_DEGRAUS) {
      autoRodando = false;
      setpoint = 0.0f;
      reiniciaCiclo();
      Serial.println("=== SEQUENCIA AUTOMATICA CONCLUIDA ===");
      return;
    }
    autoTStep = ag;
    setpoint  = sequenciaDegraus[autoIndice].valor;
    reiniciaCiclo();
    Serial.print("Passo "); Serial.print(autoIndice + 1); Serial.print("/"); Serial.print(NUM_DEGRAUS);
    Serial.print(" -> SP="); Serial.print(sequenciaDegraus[autoIndice].valor, 2);
    Serial.print("V  duracao_ms="); Serial.println(sequenciaDegraus[autoIndice].duracao_ms);
  }
}

// Maquina de estados do ciclo de controle
enum EstadoCtrl { ST_LEITURA, ST_MEDIA, ST_DECISAO, ST_ESPERA };
EstadoCtrl estado = ST_LEITURA;

#define NUM_AMOSTRAS_ADS     10
#define INTERVALO_ADS_MS     25
int   amostraCount  = 0;
float somaAmostras  = 0.0f;
unsigned long tAmostra     = 0;
unsigned long tCicloInicio = 0;

// Reinicia a janela de leitura e zera o trim a cada troca de setpoint
void reiniciaCiclo() {
  amostraCount = 0;
  somaAmostras = 0.0f;
  tAmostra     = millis();
  tCicloInicio = millis();
  tInicioDegrau = millis();
  estado       = ST_LEITURA;
  biasIntegral = 0.0f;
  jaEstabilizou = false;
  ciclosEstaveis = 0;
  erroAnterior = 0.0f;
  teUltimo_ms = -1;
}

void apagaTransistores() {
  digitalWrite(TSEG1, LOW);
  digitalWrite(TSEG2, LOW);
}

void oledCtr(int y, const char* s) {
  oledCtrSize(y, s, 1);
}

void oledCtrSize(int y, const char* s, int textSize) {
  int charW = 6 * textSize;
  int x = (128 - (int)strlen(s) * charW) / 2;
  if (x < 0) x = 0;
  oled.setCursor(x, y);
  oled.print(s);
}

// Coeficientes do modelo discreto
void recalculaModelo() {
  a_param = exp(-Ts_ms / tau_ms);
  b_param = Kgain * (1.0f - a_param);
}

// Converte 0 a 10 V em duty de 0 a 255
void aplicaComando(float v) {
  if (v < 0)  v = 0;
  if (v > 10) v = 10;
  int pwmVal = (int)((v / 10.0f) * 255.0f + 0.5f);
  if (pwmVal < 0)   pwmVal = 0;
  if (pwmVal > 255) pwmVal = 255;
  ledcWrite(PWM_PIN, pwmVal);
}

// Leitura do ADS1015 convertida para a tensao de saida
float lerADS() {
  int16_t raw  = ads.readADC_SingleEnded(0);
  float   vDiv = ads.computeVolts(raw);
  return vDiv / RATIO_ADS;
}

void desenhaBarra(float valorY, float valorSP) {
  oled.drawRect(BAR_X, BAR_Y, BAR_W, BAR_H, SSD1306_WHITE);

  float frac = valorY / BAR_MAXV;
  if (frac < 0) frac = 0;
  if (frac > 1) frac = 1;
  int fillW = (int)(frac * (BAR_W - 2) + 0.5f);
  if (fillW > 0) {
    oled.fillRect(BAR_X + 1, BAR_Y + 1, fillW, BAR_H - 2, SSD1306_WHITE);
  }

  float fracSP = valorSP / BAR_MAXV;
  if (fracSP < 0) fracSP = 0;
  if (fracSP > 1) fracSP = 1;
  int xSP = BAR_X + (int)(fracSP * (BAR_W - 2) + 0.5f);
  oled.drawLine(xSP, BAR_Y - 2, xSP, BAR_Y + BAR_H + 2, SSD1306_WHITE);

  oled.setCursor(BAR_X, BAR_Y + BAR_H + 4);
  oled.print("0V");
  char bufMax[6];
  sprintf(bufMax, "%dV", (int)BAR_MAXV);
  int wMax = strlen(bufMax) * 6;
  oled.setCursor(BAR_X + BAR_W - wMax, BAR_Y + BAR_H + 4);
  oled.print(bufMax);
}

// Tela do OLED
void mostraOLED() {
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);

  if (autoRodando) {
    char titulo[20];
    sprintf(titulo, "AUTO %d/%d", autoIndice + 1, NUM_DEGRAUS);
    oledCtr(0, titulo);
  } else {
    oledCtr(0, "FONTE DIGITAL PWM");
  }
  oled.drawLine(0, 10, 127, 10, SSD1306_WHITE);

  char bufSP[10];
  sprintf(bufSP, "SP:%.2fV", setpoint);
  oled.setCursor(0, 13);
  oled.print(bufSP);

  char bufTe[14];
  if (teUltimo_ms < 0) {
    sprintf(bufTe, "Te:--");
  } else {
    sprintf(bufTe, "Te:%ldms", teUltimo_ms);
  }
  int wTe = (int)strlen(bufTe) * 6;
  oled.setCursor(128 - wTe, 13);
  oled.print(bufTe);

  oled.setTextSize(2);
  char buf[12];
  sprintf(buf, "%.2fV", yMedido);
  oledCtrSize(23, buf, 2);
  oled.setTextSize(1);

  desenhaBarra(yMedido, setpoint);

  oled.display();
  apagaTransistores();
}

// Linha de log em CSV
void logCSV(int estavelFlag) {
  Serial.print(millis());        Serial.print(",");
  Serial.print(autoRodando ? (autoIndice + 1) : 0); Serial.print(",");
  Serial.print(setpoint, 3);     Serial.print(",");
  Serial.print(yMedido, 4);      Serial.print(",");
  Serial.print(uAtual, 4);       Serial.print(",");
  Serial.print(biasIntegral, 4); Serial.print(",");
  Serial.println(estavelFlag);
}

void mostraStatus() {
  Serial.println("---- STATUS DO MODELO ----");
  Serial.print("tau = "); Serial.print(tau_ms); Serial.println(" ms");
  Serial.print("Ts  = "); Serial.print(Ts_ms);  Serial.println(" ms");
  Serial.print("K   = "); Serial.println(Kgain);
  Serial.print("a   = "); Serial.println(a_param, 4);
  Serial.print("b   = "); Serial.println(b_param, 4);
  Serial.print("RATIO_ADS = "); Serial.println(RATIO_ADS, 4);
  Serial.print("PWM_PIN   = "); Serial.println(PWM_PIN);
  Serial.print("bias trim = "); Serial.println(biasIntegral, 4);
  Serial.println("---------------------------");
}

// Comandos pela serial
void processaUART() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (uartBuf.length() > 0) {
        uartBuf.trim();
        String cmd = uartBuf;
        cmd.toUpperCase();

        if (cmd == "STATUS") {
          mostraStatus();
        }
        else if (cmd == "RUN") {
          iniciaAuto();
        }
        else if (cmd == "STOP") {
          autoRodando = false;
          Serial.println("Sequencia automatica interrompida manualmente.");
        }
        else if (cmd.startsWith("T=")) {
          float novoTau = uartBuf.substring(2).toFloat();
          if (novoTau > 1.0f) {
            tau_ms = novoTau;
            recalculaModelo();
            Serial.print("Novo tau = "); Serial.print(tau_ms); Serial.println(" ms");
            mostraStatus();
          } else {
            Serial.println("Tau invalido (deve ser > 1 ms).");
          }
        }
        else {
          bool valido = true, temDigito = false;
          for (unsigned int i = 0; i < uartBuf.length(); i++) {
            char ch = uartBuf[i];
            if (isDigit(ch)) temDigito = true;
            else if (ch != '.' && ch != '-') { valido = false; break; }
          }
          if (!temDigito) valido = false;

          float valor = uartBuf.toFloat();
          if (valido && valor >= 0.0f && valor <= 10.0f) {
            autoRodando = false;
            setpoint = valor;
            entradaInvalida = false;
            reiniciaCiclo();
            Serial.print("Novo setpoint: "); Serial.print(setpoint, 2); Serial.println(" V");
          } else {
            entradaInvalida = true;
            Serial.print("ENTRADA INVALIDA: '"); Serial.print(uartBuf);
            Serial.println("' -- numero 0-10, 'T=valor', 'RUN', 'STOP' ou 'STATUS'. Setpoint mantido.");
          }
        }
      }
      uartBuf = "";
    } else {
      uartBuf += c;
    }
  }
}

// Inicializacao; a sequencia automatica comeca cerca de 2 s depois
void setup() {

  pinMode(TSEG1, OUTPUT); digitalWrite(TSEG1, LOW);
  pinMode(TSEG2, OUTPUT); digitalWrite(TSEG2, LOW);

  ledcAttach(PWM_PIN, PWM_FREQ, PWM_RES);
  aplicaComando(0.0f);

  Serial.begin(115200);

  Wire.begin(21, 22);

  if (!ads.begin(0x48)) {
    Serial.println("ADS1015 nao encontrado! Verifique wiring I2C e endereco.");
    while (true);
  }
  ads.setGain(GAIN_ONE);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED nao encontrado!"); while (true);
  }

  recalculaModelo();
  mostraOLED();

  tCicloInicio = millis();
  tAmostra     = millis();
  estado       = ST_LEITURA;

  Serial.println("FONTE DIGITAL PWM - Controle Deadbeat (ADS1015) pronto.");
  Serial.println("Comandos: numero 0-10 | T=valor_ms | RUN (sequencia 0-1-0-2..0-10) | STOP | STATUS");
  Serial.println("FSM: 10 leituras ADS1015 (25ms/leitura) -> decisao a cada 250ms");
  Serial.println("Pinout confirmado: PWM=GPIO18, RATIO_ADS=0.3077, K real~1.01");
  Serial.println("Log CSV continuo (formato: t_ms,passo,SP,y,u,bias,estavel)");
  Serial.println("t_ms,passo,SP,y,u,bias,estavel");
  mostraStatus();

  delay(2000);
  iniciaAuto();
}

// Ciclo de controle
void loop() {
  processaUART();
  atualizaAuto();

  unsigned long ag = millis();

  switch (estado) {

    // 10 leituras, uma a cada 25 ms
    case ST_LEITURA:
      if (ag - tAmostra >= (unsigned long)INTERVALO_ADS_MS) {
        tAmostra = ag;
        somaAmostras += lerADS();
        amostraCount++;
        if (amostraCount >= NUM_AMOSTRAS_ADS) {
          estado = ST_MEDIA;
        }
      }
      break;

    case ST_MEDIA: {
      yMedido = somaAmostras / (float)NUM_AMOSTRAS_ADS;
      estado = ST_DECISAO;
      break;
    }

    // Lei deadbeat, deteccao de regime e trim
    case ST_DECISAO: {
      if (jaEstabilizou) {
        mostraOLED();
        logCSV(1);
        estado = ST_ESPERA;
        break;
      }

      float erro = setpoint - yMedido;

      float uCalc = (setpoint - a_param * yMedido) / b_param + biasIntegral;
      uAtual = uCalc;
      if (uAtual < 0)  uAtual = 0;
      if (uAtual > 10) uAtual = 10;
      bool saturado = (uCalc != uAtual);

      aplicaComando(uAtual);
      mostraOLED();

      bool inverteuSinal = (erro > 0 && erroAnterior < 0) || (erro < 0 && erroAnterior > 0);

      if (fabs(erro) <= TOL_ESTAVEL && !inverteuSinal) {
        ciclosEstaveis++;
      } else {
        ciclosEstaveis = 0;
      }

      if (!jaEstabilizou && ciclosEstaveis >= CICLOS_P_ESTAVEL) {
        jaEstabilizou = true;
        unsigned long tAbs = millis();
        teUltimo_ms = (long)(tAbs - tInicioDegrau);
      }

      logCSV(jaEstabilizou ? 1 : 0);

      // O trim so integra fora da zona morta, sem saturacao e sem troca de sinal do erro
      if (!saturado && !inverteuSinal && fabs(erro) < ERRO_TRIM_MAX && fabs(erro) > ERRO_DEADZONE) {
        biasIntegral += KI_TRIM * erro;
        if (biasIntegral >  BIAS_LIMITE) biasIntegral =  BIAS_LIMITE;
        if (biasIntegral < -BIAS_LIMITE) biasIntegral = -BIAS_LIMITE;
      }
      erroAnterior = erro;

      estado = ST_ESPERA;
      break;
    }

    // Aguarda completar os 250 ms
    case ST_ESPERA:
      if (ag - tCicloInicio >= (unsigned long)Ts_ms) {
        tCicloInicio = ag;
        tAmostra     = ag;
        amostraCount = 0;
        somaAmostras = 0.0f;
        estado       = ST_LEITURA;
      }
      break;
  }
}
