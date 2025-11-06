/*
Não consegui fazer o alarme funcionar, pois novamente tive problemas com as portas USB do meu laptop, mas gostei de fazer de fazer o código e mexer com os componentes.
  === Atividade 3 – Medidor de Distância com HC-SR04 e LCD I2C ===
  Compatível com ESP32 core 3.x
  Inclui alarme por buzzer (ativo ou passivo)
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ----------------- PINOS -----------------
const int PIN_TRIG = 5;
const int PIN_ECHO = 18;
const int PIN_BUZZ = 19;

// Padrão do ESP32 DevKit V1
const int I2C_SDA = 21;
const int I2C_SCL = 22;

// ----------------- LCD -----------------
LiquidCrystal_I2C lcd(0x27, 16, 2);   // troque para 0x3F se não aparecer nada

// ----------------- CONFIGURAÇÕES -----------------
const bool BUZZER_ATIVO = true;   // true = buzzer ativo (liga/desliga)
                                  // false = buzzer passivo (gera tom PWM)

const float LIMIAR_CM = 20.0;     // ativa o alarme abaixo desse valor
const float MIN_VALID_CM = 2.0;
const float MAX_VALID_CM = 400.0;

const unsigned long BEEP_SLOW_MS = 600;  // pisca lento
const unsigned long BEEP_FAST_MS = 120;  // pisca rápido

const int N_SAMPLES = 5;                 // média móvel

// ----------------- VARIÁVEIS -----------------
unsigned long lastBeepToggle = 0;
bool buzzOn = false;

// ----------------- TONE PARA ESP32 CORE 3.X -----------------
void toneESP32(int pin, uint32_t freq) {
  // anexa PWM com freq e resolução de 8 bits
  ledcAttach(pin, freq, 8);
  ledcWriteTone(0, freq);
}

void noToneESP32(int pin) {
  ledcDetach(pin);
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

// ----------------- FUNÇÕES -----------------
float medirDistanciaCm() {
  float soma = 0;
  int validas = 0;

  for (int i = 0; i < N_SAMPLES; i++) {
    digitalWrite(PIN_TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(PIN_TRIG, LOW);

    unsigned long dur = pulseIn(PIN_ECHO, HIGH, 30000UL);
    if (dur == 0) continue;

    float cm = dur / 58.0;
    if (cm >= MIN_VALID_CM && cm <= MAX_VALID_CM) {
      soma += cm;
      validas++;
    }
    delay(5);
  }

  if (validas == 0) return NAN;
  return soma / validas;
}

unsigned long mapearIntervaloBeep(float cm) {
  if (cm >= LIMIAR_CM) return BEEP_SLOW_MS;
  float cmMin = 5.0;
  if (cm <= cmMin) return BEEP_FAST_MS;
  float k = (cm - cmMin) / (LIMIAR_CM - cmMin);
  return (unsigned long)(BEEP_FAST_MS + k * (BEEP_SLOW_MS - BEEP_FAST_MS));
}

void atualizarBuzzer(float cm) {
  if (isnan(cm)) {
    if (BUZZER_ATIVO) digitalWrite(PIN_BUZZ, LOW);
    else noToneESP32(PIN_BUZZ);
    return;
  }

  if (cm >= LIMIAR_CM) {
    if (BUZZER_ATIVO) digitalWrite(PIN_BUZZ, LOW);
    else noToneESP32(PIN_BUZZ);
    buzzOn = false;
    return;
  }

  unsigned long intervalo = mapearIntervaloBeep(cm);
  if (millis() - lastBeepToggle >= intervalo) {
    lastBeepToggle = millis();
    buzzOn = !buzzOn;

    if (BUZZER_ATIVO) {
      digitalWrite(PIN_BUZZ, buzzOn ? HIGH : LOW);
    } else {
      if (buzzOn) toneESP32(PIN_BUZZ, 3000);
      else noToneESP32(PIN_BUZZ);
    }
  }
}

// ----------------- SETUP -----------------
void setup() {
  Wire.begin(I2C_SDA, I2C_SCL);
  lcd.init();
  lcd.backlight();

  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  digitalWrite(PIN_BUZZ, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Sensor iniciado");
  delay(1000);
  lcd.clear();
}

// ----------------- LOOP -----------------
void loop() {
  float cm = medirDistanciaCm();

  lcd.setCursor(0, 0);
  if (isnan(cm)) {
    lcd.print("Dist: --- cm    ");
  } else {
    char linha[17];
    snprintf(linha, sizeof(linha), "Dist:%6.1f cm", cm);
    lcd.print(linha);
    for (int i = strlen(linha); i < 16; i++) lcd.print(' ');
  }

  lcd.setCursor(0, 1);
  if (!isnan(cm) && cm < LIMIAR_CM) lcd.print("Alarme: ATIVO   ");
  else lcd.print("Alarme: OFF     ");

  atualizarBuzzer(cm);
  delay(50);
}
