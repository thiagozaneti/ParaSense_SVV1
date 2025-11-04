/*****************************************************************************************
 *  Medição de vibração com ESP32 + MPU9250
 *  ------------------------------------------------------------------
 *  - Lê o acelerômetro do MPU9250 usando FIFO a 1 kHz (amostragem uniforme).
 *  - Imprime dados "RAW" (brutos) dos eixos X, Y, Z (em m/s²), decimados ~200 Hz.
 *  - A cada janela de 1024 amostras (~1,024 s), calcula e imprime:
 *      * ACC_RMS (aceleração RMS AC, m/s²)   [HPF 1 Hz remove gravidade/DC]
 *      * VEL_RMS (velocidade RMS 10–180 Hz, mm/s) [HPF 10 Hz + LPF 180 Hz + integração]
 *
 *  Formatos na Serial:
 *    RAW,ax_mps2,ay_mps2,az_mps2
 *    METRICS,ACC_RMS_mps2,VEL_RMS_10-180Hz_mms
 *
 *  Ajustes principais:
 *    - FS_HZ (taxa de amostragem real via FIFO): 1000 Hz
 *    - N (tamanho da janela para métricas/filtragem/integração): 1024
 *    - RAW_DECIM (decimação de impressão do RAW): 5 -> ~200 Hz na Serial
 *    - Cortes dos filtros:
 *        HPF para ACC_RMS: 1 Hz (remove gravidade/bias)
 *        HPF para VEL_RMS: 10 Hz (banda ISO-like: 10–180 Hz)
 *        LPF para VEL_RMS: 180 Hz (casando com DLPF ~184 Hz do sensor)
 *
 *  ⚠️ Importante:
 *    - Para medir vibração “de verdade”, a amostragem precisa ser uniforme.
 *      Por isso usamos a FIFO do MPU9250 e lemos em blocos.
 *    - A DLPF do acelerômetro (no chip) está em ~184 Hz: atua como anti-aliasing interno,
 *      coerente com Fs=1000 Hz (Nyquist=500 Hz).
 *****************************************************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <string.h>   // para memcpy

/********** Pinos de hardware **********/
#define SDA_PIN        21   // I2C SDA no ESP32
#define SCL_PIN        22   // I2C SCL no ESP32
#define BUZZER_PIN     15   // Buzzer de confirmação (opcional)

/********** PWM do buzzer (LEDC do ESP32) **********/
#define LEDC_CH        0    // canal PWM
#define BEEP_FREQ      800  // Hz do beep
#define BEEP_MS        400  // duração do beep (ms)

/********** Parâmetros de sinal **********/
static const int   FS_HZ = 1000;         // Taxa real de amostragem (via FIFO), 1000 amostras/seg
static const float DT    = 1.0f / FS_HZ; // Período de amostragem (s) = 0,001 s
static const int   N     = 1024;         // Tamanho da janela (~1,024 s a 1 kHz)

static const int   RAW_DECIM = 5;        // Imprimir 1 a cada 5 amostras -> ~200 linhas/seg

/********** Registradores (endereços) do MPU9250 **********/
#define MPU_ADDR        0x68  // Endereço I2C (AD0=GND). Se AD0=VCC, seria 0x69.
#define WHO_AM_I        0x75  // Deve retornar 0x71 para MPU9250
#define PWR_MGMT_1      0x6B
#define PWR_MGMT_2      0x6C
#define CONFIG_REG      0x1A
#define SMPLRT_DIV      0x19
#define ACCEL_CONFIG    0x1C
#define ACCEL_CONFIG2   0x1D
#define FIFO_EN_REG     0x23
#define I2C_MST_CTRL    0x24
#define USER_CTRL       0x6A
#define FIFO_R_W        0x74
#define FIFO_COUNTH     0x72
#define FIFO_COUNTL     0x73
#define INT_ENABLE      0x38
#define ACCEL_XOUT_H    0x3B

/********** Escalas/fatores do acelerômetro **********/
#define ACCEL_SENS_2G   16384.0f  // LSB por g na escala ±2 g
#define G_9_81          9.80665f  // 1 g em m/s²

/********** Buffers de uma janela (dados brutos em m/s²) **********/
static float axb[N], ayb[N], azb[N];

/*============================= Utilitários I2C =============================*
 * Funções auxiliares para escrever e ler registradores via I2C.
 * Usadas para configurar o MPU e ler dados e FIFO.
 *==========================================================================*/

// Escreve 1 byte (val) em (reg) no endereço do MPU
static uint8_t i2cWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission(); // 0 = OK
}

// Lê 'n' bytes a partir de um registrador 'reg' do MPU
static bool i2cReadN(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  // repeated start: não libera o bus entre write(reg) e requestFrom
  if (Wire.endTransmission(false) != 0) return false;
  int got = Wire.requestFrom((int)MPU_ADDR, (int)n);
  if (got != (int)n) return false;
  for (size_t i = 0; i < n; ++i) buf[i] = Wire.read();
  return true;
}

// Lê o contador de bytes na FIFO (16 bits)
static uint16_t fifoCount() {
  uint8_t h=0, l=0;
  if (!i2cReadN(FIFO_COUNTH, &h, 1)) return 0;
  if (!i2cReadN(FIFO_COUNTL, &l, 1)) return 0;
  return ((uint16_t)h << 8) | l;
}

// Reseta a FIFO (descarta conteúdo e reabilita)
static void fifoReset() {
  // USER_CTRL bit2 = FIFO_RESET
  i2cWrite8(USER_CTRL, 0x04);
  delayMicroseconds(50);
  // USER_CTRL bit6 = FIFO_EN
  i2cWrite8(USER_CTRL, 0x40);
}

/*============================= Buzzer helper ===============================*
 * Toca um beep simples para indicar que inicializou OK.
 *==========================================================================*/
static void beep(uint16_t ms = BEEP_MS, uint32_t freq = BEEP_FREQ) {
  ledcWriteTone(LEDC_CH, freq);
  delay(ms);
  ledcWriteTone(LEDC_CH, 0);
}

/*============================= Setup do MPU ================================*
 * Configura o acelerômetro, DLPF e FIFO para amostragem uniforme a 1 kHz.
 *
 * Decisões:
 *   - ACCEL ±2 g: máxima resolução (16384 LSB/g) — bom para vibração.
 *   - DLPF accel ~184 Hz: anti-aliasing interno coerente com Fs=1 kHz.
 *   - SMPLRT_DIV=0: base 1 kHz com DLPF ligado.
 *   - FIFO: habilita apenas acelerômetro (6 bytes por amostra).
 *==========================================================================*/
static bool setupMPU() {
  // Reset do dispositivo
  i2cWrite8(PWR_MGMT_1, 0x80);
  delay(100);

  // Clock PLL (estável) e liga tudo
  i2cWrite8(PWR_MGMT_1, 0x01);
  i2cWrite8(PWR_MGMT_2, 0x00);
  delay(10);

  // DLPF do giroscópio ~184 Hz (ajusta o pipeline interno)
  i2cWrite8(CONFIG_REG,    0x01);

  // Sample Rate Divider = 0 -> 1 kHz (com DLPF ativo)
  i2cWrite8(SMPLRT_DIV,    0x00);

  // Acelerômetro ±2 g (melhor resolução)
  i2cWrite8(ACCEL_CONFIG,  0x00);

  // DLPF do acelerômetro ~184 Hz (A_DLPF_CFG=1)
  i2cWrite8(ACCEL_CONFIG2, 0x01);

  // Desliga mestre I2C interno (não vamos usar magnetômetro aqui)
  i2cWrite8(I2C_MST_CTRL,  0x00);

  // Sem interrupções por enquanto (vamos varrer a FIFO)
  i2cWrite8(INT_ENABLE,    0x00);

  // FIFO_EN: habilita apenas ACCEL (bit3)
  i2cWrite8(FIFO_EN_REG,   0x08);

  // USER_CTRL: habilita FIFO_EN (bit6) e reseta FIFO (bit2)
  i2cWrite8(USER_CTRL,     0x44);
  delay(10);
  fifoReset();
  return true;
}

/*============================= Leitura da FIFO =============================*
 * Lê até 'maxSamples' amostras do acelerômetro a partir da FIFO.
 * Cada amostra ACCEL tem 6 bytes: XH XL YH YL ZH ZL.
 * Converte os valores crus (LSB) para m/s² e escreve nos vetores ax, ay, az.
 *
 * Retorno: número de amostras lidas (0..maxSamples).
 *==========================================================================*/
static int readFifoAccel(float *ax, float *ay, float *az, int maxSamples) {
  uint16_t bytes = fifoCount();
  int samplesAvail = bytes / 6;          // 6 bytes por amostra (XYZ)
  int toRead = samplesAvail;
  if (toRead > maxSamples) toRead = maxSamples;
  if (toRead <= 0) return 0;

  for (int i = 0; i < toRead; i++) {
    uint8_t b[6];
    if (!i2cReadN(FIFO_R_W, b, 6)) {
      // Se algo falhar, reseta FIFO para não "desalinhar" leitura
      fifoReset();
      return i; // até onde deu para ler
    }

    // Reconstrói valores 16 bits com sinal
    int16_t rx = (int16_t)((b[0] << 8) | b[1]);
    int16_t ry = (int16_t)((b[2] << 8) | b[3]);
    int16_t rz = (int16_t)((b[4] << 8) | b[5]);

    // Converte de LSB -> g -> m/s²
    ax[i] = (rx / ACCEL_SENS_2G) * G_9_81;
    ay[i] = (ry / ACCEL_SENS_2G) * G_9_81;
    az[i] = (rz / ACCEL_SENS_2G) * G_9_81;
  }
  return toRead;
}

/*============================= Filtros (1ª ordem) ==========================*
 * Implementações simples e leves de HPF (passa-altas) e LPF (passa-baixas).
 * - HPF: remove componente DC/baixa freq (gravidade e bias).
 * - LPF: limita banda superior (anti-alias/ruído).
 *
 * Nota: 1ª ordem é um bom compromisso simplicidade x custo CPU.
 *       Se precisar de respostas mais “quadradas”, podemos trocar por biquads.
 *==========================================================================*/

// HPF de 1ª ordem para 3 eixos simultaneamente
struct HPF1 {
  float alpha;                // α = RC/(RC + dt), RC = 1/(2πfc)
  float y_prev[3] = {0,0,0};  // saídas anteriores (um por eixo)
  float x_prev[3] = {0,0,0};  // entradas anteriores (um por eixo)

  void config(float fc, float dt) {
    float RC = 1.0f / (2.0f * PI * fc);
    alpha = RC / (RC + dt);
  }

  inline void apply3(float &x, float &y, float &z) {
    // Eixo X
    float yx = alpha*(y_prev[0] + x - x_prev[0]);
    x_prev[0] = x; y_prev[0] = yx; x = yx;

    // Eixo Y
    float yy = alpha*(y_prev[1] + y - x_prev[1]);
    x_prev[1] = y; y_prev[1] = yy; y = yy;

    // Eixo Z
    float yz = alpha*(y_prev[2] + z - x_prev[2]);
    x_prev[2] = z; y_prev[2] = yz; z = yz;
  }
};

// LPF de 1ª ordem para 3 eixos simultaneamente
struct LPF1 {
  float a = 0.0f;             // ganho incremental (0..1), calculado via fc e dt
  float y[3] = {0,0,0};       // estado interno (um por eixo)
  bool init = false;          // primeiro passo: inicializa saída com a entrada

  void config(float fc, float dt) {
    float RC = 1.0f / (2.0f * PI * fc);
    a = dt / (RC + dt);       // forma padrão de 1ª ordem (filtro exponencial)
  }

  inline void apply3(float &x, float &y_, float &z) {
    if (!init) { y[0]=x; y[1]=y_; y[2]=z; init=true; }
    y[0] = y[0] + a*(x  - y[0]); x  = y[0];
    y[1] = y[1] + a*(y_ - y[1]); y_ = y[1];
    y[2] = y[2] + a*(z  - y[2]); z  = y[2];
  }
};

/*============================= Métricas de vibração ========================*
 * rms3_axes:
 *   - Calcula RMS “vetorial” somando energia de X, Y, Z por amostra.
 *   - Útil para um valor único representando a energia total de vibração.
 *
 * integrate_trap_3:
 *   - Integra aceleração -> velocidade (método do trapézio) por eixo.
 *   - Pré-condição: faça HPF (10 Hz) + LPF (180 Hz) antes para evitar drift e ruído.
 *==========================================================================*/

// RMS vetorial (sqrt(mean(x^2 + y^2 + z^2)))
static float rms3_axes(const float *x, const float *y, const float *z, int n) {
  double acc = 0.0;
  for (int i=0;i<n;i++) {
    double s = (double)x[i]*(double)x[i]
             + (double)y[i]*(double)y[i]
             + (double)z[i]*(double)z[i];
    acc += s;
  }
  return sqrt(acc / n);
}

// Integração por trapézio (aceleração -> velocidade) para 3 eixos
static void integrate_trap_3(const float *ax, const float *ay, const float *az,
                             float *vx, const float * /*vy_in*/, float *vz,
                             int n, float dt) {

  // Começa velocidade em zero (assumindo janela curta com remoção de DC)
  float vy0 = 0.0f; // placeholder
  vx[0]=0; /*vy[0]=0;*/ vz[0]=0;
  // Se quiser guardar vy como vetor, adapte assinatura e crie o buffer.

  // Aqui integramos X e Z; para Y, ver nota acima.
  for (int i=1;i<n;i++) {
    // X
    vx[i] = vx[i-1] + 0.5f*(ax[i-1] + ax[i]) * dt;
    // Y — se precisar, crie um buffer vy[] à semelhança de vx e vz
    vy0   = vy0     + 0.5f*(ay[i-1] + ay[i]) * dt; // mantido local
    // Z
    vz[i] = vz[i-1] + 0.5f*(az[i-1] + az[i]) * dt;
  }

  // Observação:
  // Mantivemos a versão reduzida para economizar RAM. Se quiser a velocidade
  // RMS vetorial “correta” ao longo do tempo, crie vx[], vy[], vz[] e use
  // rms3_axes(vx, vy, vz, N). No loop principal abaixo, já fazemos isso.
}

/*============================= SETUP =======================================*/
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== Vibração (MPU9250 @1kHz, FIFO) — RAW + RMS ===");

  // Inicializa I2C nos pinos definidos, em 400 kHz (rápido)
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Configura PWM (LEDC) do buzzer
  ledcSetup(LEDC_CH, 2000, 10);
  ledcAttachPin(BUZZER_PIN, LEDC_CH);

  // Checa identidade do sensor
  uint8_t who=0xFF;
  if (i2cReadN(WHO_AM_I, &who, 1)) {
    Serial.printf("WHO_AM_I = 0x%02X (esperado 0x71)\n", who);
  } else {
    Serial.println("Falha ao ler WHO_AM_I.");
  }

  // Configura o MPU9250 (accel + DLPF + FIFO)
  setupMPU();
  Serial.println("MPU: ACCEL ±2g, DLPF~184Hz, ODR=1kHz, FIFO=ACCEL.");
  beep(); // feedback sonoro de ok

  // Cabeçalhos de saída na Serial
  Serial.println("Formato RAW (~200 Hz): RAW,ax_mps2,ay_mps2,az_mps2");
  Serial.println("Formato METRICS (~1 s): METRICS,ACC_RMS_mps2,VEL_RMS_10-180Hz_mms");
}

/*============================= LOOP =======================================*
 * Passos por ciclo (~1 s):
 *   1) Enche a janela N com amostras da FIFO (1 kHz).
 *   2) Enquanto lê, imprime RAW decimado (um a cada 5 = ~200 Hz).
 *   3) Copia janela para buffers de trabalho e aplica:
 *        - ACC_RMS: HPF 1 Hz (remove gravidade/bias), RMS vetorial.
 *        - VEL_RMS: HPF 10 Hz + LPF 180 Hz, integra (trapézio), RMS vetorial.
 *   4) Imprime linha unique de métricas "METRICS,..."
 *==========================================================================*/
void loop() {
  /*------------------ 1) Encher janela N com a FIFO ------------------*/
  int filled = 0;   // amostras já preenchidas
  int rawDec = 0;   // contador para decimação de impressão do RAW
  while (filled < N) {
    // Lê até (N - filled) amostras disponíveis
    int got = readFifoAccel(&axb[filled], &ayb[filled], &azb[filled], N - filled);
    if (got > 0) {
      // 2) Imprime RAW decimado: uma a cada RAW_DECIM leituras
      for (int i=0; i<got; ++i) {
        if ((rawDec++ % RAW_DECIM) == 0) {
          // RAW em m/s² (3 eixos)
          Serial.printf("RAW,%.6f,%.6f,%.6f\n", axb[filled+i], ayb[filled+i], azb[filled+i]);
        }
      }
      filled += got;
    } else {
      // Se FIFO ainda não tem dados suficientes, espera 1 ms e tenta de novo
      delay(1);
    }
  }

  /*------------------ 3) Filtragem e métricas ------------------*/
  // Fazemos cópias para aplicar filtros sem perder o RAW armazenado
  static float ax1[N], ay1[N], az1[N]; // para ACC_RMS (HPF 1 Hz)
  static float ax2[N], ay2[N], az2[N]; // para VEL_RMS (HPF 10 + LPF 180 + integração)
  memcpy(ax1, axb, sizeof(ax1));
  memcpy(ay1, ayb, sizeof(ay1));
  memcpy(az1, azb, sizeof(az1));
  memcpy(ax2, axb, sizeof(ax2));
  memcpy(ay2, ayb, sizeof(ay2));
  memcpy(az2, azb, sizeof(az2));

  /*---- ACC_RMS (m/s²) com HPF 1 Hz ----*
   * Remove gravidade (9,81 m/s²) e possíveis biases DC.
   * Resultado: componente AC de aceleração, típica para detectar impactos/alta freq.
   */
  HPF1 hpf1;
  hpf1.config(1.0f, DT); // fc = 1 Hz
  for (int i=0;i<N;i++) {
    float x=ax1[i], y=ay1[i], z=az1[i];
    hpf1.apply3(x,y,z);
    ax1[i]=x; ay1[i]=y; az1[i]=z;
  }
  float acc_rms = rms3_axes(ax1, ay1, az1, N); // RMS vetorial (m/s²)

  /*---- VEL_RMS (mm/s) na banda 10–180 Hz ----*
   * Pipeline:
   *   1) HPF 10 Hz → remove baixa freq. (desbalanceamento muito lento/gravidade).
   *   2) LPF 180 Hz → limita banda alta (coerente com DLPF~184 Hz).
   *   3) Integra aceleração -> velocidade (trapézio), por eixo.
   *   4) RMS vetorial da velocidade → converte para mm/s.
   *
   * Observação: Integração SEMPRE após HPF para reduzir drift.
   */
  HPF1 hpf10;  hpf10.config(10.0f, DT);   // 10 Hz
  for (int i=0;i<N;i++) {
    float x=ax2[i], y=ay2[i], z=az2[i];
    hpf10.apply3(x,y,z);
    ax2[i]=x; ay2[i]=y; az2[i]=z;
  }

  LPF1 lpf180; lpf180.config(180.0f, DT); // 180 Hz
  for (int i=0;i<N;i++) {
    float x=ax2[i], y=ay2[i], z=az2[i];
    lpf180.apply3(x,y,z);
    ax2[i]=x; ay2[i]=y; az2[i]=z;
  }

  // Buffers de velocidade (m/s) — aqui usamos 3 vetores completos
  static float vx[N], vy[N], vz[N];
  // Integração por trapézio (versão completa para 3 eixos)
  vx[0]=vy[0]=vz[0]=0;
  for (int i=1;i<N;i++) {
    vx[i] = vx[i-1] + 0.5f*(ax2[i-1] + ax2[i]) * DT;
    vy[i] = vy[i-1] + 0.5f*(ay2[i-1] + ay2[i]) * DT;
    vz[i] = vz[i-1] + 0.5f*(az2[i-1] + az2[i]) * DT;
  }

  // Velocidade RMS vetorial (m/s) -> converte para mm/s
  float vel_rms_m_s  = rms3_axes(vx, vy, vz, N);
  float vel_rms_mm_s = vel_rms_m_s * 1000.0f;

  /*------------------ 4) Imprime as métricas ------------------*/
  Serial.printf("METRICS,%.6f,%.6f\n", acc_rms, vel_rms_mm_s);
}

/*============================= Notas de uso ================================*
 * - Se os valores de VEL_RMS “derraparem” (drift), aumente o corte do HPF
 *   (ex.: 15 Hz) ou use biquad 2ª ordem (posso te passar os coeficientes).
 *
 * - Se quiser banda maior, troque:
 *     * DLPF do ACCEL (ACCEL_CONFIG2) para 0x00 (~460 Hz)
 *     * E aumente a taxa (Fs >= 2 kHz) — exigiria outra abordagem de leitura.
 *
 * - RAW_DECIM controla só a frequência de IMPRESSÃO. A aquisição continua 1 kHz.
 *
 * - Montagem mecânica muda TUDO: fixe o módulo rigidamente ao ponto medido.
 *   Fita dupla face amortece e “mata” alta frequência.
 *==========================================================================*/
