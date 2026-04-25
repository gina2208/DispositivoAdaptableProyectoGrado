/**
 * @file    baston_v2_final.ino
 * @brief   Firmware principal del dispositivo ORBIX — Bastón Inteligente V2
 *
 * @details Este firmware gestiona la detección de obstáculos mediante fusión
 *          sensorial (VL53L0X ToF + SR04M ultrasónico), la retroalimentación
 *          auditiva por Bluetooth A2DP hacia audífonos de conducción ósea
 *          predestinados, el monitoreo continuo de batería con indicación
 *          visual RGB, y el control de volumen y encendido por botones físicos.
 *
 * @hardware
 *   - Microcontrolador : ESP32-WROOM-32
 *   - Sensor primario  : VL53L0X (ToF, I2C — SDA=GPIO22, SCL=GPIO23)
 *   - Sensor respaldo  : SR04M  (ultrasónico, TRIG=GPIO5, ECHO=GPIO18)
 *   - Audio            : Bluetooth A2DP → audífonos conducción ósea "wireless"
 *   - LED RGB          : Ánodo común (pata larga a 3.3 V; LOW = encendido)
 *   - Batería          : Divisor de tensión en GPIO35 (factor ×2)
 *   - BTN_PWR          : GPIO14 — mantener 2 s para activar/pausar el sistema
 *   - BTN_UP           : GPIO32 — subir volumen
 *   - BTN_DN           : GPIO33 — bajar volumen
 *
 * @authors Gina · Samuel Cuta Barrera
 * @director Ing. Ingrid Ximena Acosta Mesa
 * @institution Universidad de Cundinamarca — GISTFA
 * @version 2.1.0
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include "BluetoothA2DPSource.h"

// ============================================================
//  OBJETOS GLOBALES
// ============================================================
Adafruit_VL53L0X    lox;           ///< Driver sensor VL53L0X
BluetoothA2DPSource a2dp_source;   ///< Fuente de audio Bluetooth A2DP

// ============================================================
//  CONFIGURACIÓN DE AUDÍFONOS PREDESTINADOS
// ============================================================
/**
 * @brief Nombre exacto del dispositivo Bluetooth destino.
 *
 * El sistema intentará conectarse únicamente a este nombre.
 * Si la conexión falla o se pierde, reintentará automáticamente
 * sin mostrar un menú de selección al usuario.
 */
static const char* BT_NOMBRE_DESTINO = "wireless";

/**
 * @brief Tiempo máximo de espera por intento de conexión (ms).
 * Después de este tiempo, el sistema reintenta indefinidamente.
 */
static const unsigned long BT_TIMEOUT_MS     = 15000UL;

/**
 * @brief Pausa entre reintentos de conexión (ms).
 */
static const unsigned long BT_REINTENTO_MS   = 3000UL;

// ============================================================
//  CONTROL DE VOLUMEN
// ============================================================
volatile float volumen  = 0.8f;   ///< Nivel de volumen actual (0.0 – 1.0)
const    float VOL_PASO = 0.1f;   ///< Incremento/decremento por pulsación
const    float VOL_MIN  = 0.0f;   ///< Volumen mínimo
const    float VOL_MAX  = 1.0f;   ///< Volumen máximo

// ============================================================
//  PINES DE HARDWARE
// ============================================================
const int PIN_BAT   = 35;   ///< ADC — lectura de tensión de batería (divisor ×2)
const int LED_R_PIN = 25;   ///< LED RGB — canal rojo   (ánodo común)
const int LED_G_PIN = 26;   ///< LED RGB — canal verde  (ánodo común)
const int LED_B_PIN = 27;   ///< LED RGB — canal azul   (ánodo común)
const int BTN_PWR   = 14;   ///< Botón de encendido/pausa (PULLUP — activo en LOW)
const int BTN_UP    = 32;   ///< Botón subir volumen      (PULLUP — activo en LOW)
const int BTN_DN    = 33;   ///< Botón bajar volumen      (PULLUP — activo en LOW)
const int TRIG_PIN  = 5;    ///< SR04M — pin de disparo (trigger)
const int ECHO_PIN  = 18;   ///< SR04M — pin de eco (echo)

// ============================================================
//  LÍMITES FÍSICOS DE LOS SENSORES
// ============================================================
const int VL53_MAX_MM     = 1200;  ///< Alcance máximo confiable del VL53L0X (mm)
const int SR04_MIN_MM     = 200;   ///< Distancia mínima válida del SR04M (mm)
const int SR04_MAX_MM     = 4000;  ///< Distancia máxima válida del SR04M (mm)
const int SR04_TIMEOUT_US = 30000; ///< Tiempo de espera máximo pulso eco SR04M (µs)

// ============================================================
//  UMBRALES DE ALERTA POR SENSOR
// ============================================================
// Cada sensor tiene umbrales calibrados según su rango operativo.
const int VL53_PELIGRO = 400;   ///< VL53L0X — zona de peligro inmediato (mm)
const int VL53_PRECAU  = 900;   ///< VL53L0X — zona de precaución (mm)
const int SR04_PELIGRO = 600;   ///< SR04M   — zona de peligro inmediato (mm)
const int SR04_PRECAU  = 1500;  ///< SR04M   — zona de precaución (mm)

/// Umbrales activos (se actualizan según el sensor en uso)
volatile int DISTANCIA_PELIGRO = SR04_PELIGRO;
volatile int DISTANCIA_PRECAU  = SR04_PRECAU;

// ============================================================
//  ESTADO DE SENSORES Y REDUNDANCIA
// ============================================================
bool vl53_ok         = false;   ///< Indica si el VL53L0X está operativo
bool usando_respaldo = false;   ///< Indica si se está usando el SR04M como respaldo

unsigned long tRecuperacion  = 0;            ///< Marca de tiempo del último intento de recuperación
const unsigned long RECUP_MS = 5000;         ///< Intervalo entre intentos de recuperación (ms)

/**
 * @return Cadena con el nombre del sensor activo en este ciclo.
 */
const char* fuenteActiva() {
  if (vl53_ok)         return "VL53L0X";
  if (usando_respaldo) return "SR04M";
  return "NINGUNO";
}

/**
 * @brief Actualiza los umbrales de alerta según el sensor activo.
 *
 * Si el VL53L0X está disponible, se usan sus umbrales (más precisos).
 * En caso de fallo, se aplican los umbrales del SR04M (mayor rango).
 */
void actualizarUmbrales() {
  if (vl53_ok) {
    DISTANCIA_PELIGRO = VL53_PELIGRO;
    DISTANCIA_PRECAU  = VL53_PRECAU;
  } else {
    DISTANCIA_PELIGRO = SR04_PELIGRO;
    DISTANCIA_PRECAU  = SR04_PRECAU;
  }
}

// ============================================================
//  MONITOREO DE BATERÍA
// ============================================================
const float UMBRAL_VERDE = 4.00f;  ///< Tensión mínima para estado "carga alta" (V)
const float UMBRAL_AMAR  = 3.60f;  ///< Tensión mínima para estado "carga media" (V)
const float CAL          = 1.000f; ///< Factor de calibración del divisor de tensión
const float ALPHA        = 0.05f;  ///< Coeficiente del filtro EMA de tensión

float vBatFiltrada = 0.0f;  ///< Tensión de batería suavizada por filtro EMA

/**
 * @brief Mide la tensión de batería mediante muestreo estadístico.
 *
 * Adquiere 200 muestras ADC, descarta el 30 % inferior y superior
 * (media truncada al 40 % central) y convierte el resultado a voltios
 * considerando el divisor resistivo ×2.
 *
 * @return Tensión estimada de la batería en voltios.
 */
float leerBateriaPromedio() {
  const int MUESTRAS = 200;
  int valores[MUESTRAS];

  for (int i = 0; i < MUESTRAS; i++) {
    valores[i] = analogRead(PIN_BAT);
    delay(1);
  }

  // Ordenamiento burbuja para media truncada
  for (int i = 0; i < MUESTRAS - 1; i++)
    for (int j = 0; j < MUESTRAS - i - 1; j++)
      if (valores[j] > valores[j+1]) {
        int tmp = valores[j]; valores[j] = valores[j+1]; valores[j+1] = tmp;
      }

  long suma  = 0;
  int inicio = (int)(MUESTRAS * 0.30f);
  int fin    = (int)(MUESTRAS * 0.70f);
  for (int i = inicio; i < fin; i++) suma += valores[i];

  float promedio = suma / (float)(fin - inicio);
  float vAdc     = promedio * (3.3f / 4095.0f);
  return vAdc * 2.0f * CAL;  // Compensación por divisor ×2
}

/**
 * @return Cadena descriptiva del nivel de carga según la tensión.
 */
const char* estadoBateria(float vBat) {
  if (vBat >= UMBRAL_VERDE) return "ALTA";
  if (vBat >= UMBRAL_AMAR)  return "MEDIA";
  return "BAJA";
}

// ============================================================
//  SÍNTESIS DE AUDIO A2DP
// ============================================================
volatile float tono_hz       = 0.0f;   ///< Frecuencia del tono activo (Hz); 0 = silencio
volatile int   patron_on_ms  = 0;      ///< Duración del pulso activo del patrón (ms)
volatile int   patron_off_ms = 0;      ///< Duración del silencio del patrón (ms)
static   float fase          = 0.0f;   ///< Fase acumulada del oscilador sinusoidal
static   bool  audio_on      = true;   ///< Estado momentáneo ON/OFF dentro del patrón
const    int   SAMPLE_RATE   = 44100;  ///< Frecuencia de muestreo de audio (Hz)

/**
 * @brief Callback A2DP: genera muestras de audio PCM en tiempo real.
 *
 * Implementa un oscilador sinusoidal con modulación de amplitud basada
 * en un patrón temporal (on_ms / off_ms) para codificar las zonas de alerta.
 * La función es invocada por la pila Bluetooth en su propio contexto.
 *
 * @param frame       Buffer de tramas estéreo a rellenar.
 * @param frame_count Número de tramas solicitadas.
 * @return            Número de tramas efectivamente escritas.
 */
int32_t get_sound_data(Frame* frame, int32_t frame_count) {
  unsigned long now = millis();

  // Determinar si el oscilador está en fase ON o fase OFF del patrón
  if (patron_on_ms > 0) {
    unsigned long ciclo   = patron_on_ms + patron_off_ms;
    unsigned long t_ciclo = now % ciclo;
    audio_on = (t_ciclo < (unsigned long)patron_on_ms);
  } else {
    audio_on = false;
  }

  float hz         = tono_hz;
  float delta_fase = (hz > 0.0f && audio_on)
                     ? (2.0f * M_PI * hz / SAMPLE_RATE) : 0.0f;

  for (int i = 0; i < frame_count; i++) {
    int16_t sample = 0;
    if (hz > 0.0f && audio_on) {
      sample  = (int16_t)(sinf(fase) * 20000.0f * volumen);
      fase   += delta_fase;
      if (fase > 2.0f * M_PI) fase -= 2.0f * M_PI;  // Envolver fase para evitar overflow
    }
    frame[i].channel1 = sample;
    frame[i].channel2 = sample;
  }
  return frame_count;
}

// ============================================================
//  GESTIÓN DE ALERTAS SONORAS
// ============================================================
/**
 * @brief Configura el tono y patrón de alerta según la distancia medida.
 *
 * Zonas de alerta (adaptadas al sensor activo):
 *  - Sin obstáculo (≥ DISTANCIA_PRECAU) : silencio
 *  - Precaución    (< DISTANCIA_PRECAU) : 800 Hz, patrón lento (200/600 ms)
 *  - Peligro       (< DISTANCIA_PELIGRO): 1200 Hz, patrón rápido (100/80 ms)
 *
 * @param distancia    Distancia medida en milímetros.
 * @param hayObstaculo Indica si la distancia cae dentro de alguna zona activa.
 */
void setAlerta(int distancia, bool hayObstaculo) {
  if (!hayObstaculo) {
    // Sin obstáculo detectado — silencio total
    tono_hz = 0; patron_on_ms = 0; patron_off_ms = 0;
    return;
  }

  if (distancia < DISTANCIA_PELIGRO) {
    // Zona de PELIGRO: tono agudo, ritmo rápido
    tono_hz       = 1200.0f;
    patron_on_ms  = 100;
    patron_off_ms = 80;
    Serial.printf("  [ZONA] PELIGRO    | %d mm | umbral < %d mm\n",
                  distancia, DISTANCIA_PELIGRO);
  } else {
    // Zona de PRECAUCIÓN: tono medio, ritmo pausado
    tono_hz       = 800.0f;
    patron_on_ms  = 200;
    patron_off_ms = 600;
    Serial.printf("  [ZONA] PRECAUCION | %d mm | umbral < %d mm\n",
                  distancia, DISTANCIA_PRECAU);
  }
}

// ============================================================
//  LED RGB — ÁNODO COMÚN
// ============================================================
// Con ánodo común, LOW activa el canal y HIGH lo apaga.

/**
 * @brief Establece el color del LED RGB de forma directa.
 *
 * @param v  true → encender verde  (batería alta)
 * @param a  true → encender amarillo (rojo+verde, batería media)
 * @param r  true → encender rojo   (batería baja)
 */
void setLeds(bool v, bool a, bool r) {
  if (r) {
    // ROJO — batería baja
    digitalWrite(LED_R_PIN, LOW);
    digitalWrite(LED_G_PIN, HIGH);
    digitalWrite(LED_B_PIN, HIGH);
  } else if (a) {
    // AMARILLO — batería media (mezcla rojo + verde)
    digitalWrite(LED_R_PIN, LOW);
    digitalWrite(LED_G_PIN, LOW);
    digitalWrite(LED_B_PIN, HIGH);
  } else if (v) {
    // VERDE — batería alta
    digitalWrite(LED_R_PIN, HIGH);
    digitalWrite(LED_G_PIN, LOW);
    digitalWrite(LED_B_PIN, HIGH);
  } else {
    // APAGADO
    digitalWrite(LED_R_PIN, HIGH);
    digitalWrite(LED_G_PIN, HIGH);
    digitalWrite(LED_B_PIN, HIGH);
  }
}

/**
 * @brief Actualiza el LED según la tensión de batería con filtro EMA.
 *
 * Aplica un filtro de media exponencial móvil (EMA) para evitar
 * parpadeos ante variaciones rápidas de tensión por carga del sistema.
 *
 * @param vBat Tensión de batería medida (V).
 */
void actualizarLedsPorBateria(float vBat) {
  if (vBatFiltrada == 0.0f) vBatFiltrada = vBat;
  vBatFiltrada = vBatFiltrada * (1.0f - ALPHA) + vBat * ALPHA;

  if      (vBatFiltrada >= UMBRAL_VERDE) setLeds(true,  false, false);
  else if (vBatFiltrada >= UMBRAL_AMAR)  setLeds(false, true,  false);
  else                                   setLeds(false, false, true);
}

/**
 * @brief Parpadeo rojo no bloqueante: indica fallo en ambos sensores.
 *
 * Alterna el LED rojo cada 300 ms usando temporización relativa
 * (no bloquea el loop principal).
 */
void ledErrorSensores() {
  static unsigned long tBlink = 0;
  static bool ledOn = false;
  if (millis() - tBlink >= 300) {
    tBlink = millis();
    ledOn  = !ledOn;
    digitalWrite(LED_R_PIN, ledOn ? LOW : HIGH);
    digitalWrite(LED_G_PIN, HIGH);
    digitalWrite(LED_B_PIN, HIGH);
  }
}

// ============================================================
//  LECTURA DE SENSORES
// ============================================================

/**
 * @brief Lee la distancia del sensor VL53L0X.
 *
 * Devuelve -1 si el sensor no está activo, reporta error de rango
 * (RangeStatus == 4) o la lectura está fuera del límite físico.
 *
 * @return Distancia en mm, o -1 en caso de lectura inválida.
 */
int leerVL53() {
  if (!vl53_ok) return -1;
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  if (measure.RangeStatus == 4) return -1;           // Estado de error del sensor
  int d = measure.RangeMilliMeter;
  if (d <= 0 || d > VL53_MAX_MM) return -1;          // Fuera de rango confiable
  return d;
}

/**
 * @brief Lee la distancia del sensor SR04M mediante pulso ultrasónico.
 *
 * Calcula la distancia como: d = (duración_eco × velocidad_sonido) / 2.
 * Usa una velocidad del sonido de 343 m/s a temperatura ambiente (~20 °C).
 *
 * @return Distancia en mm, o -1 en caso de timeout o valor fuera de rango.
 */
int leerSR04() {
  // Generar pulso de disparo de 10 µs
  digitalWrite(TRIG_PIN, LOW);  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, SR04_TIMEOUT_US);
  if (dur == 0) return -1;                            // Timeout: sin eco recibido

  int d = (int)((dur * 343L) / 2000);                // Conversión µs → mm
  if (d < SR04_MIN_MM || d > SR04_MAX_MM) return -1; // Descarte de valores inválidos
  return d;
}

/**
 * @brief Fusiona las lecturas de ambos sensores con prioridad al VL53L0X.
 *
 * Estrategia de fusión:
 *  1. Se intenta leer el VL53L0X (sensor primario, mayor precisión).
 *  2. Si falla, se recurre al SR04M (sensor de respaldo, mayor alcance).
 *  3. Si ambos fallan, devuelve 9999 mm y no activa ninguna alerta.
 *
 * @param[out] hayObstaculo true si la distancia cae dentro de la zona de precaución.
 * @return Distancia fusionada en mm (9999 indica ausencia de lectura válida).
 */
int leerDistanciaFusion(bool &hayObstaculo) {
  hayObstaculo  = false;
  int distancia = 9999;
  int d_vl53    = leerVL53();

  if (d_vl53 > 0) {
    // Lectura válida del sensor primario
    usando_respaldo = false;
    distancia       = d_vl53;
  } else {
    // Fallo en sensor primario: conmutar a respaldo
    if (vl53_ok) Serial.println("[SENSOR] VL53L0X sin lectura, usando SR04M");
    int d_sr04 = leerSR04();
    if (d_sr04 > 0) { usando_respaldo = true;  distancia = d_sr04; }
    else            { usando_respaldo = false; return 9999; }
  }

  if (distancia < DISTANCIA_PRECAU) hayObstaculo = true;
  return distancia;
}

/**
 * @brief Intenta inicializar el sensor VL53L0X sobre el bus I2C.
 *
 * Reinicia el bus I2C antes de cada intento para recuperarse de estados
 * de bloqueo. Si todos los intentos fallan, el sistema continúa con SR04M.
 *
 * @param maxIntentos Número de reintentos antes de declarar fallo.
 * @return true si la inicialización fue exitosa.
 */
bool iniciarVL53(int maxIntentos = 1) {
  Wire.end(); delay(100);
  Wire.begin(22, 23); delay(200);   // SDA=GPIO22, SCL=GPIO23

  for (int i = 0; i < maxIntentos; i++) {
    if (lox.begin()) {
      Serial.println("[VL53L0X] Inicialización correcta");
      return true;
    }
    Serial.printf("[VL53L0X] Intento %d/%d fallido — reintentando\n", i+1, maxIntentos);
    Wire.end(); delay(100);
    Wire.begin(22, 23); delay(200);
  }
  Serial.println("[VL53L0X] No detectado — sistema operará con SR04M");
  return false;
}

// ============================================================
//  CONEXIÓN BLUETOOTH PREDESTINADA
// ============================================================

/**
 * @brief Conecta al dispositivo Bluetooth predefinido con reintentos automáticos.
 *
 * A diferencia del modo de selección manual, esta función busca directamente
 * el dispositivo cuyo nombre coincide con BT_NOMBRE_DESTINO. Reintenta
 * indefinidamente hasta lograr la conexión, con una pausa de BT_REINTENTO_MS
 * entre intentos fallidos.
 *
 * No requiere intervención del usuario. El LED azul parpadea durante la búsqueda.
 */
void conectarAudiofonosPredestinados() {
  Serial.println();
  Serial.println("========================================");
  Serial.printf("  Conectando a: \"%s\"\n", BT_NOMBRE_DESTINO);
  Serial.println("  Asegúrese de que los audífonos estén");
  Serial.println("  encendidos y en modo de emparejamiento.");
  Serial.println("========================================");

  int intento = 0;

  while (true) {
    intento++;
    Serial.printf("  [BT] Intento %d...\n", intento);

    // Indicación visual: destello azul durante la conexión
    digitalWrite(LED_B_PIN, LOW); delay(200); digitalWrite(LED_B_PIN, HIGH);

    a2dp_source.start(BT_NOMBRE_DESTINO, get_sound_data);

    unsigned long tInicio = millis();
    while (!a2dp_source.is_connected() && millis() - tInicio < BT_TIMEOUT_MS) {
      // Espera activa con parpadeo de indicación
      Serial.print(".");
      digitalWrite(LED_B_PIN, LOW);  delay(300);
      digitalWrite(LED_B_PIN, HIGH); delay(200);
    }
    Serial.println();

    if (a2dp_source.is_connected()) {
      Serial.println("  [BT] ¡Conexión establecida con los audífonos!");
      // Destello azul prolongado como confirmación de conexión exitosa
      for (int i = 0; i < 3; i++) {
        digitalWrite(LED_B_PIN, LOW);  delay(150);
        digitalWrite(LED_B_PIN, HIGH); delay(100);
      }
      return;  // Salir al obtener conexión exitosa
    }

    // Fallo en este intento: liberar recursos y reintentar
    Serial.printf("  [BT] Sin respuesta. Reintentando en %.1f s...\n",
                  BT_REINTENTO_MS / 1000.0f);
    a2dp_source.end();
    delay(BT_REINTENTO_MS);
  }
}

// ============================================================
//  MANEJO DE BOTONES (ANTIRREBOTE + ACCIONES)
// ============================================================
const unsigned long DEBOUNCE_MS  = 35;    ///< Tiempo de estabilización antirrebote (ms)
const unsigned long PWR_HOLD_MS  = 2000;  ///< Tiempo de pulsación larga para encender/pausar (ms)

// Variables de estado para antirrebote por botón
bool pwrStable = HIGH, upStable = HIGH, dnStable = HIGH;
bool pwrLast   = HIGH, upLast   = HIGH, dnLast   = HIGH;
unsigned long pwrTC = 0, upTC = 0, dnTC = 0;

unsigned long pwrDownAt    = 0;      ///< Marca de tiempo del flanco descendente del BTN_PWR
bool          pwrLongFired = false;  ///< Bandera: pulsación larga ya procesada
bool          sistemaActivo = false; ///< Estado global del sistema (activo/pausado)

/**
 * @brief Actualiza el estado de un botón con antirrebote por tiempo mínimo.
 *
 * @param pin    GPIO del botón a leer.
 * @param stable Estado estable actual (modificado si hay transición válida).
 * @param last   Estado previo estable.
 * @param tc     Marca de tiempo del último cambio detectado.
 * @return true si ocurrió una transición estable (flanco válido).
 */
bool debounceRead(int pin, bool &stable, bool &last, unsigned long &tc) {
  bool raw = digitalRead(pin);
  unsigned long now = millis();
  if (raw != stable) {
    if (now - tc >= DEBOUNCE_MS) { last = stable; stable = raw; tc = now; return true; }
  } else {
    tc = now;
  }
  return false;
}

/**
 * @brief Acción al pulsar BTN_UP: incrementa el volumen un paso.
 * Emite un destello azul como confirmación visual.
 */
void accionVolUp() {
  volumen = min(volumen + VOL_PASO, VOL_MAX);
  Serial.printf("[VOL] ▲  %d%%\n", (int)(volumen * 100));
  digitalWrite(LED_B_PIN, LOW);  delay(80);
  digitalWrite(LED_B_PIN, HIGH);
}

/**
 * @brief Acción al pulsar BTN_DN: decrementa el volumen un paso.
 * Emite un destello rojo como confirmación visual.
 */
void accionVolDown() {
  volumen = max(volumen - VOL_PASO, VOL_MIN);
  Serial.printf("[VOL] ▼  %d%%\n", (int)(volumen * 100));
  digitalWrite(LED_R_PIN, LOW);  delay(80);
  digitalWrite(LED_R_PIN, HIGH);
}

/**
 * @brief Procesa todos los botones en cada ciclo del loop principal.
 *
 * - BTN_PWR (mantener 2 s): alterna entre sistema activo y pausado.
 * - BTN_UP  (pulso corto) : sube volumen.
 * - BTN_DN  (pulso corto) : baja volumen.
 */
void handleButtons() {
  unsigned long now = millis();

  // --- BTN_PWR: detección de pulsación larga ---
  if (debounceRead(BTN_PWR, pwrStable, pwrLast, pwrTC)) {
    if (pwrStable == LOW) { pwrDownAt = now; pwrLongFired = false; }
  }
  if (pwrStable == LOW && !pwrLongFired && now - pwrDownAt >= PWR_HOLD_MS) {
    pwrLongFired  = true;
    sistemaActivo = !sistemaActivo;
    if (!sistemaActivo) {
      // Pausar: silencio y LED apagado
      tono_hz = 0; patron_on_ms = 0; patron_off_ms = 0;
      setLeds(false, false, false);
      Serial.println("[PWR] Sistema PAUSADO");
    } else {
      Serial.println("[PWR] Sistema ACTIVO");
    }
  }

  // --- BTN_UP: flanco descendente = subir volumen ---
  if (debounceRead(BTN_UP, upStable, upLast, upTC))
    if (upStable == LOW) accionVolUp();

  // --- BTN_DN: flanco descendente = bajar volumen ---
  if (debounceRead(BTN_DN, dnStable, dnLast, dnTC))
    if (dnStable == LOW) accionVolDown();
}

// ============================================================
//  TEMPORIZACIÓN DEL LOG SERIAL
// ============================================================
unsigned long tPrint = 0;
const unsigned long PRINT_MS = 2000;  ///< Intervalo de reporte por consola serial (ms)

// ============================================================
//  SETUP — INICIALIZACIÓN DEL SISTEMA
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("       ORBIX — Bastón Inteligente V2   ");
  Serial.println("========================================");

  // --- Configuración de pines ---
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);
  pinMode(BTN_PWR,   INPUT_PULLUP);
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DN,    INPUT_PULLUP);
  pinMode(TRIG_PIN,  OUTPUT);
  pinMode(ECHO_PIN,  INPUT);

  // --- Prueba visual del LED RGB: secuencia rojo → verde → azul → blanco → apagado ---
  digitalWrite(LED_R_PIN, LOW);  digitalWrite(LED_G_PIN, HIGH); digitalWrite(LED_B_PIN, HIGH); delay(300);
  digitalWrite(LED_R_PIN, HIGH); digitalWrite(LED_G_PIN, LOW);  digitalWrite(LED_B_PIN, HIGH); delay(300);
  digitalWrite(LED_R_PIN, HIGH); digitalWrite(LED_G_PIN, HIGH); digitalWrite(LED_B_PIN, LOW);  delay(300);
  digitalWrite(LED_R_PIN, LOW);  digitalWrite(LED_G_PIN, LOW);  digitalWrite(LED_B_PIN, LOW);  delay(300);
  digitalWrite(LED_R_PIN, HIGH); digitalWrite(LED_G_PIN, HIGH); digitalWrite(LED_B_PIN, HIGH);

  // --- Secuencia de encendido: mantener BTN_PWR durante PWR_HOLD_MS ---
  Serial.println("\n  Mantenga BTN_PWR para encender el dispositivo...");
  while (true) {
    if (digitalRead(BTN_PWR) == LOW) {
      delay(PWR_HOLD_MS);
      if (digitalRead(BTN_PWR) == LOW) {
        Serial.println("  Encendiendo sistema...");
        // Destello blanco de confirmación
        digitalWrite(LED_R_PIN, LOW); digitalWrite(LED_G_PIN, LOW); digitalWrite(LED_B_PIN, LOW);
        delay(300);
        digitalWrite(LED_R_PIN, HIGH); digitalWrite(LED_G_PIN, HIGH); digitalWrite(LED_B_PIN, HIGH);
        sistemaActivo = true;
        break;
      }
    }
    delay(50);
  }

  // --- Paso 1: Conectar a los audífonos predestinados ---
  conectarAudiofonosPredestinados();

  // --- Paso 2: Inicializar sensor VL53L0X (con un intento) ---
  Serial.println("[VL53L0X] Iniciando sensor ToF...");
  vl53_ok = iniciarVL53(1);
  actualizarUmbrales();

  // Verificar sensor de respaldo si el primario no está disponible
  if (!vl53_ok) {
    int test = leerSR04();
    if (test > 0) Serial.printf("[SR04M] Operativo. Lectura inicial: %d mm\n", test);
    else          Serial.println("[AVISO] SR04M sin respuesta en la prueba inicial.");
  }

  // Lectura inicial de batería para el filtro EMA
  vBatFiltrada = leerBateriaPromedio();

  // --- Resumen de inicio ---
  Serial.println();
  Serial.println("[OK] Dispositivo ORBIX listo.");
  Serial.printf("     Audífonos  : %s\n",     BT_NOMBRE_DESTINO);
  Serial.printf("     Sensor     : %s\n",     vl53_ok ? "VL53L0X (primario)" : "SR04M (respaldo)");
  Serial.printf("     Peligro    : < %d mm\n", DISTANCIA_PELIGRO);
  Serial.printf("     Precaución : < %d mm\n", DISTANCIA_PRECAU);
  Serial.printf("     Volumen    : %d%%\n",    (int)(volumen * 100));
  Serial.printf("     Batería    : %.2f V\n",  vBatFiltrada);
  Serial.println("========================================");
}

// ============================================================
//  LOOP — CICLO PRINCIPAL DE OPERACIÓN
// ============================================================
void loop() {
  // Actualizar umbrales por si cambió el sensor activo
  actualizarUmbrales();

  // Procesar eventos de botones
  handleButtons();

  // Sistema pausado: no realizar detección ni alertas
  if (!sistemaActivo) { delay(50); return; }

  // --- Reconexión automática si el enlace Bluetooth se pierde ---
  if (!a2dp_source.is_connected()) {
    Serial.println("[BT] Enlace perdido. Reconectando a los audífonos...");
    a2dp_source.end();
    delay(300);
    conectarAudiofonosPredestinados();
  }

  // --- Recuperación periódica del VL53L0X si falló ---
  if (!vl53_ok && millis() - tRecuperacion >= RECUP_MS) {
    tRecuperacion = millis();
    vl53_ok       = iniciarVL53(1);
    if (vl53_ok) {
      Serial.println("[VL53L0X] Sensor recuperado exitosamente.");
      usando_respaldo = false;
      actualizarUmbrales();
    }
  }

  // --- Lectura fusionada de distancia ---
  bool hayObstaculo = false;
  int  distancia    = leerDistanciaFusion(hayObstaculo);

  // --- Gestión de alerta sonora o indicación de error ---
  if (!vl53_ok && !usando_respaldo) {
    // Ambos sensores inoperativos: silencio y parpadeo de error
    tono_hz = 0; patron_on_ms = 0; patron_off_ms = 0;
    ledErrorSensores();
  } else {
    setAlerta(distancia, hayObstaculo);
  }

  // --- Reporte periódico de telemetría por consola serial ---
  if (millis() - tPrint >= PRINT_MS) {
    tPrint = millis();
    float vBat = leerBateriaPromedio();
    if (vl53_ok || usando_respaldo) actualizarLedsPorBateria(vBat);
    Serial.printf("[BAT] %.2fV  filtrado: %.2fV  [%s] | Sensor: %s | Vol: %d%%\n",
                  vBat, vBatFiltrada, estadoBateria(vBatFiltrada),
                  fuenteActiva(), (int)(volumen * 100));
  }

  delay(50);  // Cadencia del loop: ~20 ciclos/segundo
}
