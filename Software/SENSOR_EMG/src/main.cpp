#include <stdio.h>                   // Librería estándar para entrada/salida
#include <math.h>                    // Librería para funciones matemáticas 
#include <freertos/FreeRTOS.h>       // Librería base de FreeRTOS
#include <freertos/task.h>           // Funciones para crear y manejar tareas en FreeRTOS
#include <esp_timer.h>               // Temporizadores de alta resolución en ESP-IDF
#include <esp_adc/adc_oneshot.h>     // Lectura ADC en modo one-shot
#include <esp_adc/adc_cali.h>        // Calibración ADC
#include <esp_adc/adc_cali_scheme.h> // Esquema de calibración ADC
#include <esp_log.h>                 // Log para depuración
#include <soc/soc_caps.h>            // Capacidades del chip

#include "EMGFilters.h" // Filtros personalizados para señal EMG

#define ADC_GPIO 35                       // Pin GPIO que usará el ADC
#define SAMPLE_FREQ_HZ SAMPLE_FREQ_1000HZ // Frecuencia de muestreo: 1 kHz
#define HUM_FREQ NOTCH_FREQ_60HZ          // Filtro notch para 60 Hz (ruido de red eléctrica)
#define RMS_WINDOW 125                    // Ventana de cálculo RMS (~125 ms a 1 kHz)

int pulso = 0;                                // Estado de detección de pulso
int valor_digital_contraccion = 0;            // Variable binaria: 1 si hay contracción, 0 si no
adc_oneshot_unit_handle_t adc_oneshot_handle; // Manejador para ADC en modo one-shot
adc_cali_handle_t adc_cali_handle;            // Manejador para calibración ADC
EMGFilters emgFilter;                         // Objeto para aplicar filtros EMG

// Variables para RMS filtrado
double sum_sq_filt = 0; // Suma de cuadrados para cálculo de RMS
int count_filt = 0;     // Contador de muestras para RMS

// Offset y ruido base calculados en calibración
int offset_mv = 0;       // Desplazamiento DC (mV)
double ruido_base = 0.0; // Desviación estándar de señal en reposo

// Umbral para detectar contracción
double umbral = 0.0; // Valor límite para detección

// Inicializa la calibración ADC
void init_adc_calibration()
{
  adc_unit_t adc_unit;
  adc_channel_t adc_channel;
  // Convierte GPIO a unidad y canal ADC
  adc_oneshot_io_to_channel(ADC_GPIO, &adc_unit, &adc_channel);

  // Configuración de calibración por ajuste lineal
  adc_cali_line_fitting_config_t adc_cali_line_fitting_config = {
      .unit_id = adc_unit,
      .atten = ADC_ATTEN_DB_12,    // Atenuación de 12 dB (mediciones hasta ~3.6 V)
      .bitwidth = ADC_BITWIDTH_12, // Resolución de 12 bits
  };
  // Crea el manejador de calibración
  adc_cali_create_scheme_line_fitting(&adc_cali_line_fitting_config, &adc_cali_handle);
}

// Convierte valor RAW a voltaje calibrado (mV)
static int calibrate_adc(int raw)
{
  int voltaje = 0;
  adc_cali_raw_to_voltage(adc_cali_handle, raw, &voltaje);
  return voltaje;
}

// Inicializa el ADC en modo one-shot
void init_adc_oneshot()
{
  adc_unit_t adc_unit;
  adc_channel_t adc_channel;
  adc_oneshot_io_to_channel(ADC_GPIO, &adc_unit, &adc_channel);

  // Configuración de la unidad ADC
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT_1,
      .ulp_mode = ADC_ULP_MODE_DISABLE};
  adc_oneshot_new_unit(&init_config, &adc_oneshot_handle);

  // Configuración de canal
  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_12};
  adc_oneshot_config_channel(adc_oneshot_handle, adc_channel, &config);
}

// Calibra offset y ruido base durante 5 segundos (músculo en reposo)
void calibrar_offset_y_ruido()
{
  printf("Calibrando offset y ruido durante 5 segundos, mantén el músculo relajado...\n");

  adc_unit_t adc_unit;
  adc_channel_t adc_channel;
  adc_oneshot_io_to_channel(ADC_GPIO, &adc_unit, &adc_channel);

  int64_t start_time = esp_timer_get_time(); // Tiempo inicial (µs)
  int64_t elapsed = 0;

  int64_t sum = 0;    // Suma de valores
  int64_t sum_sq = 0; // Suma de cuadrados
  int count = 0;      // Contador de muestras
  int raw = 0;        // Valor leído ADC

  // Lee datos durante 5 segundos
  while (elapsed < 5000000)
  {
    adc_oneshot_read(adc_oneshot_handle, adc_channel, &raw);
    int mv = calibrate_adc(raw);         // Convierte a mV
    sum += mv;                           // Suma valores
    sum_sq += (int64_t)mv * (int64_t)mv; // Suma de cuadrados
    count++;
    vTaskDelay(pdMS_TO_TICKS(1)); // 1 ms de retardo (1 kHz)
    elapsed = esp_timer_get_time() - start_time;
  }

  offset_mv = (int)(sum / count); // Promedio = offset DC

  // Varianza = E[x^2] - (E[x])^2
  double varianza = ((double)sum_sq / count) - ((double)offset_mv * (double)offset_mv);
  ruido_base = sqrt(varianza); // Desviación estándar

  // Umbral para contracción = 1.5 veces el ruido base
  umbral = 1.5 * ruido_base;

  // Mostrar resultados de calibración
  printf("Offset calculado: %d mV\n", offset_mv);
  printf("Ruido base (std dev): %.2f mV\n", ruido_base);
  printf("Umbral para contracción: %.2f mV\n", umbral);
}

// Callback de temporizador para lectura ADC y procesamiento EMG
static void adc_timer_callback(void *arg)
{
  int raw;
  adc_unit_t adc_unit;
  adc_channel_t adc_channel;
  adc_oneshot_io_to_channel(ADC_GPIO, &adc_unit, &adc_channel);

  // Leer ADC
  adc_oneshot_read(adc_oneshot_handle, adc_channel, &raw);

  int mv = calibrate_adc(raw); // Conversión a mV

  // Quitar offset antes de filtrar
  int mv_no_offset = mv - offset_mv;

  // Aplicar filtros EMG
  int filtered = emgFilter.update(mv_no_offset);

  // Acumular cuadrados para RMS
  sum_sq_filt += (double)filtered * (double)filtered;
  count_filt++;

  printf(">raw:%i\n", raw); // Imprime valor crudo

  // Cuando se completa la ventana RMS
  if (count_filt >= RMS_WINDOW)
  {
    double rms_filt = sqrt(sum_sq_filt / RMS_WINDOW); // RMS filtrado

    // Si RMS supera umbral, marcar inicio de pulso
    if (rms_filt >= umbral)
    {
      pulso = 1;
    }

    // Confirmar contracción si RMS es alto
    if (pulso && rms_filt >= 5.00)
    {
      valor_digital_contraccion = 1;
      printf("Contracción detectada! RMS filtrado: %.2f mV, Umbral: %.2f mV\n", rms_filt, umbral);
    }
    else
    {
      valor_digital_contraccion = 0;
      printf("Reposo. RMS filtrado: %.2f mV, Umbral: %.2f mV\n", rms_filt, umbral);
      pulso = 0;
    }

    // Reiniciar acumuladores
    sum_sq_filt = 0;
    count_filt = 0;
  }
}

// Función principal de la app
extern "C" void app_main(void)
{
  // Inicializar filtros EMG
  emgFilter.init(SAMPLE_FREQ_HZ, HUM_FREQ, true, true, true);

  // Inicializar ADC
  init_adc_oneshot();
  init_adc_calibration();

  // Calibrar offset y ruido al inicio
  calibrar_offset_y_ruido();

  // Configuración del temporizador periódico para muestreo
  const esp_timer_create_args_t periodic_timer_args = {
      .callback = &adc_timer_callback,
      .name = "adc_timer"};
  esp_timer_handle_t periodic_timer;
  esp_timer_create(&periodic_timer_args, &periodic_timer);

  // Inicia temporizador: 1000 µs = 1 ms => 1 kHz
  esp_timer_start_periodic(periodic_timer, 1000);
}
