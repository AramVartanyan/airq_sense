/*
//Air Quality sensor + Temperature and Humidity sensor
 */

#include <stdio.h>
#include <espressif/esp_wifi.h>
#include <espressif/esp_sta.h>
#include <espressif/esp_common.h>
#include <esp/uart.h>
#include <esp8266.h>
#include <FreeRTOS.h>
#include <task.h>

#include <homekit/homekit.h>
#include <homekit/characteristics.h>
#include <wifi_config.h>

#include <ccs811.h>

#include <adv_button.h>

#define LED_GPIO      2   //D4
#ifndef BUTTON_GPIO
#define BUTTON_GPIO   0   //D3
#endif

#define I2C_SCL_PIN   5   //D1
#define I2C_SDA_PIN   4   //D2
#define NINT_PIN      12  //D6

#define I2C_BUS       0
#define I2C_FREQ      I2C_FREQ_100K
#define TASK_STACK_DEPTH 256

#define INT_USED //in case an interrupt notification is used

//#define INT_DATA_RDY_USED //in case an interrupt on data ready is used

// #define INT_THRESHOLD_USED //in case a treshold interrupt is used

#define CO2_TRESHOLD 1100
#define VOC_EXCELLENT 65
#define VOC_GOOD 220
#define VOC_FAIR 660
#define VOC_INFERIOR 2200
#define VOC_POOR 5500

//uint16_t co2_peak = 0;
bool sensor_int = false;
int t_count=0;
uint8_t air_q_old = 0;
uint8_t co2_d_old = 0;
uint16_t tvoc = 0;
uint16_t eco2 = 0;
uint16_t tvoc_calc = 0;
uint16_t eco2_calc = 0;

homekit_value_t air_quality_get ();
homekit_value_t voc_density_get ();
homekit_value_t co2_detected_get ();
homekit_value_t co2_lvl_get ();

static ccs811_sensor_t* sensor;

homekit_characteristic_t air_quality = HOMEKIT_CHARACTERISTIC_(AIR_QUALITY, 0, .getter=air_quality_get);

//homekit_characteristic_t voc_density = HOMEKIT_CHARACTERISTIC_(VOC_DENSITY, 0);
homekit_characteristic_t voc_density = HOMEKIT_CHARACTERISTIC_(VOC_DENSITY, 0, .getter=voc_density_get);

homekit_characteristic_t co2_detected = HOMEKIT_CHARACTERISTIC_(CARBON_DIOXIDE_DETECTED, 0, .getter=co2_detected_get); //uint8_t 0 - normal 1 - abnormal
//homekit_characteristic_t co2_lvl = HOMEKIT_CHARACTERISTIC_(CARBON_DIOXIDE_LEVEL, 0);
homekit_characteristic_t co2_lvl = HOMEKIT_CHARACTERISTIC_(CARBON_DIOXIDE_LEVEL, 0, .getter=co2_lvl_get);
//homekit_characteristic_t co2_peak_lvl = HOMEKIT_CHARACTERISTIC_(CARBON_DIOXIDE_PEAK_LEVEL, 0);

void led_write(bool on) {
    gpio_write(LED_GPIO, on ? 0 : 1);
}

void reset_configuration_task() {
  //Flash the LED before the reset
  for (int i=0; i<3; i++) {
      led_write(true);
      vTaskDelay(100 / portTICK_PERIOD_MS);
      led_write(false);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    printf("Resetting Wifi Config\n");
    wifi_config_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Resetting HomeKit Config\n");
    homekit_server_reset();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    printf("Restarting\n");
    sdk_system_restart();
    vTaskDelete(NULL);
}

void reset_configuration(const uint8_t gpio) {
    printf("Resetting device configuration\n");
    xTaskCreate(reset_configuration_task, "Reset configuration", 256, NULL, 2, NULL);
}

void AQS_task_interrupt (void *_args) {
  while (1){
    if (sensor_int == true) {
      sensor_int = false;
    if (ccs811_get_results (sensor, &tvoc, &eco2, NULL, NULL)) {
    t_count++;
//      printf("%.3f CCS811 Sensor interrupt: TVOC %d ppb, eCO2 %d ppm\n", (double)sdk_system_get_time()*1e-3, tvoc, eco2);
      printf("CCS811 Sensor interrupt: TVOC %d ppb, eCO2 %d ppm\n", tvoc, eco2);
      tvoc_calc = tvoc_calc + tvoc;
      eco2_calc = eco2_calc + eco2;
      }

      if (t_count == 5) {
        t_count = 0;
        tvoc = tvoc_calc / 5;
        eco2 = eco2_calc / 5;

        tvoc_calc = 0;
        eco2_calc = 0;

        if (eco2 >= CO2_TRESHOLD) {
          co2_detected.value.bool_value = 1;
        } else {
          co2_detected.value.bool_value = 0;
        }

    //    if (eco2 > co2_peak) {
    //      co2_peak = eco2;
    //    }

        if (tvoc > VOC_INFERIOR) {
          air_quality.value.int_value = 5;
        } else {
          if (tvoc > VOC_FAIR && tvoc <= VOC_INFERIOR) {
            air_quality.value.int_value = 4;
          } else {
            if (tvoc > VOC_GOOD && tvoc <= VOC_FAIR) {
              air_quality.value.int_value = 3;
            } else {
              if (tvoc > VOC_EXCELLENT && tvoc <= VOC_GOOD){
                air_quality.value.int_value = 2;
              } else {
                if (tvoc >=0 && tvoc <= VOC_EXCELLENT){
                  air_quality.value.int_value = 1;
                } else {
                  air_quality.value.int_value = 0;
                }
              }
            }
          }
        }
      }

      if(air_q_old != air_quality.value.int_value) {
        homekit_characteristic_notify(&air_quality, air_quality.value);
        air_q_old = air_quality.value.int_value;
      }

      if(co2_d_old != co2_detected.value.bool_value) {
        homekit_characteristic_notify(&co2_detected, co2_detected.value);
        co2_d_old = co2_detected.value.bool_value;
      }

      if (voc_density.value.float_value != tvoc) {
        //homekit_characteristic_notify(&voc_density, voc_density.value);
        voc_density.value.float_value = tvoc;
      }

      if (co2_lvl.value.float_value != eco2) {
        //homekit_characteristic_notify(&co2_lvl, co2_lvl.value);
        co2_lvl.value.float_value = eco2;
      }

  //    co2_peak_lvl.value.float_value = co2_peak;
  //    homekit_characteristic_notify(&co2_peak_lvl, co2_peak_lvl.value);
  }
  vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
  //vTaskDelete(NULL);
}

void intr_handler(uint8_t gpio_num) {
  sensor_int = true;
}

void gpio_init() {
    gpio_enable(LED_GPIO, GPIO_OUTPUT);
    led_write(false);
    // init all I2C bus interfaces at which CCS811 sensors are connected
    i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    // longer clock stretching is required for CCS811
    i2c_set_clock_stretch (I2C_BUS, CCS811_I2C_CLOCK_STRETCH);
    // init the sensor with slave address CCS811_I2C_ADDRESS_1 connected I2C_BUS.
    sensor = ccs811_init_sensor (I2C_BUS, CCS811_I2C_ADDRESS_1);

    if (sensor) {
        // activate the interrupt for NINT_PIN and set the interrupt handler
        gpio_enable(NINT_PIN, GPIO_INPUT);
        gpio_set_interrupt(NINT_PIN, GPIO_INTTYPE_EDGE_NEG, intr_handler);
        // start periodic measurement with one measurement per minute
        ccs811_set_mode (sensor, ccs811_mode_60s);
        // If INT_DATA_RDY_USED:
        // enable the data ready interrupt
        ccs811_enable_interrupt (sensor, true);
        // If INT_THRESHOLD_USED:
        // set threshold parameters and enable threshold interrupt mode
        //ccs811_set_eco2_thresholds (sensor, 600, 1100, 40);
        // create a task that is resumed by interrupt handler to use the sensor
    } else {
      printf("Could not initialize the CCS811 sensor\n");
    }
    xTaskCreate(AQS_task_interrupt, "AQS interrupt", TASK_STACK_DEPTH, NULL, 2, NULL);
}

void sensor_identify_task(void *_args) {
  // Identify the device by Flashing it's LED.
  for (int i=0; i<3; i++) {
      for (int j=0; j<2; j++) {
          led_write(true);
          vTaskDelay(100 / portTICK_PERIOD_MS);
          led_write(false);
          vTaskDelay(100 / portTICK_PERIOD_MS);
      }
      vTaskDelay(250 / portTICK_PERIOD_MS);
  }
  led_write(false);
  vTaskDelete(NULL);
}


void sensor_identify(homekit_value_t _value) {
    printf("Sensor identify\n");
    xTaskCreate(sensor_identify_task, "Sensor identify", 128, NULL, 2, NULL);
}

homekit_characteristic_t name = HOMEKIT_CHARACTERISTIC_(NAME, "AQ Sensor");

homekit_accessory_t *accessories[] = {
    HOMEKIT_ACCESSORY(.id=1, .category=homekit_accessory_category_sensor, .services=(homekit_service_t*[]){
        HOMEKIT_SERVICE(ACCESSORY_INFORMATION, .characteristics=(homekit_characteristic_t*[]){
            &name,
            HOMEKIT_CHARACTERISTIC(MANUFACTURER, "Armo Ltd."),
            HOMEKIT_CHARACTERISTIC(SERIAL_NUMBER, "003A1AVBG07P"),
            HOMEKIT_CHARACTERISTIC(MODEL, "Air Quality Sense"),
            HOMEKIT_CHARACTERISTIC(FIRMWARE_REVISION, "0.1.2"),
            HOMEKIT_CHARACTERISTIC(IDENTIFY, sensor_identify),
            NULL
        }),
        HOMEKIT_SERVICE(AIR_QUALITY_SENSOR, .primary=true, .characteristics=(homekit_characteristic_t*[]){
            HOMEKIT_CHARACTERISTIC(NAME, "Air Quality Sensor"),
            &air_quality,
            &voc_density,
            NULL
        }),
        HOMEKIT_SERVICE(CARBON_DIOXIDE_SENSOR, .primary=false, .characteristics=(homekit_characteristic_t*[]) {
            HOMEKIT_CHARACTERISTIC(NAME, "Carbon Dioxide Sensor"),
            &co2_detected,
            &co2_lvl,
//            &co2_peak_lvl,
            NULL
        }),
        NULL
    }),
    NULL
};

homekit_value_t air_quality_get () {
  return HOMEKIT_UINT8(air_quality.value.int_value);
}

homekit_value_t voc_density_get () {
  return HOMEKIT_FLOAT(voc_density.value.float_value);
}

homekit_value_t co2_detected_get () {
  return HOMEKIT_UINT8(co2_detected.value.bool_value);
}

homekit_value_t co2_lvl_get () {
  return HOMEKIT_FLOAT(co2_lvl.value.float_value);
}

homekit_server_config_t config = {
    .accessories = accessories,
    .password = "320-10-148"
};

void on_wifi_event(wifi_config_event_t event) {
    switch (event) {
        case WIFI_CONFIG_CONNECTED:
            printf("Connected\n");
            homekit_server_init(&config);
            break;
        case WIFI_CONFIG_DISCONNECTED:
            printf("Disconnected\n");
            break;
    }
}

void create_accessory_name() {
    uint8_t macaddr[6];
    sdk_wifi_get_macaddr(STATION_IF, macaddr);
    int name_len = snprintf(NULL, 0, "AQSensor-%02X%02X%02X",
                            macaddr[3], macaddr[4], macaddr[5]);
    char *name_value = malloc(name_len+1);
    snprintf(name_value, name_len+1, "AQSensor-%02X%02X%02X",
             macaddr[3], macaddr[4], macaddr[5]);
    name.value = HOMEKIT_STRING(name_value);
}

void user_init(void) {
    uart_set_baud(0, 115200);
    create_accessory_name();
    gpio_init();
    wifi_config_init2("AQSensor", NULL, on_wifi_event);
    adv_button_create(BUTTON_GPIO, true);
    adv_button_register_callback_fn(BUTTON_GPIO, reset_configuration, 5);
}
