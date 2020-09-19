/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
//#include <string.h>

#include <driver/gpio.h>
#include <driver/adc.h>
#include <driver/rtc_io.h>
#include <driver/spi_master.h>
//#include <driver/rtc_cntl.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_sleep.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_err.h> // 
#include <esp_netif.h>
#include <esp_intr_alloc.h>
#include <lwip/err.h>
#include <lwip/sockets.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <soc/sens_reg.h>
#include <soc/rtc_periph.h>
#include <soc/rtc.h>
#include <esp32/ulp.h>
#include <ulp_main.h>
//#include <soc/rtc_cntl_reg.h>
//#include <soc/rtc_io_reg.h>
//#include <soc/soc_ulp.h>

#include <lwip/err.h>
#include <lwip/sys.h>

#include <sdkconfig.h>

//#define HOST_IP "54.157.172.217"
#define HOST_IP "192.168.15.10"
//#define PORT 1026
#define PORT 40404

#define SSID      CONFIG_ESP_WIFI_SSID
#define PASS      CONFIG_ESP_WIFI_PASS
#define MAX_RETRY CONFIG_ESP_MAXIMUM_RETRY

static EventGroupHandle_t s_wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

static int s_retry_num = 0;

static const char* TAG = "TEST";

/*static void IRAM_ATTR rtc_intr_handler(void* arg)
{
    ulp_main_cpu_awake = 1;
    printf("interrupt\n");
    (void)arg;
}*/

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT)
    {
        if (event_id == WIFI_EVENT_STA_START)
        {
            esp_wifi_connect();
            ESP_LOGI(TAG, "Connecting...");
        }
        else if (event_id == WIFI_EVENT_STA_DISCONNECTED)
        {
            if (s_retry_num >= MAX_RETRY)
            {
                xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGI(TAG, "Max retry reached.");
                return;
            }
            else
            {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGI(TAG, "Retrying...");
            }
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        //ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Connected");
        //connected
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = 
    {
        .sta = 
        {
            .ssid = SSID,
            .password = PASS
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Conectado a: %s", SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "Erro na connexão wifi");
    }
    else
    {
        ESP_LOGW(TAG, "Evento desconhecido");
    }
    
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_got_ip));
    vEventGroupDelete(s_wifi_event_group);
}

void tcp_test()//void* pvParameters)
{
    //static const char* test_payload = "POST /v2/entities HTTP/1.0\r\nContent-Type: application/json\r\nContent-Length:28\r\n\r\n"
    //                                "{\"id\":\"Test2\",\"type\":\"Test\"}";
    //static const char* test_payload = "test package";
    //char test_payload[50];
    //sprintf(test_payload, "test package #%i", ulp_pir_sensor_readings & UINT16_MAX);
    //ESP_LOGI(test_payload, "");
    printf("||| TRYING TO CONNECT |||\n");
    //printf(test_payload);
    //printf("\n");
    /*
    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    struct sockaddr_in addr;
    addr.sin_addr.s_addr = inet_addr(HOST_IP);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Socket error: %d", errno);
        return;
    }
    int err = connect(sock, (struct sockaddr*) &addr, sizeof(struct sockaddr_in));
    if (err != 0)
    {
        ESP_LOGE(TAG, "Connect error: %d", err);
        return;
    }
    err = send(sock, test_payload, strlen(test_payload), 0);
    if (err < 0)
    {
        ESP_LOGE(TAG, "Send error: %d", errno);
        return;
    }
    char recv_buffer[128];
    int len = recv(sock, recv_buffer, sizeof(recv_buffer)-1, 0);
    if (len < 0)
    {
        ESP_LOGE(TAG, "Recv error: %d", errno);
        return;
    }
    if (sock != -1)
    {
        ESP_LOGI(TAG, "Transmission Concluded.");
        shutdown(sock, 0);
        close(sock);
    }*/
}

void init_ulp_program()
{
    //SET_PERI_REG_MASK(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA);
    //REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA);
    //ESP_ERROR_CHECK(esp_intr_alloc(ETS_RTC_CORE_INTR_SOURCE, 0/*ESP_INTR_FLAG_LOWMED*/, rtc_intr_handler, NULL, NULL));
    //ESP_ERROR_CHECK(rtc_isr_register(rtc_intr_handler, NULL, RTC_CNTL_ULP_CP_INT_ENA));
    //uint32_t rtc_8md256_period = rtc_clk_cal(RTC_CAL_8MD256, 100);
    //uint32_t rtc_fast_freq_hz = 1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_8md256_period; // 8680393

    adc_power_on();
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_ulp_enable();

    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    gpio_num_t gpio_num_pir = GPIO_NUM_4;
    gpio_num_t gpio_num_ldr = GPIO_NUM_34;
    int rtcio_num_pir = rtc_io_number_get(gpio_num_pir);
    //int rtcio_num_ldr = rtc_io_number_get(gpio_num_ldr);

    ulp_pir_sensor_readings = 0;
    ulp_pir_sensor_reading_number = 0;
    ulp_ldr_sensor_readings = 0;
    *(&ulp_ldr_sensor_readings+1) = 0;
    ulp_ldr_sensor_reading_number = 0;
    ulp_pir_io_number = rtcio_num_pir;
    ulp_general_counter = 0;
    ulp_finished_handling_data = 0;
    ulp_main_cpu_awake = 1;
    ulp_data_loss = 0;

    ulp_debug = 0;

    rtc_gpio_init(gpio_num_pir);
    rtc_gpio_init(gpio_num_ldr);
    rtc_gpio_set_direction(gpio_num_pir, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_set_direction(gpio_num_ldr, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(gpio_num_pir);
    rtc_gpio_pulldown_dis(gpio_num_ldr);
    rtc_gpio_pullup_dis(gpio_num_pir);
    rtc_gpio_pullup_dis(gpio_num_ldr);
    rtc_gpio_hold_en(gpio_num_pir);
    rtc_gpio_hold_en(gpio_num_ldr);

    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging();

    //ulp_set_wakeup_period(0, 50);
    ulp_set_wakeup_period(0, 100000);

    err = ulp_run(&ulp_start - RTC_SLOW_MEM);
    printf("||| START COUNT |||\n");
    ESP_ERROR_CHECK(err);
}

void app_main()
{
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    ulp_main_cpu_awake = 1;

    if (cause != ESP_SLEEP_WAKEUP_ULP && cause != ESP_SLEEP_WAKEUP_TIMER)
    {
        ESP_LOGI(TAG, "Initializing ULP.");
        init_ulp_program();
    }
    else
    {/*
        ESP_LOGI(TAG, "Waking up, trying to send package.");
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_init());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());*/

        //wifi_init_sta();
        tcp_test();
        //esp_wifi_stop();
    }
    ESP_LOGI(TAG, "Entering deep sleep.");
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    //ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(60000000));
    //unsigned long aux = 0;
    unsigned long tst, tst2;
    unsigned long fck;
    //while (1)
    for (int fkfk = 0; fkfk<2; fkfk++)
    {
        //printf("|| %lu ||\n", (unsigned long) (ulp_main_cpu_awake & UINT16_MAX));
        fck = ulp_ldr_sensor_reading_number & UINT16_MAX;
        tst = *(&ulp_ldr_sensor_readings+0) & UINT16_MAX;
        tst2 = *(&ulp_ldr_sensor_readings+1) & UINT16_MAX;
        //if (aux != tst)
        {
            //aux = tst;
            printf("||| %lu, %lu, %lu |||\n", tst, tst2, fck);
        }
        /*if (tst > 500)
            ulp_main_cpu_awake = 0;*/
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    ulp_main_cpu_awake = 0;
    esp_deep_sleep_start();
    /*
    printf("Hello world!\n");

    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    for (int i = 10; i >= 0; i--)
    {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();*/
}
