#include <stdio.h>

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
#include <esp_err.h>
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

#include <lwip/err.h>
#include <lwip/sys.h>

#include <sdkconfig.h>

#include "protocol.c"

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

static const proto8_t self_tag[] = {2};
static const proto8_t server_tag[] = {1};

#define SELF_TAG_SIZE sizeof(self_tag)
#define SERVER_TAG_SIZE sizeof(server_tag)

#define PIR_SEND_SIZE 3
#define LDR_SEND_SIZE 1
#define PIR_MAX_SIZE 6
#define LDR_MAX_SIZE 2

#define PIR_PDU_SIZE(X) (PIR_HEADER_SIZE+SELF_TAG_SIZE+SERVER_TAG_SIZE+2+PIR_PAYLOAD_SIZE(X))
#define LDR_PDU_SIZE(X) (LDR_HEADER_SIZE+SELF_TAG_SIZE+SERVER_TAG_SIZE+2+LDR_PAYLOAD_SIZE(X))

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

int udp_comunicate(char payload[], int length, char response_buffer[], int response_length)//void* pvParameters)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    struct sockaddr_in addr;
    addr.sin_addr.s_addr = inet_addr(HOST_IP);
    addr.sin_family = AF_INET;
    addr.sin_port = htons(PORT);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "Socket error: %d", errno);
        return -1;
    }

    int err = sendto(sock, payload, length, 0, (struct sockaddr*) &addr, sizeof(struct sockaddr_in));
    if (err < 0)
    {
        ESP_LOGE(TAG, "Send error: %d", errno);
        return -1;
    }
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(struct sockaddr_in);
    int len = recvfrom(sock, response_buffer, response_length-1, 0, (struct sockaddr*) &source_addr, &socklen);
    if (len < 0)
    {
        ESP_LOGE(TAG, "Recv error: %d", errno);
        return len;
    }
    if (sock != -1)
    {
        ESP_LOGI(TAG, "Transmission Concluded.");
        shutdown(sock, 0);
        close(sock);
    }
    return len;
}

void reset_array(unsigned int array[], unsigned int size)
{
    for (unsigned int i = 0; i < size; i++)
    {
        array[i] = 0;
    }
       
}

void init_ulp_program()
{
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

    reset_array(&ulp_pir_sensor_readings, PIR_MAX_SIZE);
    reset_array(&ulp_ldr_sensor_readings, LDR_MAX_SIZE);
    ulp_pir_sensor_reading_number = 0;
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

    ulp_set_wakeup_period(0, 100000);

    err = ulp_run(&ulp_start - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}

void ulp_data16_to_proto8_t(unsigned int origin[], proto16_t size, proto8_t output_buffer[])
{
    for (int i = 0; i < size; i++)
    {
        output_buffer[i << 1] = (origin[i] >> 8) & UCHAR_MAX;
        output_buffer[(i << 1) + 1] = origin[i] & UCHAR_MAX;
    }
}

void ulp_data1_to_proto8_t(unsigned int origin[], proto16_t size, proto8_t output_buffer[])
{
    for (int i = 0; i < size; i++)
    {
        output_buffer[i >> 3] = origin[i] & (1 << (i & 7));
    }
}

void send_pir()
{
    proto8_t payload[PIR_PAYLOAD_SIZE(PIR_MAX_SIZE)];
    proto16_t length = ulp_pir_sensor_reading_number & UINT16_MAX;
    ulp_data1_to_proto8_t(&ulp_pir_sensor_readings, length, payload);
    proto8_t data_unit[PIR_PDU_SIZE(PIR_MAX_SIZE)];
    int tcp_length = make_send_pir_data_unit(data_unit, (proto8_t*) self_tag, (proto8_t*) server_tag, payload, PIR_PAYLOAD_SIZE(length));
    char* tcp_payload = (char*) data_unit;
    char response_buffer[256];
    udp_comunicate(tcp_payload, tcp_length, response_buffer, 256);
}

void send_ldr()
{
    proto8_t payload[LDR_PAYLOAD_SIZE(LDR_MAX_SIZE)];
    proto16_t length = ulp_ldr_sensor_reading_number & UINT16_MAX;
    ulp_data16_to_proto8_t(&ulp_ldr_sensor_readings, length, payload);
    proto8_t data_unit[LDR_PDU_SIZE(LDR_MAX_SIZE)];
    int tcp_length = make_send_ldr_data_unit(data_unit, (proto8_t*) self_tag, (proto8_t*) server_tag, payload, LDR_PAYLOAD_SIZE(length));
    
    char* tcp_payload = (char*) data_unit;
    char response_buffer[256];
    int response_length = udp_comunicate(tcp_payload, tcp_length, response_buffer, 256);
}

void wake_up_routine()
{
    if (ulp_pir_sensor_reading_number >= PIR_SEND_SIZE && ulp_ldr_sensor_reading_number >= LDR_SEND_SIZE)
    {
        send_pir();
        send_ldr();

        reset_array(&ulp_pir_sensor_readings, PIR_MAX_SIZE);
        reset_array(&ulp_ldr_sensor_readings, LDR_MAX_SIZE);

        ulp_finished_handling_data = 1;
    }
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
    {
        ESP_LOGI(TAG, "Waking up, trying to send package.");
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
        {
            ESP_ERROR_CHECK(nvs_flash_init());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);

        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());

        wifi_init_sta();
        wake_up_routine();
        esp_wifi_stop();
    }
    ESP_LOGI(TAG, "Entering deep sleep.");
    ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
    //ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(60000000));

    ulp_main_cpu_awake = 0;
    esp_deep_sleep_start();
}
