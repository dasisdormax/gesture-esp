#include "wireless.h"


static const char * TAG = "wifi";

static uint64_t t_btn_prev;
static uint64_t t_btn_down;
static esp_timer_handle_t timer_longpress = NULL;
static bool button_pressed = false;
static xQueueHandle button_queue = NULL;

static bool is_configured = false;
static bool is_connecting = false;
static esp_timer_handle_t timer_blink = NULL;
static bool blink_state = false;

static uint64_t blink_delay;

typedef struct {
    const char *ssid;       // WiFi SSID
    const char *passphrase; // WiFi passphrase
    const char *ip;         // IP address of smartphone
} wifi_def;

wifi_def known_wifi[] = {
    {
        .ssid = "THEwifi",
        .passphrase = "",
        .ip = "192.168.43.148",
    },
    {
        .ssid = "Orisa",
        .passphrase = "",
        .ip = "192.168.1.223",
    },
    {
        .ssid = "esphotspot",
        .passphrase = "esphotspot",
        .ip = "192.168.4.50"
    },
    {
        .ssid = "MST_Z_MESH",
        .passphrase = "esphotspot",
        .ip = "192.168.4.50"
    },
};

wifi_def *wifi_data = NULL;


////////////////////////////////////////////////////
//////// BUTTON INTERRUPT HANDLING
////////////////////////////////////////////////////

static void blink_scan(void *args) {
    gpio_set_level(LED_GPIO, blink_state);
    blink_state = !blink_state;
    esp_timer_start_once(timer_blink, blink_delay);
}

static void on_button_long_pressed(void *args) {
    is_connecting = true;
    is_configured = false;
    blink_state = false;
    blink_delay = 500000;
    ESP_LOGI(TAG, "button long press detected.");
    if(!timer_blink) {
        esp_timer_create_args_t args;
        args.callback = blink_scan;
        esp_timer_create(&args, &timer_blink);
    }
    blink_scan(NULL);
    wireless_reset();
}

static void on_connect() {
    if(timer_blink)
        esp_timer_stop(timer_blink);
    is_connecting = false;
    is_configured = true;
    gpio_set_level(LED_GPIO, 0);
}

static void on_disconnect() {
    if(timer_blink)
        esp_timer_stop(timer_blink);
    is_connecting = false;
    is_configured = false;
    wifi_data = NULL;
    gpio_set_level(LED_GPIO, 0);
    esp_wifi_stop();
}

static void on_button_pressed()
{
    if(is_connecting) return;
    t_btn_down = esp_timer_get_time();
    ESP_LOGD(TAG, "Button pressed.");
    gpio_set_level(LED_GPIO, 1);
    // Create long press timer
    if(!timer_longpress) {
        esp_timer_create_args_t args;
        args.callback = on_button_long_pressed;
        esp_timer_create(&args, &timer_longpress);
    }
    esp_timer_start_once(timer_longpress, 1100000);
}

static void on_button_released() {
    // Scanning or long press
    if(is_connecting) return;
    esp_timer_stop(timer_longpress);
    // Double-press detected
    if(t_btn_down - t_btn_prev < 500000) {
        ESP_LOGI(TAG, "Button double press detected.");
    }
    t_btn_prev = t_btn_down;
    ESP_LOGD(TAG, "Button released.");
    gpio_set_level(LED_GPIO, 0);
}

static void IRAM_ATTR button_isr(void* data) {
    uint32_t gpio_num = (uint32_t)data;
    xQueueSendFromISR(button_queue, &gpio_num, NULL);
}

static void button_queue_task(void *arg) {
    uint32_t io_num;
    ESP_LOGI(TAG, "Installed button queue");
    for(;;) {
        if(xQueueReceive(button_queue, &io_num, portMAX_DELAY)) {
            button_pressed = !gpio_get_level((gpio_num_t)io_num);
            if(button_pressed)
                on_button_pressed();
            else
                on_button_released();
        }
    }
}

static void configure_button(void)
{
    if(button_queue) return;
    gpio_reset_pin(LED_GPIO);
    gpio_reset_pin(BTN_GPIO);

    button_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(button_queue_task, "gpio_task_example", 2048, NULL, 10, NULL);

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BTN_GPIO, GPIO_MODE_INPUT);

    gpio_intr_enable(BTN_GPIO);
    gpio_set_intr_type(BTN_GPIO, GPIO_INTR_ANYEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_GPIO, button_isr, (void*) (uint32_t) BTN_GPIO);
}




////////////////////////////////////////////////////
//////// WIRELESS CONNECTION HANDLING
////////////////////////////////////////////////////

static wifi_ap_record_t records[16];

static esp_netif_ip_info_t ip_info;

static wifi_def * wifi_known_data(const char *ssid) {
    int numData = sizeof(known_wifi) / sizeof(wifi_def);
    for(int i = 0; i < numData; i++) {
        wifi_def *curr = &known_wifi[i];
        if(!strcmp(curr->ssid, ssid)) {
            return curr;
        }
    }
    return NULL;
}

static int connection_tries = 0;

static void wifi_event_handler (
    void* arg, esp_event_base_t event_base,
    int32_t event_id, void* event_data
) {
    if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // TODO: start smartconfig
        // scan networks.
        memset(&records, 0, sizeof(records));
        esp_wifi_scan_start(NULL, false);
    } else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE) {
        uint16_t num_records = 16;
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&num_records, records));
        for (int i = 0; i < num_records; i++) {
            wifi_ap_record_t ap = records[i];
            ESP_LOGI(TAG, "[%d] SSID %s, RSSI %d, C %d", i, ap.ssid, ap.rssi, ap.primary);
            wifi_data = wifi_known_data((char *)ap.ssid);
            if(wifi_data) {
                ESP_LOGI(TAG, "Connecting to %s ...", wifi_data->ssid);
                wifi_config_t config = {
                    .sta = {
                        .pmf_cfg = {
                            .capable = true,
                            .required = false,
                        },
                    },
                };
                strcpy((char *)config.sta.ssid, wifi_data->ssid);
                strcpy((char *)config.sta.password, wifi_data->passphrase);
                config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
                ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &config) );
                connection_tries = 0;
                esp_wifi_connect();
                return;
            }
        }
        // No suitable wifi was found
        on_disconnect();
    } else if(event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        // Connected and assigned IP address
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ip_info = event->ip_info;
        ESP_LOGI(TAG, "connected! got ip: " IPSTR, IP2STR(&ip_info.ip));
        on_connect();

    } else if(event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if(is_connecting) {
            // connection failed, retry or cancel
            if(++connection_tries < 5) {
                esp_wifi_connect();
                ESP_LOGI(TAG, "retrying to connect ...");
                return;
            } else {
                ESP_LOGI(TAG, "Too many failed connection attempts!");
            }
        }
        // Disconnected or too many failed attempts
        on_disconnect();
    }
}

void wireless_setup() {
    is_connecting = true;
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start() );
}

void wireless_init() {
    configure_button();

    // Configure wifi
    ESP_ERROR_CHECK( nvs_flash_init() );
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();


    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL) );
    //ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL) );

    wireless_setup();
}

int sock;

void wireless_send(char *buf) {
    // TODO: connect and send in a separate FreeRTOS task
    //       to prevent blocking of measurement loop
    if(!is_configured) return;
    if(!wifi_data) return;
    const char *ip = wifi_data->ip;
    if(!ip) return;
    // TODO: add network discovery
    if(sock < 0) {
        // Create and configure socket
        sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
        sockaddr_in addr;
        addr.sin_addr.s_addr = inet_addr(ip);
        addr.sin_family = AF_INET;
        addr.sin_port = htons(5000);
        if(connect(sock, (sockaddr *)&addr, sizeof(addr))) {
            close(sock);
            sock = -1;
            return;
        };
    }
    if(sock < 0) return;

    // Send data
    int err = send(sock, buf, strlen(buf), 0);
    if(err < 0) {
        ESP_LOGI(TAG, "socket disconnected ...");
        shutdown(sock, 0);
        close(sock);
        sock = -1;
    }
}

void wireless_reset() {
    esp_wifi_stop();
    wifi_data = NULL;
    wireless_setup();
}

bool wireless_is_configured() {
    return is_configured;
}