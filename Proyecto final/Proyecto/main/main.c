#include "hal/adc_types.h"
#include "hal/gpio_types.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds18x20.h>
#include <esp_log.h>
#include <esp_err.h>
#include <driver/gpio.h>
#include <unistd.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <stdlib.h>
#include <string.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <mqtt_client.h>
#include <lwip/sys.h>
#include <esp_http_client.h>
#include <esp_http_server.h>
#include <esp_system.h>
#include <esp_spiffs.h>
#include "driver/i2c.h"
#include "i2c-lcd.h"


// Modo AP
#define AP_SSID      "espSBC24M11B"
#define AP_PASS      "1234567890"

// HTML modo AP
char *web = "<!DOCTYPE html>"
            "<html>"
            "<head>"
            "<style>"
            "body { display: flex; justify-content: center; align-items: center; height: 100vh; margin: 0; font-family: Arial, sans-serif; }"
            "form { text-align: center; }"
            "h2 { font-size: 40px; margin-bottom: 20px; }"
            "label { font-size: 28px; }"
            "input[type='text'] { width: 100%; padding: 20px; margin: 10px 0; border: 2px solid #ccc; border-radius: 4px; font-size: 24px; }"  // Aumentar tamaño de los cuadros de texto
            "input[type='submit'] { padding: 20px 40px; background-color: #4CAF50; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 26px; }"  // Aumentar tamaño del botón
            "input[type='submit']:hover { background-color: #45a049; }"
            "</style>"
            "</head>"
            "<body>"
            "<form action=\"/submit\" method=\"post\">"
            "<h2>Introduzca SSID y PASSWORD:</h2>"
            "<label for=\"ssid\">SSID:</label><br>"
            "<input type=\"text\" id=\"ssid\" name=\"ssid\"><br>"
            "<label for=\"password\">PASSWORD:</label><br>"
            "<input type=\"text\" id=\"password\" name=\"password\"><br>"
            "<input type=\"submit\" value=\"Enviar\">"
            "</form>"
            "</body>"
            "</html>";

// Modo Wifi
char WIFI_SSID[64]; //SBC
char WIFI_PASS[64]; //SBCwifi$

// Thingsboard
#define THINGSBOARD_BROKER_URI "mqtt://demo.thingsboard.io"
#define THINGSBOARD_ACCESS_TOKEN "pl2uch7h6tb51pjy3pp2"
#define THINGSBOARD_URL "http://demo.thingsboard.io/api/v1/" THINGSBOARD_ACCESS_TOKEN "/telemetry"

// Variables Thingsboard
float ntu = 0.0;
int lvl_water =  0;
float temp = 0.0;	

// MQTT
static esp_mqtt_client_handle_t mqtt_client = NULL;

// Variables LCD
#define I2C_MASTER_SCL_IO           GPIO_NUM_22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                      /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// I2C
static esp_err_t i2c_master_init(void);

static const char *TAGlcd = "i2c-simple-example";

// Funciones TXT
void txt_init();								// Inicializa spiffs
void create_txt();								// Crea credenciales.txt
bool exists_txt();								// Verifica si existe credenciales.txt
bool empty_txt();								// Verifica si hay algo escrito en credenciales.txt
void read_txt();								// Lee una linea de credenciales.txt y asigna a WIFI_SSID y WIFI_PASS
void write_txt(char *ssid, char *password);		// Escribe ssid y password en credenciales.txt
void writeDollar(char *str);					// Escribe '$' en vez de '%24'
void delete_txt();								// Borra credenciales.txt

// Funciones STA
void sta_init(const char* ssid, const char* password);					// Inicializa modo wifi
static void wifi_event_handler(void* arg, esp_event_base_t event_base, 	// Controla modo wifi y cuando conecta
							   int32_t event_id, void* event_data);		// inicia programa "main"

// Funciones AP
void ap_init();									// Inicializa access point
void start_server();							// Inicia servidor
esp_err_t root_handler(httpd_req_t *req);		// Controla html
esp_err_t submit_handler(httpd_req_t *req);		// Controla inputs cliente

// Funciones ADC-LDR
void adc_init();								// Inicializa ADC para leer LDR

// Funciones MQTT
void mqtt_init();														// Inicializa MQTT
static void mqtt_event_handler(void *arg, esp_event_base_t event_base,	// Controla MQTT
							   int32_t event_id, void *event_data);
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);	// Controla MQTT callback
void publish_adc_value_mqtt(int adc_value);								// Manda valor del ADC a Thingsboard

//Main
void pMain();									// Programa "main"

// DS18B20
static const gpio_num_t SENSOR_GPIO = 17;
static const int MAX_SENSORS = 8;
static const int RESCAN_INTERVAL = 8;
static const uint32_t LOOP_DELAY_MS = 500;

static const char *TAGtemp = "DS18B20";

void ds18x20_test(void *pvParameter);

// SEN0204
static const char *TAGlevel = "SEN0204";

void sen0204_test(void *pvParameter);

// SEN0189
static const char *TAGturb = "SEN0189";
	// Resistencias del divisor de voltaje para el sensor de turbidez
	#define R1 4700.0  				   // 4.7 kΩ
	#define R2 10000.0 				   // 10.0 kΩ

void sen0189_test(void *pvParameter);
float calcular_ntu(float v_sensor);

// Pines ESP32
#define sen0204PIN 5			   // GPIO5
#define calentador 18			   // GPIO18
#define bombaAgua 19			   // GPIO17
#define ADC_CHANNEL ADC1_CHANNEL_4 // GPIO32
#define ADC_WIDTH ADC_WIDTH_BIT_12 // Resolución del ADC (12 bits)
#define ADC_ATTEN ADC_ATTEN_DB_11  // Atenuación para rango completo (hasta 3.3V)

// LCD Display
void display_init();
void lcd_test(void *pvParameter);

//-----------------------------------------------------------------------------------------------------------//
//---------------------------------- DESARROLLO DE LAS FUNCIONES DEFINIDAS ----------------------------------//
//-----------------------------------------------------------------------------------------------------------//

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE,
    						  I2C_MASTER_TX_BUF_DISABLE, 0);
}

void txt_init(){
	esp_vfs_spiffs_conf_t config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_vfs_spiffs_register(&config);
}

void create_txt(){
	printf("Creando archivo: credenciales.txt\n");
    FILE *f = fopen("/spiffs/credenciales.txt", "w");
    if (f == NULL)
    {
        printf("Fallo a la hora de abrir el archivo para escribir\n");
        return;
    }
    fclose(f);
}

bool exists_txt(){
	bool ok = true;
    FILE *f = fopen("/spiffs/credenciales.txt", "r");
    if (f == NULL){
        ok = false;
        printf("No existe credenciales.txt\n");
    }
    else printf("Existe credenciales.txt\n");
    fclose(f);
    return ok;
}

bool empty_txt(){
    FILE *f = fopen("/spiffs/credenciales.txt", "r");
    if (f == NULL)
    {
        printf("No existe credenciales.txt");
    }
	fseek(f, 0, SEEK_END);
  	long size = ftell(f);
   	fclose(f);  
    return size == 0;
}

void read_txt(){
	
	FILE *file = fopen("/spiffs/credenciales.txt", "r");
    char buffer[256]; 
    char SSID[64];
    char PASS[64];
    while (fgets(buffer, sizeof(buffer), file)) {
        if (sscanf(buffer, "%63s %63s", SSID, PASS) == 2) {
			writeDollar(SSID);
			writeDollar(PASS);
			
			printf("SSID: %s, PASSWORD: %s\n", SSID, PASS);
			strcpy(WIFI_SSID, SSID);
			strcpy(WIFI_PASS, PASS);
        }
    }
    fclose(file);
}

void write_txt(char *ssid, char *password){
	FILE *f = fopen("/spiffs/credenciales.txt", "a");
	printf("Escribiendo en credenciales.txt\n");
    fprintf(f, "%s %s\n", ssid, password);				// Escribimos credenciales.txt
    fclose(f);
    printf("Escritura finalizada\n");
}

void writeDollar(char *str) {
    char *pos;
    while ((pos = strstr(str, "%24")) != NULL) {
        *pos = '$';                        // Sustituimos % por $
        memmove(pos + 1, pos + 3, strlen(pos + 3) + 1);  // Mueve el resto de la cadena
    }
}

void delete_txt()
{
	remove("/spiffs/credenciales.txt");
}

void sta_init(const char* ssid, const char* password) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "",
            .password = "",
        },
    };

    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
    
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
	bool conectado = false;
    if (event_base == WIFI_EVENT) {
		if (event_id == WIFI_EVENT_STA_START) {
			esp_wifi_connect();
        } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
            printf("Conexion con intento fallida\n");
            conectado = false;
            create_txt();
            esp_restart();
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
		printf("Got IP Address: %d.%d.%d.%d\n", IP2STR(&event->ip_info.ip));            
        conectado = true;
    }
    if (conectado)
    {
		pMain();
	} else {
		delete_txt();
	}
}

void ap_init() {		//	Configuración esp32 para que sea un access point, inicializa la web
    esp_netif_init();										//	Inicializamos interfaz de red
    esp_event_loop_create_default();						//	Bucle de eventos por defecto
    esp_netif_create_default_wifi_ap();						//	Interfaz de red para el modo AP
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();	//	Config prefeterminada del wifi
    esp_wifi_init(&cfg);								//	Inicializa el wifi con la config asignada
    
    wifi_config_t wifi_config = {							//	Config del AP
        .ap = {
            .ssid = AP_SSID,
            .password = AP_PASS,
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    esp_wifi_set_mode(WIFI_MODE_AP);								//	Modo AP
    esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config);	//	Aplicamos config AP
    esp_wifi_start();													//	Inicia el wifi
    start_server();														//	Inicia la web
}

void start_server() {		// 	Configuración servidor http
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();				//	Config predeterminada
    config.server_port = 80;									//	Establecemos el puerto 80 para la web
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK) {//	Iniciamos servidor
    	//ruta para root_handler
        httpd_register_uri_handler(server, &(httpd_uri_t){ .uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = NULL });
        //ruta para submit_handler
        httpd_register_uri_handler(server, &(httpd_uri_t){ .uri = "/submit", .method = HTTP_POST, .handler = submit_handler, .user_ctx = NULL });
    }
}

//esp_err_t es un tipo de dato que representa error o exito
esp_err_t root_handler(httpd_req_t *req) {
    const char *html = web;
    httpd_resp_send(req, html, strlen(html));		//	Enviamos el HTML al cliente
    return ESP_OK;			//	Devolvemos exito
}

esp_err_t submit_handler(httpd_req_t *req) {		//	Guardar SSID y PW y mostrarla en terminal
	char buf[200];
	int received = httpd_req_recv(req, buf, sizeof(buf) - 1);
	if (received <= 0) return ESP_FAIL;
	buf[received] = '\0';							//	Se añade \0 para que sea una cadena de texto valida
	char *ssid, *password;
	//	Busca cadena ssid= en los datos recibidos, lo mismo con password=
    ssid = strstr(buf, "ssid=");
    password = strstr(buf, "password=");
    if (ssid && password) {
        ssid += strlen("ssid=");			//	El puntero de ssid se situa a continuacion de ssid=
        password += strlen("password=");	//	El puntero de password se situa a continuacion de password=
        char *end = strchr(ssid, '&');		//	Busca el & que separa los datos ingresados en la web 
        if (end) *end = '\0';				//	Si lo encuentra, mete un \0 para cortar ahi y quedarse solo con el ssid

        for (char *p = ssid; *p; p++) if (*p == '+') *p = ' ';		// Cambiar '+' a espacio
        
        end = strchr(password , '&');
        if (end) *end = '\0';
         
        for (char *p = password; *p; p++) if (*p == '+') *p = ' ';
         
        printf("SSID recibido: %s\n", ssid);
        printf("Password recibido: %s\n", password);
        write_txt(ssid, password);
		read_txt();
		esp_restart();
                
    }

    httpd_resp_set_status(req, "303 See Other");		//	Codigo de respuesta (redireccion)
    httpd_resp_set_hdr(req, "Location", "/");		//	Dirigimos al cliente a la pagina principal (/)
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

void adc_init(){
	adc1_config_width(ADC_WIDTH_BIT_12); // Resolución de 12 bits
	adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11); // Rango 0 - 3.3V
	gpio_set_pull_mode(GPIO_NUM_32, GPIO_PULLDOWN_ONLY);	
}

void mqtt_init() {
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = THINGSBOARD_BROKER_URI,    // Correct field for broker URI
        .credentials.username = THINGSBOARD_ACCESS_TOKEN,  // Correct field for ThingsBoard access token
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);  // Initialize MQTT client
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);  // Start the client
}

static void mqtt_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    printf("Event dispatched from event loop base=%s, event_id=%d\n", event_base, (int)event_id);
    mqtt_event_handler_cb(event_data);
}

static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event) {
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            printf("MQTT_EVENT_CONNECTED\n");
            break;
        case MQTT_EVENT_DISCONNECTED:
            printf("MQTT_EVENT_DISCONNECTED\n");
            break;
        default:
            printf("Other event id:%d\n", event->event_id);
            break;
    }
    return ESP_OK;
}

void publish_adc_value_mqtt(int adc_value) {
    char message[100];
    snprintf(message, sizeof(message), "{\"adc_value\": %d}", adc_value);  // JSON format

    // Publish to ThingsBoard telemetry topic
    esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry", message, 0, 1, 0);
}

void ds18x20_test(void *pvParameter)
{
    onewire_addr_t addrs[MAX_SENSORS];
    float temps[MAX_SENSORS];
    size_t sensor_count = 0;
    gpio_set_pull_mode(SENSOR_GPIO, GPIO_PULLUP_ONLY);
	float temp_c;
	gpio_set_direction(calentador,GPIO_MODE_OUTPUT);
	gpio_set_level(calentador, 0);
    esp_err_t res;
    while (1)
    {
        res = ds18x20_scan_devices(SENSOR_GPIO, addrs, MAX_SENSORS, &sensor_count);
        if (res != ESP_OK)
        {
            ESP_LOGE(TAGtemp, "Sensors scan error %d (%s)", res, esp_err_to_name(res));
            continue;
        }

        if (!sensor_count)
        {
            ESP_LOGW(TAGtemp, "No sensors detected!");
            continue;
        }

        ESP_LOGI(TAGtemp, "%d sensors detected", sensor_count);
        if (sensor_count > MAX_SENSORS)
            sensor_count = MAX_SENSORS;

        for (int i = 0; i < RESCAN_INTERVAL; i++)
        {
            ESP_LOGI(TAGtemp, "Measuring...");

            res = ds18x20_measure_and_read_multi(SENSOR_GPIO, addrs, sensor_count, temps);
            if (res != ESP_OK)
            {
                ESP_LOGE(TAGtemp, "Sensors read error %d (%s)", res, esp_err_to_name(res));
                continue;
            }

            for (int j = 0; j < sensor_count; j++)
            {
                temp_c = temps[j];
                ESP_LOGI(TAGtemp, "Sensor %08" PRIx32 "%08" PRIx32 " reports %.3f°C",
                        (uint32_t)(addrs[j] >> 32), (uint32_t)addrs[j], temp_c);
            }
        }
        temp = temp_c;
        if (temp_c < 25)
        {
			printf("fria");
			gpio_set_level(calentador, 1);
		} else {
			printf("caliente");
			gpio_set_level(calentador, 0);
		}
		vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

void sen0204_test(void *pvParameter)
{
    int sensor_state = 0;
    gpio_set_direction(bombaAgua, GPIO_MODE_OUTPUT);
	gpio_set_level(bombaAgua, 0);

    while (1)
    {
        sensor_state = gpio_get_level(sen0204PIN);
        lvl_water = sensor_state;

        if (sensor_state == 0)
        {
			gpio_set_level(bombaAgua, 1);
            ESP_LOGI(TAGlevel, "NO water detected");
            lvl_water = 0;
        }
        else
        {
			gpio_set_level(bombaAgua, 0);
            ESP_LOGI(TAGlevel, "Water detected");
            lvl_water = 1;
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

float calcular_ntu(float v_sensor) {
    float ntu = -1666.67 * v_sensor + 6666.67;
    if (ntu < 0) ntu = 0;           // Limitar valores negativos
    if (ntu > 3000) ntu = 3000;     // Limitar a 3000 NTU máximo
    return ntu;
}

void sen0189_test(void *pvParameter) {
    // Configurar ADC
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    while (1) {
        // Leer valor bruto del ADC
        int adc_value = adc1_get_raw(ADC_CHANNEL);

        // Convertir valor ADC a voltaje en el ADC
        float v_adc = adc_value * (3.3 / 4095.0);

        // Convertir voltaje del ADC al voltaje del sensor
        float v_sensor = v_adc * ((R1 + R2) / R2);

		
		//float escalado = 4.5/3000.0;
        // Convertir voltaje del sensor a NTU
        //float ntu = v_sensor * escalado;

        // Mostrar resultados
       	
		float current_ntu = calcular_ntu(v_sensor);
		ESP_LOGI(TAGturb, "Voltaje: %.2f V, NTU: %.2f\n", v_sensor, current_ntu);
		ntu = current_ntu;
        // Pausa de 1 segundo
        vTaskDelay(1500 / portTICK_PERIOD_MS);
    	}
	}

void publish_values_mqtt_test(void *pvParameter) {
	while(1){
    	char message[200];
    	snprintf(message, sizeof(message), "{\"ntu\": %f, \"temp\": %f, \"lvl_water\": %d}", ntu, temp, lvl_water);  // JSON format

    	// Publish to ThingsBoard telemetry topic
    	esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry", message, 0, 1, 0);
    	vTaskDelay(1500 / portTICK_PERIOD_MS);
    }
}

void display_init(){
	ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAGlcd, "I2C initialized successfully");
    
    lcd_init();
    lcd_clear();
}

void lcd_test(void *pvParameter){
	while (1){
		
		lcd_put_cur(0, 0);
		lcd_send_string("Temp: ");
		
		lcd_put_cur(0, 6);
        lcd_send_float(temp, 2);
        
        lcd_put_cur(1, 0);
        lcd_send_string("NTU: ");
        
        lcd_put_cur(1, 5);
        lcd_send_float(ntu, 0);
        
        lcd_put_cur(1, 11);
        lcd_send_string("WL: ");
        
        lcd_put_cur(1, 16);
        lcd_send_float(lvl_water, 0);
		}
	}
	vTaskDelay(3000 / portTICK_PERIOD_MS);
}

// 	CONEXIONES ESP32:
//		Sensor de temperatura (DS18B20): GPIO17
//		Calentador: GPIO18
//		Sensor nivel de agua (SEN0240): GPIO5
//      Bomba de agua: GPIO19	
//		Sensor de turbidez (SEN0189): GPIO32
//		Display LCD:
//		  - SDA: GPIO21
//		  - SCL: GPIO22
//	TODO DEBE ESTAR EN UNA MISMA TIERRA. 
//	RELE, DISPLAY Y SEN0189 SE ALIMENTAN CON 5V, LO DEMAS CON 3.3V.

//	1. Martillazo simple_ota_example
//	2. Abrimos servidor en la carpeta donde tengamos el update.bin (python3 -m http.server 8070)
//	3. SDKconfig -> Example Connection Configuration -> Wifi al que este conectado el orddenador
//	4. SDKconfig  -> Example Configuration -> http://<ip que te de el ordenador>:8070/update.bin
//	5. Compilamos simple_ota_example
	
void pMain(){

	mqtt_init();
	adc_init();
	display_init();

	//	DS18B20
	xTaskCreate(ds18x20_test, "DS18B20", configMINIMAL_STACK_SIZE * 4, 
    		    NULL, 5, NULL);
    
    //	SEN0204
    xTaskCreate(sen0204_test, "SEN0204", configMINIMAL_STACK_SIZE * 4,
    		    NULL, 4, NULL);
    
    //	SEN0189
    xTaskCreate(sen0189_test, "SEN0189", configMINIMAL_STACK_SIZE * 4,
    	        NULL, 3, NULL);
    
    //  LCD
    xTaskCreate(lcd_test, "DISPLAY", configMINIMAL_STACK_SIZE * 4, 
    		    NULL, 1, NULL);
    //  MQTT
    xTaskCreate(publish_values_mqtt_test, "publish_values_mqtt",
			    configMINIMAL_STACK_SIZE * 2, NULL, 4, NULL);
}

// IP MODO AP: 192.168.4.1
// SSID: SBC ; PASSWORD: SBCwifi$

void app_main()
{
	nvs_flash_init();
	txt_init();
	//sta_init("SBC","SBCwifi$");
	if(exists_txt()) {
		if(empty_txt()) {
			printf("Credenciales.txt esta vacio. Inicializando modo AP.\n");
			ap_init();
		}
		else {
			printf("Credenciales.txt no esta vacio. Inicializando modo WIFI e intentando conectar.\n");
			read_txt();
			sta_init(WIFI_SSID, WIFI_PASS);
		}
	}
	else {
		create_txt();
		esp_restart();
	}
}
