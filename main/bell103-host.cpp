#include <stdio.h>
#include <string.h>
#include <thread>
#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <netdb.h>            // struct addrinfo
#include <arpa/inet.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"

#include "cmx865a.h"
#include "ShellProcessor.h"

#define MAXIMUM_AP 20

// In order on DEVKIT V1 board
#define PIN_HOOKSWITCH GPIO_NUM_26
#define PIN_LED GPIO_NUM_2

#define EXAMPLE_ESP_WIFI_SSID      "Gloucester Island Municipal WIFI"
#define EXAMPLE_ESP_WIFI_PASS      "emergency"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

#define CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK 1
#define CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK 1

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif

#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/**
 * Enables modem (data) mode
 */
static void setupModem(cmx865a& modem) {
  // TX control
  // We are the answering modem (high band)
  //                    5432109876543210    
  modem.write16(0xe1, 0b0111011000010110);
  // TX OFF
  //modem.write16(0xe1, 0b0000111000010110);

  // RX control
  // This is the calling modem setting used for echo test:
  //              5432109876543210    
  //modem.write16(0xe2, 0b0111111000110110); 
  // We are the answering modem (low band)
  // NORMAL:
  //                    5432109876543210    
  modem.write16(0xe2, 0b0110000000110110); 
}

static void sendSilence(cmx865a& modem) {
  // TX control: Tone pair TA
  modem.write16(0xe1, 0b0001000000000000);
}

static void sendDialTone(cmx865a& modem) {
  // TX control: Tone pair TA
  modem.write16(0xe1, 0b0001000000001100);
}

static void sendRingTone(cmx865a& modem) {
  // TX control: Tone pair TB
  modem.write16(0xe1, 0b0001000000001101);
}

/**
 * Simulation of the Arduino delay(ms) function.  Just sleeps.
 */
static void delay(unsigned long ms) {
  const TickType_t delay_ticks = ms / portTICK_PERIOD_MS;
  vTaskDelay(delay_ticks);
}

static unsigned long millis() {
  int64_t us_since_boot = esp_timer_get_time();
  return us_since_boot / 1000;
}

static int telnet_setup();

// ------ Integration for SerialProcessor -----------------------------------------

class TestPort : public SerialPort {
public:

    TestPort(cmx865a* modem)
    : _modem(modem) {        
    }

    void write(uint8_t c) {
      // Wait util we can send
      while (!_modem->txReady()) {
      }
      // Send
      _modem->write8(0xe3, c);
    }

    uint8_t read() {
      return 0;
    }

    bool isReadPending() {
      return false;
    }

private:

  cmx865a* _modem;
};

class TestEvent : public ShellProcessorEvent {
public:

    TestEvent(cmx865a* modem, int* telnetSocket)
    : _modem(modem),
      _telnetSocket(telnetSocket) {        
    }

    void handleCommand(const uint8_t* cmd) {       

      _modem->send("GOT COMMAND: [");
      _modem->send((const char*)cmd);
      _modem->send("]\r\n"); 

      if (strcmp((const char*)cmd, "connect") == 0) {
        // Open the telnet session 
        *_telnetSocket = telnet_setup();
        if (*_telnetSocket != 0) {
          _modem->send("Telnet connected\r\n");
        } else {
          _modem->send("Telnet not connected\r\n");          
        }
      } else {
        _modem->send("Unrecognized command\r\n");
      }
    }

private:

  cmx865a* _modem;
  int* _telnetSocket;
};

static void wifi_scan(SerialPort& port);

#define ON_HOOK 1
#define OFF_HOOK 0
static const char *TAG = "wifi";

static void run() {

  gpio_reset_pin(PIN_HOOKSWITCH);
  gpio_set_direction(PIN_HOOKSWITCH, GPIO_MODE_INPUT);
  gpio_reset_pin(PIN_LED);
  gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);

  int telnetSocket = 0;

  cmx865a modem;

  // Serial processor handles characters that come in on the model
  TestPort port(&modem);
  TestEvent event(&modem, &telnetSocket);
  ShellProcessor shellProc(&port, &event);

  // General reset of the CMX865A
  modem.write0(0x01);

  delay(20);

  // Write general control to power on
  //write16(0xe0, 0x0180); 
  //                    5432109876543210
  modem.write16(0xe0, 0b0000000110000000);
  
  // Per datasheet, start clock
  delay(20);

  //write16(0xe0, 0x0900);
  // Disable analog loopback, power on
  //                    5432109876543210
  modem.write16(0xe0, 0b0000000100000000);
    
  // Program the tone pairs
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x8000);

  // TA is used for US dial tone
  // TA1 frequency is 440, 440 * 3.414 = 1502, hex = 05DE
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x05DE);
  // TA1 amplitude is 0.5Vrms * 93780 / 3.3 = 14,209, hex = 3781
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x3981);
  // TA2 frequency is 350, 350 * 3.414 = 1194, hex = 04AA
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x04AA);
  // TA2 amplitude is 0.5Vrms * 93780 / 3.3 = 14,209, hex = 3781
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x3981);

  // TB is used for US ring tone
  // TB1 frequency is 440, 440 * 3.414 = 1502, hex = 05DE
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x05DE);
  // TB1 amplitude is 0.5Vrms * 93780 / 3.3 = 14,209, hex = 3781
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x3981);
  // TB2 frequency is 480, 480 * 3.414 = 1638 , hex = 0666 
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x0666);
  // TB2 amplitude is 0.5Vrms * 93780 / 3.3 = 14,209, hex = 3781
  while (!modem.programReady()) { }
  modem.write16(0xe8, 0x3981);

  // Show good startup
  gpio_set_level(PIN_LED, 1);
  delay(250);
  gpio_set_level(PIN_LED, 0);
  delay(250);
  gpio_set_level(PIN_LED, 1);
  delay(250);
  gpio_set_level(PIN_LED, 0);

  int lastHs = 1;
  long lastHsTransition = 0;
  int hsState = ON_HOOK;
  int state = 0;
  long stateChangeStamp = 0;
  int clickCount = 0;
  int digitCount = 0;
  int dialDigits[10];

  while (true) {
       
    if (state == 11) {

      // Check for inbound on the modem
      if (modem.rxReady()) {

        uint8_t d = modem.read8(0xe5);

        // If telnet isn't connected then pass input to the shell
        if (telnetSocket == 0) {
          shellProc.processInput(d);
        } 
        // Otherwise, pass input on the telnet session
        else {
          char buf[1];
          buf[0] = d;
          int written = send(telnetSocket, buf, 1, 0);
          if (written < 0 && errno != EINPROGRESS && errno != EAGAIN && errno != EWOULDBLOCK) {
            modem.send("Send error\r\n");
          }
        }
      }
    }

    // Check for inbound from telnet
    if (telnetSocket != 0) {
      // Non-blocking read
      char rx_buffer[128];
      int len = recv(telnetSocket, rx_buffer, sizeof(rx_buffer) - 1, 0);
      // Error occurred during receiving
      if (len < 0) {
          if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
            // Not an error
          } else {
            ESP_LOGE(TAG, "recv failed: errno %d", errno);
          }
      }
      // Data received
      else {
          // Null-terminate whatever we received and treat like a string        
          rx_buffer[len] = 0; 
          modem.send(rx_buffer);
          //ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
          //printf("%s", rx_buffer);
      }

    }

    if (state == 11 || state == 12) {
      // Show RX energy indication on the LED
      if (modem.rxEnergy()) {
        gpio_set_level(PIN_LED, 1);
      } else {
        gpio_set_level(PIN_LED, 0);
      }
    }

    // Look for a hookswitch transition  
    const int hs = gpio_get_level(PIN_HOOKSWITCH);
    if (hs != lastHs) {
      lastHs = hs;
      lastHsTransition = millis();
    }  

    // Look for debounced hookswitch transition
    if (millis() - lastHsTransition > 50 && hsState != lastHs) {
      hsState = lastHs;
    }

    // State machine

    // Hung up
    if (state == 0) {
      if (hsState == OFF_HOOK) {
        // Short delay before the tone starts
        delay(200);      
        sendDialTone(modem);
        
        clickCount = 0;
        digitCount = 0;
        state = 1;
        stateChangeStamp = millis();
      }
    }
    // Sending dial tone
    else if (state == 1) {
      // Look for timeout
      if (millis() - stateChangeStamp > 15000) {
        sendSilence(modem);
        state = 99;
        stateChangeStamp = millis();
      }
      // Look for break.  Could be hang up or dialing.
      if (hsState == ON_HOOK) {
        // Turn off dial tone
        sendSilence(modem);
        state = 2;
        stateChangeStamp = millis();
      }
    }
    // A first break was detected, look for dialing
    else if (state == 2) {
      if (hsState == ON_HOOK) {
        // Look for hang up
        if (millis() - stateChangeStamp > 500) {
          state = 0;
          stateChangeStamp = millis();
        }
      }
      // Look for transition back off hook.  This indicates
      // dialing
      else if (hsState == OFF_HOOK) {
        state = 4;
        stateChangeStamp = millis();
        clickCount++;
      }
    }
    // In this state a click (rising edge) was just recorded
    else if (state == 4) {
      if (hsState == ON_HOOK) {
        state = 2;
        stateChangeStamp = millis();
      } else {
        // Look for digit timeout
        if (millis() - stateChangeStamp > 250) {
          dialDigits[digitCount] = clickCount;
          clickCount = 0;        
          digitCount++;
          // Check to see if we have a full number
          if (digitCount == 7) {
            state = 10;
          } else {
            state = 5;   
          }
        }
      }
    }
    // In this state we are waiting for the next digit to be dialed,
    // or timeout
    else if (state == 5) {
      // Look for break.  Could be hang up or more dialing.
      if (hsState == ON_HOOK) {
        state = 2;
        stateChangeStamp = millis();
      } else {
        // Look for inter-digit timeout
        if (millis() - stateChangeStamp > 10000) {
          // Send error
          state = 99;
          stateChangeStamp = millis();
        }
      }
    }
    else if (state == 10) {

      //Serial.print("Connecting to ");
      //for (int i = 0; i < digitCount; i++) 
      //  Serial.print(dialDigits[i]);
      //Serial.println();

      delay(1000);
      sendRingTone(modem);
      delay(2000);
      sendSilence(modem);
      delay(4000);
      sendRingTone(modem);
      delay(2000);
      sendSilence(modem);
      delay(4000);    

      setupModem(modem);
      state = 11;
      stateChangeStamp = millis();

      // Notify that we have a connection
      //Serial.write(10);   
      //Serial.print("AT+CONN=1.1.1.1");   
      //Serial.write(10);   

      // Send a message to the remote station
      delay(1000);
      modem.send("You are now connected.\r\nWelcome to the 1980's\r\n\r\n");
      // Show the WIFI networks
      //port.send("WIFI networks:\r\n");
      //wifi_scan(port);
      //port.send("\r\n");

      // Establish telnet session
      telnetSocket = telnet_setup();
      if (telnetSocket != 0) {
        modem.send("telnet connected\r\n");
      } else {
        modem.send("telnet not connected\r\n");
      }
    }
    else if (state == 11) {
      
      // Wait for hangup
      if (hsState == ON_HOOK) {

        //Serial.write(26);
        //Serial.write(26);
        //Serial.print("AT+DISC");
        //Serial.write(10);
        
        // Disconnect telnet if necessary
        if (telnetSocket != 0) {
          close(telnetSocket);
          telnetSocket = 0;
        }

        sendSilence(modem);
        
        state = 0;
        stateChangeStamp = millis();
      }
    }
    else if (state == 99) {
      if (hsState == ON_HOOK) {
        state = 0;
        stateChangeStamp = millis();
      }
    }

    std::this_thread::yield();
  }
}

#define DEFAULT_SCAN_LIST_SIZE 20

/*
static void print_auth_mode(int authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OPEN");
        break;
    case WIFI_AUTH_OWE:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OWE");
        break;
    case WIFI_AUTH_WEP:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WEP");
        break;
    case WIFI_AUTH_WPA_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_PSK");
        break;
    case WIFI_AUTH_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_ENTERPRISE");
        break;
    case WIFI_AUTH_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_PSK");
        break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
        break;
    default:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_UNKNOWN");
        break;
    }
}

static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    case WIFI_CIPHER_TYPE_AES_CMAC128:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_AES_CMAC128");
        break;
    case WIFI_CIPHER_TYPE_SMS4:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_SMS4");
        break;
    case WIFI_CIPHER_TYPE_GCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_GCMP");
        break;
    case WIFI_CIPHER_TYPE_GCMP256:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_GCMP256");
        break;
    default:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }

    switch (group_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    case WIFI_CIPHER_TYPE_SMS4:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_SMS4");
        break;
    case WIFI_CIPHER_TYPE_GCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_GCMP");
        break;
    case WIFI_CIPHER_TYPE_GCMP256:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_GCMP256");
        break;
    default:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }
}
*/
// FreeRTOS event group to signal when we are connected
static EventGroupHandle_t s_wifi_event_group;

// The event group allows multiple bits for each event, but we only care about two events:
// - we are connected to the AP with an IP
// - we failed to connect after the maximum amount of retries 

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
  int32_t event_id, void* event_data) {

  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
      esp_wifi_connect();
  } 
  else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
        esp_wifi_connect();
        s_retry_num++;
        ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
        xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG,"connect to the AP fail");
  }
}

static void ip_event_handler(void* arg, esp_event_base_t event_base,
  int32_t event_id, void* event_data) {
  if (event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
  }
}

static void wifi_setup() {

  s_wifi_event_group = xEventGroupCreate();

  ESP_ERROR_CHECK(esp_netif_init());

  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  esp_event_handler_instance_t instance_any_id;
  esp_event_handler_instance_t instance_got_ip;

  ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                      ESP_EVENT_ANY_ID,
                                                      &wifi_event_handler,
                                                      NULL,
                                                      &instance_any_id));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                      IP_EVENT_STA_GOT_IP,
                                                      &ip_event_handler,
                                                      NULL,
                                                      &instance_got_ip));

  // Clear configuration structure before using
  wifi_config_t wifi_config;
  memset((void*)&wifi_config, 0, sizeof(wifi_config));
  // Setup SSID and password
  strcpy((char*)wifi_config.sta.ssid, EXAMPLE_ESP_WIFI_SSID);
  strcpy((char*)wifi_config.sta.password, EXAMPLE_ESP_WIFI_PASS);
  wifi_config.sta.scan_method = WIFI_FAST_SCAN;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  // Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
  // number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) 
  EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
          WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
          pdFALSE,
          pdFALSE,
          portMAX_DELAY);

  // xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
  // happened. 
  if (bits & WIFI_CONNECTED_BIT) {
      ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  } else if (bits & WIFI_FAIL_BIT) {
      ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  } else {
      ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

/**
 * Opens a telnet session and returns the socket file descriptor if successful.
 * Otherwise, returns 0.
 */
static int telnet_setup() {

  // Try to connect to something  
  //char rx_buffer[128];
  char host_ip[] = "64.13.139.230";
  int host_port = 23;
  int addr_family = 0;
  int ip_protocol = 0;

  struct sockaddr_in dest_addr;
  inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
  dest_addr.sin_family = AF_INET;
  dest_addr.sin_port = htons(host_port);
  addr_family = AF_INET;
  ip_protocol = IPPROTO_IP;

  int sock =  socket(addr_family, SOCK_STREAM, ip_protocol);
  if (sock < 0) {
      ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
      return 0;
  }
  ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, host_port);

  int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
  if (err != 0) {
      ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
      close(sock);
      return 0;
  }

  // Marking the socket as non-blocking
  int flags = fcntl(sock, F_GETFL);
  if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) {
      ESP_LOGE(TAG, "Unable to set socket non blocking");
      close(sock);
      return 0;
  }

  return sock;
}

/*
  while (1) {
    // Normally a blocking call    
    int len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
    // Error occurred during receiving
    if (len < 0) {
        if (errno == EINPROGRESS || errno == EAGAIN || errno == EWOULDBLOCK) {
          // Not an error
        } else {
          ESP_LOGE(TAG, "recv failed: errno %d", errno);
          break;
        }
    }
    // Data received
    else {
        // Null-terminate whatever we received and treat like a string        
        rx_buffer[len] = 0; 
        //ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
        printf("%s", rx_buffer);
    }
  }
}
*/

static void wifi_scan(SerialPort& port) {

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    esp_wifi_scan_start(NULL, true);
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

    for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
        char buf[80];
        sprintf(buf, "%s\r\n", ap_info[i].ssid);
        port.send(buf);
        //ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
        //print_auth_mode(ap_info[i].authmode);
        //if (ap_info[i].authmode != WIFI_AUTH_WEP) {
        //    print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
        //}
        //ESP_LOGI(TAG, "Channel \t\t%d", ap_info[i].primary);
    }
}

extern "C" {

  void app_main() {

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    wifi_setup();

    ESP_LOGI(TAG, "Entering main loop");
   
    run();
  }
}
