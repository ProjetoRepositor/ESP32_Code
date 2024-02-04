#include <driver/i2s.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <String.h>
#include <WiFiClientSecure.h>
#include "esp_heap_caps.h"

#define I2S_WS 15
#define I2S_SD 13
#define I2S_SCK 2
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000)
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (16 * 1024)
#define RECORD_TIME       (10) //Seconds
#define I2S_CHANNEL_NUM   (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)

// Definições para o Botão e LED
#define BUTTON_PIN 23
#define LED_PIN 22

// Variável para controlar a gravação
volatile bool startRecording = false;

File file;
const char filename[] = "/recording.wav";
const int headerSize = 44;
bool isWIFIConnected;

char i = '0';
char a[15] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};
int k = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Inicializa a conexão WiFi
  SPIFFSInit();
  i2sInit();
  // xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2, NULL, 1, NULL);
  delay(500);
  Serial.print("Memória heap livre depois do Setup: ");
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // Comunicação Serial com RX=2, TX=3
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Botão com resistor de pull-up
  pinMode(LED_PIN, OUTPUT);  // LED como saída
  digitalWrite(LED_PIN, LOW);
  Serial.println(esp_get_free_heap_size()); // Comunicação Serial com o computador
}

void loop() {
   if (Serial2.available()) {
        i = Serial2.read();
        a[k] = i;
        k++;
       String codigoEan="";
       if (k == 15) {
          Serial.println("===> Upload Ean to API BACKEND");

          for (int j = 0; j < 13; j++) {
               codigoEan+=a[j];
            }
        

        

          String serverPath="http://18.230.45.183:5000/api/v1/produto/"+codigoEan;                  

          Serial.println("ID: "+codigoEan);
          conectaWIFI();
              // Instancia o cliente HTTP e WiFiClientSecure
           HTTPClient http;

          http.begin(serverPath);
          http.addHeader("Content-Type", "application/json");
          int httpResponseCode = http.GET();

            if (httpResponseCode > 0) {
                // Obtém a resposta do servidor
                String response = http.getString();
                Serial.println(response);
            } else {
                Serial.print("Erro na requisição: ");
                Serial.println(httpResponseCode);
            }

            // Encerra a conexão
            http.end();
            desconectaWIFI();
     
  k = 0;
}
    }

     if (digitalRead(BUTTON_PIN) == HIGH && !startRecording) {
        startRecording = true;
        digitalWrite(LED_PIN, HIGH); // Acende o LED
        delay(500); // Espera um pouco para evitar detecção múltipla
        i2s_adc();
       
  }
}

void conectaWIFI(){
          
       // Se já estiver conectado, não faz nada
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi já está conectado.");
        return;
    }

    // Configura o modo WiFi para estação (cliente)
    WiFi.mode(WIFI_STA);

    // Inicia a conexão WiFi
    WiFi.begin("VITAOFODA", "vitaofoda123");
    Serial.println("Conectando ao WiFi...");

    // Tenta se conectar ao WiFi por um tempo máximo (por exemplo, 10 segundos)
    unsigned long startTime = millis();
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        
        // Verifica se excedeu o tempo máximo de tentativa de conexão
        if (millis() - startTime > 10000) {
            Serial.println("\nFalha ao conectar ao WiFi.");
            return;
        }
    }

    isWIFIConnected = true;
    Serial.println("\nWiFi conectado!");

}

void desconectaWIFI(){
        isWIFIConnected=false;
        WiFi.disconnect();    
        delay(100); // Aguarda 100ms para processamento
        WiFi.mode(WIFI_OFF);  
        Serial.println("Wifi desconectado!!");
}

void SPIFFSInit(){
  if(!SPIFFS.begin(true)){
    Serial.println("SPIFFS initialisation failed!");
    while(1) yield();
  }

  SPIFFS.remove(filename);
  file = SPIFFS.open(filename, FILE_WRITE);
  if(!file){
    Serial.println("File is not available!");
  }

  byte header[headerSize];
  wavHeader(header, FLASH_RECORD_SIZE);

  file.write(header, headerSize);
  listSPIFFS();
}

void i2sInit(){
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(I2S_SAMPLE_BITS),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = 0,
    .dma_buf_count = 64,
    .dma_buf_len = 1024,
    .use_apll = 1
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}


void i2s_adc_data_scale(uint8_t * d_buff, uint8_t* s_buff, uint32_t len)
{
    uint32_t j = 0;
    uint32_t dac_value = 0;
    for (int i = 0; i < len; i += 2) {
        dac_value = ((((uint16_t) (s_buff[i + 1] & 0xf) << 8) | ((s_buff[i + 0]))));
        d_buff[j++] = 0;
        d_buff[j++] = dac_value * 256 / 2048;
    }
}
  

void i2s_adc() {
    if (!startRecording) {
        return;
    }

    int i2s_read_len = I2S_READ_LEN;
    int flash_wr_size = 0;
    size_t bytes_read;

    Serial.println("Memória heap disponível antes da alocação do buffer i2s_read_buff: "); 
    Serial.println(esp_get_free_heap_size()); 
    char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
    Serial.println("Memória heap disponível antes da alocação do buffer flash_write_buff: "); 
    Serial.println(esp_get_free_heap_size()); 
    uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));

    if (i2s_read_buff == NULL || flash_write_buff == NULL) {
        Serial.println("Falha ao alocar memória");
        free(i2s_read_buff);
        free(flash_write_buff);
        return;
    }

    digitalWrite(LED_PIN, HIGH);
    Serial.println(" *** Recording Start *** ");

    while (flash_wr_size < FLASH_RECORD_SIZE) {
        i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);
        i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
        file.write((const byte*) flash_write_buff, i2s_read_len);
        flash_wr_size += i2s_read_len;

        // Progresso da gravação
        ets_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
    }

    file.close();
    digitalWrite(LED_PIN, LOW);
    Serial.println(" *** Recording End *** ");

   free(i2s_read_buff);
   free(flash_write_buff);
   Serial.println("Memória heap disponível antes de ligar o Wi Fi: "); 
    Serial.println(esp_get_free_heap_size()); 
    conectaWIFI();
     Serial.println("Memória heap disponível depois de ligar o Wi Fi: "); 
    Serial.println(esp_get_free_heap_size()); 
    // Upload do arquivo, se conectado ao WiFi
    // if (isWIFIConnected) {
    //     uploadFile();
    // }
    desconectaWIFI();
    Serial.println("Memória heap disponível depois de desligar o Wi Fi: "); 
    Serial.println(esp_get_free_heap_size()); 
    

 

    startRecording = false;
}


void formatSPIFFS() {
  if (!SPIFFS.begin(true)) {
    Serial.println("Erro ao montar SPIFFS");
    return;
  }

  Serial.println("Formatando SPIFFS...");
  SPIFFS.format();
  Serial.println("SPIFFS formatado.");
}

void example_disp_buf(uint8_t* buf, int length)
{
    printf("======\n");
    for (int i = 0; i < length; i++) {
        printf("%02x ", buf[i]);
        if ((i + 1) % 8 == 0) {
            printf("\n");
        }
    }
    printf("======\n");
}

void wavHeader(byte* header, int wavSize){
  header[0] = 'R';
  header[1] = 'I';
  header[2] = 'F';
  header[3] = 'F';
  unsigned int fileSize = wavSize + headerSize - 8;
  header[4] = (byte)(fileSize & 0xFF);
  header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF);
  header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W';
  header[9] = 'A';
  header[10] = 'V';
  header[11] = 'E';
  header[12] = 'f';
  header[13] = 'm';
  header[14] = 't';
  header[15] = ' ';
  header[16] = 0x10;
  header[17] = 0x00;
  header[18] = 0x00;
  header[19] = 0x00;
  header[20] = 0x01;
  header[21] = 0x00;
  header[22] = 0x01;
  header[23] = 0x00;
  header[24] = 0x80;
  header[25] = 0x3E;
  header[26] = 0x00;
  header[27] = 0x00;
  header[28] = 0x00;
  header[29] = 0x7D;
  header[30] = 0x01;
  header[31] = 0x00;
  header[32] = 0x02;
  header[33] = 0x00;
  header[34] = 0x10;
  header[35] = 0x00;
  header[36] = 'd';
  header[37] = 'a';
  header[38] = 't';
  header[39] = 'a';
  header[40] = (byte)(wavSize & 0xFF);
  header[41] = (byte)((wavSize >> 8) & 0xFF);
  header[42] = (byte)((wavSize >> 16) & 0xFF);
  header[43] = (byte)((wavSize >> 24) & 0xFF);
  
}


void listSPIFFS(void) {
  Serial.println(F("\r\nListing SPIFFS files:"));
  static const char line[] PROGMEM =  "=================================================";

  Serial.println(FPSTR(line));
  Serial.println(F("  File name                              Size"));
  Serial.println(FPSTR(line));

  fs::File root = SPIFFS.open("/");
  if (!root) {
    Serial.println(F("Failed to open directory"));
    return;
  }
  if (!root.isDirectory()) {
    Serial.println(F("Not a directory"));
    return;
  }

  fs::File file = root.openNextFile();
  while (file) {

    if (file.isDirectory()) {
      Serial.print("DIR : ");
      String fileName = file.name();
      Serial.print(fileName);
    } else {
      String fileName = file.name();
      Serial.print("  " + fileName);
      // File path can be 31 characters maximum in SPIFFS
      int spaces = 33 - fileName.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      String fileSize = (String) file.size();
      spaces = 10 - fileSize.length(); // Tabulate nicely
      if (spaces < 1) spaces = 1;
      while (spaces--) Serial.print(" ");
      Serial.println(fileSize + " bytes");
    }

    file = root.openNextFile();
  }

  Serial.println(FPSTR(line));
  Serial.println();
  delay(1000);
}



void uploadFile(){
  file = SPIFFS.open(filename, FILE_READ);
  if(!file){
    Serial.println("FILE IS NOT AVAILABLE!");
    return;
  }

  Serial.println("===> Upload FILE to Node.js Server");

  HTTPClient client;
  client.begin("http://192.168.1.124:8888/uploadAudio");
  client.addHeader("Content-Type", "audio/wav");
  int httpResponseCode = client.sendRequest("POST", &file, file.size());
  Serial.print("httpResponseCode : ");
  Serial.println(httpResponseCode);

  if(httpResponseCode == 200){
    String response = client.getString();
    Serial.println("==================== Transcription ====================");
    Serial.println(response);
    Serial.println("====================      End      ====================");
  }else{
    Serial.println("Error");
  }
  file.close();
  client.end();
}