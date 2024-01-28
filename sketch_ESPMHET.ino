#include <driver/i2s.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <String.h>


//pinos para leitura de áudio
#define I2S_WS 15
#define I2S_SD 13
#define I2S_SCK 2
#define I2S_PORT I2S_NUM_0
#define I2S_SAMPLE_RATE   (16000)
#define I2S_SAMPLE_BITS   (16)
#define I2S_READ_LEN      (16 * 1024)
#define RECORD_TIME       (20) //Seconds
#define I2S_CHANNEL_NUM   (1)
#define FLASH_RECORD_SIZE (I2S_CHANNEL_NUM * I2S_SAMPLE_RATE * I2S_SAMPLE_BITS / 8 * RECORD_TIME)

// Definições para o Botão e LED
#define BUTTON_PIN 23
#define LED_PIN 22

// Variável para controlar a gravação
volatile bool startRecording = false;

//define o arquivo para ser enviado para o servidor e fazer o reconhecimento de voz
File file;
const char filename[] = "/recording.wav";
const int headerSize = 44;
bool isWIFIConnected=false;

char i = '0';
char a[15] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'};
int k = 0;

void setup() {
  Serial.begin(115200);  // Comunicação Serial com o computador
  // Serial1.begin(115200); // Para o microfone
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // Comunicação Serial com RX=2, TX=3

// Inicializa a conexão WiFi
    WiFi.begin("VITAOFODA", "vitaofoda123");
    Serial.println("Conectando ao WiFi...");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    isWIFIConnected=true;
    Serial.println("\nWiFi conectado!");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());

//Inicializa  a task de leitura de áudio
  SPIFFSInit();
  i2sInit();
  xTaskCreate(i2s_adc, "i2s_adc", 1024 * 2, NULL, 1, NULL);
  delay(500);
  // xTaskCreate(wifiConnect, "wifi_Connect", 4096, NULL, 0, NULL);
  // Configuração do Botão e LED
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Botão com resistor de pull-up
  pinMode(LED_PIN, OUTPUT);           // LED como saída
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
          HTTPClient http;
          String serverPath="https://vp4pbajd60.execute-api.sa-east-1.amazonaws.com/Prod/api/v1/produto/"+codigoEan;
         
          // String serverPath="http://192.168.68.110:8080";

          Serial.println("ID: "+codigoEan);
          http.begin(serverPath.c_str());
          // Send HTTP GET request
          int httpResponseCode = http.GET();
          
          if (httpResponseCode>0) {
            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);
            String payload = http.getString();
            Serial.println(payload);
          }
          else {
            Serial.print("Error code: ");
            Serial.println(httpResponseCode);
          }
          // Free resources
          http.end();


      //     client.begin();
      //     int httpResponseCode = client.sendRequest("GET");
      //    if (httpResponseCode == HTTP_CODE_OK) {
      //   String response = client.getString();
      //   // Processar resposta
      //   Serial.println(response);
      //   codigoEan="";
      //   } else {
      //       Serial.print("Erro na requisição: ");
      //       Serial.println(httpResponseCode);
      //   }

      //  client.end();
  // Serial.print("ID : ");
  // for (int j = 0; j < 13; j++) {
  //     Serial.print(a[j]);
  // }
  // Serial.println();

  // if (strncmp(a, "8851959132166", 13) == 0) {
  //     Serial.println("Name : Fanta Orange");
  //     Serial.println();
  // }
  // if (strncmp(a, "8851959132173", 13) == 0) {
  //     Serial.println("Name : Fanta Strawberry");
  //     Serial.println();
  // }
  k = 0;
}
    }

    // Verifica se o botão é pressionado
    // if (digitalRead(BUTTON_PIN) == HIGH) {
    //     startRecording = true;
    //     digitalWrite(LED_PIN, HIGH); // Acende o LED

    //     // Espera um pouco para evitar detecção múltipla
    //     // delay(500);
    // }

     
           
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



void i2s_adc(void *arg) {
    int i2s_read_len = I2S_READ_LEN;
    size_t bytes_read;
    char* i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
    uint8_t* flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
    int flash_wr_size = 0;
    unsigned long startTime = 0;

    while (true) {
        if (startRecording) {
            digitalWrite(LED_PIN, HIGH); // Acende o LED
            Serial.println(" *** Recording Start *** ");
            startTime = millis(); // Armazena o momento em que a gravação começou

            flash_wr_size = 0;
            while (millis() - startTime < 10000) { // Grava por 10 segundos
                // Lê dados do I2S
                i2s_read(I2S_PORT, (void*) i2s_read_buff, i2s_read_len, &bytes_read, portMAX_DELAY);

                // Escala e escreve os dados no arquivo
                i2s_adc_data_scale(flash_write_buff, (uint8_t*)i2s_read_buff, i2s_read_len);
                file.write((const byte*) flash_write_buff, i2s_read_len);

                flash_wr_size += i2s_read_len;
                ets_printf("Sound recording %u%%\n", flash_wr_size * 100 / FLASH_RECORD_SIZE);
            }

            file.close();
            digitalWrite(LED_PIN, LOW); // Apaga o LED
            Serial.println(" *** Recording End *** ");

            // Lista os arquivos no SPIFFS e envia para o servidor se WiFi conectado
            listSPIFFS();
            if (isWIFIConnected) {
                uploadFile();
            }

            startRecording = false;

            // Libera os buffers
            free(i2s_read_buff);
            free(flash_write_buff);

            // Recria os buffers para a próxima gravação
            i2s_read_buff = (char*) calloc(i2s_read_len, sizeof(char));
            flash_write_buff = (uint8_t*) calloc(i2s_read_len, sizeof(char));
        }

        // Pausa breve para evitar uso excessivo de CPU
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
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

// void wifiConnect(void *pvParameters) {
//     isWIFIConnected = false;
//     char* ssid = "VITAOFODA";
//     char* password = "vitaofoda123";

//     Serial.println("Conectando ao WiFi...");
//     WiFi.begin(ssid, password);

//     int maxTentativas = 10;
//     for (int tentativas = 0; tentativas < maxTentativas; tentativas++) {
//         if (WiFi.status() == WL_CONNECTED) {
//             isWIFIConnected = true;
//             Serial.println("Conectado com sucesso ao WiFi!");
//             break;
//         }
//         Serial.print(".");
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }

//     if (!isWIFIConnected) {
//         Serial.println("\nFalha ao conectar ao WiFi. Verifique as credenciais ou o sinal.");
//         Serial.print("Código de erro do WiFi: ");
//         Serial.println(WiFi.status());
//     }

//     vTaskDelete(NULL); // Termina a task se a conexão falhar ou for bem-sucedida
// }

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


