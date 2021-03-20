#include "esp_camera.h"
#include <WiFi.h>
#include "BluetoothSerial.h"
#include <EEPROM.h>
#define CAMERA_MODEL_AI_THINKER
#define CAM_PIN_RESET 32 
#include "camera_pins.h"
#define EEPROM_SIZE 128
// define two tasks for Blink & AnalogRead
void TaskBluetooth( void *pvParameters );
void TaskWifi( void *pvParameters );



//WIFI 
uint8_t ssid_size;
uint8_t password_size;
uint8_t * ssid;
uint8_t * password;
uint8_t isConnected;
uint8_t packet_counter;
uint8_t TOTALPACKET = 5; 
unsigned long millisPassed; 

// BT 
BluetoothSerial ESP_BT; //Object for Bluetooth
WiFiClient clientTCP;

//queues
QueueHandle_t bluetoothQueue;

//void startCameraServer();

// TCP details 
char* host = "192.168.1.44";
const uint16_t port = 3333;

// functions 
void process_image(camera_fb_t * fb ){


    char *fbBuf = (char*)fb->buf;
    size_t fbLen = fb->len;
    uint32_t len = fbLen;
    char lenInChar[4];
    sprintf( lenInChar , "%lu" , len);
    int32_t return_write = clientTCP.write(lenInChar , 4  );
    return_write += clientTCP.write(fbBuf , fbLen);
    Serial.printf("Wrote to TCP: %d bytes\n", fbLen); 
    //Serial.printf("Wrote %d bytes\n", return_write); 

}

esp_err_t camera_capture(){
    //acquire a frame
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera Capture Failed");
        return ESP_FAIL;
    }
    //replace this with your own function
    //Serial.println("going for processing image");
    process_image(fb);
    //return the frame buffer back to the driver for reuse
    esp_camera_fb_return(fb);
    return ESP_OK;
}






// the setup function runs once when you press reset or power the board
void setup() {
  
  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  
  //bluetooth start
   ESP_BT.begin("ESP32_DEVICE_CONTROL"); 
  //end bluetooth 
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_SXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SXGA;
    config.jpeg_quality = 0;
    config.fb_count = 1;
  }
  

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);//flip it back
    s->set_brightness(s, 1);//up the blightness just a bit
    s->set_saturation(s, -2);//lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

isConnected = false;
  

  
  // Now set up two tasks to run independently.
  xTaskCreatePinnedToCore(
    TaskBluetooth
    ,  "TaskBluetooth"   // A name just for humans
    ,  5 * 1024  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  3  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  0);

  xTaskCreatePinnedToCore(
    TaskWifi
    ,  "TaskWifi"
    ,  10 * 1024  // Stack size
    ,  NULL
    ,  3  // Priority
    ,  NULL 
    ,  1);

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.

  //queues 
  bluetoothQueue = xQueueCreate( 100, sizeof( byte ) );

    //EEPROM ( memory )
    EEPROM.begin(EEPROM_SIZE);
    //wi-fi hasnt entered yet
    uint8_t isWifiConnected  = EEPROM.read(0); 
    Serial.println("isWifiConnected ??????????"  );
    Serial.println( isWifiConnected);
    if( isWifiConnected == 1  ){
      ssid_size = EEPROM.read(1);
      ssid = (uint8_t*)malloc( sizeof(uint8_t) * ssid_size + 1 );
      for( uint8_t i = 0; i < ssid_size; i++ ){
        ssid[i] = EEPROM.read(2 + i );
      }
      ssid[ssid_size] = '\0';
      password_size = EEPROM.read( ssid_size + 2);
      password = (uint8_t*)malloc( sizeof(uint8_t) * password_size + 1  );
      for( uint8_t i = 0; i < password_size; i++ ){
        password[i] = EEPROM.read(ssid_size + 3 + i );
      }
      password[password_size] = '\0';
      Serial.print("ssid size " );
      Serial.println( ssid_size );
      //Serial.print("ssid  %s " , ssid );
      Serial.println("passwordsize");
      Serial.print( password_size  );
      //Serial.print("ssid size %s \n" , password );
      WiFi.mode(WIFI_STA);
      WiFi.begin((char*)ssid, (char*)password);
     
      Serial.println("ssid" );

      
      for( int i =0; i < ssid_size ; i++ ){
        Serial.print((char)ssid[i]);
      }
      Serial.println("password");
      for( uint8_t i = 0; i < password_size; i++ ){
        Serial.print((char)password[i]);
      }
      while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
      }
      isConnected = true; 
      Serial.println("WiFi connected");
      for( int i = 0; i < 20; i++ ){
        uint8_t c = EEPROM.read(i);
        Serial.println(c);
      }

      /*if( clientTCP.connect(host, port)){
        Serial.println("TCP connected ");
      }*/
    }
    packet_counter = 0;
    millisPassed = millis();
    /*Serial.println("iswificonnected");
    //Serial.println(isWifiConnected);
    EEPROM.write(0,0);
    EEPROM.commit();
    uint8_t muzo =  EEPROM.read(0);
    Serial.println(muzo);*/

//   char a;
//   Serial.print("%d", &a);
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskBluetooth(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
    
  If you want to know what pin the on-board LED is connected to on your ESP32 model, check
  the Technical Specs of your board.
*/

  // initialize digital LED_BUILTIN on pin 13 as an output.

  for (;;) // A Task shall never return or exit.
  {
    delay(1000);
    if (ESP_BT.available()) //Check if we receive anything from Bluetooth
  {
    int incoming = ESP_BT.read(); //Read what we recevive
    Serial.print("Received:"); Serial.println(incoming);
    if( incoming == 71 ) // G for get 
    {
      if( isConnected == true){
        char ip[20];
        WiFi.localIP().toString().toCharArray(ip , 20);
        ESP_BT.write( (uint8_t*)ip , 20) ;
        ESP_BT.write( (uint8_t*)ssid , ssid_size ); 
        ESP_BT.write( (uint8_t*)password , password_size ); 
        //gives the status 
      }
      else{
        uint8_t notConnected = 78;
        ESP_BT.write( notConnected );
      }
    }
    else if( incoming == 67 ) // C for connect 
    {
      // 1 - send the size of ssid 
      // 2 - send ssid 
      // 3 - send the size of password 
      // 4 - send password
      

      ssid_size = ESP_BT.read(); // get ssid size 
      ssid = (uint8_t*)malloc( sizeof(uint8_t) * ssid_size + 1);

      for( int i = 0; i < ssid_size ;i++){
        ssid[i] = ESP_BT.read();
      }
      ssid[ssid_size] = '\0';
      
      password_size = ESP_BT.read();
      password = (uint8_t*)malloc( sizeof(uint8_t) * password_size + 1);
      
      for( int i = 0; i < password_size ;i++){
        password[i] = ESP_BT.read();
      }
      password[password_size] = '\0';
      Serial.println((char)ssid_size );
      Serial.println((char*)ssid);
      Serial.println((char)password_size );
      Serial.println((char*)password); 
        WiFi.mode(WIFI_STA);
        WiFi.begin((char*)ssid, (char*)password);

        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
      
//        startCameraServer();
        Serial.print(WiFi.localIP());
        isConnected = true;
        
        // write the wifi etc to the EEPROM 
        EEPROM.write(0,1);// written that it is connected
        // write the length of wifi 
        EEPROM.write(1, ssid_size );
        //write the ssid 
        for( uint8_t i = 0; i < ssid_size   ; i++){
           EEPROM.write( i + 2 , ssid[i] ) ;
        }
        //write the size of password
        EEPROM.write(  ssid_size + 2 , password_size  );
        for( uint8_t i = 0; i < password_size; i++ ){
          EEPROM.write( ssid_size + 3 + i , password[i] );
        }
        
        EEPROM.commit();
        if(clientTCP.connect(host, port)){
                Serial.println("TCP connected ");
        }

    }
    else if( incoming == 68 ){ // D for disconnect 
      WiFi.disconnect();
      EEPROM.write(0,0);
      EEPROM.commit();
      isConnected = false; 
      clientTCP.stop();
    }
    else if( incoming == 48){ // H for host device ip 
     
    }
    
  }
  else{
    struct timeval tv_now;
    //Serial.println( " muzooooooooooooooooooooooooooooooooooooooo");
    gettimeofday(&tv_now , NULL );
    int64_t num = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    uint32_t low = num % 0xFFFFFFFF; 
    uint32_t high = (num >> 32) % 0xFFFFFFFF;

    
    //Serial.print(high);
    //Serial.print(low);
  }
  }
}

void TaskWifi(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
/*
  AnalogReadSerial
  Reads an analog input on pin A3, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A3, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/

  for (;;)
  { 
    
    /*struct timeval tv_now;
    gettimeofday(&tv_now , NULL );
    int64_t num_before  = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    int64_t num_after  = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    int64_t diff = num_after - num_before;
    uint32_t low = num_after % 0xFFFFFFFF; 
    uint32_t high = (num_after >> 32) % 0xFFFFFFFF;*/
    //Serial.print(high);
    //Serial.print(low);
//      delay(50);
      
    //
    if( WiFi.status() == WL_CONNECTED){
//      camera_capture();
        if( isConnected ){            
            if (clientTCP.connected())
            {
              unsigned long tempSecond = millis();
              if( tempSecond - millisPassed < 1000  ){ // inside the second
                if ( TOTALPACKET > packet_counter ){ // check the total packets 
                  camera_capture();
                  packet_counter = packet_counter + 1;
                }
                             
              }
              else{
                millisPassed = millis(); // set new time 
                packet_counter = 0; 
                
              }
            }
            else 
            {
              millisPassed = millis();
              packet_counter = 0;
              clientTCP.connect(host , port );
            }
        }
          /*Serial.println(fbLen );
          clientTCP.write( fbBuf , fbLen );
          if( !clientTCP.connected()){
              Serial.println("connection has been sabotaged");
          }*/
        
        else{
          Serial.println("muzooooo");
        }
    }
    else {
      Serial.println("Connection lost! reconnecting to wifi!");
      WiFi.mode(WIFI_STA);
      WiFi.begin((char*)ssid, (char*)password);
      while (WiFi.status() != WL_CONNECTED) {
            delay(500);
            Serial.print(".");
      }
    }
    
  }
}
