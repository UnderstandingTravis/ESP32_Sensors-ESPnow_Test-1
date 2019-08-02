#include <DHTesp.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_now.h>
#include <WiFi.h>

#define BME_SCK 18  //  onboard as SCL .... also i2c
#define BME_MISO 19 //aka SDO  
#define BME_MOSI 23 //aka SDI  on board as SDA .... also i2c
#define BME_CS 15  // on board CSB ?
//#define SEALEVELPRESSURE_HPA (1013.25)

#define WIFI_CHANNEL 1

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI  SCL=14, SDA=13, CSB=15, SDO=12
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI ... SCL=18, SDA=23, CSB=5, SDO=19

//DHT22 sensor stuff
DHTesp cheaptempsensor1; //aka cts1
int cts1Pin = 21;

//esp communication stuff
//uint8_t remoteMac[] = {0x30, 0xAE, 0xA4, 0xF2, 0xB4, 0x45};
uint8_t remoteMac[] = {0x30, 0xAE, 0xA4, 0xF2, 0xE1, 0x39};
const uint8_t maxDataFrameSize=250;
esp_now_peer_info_t slave;
const esp_now_peer_info_t *peer = &slave;
uint8_t dataToSend[maxDataFrameSize];
uint8_t testdata[250];
uint8_t cnt=0;
uint32_t timers[3];


void setup() {
  // use Serial 1 (Serial1, yes thats the number one) ; Modbus baud rate
  Serial1.begin(9600); //RS485 (level converter used), UART, baud rate 9600, 8 data bits, 1 stop bit, no parity
  // put your setup code here, to run once:
  Serial.begin(115200);
  while(!Serial); 
  
  cheaptempsensor1.setup(cts1Pin, DHTesp::DHT22);
  
  unsigned status;
  status = bme.begin(0x76);
  if (!status) {
    //do erro things
  }
  
  WiFi.mode(WIFI_STA);
  Serial.println( WiFi.macAddress() );
  WiFi.disconnect(); 
    if(esp_now_init() == ESP_OK)
  {
    Serial.println("ESP NOW INIT!");
  }
  else
  {
    //TODO error handleing 
    Serial.println("ESP NOW INIT FAILED....");
  }
  memcpy( &slave.peer_addr, &remoteMac, 6 );
  slave.channel = WIFI_CHANNEL;
  slave.encrypt = 0;
  if( esp_now_add_peer(peer) == ESP_OK)
  {
    Serial.println("Added Peer!");
  }
  //else
  //{ // TODO add error handeling }
  

  esp_now_register_send_cb(OnDataSent); //see functions wayyy below
  esp_now_register_recv_cb(OnDataRecv); //see functions wayyy below 
}

void loop() {

    uint8_t buf[249];
    float tempfloat;
  
  // put your main code here, to run repeatedly:

  TempAndHumidity cts1TnH = cheaptempsensor1.getTempAndHumidity();
  if (cheaptempsensor1.getStatus() != 0) {
    Serial.println( String(cheaptempsensor1.getStatusString()));
  }

  //cts1 = cheap temp sensor 1 , heati = heat index, dew = dew point
  //big D mean decimal ( or the numbers after the decmial point

//CONFUSION CITY! I'm replaceing all the ints and stuff, with their respective position in the buffer that is to be sent
//maybe next automization is to just send?
  
  buf[11] = long((cheaptempsensor1.computeHeatIndex(cts1TnH.temperature, cts1TnH.humidity))*10) % 10 ;
  buf[10] = long(cheaptempsensor1.computeHeatIndex(cts1TnH.temperature, cts1TnH.humidity)) ;
  buf[13] = long((cheaptempsensor1.computeDewPoint(cts1TnH.temperature, cts1TnH.humidity))*10) % 10 ;
  buf[12] = long(cheaptempsensor1.computeDewPoint(cts1TnH.temperature, cts1TnH.humidity)) ;
  //get and convert the temp and humidity data
  buf[15] = long(cts1TnH.temperature*10) % 10 ;
  buf[14] = long(cts1TnH.temperature) ;
  buf[17] = long(cts1TnH.humidity*10) % 10 ;
  buf[16] = long(cts1TnH.humidity) ;
  
  Serial.printf("\nTemp: %d.%d" , buf[14], buf[15] );
  Serial.printf("  Humid: %d.%d", buf[16], buf[17] );

  Serial.printf("  Heat Index: %d.%d", buf[10],buf[11] ); 
  Serial.printf("  Dew point: %d.%d", buf[12],buf[13] ); 
  

  Serial.print("\nTemperature = ");
  tempfloat = bme.readTemperature() ;
  buf[18] = int(tempfloat);
  buf[19] = int(tempfloat*10) %10;
  Serial.printf("%d.%d",buf[18],buf[19]);
  Serial.print(" *C  ");
  
  Serial.print("Humidity = ");
  tempfloat = bme.readHumidity();
  buf[20] = int(tempfloat);
  buf[21] = int(tempfloat*10) %10;
  Serial.printf("%d.%d",buf[20],buf[21]);
  Serial.print(" %");
  
  Serial.print("Pressure = ");
  tempfloat = bme.readPressure() / 100;
  buf[22] = int(tempfloat);
  buf[23] = int(tempfloat*10) %10;
  Serial.printf("%d.%d",buf[20],buf[21]);
  Serial.print(" hPa  ");


 
  
  Serial.printf(" \n");
  uint8_t buf1[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x0A, 0x70, 0x0D}; //8 bytes?
  //TODO: make CRC stuff work. 16bit CRC 
  
  Serial1.write(buf1,8); // send the bytes from above out to the RS485 - Modbus
  
  //Read in from the serial input buffer... one byte at a time :-/   
  uint8_t buf2[63]; //probably to large
  //looks like it sends 25 bytes max? should probably check this
  for (int j = 0; j < 63; j++)
  {
    buf2[j] = Serial1.read();
    //TODO: make CRC stuff work. 16bit CRC 
  }
  //wipe out read buffer becauae its not clearing out?? 
  //maybe my buffer isn't clearing out ??? TODO

  //serial.peek useful here? 

  //TODO: make CRC stuff work. 16bit CRC 
  
  uint16_t voltage =((uint8_t)buf2[3] <<  8) | buf2[4];
  uint8_t voltageD = voltage % 10;
  voltage = voltage / 10;
  Serial.printf("Voltage : %d.%d ", voltage, voltageD);
  //the order is low 16 bits first, AND THEN the high 16 bits... so 7,8,5,6
  uint32_t amps = ((uint8_t)buf2[7] << 24) | ((uint8_t)buf2[8] << 16) | ((uint8_t)buf2[5] <<  8) | buf2[6];
  uint16_t ampsD = amps % 1000;
  amps = amps / 1000;
  Serial.printf(" Amps : %d.%d ",amps, ampsD); 
  //now energy
  uint32_t watts = ((uint8_t)buf2[11] << 24) | ((uint8_t)buf2[12] << 16) | ((uint8_t)buf2[9] <<  8) | buf2[10];
  uint8_t wattsD = watts % 10;
  watts = watts / 10;
  Serial.printf(" Watts : %d ", watts); 
  uint32_t Wh = ((uint8_t)buf2[15] << 24) | ((uint8_t)buf2[16] << 16) | ((uint8_t)buf2[13] <<  8) | buf2[14];
  Serial.printf(" Wh : %d ", Wh); 
  //insert witty comment here... also specs stuff
  uint16_t Hz = ((uint8_t)buf2[17] <<  8) | buf2[18];
  uint8_t HzD = Hz % 10;
  Hz = Hz / 10;
  Serial.printf(" Hz : %d.%d ", Hz, HzD);
  uint16_t Pf = ((uint8_t)buf2[19] <<  8) | buf2[20];
  uint8_t PfD = Pf % 100;
  Pf = Pf /100;
  Serial.printf(" Power Factor : %d.%d ", Pf, PfD);

   //alarm codes?
   
  Serial.printf("\n \n");

  

  //int lengthofdata = snprintf(buf, sizeof(buf), "i can send letter but can i send values??? %d", testdata[0] );
  esp_now_send(slave.peer_addr, (uint8_t *)buf, maxDataFrameSize);


  
  delay(3000);
}




void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  Serial.print("\r\nLast Packet Sent Status:\t");
  Serial.print(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  Serial.print("\n");
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
  if(data[0] == dataToSend[0])
  {
    timers[1] = micros();
    timers[2] = timers[1]-timers[0];
    Serial.printf("\r\nReceived\t%d Bytes val\t%d\tin %d micros", data_len, data[0], timers[2]);
  }
  else
  {
    Serial.printf("\r\nReceived\t%d Bytes\t%d", data_len, data[0]);
  }
}
