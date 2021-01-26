#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <SPI.h>
#include <FirebaseArduino.h>
#include <SoftwareSerial.h>
#define FIREBASE_HOST "piscigranja-137b5.firebaseio.com"
#define FIREBASE_AUTH "TKD4iKNAlATPS4u2bzE1lMezXqD57dmzIEMRN63T"
#include <Separador.h>
#define WIFI_SSID "Dannyroot"
#define WIFI_PASSWORD "12345678"

#define DHTTYPE DHT11
#define DHTPIN  4

unsigned long previousMillis = 0;       
const long interval = 2300; 
char server[] = "192.168.0.11";   //eg: 192.168.0.222  "107.180.41.44";
long randomNumber;
Separador s;

WiFiClient client;

String temperatura ;
String ph ;
String turbidez ;
String oxy ;

void setup() {
Serial.begin(9600);
while (!Serial) {
; 
}
WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
Serial.print("connectando");
while (WiFi.status() != WL_CONNECTED) {
Serial.print(".");
delay(500);
}
 Serial.println();
 Serial.print("Conectado: ");
 Serial.println(WiFi.localIP());

Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
randomSeed(26);

 }

void loop() {

if (Serial.available()) {
 String datosrecibidos = Serial.readString();
 temperatura = s.separa(datosrecibidos, ',', 0);
 ph = s.separa(datosrecibidos, ',', 1);
 turbidez = s.separa(datosrecibidos, ',', 2);
 oxy = s.separa(datosrecibidos, ',', 3);
 String temp = s.separa(datosrecibidos, ',', 4);
//Serial.write(Serial.read());
}

Serial.print("Temperatura firebase = ");
Serial.println(temperatura);
Serial.print("ph firebase = ");
Serial.println(ph);
Serial.print("turbidez firebase = ");
Serial.println(turbidez);
Serial.print("oxygeno firebase = ");
Serial.println(oxy);
Firebase.setString("dispositivos/x001/temperatura",temperatura);
Firebase.setString("dispositivos/x001/ph",ph);
Firebase.setString("dispositivos/x001/ntu",turbidez);
Firebase.setString("dispositivos/x001/oxigeno",oxy);
Serial.print("Enviando data al hosting ");
Sending_To_phpmyadmindatabase();
delay(10000); 
}

void Sending_To_phpmyadmindatabase()   //CONNECTING WITH MYSQL
 {
   if (client.connect(server, 80)) {
    Serial.println("Conectado al server");
    // Make a HTTP request:
    //Serial.print("GET http://piscigranja.org/insertData/x001.php?temperatura=");
    client.print(String ("GET ")+"http://192.168.0.11/insertData/x001.php?temperatura="+temperatura
    +"&ph="+ph+"&turbidez="+turbidez+"&oxigeno="+oxy);     //YOUR URL
    //client.print(temperatura);
    //Serial.println(temperatura);
    client.print(" ");      //SPACE BEFORE HTTP/1.1
    client.print("HTTP/1.1");
    client.println();
    client.println("Host: tu IP local");
    client.println("Conneccion: cerrada");
    client.println();
  } else {
    // if you didn't get a connection to the server:
    Serial.println("conneccion fallida");
  }
 }

void serialEvent()
{
 
 
}
