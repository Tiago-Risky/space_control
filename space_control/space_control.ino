

#include <dht_nonblocking.h>

/* Uncomment according to your sensortype. */
#define DHT_SENSOR_TYPE DHT_TYPE_11

#define SensorPin 2
#define LedPin 3
#define LDR A0
#define LDR_OFF 500
static const int DHT_SENSOR_PIN = 10;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );
int radar =0;
float temperature =0;
float humidity =0;
int lumens=0;
int sensor =1;

String message = "";

void setup()
{
  Serial.begin(9600);
  delay(1000);//Aguarda 1 seg antes de acessar as informações do sensor
  pinMode (SensorPin, INPUT);
  pinMode (LedPin, OUTPUT);
  pinMode (LDR,INPUT);
  }
static bool measure_environment( float *temperature, float *humidity )
{
  static unsigned long measurement_timestamp = millis( );

  if( millis( ) - measurement_timestamp > 4000ul )
  {
    if( dht_sensor.measure( temperature, humidity ) == true )
    {
      measurement_timestamp = millis( );
      return( true );
    }
  }

  return( false );
}
//float temperature;
//float humidity;

    float t = 0;
    float h = 0;


void loop(){ 
  if( measure_environment( &temperature, &humidity ) == true ){
    //Serial.print( "T = " );
    //Serial.print( temperature, 1 );
    //Serial.print( " deg. C, H = " );
    //Serial.print( humidity, 1 );
    //Serial.println( "%" );
    t = temperature;
    h = humidity;
  }

     
    int ldrRead = analogRead (LDR);
    delay(100);
    //Serial.println(ldrRead);
    if(ldrRead < LDR_OFF) {
       analogWrite(LedPin, 0);
    } else {
       analogWrite(LedPin,map(ldrRead, 0, 1023, 0, 255) );
    }
    lumens = ldrRead;

    int sensorValue = digitalRead(SensorPin);
    if (sensorValue==HIGH){
       digitalWrite (LedPin,HIGH);
       radar = 1;
    } else {
       digitalWrite (LedPin,LOW);
       radar = 0;
    }


    //message = String(sensor) + "," + String(radar) + "," + String(humidity) + "," + String(temperature) + "," + String(lumens);
    message = String(sensor) + "," + String(radar) + "," + String(h,1) + "," + String(t,1) + "," + String(lumens);
    
    Serial.println(message);

}
  