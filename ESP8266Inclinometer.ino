/******************************************************************************
ESP8266Inclinometer.ino

D. Gibson KJ6FO - SquirrelEngineering.Com
Dec 15th 2019

Useful links
https://www.adafruit.com/product/2809 -- LIS3DH Board used in this project
https://github.com/sparkfun/LIS3DH_Breakout
https://github.com/sparkfun/SparkFun_LIS3DH_Arduino_Library
https://bitbucket.org/hyOzd/serialplot/src/default/ -- Serial Plot program, much better than the Arduino serial plotter.

Description:
A project measure tilt using a LIS3DH Accelerometer to detect changes in tilt relative to a level surface and or movement.
The project is battery operated and is optimized to use as little battery current as possible using the ESP8266 DeepSleep mode and
runtime optimizations to reduct the time the ESP8266 is running at full power.


NOTE:
This code is designed for an ESP8266 ESP-12F in the module only form. (See published scematic). However this code will also run on
a NODEMCU board and a Wemo D1 board, but at much higher current consuption. A raw ESP8266 module is needed to maximize power savings for battery ops.
The project circuit uses an Adafruit LIS3DH accelerometer, but the Sparkfun library had more functions, so it was used.

******************************************************************************/
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>

#include "SparkFunLIS3DH.h"
#include "Wire.h"

#define DEBUG 0

#define DEEPSLEEP 1         // Enable=1 / Disable=0 deep sleep mode. Turn off for basic development/test/debug and turn on for production operation.
#define SLEEPSECONDS 120    // Time to sleep between measurements. Less often will save battery power.

#if DEBUG
// Macros that will display debug output only when DEBUG is true
#define DEBUGOUT(s) (Serial.print(s))
#define DEBUGOUTLN(s) (Serial.println(s))
#else
// else do nothing.
#define DEBUGOUT(s) ((void)0)
#define DEBUGOUTLN(s) ((void)0)
#endif

// Samples from the LIS3DH are averaged over a larg-ish sample size to even out any noise in the data.
// A tradeoff between more samples = less noise variation, but takes more time. 500 is about right, but you can experiment.
#define NSAMPLES 500 // # of samples
float XAve = 0;      // Average X value
float YAve = 0;      // Average Y Value
// NOTE: Z-Axis omitted. It is not needed to detect bed deflection.

// Power up the LIS3DH only when needed. It is a very low current draw device, so we can power it off a GPOI Pin when needed.
#define IMU_POWERPIN 13  

// LIS3DH Sensor object
LIS3DH myIMU(I2C_MODE,0x18); //Override to Adafruit default address 0x18 //Default constructor is I2C, addr 0x19 for the Sparkfun board.

// WiFi stuff
ESP8266WiFiMulti WiFiMulti;
const char* ssid1 = ""; // Your WiFi Access point name here.
const char* password1 = ""; // Access point password.
//const char* ssid2 = "";         // You can add additional Access points, but see note below.
//const char* password2 = "";

// Server stuff
const char* host = ""; // Webserver's domain name.

// Get the MAC unique id for the ESP module
String StationMAC = WiFi.macAddress(); // Unique ID for this ESP8266 device, used to identify the reporting station in the database.


// Setup!
void setup() {

  //Start Serial
  Serial.begin(115200);
  //Serial.setDebugOutput(true); // Uncomment for WiFi debug info helpful for resolving network issues etc.
  while (!Serial) {
      delay(1); // wait for serial port to connect. 
  }
    
  // When ESP8266 Module boots up there is typically some garbage chars in the Serial stream (Diagnostic messages sent at a different boaud rate)
  // So just put a few blank lines to separate the info from the garbage. This helps SerialPlot find the data to plot.
  Serial.println(); 
  Serial.println(); 

  //The IMU is shut off when not needed to conserve power. The IMU Vcc is hooked up to the SENSORPOWERPIN
  pinMode(IMU_POWERPIN, OUTPUT);
  IMUPower(false); //Make sure IMU is off.

  // For the future...
  /*uint16_t vcc = ESP.getVcc();
  Serial.print("Vcc="); Serial.println(vcc);*/

  // WiFi Setup
  WiFi.mode(WIFI_STA); // Station mode.
  WiFiMulti.addAP(ssid1, password1); 
  //WiFiMulti.addAP(ssid2, password2); // You can add additional access points, but this adds more run time = more battery drain. Less is more.

}


void loop()
{
    // Collect samples and average them, results in global values XAve, YAve
    CollectSamples();
   
    // Output the averaged data, suitable for SerialPlot. This is output is optional but nice to see what is going on with the IMU.
    // NOTE: Expected values on a level surface, will be close to zero.
    Serial.print(XAve,6);
    Serial.print(",");
    Serial.println(YAve,6);
    Serial.println();

    //Send Data to Webserver
    ReportData();

#if DEEPSLEEP
    //Dignostic output - remove when not needed.
    DEBUGOUT("Nighy nite ESP  @");
    DEBUGOUTLN(millis());


    ESP.deepSleep(SLEEPSECONDS * 1000000, WAKE_RF_DEFAULT);

    delay(500);   // wait for deep sleep to happen
#else
    DEBUGOUT.print("Taking a nap @");
    DEBUGOUTLN(millis());
#endif
    delay(SLEEPSECONDS * 1000); // N Sec

   
}

void ReportData()
{

    DEBUGOUT("Reportdata() @");
    DEBUGOUTLN(millis());


    // Wait for the Wi-Fi to connect: scan for Wi-Fi networks, and connect to the strongest of the networks above
    while (WiFiMulti.run() != WL_CONNECTED) { 
        delay(200);

        DEBUGOUT('.');
    }

    if ((WiFiMulti.run() == WL_CONNECTED))
    {
        // It is faster to use a static IP address because the DHCP server does not have to be contacted to get an IP address
        // If you dont know how to or cannot set up static IPs on your net, then comment these lines out.

        IPAddress ip(192, 168, 1, 30);  // Static IP address
        IPAddress gateway(192, 168, 1, 254); // Your Gateway address (i.e. router)
        IPAddress subnet(255, 255, 255, 0); // Your subnet mask. This address is typical for 192. networks.
        WiFi.config(ip, gateway, subnet);

        DEBUGOUTLN("CONNECTED");
        DEBUGOUTLN('\n');
        DEBUGOUT("Connected to ");
        DEBUGOUTLN(WiFi.SSID());              // Tell us what network we're connected to
        DEBUGOUT("IP address:\t");
        DEBUGOUTLN(WiFi.localIP());


        // Use WiFiClient class to create a TCP to the webserver
        WiFiClient client;
        const int httpPort = 80;
        if (!client.connect(host, httpPort)) {
            DEBUGOUTLN("connection failed");
            DEBUGOUT("Host="); DEBUGOUT(host); DEBUGOUTLN(":");
            DEBUGOUT("Port="); DEBUGOUTLN(httpPort);
            DEBUGOUT("Status="); DEBUGOUTLN(client.status());
            return;
        }

        // Create URL request
        String url = "/XYDataPoint.aspx?m=";
        url += StationMAC;
        url += "&x=";
        url += XAve;
        url += "&y=";
        url += YAve;

        DEBUGOUT("URL: ");
        DEBUGOUTLN(url);
        // This will send the request to the server
        client.print(String("GET ") + url + " HTTP/1.1\r\n" +
            "Host: " + host + "\r\n" +
            "Connection: close\r\n\r\n");

        delay(100);  //Alow time for server to respond?

        
#if 0  // Change this value to 1 if you want to wait for the webserver to respond and show the returned response. Only do this for 
       // debugging because waiting on the server adds more runtime and battery consumption.
        // Read all the lines of the reply from server and print them to Serial
        while(client.available()){
          String line = client.readStringUntil('\r');
          Serial.println(line);
        }
        
        Serial.println();
#endif


    }
    else // Something went wrong.
    {
        DEBUGOUTLN("CONNECT FAILED");
    }


    DEBUGOUT("Report Done @");
    DEBUGOUTLN(millis());  // Show runtime time
}


//
//  Initialize the LIS3DH IMU sensor.
//
void StartIMU()
{
    IMUPower(true); // Power up IMU

    // Config IMU
    myIMU.settings.adcEnabled = 0;   // Dont need adc
    myIMU.settings.tempEnabled = 1;  // 

    // Run at 1600hz sample rate so we can collect our samples quickly. 5000hz will probably be good, but not tested.
    myIMU.settings.accelSampleRate = 1600;  //Hz.  Can be: 0,1,10,25,50,100,200,400,1600,5000 Hz
    myIMU.settings.accelRange = 2;     // 2g Will give the best resolution in this application.  //G force resolution.  Values 2g, 4g, 8g, 16g
    myIMU.settings.xAccelEnabled = 1;
    myIMU.settings.yAccelEnabled = 1;
    myIMU.settings.zAccelEnabled = 1; // Not using Z axis.

    //Start IMU
    myIMU.begin();
}

// Turn on/off power to the IMU board. OnOff= True is On.
void IMUPower(bool OnOff)
{
    if (OnOff)
    {
    digitalWrite(IMU_POWERPIN, HIGH);
    delay(500); // Allow for a little bit of start up time for the board to be ready.  TODO: Fine tune this number, probably too large.
    }
    else
    {
        digitalWrite(IMU_POWERPIN, LOW);
    }
}

// Collect NSAMPLES worth of X/Y data and average the samples.
void CollectSamples()
{
    DEBUGOUT("Collecting data @");
    DEBUGOUTLN(millis());

    StartIMU(); // Get the IMU running

    // Init average values
    XAve = 0;
    YAve = 0;
    
    // Read samples and sum the values
    for (int i = 0; i < NSAMPLES; i++)
    {
        XAve += myIMU.readFloatAccelX();
        YAve += myIMU.readFloatAccelY();
        
        delay(1); // Wait for new sample, Sensor is running at 1600Hz, so sample time is actually less than 1 ms
        // ESP8266 needs delay() or yield() to allow it to manage the network connection, so this also helps the ESP8266 run properly.
    }

    // Calc averages
    XAve /= (float) NSAMPLES;  
    YAve /= (float)NSAMPLES;
    
    IMUPower(false); // Turn off sensor, we are done with it.
}

