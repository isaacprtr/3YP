#include <Encoder.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>

//Pins M1 = 11, M2 = 10, Enc1 A,B = 2,4, Enc2 A,B = 3,5, Index Enc1 = 9, Index Enc2 = 6, Pot = A0

//#defines for the I/O pins
#define MOTOR1_PIN 11
#define MOTOR2_PIN 10
#define potpin A0

//use #defines for the OLED
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixel

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)

#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//Encoder1 and Encoder2
Encoder encoder1(4, 2);
Encoder encoder2(3, 5);

//declare global variables
long position1, position2, RPM1, RPM2 = 0;
int motor1Pos, potValue, motor2Pos= 0;
float b, c, v = 0;

//PID parameters
double Kp=0.25, Ki=1, Kd=0; 
double Setpoint, Output, Input;

//PID libary
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  //set pinModes
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT);
  pinMode(9, INPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(potpin, INPUT);

  //Baud Rate
  Serial.begin(9600);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
//enable display
   display.display();

 //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //sets PID output limits
  myPID.SetOutputLimits(0, 1023);
}

void loop() {
//PID loop Input, Output and Setpoint
Input = RPM2;
Setpoint = RPM1;
//compute PID sums
myPID.Compute();
//write the analogvalue/4 to the 2nd Motor Pin
analogWrite(MOTOR2_PIN, Output/4);

//PWM Subroutine
PWM();
//Encoder position Subroutine
encposition();
//Calculating RPM
RPMcalculator();
//OLED
OLED();
//Function to close the gaps (synchronise the position)
changinggap();

//Display to serial plotter
Serial.print(potValue);
Serial.print(",");
Serial.print(motor1Pos);
Serial.print(",");
Serial.print(motor2Pos);
Serial.print(",");
Serial.print(RPM1);
Serial.print(",");
Serial.println(RPM2);


}

void PWM(void){
  //take the PWM analog value from a potentiometer attached on pin A0
  potValue = analogRead(A0);
  //drive 1st motor with analog value/4 to convert to digital value
  analogWrite(MOTOR1_PIN, (potValue/4));
  //make sure the 2nd motor stops when the 1st motor is not moving
  if (potValue <= 10){
    analogWrite(MOTOR2_PIN, LOW);
    delay(10);
    }
}

void encposition(void){
  //read current position value in degrees
  position1 = encoder1.read();
  position2 = encoder2.read();
  //map the 0-1023 voltage value to 0-360 degrees
  motor1Pos = map(position1, 0, 1023, 0, 360);
  motor2Pos = map(position2, 0, 1023, 0, 360);
  //make sure encoder position inverts for 2nd encoder
  motor2Pos = -motor2Pos;
//reset encoder position if the position goes larger than 360 degrees or index pin 9 goes high
  if (motor1Pos>359||digitalRead(9)==1){
    encoder1.write(0);
      b = b + 1;
  }
  //reset encoder position if the position goes larger than 360 degrees or index pin 6 goes high
  if (motor2Pos>359||digitalRead(6)==1){
    encoder2.write(0);
     c = c + 1;
  }
}

void RPMcalculator(void){
  //records on and off time for a pin attached to encoder 1 and 2 (in this case 4 and 3)
long ontime1 = pulseIn(4, HIGH);
long offtime1 = pulseIn(4, LOW);
long ontime2 = pulseIn(3, HIGH);
long offtime2 = pulseIn(3, LOW);
//calculates frequency from on and off time
long frequency1 = (1000000/(ontime1 + offtime1));
long frequency2 = (1000000/(ontime2 + offtime2));
//calculates frequency from 60f/PPR
RPM1 = ((frequency1*60)/500);
RPM2 = ((frequency2*60)/500);
}


void changinggap(void){
  //changes setpoint based on positional difference between M1 & M2
if ((motor2Pos-motor1Pos) > 45 && (motor2Pos-motor1Pos) < 5){
  Setpoint = Setpoint * 0.8;
}  
if ((motor1Pos-motor2Pos) > 45 && (motor1Pos-motor2Pos) < 5){
  Setpoint = Setpoint * 1.2;
}  
if ((motor1Pos-motor2Pos) > 180 && (motor1Pos-motor2Pos) < 45){
  Setpoint = Setpoint * 1.4;
}  
if ((motor1Pos-motor2Pos) > 181){
  Setpoint = Setpoint * 2;
} 
}


void OLED(void){
  //Display information
  //Clears the Display
  display.clearDisplay(); 
  //Sets the text size
  display.setTextSize(1);
  //Sets the text colour
  display.setTextColor(SSD1306_WHITE);
  //Draw rectangles
  display.drawRect(0, 0, 128, 22, SSD1306_WHITE);
  display.drawRect(0, 24, 128, 22, SSD1306_WHITE);
  display.drawRect(0, 48, 128, 12, SSD1306_WHITE);
  //Start writing tex
  display.setCursor(3,3); // Start at top-left corner
  display.println(F("Position 1")); // Print first line
  display.setCursor(3,13); // Start at top-left corner
  display.print(motor1Pos); //Print next line
  //print degree sign 
  display.print((char)247); //Print next line
  display.setCursor(3,26); // Start at top-left corner

  display.println(F("Position 2")); // Print first line
  display.setCursor(3,36); // Start at top-left corner
  display.print(motor2Pos); //Print next line
  display.print((char)247); //Print next line

  display.setCursor(76,3); // Start at top-left corner
  display.println(F("M1 Speed")); // Print first line
  display.setCursor(76,13); // Start at top-left corner
  display.print(RPM1); //Print next line
  display.print("RPM");
  
  display.setCursor(76,26); // Start at top-left corner
  display.println(F("M2 Speed")); // Print first line
  display.setCursor(76,36); // Start at top-left corner
  display.print(RPM2); //Print next line
  display.print("RPM");

  display.setCursor(3,50); // Start at top-left corner
  display.print(F("Difference: ")); // Print first line
  display.print(motor1Pos-motor2Pos); //Print next line
  display.print((char)247); //Print next line
  display.display();
  
}
