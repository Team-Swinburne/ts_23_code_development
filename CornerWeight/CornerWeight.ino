/*
 Example using the SparkFun HX711 breakout board with a scale
 By: Nathan Seidle
 SparkFun Electronics
 Date: November 19th, 2014
 License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

 This is the calibration sketch. Use it to determine the calibration_factor that the main example uses. It also
 outputs the zero_factor useful for projects that have a permanent mass on the scale in between power cycles.

 Setup your scale and start the sketch WITHOUT a weight on the scale
 Once readings are displayed place the weight on the scale
 Press +/- or a/z to adjust the calibration_factor until the output readings match the known weight
 Use this calibration_factor on the example sketch

 This example assumes pounds (lbs). If you prefer kilograms, change the Serial.print(" lbs"); line to kg. The
 calibration factor will be significantly different but it will be linearly related to lbs (1 lbs = 0.453592 kg).

 Your calibration factor may be very positive or very negative. It all depends on the setup of your scale system
 and the direction the sensors deflect from zero state
 This example code uses bogde's excellent library: https://github.com/bogde/HX711
 bogde's library is released under a GNU GENERAL PUBLIC LICENSE
 Arduino pin 2 -> HX711 CLK
 3 -> DOUT
 5V -> VCC
 GND -> GND

 Most any pin on the Arduino Uno will be compatible with DOUT/CLK.

 The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power should be fine.

*/

#include "HX711.h"

#define CLK  2

HX711 scale;

HX711 fr;
HX711 fl;
HX711 rr;
HX711 rl;

float calibration_factor = 6920; //-7050 worked for my 440lb max scale setup

void setup() {
  Serial.begin(115200);
  Serial.println("Team Swinburne corner weight");
  Serial.println("Set all scale on flat surface");
  Serial.println("Remove all weight from scale");
  // Serial.println("After readings begin, place known weight on scale");
  // Serial.println("Press + or a to increase calibration factor");
  // Serial.println("Press - or z to decrease calibration factor");

  delay(2000);

  fr.begin(3,CLK);
  fl.begin(4, CLK);
  rr.begin(5, CLK);
  rl.begin(6, CLK);

  fr.set_scale(calibration_factor);
  fl.set_scale(calibration_factor);
  rr.set_scale(calibration_factor);
  rl.set_scale(calibration_factor);

  Serial.println("Tare.....");
  fr.tare();
  fl.tare();
  rr.tare();
  rl.tare();

  // long zero_factor = scale.read_average(); //Get a baseline reading
  // Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  // Serial.println(zero_factor);
}

void loop() {

  float fr_weight = fr.get_units();
  float fl_weight = fl.get_units();
  float rr_weight = rr.get_units();
  float rl_weight = rl.get_units();
  float car_weight = fr_weight+fl_weight+rr_weight+rl_weight;

  float front_ratio = (fr_weight+fl_weight)/car_weight;
  float rear_ratio = 1.0 - front_ratio;

  float left_ratio = (fl_weight+rl_weight)/car_weight;
  float right_ratio = 1.0 - left_ratio;

  Serial.print("Car total weight (kg): ");
  Serial.println(car_weight,1);
  Serial.print("FR: ");
  Serial.print(fr_weight,1);
  Serial.print("     FL: ");
  Serial.print(fl_weight,1);
  Serial.print("     RR: ");
  Serial.print(rr_weight,1);
  Serial.print("     RL: ");
  Serial.print(rl_weight,1);
  Serial.println();

  Serial.print("Front/Rear ratio: ");
  Serial.print(front_ratio,2);
  Serial.print("/");
  Serial.print(rear_ratio,2);
  Serial.print("        Left/Right ratio: ");
  Serial.print(left_ratio,2);
  Serial.print("/");
  Serial.print(right_ratio,2);
  Serial.println();

  // Serial.print("Reading: ";
  // Serial.print(scale.get_units(), 1);
  // Serial.print(" kg"); //Change this to kg and re-adjust the calibration factor if you follow SI units like a sane person
  // Serial.print(" calibration_factor: ");
  // Serial.print(calibration_factor);
  // Serial.println();

  // if(Serial.available())
  // {
  //   char temp = Serial.read();
  //   if(temp == '+' || temp == 'a')
  //     calibration_factor += 10;
  //   else if(temp == '-' || temp == 'z')
  //     calibration_factor -= 10;
  // }
}
