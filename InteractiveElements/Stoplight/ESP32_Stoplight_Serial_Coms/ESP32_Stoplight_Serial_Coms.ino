#define redPin 21
#define greenPin 23
#define bluePin 18
#define whitePin 19
#define yellowPin 22
#define redPin2 32
#define greenPin2 26
#define bluePin2 25
#define whitePin2 33
#define yellowPin2 27

#define green_pwr 120
#define yellow_pwr 255
#define blue_pwr 120
#define white_pwr 120
#define red_pwr 255

  int red;
  int yellow;
  int green;
  int white;
  int blue;

int timer = 0;
void setup() {
  Serial.begin(9600);
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(yellowPin, OUTPUT);
  pinMode(whitePin, OUTPUT);
  pinMode(redPin2, OUTPUT);
  pinMode(greenPin2, OUTPUT);
  pinMode(bluePin2, OUTPUT);
  pinMode(yellowPin2, OUTPUT);
  pinMode(whitePin2, OUTPUT);
}

void colorState(){
if(red == 1){ red = red_pwr;}
else red =0;
if(yellow == 1){ yellow = green_pwr;}
else green =0;
if(green == 1){ green = green_pwr;}
else green =0;
if(white == 1){ white = white_pwr;}
else white =0;
if(blue == 1){ blue = blue_pwr;}
else blue =0;
}
void setColor(int rpin, int ypin, int gpin, int wpin, int bpin) {
  analogWrite(rpin, red);
  analogWrite(gpin, green);
  analogWrite(bpin, blue);
  analogWrite(ypin, yellow);
  analogWrite(wpin, white);
}



void loop() {
  if (Serial.available() >= 11) {
int red = Serial.parseInt();
int yellow = Serial.parseInt();
int green = Serial.parseInt();
int white = Serial.parseInt();
int blue = Serial.parseInt();
    colorState();
    setColor(redPin, yellowPin, greenPin, whitePin, bluePin);
    
     red = Serial.parseInt();
 yellow = Serial.parseInt();
 green = Serial.parseInt();
 white = Serial.parseInt();
 blue = Serial.parseInt();
    colorState();
    setColor(redPin2, yellowPin2, greenPin2, whitePin2, bluePin2);

    }
  }