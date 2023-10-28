const int redPin = 21;
const int greenPin = 23;
const int bluePin = 18;
const int whitePin = 19;
const int yellowPin = 22;

const int redPin2 = 32;
const int greenPin2 = 26;
const int bluePin2 = 25;
const int whitePin2 = 33;
const int yellowPin2 = 27;

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
}

void setColor(int red, int yellow, int green, int white, int blue) {
  analogWrite(redPin, red);
  analogWrite(greenPin, green);
  analogWrite(bluePin, blue);
  analogWrite(yellowPin, yellow);
  analogWrite(whitePin, white);
}
void setColor2(int red, int yellow, int green, int white, int blue) {
  analogWrite(redPin2, red);
  analogWrite(greenPin2, green);
  analogWrite(bluePin2, blue);
  analogWrite(yellowPin2, yellow);
  analogWrite(whitePin2, white);
}

void Color2(int value){

  if(value == 0){
    red = 0;
    yellow = 0;
    green = 0;
    white = 0;
    blue = 0;
  }
    if(value == 1){
    red = 255;
    yellow = 0;
    green = 0;
    white = 0;
    blue = 0;
  }
    if(value == 2){
    red = 0;
    yellow = 255;
    green = 0;
    white = 0;
    blue = 0;
  }
    if(value == 3){
    red = 0;
    yellow = 0;
    green = 255;
    white = 0;
    blue = 0;
  }
    if(value == 4){
    red = 0;
    yellow = 0;
    green = 0;
    white = 255;
    blue = 0;
  }
    if(value == 5){
    red = 0;
    yellow = 0;
    green = 0;
    white = 0;
    blue = 255;
  }
  if(value == 6){
    red = 255;
    yellow = 0;
    green = 255;
    white = 0;
    blue = 255;
  }
}
void loop() {
  if (Serial.available() >= 3) {
int value = Serial.parseInt();
int value2 = Serial.parseInt();
    Color2(value);
    setColor(red, yellow, green, white, blue);
    Color2(value2);
    setColor2(red, yellow, green, white, blue);
    }
  }

