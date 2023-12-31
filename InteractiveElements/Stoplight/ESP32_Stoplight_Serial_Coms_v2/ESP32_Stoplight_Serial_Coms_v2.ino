/* PCB Connections
Connector 1      Connector 2     Connector 3      Connector 4
Red 12               15              6                21 
Yellow 14             4            Linked 1           Linked 2
Green 26             18            Linked 1           Linked 2
White  25            19              8                23
Blue 27               5              7                22
*/

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

enum Color {Off, Red, Yellow, Green,White,Blue, Green_White, Green_Blue, Red_White, Red_Blue, All};
Color color;


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

void setColor(int rpin, int ypin, int gpin, int wpin, int bpin) {
  analogWrite(rpin, red);
  analogWrite(gpin, green);
  analogWrite(bpin, blue);
  analogWrite(ypin, yellow);
  analogWrite(wpin, white);
}

void Color_Convert_int(int value){
switch (value){
  case 0:
  color = Off;
  break;
  case 1:
  color = Red;
  break;
    case 2:
  color = Yellow;
  break;
    case 3:
  color = Green;
  break;
    case 4:
  color = White;
  break;
    case 5:
  color = Blue;
  break;
    case 6:
  color = Green_White;
  break;
    case 7:
  color = Green_Blue;
  break;
    case 8:
  color = Red_White;
  break;
    case 9:
  color = Red_Blue;
  break;
    case 10:
  color = All;
  break;
}}

void Color_state(){

switch (color){
    case Off:
    red = 0;
    yellow = 0;
    green = 0;
    white = 0;
    blue = 0;
    break;

  case Red:
    red = red_pwr;
    yellow = 0;
    green = 0;
    white = 0;
    blue = 0;
    break;

case Yellow:
    red = 0;
    yellow = yellow_pwr;
    green = 0;
    white = 0;
    blue = 0;
    break;
  
case Green:
    red = 0;
    yellow = 0;
    green = green_pwr;
    white = 0;
    blue = 0;
    break;

    case White:
    red = 0;
    yellow = 0;
    green = 0;
    white = white_pwr;
    blue = 0;
    break;

    case Blue:
    red = 0;
    yellow = 0;
    green = 0;
    white = 0;
    blue = blue_pwr;
    break;

case Green_White:
    red = 0;
    yellow = 0;
    green = green_pwr;
    white = white_pwr;
    blue = 0;
    break;

case Green_Blue:
    red = 0;
    yellow = 0;
    green = green_pwr;
    white = 0;
    blue = blue_pwr;
    break;
 
 case Red_White:
    red = red_pwr;
    yellow = 0;
    green = 0;
    white = white_pwr;
    blue = 0;
    break;

    case Red_Blue:
    red = red_pwr;
    yellow = 0;
    green = 0;
    white = 0;
    blue = blue_pwr;
    break;
    
    case All:
        red = red_pwr;
    yellow = yellow_pwr;
    green = green_pwr;
    white = blue_pwr;
    blue = blue_pwr;
    break;
default:
break;
  }
}
void loop() {
  if (Serial.available() >= 3) {
    int Pair1 = Serial.parseInt();
    int Pair2 = Serial.parseInt();
    int Pair3 = Serial.parseInt();
    int Pair4 = Serial.parseInt();
    int Pair5 = Serial.parseInt();
    int Pair6 = Serial.parseInt();
    int Pair7 = Serial.parseInt();
    int Pair8 = Serial.parseInt();
    Color_Convert_int(Pair1);
    Color_state();
    setColor(redPin, yellowPin, greenPin, whitePin, bluePin);
    
    Color_Convert_int(Pair2);
    Color_state();
    setColor(redPin2, yellowPin2, greenPin2, whitePin2, bluePin2);

    }
  }