#define led1 9
#define led2 10

String inputString;

void setup() {
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  while(Serial.available())
  {
    inputString = Serial.readStringUntil('\r');
    Serial.println(inputString);

    int x_axis = inputString.substring(0, inputString.indexOf(',')).toInt();
    int y_axis = inputString.substring(inputString.indexOf(',')+1).toInt();

    int y = map(y_axis, 0, 480, 0, 180);
    int x = map(x_axis, 0, 640, 0, 180);
    
    if( y > 90) digitalWrite(led1,HIGH);
    else digitalWrite(led1, LOW);

    if( y <= 90) digitalWrite(led2,HIGH);
    else digitalWrite(led2, LOW);

     
  }
}
