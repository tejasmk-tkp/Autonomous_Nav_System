const int x_axis = A0;
const int y_axis = A1;

void setup() {
  // put your setup code here, to run once:
  pinMode(x_axis, INPUT);
  pinMode(y_axis, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int x = analogRead(x_axis);
  int y = analogRead(y_axis);
  //Serial.print("(");
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  //Serial.println(")");
  delay(2);
}