#define PIN7 7
int i=0;
void setup() {
  pinMode(PIN7, OUTPUT);
}
void loop() { 
  digitalWrite(PIN7, HIGH);
  delay(1000);

  while(1){
  if (i<5) {
  digitalWrite(PIN7, LOW);
  delay(100);
  digitalWrite(PIN7, HIGH);
  delay(100);
  i+=1;
  } 
  else{
    digitalWrite(PIN7,LOW);
  }
} // infinite loop
}
 
