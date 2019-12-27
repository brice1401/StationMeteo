
void setup() {
  pinMode(13, OUTPUT);
  pinMode(3, INPUT);
  Serial.begin(9600);
  }

// Boucle principale:
void loop() {
  int BP = digitalRead(3); // Lecture du capteur
  if (BP == LOW) {
    digitalWrite(13, HIGH); // Allume la Led
    
  }
  else {
    digitalWrite(13, LOW); // Eteind la Led
    Serial.println("detection");
    }
  
  }
