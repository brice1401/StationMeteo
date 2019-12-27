
void setup() {
  pinMode(13, OUTPUT);
  pinMode(6, INPUT);
  }

// Boucle principale:
void loop() {
  int BP = digitalRead(6); // Lecture du capteur
  if (BP == LOW) {
    digitalWrite(13, HIGH); // Allume la Led
  }
  else {
    digitalWrite(13, LOW); // Eteind la Led
  }
  }
