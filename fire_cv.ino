#define BUZZER_PIN 27
#define MQ2_PIN 5

String incoming = "";


void setup() {
  Serial.begin(115200);
  pinMode(MQ2_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.println("ESP32 Ready for OpenCV commands");
}

void loop() {
  
  int smokeValue = digitalRead(MQ2_PIN);

  Serial.print("Smoke: ");
  Serial.println(smokeValue);

  if (smokeValue == 0) {
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("🚨 FIRE DETECTED");
  } else {
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("✅ SAFE");
  }

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      incoming.trim();

      if (incoming == "FIRE" || incoming == "SMOKE") {
        digitalWrite(BUZZER_PIN, HIGH);
        Serial.println("🚨 ALERT RECEIVED");
      }
      else if (incoming == "SAFE") {
        digitalWrite(BUZZER_PIN, LOW);
        Serial.println("✅ SAFE");
      }

      incoming = "";
    } else {
      incoming += c;
    }
  }
}
