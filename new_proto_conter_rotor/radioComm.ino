// ===================== ISR shared state =====================
// IMPORTANT: these are written inside interrupts, so mark them volatile.
volatile unsigned long rising_edge_start_1 = 0, rising_edge_start_2 = 0, rising_edge_start_3 = 0, rising_edge_start_4 = 0;
volatile unsigned long channel_1_raw = 0, channel_2_raw = 0, channel_3_raw = 0, channel_4_raw = 0;


// ===================== Setup helpers =====================
void radioSetup() {
  // Use INPUT_PULLUP only if your receiver output is open-collector / needs pullup.
  // Many RC receivers output a driven 3.3V/5V signal; INPUT (no pullup) is often safer.
  pinMode(ch1Pin, INPUT);
  pinMode(ch2Pin, INPUT);
  pinMode(ch3Pin, INPUT);
  pinMode(ch4Pin, INPUT);

  delay(20);

  attachInterrupt(digitalPinToInterrupt(ch1Pin), getCh1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch2Pin), getCh2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch3Pin), getCh3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ch4Pin), getCh4, CHANGE);
  delay(20);
}

unsigned long getRadioPWM(int ch_num) {
  // Copy volatile data atomically to avoid torn reads (especially on 8-bit AVRs).
  unsigned long v1, v2, v3, v4;
  noInterrupts();
  v1 = channel_1_raw;
  v2 = channel_2_raw;
  v3 = channel_3_raw;
  v4 = channel_4_raw;
  interrupts();

  if (ch_num == 1) return v1;
  if (ch_num == 2) return v2;
  if (ch_num == 3) return v3;
  if (ch_num == 4) return v4;
  return 0;
}

// ===================== ISRs =====================
void getCh1() {
  int trigger = digitalRead(ch1Pin);
  unsigned long now = micros();
  if (trigger == HIGH) {
    rising_edge_start_1 = now;
  } else {
    channel_1_raw = now - rising_edge_start_1; // pulse width in microseconds
  }
}

void getCh2() {
  int trigger = digitalRead(ch2Pin);
  unsigned long now = micros();
  if (trigger == HIGH) {
    rising_edge_start_2 = now;
  } else {
    channel_2_raw = now - rising_edge_start_2;
  }
}

void getCh3() {
  int trigger = digitalRead(ch3Pin);
  unsigned long now = micros();
  if (trigger == HIGH) {
    rising_edge_start_3 = now;
  } else {
    channel_3_raw = now - rising_edge_start_3;
  }
}

void getCh4() {
  int trigger = digitalRead(ch4Pin);
  unsigned long now = micros();
  if (trigger == HIGH) {
    rising_edge_start_4 = now;
  } else {
    channel_4_raw = now - rising_edge_start_4;
  }
}

// ===================== Your logic =====================
void getCommands() {
  channel_1_pwm = getRadioPWM(1);
  channel_2_pwm = getRadioPWM(2);
  channel_3_pwm = getRadioPWM(3);
  channel_4_pwm = getRadioPWM(4);
}
