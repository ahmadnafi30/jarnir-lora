#include <SPI.h>
#include <LoRa.h>

const long LORA_FREQ = 920200000;
const long LORA_BW = 125E3;
const int LORA_POWER = 10;
const uint8_t SYNC_WORD = 0x34;

int CURRENT_SF = 7;
int CURRENT_CR = 5;

const int LORA_SS = 10;
const int LORA_RST = 9;
const int LORA_DIO0 = 2;

const int TRIG_PIN = 5;
const int ECHO_PIN = 4;
const int LDR_PIN = A0;

unsigned long seq = 0;
const unsigned long TOTAL_PACKETS = 10;
const unsigned long TX_INTERVAL_MS = 15000UL;
unsigned long lastTx = 0;

const double FIXED_DISTANCE = 259.0;

unsigned long totalPacketsSent = 0;
unsigned long totalTimeOnAir = 0;

unsigned long minToA = 999999;
unsigned long maxToA = 0;
unsigned long sumToA = 0;

String nodeID = "";

void setupLoRa();
void setupSensors();
void sendDataPacket();
int readUltrasonicCM();
int readLDR();
unsigned long calculateTimeOnAir(int payloadSize, int sf, long bw, int cr);
void printToAExplanation();
void printCodingRateExplanation();
String getTimestamp();
String getCRString(int cr);
void printFinalStatistics();

void setup() {
  Serial.begin(9600);
  while (!Serial) { }
  delay(1000);

  Serial.println(F("\n-------------------------------------------"));
  Serial.println(F("  LoRa Experiment - Bab 2 Metodologi      "));
  Serial.println(F("  Study Case: Deteksi Kapasitas Sampah    "));
  Serial.println(F("  Static SF + Variable CR + ToA Analysis  "));
  Serial.println(F("  Mode: NO GPS + NO ACK (One-way)          "));
  Serial.println(F("-------------------------------------------\n"));

  randomSeed(analogRead(A5));
  nodeID = "NODE_" + String(random(1000, 9999));

  setupSensors();
  setupLoRa();

  Serial.println(F("\nSystem Ready!"));
  Serial.print(F("Node ID: ")); Serial.println(nodeID);
  Serial.println(F("\nExperiment Parameters:"));
  Serial.print(F("   - Spreading Factor: SF")); Serial.println(CURRENT_SF);
  Serial.print(F("   - Coding Rate: 4/")); Serial.print(CURRENT_CR);
  Serial.print(F(" (")); Serial.print(getCRString(CURRENT_CR)); Serial.println(F(")"));
  Serial.print(F("   - Total Packets: ")); Serial.println(TOTAL_PACKETS);
  Serial.print(F("   - TX Interval: ")); Serial.print(TX_INTERVAL_MS/1000); Serial.println(F(" s"));
  Serial.print(F("   - Fixed Distance: ")); Serial.print(FIXED_DISTANCE); Serial.println(F(" m"));
  Serial.print(F("   - Bandwidth: ")); Serial.print(LORA_BW/1000); Serial.println(F(" kHz"));
  Serial.print(F("   - TX Power: ")); Serial.print(LORA_POWER); Serial.println(F(" dBm"));
  
  printToAExplanation();
  printCodingRateExplanation();
  
  Serial.println(F("\nStarting experiment in 5 seconds..."));
  Serial.println(F("Gateway akan log otomatis ke:"));
  Serial.println(F("   /root/lora_experiment_data.csv"));
  Serial.println(F("   /root/lora_experiment_log.txt\n"));
  
  delay(5000);
  lastTx = millis();
}

void loop() {
  unsigned long now = millis();
  
  if ((now - lastTx >= TX_INTERVAL_MS) || (now < lastTx)) {
    lastTx = now;
    
    if (totalPacketsSent < TOTAL_PACKETS) {
      sendDataPacket();
    } else if (totalPacketsSent == TOTAL_PACKETS) {
      delay(2000); 
      printFinalStatistics();
      totalPacketsSent++; 
      
      Serial.println(F("\nEXPERIMENT COMPLETED!"));
      Serial.println(F("Silakan ambil data dari gateway"));
      Serial.println(F("Untuk parameter berbeda, upload ulang dengan SF/CR baru\n"));
    }
  }
  
  delay(10);
}

void setupSensors() {
  Serial.println(F("--- Sensor Setup ---"));
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);
  Serial.println(F("Sensors configured (HC-SR04 + LDR)"));
}

void setupLoRa() {
  Serial.println(F("\n--- LoRa Setup ---"));
  
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println(F("FATAL: LoRa init FAILED!"));
    while (1) { 
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(200); 
    }
  }
  
  Serial.println(F("LoRa module initialized"));
  
  LoRa.setSpreadingFactor(CURRENT_SF);
  LoRa.setSignalBandwidth(LORA_BW);
  LoRa.setTxPower(LORA_POWER);
  LoRa.setSyncWord(SYNC_WORD);
  LoRa.setCodingRate4(CURRENT_CR);
  LoRa.enableCrc();
  LoRa.setPreambleLength(8);
  
  Serial.print(F("SF: ")); Serial.println(CURRENT_SF);
  Serial.print(F("BW: ")); Serial.print(LORA_BW/1000); Serial.println(F(" kHz"));
  Serial.print(F("TX Power: ")); Serial.print(LORA_POWER); Serial.println(F(" dBm"));
  Serial.print(F("Coding Rate: 4/")); Serial.print(CURRENT_CR);
  Serial.print(F(" (")); Serial.print(getCRString(CURRENT_CR)); Serial.println(F(")"));
}

void sendDataPacket() {
  seq++;
  totalPacketsSent++;
  
  Serial.println(F("\n----------------------------------------"));
  Serial.print(F("TX #")); Serial.print(seq);
  Serial.print(F(" | SF")); Serial.print(CURRENT_SF);
  Serial.print(F(" | CR4/")); Serial.print(CURRENT_CR);
  Serial.print(F(" | Progress: ")); Serial.print(seq);
  Serial.print(F("/")); Serial.println(TOTAL_PACKETS);
  
  String timestamp = getTimestamp();
  Serial.print(F("Timestamp: ")); Serial.println(timestamp);
  
  int distance_cm = readUltrasonicCM();
  int ldr_raw = readLDR();
  float ldr_pct = (ldr_raw / 1023.0) * 100.0;
  
  char payload[120];
  snprintf(payload, sizeof(payload), 
           "%s,%lu,%d,%d,%s,%d,%d,%d",
           nodeID.c_str(), seq, CURRENT_SF, CURRENT_CR, timestamp.c_str(),
           (int)FIXED_DISTANCE, distance_cm, ldr_raw);
  
  int payloadSize = strlen(payload);
  
  unsigned long toa = calculateTimeOnAir(payloadSize, CURRENT_SF, LORA_BW, CURRENT_CR);
  totalTimeOnAir += toa;
  sumToA += toa;
  
  if (toa < minToA) minToA = toa;
  if (toa > maxToA) maxToA = toa;
  
  Serial.print(F("Payload: ")); Serial.println(payload);
  Serial.print(F("Size: ")); Serial.print(payloadSize); Serial.println(F(" bytes"));
  
  Serial.println(F("\n--- Time on Air (ToA) Calculation ---"));
  float Ts = (pow(2, CURRENT_SF) / (LORA_BW / 1000.0));
  Serial.print(F("   Symbol Duration (Ts): ")); Serial.print(Ts, 3); Serial.println(F(" ms"));
  
  int preambleLength = 8;
  float Tpreamble = (preambleLength + 4.25) * Ts;
  Serial.print(F("   Preamble Time: ")); Serial.print(Tpreamble, 2); Serial.println(F(" ms"));
  
  int DE = (CURRENT_SF >= 11) ? 1 : 0;
  float payloadSymbNb = 8 + max(
    (int)ceil((8.0 * payloadSize - 4.0 * CURRENT_SF + 28 + 16 - 20 * 0) / 
              (4.0 * (CURRENT_SF - 2 * DE))) * (CURRENT_CR + 4),
    0
  );
  float Tpayload = payloadSymbNb * Ts;
  Serial.print(F("   Payload Symbols: ")); Serial.print(payloadSymbNb, 1); Serial.println(F(" symbols"));
  Serial.print(F("   Payload Time: ")); Serial.print(Tpayload, 2); Serial.println(F(" ms"));
  
  Serial.print(F("   TOTAL ToA: ")); Serial.print(toa); Serial.println(F(" ms"));
  Serial.print(F("   ToA Formula: Tpreamble + Tpayload = "));
  Serial.print(Tpreamble, 2); Serial.print(F(" + ")); Serial.print(Tpayload, 2);
  Serial.print(F(" = ")); Serial.print(toa); Serial.println(F(" ms"));
  
  Serial.print(F("   CR Impact: Higher CR (4/")); Serial.print(CURRENT_CR);
  Serial.println(F(") = More redundancy = Longer ToA"));
  Serial.println(F("----------------------------------------"));
  
  Serial.print(F("\nDistance: ")); Serial.print(FIXED_DISTANCE); Serial.println(F(" m (Fixed)"));
  Serial.print(F("Sensors: "));
  Serial.print(F("Ultrasonic="));
  if (distance_cm == -1) {
    Serial.print(F("ERROR"));
  } else {
    Serial.print(distance_cm); Serial.print(F("cm"));
  }
  Serial.print(F(", LDR=")); Serial.print(ldr_raw); 
  Serial.print(F(" (")); Serial.print(ldr_pct, 1); Serial.print(F("%)"));
  
  if (ldr_pct > 50) {
    Serial.println(F(" [TUTUP TERBUKA]"));
  } else {
    Serial.println(F(" [TUTUP TERTUTUP]"));
  }
  
  unsigned long txStart = millis();
  LoRa.beginPacket();
  LoRa.print(payload);
  int result = LoRa.endPacket(false);
  unsigned long txTime = millis() - txStart;
  
  if (result == 1) {
    Serial.print(F("\nTX Success | Actual TX Time: ")); Serial.print(txTime); Serial.println(F(" ms"));
    Serial.print(F("   Calculated ToA: ")); Serial.print(toa); 
    Serial.print(F(" ms | Difference: ")); Serial.print(abs((long)txTime - (long)toa)); Serial.println(F(" ms"));
  } else {
    Serial.println(F("TX Failed"));
  }
  
  Serial.println(F("----------------------------------------\n"));
}

int readUltrasonicCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  
  if (duration == 0) return -1;
  
  return (int)((duration * 0.034) / 2.0);
}

int readLDR() {
  int raw = analogRead(LDR_PIN);
  return constrain(raw, 0, 1023);
}

unsigned long calculateTimeOnAir(int payloadSize, int sf, long bw, int cr) {
  float Ts = (pow(2, sf) / (bw / 1000.0));
  int preambleLength = 8;
  float Tpreamble = (preambleLength + 4.25) * Ts;
  int DE = (sf >= 11) ? 1 : 0;
  float npayload_symbols = 8 + max(
    (int)ceil((8.0 * payloadSize - 4.0 * sf + 28 + 16 - 20 * 0) / 
              (4.0 * (sf - 2 * DE))) * (cr + 4),
    0
  );
  float Tpayload = npayload_symbols * Ts;
  return (unsigned long)(Tpreamble + Tpayload);
}

void printToAExplanation() {
  Serial.println(F("\n---------------------------------------------------"));
  Serial.println(F("  1.6 Pengaruh Parameter pada Time on Air (ToA)   "));
  Serial.println(F("---------------------------------------------------"));
  Serial.println(F("\nTime on Air (ToA) adalah total durasi transmisi paket:"));
  Serial.println(F("   ToA = Tpreamble + Tpayload"));
  Serial.println();
  Serial.println(F("Dimana:"));
  Serial.println(F("   Tpreamble = (npreamble + 4.25) x Ts"));
  Serial.println(F("   Tpayload = npayload_symbols x Ts"));
  Serial.println(F("   Ts = 2^SF / BW  (Symbol duration)"));
  Serial.println();
  Serial.println(F("Faktor yang mempengaruhi ToA:"));
  Serial.println(F("   1. Spreading Factor (SF) - ↑SF = ↑ToA"));
  Serial.println(F("   2. Bandwidth (BW) - ↑BW = ↓ToA"));
  Serial.println(F("   3. Payload Size - ↑Size = ↑ToA"));
  Serial.println(F("   4. Coding Rate (CR) - ↑CR = ↑ToA"));
  Serial.println();
  Serial.println(F("ToA penting untuk:"));
  Serial.println(F("   - Duty Cycle compliance (regulasi)"));
  Serial.println(F("   - Battery consumption estimation"));
  Serial.println(F("   - Network capacity planning"));
  Serial.println(F("---------------------------------------------------\n"));
}

void printCodingRateExplanation() {
  Serial.println(F("\n---------------------------------------------------"));
  Serial.println(F("          Pengaruh Coding Rate (CR) pada ToA      "));
  Serial.println(F("---------------------------------------------------"));
  Serial.println(F("\nCoding Rate menambahkan redundancy untuk error correction:"));
  Serial.println();
  Serial.println(F("   CR 4/5 = 20% overhead (paling cepat, paling sedikit proteksi)"));
  Serial.println(F("   CR 4/6 = 50% overhead"));
  Serial.println(F("   CR 4/7 = 75% overhead"));
  Serial.println(F("   CR 4/8 = 100% overhead (paling lambat, paling banyak proteksi)"));
  Serial.println();
  Serial.println(F("Hubungan CR dengan npayload_symbols:"));
  Serial.println(F("   npayload_symbols = ... x (CR + 4)"));
  Serial.println(F("   Contoh: CR 4/5 -> (5+4) = 9"));
  Serial.println(F("           CR 4/8 -> (8+4) = 12"));
  Serial.println();
  Serial.println(F("Trade-off CR:"));
  Serial.println(F("   Higher CR = Higher Robustness (tahan noise/interference)"));
  Serial.println(F("   Higher CR = Higher ToA (lebih lama transmisi)"));
  Serial.println(F("   Higher CR = Higher Power consumption"));
  Serial.println(F("---------------------------------------------------\n"));
}

String getTimestamp() {
  unsigned long ms = millis();
  unsigned long seconds = ms / 1000;
  unsigned long minutes = seconds / 60;
  unsigned long hours = minutes / 60;
  unsigned long days = hours / 24;
  
  char buffer[20];
  
  if (days > 0) {
    snprintf(buffer, sizeof(buffer), "%lud_%02lu:%02lu:%02lu",
             days, hours % 24, minutes % 60, seconds % 60);
  } else {
    snprintf(buffer, sizeof(buffer), "%02lu:%02lu:%02lu",
             hours % 24, minutes % 60, seconds % 60);
  }
  
  return String(buffer);
}

String getCRString(int cr) {
  switch(cr) {
    case 5: return "20% overhead";
    case 6: return "50% overhead";
    case 7: return "75% overhead";
    case 8: return "100% overhead";
    default: return "Unknown";
  }
}

void printFinalStatistics() {
  Serial.println(F("\n\n"));
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F("          FINAL EXPERIMENT STATISTICS            "));
  Serial.println(F("--------------------------------------------------"));
  
  float avgToA = (totalPacketsSent > 0) ? 
                 (sumToA / (float)totalPacketsSent) : 0;
  
  Serial.println(F("\nConfiguration:"));
  Serial.print(F("   Spreading Factor: SF")); Serial.println(CURRENT_SF);
  Serial.print(F("   Coding Rate: 4/")); Serial.print(CURRENT_CR);
  Serial.print(F(" (")); Serial.print(getCRString(CURRENT_CR)); Serial.println(F(")"));
  Serial.print(F("   Bandwidth: ")); Serial.print(LORA_BW/1000); Serial.println(F(" kHz"));
  Serial.print(F("   TX Power: ")); Serial.print(LORA_POWER); Serial.println(F(" dBm"));
  
  Serial.println(F("\nTransmission Statistics:"));
  Serial.print(F("   Total Packets Sent: ")); Serial.println(totalPacketsSent);
  Serial.print(F("   Fixed Distance: ")); Serial.print(FIXED_DISTANCE); Serial.println(F(" m"));
  
  Serial.println(F("\nTime on Air (ToA) Statistics:"));
  Serial.print(F("   Total ToA: ")); Serial.print(totalTimeOnAir); Serial.println(F(" ms"));
  Serial.print(F("   Total ToA: ")); Serial.print(totalTimeOnAir / 1000.0, 2); Serial.println(F(" seconds"));
  Serial.print(F("   Average ToA per packet: ")); Serial.print(avgToA, 2); Serial.println(F(" ms"));
  Serial.print(F("   Min ToA: ")); Serial.print(minToA); Serial.println(F(" ms"));
  Serial.print(F("   Max ToA: ")); Serial.print(maxToA); Serial.println(F(" ms"));
  Serial.print(F("   ToA Range: ")); Serial.print(maxToA - minToA); Serial.println(F(" ms"));
  
  unsigned long experimentDuration = millis();
  float dutyCycle = (totalTimeOnAir * 100.0) / experimentDuration;
  Serial.println(F("\nDuty Cycle Analysis:"));
  Serial.print(F("   Experiment Duration: ")); Serial.print(experimentDuration / 1000.0, 1); Serial.println(F(" s"));
  Serial.print(F("   Duty Cycle Used: ")); Serial.print(dutyCycle, 4); Serial.println(F("%"));
  Serial.print(F("   Regulatory Limit: 1% (Indonesia)"));
  if (dutyCycle < 1.0) {
    Serial.println(F(" COMPLIANT"));
  } else {
    Serial.println(F(" EXCEEDED!"));
  }
  
  Serial.println(F("\nPerformance Metrics:"));
  float throughput = (totalPacketsSent * 8.0 * 10) / (experimentDuration / 1000.0);
  Serial.print(F("   Approx Throughput: ")); Serial.print(throughput, 2); Serial.println(F(" bps"));
  
  float airTimeUtilization = (totalTimeOnAir * 100.0) / experimentDuration;
  Serial.print(F("   Air Time Utilization: ")); Serial.print(airTimeUtilization, 2); Serial.println(F("%"));
  
  Serial.println(F("\nCoding Rate Impact:"));
  Serial.print(F("   Current CR: 4/")); Serial.print(CURRENT_CR);
  Serial.print(F(" (")); Serial.print(getCRString(CURRENT_CR)); Serial.println(F(")"));
  Serial.print(F("   Overhead: ")); 
  float overhead = ((CURRENT_CR - 4) * 100.0) / 4.0;
  Serial.print(overhead, 0); Serial.println(F("%"));
  Serial.println(F("   Higher CR = Better error correction + Longer ToA"));
  
  Serial.println(F("\nRecommendations:"));
  if (dutyCycle > 0.8) {
    Serial.println(F("   Approaching duty cycle limit!"));
    Serial.println(F("   - Consider reducing TX frequency"));
    Serial.println(F("   - Or reduce payload size"));
    Serial.println(F("   - Or use lower CR (e.g., 4/5 instead of 4/8)"));
  } else {
    Serial.println(F("   Duty cycle within safe limits"));
  }
  
  if (CURRENT_CR == 8) {
    Serial.println(F("   Using maximum CR (4/8):"));
    Serial.println(F("   - Best for noisy environments"));
    Serial.println(F("   - Highest ToA - consider if acceptable"));
  } else if (CURRENT_CR == 5) {
    Serial.println(F("   Using minimum CR (4/5):"));
    Serial.println(F("   - Shortest ToA"));
    Serial.println(F("   - Less robust in noisy environments"));
  }
  
  Serial.println(F("\n--------------------------------------------------"));
  Serial.println(F(" Untuk parameter berbeda:                         "));
  Serial.println(F(" 1. Ubah CURRENT_SF (7/9/12)                      "));
  Serial.println(F(" 2. Ubah CURRENT_CR (5/6/7/8)                     "));
  Serial.println(F(" 3. Upload ulang ke Arduino                       "));
  Serial.println(F("--------------------------------------------------\n"));
  
  Serial.println(F("\nCSV Summary (Copy to spreadsheet):"));
  Serial.println(F("SF,CR,Packets,AvgToA(ms),MinToA(ms),MaxToA(ms),TotalToA(ms),DutyCycle(%)"));
  Serial.print(CURRENT_SF); Serial.print(F(","));
  Serial.print(CURRENT_CR); Serial.print(F(","));
  Serial.print(totalPacketsSent); Serial.print(F(","));
  Serial.print(avgToA, 2); Serial.print(F(","));
  Serial.print(minToA); Serial.print(F(","));
  Serial.print(maxToA); Serial.print(F(","));
  Serial.print(totalTimeOnAir); Serial.print(F(","));
  Serial.println(dutyCycle, 4);
}