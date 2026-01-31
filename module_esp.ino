#include <SoftwareSerial.h>

// ==========================================
// CONFIGURARE WIFI (VERIFICA SA FIE CORECTE)
// ==========================================
String AP_SSID = "DIGI-DFFFA";     
String AP_PASS = "cyberman1234";   
// ==========================================

SoftwareSerial frdm(8, 9); // RX, TX
SoftwareSerial esp(2, 3);  // RX, TX

const int MAX_LOGS = 8;
String logs[MAX_LOGS]; 

void setup() {
  Serial.begin(9600);
  frdm.begin(9600);
  esp.begin(9600);

  for(int i=0; i<MAX_LOGS; i++) logs[i] = "-";

  Serial.println(F("--- START SYSTEM DIAGNOSTIC ---"));
  
  // 1. Resetare ESP
  Serial.println(F("1. Resetare ESP..."));
  sendCmd(F("AT+RST\r\n"), 3000);
  sendCmd(F("AT+CWMODE=1\r\n"), 1000);
  
  // 2. Conectare
  Serial.print(F("2. Conectare la ")); Serial.println(AP_SSID);
  String cmd = "AT+CWJAP=\"" + AP_SSID + "\",\"" + AP_PASS + "\"\r\n";
  esp.print(cmd);
  
  // Asteptam maxim 15 secunde conectarea
  long start = millis();
  while(millis() - start < 15000) {
    while(esp.available()) {
      char c = esp.read();
      // Serial.write(c); // Uncomment daca vrei sa vezi tot ce zice ESP
    }
  }
  
  // 3. Configurare Server
  Serial.println(F("3. Pornire Server..."));
  sendCmd(F("AT+CIPMUX=1\r\n"), 1000);
  sendCmd(F("AT+CIPSERVER=1,80\r\n"), 1000);
  
  Serial.println(F("\n--------------------------------"));
  Serial.println(F("SISTEM ONLINE! Verifica IP-ul mai jos:"));
  sendCmd(F("AT+CIFSR\r\n"), 1000);
  Serial.println(F("--------------------------------"));
  
  esp.listen();
}

void loop() {
  // ----------------------------------------
  // 1. VERIFICAM FRDM (CARD/PIN)
  // ----------------------------------------
  frdm.listen();
  delay(20); 
  
  if (frdm.available()) {
    String raw = frdm.readStringUntil('\n');
    raw.trim();
    if (raw.length() > 3) {
      Serial.print("LOG PRIMIT: ");
      Serial.println(raw); // Debug in consola
      addLog(raw);
    }
  }

  // ----------------------------------------
  // 2. VERIFICAM WEB (ESP)
  // ----------------------------------------
  esp.listen();
  delay(20);
  
  if (esp.available()) {
    if (esp.find("+IPD,")) {
      delay(300); // Asteptam sa intre cererea
      int id = esp.read() - 48;
      
      // Golim bufferul (ignoram request-ul lung)
      while(esp.available()) esp.read();
      
      Serial.println(F("Browser conectat! Trimit pagina..."));
      
      // === TRIMITERE PAGINA (FARA VERIFICARI > CA SA NU SE BLOCHEZE) ===
      
      // 1. Header (Se spune browserului ca e HTML)
      sendData(id, F("HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nConnection: close\r\nRefresh: 5\r\n\r\n"));
      
      // 2. CSS + Start HTML
      String head = F("<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>");
      head += F("<style>body{font-family:Arial,sans-serif;background:#eee;padding:10px}.box{background:#fff;border-radius:10px;padding:0;overflow:hidden;box-shadow:0 2px 5px rgba(0,0,0,0.1)}");
      head += F("h2{background:#007bff;color:#fff;margin:0;padding:15px;text-align:center}table{width:100%;border-collapse:collapse}td{padding:12px;border-bottom:1px solid #eee}");
      head += F(".ok{color:green;font-weight:bold}.fail{color:red;font-weight:bold}</style></head><body>");
      sendString(id, head);

      // 3. Tabel Start
      sendData(id, F("<div class='box'><h2>Monitorizare</h2><table>"));
      
      // 4. Logurile
      for(int i=0; i<MAX_LOGS; i++) {
        String row = "<tr>";
        if(logs[i].indexOf("OK") >= 0) row += "<td class='ok'>&#10004; " + logs[i] + "</td>";
        else if(logs[i].indexOf("FAIL") >= 0) row += "<td class='fail'>&#10060; " + logs[i] + "</td>";
        else row += "<td>" + logs[i] + "</td>";
        row += "</tr>";
        
        sendString(id, row);
      }
      
      // 5. Footer
      sendData(id, F("</table><p style='text-align:center;color:#777'>Live</p></div></body></html>"));
      
      // 6. Inchidere
      String close = "AT+CIPCLOSE=" + String(id) + "\r\n";
      esp.print(close);
      
      Serial.println(F("Gata! Pagina trimisa."));
    }
  }
}

// Functie "Oarba" (Blind Send) - Trimite fara sa astepte confirmare (Mai sigur pt blocaje)
void sendData(int id, const __FlashStringHelper* data) {
  esp.print("AT+CIPSEND=");
  esp.print(id);
  esp.print(",");
  esp.println(strlen_P((const char*)data));
  delay(200); // Pauza fixa - ESP are timp sa proceseze
  esp.print(data);
  delay(100); 
}

void sendString(int id, String data) {
  esp.print("AT+CIPSEND=");
  esp.print(id);
  esp.print(",");
  esp.println(data.length());
  delay(200); // Pauza fixa
  esp.print(data);
  delay(100);
}

void addLog(String newLog) {
  for (int i = MAX_LOGS - 1; i > 0; i--) logs[i] = logs[i - 1];
  logs[0] = newLog;
}

void sendCmd(const __FlashStringHelper* cmd, int wait) {
  esp.print(cmd);
  delay(wait);
  while(esp.available()) Serial.write(esp.read());
}