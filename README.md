# 🔐 Sistem de acces cu RFID (RFID Access Control System)

## 📌 Descriere generală

Acest proiect implementează un **sistem embedded de control al accesului în doi pași**, bazat pe:
- **card RFID**
- **cod PIN introdus prin tastatură 4×4**

Sistemul este realizat pe platforma **FRDM-KL25Z** și oferă autentificare sigură, feedback vizual în timp real și acționare fizică a unui mecanism de blocare. Proiectul demonstrează integrarea mai multor periferice hardware într-o arhitectură embedded modulară și scalabilă.

---

## 🎯 Obiectivele proiectului

- Implementarea unui sistem de control al accesului bazat pe **RFID + PIN**
- Integrarea interfețelor hardware: **SPI, I2C, UART, GPIO**
- Afișarea informațiilor de stare pe **OLED**
- Controlul unui **electromagnet** pentru blocare/deblocare
- Utilizarea temporizărilor neblocante prin **SysTick**
- Extinderea sistemului cu **Wi-Fi** și **loguri de evenimente**

---

## 🧠 Arhitectura sistemului

Sistemul este construit în jurul microcontrolerului **FRDM-KL25Z**, care coordonează toate modulele hardware.

### Componente hardware principale

| Componentă | Rol |
|-----------|-----|
| FRDM-KL25Z | Microcontroler (MCU) |
| RFID MFRC522 | Citire carduri RFID |
| Tastatură 4×4 | Introducere cod PIN |
| OLED SSD1306 | Interfață vizuală |
| LED roșu / verde | Semnalizare stare |
| Electromagnet + MOSFET | Mecanism de blocare |
| Buton fizic | Acces manual |
| Modul Wi-Fi (ESP8266) | Monitorizare în rețea |
| Modul microSD | Stocare loguri |

---

## 🔌 Interfețe utilizate

| Interfață | Componentă |
|----------|----------|
| SPI | RFID MFRC522, microSD |
| I2C | OLED SSD1306 |
| UART | FRDM ↔ Arduino ↔ ESP8266 |
| GPIO | Tastatură, LED-uri, electromagnet |
| SysTick | Temporizări neblocante |

---

## 🔑 Fluxul de autentificare

1. Utilizatorul apropie cardul RFID
2. UID-ul este citit și validat
3. Pe OLED apare mesajul **„Introduceți PIN”**
4. Utilizatorul introduce PIN-ul pe tastatură
5. PIN-ul este afișat în timp real
6. Sistemul validează **UID + PIN**
7. Dacă autentificarea reușește:
   - LED verde ON
   - electromagnet activ
8. După timeout → revenire în starea de repaus

---

## 🖥️ Interfața cu utilizatorul

- **OLED** – mesaje de stare (`PIN OK`, `PIN WRONG`)
- **LED verde** – acces permis
- **LED roșu** – acces blocat
- **Tastatură 4×4**
  - Cifre: introducere PIN
  - `A`: RESET PIN
  - `C`: ENTER / confirmare

---

## 🌐 Monitorizare prin Wi-Fi

Sistemul transmite mesaje de stare prin UART către un Arduino, care:
- controlează un modul **ESP8266**
- rulează un **server web local**
- afișează logurile de acces într-o pagină HTML

Funcționalități:
- monitorizare în timp real
- acces din browser
- fără servicii cloud externe

---

## 🧱 Design mecanic

Carcasa sistemului a fost proiectată în **Autodesk Fusion 360** și include:
- suport pentru placa FRDM-KL25Z
- decupaje pentru OLED, tastatură și cititor RFID
- organizare ergonomică a interfeței utilizator

Designul permite montaj stabil, mentenanță ușoară și un aspect compact.

---

## ⚙️ Tehnologii utilizate

- Limbaj: **C (bare-metal)**
- Platformă: **FRDM-KL25Z**
- CAD: **Autodesk Fusion 360**
- Comunicație: SPI, I2C, UART
- Temporizare: SysTick
- Rețea: ESP8266 (AT Commands)

---
