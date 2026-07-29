// =====================================================
// ARDUINO MEGA ? 1 SAN?YEL?K ROVER YÜRÜYÜ? KONTROLÜ
// =====================================================

// === Sol Motor Sürücü Pinleri ===
const int L_RPWM = 5;
const int L_LPWM = 6;
const int L_REN  = 7;
const int L_LEN  = 12;

// === Sa? Motor Sürücü Pinleri ===
const int R_RPWM = 13;
const int R_LPWM = 10;
const int R_REN  = 11;
const int R_LEN  = 12;

// === PWM H?z De?erleri ===
const int PWM_YAVAS = 80;
const int PWM_HIZLI = 200;

// Her hareket komutunun çal??ma süresi
const unsigned long HAREKET_SURESI_MS = 500;

// Hareketin ne zaman bitece?i
unsigned long hareketBitisZamani = 0;

// Rover hareket ediyor mu?
bool hareketAktif = false;


// =====================================================
// MOTOR KONTROL FONKS?YONLARI
// =====================================================

void dur() {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, 0);

  hareketAktif = false;
}

void hareketSuresiniBaslat() {
  hareketBitisZamani = millis() + HAREKET_SURESI_MS;
  hareketAktif = true;
}

void ileri(int hiz) {
  analogWrite(L_RPWM, hiz);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, hiz);
  analogWrite(R_LPWM, 0);

  hareketSuresiniBaslat();
}

void geri(int hiz) {
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, hiz);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, hiz);

  hareketSuresiniBaslat();
}

void sagaDon(int hiz) {
  // Sol motor ileri, sa? motor geri
  analogWrite(L_RPWM, hiz);
  analogWrite(L_LPWM, 0);

  analogWrite(R_RPWM, 0);
  analogWrite(R_LPWM, hiz);

  hareketSuresiniBaslat();
}

void solaDon(int hiz) {
  // Sol motor geri, sa? motor ileri
  analogWrite(L_RPWM, 0);
  analogWrite(L_LPWM, hiz);

  analogWrite(R_RPWM, hiz);
  analogWrite(R_LPWM, 0);

  hareketSuresiniBaslat();
}


// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);

  pinMode(L_RPWM, OUTPUT);
  pinMode(L_LPWM, OUTPUT);
  pinMode(L_REN, OUTPUT);
  pinMode(L_LEN, OUTPUT);

  pinMode(R_RPWM, OUTPUT);
  pinMode(R_LPWM, OUTPUT);
  pinMode(R_REN, OUTPUT);
  pinMode(R_LEN, OUTPUT);

  // BTS7960 sürücülerini aktif et
  digitalWrite(L_REN, HIGH);
  digitalWrite(L_LEN, HIGH);

  digitalWrite(R_REN, HIGH);
  digitalWrite(R_LEN, HIGH);

  // Ba?lang?çta motorlar? durdur
  dur();

  Serial.println("Rover yuruyus sistemi hazir.");
}


// =====================================================
// LOOP
// =====================================================

void loop() {
  // Hareket süresi dolduysa otomatik dur
  if (hareketAktif &&
      (long)(millis() - hareketBitisZamani) >= 0) {

    dur();
    Serial.println("Hareket tamamlandi, rover durdu.");
  }

  // Seri porttan komut oku
  if (Serial.available() > 0) {
    String veri = Serial.readStringUntil('\n');
    veri.trim();

    if (veri == "ileri_hizli") {
      ileri(PWM_HIZLI);
    }
    else if (veri == "ileri_yavas") {
      ileri(PWM_YAVAS);
    }
    else if (veri == "geri_hizli") {
      geri(PWM_HIZLI);
    }
    else if (veri == "geri_yavas") {
      geri(PWM_YAVAS);
    }
    else if (veri == "sag_hizli") {
      sagaDon(PWM_HIZLI);
    }
    else if (veri == "sag_yavas") {
      sagaDon(PWM_YAVAS);
    }
    else if (veri == "sol_hizli") {
      solaDon(PWM_HIZLI);
    }
    else if (veri == "sol_yavas") {
      solaDon(PWM_YAVAS);
    }
    else if (veri == "dur") {
      dur();
    }
    else {
      Serial.println("Gecersiz komut.");
    }
  }
}