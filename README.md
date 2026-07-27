# Earendil Bot - ROS 2 Otonom Gezgin Sistemi (ARC'26)

Bu çalışma alanı, Anatolian Rover Challenge (ARC'26) otonom gezgin yarışması için Earendil Bot robotuna ait ROS 2 Humble/Jazzy tabanlı yazılım mimarisini, donanım köprülerini ve görev yönetim sistemini içerir.

---

## 1. Genel Sistem Mimarisi ve Çalışma Ortamı

Sistem, robot üzerindeki dağıtık işlemci ve donanım katmanları üzerinde çalışır:

1. **Raspberry Pi 5 (Ana İşlemci / ROS 2 Sunucusu)**:
   - Paketteki tüm aktif ROS 2 düğümleri (donanım köprüleri, navigasyon motoru, görev yöneticisi, hakem köprüsü) Raspberry Pi 5 üzerinde yürütülür.
   
2. **Jetson Nano (Görüntü İşleme Birimi)**:
   - Kameralardan gelen görüntüleri işleyerek ArUco etiketlerini ve ilmenit basalt kayalarını tespit eder.
   - Raspberry Pi 5 ile Ethernet (LAN) üzerinden bağlıdır. Soket iletişimi UDP portları ile sağlanır.

3. **STM32 / H7 Mikrodenetleyici ve Arduino Kartları (Donanım Denetleyicisi)**:
   - Motor sürücüler, enkoderler, IMU, pusula ve kızılötesi sensörleri kontrol eder.
   - Raspberry Pi 5'e seri port (`/dev/ttyUSB1`, 115200 baud) üzerinden bağlanır.

---

## 2. Kod İçi Önemli Notlar ve Kurallar

Kod içerisindeki yorum satırlarından elde edilen kritik bilgiler aşağıda gruplanmıştır:

- **Çalıştırma Konumu ve Script Ayrımı**:
  - `earendil_bot/scripts/` klasöründeki dosyalar (`rock_detector.py`, `aruco_detector.py` vb.) sadece bilgisayar ortamında bağımsız testler için yazılmıştır. Projedeki tüm aktif düğümler `earendil_bot/tests/`, `earendil_bot/bridge/`, `earendil_bot/gps/` ve `earendil_bot/rscp/` dizinlerindedir.

- **Ethernet UDP Portları ve Mesaj Protokolü**:
  - ArUco Alıcısı (`aruco_receiver` / `jetson_aruco_middle.py`): UDP Veri Alımı: Port 5005 | UDP Komut Gönderimi: Port 5006
  - Taş Alıcısı (`rock_receiver` / `jetson_rock_detector.py`): UDP Veri Alımı: Port 5007 | UDP Komut Gönderimi: Port 5008
  - Raspberry Pi 5, Jetson'a 1 Hz sıklıkla `START` mesajı gönderdiği sürece Jetson tespit servisini açık tutar. `STOP` mesajı iletildiğinde Jetson servisi kapatır.

- **Dinamik Aşama (Stage) ve Kaynak Yönetimi**:
  - `rock_receiver`: Stage 1'de pasiftir (Jetson'a `START` yollamaz). Stage 2 (Shackleton Krateri) aktifleştiğinde Jetson'a `START` yollamaya başlar. Stage 3 (Tünel) başladığında Jetson'a `STOP` yollayıp nodu temiz bir şekilde kapatır (shutdown).
  - `aruco_receiver`: Stage 2 (Krater) sırasında Jetson Nano üzerinde GPU/kamera çakışmasını önlemek için pasife geçer ve Jetson'a `STOP` yollar. Stage 1, 3 ve 4 aşamalarında aktiftir.

- **RTK GPS ve RTCM Doğrulama**:
  - `roverRTK.py` düğümü, RF telemetriden ve yer istasyonundan (`baseRTK.py`) gelen RTCM düzeltme paketlerini CRC-24Q algoritması ile doğrular. Bozuk olan paketleri eler, geçerli verileri GPS modülüne yazarak `/gps/fix` (`sensor_msgs/NavSatFix`) yayınlar.

- **Donanım ve Motor Güvenliği (Watchdog)**:
  - `hardware_bridge.py` motor komutu kesildiğinde `motor_watchdog_timeout` süresi sonunda motorlara otomatik durma komutu gönderir. Sensör veri akışı kesildiğinde `sensor_watchdog_timeout` ile sistemi uyarıp diğer sensörlerle çalışmaya devam eder.

- **Üs Giriş/Çıkış Sistemi**:
  - `base_exit.py` üs binasından çıkarken başlangıç noktasını kaydeder. `base_enter.py` ise bu koordinatları ve ArUco hizalamasını kullanarak robotu üsse geri park ettirir.

- **Zirve Arama Algoritması**:
  - `peak_finder.py` (Stage 1), arama merkezinin etrafında 4x ve 2x yarıçaplı dairesel çevre taraması yaparak en yüksek GPS altimetre noktasını tespit eder.

---

## 3. Proje Klasör Yapısı ve Düğüm Rehberi

Projenin `src/` dizini altındaki paketler ve görevleri:

### earendil_bot (Ana Paket)

- **bridge/**:
  - `hardware_bridge.py`: Raspberry Pi 5 ile mikrodenetleyici arasındaki seri haberleşme düğümüdür. Enkoder, IMU, Manyetometre, IR verilerini okur; Odometri (`/odom`), IMU (`/imu/data_raw`), Pusula (`/mag/heading`) ve IR (`/ir_top`) yayınlar. `/cmd_vel` mesajlarını motor komutlarına çevirir.

- **gps/**:
  - `roverRTK.py`: Telemetri ve RTK GPS sürücüsüdür.
  - `gps_navigator_node.py`: ARC'26 otonom navigasyon sürücüsüdür. Çift fazlı sürüş (Yerinde Dönüş -> İleri Sürüş/Şerit Takibi) ve rampa kontrollü yavaşlama uygular.
  - `gps_math.py`: Haversine mesafe, azimut (bearing) ve açısal hata matematiksel fonksiyonlarını içerir.

- **rscp/**:
  - `rscp_bridge_node.py`: Hakem sistemi ile RS-232 COBS/Protobuf formatında haberleşir. Gelen istekleri ROS 2 topic'lerine çevirir; durum, ACK ve koordinat geri bildirimlerini hakeme iletir.
  - `mission_manager_node.py`: Ana görev yöneticisidir. Hakem komutlarına göre aşama durum makinesini (Stage 1-4) yönetir ve alt düğümleri tetikler.
  - `rscp_serial_handler.py`: Protobuf serileştirme ve COBS paketleme katmanıdır.

- **tests/**:
  - `aruco_receiver.py`: Jetson Nano'dan Ethernet üzerinden ArUco tespitlerini alan düğümdür.
  - `rock_receiver.py`: Jetson Nano'dan Ethernet üzerinden ilmenit basalt taş tespitlerini alan ve görevi bitince kendini kapatan düğümdür.
  - `peak_finder.py`: Stage 1 Zirve Arama görevidir.
  - `tunnel_test5.py`: Stage 3 Lava Tube / Tünel keşif görevidir. 2D LiDAR ve ArUco ile otonom duvar takibi yapar.
  - `base_enter.py`: Stage 4 Airlock otonom park etme görevidir.
  - `base_exit.py`: Üs binasından otonom çıkış görevidir.
  - `heading_test.py` / `gps_nav_test.py`: Kalibrasyon ve test düğümleridir.

- **config/**:
  - `hardware_params.yaml`: Donanım portları, IP adresleri, PID parametreleri ve aşama konfigürasyonlarını içerir.
  - `tunnel_params.yaml`: Tünel duvar takibi parametrelerini içerir.
  - `test_params.yaml`: Test parametrelerini içerir.

- **launch/**:
  - `rscp_hardware.launch.py`: Tüm yarış sistemini ve otonom modülleri başlatan ana launch dosyasıdır.
  - `h7_hardware.launch.py`: LiDAR, TF ve Donanım Köprüsü'nü başlatan temel donanım launch dosyasıdır.
  - `tunnel_hardware.launch.py`: Tünel görevi donanım bileşenlerini başlatır.
  - `rtk_test.launch.py`: RTK GPS testlerini başlatır.
  - `tunnel_bringup.launch.py`: LiDAR ve tünel otonom düğümünü başlatır.

- **hardware_check.py**:
  - LiDAR, IR, Odometri, ArUco Kamera, Taş Kamerası, IMU (`/imu/data_raw`), Pusula (`/mag/heading`), GPS Fix (`/gps/fix`) ve RSCP Görev Aşaması veri akış sürelerini ve gecikmelerini terminal ekranında canlı kontrol eden sağlık denetim düğümüdür.

### earendil_hardware
Robotun mikrodenetleyici donanım yazılımlarını, Jetson Nano üzerinde çalışan servis scriptlerini ve RTK Yer İstasyonu yazılımını içerir:

- **arduino/**:
  - `mega.ino`: Arduino Mega mikrodenetleyicisi için donanım sürücüsüdür.
  - `magneto+imu+engine.ino`: Pusula (Manyetometre), IMU ve Motor sürücü denetimini yapan birleşik donanım kodudur.
  - `magneto+engine.ino`: Pusula ve motor sürücü test kodudur.
  - `gpstest.ino`: GPS ve motor test kodudur.

- **script/**:
  - `jetson_aruco_middle.py`: Jetson Nano üzerinde bir servis olarak çalışan ArUco etiket tespit kodudur. Raspberry Pi 5 ile UDP (5005/5006) üzerinden haberleşir.
  - `jetson_rock_detector.py`: Jetson Nano üzerinde bir servis olarak çalışan ilmenit basalt kayası tespit kodudur. Raspberry Pi 5 ile UDP (5007/5008) üzerinden haberleşir.
  - `baseRTK.py`: Sabit yer istasyonu (RTK Base Station) için RTCM düzeltme verisi yayınlayan telemetri yazılımıdır.

### ldlidar_stl_ros2
- STL27L 2D LiDAR sensörünün ROS 2 sürücüsüdür. `/scan` (`sensor_msgs/LaserScan`) topic'i yayınlar.

---

## 4. Başlatma Komutları

Yarış sistemini tüm bileşenleriyle derleyip başlatmak için:

```bash
colcon build --packages-select earendil_bot ldlidar_stl_ros2
source install/setup.bash
ros2 launch earendil_bot rscp_hardware.launch.py
```

Sadece temel donanım ve sensörleri başlatmak için:

```bash
ros2 launch earendil_bot h7_hardware.launch.py
```

Donanım sağlık kontrolünü çalıştırmak için:

```bash
ros2 run earendil_bot hardware_check
```
