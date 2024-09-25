# SIYI SDK Python Kütüphanesi

**SIYI SDK**, SIYI marka kamera sistemleriyle etkileşim kurmak için oluşturulmuş güçlü bir Python kütüphanesidir. Bu kütüphane, SIYI kamera sistemleriyle haberleşme, gimbal kontrolü, zoom işlemleri, sıcaklık verileri, ve mesafe ölçümleri gibi çok sayıda işlemi kolaylaştırır. Karmaşık soket programlaması ve iş parçacıklarıyla uğraşmak zorunda kalmadan SIYI kameralarını kontrol etmenizi sağlar.

## Özellikler
- Gimbal kontrolü (Yaw, Pitch, Roll ayarları)
- Zoom ve odak kontrolü
- Termal sıcaklık verisi okuma
- Mesafe ölçer verilerini alma
- Firmware versiyon bilgisi
- UDP tabanlı hızlı iletişim protokolü

---

## Kütüphane İçeriği ve Yapısı

### Dosya ve Klasörler

- **siyi_sdk.py**: Kütüphanenin ana dosyası. Tüm gimbal kontrol ve komut işlemleri burada yapılır.
- **siyi_message.py**: Mesaj işleme fonksiyonları, kamera ile haberleşme formatı buradan yönetilir.
- **crc16_python.py**: CRC16 hesaplaması için kullanılan dosya.
- **utils.py**: Yardımcı işlevler, veri türü dönüştürmeleri ve hata ayıklama araçlarını içerir.
- **ZT30 User Manual v1.1.pdf & v1.2.pdf**: Kullanıcı rehberleri.

---

## Kurulum

### Gereksinimler
- Python 3.x
- Ağa bağlı bir SIYI kamera (IP tabanlı iletişim)

### Adımlar
1. Python ortamınızı ayarlayın.
2. Kütüphaneyi indirerek yerel ortamınıza kurun:

   ```bash
   git clone https://github.com/Numan-Aktas/SIYI_SDK.git
   cd SIYI_SDK
   ```

## Kullanım
### Kameraya Bağlanma
Kütüphaneyi kullanarak SIYI kameraya bağlanmak için şu adımları takip edin:

```bash
from siyisdk import SIYISDK

# Kamera bağlantısı oluştur
cam = SIYISDK(server_ip="192.168.144.25", port=37260, debug=True)

# Bağlantı kurulumu
if not cam.connect():
    print("Bağlantı başarısız")
    exit(1)
```
### Gimbal ve Kamera Kontrol Fonksiyonları
#### Gimbal Kontrolü
SIYI SDK, kameranın gimbal kontrolü için çeşitli modlar sunar:

```bash
# Gimbal modları
cam.requestLockMode()     # Kilit modu
cam.requestFollowMode()   # Takip modu
cam.requestFPVMode()      # FPV (First-Person View) modu
```
#### Zoom ve Odak Fonksiyonları
Zoom işlemi yapmak ve kameranın odak kontrolünü sağlamak için aşağıdaki fonksiyonları kullanabilirsiniz:
```bash
# Zoom kontrolleri
cam.requestZoomIn()        # Zoom in
cam.requestZoomOut()       # Zoom out
cam.requestAutoFocus()     # Otomatik odaklama
```
#### Sıcaklık ve Mesafe Verilerini Alma
Kameranın sensörlerinden gelen sıcaklık ve mesafe verilerini çekmek için şu fonksiyonları kullanabilirsiniz:

```bash
# Maksimum ve minimum sıcaklık değerlerini al
max_temp, max_x, max_y, min_temp, min_x, min_y = cam.getMaxMinTemprature()
print(f"Maksimum Sıcaklık: {max_temp}, Minimum Sıcaklık: {min_temp}")

# Mesafe ölçer verisini al
range_value = cam.getRangeFinder()
print(f"Mesafe: {range_value} metre")
```
#### Bağlantıyı Kapatma
İşlemler tamamlandığında bağlantıyı düzgün şekilde kapatmanız gerekmektedir:
```bash
cam.disconnect()
```
