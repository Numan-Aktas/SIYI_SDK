#SIYISDK Python Kütüphanesi Dokümantasyonu
##Genel Bakış
SIYISDK, SIYI kamera sistemleriyle etkileşim kurmak için tasarlanmış kapsamlı bir Python kütüphanesidir. Firmware sürümü istekleri, gimbal kontrolü, zoom işlevleri ve sıcaklık okumaları gibi çeşitli işlemleri kolaylaştırır. Kütüphane, karmaşık soket programlaması ve iş parçacığı detaylarını kapsayarak, SIYI kameraları kontrol etmek için kullanıcı dostu bir arayüz sağlar.

##Kurulum
Sisteminizde Python 3.x'in yüklü olduğundan emin olun. SIYISDK kütüphanesini doğrudan deposundan indirebilir veya git kullanarak klonlayabilirsiniz.

##Temel Kullanım
Özel istek ve get fonksiyonlarına girmeden önce, SIYISDK kütüphanesinin nasıl kullanılacağına dair temel bir özet:

##Başlatma ve Bağlantı:

Kütüphaneyi import edin ve SIYISDK'nın bir örneğini oluşturun. connect() metodunu çağırarak kameraya bağlanın.

from siyisdk import SIYISDK

cam = SIYISDK()
if not cam.connect():
    print("Bağlantı başarısız")
    exit(1)

##İşlemler Yapma:

Gimbal'ı hareket ettirme, zoom yapma veya sıcaklık verilerini almak gibi çeşitli işlemler için SIYISDK sınıfının metodlarını kullanın.

##Bağlantıyı Kapatma:

İşlemleriniz tamamlandığında, kaynakları serbest bırakmak için düzgün bir şekilde bağlantıyı kesin.

cam.disconnect()

##İstek Fonksiyonları
İstek fonksiyonları, SIYI kameraya komut göndermek için kullanılır. Bunlar arasında kamera kontrol komutları, zoom yapma, odaklanma ve gimbal modlarını değiştirme gibi işlemler yer alır.

###Örnekler
Otomatik Odaklama:

cam.requestAutoFocus() Bu komut, kameranın otomatik odaklama özelliğini tetikler.

Zoom İçeri ve Dışarı:

cam.requestZoomIn()
cam.requestZoomOut()

Bu komutlar, kameranın zoom işlevini kontrol eder. requestZoomIn() içeri zoom yapar, requestZoomOut() ise dışarı zoom yapar.

Gimbal Kontrolü:

cam.requestLockMode()
cam.requestFollowMode()
cam.requestFPVMode()

Bu komutlar gimbal'in hareket modunu kilitleme, takip etme veya FPV (Birinci Şahıs Görünümü) modlarına değiştirir.

##Get Fonksiyonları
Get fonksiyonları, firmware sürümü, gimbal pozisyonu veya sıcaklık okumaları gibi SIYI kameradan veri almak için kullanılır.

###Örnekler

Gimbal Firmware Sürümünü Almak:
firmware_version = cam.getGimbalFirmwareVersion()
print(firmware_version)
Gimbal'ın güncel firmware sürümünü alır.


Gimbal Tutumu Almak:
yaw, pitch, roll = cam.getAttitude()
print("Yaw:", yaw, "Pitch:", pitch, "Roll:", roll)
Gimbal'ın güncel yaw, pitch ve roll değerlerini alır.


Sıcaklık Okumaları Almak:
max_temp, max_x, max_y, min_temp, min_x, min_y = cam.getMaxMinTemprature()
print("Maks Sıcaklık:", max_temp, "konum (", max_x, ",", max_y, ")")
print("Min Sıcaklık:", min_temp, "konum (", min_x, ",", min_y, ")")
Kameranın maksimum ve minimum sıcaklık okumalarını ve ilgili koordinatlarını alır.


Mesafe Ölçer Verisini Almak:
range_value = cam.getRangeFinder()
print("Mesafe Ölçer Değeri:", range_value, "metre")
Kameranın mesafe ölçer tarafından ölçülen mesafeyi alır.


Zoom Seviyesini Almak:
zoom_level = cam.getZoomLevel()
print("Geçerli Zoom Seviyesi:", zoom_level)
Kameranın geçerli zoom seviyesini döndürür.


Kutu Sıcaklığını Almak:
startx, starty, endx, endy, max_temp, max_x, max_y, min_temp, min_x, min_y = cam.getBoxTemprature()
print("Kutu Sıcaklığı - Maks Sıcaklık:", max_temp, "Min Sıcaklık:", min_temp)

Kameranın görüş alanındaki belirli bir kutu alanı için sıcaklık verilerini sağlar.

##İleri Düzey Kullanım
Gimbal'ın rotasyonunu ayarlamak veya belirli bir zoom seviyesi talep etmek gibi bazı işlevler, komutlar ve kontroller dizisi gerektirebilir. Örneğin:

Gimbal Rotasyonunu Ayarlamak:

yaw_angle = 30
pitch_angle = -10
cam.setGimbalRotation(yaw_angle, pitch_angle)
Gimbal'ı belirtilen yaw ve pitch açılarına döndürür.

2. Belirli Bir Zoom Seviyesi Talep Etmek:
hedef_zoom = 5.0  # Hedef zoom seviyesi
cam.requestZoomSet(hedef_zoom)
Kameranın zoom'unu belirli bir seviyeye ayarlar.
