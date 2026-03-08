# YZR502_ODEV0301_25680114_merve_tutar

# Mobil Robot Hız Kontrolü - PID Denetleyici Analizi

Bu proje, bir mobil robotun doğrusal hızının P, PI, PD ve PID denetleyicileri ile kontrol edilmesini inceleyen bir Python simülasyon çalışmasıdır.

Youtube Link : https://youtu.be/TIv9l4stXiI

Projede aşağıdaki senaryolar ele alınmıştır:
- 1 m/s sabit referans hız takibi
- P, PI, PD ve PID denetleyicilerinin karşılaştırılması
- 5. saniyede uygulanan 10 N büyüklüğünde ve 0.2 s süreli bozucu etkiye karşı sistem davranışı
- 6. saniyede referans hızın 1.0 m/s değerinden 0.5 m/s değerine değiştirilmesi

Simülasyon çıktıları grafikler ve performans ölçütleri üzerinden değerlendirilir. Kod, ödev isterlerine uygun olarak Python dili ile hazırlanmıştır.

## Kullanılan Model

Sistemin dinamiği aşağıdaki diferansiyel denklem ile modellenmiştir:

m dv/dt + b v(t) = F(t)

Burada:
- `m = 10 kg` robot kütlesi
- `b = 5 Ns/m` viskoz sürtünme katsayısı
- `v(t)` robot hızı
- `F(t)` denetleyici tarafından üretilen kontrol kuvveti

## Kullanılan Teknolojiler

- Python 3
- NumPy
- SciPy
- Matplotlib
- Pandas

## Proje Dosyaları

- `script.py`: Ana simülasyon kodu
- `pid_odev_ciktlari/`: Kod çalıştırıldığında oluşan çıktı klasörü
  - denetleyici karşılaştırma grafikleri
  - bozucu etki grafiği
  - referans değişimi grafiği
  - performans ölçütleri

## Kurulum

Gerekli Python paketlerini yüklemek için:

```bash
pip install numpy scipy matplotlib pandas
```

## Kullanım

Aynı klasörde terminal açtıktan sonra aşağıdaki komutu çalıştırın:

```bash
python script.py
```

Kod çalıştırıldığında:
- terminale performans ölçütleri yazdırılır,
- `pid_odev_ciktlari` adlı klasör oluşturulur,
- ilgili figürler ve çıktı dosyaları bu klasöre kaydedilir.

## Beklenen Çıktılar

Kod çalıştırıldığında aşağıdaki türde çıktılar üretilir:
- P, PI, PD ve PID denetleyicileri için referans takip grafiği
- kontrol girdisi grafikleri
- PID denetleyici için bozucu etki cevabı
- PID denetleyici için referans değişimi cevabı
- yükselme süresi, yerleşme süresi, aşma ve kalıcı durum hatası gibi performans ölçütleri

## Kısa Değerlendirme

Genel olarak:
- P ve PD denetleyicileri kalıcı durum hatasını tamamen gideremez.
- PI denetleyicisi kalıcı durum hatasını azaltır ancak aşma miktarı artabilir.
- PID denetleyicisi geçici rejim ve kalıcı durum davranışı arasında daha dengeli bir performans sunar.

## Not

Bu repo, Robotik Sistemler ve Algoritmalar dersi kapsamında verilen mobil robot hız kontrolü ödevine yönelik hazırlanmıştır.
