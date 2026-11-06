# following_study

ROS 2 Humble + Gazebo Classic 11 simülasyonu.

**Araştırma sorusu:** "Kişi takip eden bir robotun takip mesafesinin konfor ve algılanan güvenlik üzerindeki etkisi"

Robot, nötr bir koridorda yürüyen bir kişiyi belirli bir mesafeden takip eder. Kişinin başındaki
geriye bakan kameradan farklı mesafeler için fotoğraf (STEP 5) ve video (STEP 6) üretilir, sistemin
takip başarımı ölçülür (STEP 7).

---

## Gereksinimler

Ubuntu 22.04 (Jammy), ROS 2 Humble, Gazebo Classic 11.

```bash
sudo apt-get install -y ros-humble-desktop
```

```bash
sudo apt-get install -y gazebo ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-cv-bridge ros-humble-robot-state-publisher ros-humble-rosbag2 \
  python3-colcon-common-extensions python3-yaml python3-numpy python3-opencv \
  python3-matplotlib python3-pil python3-pil.imagetk python3-tk ffmpeg
```

(`python3-pil*`/`python3-tk` yalnızca `demo_panel.py` için gereklidir.)

## Kurulum

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <repo_url> following_study
```

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select following_study
```

```bash
source ~/ros2_ws/install/setup.bash
```

Her yeni terminalde `source /opt/ros/humble/setup.bash` ve `source ~/ros2_ws/install/setup.bash`
çalıştırılmalıdır. Aşağıdaki komutların hepsi bunu varsayar.

---

## Tek parametre kaynağı

Tüm sayısal ayarlar `config/study_params.yaml` içindedir; kod ve launch dosyaları değerleri buradan okur:

| Bölüm | İçerik |
|---|---|
| `follower_node` | `target_gap`, kazançlar, `v_max`=2.0 m/s, `w_max`=1.5 rad/s, `steady_tol`=0.02 m, `steady_time`=2 s, gövde ölçüleri (`person_back_offset`=0.20, `robot_front_offset`=0.255) |
| `person_driver` | yürüme hızı (varsayılan 1.0 m/s, 0 = statik), başlama gecikmesi, durma x'i |
| `study` | spawn konumları, fotoğraf mesafeleri/toleransı, video süresi/fps, değerlendirme tekrar sayısı/seed'leri |

Kameranın çözünürlüğü (1280×720), yatay FOV'u (90°), yüksekliği (1.60 m) ve eğimi yalnızca
`models/person/model.sdf` içinde tanımlıdır; scriptler bu değerleri gelen görüntüden okur.

**Mesafe (gap) tanımı** (`following_study/geometry.py`): kişinin yürüme yönü boyunca, robotun **ön
kenarından** kişinin **sırtına** olan mesafe. Fotoğraf/video/değerlendirme scriptleri aynı fonksiyonu
kullanır ve gerçek mesafeyi her zaman ground-truth pozlardan (`/person/odom`, `/robot/odom`) hesaplar.

**Kamera eğimi:** Kişi kamerası geriye (yaw = π) ve 0.80 rad (45.8°) aşağı bakar, başın 4 cm arkasında
ve yerden 1.60 m yüksektedir. 90° yatay FOV ve 16:9 görüntüyle düşey FOV 58.7°'dir. Eğim olmasaydı
robot ancak ~2.2 m'den uzaktayken kadraja girerdi. Bu eğimle robot 0.4 m ile 3.0 m arasındaki
tüm mesafelerde tamamen kadrajdadır ve kişinin kendi gövdesi görünmez.

---

## 1. Simülasyon (elle deneme)

```bash
ros2 launch following_study simulation.launch.py
```

Kişi 3 s sonra 1.0 m/s ile +x yönünde yürümeye başlar, robot `target_gap` = 1.2 m ile takip eder.
Örnek argümanlar (boş bırakılanlar YAML'dan gelir):

```bash
ros2 launch following_study simulation.launch.py target_gap:=0.6 person_speed:=0.0 gui:=false
```

| Argüman | Varsayılan | Açıklama |
|---|---|---|
| `target_gap` | YAML (1.2) | hedef mesafe [m] |
| `person_speed` | YAML (1.0) | kişi hızı [m/s]; follower'ın ileri beslemesi de aynı değeri alır |
| `person_x` | YAML (12.0) | kişinin başlangıç x'i |
| `gui` | `true` | `false` = headless (yalnızca gzserver) |
| `seed` | `0` | Gazebo rastgele sayı seed'i |
| `start_follower` / `start_driver` / `spawn` | `true` | follower, kişi sürücüsü, model spawn |
| `record_bag`, `bag_path` | `false`, zaman damgalı | rosbag2 kaydı (pozlar, komutlar, gap, bumper, /clock) |
| `log_csv` | `''` | follower'ın adım adım CSV logu (pozlar dahil) |

Aynı anda **yalnızca bir `follower_node`** çalışabilir. Sonradan başlatılan follower, daha eski bir
follower görürse kendini kapatır (`/follower/instance`). Scriptler kendi follower'larını kullanır;
bu yüzden onları aşağıdaki launch dosyalarıyla başlatın.

## 2. Fotoğraflar (STEP 5)

```bash
ros2 launch following_study capture_stills.launch.py
```

Headless:

```bash
ros2 launch following_study capture_stills.launch.py gui:=false
```

Gazebo follower olmadan açılır. Script her mesafe için (0.4, 0.6, 0.9, 1.2, 1.5, 2.0, 3.0 m) şunları yapar:
kişiyi (statik) ve robotu yeniden spawn eder, follower'ı başlatır, `steady_state`'i bekler
(|hata| < 0.02 m, 2 s simülasyon zamanı), follower'ı durdurur, 1 s bekler, gerçek mesafeyi pozlardan
hesaplar (±0.03 m dışında kalırsa en fazla 3 kez yeniden dener), sonra çekimi yapar. Bitince her şey kapanır.

Çıktılar `~/ros2_ws/results/stills/` altındadır (`out_dir:=...` ile değiştirilebilir):
`rear_gap_{gap}m.png`, `overview_gap_{gap}m.png`, `capture_log.csv` (hedef/gerçek mesafe, pozlar,
görüntü zaman damgaları, tolerans), `contact_sheet.png` (arka + genel bakış), `follower_gap_*.csv`.

## 3. Videolar (STEP 6)

Her mesafe için ayrı çalıştırın:

```bash
ros2 launch following_study record_clip.launch.py gap:=0.6
```

```bash
ros2 launch following_study record_clip.launch.py gap:=1.2
```

```bash
ros2 launch following_study record_clip.launch.py gap:=2.0
```

Headless için `gui:=false` ekleyin. Kişi x = 3.5 m'den 1.0 m/s ile yürür, follower hedef mesafeyi
tutar. Kayıt, `steady_state` sağlandığında (ya da 8 s ısınmadan sonra) başlar. Kare seçimi görüntü
zaman damgalarına (simülasyon zamanına) göre yapılır, yani video her zaman tam 600 kare, 30 fps ve
20.0 s'dir. Video üzerine yazı basılmaz; kontrol amaçlı yazılı kopya için `record_clip.py` doğrudan
`--annotate` ile çalıştırılabilir.

Çıktılar `~/ros2_ws/results/videos/` altındadır: `clip_gap_{gap}m.mp4`, `clip_gap_{gap}m_frames.csv`
(her kare için zaman ve pozlardan gap), `follower_gap_{gap}m.csv`, `bag_gap_{gap}m/` (rosbag2).

## 4. Sistem değerlendirmesi (STEP 7)

```bash
ros2 run following_study evaluate_system.py
```

Varsayılan: 0.6, 1.2 ve 2.0 m için 10'ar koşu. Her koşu ayrı bir headless Gazebo ile yapılır.
Seed şöyledir: `base_seed + 100·i + tekrar`. Robotun başlangıç pozu bu seed'le üretilen küçük bir
bozulmayla değişir; aynı seed aynı koşuyu üretir. Kısa deneme için:

```bash
ros2 run following_study evaluate_system.py --gaps 1.2 --reps 2
```

Çıktılar `~/ros2_ws/results/eval/` altındadır:
- `eval_results.csv`: mesafe başına RMSE ortalaması/std/maks, çarpışma ve hedef kaybı sayıları
- `eval_runs.csv`: koşu başına seed, bozulma, RMSE (geçici rejim sonrası ve tümü), min gap, temaslar, kayıp süresi
- `gap_vs_time_{gap}m.png`: tüm koşular üst üste
- her koşu için `gap_<g>/rep_<nn>/` (run.csv, follower.csv, bag/, launch.log, grafik)

Metrik tanımları:
- **Çarpışma:** robot gövdesinin kişiye teması (`/robot/bumper`) ya da gap ≤ 0.
- **Hedef kaybı:** kişi, robot kamerasının yarı-FOV'unun (30°) dışına çıkar ya da gap > hedef + 1.5 m olur.
- **RMSE:** 5. saniyeden sonrası için hesaplanır (geçici rejim hariç).

## Demo paneli (isteğe bağlı)

```bash
ros2 launch following_study simulation.launch.py start_driver:=false person_speed:=0.0
```

```bash
ros2 run following_study demo_panel.py
```

Panel kişiyi kendisi yürütür. Yeni bir follower başlatmaz; çalışan follower'ın `target_gap` ve
`v_person` parametrelerini değiştirir.

---

## Önemli topikler

| Topik | Tür | Açıklama |
|---|---|---|
| `/person/rear_cam/image_raw` | sensor_msgs/Image | kişi kamerası (1280×720, 30 Hz) |
| `/overview_cam/image_raw` | sensor_msgs/Image | sabit genel bakış kamerası |
| `/person/odom`, `/robot/odom` | nav_msgs/Odometry | ground-truth pozlar (p3d) |
| `/person/cmd_vel`, `/person/walking` | Twist, Bool | kişi sürücüsü |
| `/cmd_vel` | geometry_msgs/Twist | robot hız komutu (tek follower) |
| `/follower/gap`, `/follower/gap_error`, `/follower/steady` | Float64, Float64, Bool | takip durumu |
| `/robot/bumper` | gazebo_msgs/ContactsState | gövde temasları |
| `/scan`, `/robot/camera/image_raw` | LaserScan, Image | robot sensörleri |

## Bilinen sınırlamalar

- Kişi modeli basit geometridir (silindir gövde + küre baş), Gazebo actor değildir.
- Statik modda kontrolcü geri gidemez (v ∈ [0, v_max]). Robot hedefi aşarsa `steady_state`
  oluşmayabilir; fotoğraf scripti bu durumda yeniden dener.
- `lockstep` açıktır: kameralar simülasyon zamanında tam 30 Hz üretir. Yavaş makinelerde
  simülasyon gerçek zamandan yavaş akar ama kayıtlar etkilenmez.

## Lisans

MIT
