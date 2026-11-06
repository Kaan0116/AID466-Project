# following_study

ROS 2 Humble + Gazebo Classic 11 simülasyonu.

**Araştırma sorusu:** "Kişi takip eden bir robotun takip mesafesinin konfor ve algılanan güvenlik üzerindeki etkisi"

Bir robot, koridorda yürüyen bir kişiyi belirli bir mesafeden takip eder. Farklı mesafeler için birinci şahıs kamera görüntüleri toplanır.

---

## Gereksinimler

### İşletim Sistemi
Ubuntu 22.04 (Jammy)

### ROS 2 ve Gazebo
```bash
# ROS 2 Humble (zaten yüklü değilse)
sudo apt-get install -y ros-humble-desktop

# Gazebo Classic 11 + ROS 2 köprüsü
sudo apt-get install -y gazebo ros-humble-gazebo-ros-pkgs

# Ek ROS bağımlılıkları
sudo apt-get install -y \
  ros-humble-cv-bridge \
  ros-humble-robot-state-publisher \
  ros-humble-xacro

# Python bağımlılıkları
sudo apt-get install -y python3-opencv python3-matplotlib ffmpeg xvfb
```

### Python paketleri (pip)
```bash
pip3 install transforms3d
```

---

## Kurulum

```bash
# 1. Çalışma alanına kopyala
cd ~/ros2_ws/src
git clone <repo_url> following_study

# 2. Derle
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select following_study --cmake-args -DCMAKE_BUILD_TYPE=Release

# 3. Ortamı yükle
source ~/ros2_ws/install/setup.bash
```

---

## Paket Yapısı

```
following_study/
├── config/
│   └── follower_params.yaml      # Kontrol parametreleri
├── following_study/
│   └── follower_node.py          # P-kontrolcü ROS 2 node'u
├── launch/
│   ├── simulation.launch.py      # Ana simülasyon launcher
│   └── capture_stills.launch.py  # Fotoğraf çekme launcher
├── models/
│   └── person/                   # Kişi Gazebo modeli (silindir+küre)
│       ├── model.config
│       └── model.sdf
├── scripts/
│   ├── capture_stills.py         # 7 farklı mesafe için fotoğraf çeker
│   ├── record_clip.py            # 20 s birinci şahıs video kaydeder
│   └── evaluate_system.py        # Sistem değerlendirmesi (RMSE, çarpışma)
├── urdf/
│   └── robot.urdf.xacro          # Jackal benzeri diferansiyel sürücü robot
└── worlds/
    └── corridor.world            # 30m × 2.4m × 2.8m nötr koridor
```

---

## Hızlı Başlangıç

### 1. Sadece simülasyonu başlat (elle test için)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export GAZEBO_MODEL_PATH=$(ros2 pkg prefix following_study)/share/following_study/models:$GAZEBO_MODEL_PATH

ros2 launch following_study simulation.launch.py
```

Parametreler:
```bash
ros2 launch following_study simulation.launch.py target_gap:=1.2 v_person:=0.0
```

### 2. Fotoğraf çekme (STEP 5)

Önce simülasyonu başlat, ardından (ayrı terminalde):

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

python3 $(ros2 pkg prefix following_study)/share/following_study/scripts/capture_stills.py
```

**Çıktılar** `~/ros2_ws/results/stills/` altında:
- `rear_gap_0.4m.png` … `rear_gap_3.0m.png`  — birinci şahıs görüntüler
- `overview_gap_*.png`                         — genel bakış görüntüleri
- `contact_sheet.png`                          — tüm resimlerin montajı
- `capture_log.csv`                            — hedef / gerçek mesafe / tolerans

### 3. Video kaydı (STEP 6)

Simülasyonu `v_person:=1.0` ile başlat, ardından:

```bash
python3 $(ros2 pkg prefix following_study)/share/following_study/scripts/record_clip.py \
  --gap 1.2
```

Desteklenen mesafeler: 0.6, 1.2, 2.0 m  
Çıktı: `~/ros2_ws/results/videos/clip_gap_1.2m.mp4`

### 4. Sistem değerlendirmesi (STEP 7)

```bash
python3 $(ros2 pkg prefix following_study)/share/following_study/scripts/evaluate_system.py \
  --gaps 0.6 1.2 2.0 --reps 10
```

Çıktı: `~/ros2_ws/results/eval/eval_results.csv` + gap-zaman grafikleri

---

## Headless (Ekransız) Çalıştırma

```bash
# Sanal ekran oluştur
export DISPLAY=:99
Xvfb :99 -screen 0 1280x720x24 &

# Sonra normal komutları çalıştır
ros2 launch following_study simulation.launch.py
```

---

## Parametreler

| Parametre     | Varsayılan | Açıklama                                   |
|---------------|------------|--------------------------------------------|
| `target_gap`  | 1.2 m      | Hedef takip mesafesi (robot önü–kişi arkası)|
| `kp_gap`      | 4.0        | Mesafe hatası → hız kazancı                |
| `kp_heading`  | 2.0        | Yön hatası → açısal hız kazancı            |
| `v_max`       | 1.5 m/s    | Maksimum ileri hız                         |
| `v_person`    | 0.0 m/s    | Kişi hızı (0=statik, 1.0=yürüme)          |

---

## Önemli Topikler

| Topik                           | Tür               | Açıklama                       |
|---------------------------------|-------------------|--------------------------------|
| `/person/rear_cam/image_raw`    | sensor_msgs/Image | Birinci şahıs kamera           |
| `/overview_cam/image_raw`       | sensor_msgs/Image | Genel bakış kamerası           |
| `/person/odom`                  | nav_msgs/Odometry | Kişi ground-truth pozisyonu    |
| `/robot/odom`                   | nav_msgs/Odometry | Robot ground-truth pozisyonu   |
| `/follower/gap`                 | std_msgs/Float64  | Anlık takip mesafesi           |
| `/follower/gap_error`           | std_msgs/Float64  | Hedef mesafe hatası            |
| `/follower/steady`              | std_msgs/Bool     | Kararlı hal bayrağı (±0.04m, 2s)|
| `/scan`                         | sensor_msgs/LaserScan | 2D LiDAR                  |
| `/cmd_vel`                      | geometry_msgs/Twist   | Robot hız komutu          |

---

## Bilinen Sınırlamalar

- Kişi modeli basit geometri (silindir + küre) — actor değil.
- P-kontrolcü, kişi yürürken hafif osilasyon gösterebilir; `kp_gap` arttırılabilir.
- Headless modda GPU olmadan render kalitesi düşük ama görüntüler işlevsel.
- Stills çekimi için Gazebo'nun önceden ayakta olması gerekiyor.

---

## Lisans

MIT
