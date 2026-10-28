# README — Pengujian Planning & Obstacle Avoidance CRA

Panduan pengujian **Cognitive Robotic Arm (CRA)** pada **ROS 2 Humble**, **MuJoCo**, dan **RViz** menggunakan graf GNG, **A*** di joint space, serta pemeriksaan collision seluruh lengan robot (*whole-arm collision checking*).

> **Status implementasi (26 Oktober 2026):** Build dan inisialisasi MotionComponent berhasil, A* pernah menghasilkan `PATH_FOUND` dengan 4 waypoint, dan preview lintasan muncul di RViz. **Tracking lintasan oleh robot belum selalu konsisten**; keberhasilan planning belum berarti keberhasilan eksekusi atau keselamatan dinamis telah dibuktikan.

## 1. Tujuan pengujian

1. Memastikan target dapat diubah melalui input simulasi dan diterima PlanningNode.
2. Memastikan `obstacle_1` dan `obstacle_2` dapat dipindahkan secara independen.
3. Memastikan perubahan lingkungan memicu pemeriksaan ulang node/edge graf dan path planning.
4. Memastikan `PathPlanner` memeriksa konfigurasi joint dan segmen lintasan menggunakan *whole-arm collision checker*.
5. Memastikan hasil GNG + A* berupa waypoint kelima joint serta divisualisasikan di RViz.
6. Memverifikasi pemisahan tanggung jawab Planning → Motion sebelum menguji eksekusi nyata.

**Batasan:** Collision checker memeriksa geometri robot yang direpresentasikan dalam model MuJoCo/planner. Hasilnya bergantung pada kesesuaian model, sampling collision sepanjang edge, serta ketepatan update obstacle. Pengujian ini **tidak** menjamin robot mampu mengikuti lintasan secara tepat atau menghindari setiap obstacle bergerak.

## 2. Arsitektur saat ini

```text
/simulation/set_target       /simulation/set_obstacle_1
          │                  /simulation/set_obstacle_2
          ▼                           │
      Robot Simulation (MuJoCo)       │
          │                           │
          ├── /simulation/target_marker
          ├── /simulation/obstacle_poses
          └── /joint_states
                       │
                       ▼
                PlanningNode (ROS 2)
                       │
                       ▼
                 PlanningManager
                 ├── target / obstacle updates
                 ├── planning / replanning state
                 └── STOP / RELEASE / EXECUTE decisions
                       │
                       ▼
                   PathPlanner
                   ├── GNG joint graph + A*
                   ├── node / edge filtering
                   ├── whole-arm collision checking
                   ├── goal selection
                   └── execution waypoint validation
                       │
                       ▼
                PlanningNode (ROS 2)
                 ├── /planning/status
                 ├── /planning/joint_path_preview
                 └── /motion/joint_path (jika execute_path=True)
                       │
                       ▼
                 MotionComponent
                 ├── JointController
                 ├── TrajectoryGenerator
                 └── MotionGuard
                       │
                       ▼
                     MuJoCo
```

**Pemisahan peran:** `PathPlanner` menghasilkan **lintasan geometris di joint space**; `TrajectoryGenerator` di package `motion` menghitung **interpolasi dan durasi gerakan**. PlanningNode tidak lagi menghitung `joint_path_speed` atau waktu tiap waypoint.

## 3. Prasyarat dan build

- ROS 2 Humble tersedia.
- Workspace: `~/living-lab/cra_ws`.
- Package `cra`, `planning`, `motion`, dan simulasi beserta dependensinya sudah terpasang.
- Model XML robot dan mesh STL tersedia, termasuk body `target`, `obstacle_1`, dan `obstacle_2` sesuai model yang dipakai.
- File graf `.npz` serta konfigurasi `robot.yaml` yang ditunjuk launch tersedia.
- `PathPlanner`, `PlanningManager`, `PlanningNode`, dan `MotionComponent` sudah menggunakan versi yang saling sesuai.

Pada setiap terminal:

```bash
source /opt/ros/humble/setup.bash
cd ~/living-lab/cra_ws
source install/setup.bash
```

Setelah mengubah sumber `planning` atau `motion`:

```bash
cd ~/living-lab/cra_ws
colcon build --packages-select planning motion --symlink-install
source install/setup.bash
```

Jika ada peringatan `local_setup.bash` package lain yang hilang, pemeriksaan workspace/install harus dilakukan secara terpisah; jangan menganggap peringatan tersebut sebagai error pada planner.

## 4. Menjalankan CRA

**Terminal 1 — sistem utama:**

```bash
ros2 launch cra cra.launch.py
```

Pastikan `robot_simulation`, `planning_node`, dan `motion_component` berjalan. Contoh log yang pernah terverifikasi:

```text
MotionComponent initialized
CRA MuJoCo simulation ready
[A*] PATH_FOUND: 4 waypoints
[PathPlanner] PATH_READY: 4 waypoints
Planning status: PATH_READY
```

Jumlah waypoint tergantung target, konfigurasi graf, dan obstacle.

> **Keamanan pengujian:** Mulai dengan parameter `execute_path: false` pada konfigurasi launch PlanningNode. Periksa parameter aktual menggunakan `ros2 param get /planning_node execute_path`. Jika mengubah nilai lewat launch, jalankan ulang sistem. Jangan mengaktifkan gerakan hanya karena preview terlihat benar.

## 5. Topic ROS 2 yang relevan

Topic berikut **terlihat pada `ros2 topic list` dari workspace CRA**. Untuk topic perintah, **verifikasi tipe pesan secara aktual** sebelum menerbitkan data.

| Topic | Arah | Fungsi |
|---|---|---|
| `/simulation/set_target` | Perintah ke simulator | Mengubah target |
| `/simulation/set_obstacle_1` | Perintah ke simulator | Mengatur obstacle pertama |
| `/simulation/set_obstacle_2` | Perintah ke simulator | Mengatur obstacle kedua |
| `/simulation/target_marker` | Simulator → PlanningNode | Target dalam format `visualization_msgs/msg/Marker`; PlanningNode mengharapkan frame `base_link`/frame robot sesuai konfigurasi |
| `/simulation/obstacle_poses` | Simulator → PlanningNode | Obstacle dalam `geometry_msgs/msg/PoseArray`; PlanningNode mengharapkan frame `mujoco_world` |
| `/joint_states` | Simulator → Planning/Motion | Feedback joint `sensor_msgs/msg/JointState` |
| `/planning/status` | PlanningNode → monitor | Status planning `std_msgs/msg/String` |
| `/planning/joint_path_preview` | PlanningNode → visualisasi | Waypoint joint `trajectory_msgs/msg/JointTrajectory` |
| `/motion/joint_path` | PlanningNode → Motion | Perintah waypoint joint `trajectory_msgs/msg/JointTrajectory` |
| `/motion/execution_status` | Motion → monitor/Planning | Status eksekusi `std_msgs/msg/String` |
| `/motion/stop_requested` | Planning → Motion | Perintah stop/release `std_msgs/msg/Bool` |
| `/planning/initial_path` | PlanningNode → RViz | Visualisasi path awal |
| `/planning/replanned_path` | PlanningNode → RViz | Visualisasi replanned path (jika dipublikasikan implementasi aktif) |
| `/planning/gng_nodes`, `/planning/gng_edges` | PlanningNode → RViz | Visualisasi graf GNG |
| `/planning/joint_graph_nodes`, `/planning/joint_graph_edges` | PlanningNode → RViz | Visualisasi graf joint |

**Jangan gunakan topic lama** `/planning/target_position`, `/workspace/obstacle`, `/planning/a_star_path`, `/planning/trajectory`, maupun `/obstacle_controller/...` untuk pengujian implementasi CRA saat ini: topic tersebut berasal dari arsitektur README sebelumnya dan tidak muncul pada daftar topic terbaru.

## 6. Verifikasi interface dan frame

Jalankan sebelum mengirim target atau obstacle:

```bash
ros2 topic type /simulation/set_target
ros2 topic type /simulation/set_obstacle_1
ros2 topic type /simulation/set_obstacle_2
ros2 topic info /simulation/set_target -v
ros2 topic info /simulation/set_obstacle_1 -v
ros2 topic info /simulation/set_obstacle_2 -v
```

Lihat frame dan payload yang diterbitkan simulasi:

```bash
ros2 topic echo /simulation/target_marker --once
ros2 topic echo /simulation/obstacle_poses --once
```

**Penting tentang koordinat:** `PlanningNode` mengonversi target dari frame robot melalui `FrameTransform.robot_base_to_mujoco_base()`, sedangkan `obstacle_poses` harus sudah berada pada `mujoco_world`. Jangan menyamakan frame **topic perintah** `/simulation/set_*` dengan frame feedback tanpa memeriksa implementasi subscriber di `robot_simulation`.

Contoh perintah pada bagian berikut **hanya berlaku jika tipe topic input adalah `geometry_msgs/msg/Point`**. Jika hasil `ros2 topic type` berbeda, gunakan tipe dan field pesan yang benar; jangan memaksakan contoh.

## 7. Pengujian A — Mengubah target

**Terminal 2 — pantau status:**

```bash
ros2 topic echo /planning/status
```

**Terminal 3 — jika tipe `/simulation/set_target` adalah Point:**

```bash
ros2 topic pub --once /simulation/set_target geometry_msgs/msg/Point \
  '{x: 0.20, y: 0.10, z: 0.20}'
```

Koordinat di atas **sekadar contoh**, bukan target yang sudah dipastikan terjangkau. Sesuaikan dengan frame input simulasi dan workspace robot.

Pantau:

```bash
ros2 topic echo /simulation/target_marker --once
ros2 topic echo /planning/joint_path_preview --once
```

Hasil yang mungkin:

- `PATH_READY`: goal dan lintasan joint ditemukan.
- `GOAL_UNREACHABLE`: tidak ada sampel goal yang memenuhi toleransi dan collision checking.
- `NO_PATH`: goal dipilih, tetapi A* tidak menemukan jalur aman.
- `PLANNING_ERROR`: terdapat exception saat proses planning.

## 8. Pengujian B — Mengubah obstacle pertama

**Jika tipe `/simulation/set_obstacle_1` adalah Point:**

```bash
ros2 topic pub --once /simulation/set_obstacle_1 geometry_msgs/msg/Point \
  '{x: 0.15, y: 0.10, z: 0.15}'
```

Periksa feedback:

```bash
ros2 topic echo /simulation/obstacle_poses --once
```

Pastikan posisi obstacle benar-benar berubah di MuJoCo dan jumlah pose tetap sesuai jumlah obstacle (`obstacle_1` dan `obstacle_2`). `PlanningNode` mengasumsikan urutan `PoseArray` cocok dengan **urutan nama obstacle yang diurutkan alfabetis**.

Hasil yang diharapkan: perubahan obstacle memperbarui collision environment, memvalidasi kembali graf/path, dan mencoba planning ulang jika data target/feedback tersedia. **Path tidak harus berubah** jika rute lama tetap valid.

## 9. Pengujian C — Dua obstacle secara independen

**Jika tipe `/simulation/set_obstacle_2` adalah Point:**

```bash
ros2 topic pub --once /simulation/set_obstacle_2 geometry_msgs/msg/Point \
  '{x: 0.25, y: -0.10, z: 0.15}'
```

Pastikan `obstacle_1` tidak berpindah saat `obstacle_2` diubah. Lihat kedua obstacle dalam MuJoCo dan cek:

```bash
ros2 topic echo /simulation/obstacle_poses --once
ros2 topic echo /planning/status
```

Ulangi dengan posisi obstacle dekat lintasan, **hanya pada simulasi dengan `execute_path=False`**. Perubahan lingkungan boleh menyebabkan `PATH_READY`, `NO_PATH`, atau `GOAL_UNREACHABLE`, tergantung geometri.

## 10. Pengujian D — Dynamic replanning dan kegagalan path

Dengan target yang pernah menghasilkan `PATH_READY`:

1. Catat waypoint awal dari `/planning/joint_path_preview`.
2. Pindahkan salah satu obstacle hingga menghalangi segmen yang diperkirakan dilalui robot.
3. Amati status baru dan preview yang diterbitkan.
4. Jauhkan kembali obstacle dan periksa apakah path berhasil ditemukan lagi.

Ketika lintasan tidak aman/tidak tersedia, **tidak boleh ada perintah baru untuk bergerak langsung ke target tanpa path tervalidasi**. Preview kosong atau marker path terhapus menunjukkan pembatalan tampilan path; pemeriksaan status controller tetap diperlukan untuk menyatakan robot berhenti.

> **Catatan:** Implementasi `PlanningManager` terkini menganggap perubahan obstacle sebagai alasan invalidasi path dan planning ulang. Jika posisi obstacle terus berubah, perhatikan kemungkinan replanning berulang. Perbaikan efisiensi/replanning berbasis lintasan tersisa dapat dilakukan setelah perilaku dasar terverifikasi.

## 11. Pengujian E — Integrasi Motion (setelah preview valid)

**Jangan langsung mulai dari tahap ini** jika eksekusi sebelumnya masih kadang menyimpang dari path. Uji hanya di MuJoCo dengan area aman dan pemantauan aktif.

Aktifkan `execute_path=True` **melalui parameter launch** setelah validasi preview dan state STOP/RELEASE selesai. Verifikasi nilai runtime:

```bash
ros2 param get /planning_node execute_path
```

Pantau terminal terpisah:

```bash
ros2 topic echo /planning/status
ros2 topic echo /motion/execution_status
ros2 topic echo /motion/joint_path
ros2 topic hz /joint_states
```

**Interpretasi status:**

| Status | Interpretasi |
|---|---|
| `PATH_READY` | Planning menemukan lintasan; belum berarti dieksekusi |
| `TRAJECTORY_SENT` | PlanningNode mengirim waypoint ke Motion |
| `EXECUTING` | Motion menyatakan gerak sedang dijalankan |
| `COMPLETED` | Controller menyatakan tujuan selesai; tetap periksa posisi joint aktual |
| `STOPPED` | Stop-latch aktif; jangan melepas tanpa memastikan penyebabnya |
| `FEEDBACK_TIMEOUT`, `INVALID_FEEDBACK`, `CONTROL_ERROR` | Kondisi fault; hentikan pengujian sampai penyebab diperiksa |
| `WAITING_FOR_MOTION` | Manager menunggu status siap dari Motion; periksa apakah Motion melaporkan `COMPLETED` atau `IDLE` |

**Status `COMPLETED`:** Pernah diamati MotionComponent melaporkan `COMPLETED` sementara PlanningManager menunggu `IDLE`. Penyelarasan transisi status adalah bagian pengujian integrasi yang harus diverifikasi, bukan alasan untuk mengirim RELEASE otomatis.

**Pengukuran tracking:** Bandingkan posisi joint aktual pada `/joint_states` dengan waypoint terakhir `/motion/joint_path`. Untuk memeriksa penyimpangan **di sepanjang** lintasan, diperlukan logging posisi aktual dan referensi trajectory secara bersamaan; visual MuJoCo vs RViz saja belum cukup.

**Kontrak pesan:** `/motion/joint_path` menggunakan `JointTrajectory`, tetapi implementasi CRA saat ini mengirim **posisi waypoint saja**. Field `time_from_start` tidak digunakan sebagai jadwal eksekusi oleh `JointController`; waktu dan interpolasi dihitung ulang oleh `TrajectoryGenerator`.

## 12. Monitoring dan RViz

Verifikasi topic yang tersedia:

```bash
ros2 topic list | grep -E 'planning|motion|simulation'
ros2 topic echo /planning/status --once
ros2 topic echo /planning/joint_path_preview --once
```

Gunakan Fixed Frame RViz sesuai konfigurasi `robot.frames.base` (umumnya `base_link`). Topic visualisasi yang tersedia mencakup:

- `/planning/gng_nodes` dan `/planning/gng_edges`
- `/planning/joint_graph_nodes` dan `/planning/joint_graph_edges`
- `/planning/initial_path` dan `/planning/initial_path_array`
- `/planning/replanned_path` dan `/planning/replanned_path_array`
- `/planning/invalid_path` dan `/planning/invalid_path_array`
- `/planning/current_eof`, `/planning/eof_marker`, `/planning/eof_marker_array`
- `/simulation/obstacles`, `/simulation/target_marker_array`

Periksa tipe dengan `ros2 topic type <topic>` sebelum menambahkan display RViz, karena tidak semua topic visualisasi menggunakan tipe `Marker` tunggal.

## 13. Tabel pencatatan hasil

| No. | Skenario | Perubahan target/obstacle | Status planning | Jumlah waypoint | Motion status | Catatan |
|---:|---|---|---|---:|---|---|
| 1 | Target valid | Target A | ... | ... | N/A (preview) | ... |
| 2 | Target lain | Target B | ... | ... | N/A (preview) | ... |
| 3 | Geser obstacle 1 | Obstacle dekat path | ... | ... | N/A (preview) | ... |
| 4 | Geser obstacle 2 | Obstacle kedua berubah | ... | ... | N/A (preview) | ... |
| 5 | Dua obstacle | Kedua obstacle aktif | ... | ... | N/A (preview) | ... |
| 6 | Path terhalang | Obstacle memotong path | ... | ... | N/A (preview) | ... |
| 7 | Pemulihan path | Obstacle dijauhkan | ... | ... | N/A (preview) | ... |
| 8 | Eksekusi simulasi | Target tervalidasi | ... | ... | ... | Bandingkan q akhir |

Catat nilai target, frame input, pose obstacle, log A*, serta pesan status. Jangan mengklaim *whole-arm collision-free execution* hanya dari `PATH_READY`.

## 14. Troubleshooting

| Gejala | Pemeriksaan / perbaikan |
|---|---|
| `PlanningManager.__init__() got an unexpected keyword argument 'execute_path'` | Constructor manager harus menerima `execute_path=False`; build ulang `planning`. |
| `Obstacle set changed unexpectedly` berulang | Pastikan daftar obstacle pertama boleh diinisialisasi: validasi himpunan nama hanya setelah `last_obstacles` terisi. |
| `WAITING_FOR_FEEDBACK` | Periksa `/joint_states`, urutan/nama kelima joint, serta timeout. |
| `WAITING_FOR_OBSTACLES` | Periksa `/simulation/obstacle_poses`, `frame_id=mujoco_world`, jumlah pose, dan freshness data. |
| `WAITING_FOR_TARGET` | Kirim target melalui `/simulation/set_target`; cek `/simulation/target_marker`, frame, dan nilai finite. |
| `GOAL_UNREACHABLE` | Target tidak cocok dengan sampel goal atau tertolak collision checking; periksa toleransi dan target. |
| `NO_PATH` | A* tidak menemukan koneksi valid; periksa graf, sambungan awal/akhir, dan obstacle. |
| `PATH_READY` lalu `WAITING_FOR_MOTION` | Periksa `/motion/execution_status`: `COMPLETED` pernah menyebabkan menunggu `IDLE` tanpa akhir. Sinkronkan state machine, jangan mem-bypass safety fault. |
| Path terlihat tetapi robot tidak mengikutinya | Periksa apakah `/motion/joint_path` terbit, status `EXECUTING`, command velocity, feedback joint, serta kemungkinan perbedaan frame visualisasi. |
| Status `STOPPED` | Telusuri siapa mengirim stop dan pastikan kondisi aman sebelum release. |
| Replanning terjadi terlalu sering | Pastikan obstacle benar-benar berubah; periksa toleransi pembandingan pose dan frekuensi data simulasi. |
| Obstacle tidak bergerak di MuJoCo | Periksa tipe topic `/simulation/set_obstacle_*`, subscriber, dan body/mocap simulator. |

## 15. Kriteria keberhasilan dan pekerjaan lanjutan

**Keberhasilan planning:** Target dan obstacle diterima; graph/filter diperbarui; A* menemukan waypoint joint ketika ada lintasan yang aman; preview divisualisasikan; path tak valid tidak dikirim untuk dieksekusi.

**Keberhasilan motion:** MotionComponent menerima waypoint, menggerakkan kelima joint dengan velocity dalam batas, dan mencapai tujuan dalam toleransi, dengan tracking error terukur dan tanpa collision menurut model pemeriksaan.

**Belum terverifikasi sepenuhnya:** tracking lintasan yang konsisten, status STOP/RELEASE dalam seluruh kondisi fault, serta dynamic obstacle avoidance saat robot sedang bergerak. Ketiga hal ini merupakan fokus debugging berikutnya.
