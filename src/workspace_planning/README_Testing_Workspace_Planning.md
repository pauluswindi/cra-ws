# README Testing Workspace Planning

Panduan pengujian target dan obstacle avoidance pada robot arm menggunakan MuJoCo, GNG workspace graph, workspace filter, dan A* dalam ROS 2.

## 1. Tujuan

Pastikan bahwa:

1. Target end-effector dapat diganti melalui ROS 2.
2. Obstacle box dapat ditambahkan, dipindahkan, dan dihapus saat simulasi berjalan.
3. Dua obstacle (`obstacle_1` dan `obstacle_2`) dapat digunakan bersamaan.
4. Node dan edge GNG yang bertabrakan dengan target/obstacle menjadi tidak valid.
5. A* menghitung ulang jalur setelah geometri berubah.
6. Path lama dibatalkan ketika perlu replanning, dan robot tidak mengejar target langsung saat A* gagal.

> **Batasan:** Yang difilter saat ini adalah lintasan **end-effector (EoF)**, bukan seluruh link robot. Hasil pengujian belum menjamin tidak terjadi tabrakan elbow, wrist, atau bagian lengan lainnya.

## 2. Prasyarat

- ROS 2 Humble telah terpasang.
- Workspace: `~/living-lab/cra_ws`.
- Package: `workspace_planning` sudah dapat dijalankan.
- Model `robot_model/cognitive_robotics_arm.xml` dan semua STL tersedia.
- Graf `data/workspace_graph.npz` dan `data/raw_workspace.npz` sudah dibuat.
- Versi terbaru `planning_node.py`, `workspace_filter.py`, `robot_simulation.py`, dan `robot.yaml` sudah dipasang.
- File `scripts/planning/obstacle_controller.py` sudah dibuat.
- Model XML memiliki body `obstacle_1` dan `obstacle_2` dengan `mocap="true"`.

Di setiap terminal baru:

```bash
source /opt/ros/humble/setup.bash
cd ~/living-lab/cra_ws
source install/setup.bash
```

Jika mengubah sumber package Python yang di-*install* dengan `colcon`, bangun ulang sebelum testing (kecuali package dijalankan langsung dari source atau telah memakai `--symlink-install`):

```bash
cd ~/living-lab/cra_ws
colcon build --packages-select workspace_planning --symlink-install
source install/setup.bash
```

## 3. Menjalankan sistem

### Terminal 1 — Planner dan simulasi

```bash
cd ~/living-lab/cra_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch workspace_planning simulation.launch.py
```

Pastikan `planning_node` dan `robot_simulation` berhasil mulai. Nama launch file ini mengikuti README awal; sesuaikan jika nama file pada package sudah berubah.

### Terminal 2 — Obstacle controller pertama

Di root workspace, jalankan file controller dari source tree:

```bash
cd ~/living-lab/cra_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
CONTROLLER=$(find src -name obstacle_controller.py -print -quit)
test -n "$CONTROLLER" || { echo 'obstacle_controller.py tidak ditemukan di src/'; exit 1; }
python3 "$CONTROLLER" --ros-args -p obstacle_id:=1 -p obstacle_type:=box
```

Ini membuat node `/obstacle_controller` yang mengatur obstacle dengan ID `1` dan namespace Marker `obstacles`.

> Bila controller sudah didaftarkan sebagai `console_scripts` ROS 2, alternatifnya adalah `ros2 run workspace_planning obstacle_controller`. Perintah tersebut hanya akan bekerja jika entry point memang sudah didaftarkan.

### Terminal 3 — Pemantauan

```bash
ros2 node list
ros2 topic list
ros2 topic info /workspace/obstacle
ros2 topic echo /planning/a_star_path
```

Gunakan terminal lain untuk menjalankan perintah testing. Tekan `Ctrl+C` untuk menghentikan `echo`.

**Topic penting**

| Topic | Tipe | Kegunaan |
|---|---|---|
| `/planning/target_position` | `geometry_msgs/msg/Point` | Perintah posisi target dalam `base_link` |
| `/planning/current_eof` | `geometry_msgs/msg/Point` | Posisi end-effector aktual |
| `/planning/current_target` | `geometry_msgs/msg/Point` | Target yang dilaporkan simulasi |
| `/workspace/obstacle` | `visualization_msgs/msg/Marker` | Geometri obstacle untuk planner dan simulasi |
| `/planning/a_star_path` | `nav_msgs/msg/Path` | Jalur hasil A* (bisa kosong) |
| `/planning/trajectory` | `nav_msgs/msg/Path` | Trajektori visualisasi/pemrosesan |
| `/planning/gng_nodes` | `visualization_msgs/msg/Marker` | Node GNG valid |
| `/planning/blocked_gng_nodes` | `visualization_msgs/msg/Marker` | Node GNG terblokir |
| `/planning/gng_edges` | `visualization_msgs/msg/Marker` | Edge GNG valid |
| `/planning/request_replan` | `std_msgs/msg/Empty` | Permintaan replanning dari executor |
| `/obstacle_controller/set_position` | `geometry_msgs/msg/Point` | Mengubah posisi obstacle ID 1 |
| `/obstacle_controller/remove` | `std_msgs/msg/Empty` | Menghapus obstacle ID 1 |
| `/obstacle_controller/enable` | `std_msgs/msg/Empty` | Mengaktifkan ulang obstacle ID 1 |

Semua koordinat contoh adalah **meter** dalam frame `base_link`. Pastikan target berada pada wilayah yang terjangkau robot.

## 4. Pengujian A — Mengubah target

**Target pertama:**

```bash
ros2 topic pub --once /planning/target_position geometry_msgs/msg/Point \
'{x: 0.25, y: 0.10, z: -0.30}'
```

**Target kedua:**

```bash
ros2 topic pub --once /planning/target_position geometry_msgs/msg/Point \
'{x: 0.20, y: -0.10, z: -0.35}'
```

Lihat posisi EoF dan path:

```bash
ros2 topic echo /planning/current_eof
ros2 topic echo /planning/a_star_path
```

**Harapan:** Planner menerima target baru, memperbarui target bounds, memfilter node/edge yang terdampak, dan menerbitkan path baru bila tersedia. Jika target tidak dapat dijangkau, pilih target lain yang telah berhasil diuji.

## 5. Pengujian B — Menambahkan obstacle pertama

Controller yang dimulai pada bagian 3 otomatis memublikasikan obstacle ID 1 dengan posisi default `(0.3, -0.1, -0.3)` dan ukuran `0.08 × 0.08 × 0.08 m`.

Untuk memastikan obstacle aktif:

```bash
ros2 topic echo /workspace/obstacle
```

Periksa bahwa Marker memiliki `ns: obstacles`, `id: 1`, `type: 1` (CUBE), `action: 0` (ADD), dan `frame_id: base_link`.

Atur posisi obstacle di dekat salah satu segmen path aktual (contoh):

```bash
ros2 topic pub --once /obstacle_controller/set_position geometry_msgs/msg/Point \
'{x: 0.30, y: -0.10, z: -0.30}'
```

**Harapan:** Di log `planning_node`, jumlah `blocked nodes` dan/atau `blocked edges` berubah bila obstacle menutupi graf. Periksa `/planning/a_star_path`: path baru harus menghindari obstacle atau kosong apabila jalur tidak tersedia. Jika posisi contoh tidak memotong graf, sesuaikan dengan jalur aktual yang terlihat.

> Pada visualisasi MuJoCo, body `obstacle_1` harus ada di XML dan sinkronisasi simulasi harus aktif. Marker pada ROS 2 saja tidak membuktikan bahwa body MuJoCo telah tersinkronisasi.

## 6. Pengujian C — Memindahkan obstacle dinamis

Jalankan perintah berikut saat robot memiliki jalur aktif:

```bash
ros2 topic pub --once /obstacle_controller/set_position geometry_msgs/msg/Point \
'{x: 0.40, y: 0.10, z: -0.20}'
```

**Harapan:** Obstacle yang sama (`obstacles/1`) berpindah, filter menghitung ulang validitas graf, path lama dibatalkan, lalu A* mencoba menghitung jalur pengganti. Karena controller mengirim ulang Marker secara periodik, **geometri yang tidak berubah tidak boleh memicu replanning berulang**.

Untuk memeriksa perubahan mask dan jalur, gunakan log planner serta:

```bash
ros2 topic echo /planning/blocked_gng_nodes
ros2 topic echo /planning/a_star_path
```

## 7. Pengujian D — Mengubah ukuran dan bentuk obstacle

Untuk mengubah ukuran box menjadi `0.12 × 0.10 × 0.08 m`:

```bash
ros2 param set /obstacle_controller size_x 0.12
ros2 param set /obstacle_controller size_y 0.10
ros2 param set /obstacle_controller size_z 0.08
```

Uji perubahan bentuk untuk **filter ROS 2**:

```bash
ros2 param set /obstacle_controller obstacle_type sphere
ros2 param set /obstacle_controller obstacle_type cylinder
ros2 param set /obstacle_controller obstacle_type box
```

**Harapan:** Marker dan workspace filter mengikuti geometri yang dipublikasikan.

> **Penting:** XML contoh `obstacle_1` dan `obstacle_2` menggunakan `type="box"`. Mengubah parameter menjadi sphere/cylinder pada controller tidak otomatis mengganti tipe geometri collision MuJoCo. Untuk pengujian kesesuaian **fisik** dengan planner, gunakan `box` sampai pembaruan bentuk geometri MuJoCo selesai.

## 8. Pengujian E — Dua obstacle sekaligus

Di **terminal keempat**, jalankan controller kedua dari source tree:

```bash
cd ~/living-lab/cra_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
CONTROLLER=$(find src -name obstacle_controller.py -print -quit)
python3 "$CONTROLLER" --ros-args \
-r __node:=obstacle_controller_2 \
-p obstacle_id:=2 \
-p obstacle_type:=box \
-p x:=0.35 -p y:=0.10 -p z:=-0.25
```

Controller kedua menerbitkan Marker dengan `ns: obstacles` dan `id: 2` pada topic yang sama `/workspace/obstacle`.

Ubah posisi obstacle kedua secara independen:

```bash
ros2 topic pub --once /obstacle_controller_2/set_position geometry_msgs/msg/Point \
'{x: 0.28, y: 0.12, z: -0.32}'
```

**Harapan:** Kedua obstacle diproses simultan, menghalangi node/edge sesuai geometri masing-masing. Obstacle ID 1 tidak berubah saat ID 2 dipindahkan. Nama body MuJoCo yang cocok adalah `obstacle_1` dan `obstacle_2`.

## 9. Pengujian F — Menghapus dan mengaktifkan kembali obstacle

Hapus hanya obstacle ID 1:

```bash
ros2 topic pub --once /obstacle_controller/remove std_msgs/msg/Empty '{}'
```

Obstacle ID 2 tetap ada. Cek node/edge dan jalur baru. Setelah itu, aktifkan kembali:

```bash
ros2 topic pub --once /obstacle_controller/enable std_msgs/msg/Empty '{}'
```

Hapus obstacle ID 2:

```bash
ros2 topic pub --once /obstacle_controller_2/remove std_msgs/msg/Empty '{}'
```

**Harapan:** Node dan edge yang hanya terhalang obstacle yang dihapus dapat aktif kembali. Node/edge yang masih terhalang target atau obstacle lain tetap tidak valid.

### Penghapusan semua obstacle (uji terpisah)

Jika ingin membersihkan seluruh obstacle aktif pada planner dan simulasi tanpa controller:

```bash
ros2 topic pub --once /workspace/obstacle visualization_msgs/msg/Marker \
'{header: {frame_id: base_link}, action: 3}'
```

**Jangan jalankan perintah `DELETEALL` saat controller masih aktif**, karena controller akan memublikasikan ulang Marker aktif tiap 0.2 detik. Hentikan kedua controller terlebih dahulu atau gunakan topic `remove` masing-masing.

## 10. Pengujian G — Obstacle menutup jalur dan A* gagal

1. Tetapkan target valid yang biasanya dapat dicapai tanpa obstacle.
2. Pindahkan obstacle ke area yang memotong jalur aktual, lalu perbesar bila perlu.
3. Amati log `Workspace filter` dan status `A* failed` / `Rejected path` jika tidak ada jalan.
4. Lihat isi `/planning/a_star_path`.

**Harapan:** Ketika path lama tidak valid, planner mengirim `nav_msgs/Path` dengan `poses: []`. Executor berhenti mengikuti path lama; ia **tidak boleh bergerak langsung menuju target** tanpa path aman yang baru.

> Penghentian ini berbasis perintah software. Pada perangkat robot sungguhan, tetap diperlukan kontrol keselamatan tambahan serta pemeriksaan tabrakan seluruh link.

## 11. Pengujian H — Pemulihan jalur

Sesudah pengujian G, pindahkan obstacle menjauhi jalur:

```bash
ros2 topic pub --once /obstacle_controller/set_position geometry_msgs/msg/Point \
'{x: 0.50, y: 0.20, z: -0.20}'
```

Atau hapus obstacle melalui topic `remove`.

**Harapan:** Workspace filter mengaktifkan kembali node/edge yang kini aman, A* mencoba membuat path dari **posisi EoF aktual**, dan robot dapat mengikuti path baru jika ditemukan.

## 12. Pengujian alternatif — Marker langsung tanpa controller

Cara berikut berguna untuk menguji planner saja. **Matikan `obstacle_controller.py` terlebih dahulu** agar update tidak tertimpa publikasi periodik controller.

Tambahkan/ubah obstacle dengan namespace dan ID tetap:

```bash
ros2 topic pub --once /workspace/obstacle visualization_msgs/msg/Marker \
'{header: {frame_id: base_link}, ns: obstacles, id: 1, type: 1, action: 0, pose: {position: {x: 0.30, y: -0.10, z: -0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, scale: {x: 0.08, y: 0.08, z: 0.08}, color: {r: 1.0, g: 0.0, b: 0.0, a: 0.7}}'
```

Hapus ID 1:

```bash
ros2 topic pub --once /workspace/obstacle visualization_msgs/msg/Marker \
'{header: {frame_id: base_link}, ns: obstacles, id: 1, action: 2}'
```

`action: 0` berarti ADD/UPDATE, `action: 2` berarti DELETE, dan `action: 3` berarti DELETEALL.

## 13. Monitoring dan visualisasi

Periksa jumlah obstacle dan perubahan filtering pada **log `planning_node`**. Contoh bentuk pesan (angka akan berbeda):

```text
Workspace filter: ... valid nodes, ... blocked nodes, ... valid edges, ... blocked edges.
A* planned: ... nodes, length=... m
```

Di RViz (jika digunakan), atur `Fixed Frame` ke `base_link` dan tampilkan Marker untuk:

- `/planning/gng_nodes` (node valid)
- `/planning/blocked_gng_nodes` (node terblokir)
- `/planning/gng_edges` (edge valid)
- `/planning/target_bounds` (volume target)
- `/workspace/obstacle` (obstacle)

Tambahkan Path untuk `/planning/a_star_path` dan, bila perlu, `/planning/trajectory`.

Perlu diingat: `/planning/trajectory` adalah keluaran pemrosesan jalur; kode executor saat ini mengikuti `/planning/a_star_path`. Jangan menganggap trajectory visualisasi telah dicek tabrakan lagi jika kurvanya menyimpang dari path A*.

## 14. Tabel pencatatan hasil

| No | Skenario | Obstacle aktif | Mask berubah? | A* mengeluarkan path? | Robot mengikuti path aman? | Catatan |
|---:|---|---|---|---|---|---|
| 1 | Ubah target | 0 | ... | ... | ... | ... |
| 2 | Tambah obstacle 1 | 1 | ... | ... | ... | ... |
| 3 | Geser obstacle 1 | 1 | ... | ... | ... | ... |
| 4 | Perbesar obstacle 1 | 1 | ... | ... | ... | ... |
| 5 | Tambah obstacle 2 | 2 | ... | ... | ... | ... |
| 6 | Hapus obstacle 1 | 1 | ... | ... | ... | ... |
| 7 | Blokir seluruh jalur | 1–2 | ... | ... | ... | ... |
| 8 | Pindahkan obstacle menjauh | 0–2 | ... | ... | ... | ... |

Catat posisi target, posisi dan ukuran obstacle, jumlah blocked nodes/edges, serta pesan log `A* planned`/`A* failed` pada setiap skenario.

## 15. Troubleshooting

| Gejala | Pemeriksaan |
|---|---|
| `/workspace/obstacle` tidak ada | Pastikan controller aktif; jalankan `ros2 topic list` dan `ros2 node list`. |
| `ros2 param set` gagal | Periksa nama node dan parameter dengan `ros2 param list /obstacle_controller`. |
| Obstacle tidak bergerak di MuJoCo | Pastikan body `obstacle_1`/`obstacle_2` tersedia dan `mocap="true"`, serta frame Marker adalah `base_link`. |
| Obstacle kembali ke posisi lama | Ada controller lain yang terus memublikasikan obstacle dengan pasangan namespace/ID sama. |
| Filter tidak berubah | Obstacle mungkin berada di luar graf; cek frame, namespace/ID, ukuran, dan posisi. |
| Path masih melintasi obstacle | Pastikan planner menerima Marker, menyegarkan mask, dan memublikasikan path terbaru. Pastikan yang dibandingkan adalah path A*, bukan trajectory yang belum dicek ulang. |
| Jalur A* kosong | Target di luar jangkauan; konektor awal/akhir terhalang; atau graf terputus oleh obstacle. |
| `ros2 run` controller tidak dikenal | Jalankan file Python langsung atau tambahkan `console_scripts` ke instalasi package. |
| Bentuk di MuJoCo tidak sesuai Marker | Tipe geom MuJoCo perlu disesuaikan. Perubahan `sphere`/`cylinder` via Marker tidak otomatis mengganti `type="box"` di XML. |

## 16. Batasan dan kriteria keberhasilan

Pengujian berhasil apabila perubahan obstacle/target menghasilkan **perubahan mask yang sesuai**, A* memakai graf yang valid, path lama tidak dipakai saat terhalang, dan jalur baru (jika tersedia) menghindari volume obstacle berdasarkan model workspace filter.

Pengujian belum membuktikan keselamatan semua link robot, keterjangkauan joint untuk setiap waypoint, maupun jaminan menghindari obstacle bergerak cepat. Untuk itu diperlukan collision checking robot penuh, joint-space planning, dan validasi kontrol secara terpisah.
