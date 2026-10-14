# README Testing Workspace Planning

Dokumen ini menjelaskan pengujian dasar untuk pemindahan target
end-effector dan penambahan, pemindahan, serta penghapusan obstacle pada
package `workspace_planning`.

## 1. Tujuan

Pengujian ini memeriksa apakah: 1. Node planner dan simulasi dapat
dijalankan. 2. Target dapat diubah melalui topic ROS 2. 3. Planner
memperbarui jalur menuju target baru. 4. Obstacle memengaruhi node dan
edge graf yang dapat digunakan A\*. 5. Jalur diperbarui ketika obstacle
dipindahkan atau dihapus.

> Catatan: obstacle yang terlihat di RViz belum membuktikan planner
> menghindarinya. Periksa juga `/planning/a_star_path` dan log planner.

## 2. Prasyarat

-   ROS 2 Humble tersedia.
-   Workspace berada di `~/living-lab/cra_ws`.
-   Package `workspace_planning` berhasil dibangun.
-   Data graf workspace yang dibutuhkan planner sudah dibuat.
-   Model MuJoCo dan mesh yang diperlukan tersedia.

Pada setiap terminal baru, jalankan:

``` bash
source /opt/ros/humble/setup.bash
source ~/living-lab/cra_ws/install/setup.bash
```

## 3. Menjalankan planner dan simulasi

Terminal 1:

``` bash
cd ~/living-lab/cra_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch workspace_planning simulation.launch.py
```

Pastikan proses `planning_node` dan `robot_simulation` tidak langsung
berhenti karena error.

Terminal 2, periksa topic:

``` bash
ros2 topic list
```

Topic utama yang digunakan:

  ---------------------------------------------------------------------------------------
  Topic                         Tipe pesan                        Kegunaan
  ----------------------------- --------------------------------- -----------------------
  `/planning/target_position`   `geometry_msgs/msg/Point`         Mengirim target
                                                                  end-effector

  `/planning/current_eof`       `geometry_msgs/msg/Point`         Posisi end-effector
                                                                  saat ini

  `/planning/a_star_path`       `nav_msgs/msg/Path`               Jalur hasil A\*

  `/planning/trajectory`        `nav_msgs/msg/Path`               Trajektori hasil
                                                                  pemrosesan jalur

  `/workspace/obstacle`         `visualization_msgs/msg/Marker`   Mengirim geometri
                                                                  obstacle
  ---------------------------------------------------------------------------------------

Cocokkan nama topic tersebut dengan kode jika implementasi sudah
berubah.

Untuk melihat posisi end-effector:

``` bash
ros2 topic echo /planning/current_eof
```

Untuk melihat jalur A\*:

``` bash
ros2 topic echo /planning/a_star_path
```

Hentikan perintah `echo` dengan `Ctrl+C`.

## 4. Pengujian A: Memindahkan target

**Tujuan:** memastikan planner menerima target baru dan memperbarui
jalur.

Kirim target pertama:

``` bash
ros2 topic pub --once /planning/target_position geometry_msgs/msg/Point "{x: 0.25, y: 0.10, z: -0.30}"
```

Amati gerakan pada MuJoCo dan jalur pada `/planning/a_star_path`.
Kemudian kirim target kedua:

``` bash
ros2 topic pub --once /planning/target_position geometry_msgs/msg/Point "{x: 0.20, y: -0.10, z: -0.35}"
```

**Hasil yang diharapkan:** - Planner menerima target baru. - Jalur
diperbarui untuk target tersebut. - Simulasi berusaha menggerakkan
end-effector ke target.

Koordinat contoh menggunakan frame `base_link` dan satuan meter. Target
harus berada di dalam workspace yang dapat dijangkau. Jika perencanaan
gagal, gunakan koordinat target yang diketahui berada dalam jangkauan
robot.

## 5. Pengujian B: Menambahkan obstacle

**Tujuan:** memeriksa apakah obstacle memengaruhi node dan edge graf
yang dapat digunakan planner.

Kirim obstacle berbentuk box:

``` bash
ros2 topic pub --once /workspace/obstacle visualization_msgs/msg/Marker "{header: {frame_id: base_link}, ns: workspace_obstacle, id: 0, type: 1, action: 0, pose: {position: {x: 0.25, y: 0.10, z: -0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, scale: {x: 0.10, y: 0.30, z: 0.10}, color: {r: 1.0, g: 0.1, b: 0.1, a: 0.8}}"
```

Jika menggunakan RViz, tambahkan display `Marker` dan atur `Fixed Frame`
ke `base_link`. Selanjutnya, kirim target di sisi lain obstacle, dengan
catatan target harus tetap terjangkau:

``` bash
ros2 topic pub --once /planning/target_position geometry_msgs/msg/Point "{x: 0.25, y: 0.10, z: -0.50}"
```

Periksa jalur terbaru:

``` bash
ros2 topic echo /planning/a_star_path
```

**Hasil yang diharapkan:** - Obstacle diterima oleh filter workspace. -
Node atau edge yang berpotongan dengan obstacle difilter sesuai
implementasi `WorkspaceFilter`. - Planner mencoba mencari jalur dari
graf yang tersisa. - Jika tidak ada jalur valid, catat sebagai kegagalan
perencanaan, bukan keberhasilan menghindari obstacle.

Pengujian ini memeriksa filtering workspace EoF. Ini belum membuktikan
seluruh link robot bebas tabrakan.

## 6. Pengujian C: Memindahkan obstacle

**Tujuan:** memastikan posisi obstacle yang berubah diproses ulang.

Kirim marker dengan namespace dan ID yang sama, tetapi posisi berbeda:

``` bash
ros2 topic pub --once /workspace/obstacle visualization_msgs/msg/Marker "{header: {frame_id: base_link}, ns: workspace_obstacle, id: 0, type: 1, action: 0, pose: {position: {x: 0.35, y: 0.10, z: -0.30}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, scale: {x: 0.10, y: 0.30, z: 0.10}, color: {r: 1.0, g: 0.1, b: 0.1, a: 0.8}}"
```

Periksa apakah marker berpindah dan jalur diperbarui. Gunakan target
yang sama untuk membandingkan jalur sebelum dan sesudah obstacle
dipindahkan.

**Hasil yang diharapkan:** obstacle berpindah, filter menghitung ulang
node dan edge yang valid, dan planner memperbarui jalur berdasarkan
posisi obstacle terbaru.

## 7. Pengujian D: Menghapus obstacle

**Tujuan:** memeriksa apakah obstacle dapat dihapus dan graf dapat
digunakan kembali.

Kirim marker `DELETE` dengan namespace dan ID yang sama:

``` bash
ros2 topic pub --once /workspace/obstacle visualization_msgs/msg/Marker "{header: {frame_id: base_link}, ns: workspace_obstacle, id: 0, action: 2}"
```

Periksa kembali jalur A\*.

**Hasil yang diharapkan:** - Marker dihapus dari visualisasi. - Obstacle
tidak lagi diterapkan oleh filter. - Node dan edge yang sebelumnya
terblokir dapat digunakan lagi, selama tidak terblokir oleh aturan
lain. - Planner memperbarui perencanaan.

Jika marker hilang tetapi jalur tidak berubah, periksa apakah kode
menangani marker `DELETE` dan menghitung ulang filter setelah menerima
pesan tersebut.

## 8. Pengujian E: Perbandingan tanpa dan dengan obstacle

Gunakan satu target yang sama selama pengujian.

1.  Rencanakan jalur tanpa obstacle dan catat hasilnya.
2.  Tambahkan obstacle di area yang diperkirakan dilalui jalur.
3.  Kirim ulang target yang sama.
4.  Catat jalur setelah obstacle ditambahkan.
5.  Hapus obstacle dan periksa apakah jalur dapat direncanakan kembali.

Gunakan tabel berikut untuk mencatat hasil aktual:

  -----------------------------------------------------------------------------------
           No. Skenario      Target    Kondisi    Jalur        Jalur      Catatan
                                       obstacle   terbentuk?   berubah    
                                                               sesuai     
                                                               kondisi?   
  ------------ ------------- --------- ---------- ------------ ---------- -----------
             1 Kondisi awal  Sama      Tidak ada  Isi hasil    Isi hasil  Jalur dasar

             2 Obstacle      Sama      Ada        Isi hasil    Isi hasil  Periksa
               ditambahkan                                                node/edge
                                                                          terfilter

             3 Obstacle      Sama      Posisi     Isi hasil    Isi hasil  Periksa
               dipindahkan             berubah                            pembaruan
                                                                          jalur

             4 Obstacle      Sama      Tidak ada  Isi hasil    Isi hasil  Periksa
               dihapus                                                    pemulihan
                                                                          jalur
  -----------------------------------------------------------------------------------

Jangan menandai pengujian berhasil hanya karena node aktif atau marker
terlihat. Gunakan jalur terbaru dan log planner sebagai bukti.

## 9. Troubleshooting

### Topic tidak ditemukan

``` bash
ros2 topic list
```

Pastikan node yang melakukan publish atau subscribe terhadap topic
tersebut sudah berjalan.

### Jalur tidak berubah setelah target diganti

``` bash
ros2 topic info /planning/target_position
```

Periksa log planner, nama topic, dan tipe pesan.

### Obstacle terlihat, tetapi jalur melewatinya

Periksa apakah `planning_node.py` menerima `/workspace/obstacle`,
menerapkan `WorkspaceFilter` sebelum A\*, dan memublikasikan jalur
terbaru. Marker RViz hanya menunjukkan visualisasi.

### Jalur tidak terbentuk

Kemungkinan penyebab: - Target berada di luar workspace yang
terjangkau. - Obstacle memblokir seluruh jalur. - Graf workspace belum
dibuat atau file graf tidak ditemukan. - Posisi awal atau target tidak
memiliki node graf valid di dekatnya.

Coba target yang lebih dekat dengan posisi robot dan pindahkan atau
kecilkan obstacle untuk mengisolasi penyebabnya.

### Perintah publish hanya berjalan sekali

`--once` mengirim satu pesan lalu berhenti. Untuk mengirim berulang
kali, gunakan `-r 2` untuk 2 Hz, lalu hentikan dengan `Ctrl+C`.

## 10. Batasan pengujian

Pengujian ini memeriksa pemindahan target dan filtering obstacle pada
graf workspace EoF. Pengujian ini belum membuktikan keselamatan tabrakan
seluruh badan robot, akurasi tracking trajektori, atau performa
real-time. Hal-hal tersebut memerlukan pengujian tambahan dengan
skenario dan metrik tersendiri.
