
## Referensi Parameter Lengkap

Dokumen ini merangkum parameter konfigurasi utama untuk driver Kinect/Azure Kinect agar lebih mudah dibaca dan disesuaikan.

### 🧠 1. Body Tracking (AI & Skeleton)

| Parameter                        | Tipe  | Default  | Opsi / Range        | Penjelasan                                                                                                                                    |
| -------------------------------- | ----- | -------- | ------------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `body_tracking_enabled`          | bool  | `'true'` | `'true'`, `'false'` | Mengaktifkan SDK AI untuk mendeteksi 32 sendi tubuh manusia.                                                                                  |
| `body_tracking_smoothing_factor` | float | `'0.0'`   | `'0.0'` s/d `'1.0'` | Filter noise temporal. `0.0` = responsif tapi *jittery*; `0.3-0.5` = seimbang (rekomendasi); `1.0` = sangat halus tapi ada *delay* (lag). |

### 📡 2. Kamera Depth (Sensor Utama Tracking)

| Parameter       | Tipe   | Default           | Opsi                                                                                         | Penjelasan                                                                                                   |
| --------------- | ------ | ----------------- | -------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------ |
| `depth_enabled` | bool   | `'true'`          | `'true'`, `'false'`                                                                          | Mengaktifkan sensor Depth. Wajib bernilai `'true'` untuk body tracking.                                     |
| `depth_mode`    | string | `'NFOV_UNBINNED'` | `'NFOV_UNBINNED'`, `'WFOV_UNBINNED'`, `'NFOV_2X2BINNED'`, `'WFOV_2X2BINNED'`, `'PASSIVE_IR'` | Mode yang umum digunakan: `NFOV_UNBINNED` (paling akurat), `WFOV_UNBINNED` (sudut pandang lebar), `BINNED` (lebih ringan).

**Catatan `depth_mode`:**
- `NFOV_UNBINNED` — 640x576, resolusi penuh, paling akurat.
- `WFOV_UNBINNED` — 1024x1024, sudut pandang lebar, paling berat.
- `BINNED` — resolusi setengah, lebih cepat tetapi detail lebih rendah.

### 3. Kamera Warna (RGB)

| Parameter          | Tipe   | Default  | Opsi                                                            | Penjelasan                                                                           |
| ------------------ | ------ | -------- | --------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `color_enabled`    | bool   | `'true'` | `'true'`, `'false'`                                             | Matikan jika hanya butuh skeleton untuk menghemat bandwidth USB dan CPU.              |
| `color_resolution` | string | `'720P'` | `'720P'`, `'1080P'`, `'1440P'`, `'1536P'`, `'2160P'`, `'3072P'` | Resolusi RGB. FPS 30 hanya tersedia jika resolusi 720P.                                |
| `color_format`     | string | `'bgra'` | `'bgra'`, `'jpeg'`                                              | `bgra` = mentah (RAW), lebih cepat diproses OpenCV. `jpeg` = terkompresi, hemat USB.  |
| `fps`              | int    | `'30'`   | `'5'`, `'15'`, `'30'`                                           | Frame rate. `15` adalah titik tengah yang cocok untuk body tracking.                   |

### ☁️ 4. Point Cloud (Pemetaan 3D Ruangan)

| Parameter                    | Tipe | Default  | Opsi                | Penjelasan                                                                                   |
| ---------------------------- | ---- | -------- | ------------------- | -------------------------------------------------------------------------------------------- |
| `point_cloud`                | bool | `'true'` | `'true'`, `'false'` | Menghasilkan titik 3D dari ruangan. Sangat berat; matikan jika tidak butuh SLAM/Mapping.     |
| `rgb_point_cloud`            | bool | `'true'` | `'true'`, `'false'` | Menghasilkan titik 3D berwarna. Sangat-sangat berat.                                          |
| `point_cloud_in_depth_frame` | bool | `'true'` | `'true'`, `'false'` | Menentukan frame acuan point cloud (`depth` atau `rgb`).                                       |

### 📻 5. IMU, Infrared & Lainnya

| Parameter                           | Tipe   | Default   | Opsi                | Penjelasan                                                                                      |
| ----------------------------------- | ------ | --------- | ------------------- | ----------------------------------------------------------------------------------------------- |
| `rescale_ir_to_mono8`               | bool   | `'false'` | `'true'`, `'false'` | Mengubah gambar IR 16-bit menjadi 8-bit agar bisa dilihat di viewer standar.                    |
| `ir_mono8_scaling_factor`           | float  | `'1.0'`   | Angka desimal       | Faktor kecerahan gambar IR jika `rescale_ir_to_mono8` aktif. Gunakan `'10.0'` untuk passive IR. |
| `imu_rate_target`                   | int    | `'0'`     | `'0'` s.d `'1600'`  | Frekuensi publish IMU (Hz). `0` = maksimum (1600 Hz). Set `'100'` atau `'300'` untuk hemat CPU. |
| `wired_sync_mode`                   | int    | `'0'`     | `'0'`, `'1'`, `'2'` | `0` = Standalone, `1` = Master (mengirim sync), `2` = Subordinate (menerima sync).               |
| `subordinate_delay_off_master_usec` | int    | `'0'`     | Integer (μs)        | Jeda waktu untuk kamera subordinate agar tidak bentrok dengan master.                            |
| `sensor_sn`                         | string | `''`      | Serial Number       | Isi serial number spesifik jika menggunakan lebih dari satu Kinect.                               |
| `recording_file`                    | string | `''`      | Path `.mkv`         | Mengaktifkan mode playback. Driver membaca file ini alih-alih hardware.                           |
| `recording_loop_enabled`            | bool   | `'false'` | `'true'`, `'false'` | Memutar ulang file rekaman secara terus-menerus.                                                   |

---
