# 🚀 Azure Kinect ROS2 - Environment & Setup Guide

Dokumen ini berisi panduan konfigurasi environment, library path, dan troubleshooting untuk menjalankan **Azure Kinect Body Tracking** di ROS2 Humble. 

> **Catatan:** Dokumen ini mengasumsikan kode program (ROS2 Nodes) sudah berada di dalam workspace dan file `.deb` Azure Kinect Body Tracking SDK sudah diekstrak ke sistem.

---

##  1. Prerequisites (Persiapan Sistem)

Pastikan sistem (laptop/PC) memiliki komponen berikut sebelum menjalankan program:
- **OS:** Ubuntu 22.04 (Jammy Jellyfish)
- **ROS2:** Humble Hawksbill
- **GPU:** NVIDIA dengan CUDA support (Wajib untuk Body Tracking)
- **Library Files:** File `libk4abt.so`, `libonnxruntime.so`, dan `dnn_model_2_0_op11.onnx` sudah berada di `/usr/lib/` dan `/usr/bin/`.

---

## ⚙️ 2. Environment Setup

Karena library Body Tracking diekstrak secara manual (bukan via `apt`), ROS2 perlu diberi tahu di mana letak library tersebut melalui Environment Variables.

### Opsi A: Setup Sementara (Per Terminal)
Jalankan perintah ini **setiap kali** Anda membuka terminal baru sebelum menjalankan driver:

```bash
# 1. Set path library
export LD_LIBRARY_PATH=/usr/lib/libk4abt1.1:/usr/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/usr/lib/libk4abt1.1:$CMAKE_PREFIX_PATH

# 2. Source workspace ROS2
source ~/living-lab/cra_ws/install/setup.bash