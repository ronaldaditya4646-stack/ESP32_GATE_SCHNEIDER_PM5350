# ESP32_GATE_SCHNEIDER_PM5350

Kode untuk membantu pengambilan data energi yang berasal dari **Schneider PM5350** pada sebuah **MDP**, menggunakan **ESP32** sebagai gateway (Modbus RTU RS485).

## Fitur

- Membaca data dari power meter Schneider PM5350 lewat Modbus RTU (RS485)
- Membaca THD arus / tegangan (I / V / U)
- Membaca akumulasi energi meter (Accum Wh, VARh, VAh: Del, Rec, D+R, D-R)
- Mengirim data dalam format JSON ke Serial USB tiap siklus
- Opsi log HTTP untuk debugging request web
- Mode scan register untuk mencari alamat THD (opsional)

## Perangkat Keras

| Komponen | Keterangan |
|---|---|
| ESP32 | Mikrokontroler / gateway |
| Schneider PM5350 | Power meter di panel MDP (Modbus RTU RS485) |
| Modul RS485 (mis. MAX485) | Penghubung ESP32 ke jalur RS485 meter |

> Tambahkan komponen lain yang dipakai di proyek ini (mis. modul Ethernet) pada tabel di atas.

## Wiring

> Sesuaikan dengan pin yang dipakai di `src/main.cpp`.

| ESP32 | Modul RS485 | PM5350 |
|---|---|---|
| GPIO `...` (TX) | DI | - |
| GPIO `...` (RX) | RO | - |
| GPIO `...` (DE/RE) | DE + RE (disatukan) | - |
| 3V3 / 5V | VCC | - |
| GND | GND | - |
| - | A | D1 (+) |
| - | B | D0 (-) |

Catatan:
- Pin DE/RE hanya dipakai jika `USE_DE_RE_PIN` bernilai `1` (modul MAX485 manual).
- Pastikan parameter komunikasi (alamat Modbus, baud rate, parity) di meter sama dengan yang ada di kode.

## Struktur Proyek

```
ESP32_GATE_SCHNEIDER_PM5350/
├── platformio.ini
├── README.md
└── src/
    └── main.cpp
```

## Opsi Konfigurasi

Opsi diatur lewat `#define` di bagian atas `src/main.cpp`:

| Opsi | Default | Keterangan |
|---|---|---|
| `USE_DE_RE_PIN` | `0` | `1` = modul MAX485 manual (DE & RE disatukan ke `PIN_DE_RE`) |
| `SERIAL_JSON` | `1` | `1` = kirim JSON ke Serial USB tiap siklus |
| `LOG_HTTP` | `0` | `1` = cetak log tiap request web |
| `READ_THD` | `1` | `1` = baca THD I / V / U |
| `READ_ENERGY` | `1` | `1` = baca akumulasi energi meter (Accum Wh, VARh, VAh: Del, Rec, D+R, D-R) |
| `EN_WORD_BE` | `1` | `1` = register Int64 dibaca urutan word besar dulu (standar Schneider); `0` = dibalik |
| `THD_SCAN` | `0` | `1` = scan register untuk mencari alamat THD (alamat sudah ketemu, default `0`) |
| `SCAN_START` | `21299` | Alamat register awal saat scan THD |
| `SCAN_WINDOWS` | `4` | Jumlah window yang discan |
| `REF_V1`, `REF_V2`, `REF_V3` | `2.88f`, `2.55f`, `2.50f` | Nilai layar meter saat scan THD (hanya dipakai jika `THD_SCAN = 1`) |

## Cara Build dan Upload

1. Install [VS Code](https://code.visualstudio.com/) dan ekstensi **PlatformIO IDE**.
2. Clone repo ini:
   ```bash
   git clone https://github.com/ronaldaditya4646-stack/ESP32_GATE_SCHNEIDER_PM5350.git
   ```
3. Buka folder proyek di VS Code (PlatformIO akan membaca `platformio.ini`).
4. Sambungkan ESP32 lewat USB, lalu klik **Upload** di toolbar PlatformIO.
5. Buka **Serial Monitor** untuk melihat output JSON.

## Contoh Output

Contoh format output (sesuaikan dengan keluaran sebenarnya dari kode):

```json
{
  "...": "..."
}
```

## Catatan

- Alamat register dan urutan word mengikuti dokumentasi Modbus Schneider PM5350.
- Gunakan `THD_SCAN = 1` hanya untuk mencari alamat register THD, lalu kembalikan ke `0`.
