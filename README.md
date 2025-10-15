<!--偷看個毛線球 又沒什麼好看的-->

# 智慧垃圾桶

這是一個使用 ESP32 的物聯網專題。  
該專案使用 PlatformIO 作為開發環境，並透過 `ProjectConfig.cpp` 管理私密設定資料。

---

## 🔋 功能介紹

- 自動連接 WiFi
- 垃圾桶氣味檢測
- 垃圾桶容量檢測

---

## 🧰 系統需求

### 硬體
- ESP32-S3 開發板

### 軟體
- Arduino 框架
- 具備外部網路的 WiFi 環境
- Firebase 資料庫

---

## 📦 使用的外部程式庫

以下為此專案使用的外部程式庫：

- [`ArduinoJson`](https://github.com/bblanchon/ArduinoJson)
- [`BME68x Sensor library`](https://github.com/boschsensortec/Bosch-BME68x-Library)
- [`ESP32Servo`](https://github.com/madhephaestus/ESP32Servo)
- [`FirebaseClient`](https://github.com/mobizt/FirebaseClient)
- [`WebSockets`](https://github.com/Links2004/arduinoWebSockets)
- [`bsec2`](https://github.com/boschsensortec/Bosch-BSEC2-Library)


---

## 🚀 安裝與建置

1. **Clone 專案**

```bash
git clone https://github.com/YueYuuru/esp32-smart-trash-can.git
cd esp32-smart-trash-can
````

2. **安裝 PlatformIO 擴充套件（若尚未安裝）**

3. **建立設定檔**

> 為了保護私密資訊，`ProjectConfig.cpp` 不會包含在版本控制中。請依照以下步驟建立：

```bash
cp docs/ProjectConfig.example.cpp src/ProjectConfig.cpp
```

並編輯該檔案，填入你的 WiFi、資料庫等設定。

---

## 🧾 設定檔說明

請參考 [README_ProjectConfig.md](docs/README_ProjectConfig.md) 以了解如何建立與管理 `ProjectConfig.cpp` 設定檔。

---

## 📁 專案結構（簡要）

```
esp32-smart-trash-can
├── src
│   ├── .pin_used.txt               # 開發板腳位使用紀錄
│   ├── bsec_iaq.cfg                # BME680 環境感測器的 BESC 設定檔案
│   ├── SmartTrashCan.ino           # 主程式
│   ├── ProjectConfig.h             # 設定變數宣告
│   ├── ProjectConfig.cpp           # 設定變數定義（已 gitignore）
│   ├── DatabaseClient.cpp          # 資料庫功能模組
│   ├── EnvironmentalSensor.cpp     # BME680 環境感測模組
│   ├── TrashCanCapacity.cpp        # 垃圾桶容量檢測
│   ├── TrashCanLid.cpp             # 垃圾桶蓋控制
│   ├── TrashCanLight.cpp           # 垃圾桶燈控制
│   └── WifiConnect.cpp             # WiFi 功能模組
│
├── .gitignore
├── platformio.ini
└── README.md
```

---

## ❗ 注意事項

* 請勿將 `ProjectConfig.cpp` 上傳至公開倉庫。
* 若你 fork 或 clone 此專案，請先閱讀 [README_ProjectConfig.md](docs/README_ProjectConfig.md)。

---

## 📜 授權 License

MIT License


