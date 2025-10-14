# ProjectConfig.cpp 說明文件

`ProjectConfig.cpp` 是本專案中用來儲存私密設定與環境參數的檔案，例如 WiFi SSID、密碼、資料庫連線資訊、API 金鑰等。

由於檔案中包含敏感資訊，它**不會被上傳到 GitHub**，請開發者依照此說明手動建立。

---

## 📁 檔案位置

請將 `ProjectConfig.cpp` 放置於專案根目錄（與主 `.ino` 檔案同層）。

---

## 🧾 範例內容

```cpp
#include "ProjectConfig.h"


namespace ProjectConfig {

	// Wifi 連線設定
	const char* WIFI_SSID      = "your_wifi_ssid";
	const char* WIFI_PASSWORD  = "your_wifi_password";

	// Firebase 資料庫 專案設定
	const char* DATABASE_URL   = "https://your-project-rtdb.firebaseio.com/";
	const char* Web_API_KEY    = "your_api_key";
	

	// Firebase 使用者 Email / 密碼
	const char* USER_EMAIL     = "YourEmail@gmail.com";
	const char* USER_PASSWORD  = "YourPassword";
}

```
