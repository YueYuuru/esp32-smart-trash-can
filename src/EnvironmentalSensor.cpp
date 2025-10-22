#include <Arduino.h>
#include <string>
#include <vector>
#include <format>
#include <math.h>
#include <Wire.h>
#include <bsec2.h>
#include <Preferences.h>


#include "EnvironmentalSensor.h"


// BSEC 錯誤代碼兼容性定義
// 為了確保不同版本的BSEC庫都能正常工作
#ifndef BSEC_E_INSUFFICIENT_INPUT_BUFFER
	#ifdef BSEC_E_INSUFFICIENT_INPUT_LENGTH
		#define BSEC_E_INSUFFICIENT_INPUT_BUFFER BSEC_E_INSUFFICIENT_INPUT_LENGTH
	#else
		#define BSEC_E_INSUFFICIENT_INPUT_BUFFER 100
	#endif
#endif



// I2C腳位定義
#define I2C_SDA 9        // I2C數據線
#define I2C_SCL 10       // I2C時鐘線

// 使用內建的BSEC設定 (已複製到專案根目錄內: bsec_iaq.cfg (改惹副檔名 請不要在意) )
const uint8_t bsec_config[] = {
	// 這裡使用的是相對路徑 :D
	#include "bsec_iaq.cfg"
};

// 校正模式定義
#define CALIB_MODE_NONE 0     // 無校正
#define CALIB_MODE_FAST 1     // 快速模式 (30分鐘)
#define CALIB_MODE_SHORT 2    // 短期模式 (4小時)
#define CALIB_MODE_FULL 3     // 完整模式 (24小時)

Bsec2 iaqSensor;            // BSEC感測器物件
Preferences preferences;    // 用於儲存設定和狀態

// 函式前置宣告
bool initSensor();                               // 初始化感測器
void processSensorData();                        // 處理感測器數據
void checkSensorStatus();                        // 檢查感測器狀態
void loadCalibrationState();                     // 載入校正狀態
bool saveState();                                // 儲存狀態
void printCalibrationAdvice();                   // 顯示校正建議
void detectOdor();                               // 偵測臭味
void resetSensor();                              // 重置感測器
void startCalibration(int mode);                 // 開始校正
void handleCalibration();                        // 處理校正過程
void completeCalibration();                      // 完成校正
void printCalibrationProgress(bool initial);     // 顯示校正進度
void startValidation();                          // 開始驗證
void collectValidationSample();                  // 收集驗證樣本
bool validateCalibration();                      // 驗證校正結果
bool validateIaqAccuracy();                      // 驗證IAQ精度
bool validateGasResistanceRange();               // 驗證氣體電阻範圍
bool validateStability();                        // 驗證穩定性
bool validateEnvironment();                      // 驗證環境條件
void revertToManufacturerBaseline();             // 回退到廠商基準值
const char* getOdorScoreLevel(float score);      // 取得臭味評分等級
float calculateVocComponent(float voc);          // 計算VOCs分量
float calculateGasResComponent(float gasRes);    // 計算氣體電阻分量
float calculateIaqComponent(float iaq);          // 計算IAQ分量
float getActualGasResistance();                  // 取得實際氣體電阻值
void scanI2CDevices();                           // 掃描I2C設備
void checkSensorHealth();                        // 檢查感測器健康狀態
void checkStorageSpace();                        // 檢查儲存空間
void printHelp();                                // 顯示幫助資訊
void handleCalibrateCommand(String command);     // 處理校正指令


// 校正模式時間設定 (毫秒)
const unsigned long FAST_CALIB_DURATION  = 30 * 60 * 1000;       // 30分鐘
const unsigned long SHORT_CALIB_DURATION = 4  * 60 * 60 * 1000;  // 4小時
const unsigned long FULL_CALIB_DURATION  = 48 * 60 * 60 * 1000;  // 48小時(自動校正)

// 垃圾桶臭味偵測閾值
const float ODOR_THRESHOLD = 2.0;        // VOCs等效值閾值 (ppm)
const float GAS_RES_THRESHOLD = 100000;  // 氣體電阻閾值 (Ω)

// 儲存當前感測器輸出的指標
const bsecOutputs* outputs = nullptr;

// 環境參數基準值
float temperatureBaseline = 25.0;  // 溫度基準值
float humidityBaseline = 50.0;     // 濕度基準值

// 校正相關變數
float baselineGasResistance = 0;           // 基準氣體電阻值
unsigned long calibrationStartTime = 0;    // 校正開始時間
int currentCalibMode = CALIB_MODE_NONE;    // 當前校正模式
unsigned long calibrationDuration = 0;     // 校正持續時間
bool calibrationInProgress = false;        // 校正進行中標記
bool baselineValid = false;                // 基準線有效標記
bool autoCalibrationInProgress = false;    // 自動校正進行中標記
bool skipCalibration = false;              // 強制跳過校正標記
bool calibrationVerified = false;          // 校正驗證通過標記

// 多感測器融合權重 - 用於計算綜合臭味評分
const float VOC_WEIGHT = 0.7;        // VOCs權重70%
const float GAS_RES_WEIGHT = 0.25;   // 氣體電阻權重25%
const float IAQ_WEIGHT = 0.05;       // IAQ權重5%

// 歷史資料緩衝區 - 用於計算移動平均和趨勢分析
#define HISTORY_SIZE 15
float vocHistory[HISTORY_SIZE] = {0};      // VOCs歷史數據
float gasResHistory[HISTORY_SIZE] = {0};   // 氣體電阻歷史數據
int historyIndex = 0;                      // 當前歷史數據索引

// 垃圾桶特定參數
const float MIN_BASELINE_GAS_RES = 50000;           // 最小有效基準線氣體電阻 (Ω)
const float MAX_BASELINE_GAS_RES = 400000;          // 最大有效基準線氣體電阻 (Ω)
bool odorAlertActive = false;                       // 臭味警報啟動狀態
unsigned long lastSerialOutput = 0;                 // 上次串口輸出時間
const unsigned long SERIAL_OUTPUT_INTERVAL = 3000;  // 3秒輸出一次
int sensorErrorCount = 0;                           // 感測器錯誤計數
const int MAX_SENSOR_ERRORS = 10;                   // 最大感測器錯誤次數

// 臭味評分系統
float currentOdorScore = 0.0;                      // 當前臭味評分
unsigned long lastScoreUpdate = 0;                 // 上次評分更新時間
const unsigned long SCORE_UPDATE_INTERVAL = 5000;  // 每5秒更新一次評分

// 校正進度報告
unsigned long lastCalibProgressReport = 0;                    // 上次校正進度報告時間
const unsigned long CALIB_PROGRESS_INTERVAL = 5 * 60 * 1000;  // 每5分鐘報告一次進度

// 驗收條件相關設定
#define VALIDATION_SAMPLES 30               // 驗證樣本數量(30分鐘)
#define MAX_CV_THRESHOLD 0.05               // 最大變異係數閾值5%
#define MAX_HUMIDITY_THRESHOLD 85.0         // 最大濕度閾值85%
#define MAX_TEMP_FLUCTUATION 2.0            // 最大溫度波動2°C
#define MIN_IAQ_ACCURACY 2                  // 最小IAQ精度要求
#define TARGET_IAQ_ACCURACY 3               // 目標IAQ精度

// 驗證數據結構
struct ValidationData {
	float gasResistance[VALIDATION_SAMPLES];  // 氣體電阻驗證數據
	float temperature[VALIDATION_SAMPLES];    // 溫度驗證數據
	float humidity[VALIDATION_SAMPLES];       // 濕度驗證數據
	uint8_t iaqAccuracy[VALIDATION_SAMPLES];  // IAQ精度驗證數據
	int sampleCount;                          // 樣本計數
	int currentIndex;                         // 當前索引
};

ValidationData validationData;               // 驗證數據實例
bool validationInProgress = false;           // 驗證進行中標記
unsigned long lastValidationSampleTime = 0;  // 上次驗證採樣時間
const unsigned long VALIDATION_SAMPLE_INTERVAL = 60 * 1000;  // 每分鐘採樣一次

// 廠商預設基準值(作為備用)
const float MANUFACTURER_BASELINE_GAS_RES = 100000.0;  // 廠商預設基準值

// 清潔空氣樣本收集相關
const float CLEAN_AIR_THRESHOLD = 0.8;      // VOCs低於此值視為清潔空氣 (ppm)
float cleanAirGasResSum = 0;                // 清潔空氣氣體電阻總和
int cleanAirSampleCount = 0;                // 清潔空氣樣本計數
unsigned long lastSampleTime = 0;           // 上次採樣時間
const unsigned long SAMPLE_INTERVAL = 30 * 1000; // 30秒採樣一次

// 校正建議輸出間隔
const unsigned long CALIBRATION_ADVICE_INTERVAL = 30000;  // 30秒輸出一次校正建議

// 警報最小持續時間
unsigned long lastAlertTime = 0;                         // 上次警報時間
const unsigned long ALERT_MIN_DURATION = 5 * 60 * 1000;  // 5分鐘

// 儲存最後的感測器值和分量
float lastVoc = 0;              // 最後VOCs值
float lastGasRes = 0;           // 最後氣體電阻值
float lastIAQ = 0;              // 最後IAQ值
float lastVocComponent = 0;     // 最後VOCs分量
float lastGasResComponent = 0;  // 最後氣體電阻分量
float lastIaqComponent = 0;     // 最後IAQ分量


// 透過ID取得輸出訊號
float getOutputSignal(uint8_t outputId) {
	// 如果輸出無效 返回NaN
	if (!outputs) {
		return NAN;
	}
	// 遍歷所有輸出 尋找匹配的感測器ID
	for (int i = 0; i < outputs -> nOutputs; i++) {
		if (outputs -> output[i].sensor_id == outputId) {
			return outputs -> output[i].signal;  // 返回對應的訊號值
		}
	}
	return NAN;  // 未找到對應ID 返回NaN
}

// 取得實際氣體電阻值 (轉換對數值為實際Ω)
float getActualGasResistance() {
	// 如果輸出無效 返回NaN
	if (!outputs) {
		return NAN;
	}
	// 尋找補償氣體輸出
	for (int i = 0; i < outputs->nOutputs; i++) {
		if (outputs -> output[i].sensor_id == BSEC_OUTPUT_COMPENSATED_GAS) {
			// 正確轉換：10^log10 值(BSEC輸出的是對數值)
			return std::pow(10, outputs -> output[i].signal);
		}
	}
	return NAN;  // 未找到氣體電阻輸出 返回NaN
}

// 取得輸出精確度
uint8_t getOutputAccuracy(uint8_t outputId) {
	// 如果輸出無效 返回0
	if (!outputs) {
		return 0;
	}
	// 遍歷所有輸出 尋找匹配的感測器ID
	for (int i = 0; i < outputs -> nOutputs; i++) {
		if (outputs -> output[i].sensor_id == outputId) {
			return outputs->output[i].accuracy; // 返回對應的精度值
		}
	}
	return 0;  // 未找到對應ID 返回0
}

// 取得臭味評分等級的文字描述
const char* getOdorScoreLevel(float score) {
	if (score < 0.1) return "非常清新";
	if (score < 0.3) return "清新";
	if (score < 0.5) return "輕微異味";
	if (score < 0.7) return "明顯異味";
	return "嚴重異味";
}

// 計算VOCs分量 (0-1範圍)
float calculateVocComponent(float voc) {
	// VOC低於閾值時，評分在0.0-0.5之間(線性比例)
	if (voc < ODOR_THRESHOLD) {
		return 0.5f * (voc / ODOR_THRESHOLD);
	}
	// VOC高於閾值時 評分在0.5-1.0之間(線性比例)
	return 0.5f + 0.5f * std::min(1.0f, (voc - ODOR_THRESHOLD) / (3.0f * ODOR_THRESHOLD));
}

// 計算氣體電阻分量 (0-1範圍)
float calculateGasResComponent(float gasRes) {
	// 如果基準線無效 使用廠商預設值
	float baseline = baselineValid && baselineGasResistance > 0 ? baselineGasResistance : MANUFACTURER_BASELINE_GAS_RES;

	// 氣體電阻高於基準線時 評分在0.0-0.5之間(空氣較清新)
	if (gasRes > baseline) {
		return 0.5f * (baseline / gasRes);
	}
	// 氣體電阻低於基準線時 評分在0.5-1.0之間(空氣較污濁)
	return 0.5f + 0.5f * std::min(1.0f, (baseline - gasRes) / baseline);
}

// 計算IAQ分量 (0-1範圍)
float calculateIaqComponent(float iaq) {
	// IAQ低於100時 評分在0.0-0.5之間(空氣品質較好)
	if (iaq < 100.0f) {
		return 0.5f * (iaq / 100.0f);
	}
	// IAQ高於100時 評分在0.5-1.0之間(空氣品質較差)
	return 0.5f + 0.5f * std::min(1.0f, (iaq - 100.0f) / 100.0f);
}

// 初始化感測器函式
bool initSensor() {
	Serial.println("正在初始化感測器...");
	
	// 嘗試初始化BME680(最多嘗試5次)
	for (int attempt = 0; attempt < 5; attempt++) {
		Serial.printf("開始第 %d 次嘗試\n", attempt + 1);

		// 嘗試兩個可能的I2C地址(0x76和0x77)
		if (iaqSensor.begin(0x76, Wire) || iaqSensor.begin(0x77, Wire)) {
			Serial.printf("感測器連接成功 嘗試次數: %d\n", attempt + 1);
			
			// 應用BSEC設定檔
			if (!iaqSensor.setConfig(bsec_config)) {
				Serial.println("BSEC 設定檔應用失敗");
				continue;  // 繼續下一次嘗試
			}
			
			// 設定輸出參數(訂閱需要的感測器數據)
			bsecSensor sensorList[] = {
				BSEC_OUTPUT_IAQ,                                  // IAQ指數
				BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,  // 溫度補償值
				BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,     // 濕度補償值
				BSEC_OUTPUT_COMPENSATED_GAS,                      // 氣體補償值
				BSEC_OUTPUT_STATIC_IAQ,                           // 靜態IAQ
				BSEC_OUTPUT_CO2_EQUIVALENT,                       // CO2等效值
				BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,                // 呼吸VOCs等效值
				BSEC_OUTPUT_RAW_TEMPERATURE,                      // 原始溫度
				BSEC_OUTPUT_RAW_HUMIDITY,                         // 原始濕度
				BSEC_OUTPUT_RAW_GAS                               // 原始氣體
			};
			
			// 更新訂閱設定(低功耗模式)
			if (iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP)) {
				Serial.println("感測器訂閱成功");
				
				// 等待感測器穩定(收集10個樣本)
				Serial.println("等待感測器穩定...");
				for (int i = 0; i < 10; i++) {
					if (iaqSensor.run()) {  // 執行BSEC處理
						outputs = iaqSensor.getOutputs();  // 取得輸出數據
						if (outputs && outputs -> nOutputs > 0) {
							Serial.println("感測器穩定運行");
							return true;  // 初始化成功
						}
					}
					vTaskDelay(pdMS_TO_TICKS(500));  // 等待500毫秒
				}
			} else {
				Serial.println("感測器訂閱失敗");
			}
		}
		vTaskDelay(pdMS_TO_TICKS(500));  // 嘗試間隔500毫秒
	}
	
	Serial.println("感測器初始化失敗!");
	scanI2CDevices();  // 掃描I2C設備以診斷問題
	return false;      // 初始化失敗
}


// 檢查存儲空間函式
void checkStorageSpace() {
	size_t freeEntries = preferences.freeEntries();  // 取得剩餘存儲條目
	Serial.printf("Preferences 剩餘存儲空間: %d 條目\n", freeEntries);
	
	// 如果存儲空間不足 清理儲存
	if (freeEntries < 10) {
		Serial.println("⚠️ 存儲空間不足 清理中...");
		preferences.clear();  // 清除所有儲存數據
		Serial.println("存儲空間已清理");
		
		// 重新初始化必要值
		preferences.putBool("skip_calib", skipCalibration);
	}
}

// 載入校正狀態函式
void loadCalibrationState() {
	uint8_t state[BSEC_MAX_STATE_BLOB_SIZE] = {0};  // BSEC狀態緩衝區
	// 從儲存中讀取BSEC狀態
	size_t size = preferences.getBytes("bsec_state", state, BSEC_MAX_STATE_BLOB_SIZE);
	
	Serial.printf("加載的 BSEC 狀態大小: %d\n", size);
	
	if (size > 0) {
		// 設置BSEC狀態
		if (iaqSensor.setState(state)) {
			Serial.println("BSEC 狀態已載入");
			
			// 使用新的鍵名加載基準值
			baselineGasResistance = preferences.getFloat("base_gas", MANUFACTURER_BASELINE_GAS_RES);
			Serial.printf("載入基準線氣體電阻: %.2f Ω\n", baselineGasResistance);
			
			// 檢查基準線是否在有效範圍內
			if (baselineGasResistance >= MIN_BASELINE_GAS_RES && baselineGasResistance <= MAX_BASELINE_GAS_RES) {
				baselineValid = true;
				Serial.println("基準線有效");
			} else {
				Serial.println("警告: 載入的基準線無效");
				baselineValid = false;
			}
		} else {
			Serial.println("無法設置 BSEC 狀態");
		}
	} else {
		Serial.println("無法載入 BSEC 狀態");
		// 使用廠商預設基準值
		baselineGasResistance = MANUFACTURER_BASELINE_GAS_RES;
		Serial.printf("使用廠商基準值: %.2f Ω\n", baselineGasResistance);
	}
	
	// 添加調試信息
	Serial.printf("最終使用的基準值: %.2f Ω\n", baselineGasResistance);
}

// 儲存狀態函式
bool saveState() {
	uint8_t state[BSEC_MAX_STATE_BLOB_SIZE] = {0};  // BSEC狀態緩衝區
	
	// 取得當前 BSEC 狀態
	if (iaqSensor.getState(state)) {
		// 保存 BSEC 狀態到儲存
		size_t bytesSaved = preferences.putBytes("bsec_state", state, BSEC_MAX_STATE_BLOB_SIZE);
		Serial.printf("保存 BSEC 狀態字節數: %d\n", bytesSaved);
		
		// 保存基準值 - 使用新的鍵名
		bool baselineSaved = preferences.putFloat("base_gas", baselineGasResistance);
		Serial.printf("基準值保存結果: %s", (baselineSaved ? "成功" : "失敗"));
		
		Serial.println("數據已保存到存儲");
		Serial.printf("當前基準線: %.2f Ω\n", baselineGasResistance);
		return true;
	} else {
		Serial.println("無法取得 BSEC 狀態");
		return false;
	}
}

// 開始校正函式
void startCalibration(int mode) {
	// 檢查是否已有校正在進行
	if (calibrationInProgress) {
		Serial.println("錯誤: 已有校正在進行中\n請先中止當前校正或等待完成");
		return;
	}
	
	// 檢查感測器狀態
	if (sensorErrorCount > 0) {
		Serial.println("錯誤: 感測器狀態異常 無法開始校正\n請先解決感測器問題");
		return;
	}
	
	// 設置校正參數
	currentCalibMode = mode;
	calibrationInProgress = true;
	
	// 根據模式設置持續時間和顯示訊息
	switch (mode) {
		case CALIB_MODE_FAST: {
			calibrationDuration = FAST_CALIB_DURATION;
			Serial.println("=== 開始快速校正 (30分鐘) ===");
			break;
		}
		case CALIB_MODE_SHORT: {
			calibrationDuration = SHORT_CALIB_DURATION;
			Serial.println("=== 開始短期校正 (4小時) ===");
			break;
		}
		case CALIB_MODE_FULL: {
			calibrationDuration = FULL_CALIB_DURATION;
			if (autoCalibrationInProgress) {
				Serial.println("=== 開始48小時自動校正 ===");
			} else {
				Serial.println("=== 開始完整校正 (24小時) ===");
			}
			break;
		}
		default: {
			
		}
		return;
	}
	
	// 如果不是自動校正，顯示提示訊息
	if (!autoCalibrationInProgress) {
		Serial.println("請確保感測器在清潔空氣中\n校正期間請勿移動感測器");
	}
	
	calibrationStartTime = millis();  // 記錄開始時間
	baselineValid = false;            // 重置基準線有效標記
	
	// 重置校正資料
	cleanAirGasResSum = 0;
	cleanAirSampleCount = 0;
	lastSampleTime = 0;
	
	// 顯示預計完成時間
	printCalibrationProgress(true);
	
	// 開始驗收資料收集
	startValidation();
}

// 處理校正過程函式
void handleCalibration() {
	// 檢查是否完成校正(時間到達)
	if (millis() - calibrationStartTime >= calibrationDuration) {
		completeCalibration();  // 完成校正
		return;
	}
	
	// 定期報告進度(每5分鐘)
	if (millis() - lastCalibProgressReport >= CALIB_PROGRESS_INTERVAL) {
		printCalibrationProgress(false);
		lastCalibProgressReport = millis();
	}
	
	// 控制採樣頻率(每30秒採樣一次)
	if (millis() - lastSampleTime < SAMPLE_INTERVAL) {
		return;
	}

	lastSampleTime = millis();
	
	// 收集校正資料
	if (iaqSensor.run()) {
		outputs = iaqSensor.getOutputs();
		
		if (outputs) {
			float currentVoc = getOutputSignal(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT);
			
			// 只在清潔空氣條件下收集資料(VOCs低於閾值)
			if (currentVoc < CLEAN_AIR_THRESHOLD) {
				// 使用實際氣體電阻值
				float currentGasRes = getActualGasResistance();
				
				// 檢查值是否在合理範圍內
				if (currentGasRes > 0 && currentGasRes >= MIN_BASELINE_GAS_RES && currentGasRes <= MAX_BASELINE_GAS_RES) {
					// 累加清潔空氣樣本數據
					cleanAirGasResSum += currentGasRes;
					cleanAirSampleCount++;
					
					// 顯示採樣信息
					Serial.printf("收集清潔空氣樣本 %d: VOCs=%.3fppm, GasRes=%.2fΩ\n", cleanAirSampleCount, currentVoc, currentGasRes);
				}
			}
		}
	}
}

// 完成校正函式
void completeCalibration() {
	calibrationInProgress = false;  // 標記校正完成
	Serial.println("校正完成 開始驗收檢查...");
	
	// 停止驗收資料收集
	validationInProgress = false;
	
	// 檢查是否收集到足夠樣本
	if (cleanAirSampleCount == 0) {
		Serial.println("校正失敗: 未收集到任何清潔空氣樣本");
		revertToManufacturerBaseline();  // 回退到廠商基準值
		return;
	}
	
	// 計算平均基準值
	baselineGasResistance = cleanAirGasResSum / cleanAirSampleCount;
	
	Serial.println("校正完成!");
	Serial.printf("收集樣本數: %d\n新基準線氣體電阻: %.2f Ω\n", cleanAirSampleCount, baselineGasResistance);
	
	// 進行驗收檢查
	if (validateCalibration()) {
		// 驗收通過，使用校正後的基準值
		baselineValid = true;
		calibrationVerified = true;
		
		Serial.println("🎉 校正驗收通過!");
		
		// 儲存新基準線到儲存
		saveState();
		
		// 如果是自動校正完成，提示用戶
		if (autoCalibrationInProgress) {
			Serial.println("✅ 48小時自動校正已完成\n現在您可以手動啟動校正指令");
			autoCalibrationInProgress = false; // 重置自動校正標記
		}
	} else {
		// 驗收失敗，回退到廠商基準值
		revertToManufacturerBaseline();
	}
	
	currentCalibMode = CALIB_MODE_NONE;  // 重置校正模式
}

// 顯示校正進度函式
void printCalibrationProgress(bool initial) {
	// 計算已進行時間和進度百分比
	unsigned long elapsed = millis() - calibrationStartTime;
	float progressPercent = (elapsed * 100.0f) / calibrationDuration;
	progressPercent = std::min(progressPercent, 100.0f); // 限制最大100%
	
	// 計算剩餘時間(轉為秒)
	unsigned long remaining = (calibrationDuration - elapsed) / 1000;
	unsigned long hours = remaining / 3600;
	unsigned long minutes = (remaining % 3600) / 60;
	
	// 顯示進度信息與剩餘時間
	if (hours > 0) {
		Serial.printf("校正進度: %.1f%, 剩餘時間: %d小時 %d分鐘\n", progressPercent, hours, minutes);
	} else {
		Serial.printf("校正進度: %.1f%, 剩餘時間: %d分鐘\n", progressPercent, minutes);
	}
	
	// 如果是初始顯示，顯示校正模式信息
	if (initial) {
		Serial.println("=== 校正模式資訊 ===");
		switch (currentCalibMode) {
			case CALIB_MODE_FAST: {
				Serial.println("模式: 快速校正 (30分鐘)");
				break;
			}
			case CALIB_MODE_SHORT: {
				Serial.println("模式: 短期校正 (4小時)");
				break;
			}
			case CALIB_MODE_FULL: {
				Serial.println(autoCalibrationInProgress ? "48小時自動校正" : "完整校正 (24小時)");
				break;
			}
		}
		Serial.println("===================");
	}
	
	// 顯示已收集樣本數量
	Serial.printf("已收集清潔空氣樣本: %d 個\n", cleanAirSampleCount);
}

// 開始驗證函式
void startValidation() {
	// 重置驗收資料陣列
	for (int i = 0; i < VALIDATION_SAMPLES; i++) {
		validationData.gasResistance[i] = 0;
		validationData.temperature[i] = 0;
		validationData.humidity[i] = 0;
		validationData.iaqAccuracy[i] = 0;
	}
	// 重置驗收數據計數器和索引
	validationData.sampleCount = 0;
	validationData.currentIndex = 0;
	validationInProgress = true;        // 標記驗證進行中
	lastValidationSampleTime = 0;       // 重置上次採樣時間
	Serial.println("開始收集驗收資料...");
}

// 收集驗證樣本函式
void collectValidationSample() {
	// 檢查是否正在進行驗證且有有效輸出
	if (!validationInProgress || !outputs) {
		return;
	}
	
	// 控制採樣頻率(每分鐘一次)
	if (millis() - lastValidationSampleTime < VALIDATION_SAMPLE_INTERVAL) {
		return;
	}

	lastValidationSampleTime = millis();
	
	// 收集當前感測器數據
	float currentGasRes        = getActualGasResistance();  // 實際氣體電阻值
	float currentTemp          = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE);
	float currentHumidity      = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY);
	uint8_t currentIaqAccuracy = getOutputAccuracy(BSEC_OUTPUT_IAQ);
	
	// 儲存資料到驗證數據陣列
	validationData.gasResistance[validationData.currentIndex] = currentGasRes;
	validationData.temperature[  validationData.currentIndex] = currentTemp;
	validationData.humidity[     validationData.currentIndex] = currentHumidity;
	validationData.iaqAccuracy[  validationData.currentIndex] = currentIaqAccuracy;
	
	// 更新索引(循環緩衝區)
	validationData.currentIndex = (validationData.currentIndex + 1) % VALIDATION_SAMPLES;
	if (validationData.sampleCount < VALIDATION_SAMPLES) {
		validationData.sampleCount++;  // 增加樣本計數
	}
	
	// 顯示採樣信息
	Serial.printf(
		"驗收樣本 %d/%d: Gas=%.2fΩ, Temp=%.2f°C, Humidity=%.2f%, IAQ Acc=%d\n",
		validationData.sampleCount, VALIDATION_SAMPLES, currentGasRes, currentTemp, currentHumidity, currentIaqAccuracy
	);
}

// 驗證校正結果函式
bool validateCalibration() {
  	// 檢查樣本數量是否足夠
	if (validationData.sampleCount < VALIDATION_SAMPLES) {
		Serial.println("驗收失敗: 樣本數量不足");
		return false;
	}
	
	Serial.println("=== 開始校正驗收檢查 ===");
	
	// 檢查四個驗收條件
	bool iaqAccuracyPassed = validateIaqAccuracy();         // IAQ精度檢查
	bool gasResRangePassed = validateGasResistanceRange();  // 氣體電阻範圍檢查
	bool stabilityPassed   = validateStability();           // 穩定性檢查
	bool environmentPassed = validateEnvironment();         // 環境條件檢查
	
	// 綜合結果(所有條件都必須通過)
	bool overallPassed = iaqAccuracyPassed && gasResRangePassed && stabilityPassed && environmentPassed;
	
	// 顯示驗收結果
	Serial.println("=== 驗收結果 ===");
	Serial.printf(
		"IAQ精度: %s\n氣體電阻範圍: %s\n穩定性: %s\n環境條件: %s\n總體結果: %s\n",
		(iaqAccuracyPassed ? "通過" : "失敗"),
		(gasResRangePassed ? "通過" : "失敗"),
		(stabilityPassed ? "通過" : "失敗"),
		(environmentPassed ? "通過" : "失敗"),
		(overallPassed ? "通過" : "失敗")
	);
	
	return overallPassed;
}

// 驗證IAQ精度函式
bool validateIaqAccuracy() {
	int validSamples = 0;        // 有效樣本計數
	int highAccuracySamples = 0; // 高精度樣本計數
	
	// 統計滿足精度要求的樣本數量
	for (int i = 0; i < validationData.sampleCount; i++) {
		if (validationData.iaqAccuracy[i] >= MIN_IAQ_ACCURACY) {
			validSamples++;
		}
		if (validationData.iaqAccuracy[i] >= TARGET_IAQ_ACCURACY) {
			highAccuracySamples++;
		}
	}
	
	// 計算有效樣本比例
	float validRatio = ((float) validSamples) / validationData.sampleCount;
	
	// 顯示IAQ精度檢查結果
	Serial.printf("IAQ精度檢查: %d/%d 樣本 ≥ %d (%.1f%)\n", validSamples, validationData.sampleCount, MIN_IAQ_ACCURACY, validRatio * 100);
	
	return validRatio >= 0.8; // 至少80%樣本滿足精度要求
}

// 驗證氣體電阻範圍函式
bool validateGasResistanceRange() {
	int validSamples = 0;  // 有效樣本計數
	float minGasRes  = validationData.gasResistance[0];  // 最小值
	float maxGasRes  = validationData.gasResistance[0];  // 最大值
	
	// 檢查每個樣本的氣體電阻是否在合理範圍內
	for (int i = 0; i < validationData.sampleCount; i++) {
		float gasRes = validationData.gasResistance[i];
		if (gasRes >= MIN_BASELINE_GAS_RES && gasRes <= MAX_BASELINE_GAS_RES) {
			validSamples++;
		}
		// 更新最小值和最大值
		minGasRes = std::min(minGasRes, gasRes);
		maxGasRes = std::max(maxGasRes, gasRes);
	}
	
	// 計算有效樣本比例
	float validRatio = ((float) validSamples) / validationData.sampleCount;
	
	// 顯示氣體電阻範圍檢查結果及最小值和最大值
	Serial.printf(
		"氣體電阻範圍檢查: %d/%d 樣本在 %.1f-%.1fΩ 範圍內(%.1f%) 最小值: %.2fΩ, 最大值: %.2fΩ\n",
		validSamples, validationData.sampleCount, MIN_BASELINE_GAS_RES, MAX_BASELINE_GAS_RES, validRatio * 100,
		minGasRes, maxGasRes
	);

	return (validRatio >= 0.9); // 至少90%樣本在合理範圍內
}

// 驗證穩定性函式(使用變異係數)
bool validateStability() {
	// 計算平均值和標準差
	float sum = 0;    // 總和
	float sumSq = 0;  // 平方和
	
	for (int i = 0; i < validationData.sampleCount; i++) {
		sum   += validationData.gasResistance[i];
		sumSq += validationData.gasResistance[i] * validationData.gasResistance[i];
	}
	
	float mean     = sum / validationData.sampleCount;                      // 平均值
	float variance = (sumSq / validationData.sampleCount) - (mean * mean);  // 變異數
	float stdDev   = std::sqrt(variance);                                   // 標準差
	float cv       = stdDev / mean;                                         // 變異係數
	
	// 顯示穩定性檢查結果
	Serial.printf(
		"穩定性檢查: 變異係數 = %.2f% (閾值: %.1f%)\n平均值: %.2fΩ, 標準差: %.2fΩ\n",
		cv * 100, MAX_CV_THRESHOLD * 100, mean, stdDev
	);
	
	return (cv <= MAX_CV_THRESHOLD);  // 變異係數不超過閾值
}

// 驗證環境條件函式
bool validateEnvironment() {
	float minTemp     = validationData.temperature[0];  // 最低溫度
	float maxTemp     = validationData.temperature[0];  // 最高溫度
	float maxHumidity = validationData.humidity[0];     // 最高濕度
	int highHumiditySamples = 0;                        // 高濕度樣本計數
	
	// 計算溫度波動和濕度統計
	for (int i = 0; i < validationData.sampleCount; i++) {
		minTemp     = std::min(minTemp,     validationData.temperature[i]);
		maxTemp     = std::max(maxTemp,     validationData.temperature[i]);
		maxHumidity = std::max(maxHumidity, validationData.humidity[i]);
		
		// 統計高濕度樣本
		if (validationData.humidity[i] > MAX_HUMIDITY_THRESHOLD) {
			highHumiditySamples++;
		}
	}
	
	float tempFluctuation = maxTemp - minTemp;  // 溫度波動範圍
	float highHumidityRatio = ((float) highHumiditySamples) / validationData.sampleCount; // 高濕度比例
	
	// 顯示環境條件檢查結果
	Serial.printf(
		"環境條件檢查:\n    溫度波動: %.1f°C (閾值: %.1f°C)\n    最高濕度: %.1f% (閾值: %.1f%)\n    高濕度樣本: %d/%d (%.1f%)\n",
		tempFluctuation, MAX_TEMP_FLUCTUATION,
		maxHumidity, MAX_HUMIDITY_THRESHOLD,
		highHumiditySamples, validationData.sampleCount, highHumidityRatio * 100
	);

	// 檢查環境條件是否滿足要求
	//                 溫度波動在範圍內                        最多10%樣本濕度超標
	return ((tempFluctuation <= MAX_TEMP_FLUCTUATION) && (highHumidityRatio <= 0.1));
}

// 回退到廠商基準值函式
void revertToManufacturerBaseline() {
	Serial.println("⚠️ 校正驗收失敗，回退到廠商基準值");
	
	// 重置BSEC狀態(模擬廠商狀態)
	uint8_t initialState[BSEC_MAX_STATE_BLOB_SIZE] = {0};
	iaqSensor.setState(initialState);
	
	// 設置基準值為廠商預設值
	baselineGasResistance = MANUFACTURER_BASELINE_GAS_RES;
	baselineValid = true; // 廠商基準值視為有效
	
	Serial.printf("使用廠商基準值: %.2f Ω\n", baselineGasResistance);

	// 儲存回退狀態
	saveState();
	
	// 如果是自動校正失敗，重置標記
	if (autoCalibrationInProgress) {
		autoCalibrationInProgress = false;
	}
}

// 偵測臭味函式
void detectOdor() {
	if (!outputs) {
		currentOdorScore = 0.0; // 無法計算評分
		return;
	}
	
	// 控制評分更新頻率(每5秒更新一次)
	if (millis() - lastScoreUpdate < SCORE_UPDATE_INTERVAL) {
		return;
	}
	
	lastScoreUpdate = millis();
	
	// 取得感測器數據(使用實際氣體電阻值)
	lastVoc    = getOutputSignal(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT);
	lastIAQ    = getOutputSignal(BSEC_OUTPUT_IAQ);
	lastGasRes = getActualGasResistance();
	
	// 計算各分量
	lastVocComponent    = calculateVocComponent(lastVoc);
	lastGasResComponent = calculateGasResComponent(lastGasRes);
	lastIaqComponent    = calculateIaqComponent(lastIAQ);
	
	// 綜合評分(加權平均)
	currentOdorScore = (lastVocComponent * VOC_WEIGHT) + (lastGasResComponent * GAS_RES_WEIGHT) + (lastIaqComponent * IAQ_WEIGHT);
	
	// 確保評分在0-1範圍內
	currentOdorScore = std::min(1.0f, std::max(0.0f, currentOdorScore));
	
	// 臭味警報邏輯
	if (!odorAlertActive && currentOdorScore > 0.65f) {
		// 觸發臭味警報(評分超過0.65且滿足最小持續時間)
		if (millis() - lastAlertTime > ALERT_MIN_DURATION) {
			Serial.printf(
				"🚨 垃圾桶臭味警報!\n綜合評分: %.2f\nVOCs: %.3f ppm\n氣體電阻: %.2f Ω\nIAQ: %.2f\n評分等級: %s\n",
				currentOdorScore, lastVoc, lastGasRes, lastIAQ, getOdorScoreLevel(currentOdorScore)
			);
			odorAlertActive = true;    // 啟動警報
			lastAlertTime = millis();  // 記錄警報時間
		}
	} else if (odorAlertActive && currentOdorScore < 0.35f) {
		// 解除臭味警報(評分低於0.35)
		Serial.printf("✅ 臭味警報解除\n當前評分: %.2f\n評分等級: %s\n", currentOdorScore, getOdorScoreLevel(currentOdorScore));

		odorAlertActive = false;  // 關閉警報
	}
}

// 處理感測器數據函式
void processSensorData() {
	static uint32_t lastOutput = 0;
	// 控制輸出頻率(每3秒輸出一次)
	if (millis() - lastOutput < SERIAL_OUTPUT_INTERVAL) {
		return;
	}

	lastOutput = millis();
	
	// 確保有有效輸出
	if (!outputs) {
		return;
	}
	
	Serial.println("=== 感測器讀數 ===");
	
	// 取得BSEC輸出值
	float temperature         = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE);
	float humidity            = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY);
	float iaq                 = getOutputSignal(BSEC_OUTPUT_IAQ);
	float breathVocEquivalent = getOutputSignal(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT);
	
	// 補償氣體電阻(實際值)
	float compensatedGasRes   = getActualGasResistance();
	
	// 取得原始感測器資料(未經BSEC處理)
	float rawTemp             = getOutputSignal(BSEC_OUTPUT_RAW_TEMPERATURE);
	float rawHumidity         = getOutputSignal(BSEC_OUTPUT_RAW_HUMIDITY);
	float rawGasRes           = getOutputSignal(BSEC_OUTPUT_RAW_GAS);
	
	// 取得IAQ精確度
	uint8_t iaqAccuracy       = getOutputAccuracy(BSEC_OUTPUT_IAQ);
	
	// 顯示主要感測器讀數
	Serial.printf(
		"溫度: %.1f\n濕度: %.1f%\nIAQ: %d (精確度: %d/3)\nVOCs等效: %.3f ppm\n補償氣體電阻: %.2f Ω\n基準線氣體電阻: %.2f Ω\n",
		temperature, humidity, iaq, iaqAccuracy, breathVocEquivalent, compensatedGasRes, baselineGasResistance
	);
	
	// 新增臭味評分顯示
	Serial.printf("臭味評分: %.2f (%s)\n", currentOdorScore, getOdorScoreLevel(currentOdorScore));
	
	// 顯示詳細評分組成
	Serial.printf(
		"%s\n", (
			std::format("{}\n{}\n{}\n{}\n{}",
				"--- 評分詳細信息 ---",
				std::format("VOCs分量: {:.2f} (權重: {:.2f}) 當前VOCs: {:.3f} ppm", lastVocComponent, VOC_WEIGHT, lastVoc),
				std::format("氣體電阻分量: {:.2f} (權重: {:.2f}) 當前電阻: {:.2f} Ω 基準線: {:.2f} Ω", lastGasResComponent, GAS_RES_WEIGHT, lastGasRes, baselineGasResistance),
				std::format("IAQ分量: {:.2f} (權重: {:.2f}) 當前IAQ: {:.2f}", lastIaqComponent, IAQ_WEIGHT, lastIAQ),
				std::format("綜合評分: {:.2f}", currentOdorScore)
			)
		).c_str()
	);
	
	// 顯示原始感測器資料(用於調試)
	Serial.printf("--- 原始感測器資料 ---\n原始溫度: %.2f°C\n原始濕度: %.2f%\n原始氣體電阻: %.2fΩ\n", rawTemp, rawHumidity, rawGasRes);

	// 新增評分解釋
	Serial.println("評分等級說明:\n0.0-0.1: 非常清新\n0.1-0.3: 清新\n0.3-0.5: 輕微異味\n0.5-0.7: 明顯異味\n0.7-1.0: 嚴重異味");
	
	// 顯示校正建議
	printCalibrationAdvice();
}

// 顯示校正建議函式
void printCalibrationAdvice() {
	static uint32_t lastAdviceTime = 0;
	// 控制建議輸出頻率(每30秒一次)
	if (millis() - lastAdviceTime < CALIBRATION_ADVICE_INTERVAL) {
		return;
	}
	
	lastAdviceTime = millis();
	
	if (calibrationInProgress) {
		// 顯示校正進度
		Serial.printf("校正進度: %d / 清潔空氣樣本\n", cleanAirSampleCount);
		
		// 如果基準線無效，顯示校正提示
		if (!baselineValid) {
			Serial.println("💡 校正提示: 請確保垃圾桶已清空且乾淨\n   - 建議在清空垃圾桶後啟動系統\n   - 保持垃圾桶蓋關閉以獲得穩定讀數");
		}
	} else {
		Serial.println("校正已完成 基準線有效");
	}
}

// 處理串口指令函式
void handleSerialCommands(String command) {
	
}

// 顯示幫助資訊函式
void printHelp() {
	Serial.printf(
		"%s\n", (
			std::format("{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}\n{}",
				"=== 可用指令 ===",
				"envSensor.calibrate <模式> - 開始校正",
				"   模式選項:",
				"     1 = 快速模式 (30分鐘)",
				"     2 = 短期模式 (4小時)",
				"     3 = 完整模式 (24小時)",
				"   範例: 'envSensor.calibrate 3' 開始24小時校正",
				"envSensor.skip_calibration - 強制跳過48小時自動校正",
				"envSensor.reset - 重置校正",
				"envSensor.status - 檢視當前狀態",
				"envSensor.save - 手動儲存狀態",
				"envSensor.reset_sensor - 重置感測器",
				"envSensor.calib_status - 檢視校正狀態",
				"envSensor.score - 檢視當前臭味評分",
				"envSensor.scan_i2c - 掃描I2C設備",
				"envSensor.abort_calibration - 中止當前校正",
				"envSensor.clear_storage - 清除所有存儲數據",
				"envSensor.help - 顯示說明"
			)
		).c_str()
	);
}

// 處理校正指令函式
void handleCalibrateCommand(String command) {
	// 解析模式參數
	int spaceIndex = command.indexOf(' ');
	if (spaceIndex == -1) {
		Serial.println("錯誤: 請指定校正模式\n用法: calibrate <模式>\n模式: 1=快速(30分鐘), 2=短期(4小時), 3=完整(24小時)\n範例: calibrate 3");
		return;
	}
	
	String modeStr = command.substring(spaceIndex + 1);
	modeStr.trim();
	
	// 根據模式參數啟動對應的校正模式
	if (modeStr == "1") {
		startCalibration(CALIB_MODE_FAST);
		Serial.println("開始快速校正 (30分鐘)");
	} else if (modeStr == "2") {
		startCalibration(CALIB_MODE_SHORT);
		Serial.println("開始短期校正 (4小時)");
	} else if (modeStr == "3") {
		startCalibration(CALIB_MODE_FULL);
		Serial.println("開始完整校正 (24小時)");
	} else {
		Serial.println("錯誤: 無效的模式參數\n請使用 1, 2 或 3\n範例: calibrate 3");
	}
}

// 檢查感測器狀態函式
void checkSensorStatus() {
	// 檢查BSEC錯誤狀態
	if (iaqSensor.status != BSEC_OK) {
		Serial.print("BSEC錯誤: ");
		Serial.println(iaqSensor.status);
		
		// 增加錯誤計數器
		sensorErrorCount++;
		if (sensorErrorCount > MAX_SENSOR_ERRORS) {
			Serial.println("⚠️ 感測器錯誤過多，嘗試重置...");
			resetSensor();
			sensorErrorCount = 0;
		}
	}
	
	// 檢查BME680硬體錯誤狀態
	if (iaqSensor.sensor.status != BME68X_OK) {
		Serial.print("BME680錯誤: ");
		Serial.println(iaqSensor.sensor.status);
		
		// 根據錯誤代碼提供具體建議
		switch (iaqSensor.sensor.status) {
			case BME68X_E_COM_FAIL: {
				Serial.println("通訊失敗 檢查I2C連接\n   - 確認SDA/SCL線路連接正確\n   - 檢查電源供應是否穩定");
				break;
			}
			case BME68X_E_SELF_TEST: {
				Serial.println("自我測試失敗 感測器可能損壞\n   - 建議更換感測器模組");
				break;
			}
			case BME68X_E_INVALID_LENGTH: {
				Serial.println("無效資料長度\n   - 檢查I2C時脈頻率是否過高");
				break;
			}
			default: {
				Serial.println("未知錯誤");
			}
		}
		
		// 增加錯誤計數器
		sensorErrorCount++;
		if (sensorErrorCount > MAX_SENSOR_ERRORS) {
			Serial.println("⚠️ 感測器錯誤過多 嘗試重置...");
			resetSensor();
			sensorErrorCount = 0;
		}
	}
}

// 重置感測器函式
void resetSensor() {
	Serial.println("正在重置感測器...");
	
	// 完全結束當前I2C連接
	Wire.end();
	vTaskDelay(pdMS_TO_TICKS(200)); // 增加延遲確保總線釋放
	
	// 重新初始化I2C
	Wire.begin(I2C_SDA, I2C_SCL);
	Wire.setClock(400000);
	vTaskDelay(pdMS_TO_TICKS(100));
	
	Serial.println("嘗試重新連接感測器...");
	
	// 嘗試兩種可能的I2C地址
	bool sensorFound = false;
	if (iaqSensor.begin(0x76, Wire)) {
		Serial.println("感測器在0x76地址找到");
		sensorFound = true;
	} else if (iaqSensor.begin(0x77, Wire)) {
		Serial.println("感測器在0x77地址找到");
		sensorFound = true;
	}
	
	if (!sensorFound) {
		Serial.println("感測器未找到 嘗試掃描I2C總線...");
		scanI2CDevices();
		return;
	}
	
	// 重新應用BSEC配置
	Serial.println("重新應用BSEC配置...");
	if (!iaqSensor.setConfig(bsec_config)) {
		Serial.println("BSEC配置失敗");
		return;
	}
	
	// 重新設置訂閱
	Serial.println("重新設置感測器訂閱...");
	bsecSensor sensorList[] = {
		BSEC_OUTPUT_IAQ,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
		BSEC_OUTPUT_COMPENSATED_GAS,
		BSEC_OUTPUT_STATIC_IAQ,
		BSEC_OUTPUT_CO2_EQUIVALENT,
		BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
		BSEC_OUTPUT_RAW_TEMPERATURE,
		BSEC_OUTPUT_RAW_HUMIDITY,
		BSEC_OUTPUT_RAW_GAS
	};
	
	if (!iaqSensor.updateSubscription(sensorList, 10, BSEC_SAMPLE_RATE_LP)) {
		Serial.println("感測器訂閱失敗");
		return;
	}
	
	// 重新加載校正狀態
	Serial.println("重新加載校正狀態...");
	loadCalibrationState();
	
	Serial.println("感測器重置成功!");
}

// 掃描I2C設備函式
void scanI2CDevices() {
	Serial.println("掃描I2C設備...");
	byte error, address;
	int nDevices = 0;
	
	// 掃描所有可能的I2C地址(1-127)
	for (address = 1; address < 127; address++) {
		Wire.beginTransmission(address);
		error = Wire.endTransmission();
		
		if (error == 0) {
			// 發現設備
			Serial.printf("%s\n", std::format("發現設備地址: {:#x}", address));
			
			nDevices++;
		}
	}
	
	// 顯示掃描結果
	if (nDevices == 0) {
		Serial.println("未發現I2C設備\n請檢查:\n1. I2C連接是否正確\n2. 感測器是否供電\n3. 接線是否牢固");
	} else {
		Serial.printf("發現 %d 個設備\n", nDevices);
	}
}

// 檢查感測器健康狀態函式
void checkSensorHealth() {
	if (iaqSensor.status != BSEC_OK) {
		Serial.printf("感測器健康狀態: 錯誤 %s\n", iaqSensor.status);
		
		// 根據錯誤代碼提供解決方案
		switch (iaqSensor.status) {
			case BSEC_E_INSUFFICIENT_INPUT_BUFFER: {
				// 使用兼容性定義
				Serial.println("解決方案: 確保感測器穩定運行\n   - 等待更多數據輸入\n   - 檢查採樣率設置");
				break;
			}
			case 101: {
				// BSEC_E_DETECT_STABILITY 
				Serial.println("解決方案: 感測器需要更多時間穩定");
				break;
			}
			default: {
				Serial.println("未知錯誤 嘗試重置感測器");
				resetSensor();
			}
		}
		
		// 如果錯誤過多，重置感測器
		if (sensorErrorCount > MAX_SENSOR_ERRORS) {
			Serial.println("⚠️ 感測器錯誤過多 嘗試重置...");
			resetSensor();
			sensorErrorCount = 0;
		}
	} else {
		Serial.println("感測器健康狀態: 正常");
		sensorErrorCount = 0; // 重置錯誤計數
	}
}


EnvironmentalSensor::EnvironmentalSensor() {
	
}

EnvironmentalSensor& EnvironmentalSensor::getInstance() {
	static EnvironmentalSensor instance;
	return instance;
}

EnvironmentalSensor& environmentalSensor = EnvironmentalSensor::getInstance();

bool EnvironmentalSensor::setup() {
	
	Serial.println("=== 垃圾桶臭味偵測系統 ===");
	Serial.println("版本: 1.25 | 基準值修復版");
	
	// 初始化Preferences（非易失性儲存）
	preferences.begin("bme680", false);
	Serial.println("Preferences 命名空間: 'bme680'");
	
	// 檢查存儲空間
	checkStorageSpace();
	
	// 加載跳過校正標記（從儲存中讀取）
	skipCalibration = preferences.getBool("skip_calib", false);
	
	// 初始化I2C通訊
	Wire.begin(I2C_SDA, I2C_SCL);
	Wire.setClock(400000); // 設定I2C時脈頻率為400kHz（高速模式）
	
	// 初始化感測器
	if (!initSensor()) {
		Serial.println("感測器初始化失敗!");
		//while (true);  // 初始化失敗 停止程式
		return false;
	}
	
	Serial.println("BME680 初始化成功!");
	
	// 載入校正狀態（從儲存中讀取）
	loadCalibrationState();
	
	// 如果沒有有效基準線且未設定跳過校正 自動開始48小時校正
	if (!baselineValid && !skipCalibration) {
		Serial.println("⚠️ 警告: 未找到有效基準線\n系統將自動開始48小時完整校正\n請確保感測器在清潔空氣中\n校正期間請勿移動感測器");
		
		// 開始自動校正（完整模式）
		startCalibration(CALIB_MODE_FULL);
		autoCalibrationInProgress = true;  // 標記為自動校正
	}
	return true;
}


void EnvironmentalSensor::loop() {
	static uint32_t lastRun = 0;  // 上次運行時間
	
	// 控制BSEC運行頻率 (10Hz = 每100毫秒運行一次)
	if (millis() - lastRun >= 100) {
		if (iaqSensor.run()) {  // 執行BSEC處理
			outputs = iaqSensor.getOutputs();  // 取得輸出數據
			lastRun = millis();   // 更新上次運行時間
			
			// 確保outputs有效
			if (!outputs) {
				Serial.println("警告：無法取得感測器輸出");
				vTaskDelay(pdMS_TO_TICKS(100));
				return;  // 跳過本次循環
			}
			
			// 臭味偵測
			detectOdor();
			
			// 處理並輸出資料
			processSensorData();
			
			// 處理校正（如果正在進行中）
			if (calibrationInProgress) {
				handleCalibration();
			}
			
			// 收集驗收資料（如果正在進行中）
			if (validationInProgress) {
				collectValidationSample();
			}
			
			// 定期儲存狀態（每6小時）
			static uint32_t lastStateSave = 0;
			if (millis() - lastStateSave > 6 * 60 * 60 * 1000) {
				saveState();
				lastStateSave = millis();
				Serial.println("BSEC狀態已儲存");
			}
		} else {
			// 處理BSEC運行失敗的情況
			// 專門處理錯誤100（輸入緩衝區不足）
			if (iaqSensor.status == BSEC_E_INSUFFICIENT_INPUT_BUFFER) {
				static uint32_t lastWarning = 0;
				if (millis() - lastWarning > 5000) {  // 每5秒警告一次
					Serial.println("BSEC警告: 數據不足 增加採樣率...");
					lastWarning = millis();
				}
				// 臨時提高採樣率
				vTaskDelay(pdMS_TO_TICKS(50));
			} else {
				checkSensorStatus();  // 檢查其他感測器狀態
			}
		}
	}

	// 每5分鐘檢查感測器健康狀態
	static uint32_t lastHealthCheck = 0;
	if (millis() - lastHealthCheck > 5 * 60 * 1000) {
		checkSensorHealth();
		lastHealthCheck = millis();
	}
	
	// 每分鐘檢查存儲狀態（用於調試）
	static uint32_t lastStorageCheck = 0;
	if (millis() - lastStorageCheck > 60000) {
		Serial.println("=== 存儲狀態檢查 ===");
		Serial.print("基準值鍵存在: ");
		Serial.println(preferences.isKey("base_gas") ? "是" : "否");
		
		float storedBaseline = preferences.getFloat("base_gas", -1);
		Serial.print("存儲的基準值: ");
		Serial.println(storedBaseline);
		
		lastStorageCheck = millis();
	}
	
	vTaskDelay(pdMS_TO_TICKS(10));  // 短暫延遲 減少CPU負載
}

void EnvironmentalSensor::handleSerialCommands(std::vector<std::string> commands) {
	String command = String((commands[1]).c_str());

	// 幫助指令
	if (command == "help") {
		printHelp();
		return;
	}
	
	// 校正指令
	if (command.startsWith("calibrate")) {
		handleCalibrateCommand(command);
		return;
	}
	
	// 新增跳過校準命令
	if (command == "skip_calibration") {
		if (autoCalibrationInProgress) {
			Serial.println("⚠️ 強制跳過48小時自動校正\n現在您可以手動選擇校正模式");
			
			// 停止當前校正
			calibrationInProgress = false;
			autoCalibrationInProgress = false;
			currentCalibMode = CALIB_MODE_NONE;
			
			// 設置跳過標誌
			skipCalibration = true;
			preferences.putBool("skip_calib", true);
			
			// 使用廠商基準值
			revertToManufacturerBaseline();
		} else {
			Serial.println("沒有正在進行的自動校正可以跳過");
		}
		return;
	}
	
	// 修復後的reset命令
	if (command == "reset") {
		// 清除存儲的鍵值
		preferences.remove("bsec_state");
		preferences.remove("base_gas");
		preferences.remove("skip_calib");
		
		// 重置基準值變量為廠商預設值
		baselineGasResistance = MANUFACTURER_BASELINE_GAS_RES;
		baselineValid = false;
		
		// 重置校正狀態
		autoCalibrationInProgress = false;
		skipCalibration = false;
		calibrationInProgress = false;
		currentCalibMode = CALIB_MODE_NONE;
		
		Serial.printf("校正已重置\n使用廠商基準值: %.2f Ω\n請執行 'calibrate' 重新校正\n", baselineGasResistance);
		
		return;
	}
	
	// 狀態查詢指令
	if (command == "status") {
		if (outputs) {
			Serial.printf(
				"=== 當前感測器狀態 ===\nVOCs: %.3f ppm\n補償氣體電阻: %.2f Ω\n基準線氣體電阻: %.2f Ω\n校正狀態: %s\n臭味警報: %s\n校正進度: %d/樣本",
				getOutputSignal(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT),
				getActualGasResistance(),  // 特殊處理：補償氣體電阻需要轉換
				baselineGasResistance,
				(baselineValid ? "有效" : "無效"),
				(odorAlertActive ? "啟動中" : "未啟動"),
				cleanAirSampleCount
			);
		}
		return;
	}
	
	// 手動儲存狀態指令
	if (command == "save") {
		if (saveState()) {
			Serial.println("狀態儲存成功");
		} else {
			Serial.println("狀態儲存失敗");
		}
		return;
	}
	
	// 重置感測器指令
	if (command == "reset_sensor") {
		resetSensor();
		Serial.println("感測器已重置");
		return;
	}
	
	// 校正狀態查詢指令
	if (command == "calib_status") {
		if (calibrationInProgress) {
			Serial.print("校正進行中: ");
			switch (currentCalibMode) {
				case CALIB_MODE_FAST: {
					Serial.println("模式: 快速校正 (30分鐘)");
					break;
				}
				case CALIB_MODE_SHORT: {
					Serial.println("模式: 短期校正 (4小時)");
					break;
				}
				case CALIB_MODE_FULL: {
					Serial.println(autoCalibrationInProgress ? "48小時自動校正" : "完整校正 (24小時)");
					break;
				}
			}
			printCalibrationProgress(false);
		} else if (baselineValid) {
			Serial.printf("校正已完成 基準線: %.2f Ω\n", baselineGasResistance);
		} else {
			Serial.println("尚未校正");
		}
		return;
	}
	
	// 評分查詢指令
	if (command == "score") {
		Serial.printf("=== 當前垃圾桶評分 ===\n臭味評分: %.2f (%s)\n", currentOdorScore, getOdorScoreLevel(currentOdorScore));
		
		// 顯示詳細評分組成
		if (outputs) {
			Serial.printf(
				"%s\n", (
					std::format("{}\n{}\n{}\n{}\n{}",
						"評分組成:",
						std::format("VOCs分量: {:.2f} (權重: {:.2f}) 當前VOCs: {:.3f} ppm", lastVocComponent, VOC_WEIGHT, lastVoc),
						std::format("氣體電阻分量: {:.2f} (權重: {:.2f}) 當前電阻: {:.2f} Ω 基準線: {:.2f} Ω", lastGasResComponent, GAS_RES_WEIGHT, lastGasRes, baselineGasResistance),
						std::format("IAQ分量: {:.2f} (權重: {:.2f}) 當前IAQ: {:.2f}", lastIaqComponent, IAQ_WEIGHT, lastIAQ),
						std::format("綜合評分: {:.2f}", currentOdorScore)
					)
				).c_str()
			);
		} else {
			Serial.println("無法取得詳細評分: 感測器資料無效");
		}
		return;
	}
	
	// I2C設備掃描指令
	if (command == "scan_i2c") {
		scanI2CDevices();
		return;
	}
	
	// 中止校正指令
	if (command == "abort_calibration") {
		if (calibrationInProgress) {
			calibrationInProgress = false;
			Serial.println("校正已中止");
		} else {
			Serial.println("沒有正在進行的校正");
		}
		return;
	}
	
	// 新增清除存儲命令
	if (command == "clear_storage") {
		preferences.clear();
		Serial.println("所有存儲數據已清除");
		return;
	}
	
	// 未知指令處理
	Serial.printf("未知指令: '%s' 輸入 'help' 查看可用指令\n", command);
}
