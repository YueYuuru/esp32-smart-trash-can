#include <Arduino.h>
#include <string>
#include <vector>
#include <Wire.h>
#include <bsec2.h>
#include <Preferences.h>


#include "EnvironmentalSensor.h"


// I2C 引腳定義
#define I2C_SDA 9
#define I2C_SCL 10

// 使用內建的BSEC設定 (已複製到專案根目錄內: bsec_iaq.cfg (改惹副檔名 請不要在意) )
const uint8_t bsec_config[] = {
	// 這裡使用的是相對路徑 :D
	#include "bsec_iaq.cfg"
};

Bsec2 iaqSensor;
Preferences preferences;

// 函數前置宣告
bool initSensor();
void processSensorData();
void checkSensorStatus();
void loadCalibrationState();
void saveState();
void printCalibrationAdvice();
void calibrateBaseline();
void applyEnvironmentalCompensation();
void detectOdor();
void updateBaselineForTrashBin();
void checkForCleanAirPeriod();


// 垃圾桶臭味檢測閾值 (根據垃圾桶環境調整)
const float ODOR_THRESHOLD = 2.0;          // VOCs 等效值閾值 (ppm)
const float RAW_GAS_THRESHOLD = 100000;    // 原始氣體電阻閾值 (Ω)

// 儲存當前感測器輸出的指標
const bsecOutputs* outputs = nullptr;

// 環境補償參數
float temperatureBaseline = 25.0;
float humidityBaseline = 50.0;
float pressureBaseline = 1013.25;

// 校準資料
float baselineGasResistance = 0;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION = 24 * 60 * 60 * 1000;    // 24小時校準 (垃圾桶環境縮短)
bool calibrationInProgress = true;
bool baselineValid = false;

// 多感測器融合權重 (垃圾桶環境調整)
const float VOC_WEIGHT = 0.7;         // 提高 VOCs 權重
const float GAS_RES_WEIGHT = 0.25;    // 氣體電阻權重
const float IAQ_WEIGHT = 0.05;        // 降低 IAQ 權重

// 歷史資料緩衝區
#define HISTORY_SIZE 15    // 增加緩衝區大小
float vocHistory[HISTORY_SIZE] = {0};
float gasResHistory[HISTORY_SIZE] = {0};
int historyIndex = 0;

// 垃圾桶特定參數
const float MIN_BASELINE_GAS_RES = 30000;     // 垃圾桶最小有效基線氣體電阻 (Ω)
const float MAX_BASELINE_GAS_RES = 200000;    // 垃圾桶最大有效基線氣體電阻 (Ω)
unsigned long lastCleanAirCheck = 0;
const unsigned long CLEAN_AIR_CHECK_INTERVAL = 30 * 60 * 1000;    // 每30分鐘檢查一次清潔空氣
float cleanAirGasResSum = 0;
int cleanAirSampleCount = 0;
const int MIN_CLEAN_AIR_SAMPLES = 5;      // 垃圾桶環境減少所需樣本數
const float CLEAN_AIR_THRESHOLD = 0.8;    // VOCs低於此值視為清潔空氣 (ppm) - 垃圾桶環境放寬
const unsigned long BASELINE_UPDATE_INTERVAL = 6 * 60 * 60 * 1000;    // 每6小時更新基線
unsigned long lastBaselineUpdate = 0;
bool odorAlertActive = false;             // 臭味警報狀態


// 透過ID取得輸出訊號
float getOutputSignal(uint8_t outputId) {
	if (!outputs) {
		return NAN;
	}

	for (int i = 0; i < (outputs -> nOutputs); i++) {
		if (outputs -> output[i].sensor_id == outputId) {
			return outputs -> output[i].signal;
		}
	}
	return NAN;
}


EnvironmentalSensor::EnvironmentalSensor() {
    
}

EnvironmentalSensor& EnvironmentalSensor::getInstance() {
    static EnvironmentalSensor instance;
    return instance;
}

EnvironmentalSensor& environmentalSensor = EnvironmentalSensor::getInstance();    // 垃圾桶容量

void EnvironmentalSensor::setup() {
	
	Serial.println("=== 垃圾桶臭味檢測系統 ===");
	Serial.println("版本: 1.1 | 垃圾桶環境優化");
	
	// 初始化I2C
	Wire.begin(I2C_SDA, I2C_SCL);
	Wire.setClock(400000); // 提高I2C時鐘頻率
	
	// 初始化 Preferences
	preferences.begin("bme680", false);
	
	// 初始化感測器
	if (!initSensor()) {
		Serial.println("感測器初始化失敗! 請檢查:");
		Serial.println("1. 硬體連接是否正確");
		Serial.println("2. I2C 位址是否正確 (0x76 或 0x77)");
		Serial.println("3. 電源是否穩定 (3.3V)");
		while (true);
	}
	
	Serial.println("BME680 初始化成功!");
	Serial.printf("BSEC 版本: %s.%s.%s.%s \n",
		String(iaqSensor.version.major),
		String(iaqSensor.version.minor),
		String(iaqSensor.version.major_bugfix),
		String(iaqSensor.version.minor_bugfix)
	);
	
	// 開始校準過程
	calibrateBaseline();
	
	Serial.println("垃圾桶臭味檢測已啟動");
	Serial.println("閾值: VOCs > " + String(ODOR_THRESHOLD) + " ppm");
}


void EnvironmentalSensor::loop() {
	if (iaqSensor.run()) {
		outputs = iaqSensor.getOutputs();
		
		// 確保outputs有效
		if (!outputs) {
			Serial.println("警告：無法取得感測器輸出");
			vTaskDelay(pdMS_TO_TICKS(100));
			return;
		}
		
		// 環境補償
		applyEnvironmentalCompensation();
		
		// 臭味檢測（僅在基線有效時進行）
		if (baselineValid) {
			detectOdor();
		}
		
		// 處理並輸出資料
		processSensorData();
		
		// 定期檢查清潔空氣時段
		checkForCleanAirPeriod();
		
		// 定期更新基線（垃圾桶環境需要更頻繁更新）
		updateBaselineForTrashBin();
		
		// 定期保存狀態（每6小時）
		static uint32_t lastStateSave = 0;
		if (millis() - lastStateSave > 6 * 60 * 60 * 1000) {
			saveState();
			lastStateSave = millis();
			Serial.println("BSEC 狀態已儲存");
		}
	} else {
		checkSensorStatus();
	}

	vTaskDelay(pdMS_TO_TICKS(100));
}






bool initSensor() {
	// 強制使用0x76位址
	Serial.println("嘗試使用位址 0x76 初始化 BME680...");

	if (!iaqSensor.begin(0x76, Wire)) {
		Serial.println("無法在 0x76 位址找到 BME680 感測器 嘗試 0x77...");

		if (!iaqSensor.begin(0x77, Wire)) {
			Serial.println("無法找到BME680感測器 請檢查連接");
			return false;
		}
	}

	// 應用BSEC設定
	if (!iaqSensor.setConfig(bsec_config)) {
		Serial.println("BSEC 設定應用失敗");
		return false;
	}

	// 設定輸出參數
	bsecSensor sensorList[] = {
		BSEC_OUTPUT_IAQ,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE,
		BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY,
		BSEC_OUTPUT_RAW_GAS,
		BSEC_OUTPUT_STATIC_IAQ,
		BSEC_OUTPUT_CO2_EQUIVALENT,
		BSEC_OUTPUT_BREATH_VOC_EQUIVALENT,
		BSEC_OUTPUT_GAS_PERCENTAGE
	};

	if (!iaqSensor.updateSubscription(sensorList, 8, BSEC_SAMPLE_RATE_LP)) {
		Serial.println("感測器訂閱失敗");
		return false;
	}

	// 載入之前的校準狀態
	loadCalibrationState();

	// 取得初始環境參數
	for (int i = 0; i < 5; i++) {
		if (iaqSensor.run()) {
			outputs = iaqSensor.getOutputs();
			
			// 確保outputs有效
			if (outputs) {
				temperatureBaseline   = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE);
				humidityBaseline      = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY);
				baselineGasResistance = getOutputSignal(BSEC_OUTPUT_RAW_GAS);
			}
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}

	Serial.printf("初始環境參數:\n溫度基線: %.4f °C    濕度基線: %.4f%    初始氣體電阻: %.4f Ω \n",
		temperatureBaseline, humidityBaseline, baselineGasResistance
	);

	// 檢查初始環境是否合理
	if (baselineGasResistance < MIN_BASELINE_GAS_RES || baselineGasResistance > MAX_BASELINE_GAS_RES) {
		Serial.println("警告：初始氣體電阻超出合理範圍，可能處於污染環境");
		Serial.println("系統將持續監測清潔空氣時段以更新基線");
		baselineValid = false;
	} else {
		baselineValid = true;
		Serial.println("初始基線有效");
	}

	return true;
}



void calibrateBaseline() {
	Serial.println("=== 開始基線校準 ===");
	Serial.println("請確保垃圾桶剛清空且乾淨");
	Serial.println("系統將自動檢測清潔空氣時段來更新基線");

	calibrationStartTime = millis();
	calibrationInProgress = true;
	baselineValid = false;

	// 初始化歷史緩衝區
	for (int i = 0; i < HISTORY_SIZE; i++) {
		vocHistory[i] = 0;
		gasResHistory[i] = baselineGasResistance;
	}
}

void checkForCleanAirPeriod() {
	if (!outputs) {
		return;
	}

	// 每 30 分鐘檢查一次
	if (millis() - lastCleanAirCheck < CLEAN_AIR_CHECK_INTERVAL) {
		return;
	}


	lastCleanAirCheck = millis();

	// 獲取當前 VOCs 值
	float currentVoc = getOutputSignal(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT);

	// 如果 VOCs 低於閾值 認為是清潔空氣
	if (currentVoc < CLEAN_AIR_THRESHOLD) {
		cleanAirGasResSum += getOutputSignal(BSEC_OUTPUT_RAW_GAS);
		cleanAirSampleCount++;

		Serial.printf("檢測到清潔空氣時段 (VOC = %.3f ppm) \n", currentVoc);

		// 如果有足夠的清潔空氣樣本，更新基線
		if (cleanAirSampleCount >= MIN_CLEAN_AIR_SAMPLES) {
			float newBaseline = cleanAirGasResSum / cleanAirSampleCount;
			
			// 確保新基線在合理範圍內
			if (newBaseline >= MIN_BASELINE_GAS_RES && newBaseline <= MAX_BASELINE_GAS_RES) {
				baselineGasResistance = newBaseline;
				baselineValid = true;
				calibrationInProgress = false;
				
				Serial.println("基線已更新!");
				Serial.printf("新基線氣體電阻: %.4f Ω \n", baselineGasResistance);
				
				// 重置計數器
				cleanAirGasResSum = 0;
				cleanAirSampleCount = 0;
			}
		}
	} else {
		// 如果檢測到污染 重置計數器
		if (cleanAirSampleCount > 0) {
			Serial.println("檢測到污染 重置清潔空氣計數器");
			cleanAirGasResSum = 0;
			cleanAirSampleCount = 0;
		}
	}
}

void updateBaselineForTrashBin() {
	// 每6小時更新一次基線
	if (millis() - lastBaselineUpdate < BASELINE_UPDATE_INTERVAL) {
		return;
	}

	lastBaselineUpdate = millis();

	if (!baselineValid) {
		return;
	}

	// 計算最近一段時間的平均氣體電阻
	float sum = 0;
	int count = 0;
	for (int i = 0; i < HISTORY_SIZE; i++) {
		if (gasResHistory[i] > 0) {
			sum += gasResHistory[i];
			count++;
		}
	}

	if (count > 0) {
		float newBaseline = sum / count;

		// 確保新基線在合理範圍內
		if (newBaseline >= MIN_BASELINE_GAS_RES && newBaseline <= MAX_BASELINE_GAS_RES) {
			baselineGasResistance = newBaseline;
			Serial.println("定期更新基線氣體電阻");
			Serial.printf("新基線: %.4f Ω \n", baselineGasResistance);
		}
	}
}

void applyEnvironmentalCompensation() {
	if (!outputs || !baselineValid) {
		return;
	}

	// 取得當前環境參數
	float currentTemp     = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE);
	float currentHumidity = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY);

	// 計算環境變化因子
	float tempFactor     = 1.0 + (currentTemp  - temperatureBaseline) * 0.02;
	float humidityFactor = 1.0 + (currentHumidity - humidityBaseline) * 0.01;

	// 應用補償因子到基線
	baselineGasResistance = baselineGasResistance * tempFactor * humidityFactor;

	// 除錯輸出
	static unsigned long lastCompensationPrint = 0;
	if (millis() - lastCompensationPrint > 60000) {
		Serial.printf("環境補償:\n溫度因子: %.2f    濕度因子: %.2f    補償後氣體電阻基線: %.4f \n",
			tempFactor, humidityFactor, baselineGasResistance
		);
		lastCompensationPrint = millis();
	}
}

void detectOdor() {
	if (!outputs || !baselineValid) {
		return;
	}
	
	// 取得當前值
	float currentVoc    = getOutputSignal(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT);
	float currentGasRes = getOutputSignal(BSEC_OUTPUT_RAW_GAS);
	float currentIAQ    = getOutputSignal(BSEC_OUTPUT_IAQ);
	
	// 更新歷史資料
	historyIndex = historyIndex % HISTORY_SIZE;          // 確保索引在範圍內
	vocHistory[historyIndex] = currentVoc;
	gasResHistory[historyIndex] = currentGasRes;
	historyIndex = (historyIndex + 1) % HISTORY_SIZE;    // 循環索引
	
	// 計算移動平均值
	float avgVoc = 0;
	float avgGasRes = 0;
	for (int i = 0; i < HISTORY_SIZE; i++) {
		avgVoc += vocHistory[i];
		avgGasRes += gasResHistory[i];
	}
	avgVoc /= HISTORY_SIZE;
	avgGasRes /= HISTORY_SIZE;
	
	// 計算變化率
	float vocChange    = (currentVoc       - avgVoc) / avgVoc;
	float gasResChange = (currentGasRes - avgGasRes) / avgGasRes;
	
	// 多感測器融合演算法
	float odorScore = 0;
	
	// VOCs 分量
	float vocComponent = max(0.0f, (currentVoc - ODOR_THRESHOLD) / ODOR_THRESHOLD);
	
	// 氣體電阻分量
	float gasResComponent = max(0.0f, (baselineGasResistance - currentGasRes) / baselineGasResistance);
	
	// IAQ分量
	float iaqComponent = max(0.0f, (currentIAQ - 100) / 100.0f);
	
	// 綜合評分
	odorScore = (vocComponent * VOC_WEIGHT) + (gasResComponent * GAS_RES_WEIGHT) + (iaqComponent * IAQ_WEIGHT);
	
	// 檢測臭味
	bool odorDetected = odorScore > 0.5;
	
	// 輸出檢測結果
	if (odorDetected && !odorAlertActive) {
		Serial.printf("🚨 垃圾桶臭味警報!\n綜合評分: %.2f  VOCs: %.3f ppm (變化率: %.1f%)  氣體電阻: %.4f Ω (變化率: %.1f%)  IAQ: %f \n",
			odorScore, currentVoc, vocChange * 100, currentGasRes, gasResChange * 100, currentIAQ
		);
		odorAlertActive = true;
	} else if (!odorDetected && odorAlertActive) {
		Serial.println("✅ 臭味警報解除");
		odorAlertActive = false;
	}
}

void processSensorData() {
	static uint32_t lastOutput = 0;
	// 3秒輸出一次
	if (millis() - lastOutput < 3000) {
		return;
	}
	lastOutput = millis();
	
	if (!outputs) {
		return;
	}
	
	// 輸出感測器資料
	Serial.println("=== 感測器讀數 ===");
	
	// 使用動態取得方式
	float temperature         = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE);
	float humidity            = getOutputSignal(BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY);
	float iaq                 = getOutputSignal(BSEC_OUTPUT_IAQ);
	float staticIaq           = getOutputSignal(BSEC_OUTPUT_STATIC_IAQ);
	float co2Equivalent       = getOutputSignal(BSEC_OUTPUT_CO2_EQUIVALENT);
	float breathVocEquivalent = getOutputSignal(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT);
	float rawGas              = getOutputSignal(BSEC_OUTPUT_RAW_GAS);
	float gasPercentage       = getOutputSignal(BSEC_OUTPUT_GAS_PERCENTAGE);
	
	// 取得 IAQ 精度
	uint8_t iaqAccuracy = 0;
	for (int i = 0; i < outputs -> nOutputs; i++) {
		if (outputs -> output[i].sensor_id == BSEC_OUTPUT_IAQ) {
			iaqAccuracy = outputs -> output[i].accuracy;
			break;
		}
	}

	Serial.printf("溫度: %.1f°C | 濕度: %.1f% | IAQ: %.2f (精度: %d/3) | 靜態IAQ: %.2f | CO₂等效: %.0fppm | VOCs等效: %.3fppm | 氣體電阻: %.4f Ω | 氣體百分比: %.1f% \n",
		temperature, humidity, iaq, iaqAccuracy, staticIaq, co2Equivalent, breathVocEquivalent, rawGas, gasPercentage
	);
	
	// 校準狀態提示
	printCalibrationAdvice();
}

void printCalibrationAdvice() {
	static uint32_t lastAdviceTime = 0;
	// 每30秒提示一次
	if (millis() - lastAdviceTime < 30000) {
		return;
	}
	lastAdviceTime = millis();
	
	if (!outputs) {
		return;
	}
	
	// 取得IAQ精度
	uint8_t accuracy = 0;
	for (int i = 0; i < outputs -> nOutputs; i++) {
		if (outputs -> output[i].sensor_id == BSEC_OUTPUT_IAQ) {
			accuracy = outputs -> output[i].accuracy;
			break;
		}
	}
	
	// 顯示校準進度
	if (calibrationInProgress) {
		
		Serial.printf("校準進度: %d/%d 清潔空氣樣本 \n", cleanAirSampleCount, MIN_CLEAN_AIR_SAMPLES);
		
		if (!baselineValid) {
			Serial.println("💡 校準提示: 請確保垃圾桶已清空且乾淨");
			Serial.println("   - 建議在清空垃圾桶後啟動系統");
			Serial.println("   - 保持垃圾桶蓋關閉以獲得穩定讀數");
		}
	} else {
		Serial.println("校準已完成 基線有效");
	}
	
	// 顯示IAQ精度
	Serial.printf("IAQ 精度: %d/3 \n", accuracy);
}

void loadCalibrationState() {
	uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
	size_t size = preferences.getBytes("bsec_state", state, sizeof(state));
	
	if (size > 0) {
		if (iaqSensor.setState(state) != BSEC_OK) {    // 修正為只傳遞一個參數
			Serial.println("載入的狀態無效!");
		} else {
			Serial.println("已載入 BSEC 校準狀態");
		}
	} else {
		Serial.println("無儲存的校準狀態 開始新校準");
	}
}

void saveState() {
	// 使用動態記憶體設定避免堆疊溢位
	uint8_t* state = (uint8_t*)malloc(BSEC_MAX_STATE_BLOB_SIZE);
	if (!state) {
		Serial.println("無法設定狀態記憶體");
		return;
	}
	
	uint32_t numSerializedState = iaqSensor.getState(state);
	
	if (numSerializedState > 0) {
		preferences.putBytes("bsec_state", state, numSerializedState);
	}
	
	free(state);
}

void EnvironmentalSensor::handleSerialCommands(std::vector<std::string> commands) {
	String command = String((commands[1]).c_str());
	
	if (command == "reset") {
		preferences.remove("bsec_state");
		Serial.println("校準已重設 請重啟裝置");
	} else if (command == "status") {
		if (outputs) {
			Serial.println("=== 當前感測器狀態 ===");
			// 取得IAQ精度
			uint8_t iaqAccuracy = 0;
			for (int i = 0; i < outputs -> nOutputs; i++) {
				if (outputs -> output[i].sensor_id == BSEC_OUTPUT_IAQ) {
					iaqAccuracy = outputs -> output[i].accuracy;
					break;
				}
			}

			Serial.printf("IAQ精度: %d/3  目前VOCs: %.3f ppm  當前氣體電阻: %.2f Ω  氣體百分比: %.1f  基線氣體電阻: %.2f  校準狀態: %s  臭味警報: %s \n",
				iaqAccuracy,
				getOutputSignal(BSEC_OUTPUT_BREATH_VOC_EQUIVALENT),
				getOutputSignal(BSEC_OUTPUT_RAW_GAS),
				getOutputSignal(BSEC_OUTPUT_GAS_PERCENTAGE),
				baselineGasResistance,
				(baselineValid ? "有效" : "無效"), (odorAlertActive ? "啟動中" : "未啟動")
			);
		} else {
			Serial.println("無有效資料");
		}
	} else if (command == "calibrate") {
		calibrateBaseline();
		Serial.println("重新開始校準過程");
	} else if (command == "threshold") {
		Serial.printf("當前臭味閾值: VOCs > %.1f ppm  氣體電阻: %f Ω \n", ODOR_THRESHOLD, RAW_GAS_THRESHOLD);
	} else if (command == "weights") {
		Serial.printf("當前融合權重:\nVOCs 權重: %f  氣體電阻權重: %f  IAQ 權重: %f \n", VOC_WEIGHT, GAS_RES_WEIGHT, IAQ_WEIGHT);
	} else if (command == "help") {
		Serial.printf("%s\n%s\n%s\n%s\n%s\n%s\n%s\n",
			"可用指令:",
			"envSensor.reset - 重設校準",
			"envSensor.status - 檢視當前狀態",
			"envSensor.calibrate - 重新校準",
			"envSensor.threshold - 顯示臭味閾值",
			"envSensor.weights - 顯示融合權重",
			"envSensor.help - 顯示說明"
		);
	}
}

void checkSensorStatus() {
	if (iaqSensor.status != BSEC_OK) {
		Serial.print("BSEC 錯誤: ");
		Serial.println(iaqSensor.status);
	}
	
	if (iaqSensor.sensor.status != BME68X_OK) {
		Serial.print("BME680 錯誤: ");
		Serial.println(iaqSensor.sensor.status);
		
		switch (iaqSensor.sensor.status) {
			case BME68X_E_COM_FAIL: {
				Serial.println("通訊失敗 檢查 I2C 連接");
				break;
			}
			case BME68X_E_SELF_TEST: {
				Serial.println("自檢失敗 感測器可能損壞");
				break;
			}
			default: {
				Serial.println("未知錯誤");
			}
		}
	}
}
