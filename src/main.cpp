#include <Arduino.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <WebServer.h>
#include <Preferences.h>
#include <time.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <ArduinoJson.h>

// ==================== PIN DEFINITIONS ====================
#define PASSIVE_BEEPER 26
#define PIR_PIN 25

// OLED Display (SSD1322 SPI)
#define SCREEN_WIDTH 256
#define SCREEN_HEIGHT 64
#define PIN_CS 5       // /CS
#define PIN_DC 27      // D/C (moved from GPIO16 to free UART2)
#define PIN_RST 33     // /RES (moved from GPIO17 to free UART2)
#define PIN_CLK 18     // SCLK
#define PIN_MOSI 23    // SDIN

// Loctek ET201-75W Desk Controller (UART via RJ45 bypass)
// NOTE: GPIO16/17 cannot be used on ESP32-WROVER (PSRAM conflict)
#define DESK_UART_RX 14   // GPIO14 - receives height data from controller (RJ45 pin 5)
#define DESK_UART_TX 13   // GPIO13 - sends commands to controller (RJ45 pin 6)
#define KEYPAD_RX 4        // GPIO4 - monitors keypad button presses (RJ45 pin 6)
#define LOCTEK_PIN20 12    // GPIO12 - RJ45 pin 20 (controller screen enable, active HIGH)

// ==================== FORWARD DECLARATIONS ====================
void loadConfig();
void saveConfig();
void loadStats();
void saveStats();
void resetDailyStats();
void resetWeeklyStats();
void resetMonthlyStats();
void resetYearlyStats();
void resetAllStats();
void syncTime();
void checkPresence();
void checkDeskHeight();
void updateSessionTimes();
void checkSittingTimeout();
void finalizeSession();
bool isOperatingHours();
void checkDayRollover();
void checkWeekRollover();
void checkMonthRollover();
void checkYearRollover();
void beep(int duration);
float measureDeskHeight();
void initLoctekUART();
void sendLoctekCommand(const uint8_t* cmd, size_t len);
void sendDeskWake();
void sendDeskUp();
void sendDeskDown();
void sendDeskStop();
void sendDeskPreset(int preset);
void readDeskHeight();
float decodeLoctekHeight(uint8_t d1, uint8_t d2, uint8_t d3);
void handleDeskUp();
void handleDeskDown();
void handleDeskStop();
void handleDeskPreset();
String formatDuration(unsigned long seconds);
String formatDurationShort(unsigned long seconds);
String formatPercentage(unsigned long sit, unsigned long stand);
void printCurrentConfig();
void printCurrentStats();
void displayBootInfo();
void displayCurrentStats();
void setupWebServer();
void handleRoot();
void handleConfig();
void handleConfigSave();
void handleCalibrate();
void handleCalibrateSit();
void handleCalibrateStand();
void handleStats();
void handleResetStats();
void handleResetDaily();
void handleResetWeekly();
void handleResetMonthly();
void handleResetYearly();
void handleUpdateDailyStats();
void handleTestBeep();
void handleTestRelay();
void handleTestStanding();
void handleSensorUpdate();
void handleNotFound();
void handleResetAmazfit();

// ==================== CONFIGURATION ====================
struct Config {
  int sitTimeout;
  int operationStartHour;
  int operationEndHour;
  float sittingHeightCm;
  float standingHeightCm;
  float heightTolerance;
  bool debugMode;
  int beepDuration;
  int pirTimeoutMinutes;
  int complianceWaitSeconds;  // Time to wait for user to stand before forcing
  bool beeperEnabled;
  int positionChangeDelaySeconds;  // Time desk must stay at new height before confirming position change
};

Config config = {
  60,      // sitTimeout (60 minutes)
  8,       // operationStartHour (8 AM)
  18,      // operationEndHour (6 PM)
  75.0,    // sittingHeightCm
  115.0,   // standingHeightCm
  5.0,     // heightTolerance
  false,   // debugMode
  2000,    // beepDuration
  2,       // pirTimeoutMinutes (2 minutes default)
  30,      // complianceWaitSeconds (30 seconds default)
  true,    // beeperEnabled (true by default)
  5        // positionChangeDelaySeconds (5 seconds default)
};

// ==================== STATISTICS ====================
struct DailyStats {
  unsigned long totalAtDesk;
  unsigned long totalSitting;
  unsigned long totalStanding;
  int alertedCount;
  int forcedStandingCount;
  int ignoredWarningCount;
  int date;
};

struct WeeklyStats {
  unsigned long totalAtDesk;
  unsigned long totalSitting;
  unsigned long totalStanding;
  int alertedCount;
  int forcedStandingCount;
  int ignoredWarningCount;
  int week;
  int year;
  int month;
};

struct MonthlyStats {
  unsigned long totalAtDesk;
  unsigned long totalSitting;
  unsigned long totalStanding;
  int alertedCount;
  int forcedStandingCount;
  int ignoredWarningCount;
  int month;
  int year;
};

struct YearlyStats {
  unsigned long totalAtDesk;
  unsigned long totalSitting;
  unsigned long totalStanding;
  int alertedCount;
  int forcedStandingCount;
  int ignoredWarningCount;
  int year;
};

struct CurrentSession {
  unsigned long sessionStartTime;
  unsigned long sittingStartTime;
  unsigned long standingStartTime;
  unsigned long currentSittingTime;
  unsigned long currentStandingTime;
  unsigned long totalSessionSitting;   // Accumulated sitting time for current session
  unsigned long totalSessionStanding;  // Accumulated standing time for current session
  unsigned long lastMotionTime;
  bool atDesk;
  bool isSitting;
  bool warningIssued;
  bool forcingStanding;  // Flag to prevent re-entry during forced standing
  unsigned long positionChangeDetectedTime;  // When potential position change was first detected
  bool pendingPositionChange;  // True if we're waiting to confirm a position change
  bool pendingIsSitting;  // The pending position (true = sitting, false = standing)
};

DailyStats dailyStats = {0, 0, 0, 0, 0, 0, 0};
WeeklyStats weeklyStats = {0, 0, 0, 0, 0, 0, 0, 0, 0};
MonthlyStats monthlyStats = {0, 0, 0, 0, 0, 0, 0, 0};
YearlyStats yearlyStats = {0, 0, 0, 0, 0, 0, 0};
CurrentSession session = {0, 0, 0, 0, 0, 0, 0, 0, false, false, false, false, 0, false, false};

// ==================== LOCTEK PROTOCOL ====================
// Seven-segment display encoding lookup (hex value -> digit)
// The desk controller sends height as 3 bytes of seven-segment encoded digits
const uint8_t SEVEN_SEG_DIGITS[] = {
  0x3F, // 0
  0x06, // 1
  0x5B, // 2
  0x4F, // 3
  0x66, // 4
  0x6D, // 5
  0x7D, // 6
  0x07, // 7
  0x7F, // 8
  0x6F, // 9
};

// Loctek command frames: start(9B) + len(06) + type(02) + payload(2) + checksum(2) + end(9D)
const uint8_t CMD_WAKE[]     = {0x9b, 0x06, 0x02, 0x00, 0x00, 0x6c, 0xa1, 0x9d};
const uint8_t CMD_UP[]       = {0x9b, 0x06, 0x02, 0x01, 0x00, 0xfc, 0xa0, 0x9d};
const uint8_t CMD_DOWN[]     = {0x9b, 0x06, 0x02, 0x02, 0x00, 0x0c, 0xa0, 0x9d};
const uint8_t CMD_PRESET_1[] = {0x9b, 0x06, 0x02, 0x04, 0x00, 0xac, 0xa3, 0x9d};
const uint8_t CMD_PRESET_2[] = {0x9b, 0x06, 0x02, 0x08, 0x00, 0xac, 0xa6, 0x9d};
const uint8_t CMD_PRESET_3[] = {0x9b, 0x06, 0x02, 0x10, 0x00, 0xac, 0xac, 0x9d};  // Stand
const uint8_t CMD_PRESET_4[] = {0x9b, 0x06, 0x02, 0x00, 0x01, 0xac, 0x60, 0x9d};  // Sit
const size_t CMD_LENGTH = 8;

#define LOCTEK_BAUD 9600
#define LOCTEK_FRAME_START 0x9b
#define LOCTEK_FRAME_END 0x9d

// ==================== GLOBALS ====================
WebServer server(80);
Preferences preferences;
WiFiManager wifiManager;

// OLED Display
U8G2_SSD1322_NHD_256X64_F_4W_HW_SPI display(U8G2_R0, PIN_CS, PIN_DC, PIN_RST);

// Loctek desk controller
HardwareSerial deskSerial(2);   // UART2 for desk controller communication
float loctekHeight = 0.0;      // Current desk height from controller (cm)
unsigned long lastWakeCommand = 0;
const unsigned long WAKE_INTERVAL = 15000;  // Send wake every 15s to keep height broadcasting
uint8_t deskRxBuffer[20];      // Buffer for incoming desk controller data
int deskRxIndex = 0;
bool deskScreenActive = false;

// UART debug tracking
unsigned long uartBytesReceived = 0;    // Total bytes received on DESK_UART_RX
unsigned long uartFramesReceived = 0;   // Complete frames parsed
unsigned long uartValidHeights = 0;     // Valid height values decoded
unsigned long lastUartByteTime = 0;     // millis() of last byte received
unsigned long lastUartDebugPrint = 0;   // For periodic console debug output
const unsigned long UART_DEBUG_INTERVAL = 10000; // Print UART status every 10s

// Amazfit sensor data
int steps = 0;
int calories = 0;
float amazfitDistance = 0.0;
int heartRate = 0;
float weight = 0.0;
bool amazfitConnected = false;
String trigger_3 = "";
String trigger_6 = "";
String trigger_7 = "";
String as_sensor = "";
bool sleeping = false;
String sleepState = "";
int sleepDuration = 0;
int battery = 0;
int weeklyTotalSteps = 0;  // Accumulated steps from previous days this week

unsigned long lastPirCheck = 0;
unsigned long lastHeightCheck = 0;
unsigned long lastStatsSave = 0;
unsigned long lastDisplayUpdate = 0;
const int PIR_CHECK_INTERVAL = 1000;
const int HEIGHT_CHECK_INTERVAL = 5000;
const int STATS_SAVE_INTERVAL = 3600000;
const int DISPLAY_UPDATE_INTERVAL = 1000;  // Update display every 1 second

const int CST_OFFSET = -21600;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(3000);
  Serial.println("\n\n=== Smart Standing Desk Monitor ===");
  Serial.println("Step 1: Initializing pins...");

  pinMode(PASSIVE_BEEPER, OUTPUT);
  pinMode(PIR_PIN, INPUT);
  pinMode(LOCTEK_PIN20, OUTPUT);

  digitalWrite(PASSIVE_BEEPER, LOW);
  digitalWrite(LOCTEK_PIN20, HIGH);  // Enable controller screen via RJ45 pin 20
  
  Serial.println("Step 2: Pins initialized");

  // Initialize Loctek desk controller UART
  initLoctekUART();
  Serial.println("Step 2b: Loctek UART initialized");

  // Initialize OLED display
  Serial.println("Step 3: Initializing OLED display...");
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 10, "Desk Monitor Starting...");
  display.sendBuffer();
  Serial.println("OLED display initialized");

  preferences.begin("desk-config", false);
  Serial.println("Step 4: Preferences (EEPROM) opened");
  
  loadConfig();
  Serial.println("Step 5: Config loaded from EEPROM");
  
  loadStats();
  Serial.println("Step 6: Stats loaded from EEPROM");
  printCurrentStats();

  Serial.println("Step 7: Starting WiFi...");
  display.clearBuffer();
  display.drawStr(0, 10, "Connecting WiFi...");
  display.sendBuffer();
  
  wifiManager.setConfigPortalTimeout(180);
  
  Serial.println("Step 8: Calling autoConnect...");
  if (!wifiManager.autoConnect("DeskMonitor-Setup")) {
    Serial.println("Failed to connect, restarting...");
    display.clearBuffer();
    display.drawStr(0, 10, "WiFi Failed!");
    display.drawStr(0, 25, "Restarting...");
    display.sendBuffer();
    delay(3000);
    ESP.restart();
  }

  Serial.println("Step 9: WiFi connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  syncTime();
  Serial.println("Step 10: Time synced");
  
  checkDayRollover();
  checkWeekRollover();
  checkMonthRollover();
  checkYearRollover();

  setupWebServer();
  server.begin();
  Serial.println("Step 11: Web server started");

  // Display boot info on OLED
  displayBootInfo();
  delay(5000);  // Show boot info for 5 seconds

  beep(200);
  delay(100);
  beep(200);
  
  Serial.println("\n=== System ready! ===");
  Serial.printf("PIR Timeout: %d minutes\n", config.pirTimeoutMinutes);
  Serial.printf("Stats will be saved to EEPROM every %d minutes\n", STATS_SAVE_INTERVAL / 60000);
  printCurrentConfig();
  
  // Show initial stats
  displayCurrentStats();
}

// ==================== MAIN LOOP ====================
void loop() {
  server.handleClient();

  // Continuously read desk height data from Loctek controller
  readDeskHeight();

  unsigned long currentMillis = millis();

  // Periodically send wake command to keep controller broadcasting height
  if (currentMillis - lastWakeCommand >= WAKE_INTERVAL) {
    lastWakeCommand = currentMillis;
    sendDeskWake();
  }

  if (currentMillis - lastPirCheck >= PIR_CHECK_INTERVAL) {
    lastPirCheck = currentMillis;
    checkPresence();
  }
  
  if (currentMillis - lastHeightCheck >= HEIGHT_CHECK_INTERVAL) {
    lastHeightCheck = currentMillis;
    if (session.atDesk) {
      checkDeskHeight();
    }
  }

  // Periodic UART debug status to serial console
  if (currentMillis - lastUartDebugPrint >= UART_DEBUG_INTERVAL) {
    lastUartDebugPrint = currentMillis;
    unsigned long secsSinceLastByte = lastUartByteTime > 0 ? (currentMillis - lastUartByteTime) / 1000 : 0;
    Serial.printf("[UART DEBUG] RX Pin: GPIO%d | Bytes: %lu | Frames: %lu | Valid Heights: %lu | Height: %.1f cm | Last byte: %lus ago\n",
                  DESK_UART_RX, uartBytesReceived, uartFramesReceived, uartValidHeights, loctekHeight, secsSinceLastByte);
    if (uartBytesReceived == 0) {
      Serial.println("[UART DEBUG] WARNING: No bytes received on DESK_UART_RX - check wiring to RJ45 pin 5 / GPIO14");
    }
  }
  
  // Update session times every loop if at desk
  if (session.atDesk) {
    updateSessionTimes();
    checkSittingTimeout();
  }
  
  if (currentMillis - lastStatsSave >= STATS_SAVE_INTERVAL) {
    lastStatsSave = currentMillis;
    Serial.println("=== Hourly stats save to EEPROM ===");
    saveStats();
  }
  
  // Update OLED display periodically
  if (currentMillis - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    lastDisplayUpdate = currentMillis;
    displayCurrentStats();
    
    // Debug output to serial
    if (config.debugMode && session.atDesk) {
      if (session.isSitting) {
        Serial.printf("Display update - Sitting: %s\n", formatDurationShort(session.currentSittingTime).c_str());
      } else {
        Serial.printf("Display update - Standing: %s\n", formatDurationShort(session.currentStandingTime).c_str());
      }
    }
  }
  
  checkDayRollover();
  checkWeekRollover();
  checkMonthRollover();
  checkYearRollover();
  
  delay(10);
}

// ==================== DISPLAY FUNCTIONS ====================
void displayBootInfo() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  
  // Title
  display.drawStr(0, 10, "DESK MONITOR READY");
  
  // IP Address
  String ipStr = "IP: " + WiFi.localIP().toString();
  display.drawStr(0, 25, ipStr.c_str());
  
  // Settings
  char settingsBuf[64];
  sprintf(settingsBuf, "Timeout: %dm PIR: %dm", config.sitTimeout, config.pirTimeoutMinutes);
  display.drawStr(0, 38, settingsBuf);
  
  sprintf(settingsBuf, "Hours: %02d:00-%02d:00", config.operationStartHour, config.operationEndHour);
  display.drawStr(0, 51, settingsBuf);
  
  sprintf(settingsBuf, "Sit:%.1fcm Stand:%.1fcm", config.sittingHeightCm, config.standingHeightCm);
  display.drawStr(0, 64, settingsBuf);
  
  display.sendBuffer();
}

void displayCurrentStats() {
  display.clearBuffer();
  display.setFont(u8g2_font_6x10_tf);
  
  char buf[64];
  
  // Calculate display totals (accumulated + current session)
  unsigned long displayDailyDesk = dailyStats.totalAtDesk;
  unsigned long displayDailySitting = dailyStats.totalSitting;
  unsigned long displayDailyStanding = dailyStats.totalStanding;
  
  unsigned long displayWeeklyDesk = weeklyStats.totalAtDesk;
  unsigned long displayWeeklySitting = weeklyStats.totalSitting;
  unsigned long displayWeeklyStanding = weeklyStats.totalStanding;
  
  // Add current session to display totals
  if (session.atDesk) {
    unsigned long currentSessionTime = (millis() - session.sessionStartTime) / 1000;
    displayDailyDesk += currentSessionTime;
    displayWeeklyDesk += currentSessionTime;

    // Add accumulated session sitting/standing times
    displayDailySitting += session.totalSessionSitting;
    displayWeeklySitting += session.totalSessionSitting;
    displayDailyStanding += session.totalSessionStanding;
    displayWeeklyStanding += session.totalSessionStanding;

    // Add current continuous sitting or standing time
    if (session.isSitting && session.sittingStartTime > 0) {
      unsigned long currentSitting = (millis() - session.sittingStartTime) / 1000;
      displayDailySitting += currentSitting;
      displayWeeklySitting += currentSitting;
    } else if (!session.isSitting && session.standingStartTime > 0) {
      unsigned long currentStanding = (millis() - session.standingStartTime) / 1000;
      displayDailyStanding += currentStanding;
      displayWeeklyStanding += currentStanding;
    }
  }
  
  // Get current time for day display
  time_t now = time(nullptr) + CST_OFFSET;
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  
  // === LEFT COLUMN (Daily) ===
  // Line 1: Today: day/time
  sprintf(buf, "Today:%d/%02d %02d:%02d", 
          timeinfo.tm_mon + 1, timeinfo.tm_mday,
          timeinfo.tm_hour, timeinfo.tm_min);
  display.drawStr(0, 10, buf);
  
  // Line 2: @Desk time
  sprintf(buf, "@Desk %s", formatDurationShort(displayDailyDesk).c_str());
  display.drawStr(0, 23, buf);
  
  // Line 3: Sit vs Stand
  sprintf(buf, "Sit:%s vs %s", 
          formatDurationShort(displayDailySitting).c_str(),
          formatDurationShort(displayDailyStanding).c_str());
  display.drawStr(0, 36, buf);
  
  // Line 4: Steps
  sprintf(buf, "Steps:%d", steps);
  display.drawStr(0, 49, buf);
  
  // === RIGHT COLUMN (Weekly) ===
  // Line 1: Week: month/week
  sprintf(buf, "Week:%d/W%d", 
          weeklyStats.month, weeklyStats.week);
  display.drawStr(128, 10, buf);
  
  // Line 2: Average @Desk time per day this week
  int daysThisWeek = timeinfo.tm_wday == 0 ? 7 : timeinfo.tm_wday;
  unsigned long avgDeskPerDay = displayWeeklyDesk / daysThisWeek;
  sprintf(buf, "@Desk %s/day", formatDurationShort(avgDeskPerDay).c_str());
  display.drawStr(128, 23, buf);
  
  // Line 3: Average Sit vs Stand per day
  unsigned long avgSitPerDay = displayWeeklySitting / daysThisWeek;
  unsigned long avgStandPerDay = displayWeeklyStanding / daysThisWeek;
  sprintf(buf, "Sit:%s vs %s",
          formatDurationShort(avgSitPerDay).c_str(),
          formatDurationShort(avgStandPerDay).c_str());
  display.drawStr(128, 36, buf);
  
  // Line 4: Average steps per day this week (accumulated + today's)
  int avgSteps = (weeklyTotalSteps + steps) / daysThisWeek;
  sprintf(buf, "Avg:%d/day", avgSteps);
  display.drawStr(128, 49, buf);
  
  // === BOTTOM LINE (y=62) - Current status on left, Sit/graph/percentage on right ===
  // Status text on the left side
  if (session.atDesk) {
    if (session.isSitting) {
      unsigned long sittingTime = 0;
      if (session.sittingStartTime > 0) {
        sittingTime = (millis() - session.sittingStartTime) / 1000;
      }
      sprintf(buf, "Sitting %s", formatDurationShort(sittingTime).c_str());
    } else {
      unsigned long standingTime = 0;
      if (session.standingStartTime > 0) {
        standingTime = (millis() - session.standingStartTime) / 1000;
      }
      sprintf(buf, "Standing %s", formatDurationShort(standingTime).c_str());
    }
  } else {
    sprintf(buf, "Away");
  }
  display.drawStr(0, 62, buf);
  
  // Calculate sit/stand percentages for the bar graph (using daily stats)
  unsigned long totalPosture = displayDailySitting + displayDailyStanding;
  int sitPercent = 0;
  if (totalPosture > 0) {
    sitPercent = (displayDailySitting * 100) / totalPosture;
  }
  
  // Bar graph on the right side of bottom line
  int barWidth = 50;
  int barHeight = 8;
  int barX = 160;  // Position to fit label, bar, and percentage on bottom line
  int barY = 54;   // Position so bar aligns with text at y=62
  
  // Draw "Sit:" label before the bar on bottom line
  display.drawStr(128, 62, "Sit:");
  
  // Draw bar border
  display.drawFrame(barX, barY, barWidth, barHeight);
  
  // Fill sit portion (left side of bar)
  if (sitPercent > 0) {
    int sitWidth = (barWidth - 2) * sitPercent / 100;  // -2 for border
    if (sitWidth > 0) {
      display.drawBox(barX + 1, barY + 1, sitWidth, barHeight - 2);
    }
  }
  
  // Draw percentage after the bar on bottom line
  sprintf(buf, "%d%%", sitPercent);
  display.drawStr(barX + barWidth + 3, 62, buf);
  
  display.sendBuffer();
}

// ==================== SENSOR FUNCTIONS ====================
void checkPresence() {
  bool pirState = digitalRead(PIR_PIN);
  
  if (pirState == HIGH) {
    session.lastMotionTime = millis();

    if (!session.atDesk) {
      Serial.println("=== PRESENCE DETECTED - Starting session ===");
      session.atDesk = true;
      session.sessionStartTime = millis();
      session.warningIssued = false;

      // Check initial desk height
      checkDeskHeight();
    }
  } else {
    // Check if we've exceeded PIR timeout
    if (session.atDesk) {
      unsigned long timeSinceMotion = (millis() - session.lastMotionTime) / 1000;
      unsigned long pirTimeoutSeconds = config.pirTimeoutMinutes * 60;
      
      if (timeSinceMotion >= pirTimeoutSeconds) {
        Serial.printf("=== NO MOTION for %d minutes - Ending session ===\n", config.pirTimeoutMinutes);
        finalizeSession();
        session.atDesk = false;
        session.warningIssued = false;
        
        // Reset current session counters
        session.currentSittingTime = 0;
        session.currentStandingTime = 0;
        session.totalSessionSitting = 0;
        session.totalSessionStanding = 0;
        session.sittingStartTime = 0;
        session.standingStartTime = 0;
        session.sessionStartTime = 0;
        session.isSitting = false;  // Reset position to default
        session.pendingPositionChange = false;  // Clear any pending changes
      }
    }
  }
}

void checkDeskHeight() {
  float height = measureDeskHeight();

  if (height <= 0) {
    Serial.println("Invalid height measurement - defaulting to sitting position");
    // Default to sitting when measurement fails
    height = config.sittingHeightCm;
  }

  // Determine what position the current height indicates
  bool heightIndicatesSitting = (height <= (config.sittingHeightCm + config.heightTolerance));

  // Check if this matches our current confirmed position
  if (heightIndicatesSitting == session.isSitting) {
    // Height matches current position - cancel any pending change
    if (session.pendingPositionChange) {
      session.pendingPositionChange = false;
      Serial.println("Position change cancelled - height returned to current position");
    }

    // Initialize timer if not already running (e.g., just returned from AWAY)
    if (session.isSitting && session.sittingStartTime == 0) {
      session.sittingStartTime = millis();
      Serial.println("Initialized sitting timer (returning from AWAY in sitting position)");
    } else if (!session.isSitting && session.standingStartTime == 0) {
      session.standingStartTime = millis();
      Serial.println("Initialized standing timer (returning from AWAY in standing position)");
    }

    return;  // No change needed
  }

  // Height indicates a different position than current
  // Check if we're already tracking a pending change
  if (!session.pendingPositionChange) {
    // Start tracking a new pending position change
    session.pendingPositionChange = true;
    session.pendingIsSitting = heightIndicatesSitting;
    session.positionChangeDetectedTime = millis();
    Serial.printf("Potential position change detected: %s -> %s (waiting %d seconds)\n",
                  session.isSitting ? "SITTING" : "STANDING",
                  heightIndicatesSitting ? "SITTING" : "STANDING",
                  config.positionChangeDelaySeconds);
    return;
  }

  // We have a pending change - check if position is still consistent
  if (session.pendingIsSitting != heightIndicatesSitting) {
    // Position changed again - restart the timer
    session.pendingIsSitting = heightIndicatesSitting;
    session.positionChangeDetectedTime = millis();
    Serial.printf("Pending position changed - restarting timer for: %s\n",
                  heightIndicatesSitting ? "SITTING" : "STANDING");
    return;
  }

  // Check if enough time has elapsed
  unsigned long elapsed = (millis() - session.positionChangeDetectedTime) / 1000;
  if (elapsed < config.positionChangeDelaySeconds) {
    // Still waiting for confirmation
    return;
  }

  // Delay has elapsed - confirm the position change
  session.pendingPositionChange = false;
  bool wasSitting = session.isSitting;
  session.isSitting = heightIndicatesSitting;

  // Handle position change confirmation
  if (session.isSitting) {
    // Confirmed SITTING position
    if (!wasSitting && session.standingStartTime > 0) {
      // Transitioned from standing to sitting - accumulate standing time
      unsigned long standingDuration = (millis() - session.standingStartTime) / 1000;
      session.totalSessionStanding += standingDuration;
      Serial.println("=== Position change CONFIRMED: STANDING -> SITTING ===");
      Serial.printf("Accumulated standing time: %s\n", formatDurationShort(standingDuration).c_str());
    } else {
      // First time detecting sitting or no previous standing time tracked
      Serial.println("=== Position CONFIRMED: SITTING ===");
    }
    // Always set sitting start time and clear standing
    session.sittingStartTime = millis();
    session.standingStartTime = 0;
    session.warningIssued = false;  // Reset warning for new sitting period

  } else {
    // Confirmed STANDING position
    if (wasSitting && session.sittingStartTime > 0) {
      // Transitioned from sitting to standing - accumulate sitting time
      unsigned long sittingDuration = (millis() - session.sittingStartTime) / 1000;
      session.totalSessionSitting += sittingDuration;
      Serial.println("=== Position change CONFIRMED: SITTING -> STANDING ===");
      Serial.printf("Accumulated sitting time: %s\n", formatDurationShort(sittingDuration).c_str());
    } else {
      // First time detecting standing or no previous sitting time tracked
      Serial.println("=== Position CONFIRMED: STANDING ===");
    }
    // Always set standing start time and clear sitting
    session.standingStartTime = millis();
    session.sittingStartTime = 0;
    session.warningIssued = false;  // Reset warning when user voluntarily stands
  }
  
  if (config.debugMode) {
    Serial.printf("Height: %.1f cm | ", height);
    Serial.printf("Sit ref: %.1f | Stand ref: %.1f | ", 
                  config.sittingHeightCm, config.standingHeightCm);
    Serial.printf("Position: %s\n", session.isSitting ? "SITTING" : "STANDING");
  }
}

void updateSessionTimes() {
  if (session.isSitting && session.sittingStartTime > 0) {
    session.currentSittingTime = (millis() - session.sittingStartTime) / 1000;
  } else if (!session.isSitting && session.standingStartTime > 0) {
    session.currentStandingTime = (millis() - session.standingStartTime) / 1000;
  }
}

void checkSittingTimeout() {
  if (!isOperatingHours()) return;
  if (!session.isSitting) return;
  if (session.sittingStartTime == 0) return;
  if (session.forcingStanding) return;  // Prevent re-entry during forced standing

  unsigned long sittingMinutes = session.currentSittingTime / 60;

  if (sittingMinutes >= config.sitTimeout && !session.warningIssued) {
    Serial.println("\n=== SITTING TIMEOUT REACHED ===");
    Serial.printf("Sitting time: %d minutes (threshold: %d)\n",
                  sittingMinutes, config.sitTimeout);

    // Set flags FIRST to prevent duplicate execution
    session.warningIssued = true;
    session.forcingStanding = true;

    // Increment alerted count (warning issued)
    dailyStats.alertedCount++;
    weeklyStats.alertedCount++;
    monthlyStats.alertedCount++;
    yearlyStats.alertedCount++;

    // Issue warning beeps (if enabled)
    if (config.beeperEnabled) {
      for (int i = 0; i < 3; i++) {
        beep(config.beepDuration / 4);
        delay(200);
      }
    }

    // Wait for compliance with countdown display
    Serial.printf("Waiting %d seconds for user to stand...\n", config.complianceWaitSeconds);
    unsigned long waitStart = millis();
    unsigned long waitDuration = config.complianceWaitSeconds * 1000;

    while ((millis() - waitStart) < waitDuration) {
      server.handleClient();
      checkDeskHeight();

      if (!session.isSitting) {
        Serial.println("User complied - standing detected!");
        session.forcingStanding = false;
        // Restore normal display
        displayCurrentStats();
        return;
      }

      // Update display with countdown every second
      unsigned long elapsed = (millis() - waitStart) / 1000;
      unsigned long remaining = config.complianceWaitSeconds - elapsed;

      display.clearBuffer();
      display.setFont(u8g2_font_fur20_tf);  // Large font
      display.drawStr(30, 35, "Stand Up!");
      display.setFont(u8g2_font_fur17_tn);  // Large number font
      char countdownBuf[4];
      sprintf(countdownBuf, "%lu", remaining);
      // Center the countdown number
      int numWidth = display.getStrWidth(countdownBuf);
      display.drawStr((256 - numWidth) / 2, 60, countdownBuf);
      display.sendBuffer();

      delay(100);
    }

    // Restore normal font for subsequent displays
    display.setFont(u8g2_font_6x10_tf);
    
    // User didn't comply - force standing
    Serial.println("User did not comply - FORCING STANDING");

    // Long warning beeps (if enabled)
    if (config.beeperEnabled) {
      for (int i = 0; i < 3; i++) {
        beep(config.beepDuration);
        delay(300);
      }
    }

    // Send Loctek preset 3 (stand) to raise desk
    Serial.println("Sending Loctek stand command to force standing position...");
    sendDeskPreset(3);

    // Increment forced standing count
    dailyStats.forcedStandingCount++;
    weeklyStats.forcedStandingCount++;
    monthlyStats.forcedStandingCount++;
    yearlyStats.forcedStandingCount++;

    // Wait and check if desk reaches standing position (poll for up to 15 seconds)
    Serial.println("Monitoring desk movement to standing position...");
    bool reachedStanding = false;
    unsigned long checkStart = millis();

    while ((millis() - checkStart) < 15000) {
      delay(1000);  // Check every second
      float currentHeight = measureDeskHeight();
      Serial.printf("Height check: %.1f cm (target: %.1f cm)\n", currentHeight, config.standingHeightCm);

      if (currentHeight > 0 && abs(currentHeight - config.standingHeightCm) <= config.heightTolerance) {
        reachedStanding = true;
        Serial.println("Desk successfully reached standing position");
        break;
      }

      server.handleClient();  // Keep web server responsive
    }

    if (!reachedStanding) {
      // Desk did NOT reach standing position after 15 seconds - user likely stopped it
      Serial.println("WARNING: Desk did not reach standing position within 15 seconds - user may have stopped it");
      dailyStats.ignoredWarningCount++;
      weeklyStats.ignoredWarningCount++;
      monthlyStats.ignoredWarningCount++;
      yearlyStats.ignoredWarningCount++;

      // Reset warning flag and sitting timer so timeout can trigger again after a full sit timeout period
      // User is still sitting, so we need to allow another alert cycle
      session.warningIssued = false;
      session.sittingStartTime = millis();  // Reset sitting timer to start fresh timeout period
      Serial.println("Warning flag and sitting timer reset - fresh timeout period started");
    }

    Serial.printf("Daily stats - Alerted: %d, Forced: %d, Ignored: %d\n",
                  dailyStats.alertedCount,
                  dailyStats.forcedStandingCount,
                  dailyStats.ignoredWarningCount);

    saveStats();

    // Clear forcing flag after process completes
    session.forcingStanding = false;
  }
}

void finalizeSession() {
  if (!session.atDesk) return;
  
  Serial.println("\n=== FINALIZING SESSION ===");
  
  unsigned long sessionTime = (millis() - session.sessionStartTime) / 1000;
  
  // Add accumulated session sitting/standing times
  dailyStats.totalSitting += session.totalSessionSitting;
  weeklyStats.totalSitting += session.totalSessionSitting;
  monthlyStats.totalSitting += session.totalSessionSitting;
  yearlyStats.totalSitting += session.totalSessionSitting;

  dailyStats.totalStanding += session.totalSessionStanding;
  weeklyStats.totalStanding += session.totalSessionStanding;
  monthlyStats.totalStanding += session.totalSessionStanding;
  yearlyStats.totalStanding += session.totalSessionStanding;

  // Add current continuous sitting/standing time
  if (session.isSitting && session.sittingStartTime > 0) {
    unsigned long finalSitting = (millis() - session.sittingStartTime) / 1000;
    dailyStats.totalSitting += finalSitting;
    weeklyStats.totalSitting += finalSitting;
    monthlyStats.totalSitting += finalSitting;
    yearlyStats.totalSitting += finalSitting;
  } else if (!session.isSitting && session.standingStartTime > 0) {
    unsigned long finalStanding = (millis() - session.standingStartTime) / 1000;
    dailyStats.totalStanding += finalStanding;
    weeklyStats.totalStanding += finalStanding;
    monthlyStats.totalStanding += finalStanding;
    yearlyStats.totalStanding += finalStanding;
  }
  
  // Add session time to desk time
  dailyStats.totalAtDesk += sessionTime;
  weeklyStats.totalAtDesk += sessionTime;
  monthlyStats.totalAtDesk += sessionTime;
  yearlyStats.totalAtDesk += sessionTime;
  
  Serial.printf("Session duration: %s\n", formatDuration(sessionTime).c_str());
  Serial.printf("Total sitting: %s\n", formatDuration(dailyStats.totalSitting).c_str());
  Serial.printf("Total standing: %s\n", formatDuration(dailyStats.totalStanding).c_str());
  
  saveStats();
  
  Serial.println("Session finalized and saved");
}

float measureDeskHeight() {
  // Returns the last height read from the Loctek controller via UART
  if (loctekHeight > 0) {
    return loctekHeight;
  }
  return -1.0;
}

// ==================== LOCTEK DESK FUNCTIONS ====================
void initLoctekUART() {
  deskSerial.begin(LOCTEK_BAUD, SERIAL_8N1, DESK_UART_RX, DESK_UART_TX);
  Serial.println("Loctek UART initialized (desk on UART2)");
}

void sendLoctekCommand(const uint8_t* cmd, size_t len) {
  deskSerial.write(cmd, len);
}

void sendDeskWake() {
  // Ensure PIN 20 is HIGH to keep controller screen active
  if (!deskScreenActive) {
    digitalWrite(LOCTEK_PIN20, HIGH);
    deskScreenActive = true;
    delay(100);
  }
  sendLoctekCommand(CMD_WAKE, CMD_LENGTH);
  if (config.debugMode) {
    Serial.println("[Loctek] Wake command sent");
  }
}

void sendDeskUp() {
  sendDeskWake();
  delay(50);
  sendLoctekCommand(CMD_UP, CMD_LENGTH);
  Serial.println("[Loctek] Up command sent");
}

void sendDeskDown() {
  sendDeskWake();
  delay(50);
  sendLoctekCommand(CMD_DOWN, CMD_LENGTH);
  Serial.println("[Loctek] Down command sent");
}

void sendDeskStop() {
  // Sending wake without a movement command effectively stops movement
  sendLoctekCommand(CMD_WAKE, CMD_LENGTH);
  Serial.println("[Loctek] Stop command sent");
}

void sendDeskPreset(int preset) {
  sendDeskWake();
  delay(50);
  switch (preset) {
    case 1: sendLoctekCommand(CMD_PRESET_1, CMD_LENGTH); break;
    case 2: sendLoctekCommand(CMD_PRESET_2, CMD_LENGTH); break;
    case 3: sendLoctekCommand(CMD_PRESET_3, CMD_LENGTH); break;  // Stand
    case 4: sendLoctekCommand(CMD_PRESET_4, CMD_LENGTH); break;  // Sit
    default: return;
  }
  Serial.printf("[Loctek] Preset %d command sent\n", preset);
}

int decodeSevenSeg(uint8_t seg) {
  // Strip the decimal point bit (bit 7) before matching
  uint8_t stripped = seg & 0x7F;
  for (int i = 0; i <= 9; i++) {
    if (SEVEN_SEG_DIGITS[i] == stripped) return i;
  }
  return -1;  // Unknown segment
}

bool hasDecimalPoint(uint8_t seg) {
  return (seg & 0x80) != 0;
}

float decodeLoctekHeight(uint8_t d1, uint8_t d2, uint8_t d3) {
  int digit1 = decodeSevenSeg(d1);
  int digit2 = decodeSevenSeg(d2);
  int digit3 = decodeSevenSeg(d3);

  Serial.printf("[Loctek DECODE] raw: 0x%02X 0x%02X 0x%02X -> digits: %d %d %d | DP: %d %d %d\n",
                d1, d2, d3, digit1, digit2, digit3,
                hasDecimalPoint(d1), hasDecimalPoint(d2), hasDecimalPoint(d3));

  if (digit1 < 0 || digit2 < 0 || digit3 < 0) {
    return -1.0;  // Could not decode
  }

  // Find which digit has the decimal point
  if (hasDecimalPoint(d1)) {
    // D.DD format (unlikely but handle it)
    return (float)digit1 + (float)(digit2 * 10 + digit3) / 100.0;
  } else if (hasDecimalPoint(d2)) {
    // DD.D format (e.g., 28.8 inches)
    return (float)(digit1 * 10 + digit2) + (float)digit3 / 10.0;
  } else {
    // No decimal point: 3-digit number (e.g., 115 cm)
    return (float)(digit1 * 100 + digit2 * 10 + digit3);
  }
}

void readDeskHeight() {
  while (deskSerial.available()) {
    uint8_t b = deskSerial.read();
    uartBytesReceived++;
    lastUartByteTime = millis();

    if (b == LOCTEK_FRAME_START) {
      deskRxIndex = 0;
      deskRxBuffer[deskRxIndex++] = b;
    } else if (deskRxIndex > 0) {
      if (deskRxIndex < sizeof(deskRxBuffer)) {
        deskRxBuffer[deskRxIndex++] = b;
      }

      if (b == LOCTEK_FRAME_END) {
        uartFramesReceived++;

        // Dump raw frame hex for debugging (first 20 frames then every 100th)
        if (uartFramesReceived <= 20 || uartFramesReceived % 100 == 0) {
          Serial.printf("[Loctek RAW] Frame #%lu len=%d: ", uartFramesReceived, deskRxIndex);
          for (int i = 0; i < deskRxIndex; i++) {
            Serial.printf("%02X ", deskRxBuffer[i]);
          }
          Serial.println();
        }

        // Complete frame received - parse it
        // Height data frames have length byte of 0x07 and 3 seven-seg display bytes
        if (deskRxIndex >= 7 && deskRxBuffer[1] == 0x07 && deskRxBuffer[2] == 0x12) {
          float height = decodeLoctekHeight(deskRxBuffer[3], deskRxBuffer[4], deskRxBuffer[5]);
          if (height > 0) {
            loctekHeight = height;
            uartValidHeights++;
            if (config.debugMode) {
              Serial.printf("[Loctek] Height: %.1f cm\n", loctekHeight);
            }
          }
        }
        deskRxIndex = 0;  // Reset for next frame
      }
    }
  }
}

void syncTime() {
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
  
  Serial.print("Syncing time");
  int attempts = 0;
  while (time(nullptr) < 8 * 3600 * 2 && attempts < 20) {
    Serial.print(".");
    delay(500);
    attempts++;
  }
  
  if (attempts >= 20) {
    Serial.println("\nFailed to sync time!");
    return;
  }
  
  time_t now = time(nullptr) + CST_OFFSET;
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  
  Serial.println();
  Serial.printf("Current CST time: %04d-%02d-%02d %02d:%02d:%02d\n",
                timeinfo.tm_year + 1900,
                timeinfo.tm_mon + 1,
                timeinfo.tm_mday,
                timeinfo.tm_hour,
                timeinfo.tm_min,
                timeinfo.tm_sec);
}

bool isOperatingHours() {
  time_t now = time(nullptr) + CST_OFFSET;
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  
  int currentHour = timeinfo.tm_hour;
  
  bool inHours = (currentHour >= config.operationStartHour && 
                  currentHour < config.operationEndHour);
  
  if (config.debugMode) {
    Serial.printf("Current hour: %d, Forced standing hours: %d-%d, In hours: %s\n",
                  currentHour,
                  config.operationStartHour,
                  config.operationEndHour,
                  inHours ? "YES" : "NO");
  }
  
  return inHours;
}

void checkDayRollover() {
  time_t now = time(nullptr) + CST_OFFSET;
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  
  int currentDate = timeinfo.tm_mday;
  
  if (dailyStats.date != 0 && currentDate != dailyStats.date) {
    Serial.println("=== NEW DAY - Saving and resetting daily stats ===");
    // Accumulate today's steps into weekly total before resetting
    weeklyTotalSteps += steps;
    Serial.printf("Added %d steps to weekly total (now %d)\n", steps, weeklyTotalSteps);
    steps = 0;
    Serial.println("Steps reset to 0 for new day");
    saveStats();
    resetDailyStats();
    dailyStats.date = currentDate;
    saveStats();
  }
  
  if (dailyStats.date == 0) {
    dailyStats.date = currentDate;
    saveStats();
  }
}

void checkWeekRollover() {
  time_t now = time(nullptr) + CST_OFFSET;
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);

  // Calculate Sunday-based week number
  // tm_wday: 0=Sunday, 1=Monday, ..., 6=Saturday
  // tm_yday: 0-365 (day of year, 0=Jan 1)
  // First, figure out what day of week January 1st was
  int jan1DayOfWeek = (7 + timeinfo.tm_wday - (timeinfo.tm_yday % 7)) % 7;
  // Calculate which Sunday-based week we're in
  // Week 1 includes Jan 1 through the first Saturday
  // Week 2 starts on the first Sunday
  int currentWeek = (timeinfo.tm_yday + jan1DayOfWeek) / 7 + 1;
  int currentYear = timeinfo.tm_year + 1900;
  int currentMonth = timeinfo.tm_mon + 1;
  
  if (weeklyStats.week != 0 && 
      (currentWeek != weeklyStats.week || currentYear != weeklyStats.year)) {
    Serial.println("=== NEW WEEK - Saving and resetting weekly stats ===");
    saveStats();
    resetWeeklyStats();
    weeklyStats.week = currentWeek;
    weeklyStats.year = currentYear;
    weeklyStats.month = currentMonth;
    saveStats();
  }
  
  if (weeklyStats.week == 0) {
    weeklyStats.week = currentWeek;
    weeklyStats.year = currentYear;
    weeklyStats.month = currentMonth;
    saveStats();
  }
}

void checkMonthRollover() {
  time_t now = time(nullptr) + CST_OFFSET;
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);

  int currentMonth = timeinfo.tm_mon + 1;
  int currentYear = timeinfo.tm_year + 1900;

  if (monthlyStats.month != 0 &&
      (currentMonth != monthlyStats.month || currentYear != monthlyStats.year)) {
    Serial.println("=== NEW MONTH - Saving and resetting monthly stats ===");
    saveStats();
    resetMonthlyStats();
    monthlyStats.month = currentMonth;
    monthlyStats.year = currentYear;
    saveStats();
  }

  if (monthlyStats.month == 0) {
    monthlyStats.month = currentMonth;
    monthlyStats.year = currentYear;
    saveStats();
  }
}

void checkYearRollover() {
  time_t now = time(nullptr) + CST_OFFSET;
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);

  int currentYear = timeinfo.tm_year + 1900;

  if (yearlyStats.year != 0 && currentYear != yearlyStats.year) {
    Serial.println("=== NEW YEAR - Saving and resetting yearly stats ===");
    saveStats();
    resetYearlyStats();
    yearlyStats.year = currentYear;
    saveStats();
  }

  if (yearlyStats.year == 0) {
    yearlyStats.year = currentYear;
    saveStats();
  }
}

String formatDuration(unsigned long seconds) {
  int hours = seconds / 3600;
  int minutes = (seconds % 3600) / 60;
  int secs = seconds % 60;
  
  char buffer[32];
  sprintf(buffer, "%dh %dm %ds", hours, minutes, secs);
  return String(buffer);
}

String formatDurationShort(unsigned long seconds) {
  int hours = seconds / 3600;
  int minutes = (seconds % 3600) / 60;
  
  char buffer[16];
  if (hours > 0) {
    sprintf(buffer, "%dh%dm", hours, minutes);
  } else {
    sprintf(buffer, "%dm", minutes);
  }
  return String(buffer);
}

String formatPercentage(unsigned long sit, unsigned long stand) {
  unsigned long total = sit + stand;
  if (total == 0) {
    return "0%/0%";
  }
  
  int sitPercent = (sit * 100) / total;
  int standPercent = 100 - sitPercent;
  
  char buffer[16];
  sprintf(buffer, "%d%%/%d%%", sitPercent, standPercent);
  return String(buffer);
}

// ==================== UTILITY FUNCTIONS ====================
void beep(int duration) {
  tone(PASSIVE_BEEPER, 2000);
  delay(duration);
  noTone(PASSIVE_BEEPER);
}

// ==================== CONFIGURATION (EEPROM) ====================
void loadConfig() {
  config.sitTimeout = preferences.getInt("sitTimeout", 60);
  config.operationStartHour = preferences.getInt("startHour", 8);
  config.operationEndHour = preferences.getInt("endHour", 18);
  config.sittingHeightCm = preferences.getFloat("sitHeight", 75.0);
  config.standingHeightCm = preferences.getFloat("standHeight", 115.0);
  config.heightTolerance = preferences.getFloat("tolerance", 5.0);
  config.debugMode = preferences.getBool("debugMode", false);
  config.beepDuration = preferences.getInt("beepDur", 2000);
  config.pirTimeoutMinutes = preferences.getInt("pirTimeout", 2);
  config.complianceWaitSeconds = preferences.getInt("complianceWait", 30);
  config.beeperEnabled = preferences.getBool("beeperEnabled", true);
  config.positionChangeDelaySeconds = preferences.getInt("posChangeDelay", 5);

  Serial.println("Configuration loaded from EEPROM");
}

void saveConfig() {
  preferences.putInt("sitTimeout", config.sitTimeout);
  preferences.putInt("startHour", config.operationStartHour);
  preferences.putInt("endHour", config.operationEndHour);
  preferences.putFloat("sitHeight", config.sittingHeightCm);
  preferences.putFloat("standHeight", config.standingHeightCm);
  preferences.putFloat("tolerance", config.heightTolerance);
  preferences.putBool("debugMode", config.debugMode);
  preferences.putInt("beepDur", config.beepDuration);
  preferences.putInt("pirTimeout", config.pirTimeoutMinutes);
  preferences.putInt("complianceWait", config.complianceWaitSeconds);
  preferences.putBool("beeperEnabled", config.beeperEnabled);
  preferences.putInt("posChangeDelay", config.positionChangeDelaySeconds);

  Serial.println("Configuration saved to EEPROM");
}

void printCurrentConfig() {
  Serial.println("\n=== Current Configuration ===");
  Serial.printf("Sit Timeout: %d minutes\n", config.sitTimeout);
  Serial.printf("PIR Timeout: %d minutes\n", config.pirTimeoutMinutes);
  Serial.printf("Forced Standing Hours: %02d:00 - %02d:00\n",
                config.operationStartHour, config.operationEndHour);
  Serial.printf("Compliance Wait: %d seconds\n", config.complianceWaitSeconds);
  Serial.printf("Position Change Delay: %d seconds\n", config.positionChangeDelaySeconds);
  Serial.printf("Beeper: %s\n", config.beeperEnabled ? "ON" : "OFF");
  Serial.printf("Sitting Height: %.1f cm\n", config.sittingHeightCm);
  Serial.printf("Standing Height: %.1f cm\n", config.standingHeightCm);
  Serial.printf("Height Tolerance: %.1f cm\n", config.heightTolerance);
  Serial.printf("Debug Mode: %s\n", config.debugMode ? "ON" : "OFF");
  Serial.println("============================\n");
}

void loadStats() {
  // Daily
  dailyStats.totalAtDesk = preferences.getULong("dayDesk", 0);
  dailyStats.totalSitting = preferences.getULong("daySit", 0);
  dailyStats.totalStanding = preferences.getULong("dayStand", 0);
  dailyStats.alertedCount = preferences.getInt("dayAlerted", 0);
  dailyStats.forcedStandingCount = preferences.getInt("dayForced", 0);
  dailyStats.ignoredWarningCount = preferences.getInt("dayIgnored", 0);
  dailyStats.date = preferences.getInt("dayDate", 0);
  
  // Weekly
  weeklyStats.totalAtDesk = preferences.getULong("weekDesk", 0);
  weeklyStats.totalSitting = preferences.getULong("weekSit", 0);
  weeklyStats.totalStanding = preferences.getULong("weekStand", 0);
  weeklyStats.alertedCount = preferences.getInt("weekAlerted", 0);
  weeklyStats.forcedStandingCount = preferences.getInt("weekForced", 0);
  weeklyStats.ignoredWarningCount = preferences.getInt("weekIgnored", 0);
  weeklyStats.week = preferences.getInt("weekNum", 0);
  weeklyStats.year = preferences.getInt("weekYear", 0);
  weeklyStats.month = preferences.getInt("weekMonth", 0);
  
  // Monthly
  monthlyStats.totalAtDesk = preferences.getULong("monthDesk", 0);
  monthlyStats.totalSitting = preferences.getULong("monthSit", 0);
  monthlyStats.totalStanding = preferences.getULong("monthStand", 0);
  monthlyStats.alertedCount = preferences.getInt("monthAlerted", 0);
  monthlyStats.forcedStandingCount = preferences.getInt("monthForced", 0);
  monthlyStats.ignoredWarningCount = preferences.getInt("monthIgnored", 0);
  monthlyStats.month = preferences.getInt("monthNum", 0);
  monthlyStats.year = preferences.getInt("monthYear", 0);
  
  // Yearly
  yearlyStats.totalAtDesk = preferences.getULong("yearDesk", 0);
  yearlyStats.totalSitting = preferences.getULong("yearSit", 0);
  yearlyStats.totalStanding = preferences.getULong("yearStand", 0);
  yearlyStats.alertedCount = preferences.getInt("yearAlerted", 0);
  yearlyStats.forcedStandingCount = preferences.getInt("yearForced", 0);
  yearlyStats.ignoredWarningCount = preferences.getInt("yearIgnored", 0);
  yearlyStats.year = preferences.getInt("yearNum", 0);

  // Amazfit sensor data
  steps = preferences.getInt("azSteps", 0);
  calories = preferences.getInt("azCalories", 0);
  amazfitDistance = preferences.getFloat("azDistance", 0.0);
  heartRate = preferences.getInt("azHeartRate", 0);
  weight = preferences.getFloat("azWeight", 0.0);
  amazfitConnected = preferences.getBool("azConnected", false);
  battery = preferences.getInt("azBattery", 0);
  sleeping = preferences.getBool("azSleeping", false);
  sleepState = preferences.getString("azSleepState", "");
  sleepDuration = preferences.getInt("azSleepDur", 0);
  trigger_3 = preferences.getString("azTrigger3", "");
  trigger_6 = preferences.getString("azTrigger6", "");
  trigger_7 = preferences.getString("azTrigger7", "");
  as_sensor = preferences.getString("azAsSensor", "");
  weeklyTotalSteps = preferences.getInt("weekSteps", 0);

  Serial.println("Statistics loaded from EEPROM");
}

void saveStats() {
  // Daily
  preferences.putULong("dayDesk", dailyStats.totalAtDesk);
  preferences.putULong("daySit", dailyStats.totalSitting);
  preferences.putULong("dayStand", dailyStats.totalStanding);
  preferences.putInt("dayAlerted", dailyStats.alertedCount);
  preferences.putInt("dayForced", dailyStats.forcedStandingCount);
  preferences.putInt("dayIgnored", dailyStats.ignoredWarningCount);
  preferences.putInt("dayDate", dailyStats.date);
  
  // Weekly
  preferences.putULong("weekDesk", weeklyStats.totalAtDesk);
  preferences.putULong("weekSit", weeklyStats.totalSitting);
  preferences.putULong("weekStand", weeklyStats.totalStanding);
  preferences.putInt("weekAlerted", weeklyStats.alertedCount);
  preferences.putInt("weekForced", weeklyStats.forcedStandingCount);
  preferences.putInt("weekIgnored", weeklyStats.ignoredWarningCount);
  preferences.putInt("weekNum", weeklyStats.week);
  preferences.putInt("weekYear", weeklyStats.year);
  preferences.putInt("weekMonth", weeklyStats.month);
  
  // Monthly
  preferences.putULong("monthDesk", monthlyStats.totalAtDesk);
  preferences.putULong("monthSit", monthlyStats.totalSitting);
  preferences.putULong("monthStand", monthlyStats.totalStanding);
  preferences.putInt("monthAlerted", monthlyStats.alertedCount);
  preferences.putInt("monthForced", monthlyStats.forcedStandingCount);
  preferences.putInt("monthIgnored", monthlyStats.ignoredWarningCount);
  preferences.putInt("monthNum", monthlyStats.month);
  preferences.putInt("monthYear", monthlyStats.year);
  
  // Yearly
  preferences.putULong("yearDesk", yearlyStats.totalAtDesk);
  preferences.putULong("yearSit", yearlyStats.totalSitting);
  preferences.putULong("yearStand", yearlyStats.totalStanding);
  preferences.putInt("yearAlerted", yearlyStats.alertedCount);
  preferences.putInt("yearForced", yearlyStats.forcedStandingCount);
  preferences.putInt("yearIgnored", yearlyStats.ignoredWarningCount);
  preferences.putInt("yearNum", yearlyStats.year);

  // Amazfit sensor data
  preferences.putInt("azSteps", steps);
  preferences.putInt("azCalories", calories);
  preferences.putFloat("azDistance", amazfitDistance);
  preferences.putInt("azHeartRate", heartRate);
  preferences.putFloat("azWeight", weight);
  preferences.putBool("azConnected", amazfitConnected);
  preferences.putInt("azBattery", battery);
  preferences.putBool("azSleeping", sleeping);
  preferences.putString("azSleepState", sleepState);
  preferences.putInt("azSleepDur", sleepDuration);
  preferences.putString("azTrigger3", trigger_3);
  preferences.putString("azTrigger6", trigger_6);
  preferences.putString("azTrigger7", trigger_7);
  preferences.putString("azAsSensor", as_sensor);
  preferences.putInt("weekSteps", weeklyTotalSteps);

  Serial.println("Statistics saved to EEPROM");
}

void resetDailyStats() {
  dailyStats.totalAtDesk = 0;
  dailyStats.totalSitting = 0;
  dailyStats.totalStanding = 0;
  dailyStats.alertedCount = 0;
  dailyStats.forcedStandingCount = 0;
  dailyStats.ignoredWarningCount = 0;
  Serial.println("Daily stats reset");
}

void resetWeeklyStats() {
  weeklyStats.totalAtDesk = 0;
  weeklyStats.totalSitting = 0;
  weeklyStats.totalStanding = 0;
  weeklyStats.alertedCount = 0;
  weeklyStats.forcedStandingCount = 0;
  weeklyStats.ignoredWarningCount = 0;
  weeklyTotalSteps = 0;
  Serial.println("Weekly stats reset");
}

void resetMonthlyStats() {
  monthlyStats.totalAtDesk = 0;
  monthlyStats.totalSitting = 0;
  monthlyStats.totalStanding = 0;
  monthlyStats.alertedCount = 0;
  monthlyStats.forcedStandingCount = 0;
  monthlyStats.ignoredWarningCount = 0;
  Serial.println("Monthly stats reset");
}

void resetYearlyStats() {
  yearlyStats.totalAtDesk = 0;
  yearlyStats.totalSitting = 0;
  yearlyStats.totalStanding = 0;
  yearlyStats.alertedCount = 0;
  yearlyStats.forcedStandingCount = 0;
  yearlyStats.ignoredWarningCount = 0;
  Serial.println("Yearly stats reset");
}

void resetAllStats() {
  resetDailyStats();
  resetWeeklyStats();
  resetMonthlyStats();
  resetYearlyStats();
  saveStats();
  Serial.println("All stats reset and saved to EEPROM");
}

void printCurrentStats() {
  Serial.println("\n=== Current Statistics ===");
  Serial.printf("Daily - At Desk: %s, Sitting: %s, Standing: %s, Alerted: %d, Forced: %d, Ignored: %d\n",
                formatDuration(dailyStats.totalAtDesk).c_str(),
                formatDuration(dailyStats.totalSitting).c_str(),
                formatDuration(dailyStats.totalStanding).c_str(),
                dailyStats.alertedCount,
                dailyStats.forcedStandingCount,
                dailyStats.ignoredWarningCount);
  Serial.printf("Weekly - At Desk: %s, Sitting: %s, Standing: %s, Alerted: %d, Forced: %d, Ignored: %d\n",
                formatDuration(weeklyStats.totalAtDesk).c_str(),
                formatDuration(weeklyStats.totalSitting).c_str(),
                formatDuration(weeklyStats.totalStanding).c_str(),
                weeklyStats.alertedCount,
                weeklyStats.forcedStandingCount,
                weeklyStats.ignoredWarningCount);
  Serial.printf("Monthly - At Desk: %s, Sitting: %s, Standing: %s, Alerted: %d, Forced: %d, Ignored: %d\n",
                formatDuration(monthlyStats.totalAtDesk).c_str(),
                formatDuration(monthlyStats.totalSitting).c_str(),
                formatDuration(monthlyStats.totalStanding).c_str(),
                monthlyStats.alertedCount,
                monthlyStats.forcedStandingCount,
                monthlyStats.ignoredWarningCount);
  Serial.printf("Yearly - At Desk: %s, Sitting: %s, Standing: %s, Alerted: %d, Forced: %d, Ignored: %d\n",
                formatDuration(yearlyStats.totalAtDesk).c_str(),
                formatDuration(yearlyStats.totalSitting).c_str(),
                formatDuration(yearlyStats.totalStanding).c_str(),
                yearlyStats.alertedCount,
                yearlyStats.forcedStandingCount,
                yearlyStats.ignoredWarningCount);
  Serial.println("========================\n");
}

// ==================== AMAZFIT SENSOR HANDLER ====================
void handleSensorUpdate() {
  String uri = server.uri();
  Serial.printf("[HTTP POST] %s - Body length: %d\n", uri.c_str(), server.arg("plain").length());

  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "No body");
    return;
  }

  String body = server.arg("plain");
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, body);

  if (error) {
    Serial.printf("  JSON parse error: %s\n", error.c_str());
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }

  String sensorName = "";
  int amazfitPos = uri.indexOf("amazfit_");
  if (amazfitPos != -1) {
    sensorName = uri.substring(amazfitPos + 8);
  }

  String stateValue = doc["state"].as<String>();

  if (sensorName == "steps") {
    steps = stateValue.toInt();
    Serial.printf("  Steps: %d\n", steps);
  }
  else if (sensorName == "calories") {
    calories = stateValue.toInt();
    Serial.printf("  Calories: %d\n", calories);
  }
  else if (sensorName == "distance") {
    amazfitDistance = stateValue.toFloat();
    Serial.printf("  Distance: %.2f km\n", amazfitDistance);
  }
  else if (sensorName == "heartRate") {
    heartRate = stateValue.toInt();
    Serial.printf("  Heart Rate: %d bpm\n", heartRate);
  }
  else if (sensorName == "weight") {
    weight = stateValue.toFloat();
    Serial.printf("  Weight: %.1f kg\n", weight);
  }
  else if (sensorName == "connected") {
    amazfitConnected = (stateValue == "true" || stateValue == "1" || stateValue == "on");
    Serial.printf("  Connected: %s\n", amazfitConnected ? "Yes" : "No");
  }
  else if (sensorName == "trigger_3") {
    trigger_3 = stateValue;
    Serial.printf("  Trigger 3: %s\n", stateValue.c_str());
  }
  else if (sensorName == "trigger_6") {
    trigger_6 = stateValue;
    Serial.printf("  Trigger 6: %s\n", stateValue.c_str());
  }
  else if (sensorName == "trigger_7") {
    trigger_7 = stateValue;
    Serial.printf("  Trigger 7: %s\n", stateValue.c_str());
  }
  else if (sensorName == "as") {
    as_sensor = stateValue;
    Serial.printf("  AS: %s\n", stateValue.c_str());
  }
  else if (sensorName == "battery") {
    battery = stateValue.toInt();
    Serial.printf("  Battery: %d%%\n", battery);
  }
  else if (sensorName == "sleeping") {
    sleeping = (stateValue == "true" || stateValue == "1" || stateValue == "on");
    Serial.printf("  Sleeping: %s\n", sleeping ? "Yes" : "No");
  }
  else if (sensorName == "sleep") {
    sleepState = stateValue;
    Serial.printf("  Sleep State: %s\n", stateValue.c_str());
  }
  else if (sensorName == "sleepDuration") {
    sleepDuration = stateValue.toInt();
    Serial.printf("  Sleep Duration: %d min\n", sleepDuration);
  }
  else {
    Serial.printf("  Unknown sensor: %s = %s\n", sensorName.c_str(), stateValue.c_str());
  }

  server.send(200, "application/json", "{\"state\":\"" + stateValue + "\"}");
}

void handleNotFound() {
  Serial.printf("[HTTP %s] Unknown path: %s\n",
    server.method() == HTTP_GET ? "GET" :
    server.method() == HTTP_POST ? "POST" : "OTHER",
    server.uri().c_str());
  server.send(404, "text/plain", "Not Found");
}

// ==================== WEB SERVER ====================
void setupWebServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/config", HTTP_GET, handleConfig);
  server.on("/config", HTTP_POST, handleConfigSave);
  server.on("/calibrate", HTTP_GET, handleCalibrate);
  server.on("/calibrate-sit", HTTP_POST, handleCalibrateSit);
  server.on("/calibrate-stand", HTTP_POST, handleCalibrateStand);
  server.on("/stats", HTTP_GET, handleStats);
  server.on("/reset-all-stats", HTTP_POST, handleResetStats);
  server.on("/reset-daily", HTTP_POST, handleResetDaily);
  server.on("/reset-weekly", HTTP_POST, handleResetWeekly);
  server.on("/reset-monthly", HTTP_POST, handleResetMonthly);
  server.on("/reset-yearly", HTTP_POST, handleResetYearly);
  server.on("/update-daily-stats", HTTP_POST, handleUpdateDailyStats);
  server.on("/test-beep", HTTP_POST, handleTestBeep);
  server.on("/test-relay", HTTP_POST, handleTestRelay);
  server.on("/test-standing", HTTP_POST, handleTestStanding);
  server.on("/desk-up", HTTP_POST, handleDeskUp);
  server.on("/desk-down", HTTP_POST, handleDeskDown);
  server.on("/desk-stop", HTTP_POST, handleDeskStop);
  server.on("/desk-preset", HTTP_POST, handleDeskPreset);
  server.on("/reset-amazfit", HTTP_POST, handleResetAmazfit);

  // Amazfit sensor endpoints
  server.on("/api/states/sensor.amazfit_steps", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_calories", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_distance", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_heartRate", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_weight", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_connected", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_battery", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_sleeping", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_sleep", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_sleepDuration", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_trigger_3", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_trigger_6", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_trigger_7", HTTP_POST, handleSensorUpdate);
  server.on("/api/states/sensor.amazfit_as", HTTP_POST, handleSensorUpdate);

  server.onNotFound(handleNotFound);
}

void handleRoot() {
  // Calculate display totals (accumulated + current session)
  unsigned long displayDailyDesk = dailyStats.totalAtDesk;
  unsigned long displayDailySitting = dailyStats.totalSitting;
  unsigned long displayDailyStanding = dailyStats.totalStanding;
  
  unsigned long displayWeeklyDesk = weeklyStats.totalAtDesk;
  unsigned long displayWeeklySitting = weeklyStats.totalSitting;
  unsigned long displayWeeklyStanding = weeklyStats.totalStanding;
  
  unsigned long displayMonthlyDesk = monthlyStats.totalAtDesk;
  unsigned long displayMonthlySitting = monthlyStats.totalSitting;
  unsigned long displayMonthlyStanding = monthlyStats.totalStanding;
  
  unsigned long displayYearlyDesk = yearlyStats.totalAtDesk;
  unsigned long displayYearlySitting = yearlyStats.totalSitting;
  unsigned long displayYearlyStanding = yearlyStats.totalStanding;
  
  // Add current session if at desk
  if (session.atDesk) {
    unsigned long currentSessionTime = (millis() - session.sessionStartTime) / 1000;
    displayDailyDesk += currentSessionTime;
    displayWeeklyDesk += currentSessionTime;
    displayMonthlyDesk += currentSessionTime;
    displayYearlyDesk += currentSessionTime;
    
    if (session.isSitting && session.sittingStartTime > 0) {
      unsigned long currentSitting = (millis() - session.sittingStartTime) / 1000;
      displayDailySitting += currentSitting;
      displayWeeklySitting += currentSitting;
      displayMonthlySitting += currentSitting;
      displayYearlySitting += currentSitting;
    } else if (!session.isSitting && session.standingStartTime > 0) {
      unsigned long currentStanding = (millis() - session.standingStartTime) / 1000;
      displayDailyStanding += currentStanding;
      displayWeeklyStanding += currentStanding;
      displayMonthlyStanding += currentStanding;
      displayYearlyStanding += currentStanding;
    }
  }
  
  String html = "<html><body>";
  html += "<h1>Smart Standing Desk Monitor</h1>";
  
  html += "<h2>Current Status</h2>";
  html += "<p>Status: " + String(session.atDesk ? "AT DESK" : "AWAY") + "</p>";
  
  if (session.atDesk) {
    html += "<p>Position: " + String(session.isSitting ? "SITTING" : "STANDING") + "</p>";
    
    if (session.isSitting && session.sittingStartTime > 0) {
      unsigned long sittingTime = (millis() - session.sittingStartTime) / 1000;
      html += "<p>Current Sitting Time: " + formatDuration(sittingTime) + "</p>";
    } else if (!session.isSitting && session.standingStartTime > 0) {
      unsigned long standingTime = (millis() - session.standingStartTime) / 1000;
      html += "<p>Current Standing Time: " + formatDuration(standingTime) + "</p>";
    }
  }
  
  html += "<hr>";
  
  // Get current time for display
  time_t now = time(nullptr) + CST_OFFSET;
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  
  html += "<h2>Today (" + String(timeinfo.tm_mon + 1) + "/" + String(timeinfo.tm_mday) + "/" + String(timeinfo.tm_year + 1900) + ")</h2>";
  html += "<p>At Desk: " + formatDuration(displayDailyDesk) + " | ";
  html += "Sitting: " + formatDuration(displayDailySitting) + " | ";
  html += "Standing: " + formatDuration(displayDailyStanding) + "</p>";
  html += "<p>Ratio (Sit/Stand): " + formatPercentage(displayDailySitting, displayDailyStanding) + "</p>";
  html += "<p>Alerted: " + String(dailyStats.alertedCount) + " | ";
  html += "Forced: " + String(dailyStats.forcedStandingCount) + " | ";
  html += "Ignored: " + String(dailyStats.ignoredWarningCount) + "</p>";
  
  html += "<h2>This Week (Week " + String(weeklyStats.week) + " of " + String(weeklyStats.year) + ")</h2>";
  html += "<p>At Desk: " + formatDuration(displayWeeklyDesk) + " | ";
  html += "Sitting: " + formatDuration(displayWeeklySitting) + " | ";
  html += "Standing: " + formatDuration(displayWeeklyStanding) + "</p>";
  html += "<p>Ratio (Sit/Stand): " + formatPercentage(displayWeeklySitting, displayWeeklyStanding) + "</p>";
  html += "<p>Alerted: " + String(weeklyStats.alertedCount) + " | ";
  html += "Forced: " + String(weeklyStats.forcedStandingCount) + " | ";
  html += "Ignored: " + String(weeklyStats.ignoredWarningCount) + "</p>";
  
  html += "<h2>This Month (" + String(monthlyStats.month) + "/" + String(monthlyStats.year) + ")</h2>";
  html += "<p>At Desk: " + formatDuration(displayMonthlyDesk) + " | ";
  html += "Sitting: " + formatDuration(displayMonthlySitting) + " | ";
  html += "Standing: " + formatDuration(displayMonthlyStanding) + "</p>";
  html += "<p>Ratio (Sit/Stand): " + formatPercentage(displayMonthlySitting, displayMonthlyStanding) + "</p>";
  html += "<p>Alerted: " + String(monthlyStats.alertedCount) + " | ";
  html += "Forced: " + String(monthlyStats.forcedStandingCount) + " | ";
  html += "Ignored: " + String(monthlyStats.ignoredWarningCount) + "</p>";
  
  html += "<h2>This Year (" + String(yearlyStats.year) + ")</h2>";
  html += "<p>At Desk: " + formatDuration(displayYearlyDesk) + " | ";
  html += "Sitting: " + formatDuration(displayYearlySitting) + " | ";
  html += "Standing: " + formatDuration(displayYearlyStanding) + "</p>";
  html += "<p>Ratio (Sit/Stand): " + formatPercentage(displayYearlySitting, displayYearlyStanding) + "</p>";
  html += "<p>Alerted: " + String(yearlyStats.alertedCount) + " | ";
  html += "Forced: " + String(yearlyStats.forcedStandingCount) + " | ";
  html += "Ignored: " + String(yearlyStats.ignoredWarningCount) + "</p>";
  
  html += "<hr>";
  html += "<h2>Amazfit Fitness Data</h2>";
  html += "<p>Steps: " + String(steps) + "</p>";
  html += "<p>Calories: " + String(calories) + "</p>";
  html += "<p>Distance: " + String(amazfitDistance, 2) + " km</p>";
  html += "<p>Heart Rate: " + String(heartRate) + " bpm</p>";
  html += "<p>Weight: " + String(weight, 1) + " kg</p>";
  html += "<p>Connected: " + String(amazfitConnected ? "Yes" : "No") + "</p>";
  html += "<p>Battery: " + String(battery) + "%</p>";
  html += "<p>Sleeping: " + String(sleeping ? "Yes" : "No") + "</p>";
  html += "<p>Sleep State: " + sleepState + "</p>";
  html += "<p>Sleep Duration: " + String(sleepDuration) + " min</p>";
  html += "<p>Trigger 3: " + trigger_3 + "</p>";
  html += "<p>Trigger 6: " + trigger_6 + "</p>";
  html += "<p>Trigger 7: " + trigger_7 + "</p>";
  html += "<p>AS Sensor: " + as_sensor + "</p>";

  html += "<hr>";
  html += "<p><a href='/config'>Configuration</a> | ";
  html += "<a href='/calibrate'>Calibrate</a> | ";
  html += "<a href='/'>Refresh</a></p>";
  html += "<p><em>Stats auto-save every hour | Stats include current session</em></p>";
  
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleConfig() {
  String html = "<html><body>";
  html += "<h1>Configuration</h1>";
  
  html += "<form method='POST' action='/config'>";
  html += "<p>Sitting Timeout (min): <input type='number' name='sitTimeout' value='" + String(config.sitTimeout) + "'></p>";
  html += "<p>PIR Timeout (min): <input type='number' name='pirTimeout' value='" + String(config.pirTimeoutMinutes) + "'></p>";
  html += "<p><em>Session ends after this many minutes of no motion</em></p>";
  html += "<p>Compliance Wait Time (sec): <input type='number' name='complianceWait' value='" + String(config.complianceWaitSeconds) + "'></p>";
  html += "<p><em>Time to wait for user to stand before forcing desk to raise</em></p>";
  html += "<p>Position Change Delay (sec): <input type='number' name='posChangeDelay' value='" + String(config.positionChangeDelaySeconds) + "'></p>";
  html += "<p><em>Time desk must stay at new height before confirming position change</em></p>";
  html += "<p>Forced Standing Start Hour (0-23): <input type='number' name='startHour' value='" + String(config.operationStartHour) + "'></p>";
  html += "<p>Forced Standing End Hour (0-23): <input type='number' name='endHour' value='" + String(config.operationEndHour) + "'></p>";
  html += "<p><em>Alerts and forced standing only happen during these hours. Stats are tracked 24/7.</em></p>";
  html += "<p>Height Tolerance (cm): <input type='number' step='0.1' name='tolerance' value='" + String(config.heightTolerance) + "'></p>";
  html += "<p>Beep Duration (ms): <input type='number' name='beepDur' value='" + String(config.beepDuration) + "'></p>";
  html += "<p>Beeper Enabled: <select name='beeperEnabled'><option value='1' " + String(config.beeperEnabled ? "selected" : "") + ">ON</option><option value='0' " + String(!config.beeperEnabled ? "selected" : "") + ">OFF</option></select></p>";
  html += "<p>Debug Mode: <select name='debugMode'><option value='0'>OFF</option><option value='1' " + String(config.debugMode ? "selected" : "") + ">ON</option></select></p>";
  html += "<p><input type='submit' value='Save Configuration'></p>";
  html += "</form>";

  html += "<hr><h2>Manual Stats Adjustment (Today)</h2>";
  html += "<form method='POST' action='/update-daily-stats'>";
  html += "<p>Total Sitting Time (minutes): <input type='number' name='dailySit' value='" + String(dailyStats.totalSitting / 60) + "'></p>";
  html += "<p>Total Standing Time (minutes): <input type='number' name='dailyStand' value='" + String(dailyStats.totalStanding / 60) + "'></p>";
  html += "<p>Total At Desk Time (minutes): <input type='number' name='dailyDesk' value='" + String(dailyStats.totalAtDesk / 60) + "'></p>";
  html += "<p><em>Enter values in minutes. Current session time not included.</em></p>";
  html += "<p><input type='submit' value='Update Daily Stats'></p>";
  html += "</form>";

  html += "<hr><h2>Desk Control</h2>";
  html += "<form method='POST' action='/desk-up' style='display:inline'><input type='submit' value='Desk Up'></form> ";
  html += "<form method='POST' action='/desk-down' style='display:inline'><input type='submit' value='Desk Down'></form> ";
  html += "<form method='POST' action='/desk-stop' style='display:inline'><input type='submit' value='Stop'></form>";
  html += "<br><br>";
  html += "<form method='POST' action='/desk-preset' style='display:inline'><input type='hidden' name='preset' value='1'><input type='submit' value='Preset 1'></form> ";
  html += "<form method='POST' action='/desk-preset' style='display:inline'><input type='hidden' name='preset' value='2'><input type='submit' value='Preset 2'></form> ";
  html += "<form method='POST' action='/desk-preset' style='display:inline'><input type='hidden' name='preset' value='3'><input type='submit' value='Preset 3 (Stand)'></form> ";
  html += "<form method='POST' action='/desk-preset' style='display:inline'><input type='hidden' name='preset' value='4'><input type='submit' value='Preset 4 (Sit)'></form>";
  html += "<p>Current Height: " + String(loctekHeight, 1) + " cm</p>";

  // UART Debug Info
  unsigned long secsSinceByte = lastUartByteTime > 0 ? (millis() - lastUartByteTime) / 1000 : 0;
  html += "<hr><h2>UART Debug (GPIO" + String(DESK_UART_RX) + ")</h2>";
  html += "<p>Bytes Received: <strong>" + String(uartBytesReceived) + "</strong></p>";
  html += "<p>Frames Received: <strong>" + String(uartFramesReceived) + "</strong></p>";
  html += "<p>Valid Heights Decoded: <strong>" + String(uartValidHeights) + "</strong></p>";
  html += "<p>Last Byte Received: <strong>" + (lastUartByteTime > 0 ? String(secsSinceByte) + "s ago" : "Never") + "</strong></p>";
  if (uartBytesReceived == 0) {
    html += "<p style='color:red;font-weight:bold'>WARNING: No data received on RX pin - check wiring to RJ45 pin 5</p>";
  }

  html += "<hr><h2>Test Functions</h2>";
  html += "<form method='POST' action='/test-beep'><input type='submit' value='Test Beep'></form>";
  html += "<form method='POST' action='/test-standing'><input type='submit' value='Test Force Standing'></form>";
  
  html += "<hr><h2>Reset Statistics</h2>";
  html += "<form method='POST' action='/reset-daily'><input type='submit' value='Reset Daily Stats' onclick=\"return confirm('Reset today\\'s stats?')\"></form>";
  html += "<form method='POST' action='/reset-weekly'><input type='submit' value='Reset Weekly Stats' onclick=\"return confirm('Reset this week\\'s stats?')\"></form>";
  html += "<form method='POST' action='/reset-monthly'><input type='submit' value='Reset Monthly Stats' onclick=\"return confirm('Reset this month\\'s stats?')\"></form>";
  html += "<form method='POST' action='/reset-yearly'><input type='submit' value='Reset Yearly Stats' onclick=\"return confirm('Reset this year\\'s stats?')\"></form>";
  html += "<form method='POST' action='/reset-amazfit'><input type='submit' value='Reset Amazfit Data' onclick=\"return confirm('Reset all Amazfit sensor data?')\"></form>";
  html += "<form method='POST' action='/reset-all-stats'><input type='submit' value='Reset ALL Stats' style='background:red;color:white' onclick=\"return confirm('Reset ALL statistics? This cannot be undone!')\"></form>";
  
  html += "<hr><p><a href='/'>Home</a></p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleConfigSave() {
  if (server.hasArg("sitTimeout")) config.sitTimeout = server.arg("sitTimeout").toInt();
  if (server.hasArg("pirTimeout")) config.pirTimeoutMinutes = server.arg("pirTimeout").toInt();
  if (server.hasArg("complianceWait")) config.complianceWaitSeconds = server.arg("complianceWait").toInt();
  if (server.hasArg("posChangeDelay")) config.positionChangeDelaySeconds = server.arg("posChangeDelay").toInt();
  if (server.hasArg("startHour")) config.operationStartHour = server.arg("startHour").toInt();
  if (server.hasArg("endHour")) config.operationEndHour = server.arg("endHour").toInt();
  if (server.hasArg("tolerance")) config.heightTolerance = server.arg("tolerance").toFloat();
  if (server.hasArg("beepDur")) config.beepDuration = server.arg("beepDur").toInt();
  if (server.hasArg("beeperEnabled")) config.beeperEnabled = server.arg("beeperEnabled").toInt() == 1;
  if (server.hasArg("debugMode")) config.debugMode = server.arg("debugMode").toInt() == 1;

  saveConfig();
  printCurrentConfig();

  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleCalibrate() {
  float currentHeight = measureDeskHeight();
  
  String html = "<html><body>";
  html += "<h1>Calibrate Desk Heights</h1>";
  html += "<h2>Current Height: " + String(currentHeight, 1) + " cm</h2>";
  html += "<p>Current sitting height: " + String(config.sittingHeightCm, 1) + " cm</p>";
  html += "<p>Current standing height: " + String(config.standingHeightCm, 1) + " cm</p>";
  
  html += "<h3>Instructions:</h3>";
  html += "<ol><li>Adjust desk to sitting position</li>";
  html += "<li>Click 'Set Sitting Height'</li>";
  html += "<li>Adjust desk to standing position</li>";
  html += "<li>Click 'Set Standing Height'</li></ol>";
  
  html += "<form method='POST' action='/calibrate-sit'><input type='submit' value='Set Current Height as SITTING'></form>";
  html += "<form method='POST' action='/calibrate-stand'><input type='submit' value='Set Current Height as STANDING'></form>";
  
  html += "<hr>";
  html += "<p><a href='/'>Home</a></p>";
  html += "<p><a href='/calibrate'>Refresh</a></p>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleCalibrateSit() {
  float height = measureDeskHeight();
  if (height > 0) {
    config.sittingHeightCm = height;
    saveConfig();
    Serial.printf("Sitting height set to: %.1f cm (saved to EEPROM)\n", height);
    beep(200);
  }
  
  server.sendHeader("Location", "/calibrate");
  server.send(303);
}

void handleCalibrateStand() {
  float height = measureDeskHeight();
  if (height > 0) {
    config.standingHeightCm = height;
    saveConfig();
    Serial.printf("Standing height set to: %.1f cm (saved to EEPROM)\n", height);
    beep(200);
    delay(100);
    beep(200);
  }
  
  server.sendHeader("Location", "/calibrate");
  server.send(303);
}

void handleStats() {
  String json = "{";
  json += "\"atDesk\":" + String(session.atDesk ? "true" : "false") + ",";
  json += "\"isSitting\":" + String(session.isSitting ? "true" : "false") + ",";
  json += "\"currentSitting\":" + String(session.currentSittingTime) + ",";
  json += "\"currentStanding\":" + String(session.currentStandingTime) + ",";
  json += "\"dailyDesk\":" + String(dailyStats.totalAtDesk) + ",";
  json += "\"dailySitting\":" + String(dailyStats.totalSitting) + ",";
  json += "\"dailyStanding\":" + String(dailyStats.totalStanding) + ",";
  json += "\"dailyAlerted\":" + String(dailyStats.alertedCount) + ",";
  json += "\"dailyForced\":" + String(dailyStats.forcedStandingCount) + ",";
  json += "\"dailyIgnored\":" + String(dailyStats.ignoredWarningCount) + ",";
  json += "\"weeklyDesk\":" + String(weeklyStats.totalAtDesk) + ",";
  json += "\"weeklySitting\":" + String(weeklyStats.totalSitting) + ",";
  json += "\"weeklyStanding\":" + String(weeklyStats.totalStanding) + ",";
  json += "\"weeklyAlerted\":" + String(weeklyStats.alertedCount) + ",";
  json += "\"weeklyForced\":" + String(weeklyStats.forcedStandingCount) + ",";
  json += "\"weeklyIgnored\":" + String(weeklyStats.ignoredWarningCount) + ",";
  json += "\"monthlyDesk\":" + String(monthlyStats.totalAtDesk) + ",";
  json += "\"monthlySitting\":" + String(monthlyStats.totalSitting) + ",";
  json += "\"monthlyStanding\":" + String(monthlyStats.totalStanding) + ",";
  json += "\"monthlyAlerted\":" + String(monthlyStats.alertedCount) + ",";
  json += "\"monthlyForced\":" + String(monthlyStats.forcedStandingCount) + ",";
  json += "\"monthlyIgnored\":" + String(monthlyStats.ignoredWarningCount) + ",";
  json += "\"yearlyDesk\":" + String(yearlyStats.totalAtDesk) + ",";
  json += "\"yearlySitting\":" + String(yearlyStats.totalSitting) + ",";
  json += "\"yearlyStanding\":" + String(yearlyStats.totalStanding) + ",";
  json += "\"yearlyAlerted\":" + String(yearlyStats.alertedCount) + ",";
  json += "\"yearlyForced\":" + String(yearlyStats.forcedStandingCount) + ",";
  json += "\"yearlyIgnored\":" + String(yearlyStats.ignoredWarningCount) + ",";
  json += "\"steps\":" + String(steps) + ",";
  json += "\"calories\":" + String(calories) + ",";
  json += "\"distance\":" + String(amazfitDistance, 2) + ",";
  json += "\"heartRate\":" + String(heartRate) + ",";
  json += "\"weight\":" + String(weight, 1) + ",";
  json += "\"amazfitConnected\":" + String(amazfitConnected ? "true" : "false") + ",";
  json += "\"battery\":" + String(battery) + ",";
  json += "\"sleeping\":" + String(sleeping ? "true" : "false") + ",";
  json += "\"sleepState\":\"" + sleepState + "\",";
  json += "\"sleepDuration\":" + String(sleepDuration) + ",";
  json += "\"trigger_3\":\"" + trigger_3 + "\",";
  json += "\"trigger_6\":\"" + trigger_6 + "\",";
  json += "\"trigger_7\":\"" + trigger_7 + "\",";
  json += "\"as_sensor\":\"" + as_sensor + "\",";
  json += "\"deskHeight\":" + String(loctekHeight, 1) + ",";
  json += "\"uartBytesReceived\":" + String(uartBytesReceived) + ",";
  json += "\"uartFramesReceived\":" + String(uartFramesReceived) + ",";
  json += "\"uartValidHeights\":" + String(uartValidHeights) + ",";
  json += "\"uartLastByteSecsAgo\":" + String(lastUartByteTime > 0 ? (millis() - lastUartByteTime) / 1000 : 0);
  json += "}";

  server.send(200, "application/json", json);
}

void handleResetStats() {
  resetAllStats();
  
  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleResetDaily() {
  resetDailyStats();
  saveStats();
  
  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleResetWeekly() {
  resetWeeklyStats();
  saveStats();
  
  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleResetMonthly() {
  resetMonthlyStats();
  saveStats();
  
  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleResetYearly() {
  resetYearlyStats();
  saveStats();

  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleResetAmazfit() {
  steps = 0;
  calories = 0;
  amazfitDistance = 0.0;
  heartRate = 0;
  weight = 0.0;
  amazfitConnected = false;
  battery = 0;
  sleeping = false;
  sleepState = "";
  sleepDuration = 0;
  trigger_3 = "";
  trigger_6 = "";
  trigger_7 = "";
  as_sensor = "";
  saveStats();
  Serial.println("Amazfit sensor data reset");

  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleUpdateDailyStats() {
  if (server.hasArg("dailySit")) {
    unsigned long minutes = server.arg("dailySit").toInt();
    dailyStats.totalSitting = minutes * 60;
  }
  if (server.hasArg("dailyStand")) {
    unsigned long minutes = server.arg("dailyStand").toInt();
    dailyStats.totalStanding = minutes * 60;
  }
  if (server.hasArg("dailyDesk")) {
    unsigned long minutes = server.arg("dailyDesk").toInt();
    dailyStats.totalAtDesk = minutes * 60;
  }

  saveStats();
  Serial.println("Daily stats manually updated");
  Serial.printf("Sitting: %s, Standing: %s, At Desk: %s\n",
                formatDuration(dailyStats.totalSitting).c_str(),
                formatDuration(dailyStats.totalStanding).c_str(),
                formatDuration(dailyStats.totalAtDesk).c_str());

  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleTestBeep() {
  beep(500);
  delay(200);
  beep(500);
  
  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleTestRelay() {
  // Legacy endpoint - now sends desk up command
  Serial.println("Test: Sending desk up command");
  sendDeskUp();

  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleTestStanding() {
  Serial.println("=== MANUAL STANDING TEST ===");

  for (int i = 0; i < 3; i++) {
    beep(500);
    delay(200);
  }

  Serial.println("Sending Loctek stand preset...");
  sendDeskPreset(3);
  Serial.println("Standing test complete");

  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleDeskUp() {
  sendDeskUp();
  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleDeskDown() {
  sendDeskDown();
  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleDeskStop() {
  sendDeskStop();
  server.sendHeader("Location", "/config");
  server.send(303);
}

void handleDeskPreset() {
  int preset = 1;
  if (server.hasArg("preset")) {
    preset = server.arg("preset").toInt();
  }
  sendDeskPreset(preset);
  server.sendHeader("Location", "/config");
  server.send(303);
}