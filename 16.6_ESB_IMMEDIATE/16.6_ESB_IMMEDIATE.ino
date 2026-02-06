#include <Arduino.h>
#include "Adafruit_TinyUSB.h"
#include "nrf_to_nrf.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include "InternalFileSystem.h"
#include "flash/flash_nrf5x.h"
#include "variant.h"
#include "nrf_power.h"
#include "nrf_gpio.h"
#include "nrf_uart.h"
#include "nrf_twi.h"
#include "nrf_spi.h"
#include <stdarg.h>
#include <math.h>
#include <algorithm>
#include "nrf_radio.h"
#include <bluefruit.h>
#include <avr/pgmspace.h>

// --- WebUSB Configuration ---
Adafruit_USBD_WebUSB webusb;
WEBUSB_URL_DEF(landingPage, 1, "diningwork.space/"); // Change to your desired URL

#define TFT_CS 11
#define TFT_DC 12
#define TFT_RST 20
#define TFT_MOSI 19
#define TFT_SCK 18
#define TFT_BL 10

#define THEME_BG 0xFFFF
#define THEME_FG 0x0000
#define THEME_ACCENT 0x0000
#define THEME_INACTIVE 0xC618
#define THEME_INDICATOR_ON 0x0000
#define THEME_INDICATOR_OFF 0xFFFF

SPIClass customSPI(NRF_SPIM3, -1, TFT_SCK, TFT_MOSI);
Adafruit_ST7789 tft = Adafruit_ST7789(&customSPI, TFT_CS, TFT_DC, TFT_RST);

const int16_t SCREEN_WIDTH = 240;
const int16_t SCREEN_HEIGHT = 280;

Adafruit_USBD_MIDI usb_midi;
nrf_to_nrf nrf;

BLEDfu bledfu;
BLEDis bledis;
BLEMidi blemidi;
BLEBas blebas;
bool bleRunning = false;
uint16_t connHandle = BLE_CONN_HANDLE_INVALID;
char blePeerName[32] = {0};
unsigned long lastBleBatteryUpdateTime = 0;

const int S0_PIN = 13, S1_PIN = 14, S2_PIN = 15, S3_PIN = 16, SIG_PIN = 17;
const int NUM_SAMPLES_PER_CHANNEL = 1;

#define NUM_BANKS 12
#define NUM_ANALOG_POTS 8
#define NUM_DIGITAL_MIDI_BUTTONS 4
#define TOTAL_ASSIGNABLE_INPUTS (NUM_ANALOG_POTS + NUM_DIGITAL_MIDI_BUTTONS)
#define POT_LABEL_LENGTH 4

struct BankSettings {
    uint8_t midiChannel;
    uint8_t buttonMode;
    uint8_t assignableCCs[TOTAL_ASSIGNABLE_INPUTS];
    char potLabels[NUM_ANALOG_POTS][POT_LABEL_LENGTH + 1];
    char buttonLabels[NUM_DIGITAL_MIDI_BUTTONS][POT_LABEL_LENGTH + 1];
    int16_t potMinRaw[NUM_ANALOG_POTS];
    int16_t potMaxRaw[NUM_ANALOG_POTS];
};

struct GlobalSettings {
    uint8_t esbChannel;
    uint8_t activeBank;
    uint8_t brightness;
    bool autoCalEnabled;
    int16_t autoCalMaxValue;
};

#define GLOBAL_SETTINGS_ADDR 0x7F000
#define BANK_SETTINGS_ADDR 0x7F020

GlobalSettings globalSettings;
BankSettings currentBankSettings, previewBankSettings;
const BankSettings* activeMidiSettings;

const uint8_t ANALOG_CHANNEL_COUNT = 4, BUTTON_COUNT = 4;
const uint32_t SETUP_TIMEOUT_MS = 7000;
const uint16_t BANK_SWITCH_HOLD_MS = 300;
const int16_t SNAP_THRESHOLD_RAW = 30;
const uint32_t NRF_IDLE_TIMEOUT_MS = 1000;
const unsigned long AUTO_SYSTEM_OFF_TIMEOUT_MS = 60UL * 20UL * 1UL * 1000UL;
const unsigned long BATTERY_UPDATE_INTERVAL_MS = 5000;
const unsigned long POT_ACTIVITY_TIMEOUT_MS = 2000;

const unsigned long POST_UNPLUG_ALIVE_MS = 60UL * 5UL * 1000UL;
const unsigned long DIM_INACTIVITY_TIMEOUT_MS = 60UL * 2UL * 1000UL;
const unsigned long FADE_DURATION_US = 1000000;
const uint8_t DIM_BRIGHTNESS = 12; // Stored as 8-bit for comparison
const uint16_t BRIGHTNESS_MIN_14BIT = 600;
const uint16_t BRIGHTNESS_MAX_14BIT = 16383;
const uint16_t DIM_BRIGHTNESS_14BIT = 700; // Approx (12/255.0 * 16383)
static uint16_t brightnessBeforeFade_14bit = 0;
static uint16_t currentBrightness_14bit = 0;


enum ScreenState { BRIGHT, DIM, FADING_UP, FADING_DOWN, FADING_IN, FADING_REVERT };

uint16_t tempBrightnessStep;
uint8_t brightnessOnEnterEdit;
ScreenState screenState = FADING_IN;

bool wasPluggedInLong = false;
bool isAwaitingPostUnplugShutdown = false;
unsigned long fadeStartTime = 0;
uint8_t brightnessBeforeDim = 0;

const uint8_t TOTAL_MAIN_MENU_ITEMS = 7;
const uint8_t TOTAL_DEVICE_SETUP_ITEMS = 6;
const uint8_t TOTAL_BLUETOOTH_MENU_ITEMS = 2;

const uint8_t buttonPins[BUTTON_COUNT] = {6, 7, 8, 5};
const uint8_t NAV_UP_BUTTON = 6, NAV_DOWN_BUTTON = 7, NAV_CONFIRM_BUTTON = 8, NAV_CANCEL_BUTTON = 5;

#define EXT_VCC_PIN NRF_GPIO_PIN_MAP(0, 13)
#define LED_BUILTIN_NRF_PIN NRF_GPIO_PIN_MAP(0, 15)

#define VBAT_ADC_RES 16383.0f
#define VBAT_CORRECTION_FACTOR 5.12f

const int ADC_MAX_VALUE_14BIT = 15000, MIDI_MAX_VALUE = 127;
const float ADC_HYSTERESIS_PERCENT = 1.2;

bool lastButtonState[BUTTON_COUNT] = {0};
uint8_t lastSentCCValue[TOTAL_ASSIGNABLE_INPUTS] = {255};
bool buttonToggles[BUTTON_COUNT] = {0};
float prevSentADCEquivalent[NUM_ANALOG_POTS];
bool analogInputReady = false, tftConnected = false, lastUSBState = false;
bool radioActive = true;
unsigned long lastRadioActivityTime = 0, lastMidiActivityTime = 0, lastPotActivityTime = 0;
int lastDisplayedCCValueForPot[4] = {-1, -1, -1, -1};
int16_t lastRawValueForDisplay[NUM_ANALOG_POTS] = {0};
bool forceFullRedraw = true;

enum AppMode { NORMAL_OPERATION, SETUP_MENU, ESB_SWEEP };
AppMode currentAppMode = NORMAL_OPERATION;

enum ShutdownReason { NORMAL, LOW_BATTERY_SHUTDOWN, WAKE_LOW_BATTERY };

enum SetupSubMode {
    MAIN_MENU, CC_ASSIGN_LIST, CC_EDIT_VALUE, LABEL_ASSIGN_LIST, LABEL_EDIT_CHAR,
    MIDI_CHANNEL_EDIT, BUTTON_MODE_EDIT, DEVICE_SETUP_MENU, BLUETOOTH_MENU,
    ESB_CHANNEL_EDIT, CALIBRATION_MAIN, EASY_CAL_POT_SELECT, EASY_CAL_SET_MIN,
    EASY_CAL_SET_MAX, CALIBRATION_MANUAL_POT_SELECT, CALIBRATION_MANUAL_MIN_MAX_SELECT,
    CALIBRATION_MANUAL_VALUE_EDIT, FACTORY_RESET_CONFIRM_1, FACTORY_RESET_CONFIRM_2,
    FACTORY_CAL_CONFIRM, BRIGHTNESS_EDIT, HARDWARE_TEST, POWER_OFF_CONFIRM,
    ESB_SWEEP_PROGRESS, ESB_CONNECT_MENU
};
SetupSubMode currentSetupSubMode = MAIN_MENU;

const uint8_t ESB_SWEEP_SAMPLES = 64;
const uint8_t ESB_SWEEP_ACK_THRESHOLD = 10;
const uint8_t NUM_ESB_OPTIONS = 21;
uint8_t foundEsbChannels[NUM_ESB_OPTIONS];
uint8_t foundEsbChannelCount = 0;

bool setupMode = false;
uint8_t selectedSetting = 0, menuCursor = 0, subMenuCursor = 0, lastSelectedMainMenuIndex = 0;
unsigned long setupStartTime = 0;
bool isBankSwitching = false;
unsigned long d0_pressTime = 0;
uint8_t tempBankSelection = 0;
bool d0_wasPressed = false;
bool isD13HeldForCCPreview = false;
static bool lastD13State = false;
static float averagedBatteryVoltage = 0.0f;
static unsigned long lastBatterySampleTime = 0;
const int BATTERY_SAMPLE_COUNT = 10;
static unsigned long lastDisplayUpdateTime = 0;
uint8_t ccAssignmentMode = 0, currentEditingInputIndex = 0;
int currentCCEditValue = 0;
uint8_t currentCCDigitIndex = 0;
uint8_t labelAssignmentMode = 0, currentEditingLabelIndex = 0;
char currentLabelEditBuffer[POT_LABEL_LENGTH + 1];
uint8_t currentLabelCharIndex = 0;
const char VALID_LABEL_CHARS[] = " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789#-_[]|/<>";
const int VALID_CHARS_LEN = sizeof(VALID_LABEL_CHARS) - 1;
int tempMidiChannel = 1, tempEsbChannelIndex = 0;
const uint8_t esbOptions[] = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100};
static unsigned long lastCCButtonPressTime = 0;
static int ccChangeCounter = 0;
static bool ccButtonHeld = false;

uint8_t selectedPotForCal = 0;
bool isEditingMinCal = true;
int16_t tempCalValue = 0, capturedRawValue = 0;
uint8_t calValDigitIndex = 0;
const int DEFAULT_FONT_HEIGHT_PIXELS = 8;
static int16_t lastRawPotValuesForTouch[NUM_ANALOG_POTS] = {0};
static int lastTouchedPotForEditIndex = -1;
static uint8_t lastTouchedPotForEditValue = 0;
static unsigned long lastPotTouchTime = 0;
static const int POT_TOUCH_THRESHOLD_RAW = 50;
static int16_t lastDrawnHwTestValues[NUM_ANALOG_POTS];

unsigned long lastBlinkToggleTime = 0;
bool blinkState = false;
const unsigned long BLINK_INTERVAL_MS = 300;

const int CHAR_SLOT_WIDTH = 40;
const int CHAR_SLOT_HEIGHT = 50;
const int CHAR_SLOT_Y_START = 100;
const int CURSOR_Y_OFFSET = 5;

const int indicator_radius = 10;
const int indicator_y_offset = -18;

const char PGM_MENU_ITEM_1[] PROGMEM = "1. Bluetooth";
const char PGM_MENU_ITEM_2[] PROGMEM = "2. CC Assign";
const char PGM_MENU_ITEM_3[] PROGMEM = "3. Label Assign";
const char PGM_MENU_ITEM_4[] PROGMEM = "4. Button Mode";
const char PGM_MENU_ITEM_5[] PROGMEM = "5. MIDI Channel";
const char PGM_MENU_ITEM_6[] PROGMEM = "6. Device Setup";
const char PGM_MENU_ITEM_7[] PROGMEM = "7. Power Off";
const char* const menuItems[] PROGMEM = { PGM_MENU_ITEM_1, PGM_MENU_ITEM_2, PGM_MENU_ITEM_3, PGM_MENU_ITEM_4, PGM_MENU_ITEM_5, PGM_MENU_ITEM_6, PGM_MENU_ITEM_7 };

const char PGM_DEV_MENU_ITEM_1[] PROGMEM = "1. ESB Ch (Global)";
const char PGM_DEV_MENU_ITEM_2[] PROGMEM = "2. Calibrate";
const char PGM_DEV_MENU_ITEM_3[] PROGMEM = "3. Brightness";
const char PGM_DEV_MENU_ITEM_4[] PROGMEM = "4. Factory Reset";
const char PGM_DEV_MENU_ITEM_5[] PROGMEM = "5. Hardware Test";
const char PGM_DEV_MENU_ITEM_6[] PROGMEM = "6. Scan for RX";
const char* const deviceSetupMenuItems[] PROGMEM = { PGM_DEV_MENU_ITEM_1, PGM_DEV_MENU_ITEM_2, PGM_DEV_MENU_ITEM_3, PGM_DEV_MENU_ITEM_4, PGM_DEV_MENU_ITEM_5, PGM_DEV_MENU_ITEM_6 };

bool forceFooterRedraw = true;

// --- Forward Declarations ---
void readVccReference();
void loadSettings();
void saveCurrentBankSettings();
void saveGlobalSettings();
void factoryReset();
void factoryCalibrateCurrentBank();
void loadSpecificBankSettings(uint8_t bankIndex, BankSettings& targetSettings);
void updateNormalStatusBar(bool force);
void drawTopStatusBar(bool isUSB, bool isBankSwitchingPreview, uint8_t previewBankIndex, uint8_t previewMidiChannel, bool isD13HeldForCCPreview);
void drawTopStatusBar_BankSwitchPreview(uint8_t previewBankIndex, uint8_t previewMidiChannel, bool isD13HeldForCCPreview);
void drawMainScreenPotentiometerLayout(const BankSettings& settingsToDisplay, bool showCCs);
void updateMainScreenLabelsOnly(const BankSettings& settingsToDisplay, bool showCCs);
void redrawAllPotValues(const BankSettings& settingsToDisplay);
void updatePotVisual(uint8_t potIndex, uint8_t newCCValue, const BankSettings& settingsToDisplay);
void partialRedrawMainMenu(uint8_t oldCursor, uint8_t newCursor);
void partialRedrawListMenu(uint8_t oldCursor, uint8_t newCursor, uint8_t menuType);
void drawSubMenu(const __FlashStringHelper* title, const char* const options[], uint8_t cursorPosition, uint8_t numOptions);
void drawSubMenu(const char* title, const char* const options[], uint8_t cursorPosition, uint8_t numOptions);
void partialRedrawSubMenu(const char* const options[], uint8_t oldCursor, uint8_t newCursor, uint8_t numOptions);
void drawButtonModeMenu(uint8_t cursorPosition);
void partialRedrawButtonModeMenu(uint8_t oldCursor, uint8_t newCursor);
void drawMidiChannelEditMenu(uint8_t channel, bool initialDraw);
void updateMidiChannelValueOnly(uint8_t channel, bool forceRedraw = false);
void drawESBChannelEditMenu(uint8_t channelValue, bool initialDraw);
void updateESBChannelValueOnly(uint8_t channelValue, bool forceRedraw = false);
void drawCCAssignmentEditMenu(uint8_t inputIndex, int ccValue, bool initialDraw);
void updateCCAssignmentValueOnly(int ccValue, bool forceRedraw = false);
void drawLabelEditMenu(uint8_t inputIndex, char* labelBuffer, uint8_t charIndex);
void updateEditedLabelChar(uint8_t index);
void updateLabelCursor(uint8_t oldIndex, uint8_t newIndex);
void updateLabelEditInstructions(uint8_t charIndex);
void drawCalibrationMenu();
void partialRedrawCalibrationMenu(uint8_t oldCursor, uint8_t newCursor);
void drawEasyCalPotSelectMenu();
void drawEasyCalPromptMenu(uint8_t potIndex, bool isSettingMin);
void updateEasyCalPromptValueOnly();
void drawManualCalPotSelectMenu();
void drawManualCalValueSelectMenu(uint8_t potIndex);
void partialRedrawManualCalValueSelectMenu(uint8_t potIndex, uint8_t oldCursor, uint8_t newCursor);
void drawManualCalValueEditMenu(uint8_t potIndex, bool isEditingMin, int16_t value, uint8_t digitIndex, bool initialDraw);
void updateManualCalDigit(uint8_t digitIndex, uint8_t digitValue);
void updateManualCalCursor(uint8_t oldIndex, uint8_t newIndex);
void updateManualCalFooterInstructions(uint8_t digitIndex);
void drawFactoryResetConfirmMenu();
void drawBrightnessMenu(uint8_t initialBrightness);
void updateBrightnessBar(uint8_t brightnessValue, bool forceRedraw);
void drawADCNotFound();
void drawHardwareTestLayout();
void updateHardwareTestValues();
void drawSystemOffConfirmMenu();
void drawButtonPressIndicator(uint8_t buttonIndex, uint16_t color);
void handleSetupInput();
void sendCC(uint8_t cc, uint8_t value);
void sendPotCC(uint8_t potIndex, uint8_t value);
void resetRadio();
void idleRadioCheck();
void enterSystemOff(ShutdownReason reason = NORMAL);
void selectChannel(int channel);
int readMedianADC(int analogPin);
void blinkAssignedInputLabelAndIndicator();
void clearBlinkingArea(uint8_t inputIndex, bool isCCAssignmentMode);
void drawFooterInstructions(const __FlashStringHelper* text);
void drawActionFooter(const __FlashStringHelper* left, const __FlashStringHelper* center, const __FlashStringHelper* right);
void drawDeviceSetupMenu();
void drawBluetoothMenu();
void partialRedrawBluetoothMenu(uint8_t oldCursor, uint8_t newCursor);
void handleScreenFade();
void line_state_callback(bool connected);
void startBLE();
void stopBLE();
void drawBleConnectionStatus();
void performEsbSweep();
void drawEsbSweepScreen(uint8_t channel, uint8_t sample, uint8_t acks, bool initial);
void drawEsbConnectMenu();
void partialRedrawEsbConnectMenu(uint8_t oldCursor, uint8_t newCursor);

// --- WebUSB Functions ---
void sendSettingsToWeb() {
    if (!webusb.connected()) return;
    String json = "{";
    json += "\"activeBank\":" + String(globalSettings.activeBank) + ",";
    json += "\"brightness\":" + String(globalSettings.brightness) + ",";
    json += "\"midiChannel\":" + String(currentBankSettings.midiChannel) + ",";
    json += "\"buttonMode\":" + String(currentBankSettings.buttonMode) + ",";
    json += "\"esbChannel\":" + String(globalSettings.esbChannel) + ",";
    json += "\"bleOn\":" + String(bleRunning ? "true" : "false") + ",";
    json += "\"blePeerName\":\"" + String(blePeerName) + "\",";
    json += "\"labels\":[";
    for (int i = 0; i < TOTAL_ASSIGNABLE_INPUTS; i++) {
        json += "\"";
        if (i < NUM_ANALOG_POTS) { json += currentBankSettings.potLabels[i]; }
        else { json += currentBankSettings.buttonLabels[i - NUM_ANALOG_POTS]; }
        json += "\"";
        if (i < TOTAL_ASSIGNABLE_INPUTS - 1) json += ",";
    }
    json += "],";
    json += "\"ccs\":[";
    for (int i = 0; i < TOTAL_ASSIGNABLE_INPUTS; i++) {
        json += String(currentBankSettings.assignableCCs[i]);
        if (i < TOTAL_ASSIGNABLE_INPUTS - 1) json += ",";
    }
    json += "],";
    json += "\"currentPotValues\":[";
    for (int i = 0; i < NUM_ANALOG_POTS; i++) {
        json += String(lastSentCCValue[i]);
        if (i < NUM_ANALOG_POTS - 1) json += ",";
    }
    json += "]";
    json += "}\n";
    webusb.print(json);
    webusb.flush();
}

void setBacklight(uint16_t value) {
  currentBrightness_14bit = constrain(value, 0, BRIGHTNESS_MAX_14BIT);
  analogWrite(TFT_BL, currentBrightness_14bit);
}

void drawEsbSweepScreen(uint8_t channel, uint8_t sample, uint8_t acks, bool initial) {
    if (!tftConnected) return;

    // Static variables to track the last drawn state to avoid unnecessary redraws.
    static uint8_t last_acks = 255;
    static int last_progress_w = -1;

    // When a new channel scan starts (`initial` is true)
    if (initial) {
        // Reset per-channel trackers
        last_acks = 0; 
        last_progress_w = 0;

        tft.startWrite();
        tft.setFont(NULL);
        tft.setTextSize(1.8);
        tft.setTextColor(THEME_FG);
        
        // Clear area for channel number and draw the new one
        tft.fillRect(120, 78, 80, 20, THEME_BG); 
        tft.setCursor(120, 80);
        tft.print(channel);

        // Clear area for ACK text and draw initial count
        char ackStr[10];
        sprintf(ackStr, "%d / %d", acks, ESB_SWEEP_SAMPLES);
        tft.fillRect(120, 108, 100, 20, THEME_BG);
        tft.setCursor(120, 110);
        tft.print(ackStr);
        
        // Clear inside of progress bar for the new channel
        tft.fillRect(22, 162, SCREEN_WIDTH - 44, 16, THEME_BG); 
        
        tft.endWrite();
        return; // Exit, the first sample update will come in the next call.
    }
    
    // This block runs for every sample update during a scan for a single channel.
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    tft.setTextColor(THEME_FG);
    
    // 1. Update ACK Count (only if it changes)
    if (acks != last_acks) {
        char ackStr[10];
        sprintf(ackStr, "%d / %d", acks, ESB_SWEEP_SAMPLES);
        tft.fillRect(120, 108, 100, 20, THEME_BG);
        tft.setCursor(120, 110);
        tft.print(ackStr);
        last_acks = acks;
    }

    // 2. Update Progress Bar Partially
    int progress_w = map(sample, 0, ESB_SWEEP_SAMPLES, 0, SCREEN_WIDTH - 44);
    if (progress_w > last_progress_w) {
        // Only draw the new segment of the bar, don't clear the whole thing.
        tft.fillRect(22 + last_progress_w, 162, progress_w - last_progress_w, 16, THEME_ACCENT);
        last_progress_w = progress_w;
    }

    tft.endWrite();
}

void drawEsbConnectMenu() {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;
    
    if (foundEsbChannelCount == 0) {
        tft.getTextBounds("No RX Found", 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH - w) / 2, 100);
        tft.print(F("No RX Found"));
        drawFooterInstructions(F("OK"));
    } else {
        tft.getTextBounds("Select ESB Channel", 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
        tft.print(F("Select ESB Channel"));

        tft.setTextSize(1.7);
        int16_t y_start = 50;
        int16_t line_height = 22;

        for (int i = 0; i < foundEsbChannelCount; i++) {
            char buffer[20];
            sprintf(buffer, "Channel %d", foundEsbChannels[i]);
            int16_t current_y = y_start + i * line_height;
            if (i == menuCursor) {
                tft.fillRect(0, current_y - 4, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
                tft.setTextColor(THEME_BG);
            } else {
                tft.setTextColor(THEME_FG);
            }
            tft.setCursor(25, current_y);
            tft.print(buffer);
        }
        drawActionFooter(F("UP/DN"), F("SELECT"), F("BACK"));
    }
    tft.endWrite();
}

void partialRedrawEsbConnectMenu(uint8_t oldCursor, uint8_t newCursor) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.7);
    int16_t y_start = 50;
    int16_t line_height = 22;
    char buffer[20];

    int16_t old_y = y_start + oldCursor * line_height;
    tft.fillRect(0, old_y - 4, SCREEN_WIDTH, line_height - 2, THEME_BG);
    tft.setTextColor(THEME_FG);
    tft.setCursor(25, old_y);
    sprintf(buffer, "Channel %d", foundEsbChannels[oldCursor]);
    tft.print(buffer);

    int16_t new_y = y_start + newCursor * line_height;
    tft.fillRect(0, new_y - 4, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
    tft.setTextColor(THEME_BG);
    tft.setCursor(25, new_y);
    sprintf(buffer, "Channel %d", foundEsbChannels[newCursor]);
    tft.print(buffer);

    tft.endWrite();
}

void performEsbSweep() {
    static uint8_t sweepChannelIndex = 0;
    static uint8_t sampleCount = 0;
    static uint8_t ackCount = 0;
    static bool isFirstScanOfSweep = true; // ADDED: To track the very first scan

    if (!digitalRead(NAV_CANCEL_BUTTON)) {
        resetRadio(); 
        currentAppMode = SETUP_MENU;
        currentSetupSubMode = DEVICE_SETUP_MENU;
        menuCursor = 5;
        drawDeviceSetupMenu();
        sweepChannelIndex = 0; sampleCount = 0; ackCount = 0;
        isFirstScanOfSweep = true; // ADDED: Reset the flag on cancel
        return;
    }

    if (sampleCount == 0) { 
        if (isFirstScanOfSweep) {
            // This block now runs only ONCE for the entire sweep process.
            tft.startWrite();
            tft.fillScreen(THEME_BG);
            tft.setFont(NULL);
            tft.setTextSize(2.0);
            tft.setTextColor(THEME_FG);
            int16_t x1, y1; uint16_t w, h;
            tft.getTextBounds("Scanning for RX", 0, 0, &x1, &y1, &w, &h);
            tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
            tft.print(F("Scanning for RX"));
            
            tft.setTextSize(1.8);
            tft.setCursor(20, 80);
            tft.print("Channel:");

            tft.setCursor(20, 110);
            tft.print("ACKs:");
            
            // Draw the progress bar frame
            tft.drawRect(20, 160, SCREEN_WIDTH - 40, 20, THEME_FG);
            
            tft.endWrite();
            drawFooterInstructions(F("CANCEL"));
            isFirstScanOfSweep = false; // The layout is now drawn, don't do it again.
        }

        if (sweepChannelIndex == 0) { 
            if (!radioActive) {
                nrf.begin(); nrf.setPALevel(NRF_PA_MAX);
                nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_POS8DBM);
                nrf.setAutoAck(true); nrf.setRetries(0, 2); // Use faster retries for scanning
                radioActive = true;
            }
        }
        uint8_t currentChannel = esbOptions[sweepChannelIndex];
        nrf.setChannel(currentChannel);
        nrf.stopListening();
        drawEsbSweepScreen(currentChannel, 0, 0, true);
    }

    if (sampleCount < ESB_SWEEP_SAMPLES) {
        uint8_t payload[] = "PING";
        if (nrf.write(payload, sizeof(payload))) {
            ackCount++;
        }
        sampleCount++;
        drawEsbSweepScreen(esbOptions[sweepChannelIndex], sampleCount, ackCount, false); 
        delay(0); 
    }

    if (sampleCount >= ESB_SWEEP_SAMPLES) {
        if (ackCount >= ESB_SWEEP_ACK_THRESHOLD) {
            if (foundEsbChannelCount < NUM_ESB_OPTIONS) {
                foundEsbChannels[foundEsbChannelCount++] = esbOptions[sweepChannelIndex];
            }
        }
        
        sweepChannelIndex++;
        sampleCount = 0;
        ackCount = 0;

        if (sweepChannelIndex >= NUM_ESB_OPTIONS) {
            currentAppMode = SETUP_MENU;
            currentSetupSubMode = ESB_CONNECT_MENU;
            menuCursor = 0;
            drawEsbConnectMenu();
            sweepChannelIndex = 0;
            isFirstScanOfSweep = true; // ADDED: Reset the flag when the sweep is done
            resetRadio(); // This will revert retries back to (0, 5)
            setupStartTime = millis(); // FIX: Reset setup timeout timer
        }
    }
}

void handleWebUSBCommands() {
    static bool requestStartBle = false;
    static unsigned long startBleDelayTime = 0;
    const unsigned long BLE_START_DELAY_MS = 1;

    if (requestStartBle && millis() - startBleDelayTime >= BLE_START_DELAY_MS) {
        requestStartBle = false;
        startBLE();
        return;
    }

    if (webusb.available()) {
        String command = webusb.readStringUntil('\n');
        command.trim();

        if (command.equals("get_settings")) {
            sendSettingsToWeb();
        } else if (command.startsWith("set_brightness=")) {
            int brightness_8bit = command.substring(15).toInt();
            brightness_8bit = constrain(brightness_8bit, 3, 255);

            if (globalSettings.brightness != brightness_8bit) {
                brightnessBeforeFade_14bit = currentBrightness_14bit;
                globalSettings.brightness = brightness_8bit;
                screenState = FADING_UP;
                fadeStartTime = micros();
            }
        } else if (command.startsWith("set_midi_ch=")) {
            int channel = command.substring(12).toInt();
            currentBankSettings.midiChannel = constrain(channel, 1, 16);
        } else if (command.startsWith("set_cc=")) {
            int firstComma = command.indexOf(',');
            int index = command.substring(7, firstComma).toInt();
            int value = command.substring(firstComma + 1).toInt();
            if (index >= 0 && index < TOTAL_ASSIGNABLE_INPUTS) {
                currentBankSettings.assignableCCs[index] = constrain(value, 0, 127);
                saveCurrentBankSettings();
            }
        } else if (command.startsWith("set_label=")) {
            int firstComma = command.indexOf(',');
            int index = command.substring(10, firstComma).toInt();
            String label = command.substring(firstComma + 1);
            if (index >= 0 && index < TOTAL_ASSIGNABLE_INPUTS) {
                if (index < NUM_ANALOG_POTS) {
                    strncpy(currentBankSettings.potLabels[index], label.c_str(), POT_LABEL_LENGTH);
                    currentBankSettings.potLabels[index][POT_LABEL_LENGTH] = '\0';
                } else {
                    strncpy(currentBankSettings.buttonLabels[index - NUM_ANALOG_POTS], label.c_str(), POT_LABEL_LENGTH);
                    currentBankSettings.buttonLabels[index - NUM_ANALOG_POTS][POT_LABEL_LENGTH] = '\0';
                }
                saveCurrentBankSettings();
                forceFullRedraw = true;
            }
        } else if (command.startsWith("set_bank=")) {
            int bank = command.substring(9).toInt();
            if (bank >= 0 && bank < NUM_BANKS) {
                globalSettings.activeBank = bank;
                saveGlobalSettings();
                loadSettings();
                activeMidiSettings = &currentBankSettings;
                updateMainScreenLabelsOnly(currentBankSettings, false);
                forceFullRedraw = true;
                if (webusb.connected()) {
                    webusb.print("bank_changed:" + String(globalSettings.activeBank) + "\n");
                    webusb.flush();
                }
                sendSettingsToWeb();
            }
        } else if (command.startsWith("set_button_mode=")) {
            int mode = command.substring(16).toInt();
            currentBankSettings.buttonMode = constrain(mode, 0, 1);
            saveCurrentBankSettings();
        } else if (command.startsWith("set_esb_ch=")) {
            int esb_ch = command.substring(11).toInt();
            globalSettings.esbChannel = esb_ch;
            saveGlobalSettings();
            resetRadio();
        } else if (command.equals("toggle_ble")) {
            if (bleRunning) {
                stopBLE();
            } else {
                requestStartBle = true;
                startBleDelayTime = millis();
            }
        } else if (command.equals("start_cal")) {
            currentAppMode = SETUP_MENU;
            currentSetupSubMode = CALIBRATION_MAIN;
            menuCursor = 1;
            subMenuCursor = 1;
            drawCalibrationMenu();
            setupStartTime = millis();
        }
    }
}

void line_state_callback(bool connected) {}

void connect_callback(uint16_t conn_handle) {
    connHandle = conn_handle;
    BLEConnection* conn = Bluefruit.Connection(conn_handle);
    if (conn) {
        conn->getPeerName(blePeerName, sizeof(blePeerName));
        conn->requestConnectionParameter(6, 0, 500);
    }
    forceFullRedraw = true;
    sendSettingsToWeb();
}

void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
    connHandle = BLE_CONN_HANDLE_INVALID;
    memset(blePeerName, 0, sizeof(blePeerName));
    forceFullRedraw = true;
    sendSettingsToWeb();
}

void startBLE() {
    nrf.powerDown(); radioActive = false;
    Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);
    Bluefruit.configCentralBandwidth(BANDWIDTH_MAX);
    Bluefruit.begin();
    Bluefruit.Advertising.addAppearance(964);
    Bluefruit.setName("Plate 1");
    Bluefruit.setTxPower(8);
    Bluefruit.autoConnLed(true);
    bledis.setModel("Plate 1 BLE");
    bledis.setFirmwareRev("Plate A 1.0.0");
    bledis.setHardwareRev("Plate A 1.0.0");
    bledis.setSoftwareRev("1.0.2");
    bledis.setManufacturer("Dining Work");
    const char pnp_id_string[] = {0x01, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00};
    bledis.setPNPID(pnp_id_string, sizeof(pnp_id_string));
    Bluefruit.Periph.setConnectCallback(connect_callback);
    Bluefruit.Periph.setDisconnectCallback(disconnect_callback);
    bledfu.begin();
    bledis.begin();
    blebas.begin();
    blebas.write(100);
    blemidi.begin();
    Bluefruit.Advertising.addService(blemidi);
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.start(0);
    bleRunning = true;
    sendSettingsToWeb();
}

void stopBLE() {
    if (connHandle != BLE_CONN_HANDLE_INVALID) {
        Bluefruit.disconnect(connHandle);
    }
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.stop();
    bleRunning = false;
    memset(blePeerName, 0, sizeof(blePeerName));
    sendSettingsToWeb();
    NVIC_SystemReset();
}

int findCharIndexInValidSet(char c) {
    for (int i = 0; i < VALID_CHARS_LEN; i++) {
        if (VALID_LABEL_CHARS[i] == c) return i;
    }
    return 0;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void drawButtonPressIndicator(uint8_t buttonIndex, uint16_t color) {
    if (!tftConnected || buttonIndex >= 4) return;
    const int bar_area_width = SCREEN_WIDTH - 65, bar_colWidth = bar_area_width / 4;
    const int bar_y_start = 66;
    int xCenter = (buttonIndex * bar_colWidth) + (bar_colWidth / 2) + 10;
    int yCenter = bar_y_start + indicator_y_offset;
    tft.startWrite();
    tft.fillCircle(xCenter, yCenter, indicator_radius, color);
    tft.endWrite();
}

const char* getInputName(uint8_t index) {
    if (index < NUM_ANALOG_POTS) return activeMidiSettings->potLabels[index];
    else if (index < TOTAL_ASSIGNABLE_INPUTS) return activeMidiSettings->buttonLabels[index - NUM_ANALOG_POTS];
    static char name[12];
    strcpy(name, "Unknown");
    return name;
}

void saveCurrentBankSettings() {
    BankSettings tempBanks[NUM_BANKS];
    uint32_t bankArraySize = sizeof(BankSettings) * NUM_BANKS;
    flash_nrf5x_read(tempBanks, BANK_SETTINGS_ADDR, bankArraySize);
    tempBanks[globalSettings.activeBank] = currentBankSettings;
    flash_nrf5x_erase(GLOBAL_SETTINGS_ADDR);
    flash_nrf5x_write(GLOBAL_SETTINGS_ADDR, &globalSettings, sizeof(GlobalSettings));
    flash_nrf5x_write(BANK_SETTINGS_ADDR, tempBanks, bankArraySize);
    flash_nrf5x_flush();
}

void saveGlobalSettings() {
    BankSettings tempBanks[NUM_BANKS];
    uint32_t bankArraySize = sizeof(BankSettings) * NUM_BANKS;
    flash_nrf5x_read(tempBanks, BANK_SETTINGS_ADDR, bankArraySize);
    flash_nrf5x_erase(GLOBAL_SETTINGS_ADDR);
    flash_nrf5x_write(GLOBAL_SETTINGS_ADDR, &globalSettings, sizeof(GlobalSettings));
    flash_nrf5x_write(BANK_SETTINGS_ADDR, tempBanks, bankArraySize);
    flash_nrf5x_flush();
}

void factoryCalibrateCurrentBank() {
    for (int j = 0; j < NUM_ANALOG_POTS; j++) {
        currentBankSettings.potMinRaw[j] = 0;
        currentBankSettings.potMaxRaw[j] = ADC_MAX_VALUE_14BIT;
    }
    saveCurrentBankSettings();
}

void factoryReset() {
    globalSettings = {0, 0, 200, false, ADC_MAX_VALUE_14BIT};
    flash_nrf5x_erase(GLOBAL_SETTINGS_ADDR);
    flash_nrf5x_write(GLOBAL_SETTINGS_ADDR, &globalSettings, sizeof(GlobalSettings));
    uint32_t bankArrayAddr = BANK_SETTINGS_ADDR;
    for (int i = 0; i < NUM_BANKS; i++) {
        BankSettings bankToSave;
        bankToSave.midiChannel = i + 1;
        bankToSave.buttonMode = 0;
        for (int j = 0; j < NUM_ANALOG_POTS; j++) {
            bankToSave.assignableCCs[j] = 20 + j;
            sprintf(bankToSave.potLabels[j], "P%02d", j + 1);
            bankToSave.potMinRaw[j] = 0;
            bankToSave.potMaxRaw[j] = ADC_MAX_VALUE_14BIT;
        }
        for (int j = 0; j < NUM_DIGITAL_MIDI_BUTTONS; j++) {
            int button_global_index = NUM_ANALOG_POTS + j;
            bankToSave.assignableCCs[button_global_index] = 10 + j;
            sprintf(bankToSave.buttonLabels[j], "B%02d", j + 1);
        }
        flash_nrf5x_write(bankArrayAddr, &bankToSave, sizeof(BankSettings));
        bankArrayAddr += sizeof(BankSettings);
    }
    flash_nrf5x_flush();
    loadSettings();
}

void loadSettings() {
    flash_nrf5x_read(&globalSettings, GLOBAL_SETTINGS_ADDR, sizeof(GlobalSettings));
    bool global_invalid = false;
    if (globalSettings.esbChannel > 100) global_invalid = true;
    if (globalSettings.activeBank >= NUM_BANKS) global_invalid = true;
    if (globalSettings.brightness > 255) global_invalid = true;
    if (*((uint8_t*)&globalSettings.autoCalEnabled) == 0xFF) {
        globalSettings.autoCalEnabled = false;
        globalSettings.autoCalMaxValue = ADC_MAX_VALUE_14BIT;
    }
    globalSettings.brightness = constrain(globalSettings.brightness, 3, 255);
    if (global_invalid) { factoryReset(); return; }

    uint32_t addr = BANK_SETTINGS_ADDR + (globalSettings.activeBank * sizeof(BankSettings));
    flash_nrf5x_read(&currentBankSettings, addr, sizeof(BankSettings));

    bool bank_invalid = false;
    for (int i = 0; i < TOTAL_ASSIGNABLE_INPUTS; i++) { if (currentBankSettings.assignableCCs[i] > 127) bank_invalid = true; }
    for (int i = 0; i < NUM_ANALOG_POTS; i++) {
        currentBankSettings.potLabels[i][POT_LABEL_LENGTH] = '\0';
        if (currentBankSettings.potMinRaw[i] >= currentBankSettings.potMaxRaw[i]) bank_invalid = true;
        if (currentBankSettings.potMinRaw[i] < 0 || currentBankSettings.potMinRaw[i] > 16383 || currentBankSettings.potMaxRaw[i] < 0 || currentBankSettings.potMaxRaw[i] > 16383) bank_invalid = true;
    }
    for (int i = 0; i < NUM_DIGITAL_MIDI_BUTTONS; i++) { currentBankSettings.buttonLabels[i][POT_LABEL_LENGTH] = '\0'; }
    if (currentBankSettings.midiChannel < 1 || currentBankSettings.midiChannel > 16) bank_invalid = true;
    if (currentBankSettings.buttonMode > 1) bank_invalid = true;

    if (bank_invalid) {
        factoryReset();
        addr = BANK_SETTINGS_ADDR + (globalSettings.activeBank * sizeof(BankSettings));
        flash_nrf5x_read(&currentBankSettings, addr, sizeof(BankSettings));
    }
    if (globalSettings.autoCalEnabled) {
        for (int i = 0; i < NUM_ANALOG_POTS; i++) {
            currentBankSettings.potMaxRaw[i] = globalSettings.autoCalMaxValue;
        }
    }
}

void loadSpecificBankSettings(uint8_t bankIndex, BankSettings& targetSettings) {
    uint32_t addr = BANK_SETTINGS_ADDR + (bankIndex * sizeof(BankSettings));
    flash_nrf5x_read(&targetSettings, addr, sizeof(BankSettings));
    for (int i = 0; i < NUM_ANALOG_POTS; i++) targetSettings.potLabels[i][POT_LABEL_LENGTH] = '\0';
    for (int i = 0; i < NUM_DIGITAL_MIDI_BUTTONS; i++) targetSettings.buttonLabels[i][POT_LABEL_LENGTH] = '\0';
    if (globalSettings.autoCalEnabled) {
        for (int i = 0; i < NUM_ANALOG_POTS; i++) {
            targetSettings.potMaxRaw[i] = globalSettings.autoCalMaxValue;
        }
    }
}

void readVccReference() {
    selectChannel(8);
    delayMicroseconds(50);
    analogSampleTime(3);
    analogOversampling(256);
    long vccRefSum = 0;
    for (int i = 0; i < 5; i++) {
        vccRefSum += analogRead(SIG_PIN);
        delayMicroseconds(50);
    }
    globalSettings.autoCalMaxValue = vccRefSum / 5;
    if (globalSettings.autoCalMaxValue < 14000 || globalSettings.autoCalMaxValue > 16000) {
        globalSettings.autoCalMaxValue = ADC_MAX_VALUE_14BIT;
    }
    analogOversampling(64);
}

void drawTopStatusBar_BankSwitchPreview(uint8_t previewBankIndex, uint8_t previewMidiChannel, bool isD13HeldForCCPreview) {
    tft.startWrite();
    tft.fillRect(0, 0, SCREEN_WIDTH, 31, THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    tft.setTextColor(THEME_FG);
    if (isD13HeldForCCPreview) {
        char bankPreviewStr[35];
        sprintf(bankPreviewStr, "Assigned CC Bank %d", previewBankIndex + 1);
        int16_t x1, y1; uint16_t w, h;
        tft.getTextBounds(bankPreviewStr, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH / 2) - (w / 2), 12);
        tft.print(bankPreviewStr);
    } else {
        char bankLabel[] = "Select Bank ", chLabel[] = " Ch ", bankNumStr[4], chNumStr[4];
        sprintf(bankNumStr, "%d", previewBankIndex + 1);
        sprintf(chNumStr, "%d", previewMidiChannel);
        int16_t x1, y1;
        uint16_t w_bankLabel, h_bankLabel, w_bankNum, h_bankNum, w_chLabel, h_chLabel, w_chNum, h_chNum;
        tft.getTextBounds(bankLabel, 0, 0, &x1, &y1, &w_bankLabel, &h_bankLabel);
        tft.getTextBounds(bankNumStr, 0, 0, &x1, &y1, &w_bankNum, &h_bankNum);
        tft.getTextBounds(chLabel, 0, 0, &x1, &y1, &w_chLabel, &h_chLabel);
        tft.getTextBounds(chNumStr, 0, 0, &x1, &y1, &w_chNum, &h_chNum);
        uint16_t totalWidth = w_bankLabel + w_bankNum + w_chLabel + w_chNum;
        uint16_t currentX = (SCREEN_WIDTH - totalWidth) / 2;
        tft.setCursor(currentX, 12); tft.print(bankLabel); currentX += w_bankLabel;
        tft.setCursor(currentX, 12); tft.print(bankNumStr); currentX += w_bankNum;
        tft.setCursor(currentX, 12); tft.print(chLabel); currentX += w_chLabel;
        tft.setCursor(currentX, 12); tft.print(chNumStr);
    }
    tft.drawFastHLine(0, 30, SCREEN_WIDTH, THEME_FG);
    tft.endWrite();
}

void updateNormalStatusBar(bool force) {
    if (!tftConnected) return;
    static char lastBankStr[10] = "";
    static char lastMidiChStr[10] = "";
    static char lastStatusRight[16] = "";
    char bankStr[10];
    sprintf(bankStr, "B:%d", globalSettings.activeBank + 1);
    char midiChStr[10];
    sprintf(midiChStr, "ch:%d", currentBankSettings.midiChannel);
    char statusRight[16];
    bool isUSB = TinyUSBDevice.mounted();
    const float CHARGING_VOLTAGE_THRESHOLD = 4.5f;
    char transportMode[4];
    if (isUSB) {
        strcpy(transportMode, "USB");
    } else if (bleRunning) {
        strcpy(transportMode, "BLE");
    } else {
        strcpy(transportMode, "ESB");
    }
    if (isUSB || averagedBatteryVoltage > CHARGING_VOLTAGE_THRESHOLD) {
        sprintf(statusRight, "%s CHG", transportMode);
    } else {
        int batteryPercent = (int)roundf(((averagedBatteryVoltage - 3.4f) / (4.15f - 3.4f)) * 100.0f);
        batteryPercent = constrain(batteryPercent, 0, 100);
        sprintf(statusRight, "%s %d%%", transportMode, batteryPercent);
    }

    if (!force && strcmp(lastBankStr, bankStr) == 0 && strcmp(lastMidiChStr, midiChStr) == 0 && strcmp(lastStatusRight, statusRight) == 0) {
        return;
    }

    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;

    if (force) {
        tft.fillRect(0, 0, SCREEN_WIDTH, 31, THEME_BG);
        tft.drawFastHLine(0, 30, SCREEN_WIDTH, THEME_FG);
        lastBankStr[0] = '\0'; lastMidiChStr[0] = '\0'; lastStatusRight[0] = '\0';
    }

    if (strcmp(lastBankStr, bankStr) != 0) {
        tft.fillRect(15, 5, 40, 20, THEME_BG);
        tft.setCursor(15, 12);
        tft.print(bankStr);
        strcpy(lastBankStr, bankStr);
    }
    if (strcmp(lastMidiChStr, midiChStr) != 0) {
        tft.getTextBounds("ch:16", 0, 0, &x1, &y1, &w, &h);
        tft.fillRect((SCREEN_WIDTH / 2) - (w / 2) - 2, 5, w + 4, 20, THEME_BG);
        tft.getTextBounds(midiChStr, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH / 2) - (w / 2), 12);
        tft.print(midiChStr);
        strcpy(lastMidiChStr, midiChStr);
    }
    if (strcmp(lastStatusRight, statusRight) != 0) {
        tft.getTextBounds("ESB 100%", 0, 0, &x1, &y1, &w, &h);
        tft.fillRect(SCREEN_WIDTH - w - 15 - 2, 5, w + 4, 20, THEME_BG);
        tft.getTextBounds(statusRight, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor(SCREEN_WIDTH - w - 15, 12);
        tft.print(statusRight);
        strcpy(lastStatusRight, statusRight);
    }
    tft.endWrite();
}

void drawTopStatusBar(bool isUSB, bool isBankSwitchingPreview, uint8_t previewBankIndex, uint8_t previewMidiChannel, bool isD13HeldForCCPreview) {
    if (isBankSwitchingPreview) {
        drawTopStatusBar_BankSwitchPreview(previewBankIndex, previewMidiChannel, isD13HeldForCCPreview);
    } else {
        updateNormalStatusBar(forceFullRedraw);
    }
}

void drawMainScreenPotentiometerLayout(const BankSettings& settingsToDisplay, bool showCCs) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    tft.setTextColor(THEME_FG);
    if (currentAppMode == SETUP_MENU && currentSetupSubMode == CC_ASSIGN_LIST) {
        int16_t x1, y1; uint16_t w, h;
        tft.getTextBounds("CC Assignment", 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH - w) / 2, 12);
        tft.print(F("CC Assignment"));
        tft.drawFastHLine(0, 30, SCREEN_WIDTH, THEME_FG);
    } else if (currentAppMode == SETUP_MENU && currentSetupSubMode == LABEL_ASSIGN_LIST) {
        int16_t x1, y1; uint16_t w, h;
        tft.getTextBounds("Label Assignment", 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH - w) / 2, 12);
        tft.print(F("Label Assignment"));
        tft.drawFastHLine(0, 30, SCREEN_WIDTH, THEME_FG);
    }

    tft.setTextSize(1.8);
    tft.setTextColor(THEME_FG);
    const int pot_num_display_width = 54, pot_num_display_height = 30;
    const int pot_num_x_center_offset = SCREEN_WIDTH - 32;
    const int pot_num_x_start = pot_num_x_center_offset - (pot_num_display_width / 2);
    const int pot_num_y_start = 38, pot_num_spacing_y = 59;
    for (int i = 0; i < 4; i++) {
        int xCenterNumBox = pot_num_x_start + (pot_num_display_width / 2);
        tft.drawRect(pot_num_x_start, pot_num_y_start + (i * pot_num_spacing_y), pot_num_display_width, pot_num_display_height + 1, THEME_INACTIVE);
        char labelStr[POT_LABEL_LENGTH + 1];
        if (showCCs) { sprintf(labelStr, "%d", settingsToDisplay.assignableCCs[i]); }
        else { strncpy(labelStr, settingsToDisplay.potLabels[i], POT_LABEL_LENGTH); labelStr[POT_LABEL_LENGTH] = '\0'; }
        char* start = labelStr;
        while (*start && isspace(*start)) start++;
        char* end = labelStr + strlen(labelStr) - 1;
        while (end > start && isspace(*end)) end--;
        *(end + 1) = '\0';
        int16_t xl, yl; uint16_t wl, hl;
        tft.getTextBounds(start, 0, 0, &xl, &yl, &wl, &hl);
        tft.setCursor(xCenterNumBox - (wl / 2), pot_num_y_start + (i * pot_num_spacing_y) + pot_num_display_height + 7);
        tft.print(start);
    }

    const int bar_area_width = SCREEN_WIDTH - 65, bar_colWidth = bar_area_width / 4;
    const int barWidth = 20, barMaxHeight = 180, bar_y_start = 65, bar_label_y = bar_y_start + barMaxHeight + 15;
    for (int i = 4; i < NUM_ANALOG_POTS; i++) {
        int col = i - 4;
        int xCenter = (col * bar_colWidth) + (bar_colWidth / 2) + 10;
        int barX = xCenter - (barWidth / 2);
        tft.fillRect(barX, bar_y_start, barWidth, barMaxHeight, THEME_INACTIVE);
        char displayStr[10];
        if (showCCs) { sprintf(displayStr, "%d", settingsToDisplay.assignableCCs[i]); }
        else { strncpy(displayStr, settingsToDisplay.potLabels[i], POT_LABEL_LENGTH); displayStr[POT_LABEL_LENGTH] = '\0'; }
        char* start = displayStr;
        while (*start && isspace(*start)) start++;
        char* end = displayStr + strlen(displayStr) - 1;
        while (end > start && isspace(*end)) end--;
        *(end + 1) = '\0';
        int16_t xl, yl; uint16_t wl, hl;
        tft.getTextBounds(start, 0, 0, &xl, &yl, &wl, &hl);
        tft.setCursor(xCenter - (wl / 2), bar_label_y - 8);
        tft.print(start);
    }
    for (int i = 0; i < NUM_DIGITAL_MIDI_BUTTONS; i++) {
        int inputIndex = NUM_ANALOG_POTS + i;
        int xCenter = (i * bar_colWidth) + (bar_colWidth / 2) + 10;
        int yCenter_indicator = bar_y_start + indicator_y_offset;
        tft.fillCircle(xCenter, yCenter_indicator, indicator_radius, THEME_BG);
        int16_t xl, yl; uint16_t wl, hl;
        if (showCCs) {
            char displayStr[10];
            sprintf(displayStr, "%d", settingsToDisplay.assignableCCs[inputIndex]);
            tft.getTextBounds(displayStr, 0, 0, &xl, &yl, &wl, &hl);
            int cc_x_pos = xCenter - (wl / 2);
            int cc_y_pos = yCenter_indicator - (hl / 2);
            tft.setTextColor(THEME_FG);
            tft.setCursor(cc_x_pos, cc_y_pos);
            tft.print(displayStr);
        } else {
            if (currentAppMode == SETUP_MENU && currentSetupSubMode == LABEL_ASSIGN_LIST) {
                char displayStr[POT_LABEL_LENGTH + 1];
                strncpy(displayStr, settingsToDisplay.buttonLabels[i], POT_LABEL_LENGTH);
                displayStr[POT_LABEL_LENGTH] = '\0';
                char* start = displayStr;
                while (*start && isspace(*start)) start++;
                char* end = displayStr + strlen(displayStr) - 1;
                while (end > start && isspace(*end)) end--;
                *(end + 1) = '\0';
                tft.getTextBounds(start, 0, 0, &xl, &yl, &wl, &hl);
                int label_x_pos = xCenter - (wl / 2);
                int label_y_pos = yCenter_indicator - (hl / 2);
                tft.setTextColor(THEME_FG);
                tft.setCursor(label_x_pos, label_y_pos);
                tft.print(start);
            }
        }
    }
    tft.endWrite();
}

void updateMainScreenLabelsOnly(const BankSettings& settingsToDisplay, bool showCCs) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    const int pot_num_display_width = 54, pot_num_display_height = 30;
    const int pot_num_x_center_offset = SCREEN_WIDTH - 32;
    const int pot_num_x_start = pot_num_x_center_offset - (pot_num_display_width / 2);
    const int pot_num_y_start = 38, pot_num_spacing_y = 59;
    const int bar_area_width = SCREEN_WIDTH - 65, bar_colWidth = bar_area_width / 4;
    const int barMaxHeight = 180, bar_y_start = 65, bar_label_y = bar_y_start + barMaxHeight + 15;

    for (int i = 0; i < 4; i++) {
        char displayStr[10];
        char trimmedLabel[POT_LABEL_LENGTH + 1] = "";

        if (showCCs) {
            sprintf(displayStr, "%d", settingsToDisplay.assignableCCs[i]);
            strcpy(trimmedLabel, displayStr);
        } else {
            strncpy(displayStr, settingsToDisplay.potLabels[i], POT_LABEL_LENGTH);
            displayStr[POT_LABEL_LENGTH] = '\0';

            char* start = displayStr;
            while (*start && isspace(*start)) start++;
            char* end = displayStr + strlen(displayStr) - 1;
            while (end > start && isspace(*end)) end--;
            *(end + 1) = '\0';
            strcpy(trimmedLabel, start);
        }

        int16_t xl, yl; uint16_t wl, hl;
        tft.getTextBounds(trimmedLabel, 0, 0, &xl, &yl, &wl, &hl);

        int xCenterNumBox = pot_num_x_start + (pot_num_display_width / 2);
        int label_y_pos = pot_num_y_start + (i * pot_num_spacing_y) + pot_num_display_height + 7;

        tft.fillRect(pot_num_x_start - 10, pot_num_y_start + (i * pot_num_spacing_y) + pot_num_display_height + 5, pot_num_display_width + 20, 20, THEME_BG);
        tft.setTextColor(THEME_FG);
        tft.setCursor(xCenterNumBox - (wl / 2), label_y_pos);
        tft.print(trimmedLabel);
    }

    for (int i = 4; i < NUM_ANALOG_POTS; i++) {
        char displayStr[10];
        char trimmedLabel[POT_LABEL_LENGTH + 1] = "";

        if (showCCs) {
            sprintf(displayStr, "%d", settingsToDisplay.assignableCCs[i]);
            strcpy(trimmedLabel, displayStr);
        } else {
            strncpy(displayStr, settingsToDisplay.potLabels[i], POT_LABEL_LENGTH);
            displayStr[POT_LABEL_LENGTH] = '\0';

            char* start = displayStr;
            while (*start && isspace(*start)) start++;
            char* end = displayStr + strlen(displayStr) - 1;
            while (end > start && isspace(*end)) end--;
            *(end + 1) = '\0';
            strcpy(trimmedLabel, start);
        }

        int16_t xl, yl; uint16_t wl, hl;
        tft.getTextBounds(trimmedLabel, 0, 0, &xl, &yl, &wl, &hl);

        int col = i - 4;
        int xCenter = (col * bar_colWidth) + (bar_colWidth / 2) + 10;
        int label_y_pos = bar_label_y - 8;

        tft.fillRect((col * bar_colWidth) + 10, bar_label_y - 12, bar_colWidth, 20, THEME_BG);
        tft.setTextColor(THEME_FG);
        tft.setCursor(xCenter - (wl / 2), label_y_pos);
        tft.print(trimmedLabel);
    }

    for (int i = 0; i < NUM_DIGITAL_MIDI_BUTTONS; i++) {
        uint8_t inputIndex = NUM_ANALOG_POTS + i;

        int xCenter = (i * bar_colWidth) + (bar_colWidth / 2) + 10;
        int yCenter_indicator = bar_y_start + indicator_y_offset;

        tft.fillCircle(xCenter, yCenter_indicator, indicator_radius, THEME_BG);

        int16_t xl, yl; uint16_t wl, hl;
        if (showCCs) {
            char displayStr[10];
            sprintf(displayStr, "%d", settingsToDisplay.assignableCCs[inputIndex]);
            tft.getTextBounds(displayStr, 0, 0, &xl, &yl, &wl, &hl);
            int cc_x_pos = xCenter - (wl / 2);
            int cc_y_pos = yCenter_indicator - (hl / 2);
            tft.fillRect(cc_x_pos - 5, cc_y_pos - 5, wl + 10, hl + 10, THEME_BG);
            tft.setTextColor(THEME_FG);
            tft.setCursor(cc_x_pos, cc_y_pos);
            tft.print(displayStr);
        } else {
            // Clear the area, don't draw label
            tft.getTextBounds("WWWW", 0, 0, &xl, &yl, &wl, &hl); // Use a wide string to ensure full clearing
            int label_x_pos = xCenter - (wl / 2);
            int label_y_pos = yCenter_indicator - (hl / 2);
            tft.fillRect(label_x_pos - 9, label_y_pos - 5, wl + 10, hl + 10, THEME_BG);
        }
    }

    tft.endWrite();
}

void redrawAllPotValues(const BankSettings& settingsToDisplay) {
    if (!analogInputReady || !tftConnected) return;
    int16_t currentRawPotValues[NUM_ANALOG_POTS];
    for (int i = 0; i < NUM_ANALOG_POTS; i++) {
        selectChannel(i);
        delayMicroseconds(5);
        currentRawPotValues[i] = readMedianADC(SIG_PIN);
    }
    tft.startWrite();
    const int pot_num_display_width = 54, pot_num_display_height = 30;
    const int pot_num_x_center_offset = SCREEN_WIDTH - 32;
    const int pot_num_x_start = pot_num_x_center_offset - (pot_num_display_width / 2);
    const int pot_num_y_start = 38, pot_num_spacing_y = 59;
    const int bar_area_width = SCREEN_WIDTH - 65, bar_colWidth = bar_area_width / 4;
    const int barWidth = 20, barMaxHeight = 180, bar_y_start = 65;
    for (uint8_t i = 0; i < NUM_ANALOG_POTS; i++) {
        int16_t minRaw = settingsToDisplay.potMinRaw[i];
        int16_t maxRaw = settingsToDisplay.potMaxRaw[i];
        int16_t constrainedNewRaw = constrain(currentRawPotValues[i], minRaw, maxRaw);
        uint8_t current_mapped_val = map(constrainedNewRaw, minRaw, maxRaw, 0, 128);
        current_mapped_val = constrain(current_mapped_val, 0, 127);
        if (i < 4) {
            int x_num_pos = pot_num_x_start + pot_num_display_width / 2;
            int y_num_pos = pot_num_y_start + (i * pot_num_spacing_y) + pot_num_display_height / 2;
            tft.setFont(NULL);
            tft.setTextSize(3);
            tft.setTextColor(THEME_FG);
            char valStr[5]; sprintf(valStr, "%d", current_mapped_val);
            int16_t x1, y1; uint16_t w, h;
            tft.getTextBounds(valStr, 0, 0, &x1, &y1, &w, &h);
            tft.setCursor(x_num_pos - (w / 2), y_num_pos + (h / 2) - 22);
            tft.print(valStr);
            lastDisplayedCCValueForPot[i] = current_mapped_val;
            tft.setTextSize(1);
        } else {
            int col = i - 4;
            int xCenter = (col * bar_colWidth) + (bar_colWidth / 2) + 10;
            int barX = xCenter - (barWidth / 2);
            tft.fillRect(barX, bar_y_start, barWidth, barMaxHeight, THEME_INACTIVE);
            int newFillHeight = map(constrainedNewRaw, minRaw, maxRaw, 0, barMaxHeight);
            if (newFillHeight > 0) {
                int yPos = bar_y_start + (barMaxHeight - newFillHeight);
                tft.fillRect(barX, yPos, barWidth, newFillHeight, THEME_FG);
            }
            lastRawValueForDisplay[i] = currentRawPotValues[i];
        }
    }
    tft.endWrite();
}

void updatePotVisual(uint8_t potIndex, uint8_t newCCValue, const BankSettings& settingsToDisplay) {
    if (!tftConnected) return;
    tft.startWrite();
    if (potIndex < 4) {
        if (newCCValue == lastDisplayedCCValueForPot[potIndex]) { tft.endWrite(); return; }
        const int pot_num_display_width = 54, pot_num_display_height = 30;
        const int pot_num_x_center_offset = SCREEN_WIDTH - 32;
        const int pot_num_x_start = pot_num_x_center_offset - (pot_num_display_width / 2);
        const int pot_num_y_start = 38, pot_num_spacing_y = 59;
        int x_num_pos = pot_num_x_start + pot_num_display_width / 2;
        int y_num_pos = pot_num_y_start + (potIndex * pot_num_spacing_y) + pot_num_display_height / 2;
        tft.fillRect(pot_num_x_start + 1, pot_num_y_start + (potIndex * pot_num_spacing_y) + 1, pot_num_display_width - 2, pot_num_display_height - 2, THEME_BG);
        tft.setFont(NULL);
        tft.setTextSize(3);
        tft.setTextColor(THEME_FG);
        char valStr[5]; sprintf(valStr, "%d", newCCValue);
        int16_t x1, y1; uint16_t w, h;
        tft.getTextBounds(valStr, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor(x_num_pos - (w / 2), y_num_pos + (h / 2) - 22);
        tft.print(valStr);
        lastDisplayedCCValueForPot[potIndex] = newCCValue;
        tft.setTextSize(1);
    } else {
        int16_t oldRawValue = lastRawValueForDisplay[potIndex];
        int16_t minRaw = settingsToDisplay.potMinRaw[potIndex], maxRaw = settingsToDisplay.potMaxRaw[potIndex];
        int16_t newRawValueApprox = map(newCCValue, 0, 127, minRaw, maxRaw);
        newRawValueApprox = constrain(newRawValueApprox, minRaw, maxRaw);
        const int bar_area_width = SCREEN_WIDTH - 65, bar_colWidth = bar_area_width / 4;
        const int barWidth = 20, barMaxHeight = 180, bar_y_start = 65;
        int col = potIndex - 4;
        int xCenter = (col * bar_colWidth) + (bar_colWidth / 2) + 10;
        int barX = xCenter - (barWidth / 2);
        int oldFillHeight = map(constrain(oldRawValue, minRaw, maxRaw), minRaw, maxRaw, 0, barMaxHeight);
        int newFillHeight = map(newRawValueApprox, minRaw, maxRaw, 0, barMaxHeight);
        if (newFillHeight > oldFillHeight) {
            int diffHeight = newFillHeight - oldFillHeight;
            int yPos = bar_y_start + (barMaxHeight - newFillHeight);
            tft.fillRect(barX, yPos, barWidth, diffHeight, THEME_FG);
        } else if (newFillHeight < oldFillHeight) {
            int diffHeight = oldFillHeight - newFillHeight;
            int yPos = bar_y_start + (barMaxHeight - oldFillHeight);
            tft.fillRect(barX, yPos, barWidth, diffHeight, THEME_INACTIVE);
        }
        lastRawValueForDisplay[potIndex] = newRawValueApprox;
    }
    tft.endWrite();
}

void resetRadio() {
    // Power down the radio if it's currently active to ensure settings are applied.
    if (radioActive) {
        nrf.powerDown();
    }
    // Re-initialize the radio with the current global settings.
    nrf.begin(); 
    nrf.setPALevel(NRF_PA_MAX);
    nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_POS8DBM);
    nrf.setAutoAck(true); 
    nrf.setRetries(0, 5); // Set standard retries
    nrf.setChannel(globalSettings.esbChannel); // Apply the ESB channel from global settings
    nrf.stopListening(); 
    
    // Mark the radio as active and update the last activity timestamp.
    radioActive = true;
    lastRadioActivityTime = millis();
}

void idleRadioCheck() {
    if (radioActive && millis() - lastRadioActivityTime > NRF_IDLE_TIMEOUT_MS) {
        nrf.powerDown(); radioActive = false;
    }
}

void handleScreenFade() {
    // Static variable to track the screen state from the previous call.
    static ScreenState lastScreenState = screenState;
    // Static variable to store the brightness at the moment a fade begins.
    static uint16_t fadeStartBrightness_14bit = 0;

    // If a new fade sequence is starting, capture the current brightness.
    if (screenState != lastScreenState &&
        (screenState == FADING_UP || screenState == FADING_DOWN || screenState == FADING_REVERT)) {
        fadeStartBrightness_14bit = currentBrightness_14bit;
    }

    // If not fading, there's nothing to do. Update state and exit.
    if (screenState == BRIGHT || screenState == DIM) {
        lastScreenState = screenState;
        return;
    }

    unsigned long currentTimeUs = micros();
    unsigned long elapsedTime = currentTimeUs - fadeStartTime;
    uint16_t targetBrightness_14bit = map(globalSettings.brightness, 3, 255, BRIGHTNESS_MIN_14BIT, BRIGHTNESS_MAX_14BIT);

    if (elapsedTime >= FADE_DURATION_US) {
        // Fade is complete, snap to the final value.
        if (screenState == FADING_DOWN) {
            screenState = DIM;
            setBacklight(DIM_BRIGHTNESS_14BIT);
        } else if (screenState == FADING_UP || screenState == FADING_IN) {
            screenState = BRIGHT;
            setBacklight(targetBrightness_14bit);
        } else if (screenState == FADING_REVERT) {
            screenState = BRIGHT;
            uint16_t targetPwmValue = map(brightnessOnEnterEdit, 3, 255, BRIGHTNESS_MIN_14BIT, BRIGHTNESS_MAX_14BIT);
            setBacklight(targetPwmValue);
        }
    } else {
        // Fade is in progress, calculate the intermediate value.
        float progress = (float)elapsedTime / FADE_DURATION_US;
        uint16_t calculatedBrightness;

        if (screenState == FADING_DOWN) {
            // *** MODIFICATION: Apply an ease-out curve for a smoother dimming effect ***
            // This transforms the linear progress into a curve that starts faster and
            // gently slows down as it approaches the final dimmed state, avoiding the "braking" feeling.
            float easedProgress = 1.0f - pow(1.0f - progress, 3.0f); // Cubic ease-out curve
            calculatedBrightness = mapFloat(easedProgress, 0.0, 1.0, fadeStartBrightness_14bit, DIM_BRIGHTNESS_14BIT);
        } else if (screenState == FADING_UP) {
            // Fade from the captured start brightness up to the target bright level.
            calculatedBrightness = mapFloat(progress, 0.0, 1.0, fadeStartBrightness_14bit, targetBrightness_14bit);
        } else if (screenState == FADING_REVERT) {
             // Fade from the captured start brightness back to the level before editing began.
            uint16_t targetPwmValue = map(brightnessOnEnterEdit, 3, 255, BRIGHTNESS_MIN_14BIT, BRIGHTNESS_MAX_14BIT);
            calculatedBrightness = mapFloat(progress, 0.0, 1.0, fadeStartBrightness_14bit, targetPwmValue);
        } else { // FADING_IN (Boot-up)
            // The boot-up fade always starts from 0.
            calculatedBrightness = mapFloat(progress, 0.0, 1.0, 0, targetBrightness_14bit);
        }
        setBacklight(calculatedBrightness);
    }

    // Update the state for the next function call.
    lastScreenState = screenState;
}

void enterSystemOff(ShutdownReason reason) {
    saveCurrentBankSettings();
    if (tftConnected) {
        tft.startWrite();
        tft.fillScreen(THEME_BG);
        tft.setTextColor(THEME_FG);
        int16_t x1, y1;
        uint16_t w, h;

        switch (reason) {
            // ... (case statements are unchanged)
            case LOW_BATTERY_SHUTDOWN:
                tft.setTextSize(2);
                tft.getTextBounds("Shutting Down", 0, 0, &x1, &y1, &w, &h);
                tft.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT / 2) + 10);
                tft.print(F("Shutting Down"));
                tft.getTextBounds("Low Battery,", 0, 0, &x1, &y1, &w, &h);
                tft.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT / 2) - 30);
                tft.print(F("Low Battery,"));
                tft.endWrite();
                delay(4000);
                break;
            case WAKE_LOW_BATTERY:
                tft.setTextSize(2);
                tft.getTextBounds("Please recharge", 0, 0, &x1, &y1, &w, &h);
                tft.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT / 2) + 10);
                tft.print(F("Please recharge"));
                tft.getTextBounds("Battery low,", 0, 0, &x1, &y1, &w, &h);
                tft.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT / 2) - 30);
                tft.print(F("Battery low,"));
                tft.endWrite();
                delay(4000);
                break;
            case NORMAL:
            default:
                tft.setTextSize(2);
                tft.getTextBounds("Sleeping...", 0, 0, &x1, &y1, &w, &h);
                tft.setCursor((SCREEN_WIDTH - w) / 2, (SCREEN_HEIGHT - h) / 2);
                tft.print(F("Sleeping..."));
                tft.endWrite();
                delay(500);
                break;
        }

        // Use the current brightness value as the starting point for the fade out
        uint16_t startBrightness_14bit = currentBrightness_14bit;

        if (startBrightness_14bit > 0) {
            unsigned long fadeOutStartTime = micros();
            while (micros() - fadeOutStartTime < FADE_DURATION_US) {
                float progress = (float)(micros() - fadeOutStartTime) / FADE_DURATION_US;
                uint16_t currentBrightness = mapFloat(progress, 0.0, 1.0, startBrightness_14bit, 0);
                setBacklight(currentBrightness); // Use the new helper function
            }
        }
        setBacklight(0); // Ensure it's fully off

        tft.startWrite();
        tft.writeCommand(ST77XX_DISPOFF);
        tft.endWrite();
        delay(120);
    }

    // ... (rest of the function is unchanged)
    pinMode(EXT_VCC_PIN, OUTPUT); digitalWrite(EXT_VCC_PIN, LOW);
    nrf.powerDown(); radioActive = false;
    const uint32_t WAKEUP_PINS[] = { NRF_GPIO_PIN_MAP(0, 6)};
    const size_t NUM_WAKEUP_PINS = sizeof(WAKEUP_PINS) / sizeof(WAKEUP_PINS[0]);
    const uint32_t EXPLICIT_LOW_OUTPUT_PINS[] = { EXT_VCC_PIN, LED_BUILTIN_NRF_PIN, NRF_GPIO_PIN_MAP(0, TFT_BL), };
    const size_t NUM_EXPLICIT_LOW_OUTPUT_PINS = sizeof(EXPLICIT_LOW_OUTPUT_PINS) / sizeof(EXPLICIT_LOW_OUTPUT_PINS[0]);
    auto is_pin_in_list = [](uint32_t pin_map, const uint32_t* list, size_t size) {
        for (size_t i = 0; i < size; ++i) if (list[i] == pin_map) return true;
        return false;
    };
    for (uint32_t pin = 0; pin < 32; pin++) {
        uint32_t current_pin_map = NRF_GPIO_PIN_MAP(0, pin);
        if (is_pin_in_list(current_pin_map, WAKEUP_PINS, NUM_WAKEUP_PINS)) nrf_gpio_cfg_sense_input(current_pin_map, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
        else if (is_pin_in_list(current_pin_map, EXPLICIT_LOW_OUTPUT_PINS, NUM_EXPLICIT_LOW_OUTPUT_PINS)) { nrf_gpio_cfg_output(current_pin_map); nrf_gpio_pin_clear(current_pin_map); }
        else NRF_P0->PIN_CNF[pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    }
    for (uint32_t pin = 0; pin <= 15; pin++) {
        uint32_t current_pin_map = NRF_GPIO_PIN_MAP(1, pin);
        if (is_pin_in_list(current_pin_map, WAKEUP_PINS, NUM_WAKEUP_PINS)) nrf_gpio_cfg_sense_input(current_pin_map, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_LOW);
        else NRF_P1->PIN_CNF[pin] = (GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos) | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos) | (GPIO_PIN_CNF_PULL_Disabled << GPIO_PIN_CNF_PULL_Pos) | (GPIO_PIN_CNF_INPUT_Disconnect << GPIO_PIN_CNF_INPUT_Pos) | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    }
    setBacklight(0);
    NRF_UART0->ENABLE = 0; NRF_TWI0->ENABLE = 0; NRF_TWI1->ENABLE = 0; NRF_SPI0->ENABLE = 0; NRF_SPIM3->ENABLE = 0;
    Serial.flush(); delay(10);
    NRF_POWER->DCDCEN = 1; NRF_POWER->SYSTEMOFF = 1;
    while (1);
}


void drawFooterInstructions(const __FlashStringHelper* text) {
    if (!tftConnected) return;
    forceFooterRedraw = true;
    tft.startWrite();
    int16_t y_pos = SCREEN_HEIGHT - 20;
    tft.fillRect(0, y_pos - 5, SCREEN_WIDTH, 25, THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(1.55);
    tft.setTextColor(THEME_FG);

    int16_t x1, y1;
    uint16_t w, h;
    char buffer[30];
    strcpy_P(buffer, (const char *)text);
    tft.getTextBounds(buffer, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, y_pos);
    tft.print(buffer);

    tft.endWrite();
}

void drawActionFooter(const __FlashStringHelper* left, const __FlashStringHelper* center, const __FlashStringHelper* right) {
    static char lastLeft[12] = "";
    static char lastCenter[12] = "";
    static char lastRight[12] = "";

    if (!tftConnected) return;

    char left_buf[12], center_buf[12], right_buf[12];
    strcpy_P(left_buf, (const char*)left);
    strcpy_P(center_buf, (const char*)center);
    strcpy_P(right_buf, (const char*)right);

    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.55);

    int16_t y_pos = SCREEN_HEIGHT - 20;
    int16_t text_h = 12;
    int16_t col_width = SCREEN_WIDTH / 4;
    int16_t margin = (SCREEN_WIDTH - (col_width * 3)) / 2;

    if (forceFooterRedraw) {
        tft.fillRect(0, y_pos - 5, SCREEN_WIDTH, 25, THEME_BG);
        tft.setTextColor(THEME_FG);
        tft.setCursor(margin + col_width, y_pos);
        tft.print(F("|"));
        tft.setCursor(margin + col_width * 2, y_pos);
        tft.print(F("|"));
        strcpy(lastLeft, " ");
        strcpy(lastCenter, " ");
        strcpy(lastRight, " ");
        forceFooterRedraw = false;
    }

    int16_t x1, y1; uint16_t w, h;

    if (strcmp(left_buf, lastLeft) != 0) {
        tft.fillRect(margin, y_pos - 2, col_width - 1, text_h + 2, THEME_BG);
        tft.getTextBounds(left_buf, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor(margin + (col_width - w) / 2, y_pos);
        tft.setTextColor(THEME_FG);
        tft.print(left_buf);
        strcpy(lastLeft, left_buf);
    }

    if (strcmp(center_buf, lastCenter) != 0) {
        tft.fillRect(margin + col_width + 1, y_pos - 2, col_width - 2, text_h + 2, THEME_BG);
        tft.getTextBounds(center_buf, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor(margin + col_width + (col_width - w) / 2, y_pos);
        tft.setTextColor(THEME_FG);
        tft.print(center_buf);
        strcpy(lastCenter, center_buf);
    }

    if (strcmp(right_buf, lastRight) != 0) {
        tft.fillRect(margin + col_width * 2 + 1, y_pos - 2, col_width - 1, text_h + 2, THEME_BG);
        tft.getTextBounds(right_buf, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor(margin + col_width * 2 + (col_width - w) / 2, y_pos);
        tft.setTextColor(THEME_FG);
        tft.print(right_buf);
        strcpy(lastRight, right_buf);
    }

    tft.endWrite();
}

// --- ADDED: Helper function to show BLE connection status ---
void drawBleConnectionStatus() {
    // Clear the area first to prevent overdrawing old text
    tft.fillRect(0, SCREEN_HEIGHT - 45, SCREEN_WIDTH, 30, THEME_BG);
    if (bleRunning && connHandle != BLE_CONN_HANDLE_INVALID && strlen(blePeerName) > 0) {
        tft.setTextSize(1.5);
        tft.setTextColor(THEME_FG);
        char connInfo[40];
        sprintf(connInfo, "Connected: %s", blePeerName);
        int16_t x1, y1; uint16_t w, h;
        tft.getTextBounds(connInfo, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH - w) / 2, SCREEN_HEIGHT - 40);
        tft.print(connInfo);
    }
}

void drawSetupMenu() {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);

    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    char title[25];
    sprintf(title, "SETUP (Bank %d)", globalSettings.activeBank + 1);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(title);

    tft.setTextSize(1.7);
    int16_t y_start = 50;
    int16_t line_height = 22;

    char buffer[30];
    for (int i = 0; i < TOTAL_MAIN_MENU_ITEMS; i++) {
        strcpy_P(buffer, (char*)pgm_read_ptr(&(menuItems[i])));
        int16_t current_y = y_start + i * line_height;
        if (i == menuCursor) {
            tft.fillRect(0, current_y - 4, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
            tft.setTextColor(THEME_BG);
        } else {
            tft.setTextColor(THEME_FG);
        }
        tft.setCursor(5, current_y);
        tft.print(buffer);
    }
    
    // --- MODIFIED: Show connection status in main setup menu ---
    drawBleConnectionStatus();

    tft.endWrite();
}

void drawDeviceSetupMenu() {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);

    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds("Device Setup", 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(F("Device Setup"));

    tft.setTextSize(1.7);
    int16_t y_start = 50;
    int16_t line_height = 22;

    char buffer[30];
    for (int i = 0; i < TOTAL_DEVICE_SETUP_ITEMS; i++) {
        strcpy_P(buffer, (char*)pgm_read_ptr(&(deviceSetupMenuItems[i])));
        int16_t current_y = y_start + i * line_height;
        if (i == menuCursor) {
            tft.fillRect(0, current_y - 4, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
            tft.setTextColor(THEME_BG);
        } else {
            tft.setTextColor(THEME_FG);
        }
        tft.setCursor(5, current_y);
        tft.print(buffer);
    }
    
    tft.endWrite();
}

void drawBluetoothMenu() {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);

    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds("Bluetooth", 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(F("Bluetooth"));

    tft.setTextSize(1.8);
    int16_t y_start = 85;
    int16_t line_height = 40;

    const char* bleOption1 = bleRunning ? "1. Turn off BLE" : "1. Turn on BLE";
    const char* options[] = {bleOption1, "2. Disconnect"};

    for (int i = 0; i < TOTAL_BLUETOOTH_MENU_ITEMS; i++) {
        int16_t current_y = y_start + i * line_height;
        tft.getTextBounds(options[i], 0, 0, &x1, &y1, &w, &h);

        if (i == menuCursor) {
            tft.fillRect(0, current_y - 10, SCREEN_WIDTH, line_height - 4, THEME_ACCENT);
            tft.setTextColor(THEME_BG);
        } else {
            tft.setTextColor(THEME_FG);
        }
        tft.setCursor((SCREEN_WIDTH - w) / 2, current_y);
        tft.print(options[i]);
    }

    // --- MODIFIED: Show connection status in Bluetooth menu ---
    drawBleConnectionStatus();

    tft.endWrite();
}

void partialRedrawBluetoothMenu(uint8_t oldCursor, uint8_t newCursor) {
    if (!tftConnected) return;
    const char* bleOption1 = bleRunning ? "1. Turn off BLE" : "1. Turn on BLE";
    const char* options[] = {bleOption1, "2. Disconnect"};

    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    int16_t y_start = 85;
    int16_t line_height = 40;
    int16_t x1, y1; uint16_t w, h;

    int16_t old_y = y_start + oldCursor * line_height;
    tft.getTextBounds(options[oldCursor], 0, 0, &x1, &y1, &w, &h);
    tft.fillRect(0, old_y - 10, SCREEN_WIDTH, line_height - 4, THEME_BG);
    tft.setTextColor(THEME_FG);
    tft.setCursor((SCREEN_WIDTH - w) / 2, old_y);
    tft.print(options[oldCursor]);

    int16_t new_y = y_start + newCursor * line_height;
    tft.getTextBounds(options[newCursor], 0, 0, &x1, &y1, &w, &h);
    tft.fillRect(0, new_y - 10, SCREEN_WIDTH, line_height - 4, THEME_ACCENT);
    tft.setTextColor(THEME_BG);
    tft.setCursor((SCREEN_WIDTH - w) / 2, new_y);
    tft.print(options[newCursor]);

    tft.endWrite();
}


void drawSubMenu(const __FlashStringHelper* title, const char* const options[], uint8_t cursorPosition, uint8_t numOptions) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);

    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;
    char buffer[30];
    strcpy_P(buffer, (const char*)title);
    tft.getTextBounds(buffer, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(buffer);

    tft.setTextSize(1.8);
    int16_t y_start = 65;
    int16_t line_height = 30;

    for (int i = 0; i < numOptions; i++) {
        int16_t current_y = y_start + i * line_height;
        if (i == cursorPosition) {
            tft.fillRect(0, current_y - 6, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
            tft.setTextColor(THEME_BG);
        } else {
            tft.setTextColor(THEME_FG);
        }
        tft.setCursor(15, current_y);
        tft.print(options[i]);
    }
    tft.endWrite();
}

void drawSubMenu(const char* title, const char* const options[], uint8_t cursorPosition, uint8_t numOptions) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);

    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(title);

    tft.setTextSize(1.8);
    int16_t y_start = 65;
    int16_t line_height = 30;

    for (int i = 0; i < numOptions; i++) {
        int16_t current_y = y_start + i * line_height;
        if (i == cursorPosition) {
            tft.fillRect(0, current_y - 6, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
            tft.setTextColor(THEME_BG);
        } else {
            tft.setTextColor(THEME_FG);
        }
        tft.setCursor(15, current_y);
        tft.print(options[i]);
    }
    tft.endWrite();
}

void drawButtonModeMenu(uint8_t cursorPosition) {
    if (!tftConnected) return;
    const char* options[] = {"Momentary", "Toggle"};
    uint8_t numOptions = 2;

    tft.startWrite();
    tft.fillScreen(THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);

    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds("Button Mode", 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(F("Button Mode"));

    tft.setTextSize(1.8);
    int16_t y_start = 85;
    int16_t line_height = 40;

    for (int i = 0; i < numOptions; i++) {
        int16_t current_y = y_start + i * line_height;
        tft.getTextBounds(options[i], 0, 0, &x1, &y1, &w, &h);

        if (i == cursorPosition) {
            tft.fillRect(0, current_y - 10, SCREEN_WIDTH, line_height - 4, THEME_ACCENT);
            tft.setTextColor(THEME_BG);
        } else {
            tft.setTextColor(THEME_FG);
        }
        tft.setCursor((SCREEN_WIDTH - w) / 2, current_y);
        tft.print(options[i]);
    }
    tft.endWrite();
}

void drawMidiChannelEditMenu(uint8_t channel, bool initialDraw) {
    if (!tftConnected) return;
    if (initialDraw) {
        tft.startWrite();
        tft.fillScreen(THEME_BG);
        tft.setFont(NULL);
        tft.setTextSize(2.0);
        tft.setTextColor(THEME_FG);
        int16_t x1, y1; uint16_t w, h;
        tft.getTextBounds("MIDI Channel", 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
        tft.print(F("MIDI Channel"));
        tft.endWrite();
        drawFooterInstructions(F("UP/DN  SAVE  BACK"));
    }
    updateMidiChannelValueOnly(channel, initialDraw);
}

void updateMidiChannelValueOnly(uint8_t channel, bool forceRedraw) {
    if (!tftConnected) return;
    static char lastValStr[4] = "";
    if (forceRedraw) lastValStr[0] = '\0';
    char valueStr[4]; sprintf(valueStr, "%d", channel);
    if (strcmp(lastValStr, valueStr) == 0) return;
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(3.5);

    int16_t x1_max, y1_max; uint16_t w_max, h_max;
    tft.getTextBounds("16", 0, 0, &x1_max, &y1_max, &w_max, &h_max);

    int clear_area_width = w_max + 10;
    int clear_area_height = h_max + 5;
    int clear_area_x = (SCREEN_WIDTH - clear_area_width) / 2;
    int clear_area_y = 100;

    tft.fillRect(clear_area_x, clear_area_y, clear_area_width, clear_area_height, THEME_BG);

    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(valueStr, 0, 0, &x1, &y1, &w, &h);

    tft.setCursor(clear_area_x + (clear_area_width - w) / 2, clear_area_y + (clear_area_height - h) / 2);
    tft.setTextColor(THEME_FG);
    tft.print(valueStr);
    tft.endWrite();
    strcpy(lastValStr, valueStr);
}

void drawESBChannelEditMenu(uint8_t channelValue, bool initialDraw) {
    if (!tftConnected) return;
    if (initialDraw) {
        tft.startWrite();
        tft.fillScreen(THEME_BG);
        tft.setFont(NULL);
        tft.setTextSize(2.0);
        tft.setTextColor(THEME_FG);
        int16_t x1, y1; uint16_t w, h;
        tft.getTextBounds("ESB Ch (Global)", 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
        tft.print(F("ESB Ch (Global)"));
        tft.endWrite();
        drawFooterInstructions(F("UP/DN  SAVE  BACK"));
    }
    updateESBChannelValueOnly(channelValue, initialDraw);
}

void updateESBChannelValueOnly(uint8_t channelValue, bool forceRedraw) {
    if (!tftConnected) return;
    static char lastValStr[8] = "";
    if(forceRedraw) lastValStr[0] = '\0';
    char textStr[8]; sprintf(textStr, "Ch %d", channelValue);
    if (strcmp(lastValStr, textStr) == 0) return;
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(3.5);

    int16_t x1_max, y1_max; uint16_t w_max, h_max;
    tft.getTextBounds("Ch 100", 0, 0, &x1_max, &y1_max, &w_max, &h_max);

    int clear_area_width = w_max + 10;
    int clear_area_height = h_max + 5;
    int clear_area_x = (SCREEN_WIDTH - clear_area_width) / 2;
    int clear_area_y = 100;

    tft.fillRect(clear_area_x, clear_area_y, clear_area_width, clear_area_height, THEME_BG);

    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(textStr, 0, 0, &x1, &y1, &w, &h);

    tft.setCursor(clear_area_x + (clear_area_width - w) / 2, clear_area_y + (clear_area_height - h) / 2);
    tft.setTextColor(THEME_FG);
    tft.print(textStr);
    tft.endWrite();
    strcpy(lastValStr, textStr);
}

void drawCCAssignmentEditMenu(uint8_t inputIndex, int ccValue, bool initialDraw) {
    if (!tftConnected) return;
    if (initialDraw) {
        tft.startWrite();
        tft.fillScreen(THEME_BG);
        tft.setFont(NULL);
        tft.setTextSize(1.8);
        tft.setTextColor(THEME_FG);

        char title_str[30];
        sprintf(title_str, "Edit %s CC:", getInputName(inputIndex));
        int16_t x1, y1; uint16_t w, h;
        tft.getTextBounds(title_str, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH - w) / 2, 12);
        tft.print(title_str);
        tft.drawFastHLine(0, 30, SCREEN_WIDTH, THEME_FG);
        tft.endWrite();
        drawFooterInstructions(F("UP/DN  SAVE  BACK"));
    }
    updateCCAssignmentValueOnly(ccValue, initialDraw);
}

void updateCCAssignmentValueOnly(int ccValue, bool forceRedraw) {
    if (!tftConnected) return;
    static char lastValStr[4] = "";
    if (forceRedraw) lastValStr[0] = '\0';
    char valueStr[4]; sprintf(valueStr, "%d", ccValue);
    if (strcmp(lastValStr, valueStr) == 0) return;
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(3.5);

    int16_t x1_max, y1_max; uint16_t w_max, h_max;
    tft.getTextBounds("127", 0, 0, &x1_max, &y1_max, &w_max, &h_max);

    int clear_area_width = w_max + 10;
    int clear_area_height = h_max + 5;
    int clear_area_x = (SCREEN_WIDTH - clear_area_width) / 2;
    int clear_area_y = 100;

    tft.fillRect(clear_area_x, clear_area_y, clear_area_width, clear_area_height, THEME_BG);

    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(valueStr, 0, 0, &x1, &y1, &w, &h);

    tft.setCursor(clear_area_x + (clear_area_width - w) / 2, clear_area_y + (clear_area_height - h) / 2);
    tft.setTextColor(THEME_FG);
    tft.print(valueStr);
    tft.endWrite();
    strcpy(lastValStr, valueStr);
}

void drawLabelEditMenu(uint8_t inputIndex, char* labelBuffer, uint8_t charIndex) {
    if (!tftConnected) return;
    forceFooterRedraw = true;
    tft.startWrite();
    tft.fillScreen(THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    tft.setTextColor(THEME_FG);

    char title_str[30];
    if (inputIndex < NUM_ANALOG_POTS) sprintf(title_str, "Edit P%02d Label:", inputIndex + 1);
    else sprintf(title_str, "Edit B%02d Label:", (inputIndex - NUM_ANALOG_POTS) + 1);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(title_str, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 12);
    tft.print(title_str);
    tft.drawFastHLine(0, 30, SCREEN_WIDTH, THEME_FG);

    tft.setTextSize(3.5);
    for (int i = 0; i < POT_LABEL_LENGTH; i++) {
        char singleCharStr[2] = { labelBuffer[i], '\0' };
        tft.getTextBounds(singleCharStr, 0, 0, &x1, &y1, &w, &h);

        int16_t center_x = SCREEN_WIDTH * (0.2 + i * 0.2);
        int16_t slot_x = center_x - (CHAR_SLOT_WIDTH / 2);

        tft.fillRect(slot_x, CHAR_SLOT_Y_START, CHAR_SLOT_WIDTH, CHAR_SLOT_HEIGHT, THEME_BG);

        tft.setCursor(slot_x + (CHAR_SLOT_WIDTH - w) / 2, CHAR_SLOT_Y_START + (CHAR_SLOT_HEIGHT - h) / 2);
        tft.print(singleCharStr);
    }
    tft.endWrite();
    updateLabelCursor(charIndex, charIndex);
    updateLabelEditInstructions(charIndex);
}

void updateEditedLabelChar(uint8_t index) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(3.5);
    tft.setTextColor(THEME_FG);

    char singleCharStr[2] = { currentLabelEditBuffer[index], '\0' };
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(singleCharStr, 0, 0, &x1, &y1, &w, &h);

    int16_t center_x = SCREEN_WIDTH * (0.2 + index * 0.2);
    int16_t slot_x = center_x - (CHAR_SLOT_WIDTH / 2);

    tft.fillRect(slot_x, CHAR_SLOT_Y_START, CHAR_SLOT_WIDTH, CHAR_SLOT_HEIGHT, THEME_BG);

    tft.setCursor(slot_x + (CHAR_SLOT_WIDTH - w) / 2, CHAR_SLOT_Y_START + (CHAR_SLOT_HEIGHT - h) / 2);
    tft.print(singleCharStr);
    tft.endWrite();
}

void updateLabelCursor(uint8_t oldIndex, uint8_t newIndex) {
    if (!tftConnected) return;
    tft.startWrite();
    const int16_t cursor_w = 20, cursor_h = 3;

    int16_t cursor_y = CHAR_SLOT_Y_START + CHAR_SLOT_HEIGHT + CURSOR_Y_OFFSET;

    int16_t old_center_x = SCREEN_WIDTH * (0.2 + oldIndex * 0.2);
    int16_t old_cursor_x = old_center_x - (cursor_w / 2);
    tft.fillRect(old_cursor_x, cursor_y, cursor_w, cursor_h, THEME_BG);

    int16_t new_center_x = SCREEN_WIDTH * (0.2 + newIndex * 0.2);
    int16_t new_cursor_x = new_center_x - (cursor_w / 2);
    tft.fillRect(new_cursor_x, cursor_y, cursor_w, cursor_h, THEME_FG);
    tft.endWrite();
}

void updateLabelEditInstructions(uint8_t charIndex) {
    if (charIndex == 0) {
        drawActionFooter(F("UP/DN"), F("NEXT"), F("EXIT"));
    } else if (charIndex < POT_LABEL_LENGTH - 1) {
        drawActionFooter(F("UP/DN"), F("NEXT"), F("BACK"));
    } else {
        drawActionFooter(F("UP/DN"), F("SAVE"), F("BACK"));
    }
}

void drawCalibrationMenu() {
    if (!tftConnected) return;
    const char* autoCalOption = globalSettings.autoCalEnabled ? "1. Turn Off Auto Cal" : "1. Turn On Auto Cal";
    const char* options[] = {autoCalOption, "2. Easy Cal", "3. Manual Cal", "4. Factory Cal"};
    drawSubMenu(F("Calibration"), options, subMenuCursor, 4);
}

void partialRedrawCalibrationMenu(uint8_t oldCursor, uint8_t newCursor) {
    if (!tftConnected) return;
    const char* autoCalOption = globalSettings.autoCalEnabled ? "1. Turn Off Auto Cal" : "1. Turn On Auto Cal";
    const char* options[] = {autoCalOption, "2. Easy Cal", "3. Manual Cal", "4. Factory Cal"};

    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    int16_t y_start = 65;
    int16_t line_height = 30;

    int16_t old_y = y_start + oldCursor * line_height;
    tft.fillRect(0, old_y - 6, SCREEN_WIDTH, line_height - 2, THEME_BG);
    tft.setTextColor(THEME_FG);
    tft.setCursor(15, old_y);
    tft.print(options[oldCursor]);

    int16_t new_y = y_start + newCursor * line_height;
    tft.fillRect(0, new_y - 6, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
    tft.setTextColor(THEME_BG);
    tft.setCursor(15, new_y);
    tft.print(options[newCursor]);
    tft.endWrite();
}

void drawEasyCalPotSelectMenu() {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds("Easy-Cal Pot", 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(F("Easy-Cal Pot"));

    tft.setTextSize(1.7);
    int16_t y_start = 50;
    int16_t line_height = 22;
    for (int i = 0; i < NUM_ANALOG_POTS; i++) {
        int16_t current_y = y_start + i * line_height;
        char line[40];
        int16_t x1_line, y1_line;
        uint16_t w_line, h_line;
        sprintf(line, "P%d Min:%d Max:%d", i + 1, currentBankSettings.potMinRaw[i], currentBankSettings.potMaxRaw[i]);
        tft.getTextBounds(line, 0, 0, &x1_line, &y1_line, &w_line, &h_line);

        if (i == menuCursor) {
            tft.fillRect(0, current_y - 3, SCREEN_WIDTH, line_height-2, THEME_ACCENT);
            tft.setTextColor(THEME_BG);
        } else {
            tft.setTextColor(THEME_FG);
        }
        tft.setCursor(5, current_y);
        tft.print(line);
    }
    tft.endWrite();
}

void drawEasyCalPromptMenu(uint8_t potIndex, bool isSettingMin) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);

    char title[20]; sprintf(title, "Set Pot %d %s", potIndex + 1, isSettingMin ? "MIN" : "MAX");
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(title);

    tft.setTextSize(2);
    tft.setCursor(5, 80);
    tft.print(isSettingMin ? F("Move to MINIMUM") : F("Move to MAXIMUM"));

    tft.setFont(NULL);
    tft.setTextSize(3);
    tft.setCursor(5, 200); tft.print(F("Raw: "));
    tft.endWrite();
    drawFooterInstructions(F("SAVE | EXIT"));
    capturedRawValue = -32768;
    updateEasyCalPromptValueOnly();
}

void updateEasyCalPromptValueOnly() {
    if (!tftConnected) return;
    int16_t currentRawPotValues[NUM_ANALOG_POTS];
    if (analogInputReady) {
        for (int i = 0; i < NUM_ANALOG_POTS; i++) {
            selectChannel(i); delayMicroseconds(5);
            currentRawPotValues[i] = readMedianADC(SIG_PIN);
        }
    } else { for(int i=0; i<NUM_ANALOG_POTS; i++) currentRawPotValues[i] = 0; }
    int16_t newCapturedValue = currentRawPotValues[selectedPotForCal];
    if (newCapturedValue == capturedRawValue) return;
    capturedRawValue = newCapturedValue;
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(3);
    tft.setTextColor(THEME_FG);

    tft.fillRect(75, 195, 120, 30, THEME_BG);

    tft.setCursor(75, 200);
    tft.print(capturedRawValue);
    tft.endWrite();
}

void drawManualCalPotSelectMenu() {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds("Manual-Cal Pot", 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(F("Manual-Cal Pot"));

    tft.setTextSize(1.7);
    int16_t y_start = 50;
    int16_t line_height = 22;
    for (int i = 0; i < NUM_ANALOG_POTS; i++) {
        int16_t current_y = y_start + i * line_height;
        char line[40]; sprintf(line, "P%d Min:%d Max:%d", i + 1, currentBankSettings.potMinRaw[i], currentBankSettings.potMaxRaw[i]);
        if (i == menuCursor) {
            tft.fillRect(0, current_y - 3, SCREEN_WIDTH, line_height-2, THEME_ACCENT);
            tft.setTextColor(THEME_BG);
        } else {
            tft.setTextColor(THEME_FG);
        }
        tft.setCursor(5, current_y);
        tft.print(line);
    }
    tft.endWrite();
}

void drawManualCalValueSelectMenu(uint8_t potIndex) {
    if (!tftConnected) return;
    char title_buf[30];
    sprintf(title_buf, "Manual-Cal: Pot %d", potIndex + 1);
    const char* options[] = {"1. Set Min Raw", "2. Set Max Raw"};
    drawSubMenu(title_buf, options, subMenuCursor, 2);
}

void partialRedrawManualCalValueSelectMenu(uint8_t potIndex, uint8_t oldCursor, uint8_t newCursor) {
    if (!tftConnected) return;
    const char* options[] = {"1. Set Min Raw", "2. Set Max Raw"};
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    int16_t y_start = 65;
    int16_t line_height = 30;

    int16_t old_y = y_start + oldCursor * line_height;
    tft.fillRect(0, old_y - 6, SCREEN_WIDTH, line_height - 2, THEME_BG);
    tft.setTextColor(THEME_FG);
    tft.setCursor(15, old_y);
    tft.print(options[oldCursor]);

    int16_t new_y = y_start + newCursor * line_height;
    tft.fillRect(0, new_y - 6, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
    tft.setTextColor(THEME_BG);
    tft.setCursor(15, new_y);
    tft.print(options[newCursor]);
    tft.endWrite();
}

void drawManualCalValueEditMenu(uint8_t potIndex, bool isEditingMin, int16_t value, uint8_t digitIndex, bool initialDraw) {
    if (!tftConnected) return;
    if (initialDraw) {
        tft.startWrite();
        tft.fillScreen(THEME_BG);
        tft.setFont(NULL);
        tft.setTextSize(2.0);
        tft.setTextColor(THEME_FG);

        char title[30]; sprintf(title, "Edit Pot %d %s", potIndex + 1, isEditingMin ? "Min Raw" : "Max Raw");
        int16_t x1, y1; uint16_t w, h;
        tft.getTextBounds(title, 0, 0, &x1, &y1, &w, &h);
        tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
        tft.print(title);
        tft.endWrite();
        updateManualCalFooterInstructions(digitIndex);
    }

    int temp_val = value;
    for (int i = 4; i >= 0; i--) {
        updateManualCalDigit(i, temp_val % 10);
        temp_val /= 10;
    }
    updateManualCalCursor(digitIndex, digitIndex);
}

void updateManualCalDigit(uint8_t digitIndex, uint8_t digitValue) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(3);
    tft.setTextColor(THEME_FG);

    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds("0", 0, 0, &x1, &y1, &w, &h);
    int char_width = w;
    int char_spacing = 4;
    int total_width = (char_width * 5) + (char_spacing * 4);
    int16_t start_x = (SCREEN_WIDTH - total_width) / 2;

    int16_t x_pos = start_x + (digitIndex * (char_width + char_spacing));

    tft.fillRect(x_pos, 115, char_width, 35, THEME_BG);

    char digit_char[2];
    sprintf(digit_char, "%d", digitValue);

    int16_t x1_actual, y1_actual; uint16_t w_actual, h_actual;
    tft.getTextBounds(digit_char, 0, 0, &x1_actual, &y1_actual, &w_actual, &h_actual);

    tft.setCursor(x_pos + (char_width - w_actual) / 2, 120);
    tft.setTextColor(THEME_FG);
    tft.print(digit_char);

    tft.endWrite();
}

void updateManualCalCursor(uint8_t oldIndex, uint8_t newIndex) {
    if (!tftConnected) return;
    tft.startWrite();

    int16_t x1, y1; uint16_t w, h;
    tft.setFont(NULL);
    tft.setTextSize(3);
    tft.getTextBounds("0", 0, 0, &x1, &y1, &w, &h);
    int char_width = w;
    int char_spacing = 4;
    int total_width = (char_width * 5) + (char_spacing * 4);
    int16_t start_x = (SCREEN_WIDTH - total_width) / 2;

    int16_t old_x_pos = start_x + (oldIndex * (char_width + char_spacing));
    tft.fillRect(old_x_pos, 145, char_width, 3, THEME_BG);

    int16_t new_x_pos = start_x + (newIndex * (char_width + char_spacing));
    tft.fillRect(new_x_pos, 145, char_width, 3, THEME_FG);

    tft.endWrite();
}

void updateManualCalFooterInstructions(uint8_t digitIndex) {
    if (digitIndex == 0) {
        drawActionFooter(F("UP/DN"), F("NEXT"), F("EXIT"));
    } else if (digitIndex < 4) {
        drawActionFooter(F("UP/DN"), F("NEXT"), F("BACK"));
    } else {
        drawActionFooter(F("UP/DN"), F("SAVE"), F("BACK"));
    }
}


void drawFactoryResetConfirmMenu() {
    const char* opts[] = {"NO", "YES"};
    drawSubMenu(F("Factory Reset?"), opts, subMenuCursor, 2);
}

void drawBrightnessMenu(uint8_t initialBrightness) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds("Brightness", 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(F("Brightness"));
    tft.endWrite();

    uint16_t initialStep = map(initialBrightness, 3, 255, 1, 100);
    updateBrightnessBar(initialStep, true);

    drawFooterInstructions(F("UP/DN  SAVE  BACK"));
}

void updateBrightnessBar(uint16_t brightnessStep, bool forceRedraw) {
    if (!tftConnected) return;

    // Live preview the brightness change
    uint16_t pwmValue = map(brightnessStep, 1, 100, BRIGHTNESS_MIN_14BIT, BRIGHTNESS_MAX_14BIT);
    setBacklight(pwmValue); // Use the new helper function

    tft.startWrite();
    int bar_y = 100, bar_h = 30, bar_max_w = SCREEN_WIDTH - 20 - 4;
    static int last_fill_w = -1;

    int fill_w = map(brightnessStep, 1, 100, 0, bar_max_w);

    if (forceRedraw) {
        tft.drawRect(10, bar_y, SCREEN_WIDTH - 20, bar_h, THEME_FG);
        tft.fillRect(12, bar_y + 2, bar_max_w, bar_h - 4, THEME_BG);
        if (fill_w > 0) tft.fillRect(12, bar_y + 2, fill_w, bar_h - 4, THEME_FG);
    } else {
        if (fill_w != last_fill_w) {
            if (fill_w > last_fill_w) {
                tft.fillRect(12 + last_fill_w, bar_y + 2, fill_w - last_fill_w, bar_h - 4, THEME_FG);
            } else {
                tft.fillRect(12 + fill_w, bar_y + 2, last_fill_w - fill_w, bar_h - 4, THEME_BG);
            }
        }
    }
    last_fill_w = fill_w;

    char percent[10];
    sprintf(percent, "%d%%", brightnessStep);

    tft.setFont(NULL);
    tft.setTextSize(3);

    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds("100%", 0, 0, &x1, &y1, &w, &h);
    tft.fillRect((SCREEN_WIDTH - w)/2 - 5, 150, w + 10, h + 5, THEME_BG);

    tft.getTextBounds(percent, 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(SCREEN_WIDTH/2 - w/2, 155);
    tft.setTextColor(THEME_FG);
    tft.print(percent);
    tft.endWrite();
}

void drawSystemOffConfirmMenu() {
    const char* opts[] = {"NO", "Power Off"};
    drawSubMenu(F("Power Off?"), opts, subMenuCursor, 2);
}

void drawADCNotFound() {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);
    tft.setFont(NULL);
    tft.setTextSize(3);
    tft.setTextColor(THEME_FG);
    tft.setCursor(10, SCREEN_HEIGHT / 2 - 20); tft.println(F("ADC NOT"));
    tft.setCursor(25, SCREEN_HEIGHT / 2 + 20); tft.println(F("FOUND!"));
    tft.endWrite();
}

void drawHardwareTestLayout() {
    if (!tftConnected) return;
    tft.startWrite();
    tft.fillScreen(THEME_BG);

    tft.setFont(NULL);
    tft.setTextSize(2.0);
    tft.setTextColor(THEME_FG);
    int16_t x1, y1; uint16_t w, h;
    tft.getTextBounds("Hardware Test", 0, 0, &x1, &y1, &w, &h);
    tft.setCursor((SCREEN_WIDTH - w) / 2, 20);
    tft.print(F("Hardware Test"));

    tft.setTextSize(1.7);
    for (int i=0; i<NUM_ANALOG_POTS; i++) {
        tft.setCursor(15, 50 + i * 25);
        char label[5]; sprintf(label, "P%d:", i+1);
        tft.print(label);
    }
    tft.endWrite();
}

void updateHardwareTestValues() {
    if (!tftConnected) return;

    int16_t currentRawPotValues[NUM_ANALOG_POTS];
    if (analogInputReady) {
        for (int i = 0; i < NUM_ANALOG_POTS; i++) {
            selectChannel(i);
            delayMicroseconds(5);
            currentRawPotValues[i] = readMedianADC(SIG_PIN);
        }
    } else {
        for (int i = 0; i < NUM_ANALOG_POTS; i++) currentRawPotValues[i] = 0;
    }

    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(2.0);

    for (int i = 0; i < NUM_ANALOG_POTS; i++) {
        if (lastDrawnHwTestValues[i] != currentRawPotValues[i]) {
            int16_t x1, y1;
            uint16_t w_max, h_max;
            tft.getTextBounds("16383", 0, 0, &x1, &y1, &w_max, &h_max);

            int x_pos = 70;
            int y_pos = 50 + i * 25;

            tft.fillRect(x_pos, y_pos, w_max + 5, h_max, THEME_BG);

            char valStr[8];
            sprintf(valStr, "%d", currentRawPotValues[i]);
            tft.setTextColor(THEME_FG);
            tft.setCursor(x_pos, y_pos);
            tft.print(valStr);

            lastDrawnHwTestValues[i] = currentRawPotValues[i];
        }
    }
    tft.endWrite();
}

void selectChannel(int channel) {
    digitalWrite(S0_PIN, (channel & B0001) ? HIGH : LOW);
    digitalWrite(S1_PIN, (channel & B0010) ? HIGH : LOW);
    digitalWrite(S2_PIN, (channel & B0100) ? HIGH : LOW);
    digitalWrite(S3_PIN, (channel & B1000) ? HIGH : LOW);
}

int readMedianADC(int analogPin) {
    int samples[NUM_SAMPLES_PER_CHANNEL];
    for (int i = 0; i < NUM_SAMPLES_PER_CHANNEL; i++) samples[i] = analogRead(analogPin);
    if (NUM_SAMPLES_PER_CHANNEL == 1) return samples[0];
    for (int i = 0; i < NUM_SAMPLES_PER_CHANNEL - 1; i++) {
        for (int j = 0; j < NUM_SAMPLES_PER_CHANNEL - i - 1; j++) {
            if (samples[j] > samples[j + 1]) {
                int temp = samples[j];
                samples[j] = samples[j + 1];
                samples[j + 1] = temp;
            }
        }
    }
    if (NUM_SAMPLES_PER_CHANNEL % 2 == 1) return samples[NUM_SAMPLES_PER_CHANNEL / 2];
    else return (samples[NUM_SAMPLES_PER_CHANNEL / 2 - 1] + samples[NUM_SAMPLES_PER_CHANNEL / 2]) / 2;
}

void clearBlinkingArea(uint8_t inputIndex, bool isCCAssignmentMode) {
    tft.startWrite();
    tft.setFont(NULL);

    const int pot_num_display_width = 54, pot_num_display_height = 30;
    const int pot_num_x_center_offset = SCREEN_WIDTH - 32;
    const int pot_num_x_start = pot_num_x_center_offset - (pot_num_display_width / 2);
    const int pot_num_y_start = 38, pot_num_spacing_y = 59;

    const int bar_area_width = SCREEN_WIDTH - 65, bar_colWidth = bar_area_width / 4;
    const int barMaxHeight = 180, bar_y_start = 65, bar_label_y = bar_y_start + barMaxHeight + 15;

    if (inputIndex < NUM_ANALOG_POTS) {
        char displayStr[10];
        if (isCCAssignmentMode) {
            sprintf(displayStr, "%d", currentBankSettings.assignableCCs[inputIndex]);
            tft.setTextSize(1.8);
        } else {
            strncpy(displayStr, currentBankSettings.potLabels[inputIndex], POT_LABEL_LENGTH);
            displayStr[POT_LABEL_LENGTH] = '\0';
            char* start = displayStr; while (*start && isspace(*start)) start++;
            char* end = displayStr + strlen(displayStr) - 1; while (end > start && isspace(*end)) end--; *(end + 1) = '\0';
            strcpy(displayStr, start);
            tft.setTextSize(1.8);
        }

        int16_t xl, yl; uint16_t wl, hl;
        tft.getTextBounds(displayStr, 0, 0, &xl, &yl, &wl, &hl);

        if (inputIndex < 4) {
            int xCenterNumBox = pot_num_x_start + (pot_num_display_width / 2);
            int label_y_pos = pot_num_y_start + (inputIndex * pot_num_spacing_y) + pot_num_display_height + 7;
            tft.fillRect(pot_num_x_start, pot_num_y_start + (inputIndex * pot_num_spacing_y) + pot_num_display_height + 4, pot_num_display_width, 20, THEME_BG);
            tft.setTextColor(THEME_FG);
            tft.setCursor(xCenterNumBox - (wl / 2), label_y_pos);
            tft.print(displayStr);
        } else {
            int col = inputIndex - 4;
            int xCenter = (col * bar_colWidth) + (bar_colWidth / 2) + 10;
            int label_y_pos = bar_label_y - 8;
            tft.fillRect((col * bar_colWidth) + 10, bar_label_y - 12, bar_colWidth, 20, THEME_BG);
            tft.setTextColor(THEME_FG);
            tft.setCursor(xCenter - (wl / 2), label_y_pos);
            tft.print(displayStr);
        }
    } else {
        uint8_t buttonIndex = inputIndex - NUM_ANALOG_POTS;

        int xCenter = (buttonIndex * bar_colWidth) + (bar_colWidth / 2) + 10;
        int yCenter_indicator = bar_y_start + indicator_y_offset;

        char displayStr[10];
        if (isCCAssignmentMode) {
            sprintf(displayStr, "%d", currentBankSettings.assignableCCs[inputIndex]);
            tft.setTextSize(1.8);
        } else {
            strncpy(displayStr, currentBankSettings.buttonLabels[buttonIndex], POT_LABEL_LENGTH);
            displayStr[POT_LABEL_LENGTH] = '\0';
            char* start = displayStr; while (*start && isspace(*start)) start++;
            char* end = displayStr + strlen(displayStr) - 1; while (end > start && isspace(*end)) end--; *(end + 1) = '\0';
            strcpy(displayStr, start);
            tft.setTextSize(1.8);
        }
        int16_t xl, yl; uint16_t wl, hl;
        tft.getTextBounds(displayStr, 0, 0, &xl, &yl, &wl, &hl);
        int cc_x_pos = xCenter - (wl / 2);
        int cc_y_pos = yCenter_indicator - (hl / 2);

        tft.fillCircle(xCenter, yCenter_indicator, indicator_radius, THEME_BG);

        tft.fillRect(cc_x_pos - 5, cc_y_pos - 5, wl + 10, hl + 10, THEME_BG);
        tft.setTextColor(THEME_FG);
        tft.setCursor(cc_x_pos, cc_y_pos);
        tft.print(displayStr);
    }
    tft.endWrite();
}

void blinkAssignedInputLabelAndIndicator() {
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkToggleTime > BLINK_INTERVAL_MS) {
        blinkState = !blinkState;
        lastBlinkToggleTime = currentTime;

        tft.startWrite();
        tft.setFont(NULL);

        uint16_t textColor = blinkState ? THEME_BG : THEME_FG;
        uint16_t bgColor = blinkState ? THEME_ACCENT : THEME_BG;
        uint16_t indicatorColor = blinkState ? THEME_INDICATOR_ON : THEME_INDICATOR_OFF;

        const int pot_num_display_width = 54, pot_num_display_height = 30;
        const int pot_num_x_center_offset = SCREEN_WIDTH - 32;
        const int pot_num_x_start = pot_num_x_center_offset - (pot_num_display_width / 2);
        const int pot_num_y_start = 38, pot_num_spacing_y = 59;

        const int bar_area_width = SCREEN_WIDTH - 65, bar_colWidth = bar_area_width / 4;
        const int barMaxHeight = 180, bar_y_start = 65, bar_label_y = bar_y_start + barMaxHeight + 15;

        bool isCCAssignmentMode = (currentSetupSubMode == CC_ASSIGN_LIST);

        if (menuCursor < NUM_ANALOG_POTS) {
            char displayStr[10];
            if (isCCAssignmentMode) {
                if (blinkState) {
                    sprintf(displayStr, "%d", currentBankSettings.assignableCCs[menuCursor]);
                } else {
                    strncpy(displayStr, currentBankSettings.potLabels[menuCursor], POT_LABEL_LENGTH);
                    displayStr[POT_LABEL_LENGTH] = '\0';
                    char* start = displayStr; while (*start && isspace(*start)) start++;
                    char* end = displayStr + strlen(displayStr) - 1; while (end > start && isspace(*end)) end--; *(end + 1) = '\0';
                    strcpy(displayStr, start);
                }
                tft.setTextSize(1.8);
            } else {
                strncpy(displayStr, currentBankSettings.potLabels[menuCursor], POT_LABEL_LENGTH);
                displayStr[POT_LABEL_LENGTH] = '\0';
                char* start = displayStr; while (*start && isspace(*start)) start++;
                char* end = displayStr + strlen(displayStr) - 1; while (end > start && isspace(*end)) end--; *(end + 1) = '\0';
                strcpy(displayStr, start);
                tft.setTextSize(1.8);
            }

            int16_t xl, yl; uint16_t wl, hl;
            tft.getTextBounds(displayStr, 0, 0, &xl, &yl, &wl, &hl);

            if (menuCursor < 4) {
                int xCenterNumBox = pot_num_x_start + (pot_num_display_width / 2);
                int label_y_pos = pot_num_y_start + (menuCursor * pot_num_spacing_y) + pot_num_display_height + 7;
                tft.fillRect(pot_num_x_start, pot_num_y_start + (menuCursor * pot_num_spacing_y) + pot_num_display_height + 4, pot_num_display_width, 20, bgColor);
                tft.setTextColor(textColor);
                tft.setCursor(xCenterNumBox - (wl / 2), label_y_pos);
                tft.print(displayStr);
            } else {
                int col = menuCursor - 4;
                int xCenter = (col * bar_colWidth) + (bar_colWidth / 2) + 10;
                int label_y_pos = bar_label_y - 8;
                tft.fillRect((col * bar_colWidth) + 10, bar_label_y - 12, bar_colWidth, 20, bgColor);
                tft.setTextColor(textColor);
                tft.setCursor(xCenter - (wl / 2), label_y_pos);
                tft.print(displayStr);
            }
        } else {
            uint8_t buttonIndex = menuCursor - NUM_ANALOG_POTS;

            int xCenter = (buttonIndex * bar_colWidth) + (bar_colWidth / 2) + 10;
            int yCenter_indicator = bar_y_start + indicator_y_offset;

            char displayStr[10];
            if (isCCAssignmentMode) {
                sprintf(displayStr, "%d", currentBankSettings.assignableCCs[menuCursor]);
                tft.setTextSize(1.8);
            } else {
                strncpy(displayStr, currentBankSettings.buttonLabels[buttonIndex], POT_LABEL_LENGTH);
                displayStr[POT_LABEL_LENGTH] = '\0';
                char* start = displayStr; while (*start && isspace(*start)) start++;
                char* end = displayStr + strlen(displayStr) - 1; while (end > start && isspace(*end)) end--; *(end + 1) = '\0';
                strcpy(displayStr, start);
                tft.setTextSize(1.8);
            }
            int16_t xl, yl; uint16_t wl, hl;
            tft.getTextBounds(displayStr, 0, 0, &xl, &yl, &wl, &hl);
            int cc_x_pos = xCenter - (wl / 2);
            int cc_y_pos = yCenter_indicator - (hl / 2);

            if (blinkState) {
                tft.fillCircle(xCenter, yCenter_indicator, indicatorColor, indicatorColor);
                tft.fillRect(cc_x_pos - 5, cc_y_pos - 5, wl + 10, hl + 10, THEME_BG);
            } else {
                tft.fillCircle(xCenter, yCenter_indicator, indicator_radius, THEME_BG);
                tft.fillRect(cc_x_pos - 5, cc_y_pos - 5, wl + 10, hl + 10, THEME_BG);
                tft.setTextColor(THEME_FG);
                tft.setCursor(cc_x_pos, cc_y_pos);
                tft.print(displayStr);
            }
        }
        tft.endWrite();
    }
}


void handleSetupInput() {
    unsigned long currentTimeout = SETUP_TIMEOUT_MS;
    if (currentSetupSubMode == CALIBRATION_MAIN || currentSetupSubMode == EASY_CAL_POT_SELECT ||
        currentSetupSubMode == EASY_CAL_SET_MIN || currentSetupSubMode == EASY_CAL_SET_MAX ||
        currentSetupSubMode == CALIBRATION_MANUAL_POT_SELECT || currentSetupSubMode == CALIBRATION_MANUAL_MIN_MAX_SELECT ||
        currentSetupSubMode == CALIBRATION_MANUAL_VALUE_EDIT) {
        currentTimeout = 20000;
    } else if (currentSetupSubMode == HARDWARE_TEST) {
        currentTimeout += 30000;
    }

    if (millis() - setupStartTime > currentTimeout) {
        if (currentSetupSubMode == BRIGHTNESS_EDIT) {
            fadeStartTime = micros();
            screenState = FADING_REVERT;
        }
        
        lastRadioActivityTime = millis(); // MODIFIED: Reset idle timer on exit
        currentAppMode = NORMAL_OPERATION; loadSettings(); activeMidiSettings = &currentBankSettings;
        tft.fillScreen(THEME_BG);
        drawMainScreenPotentiometerLayout(currentBankSettings, false);
        redrawAllPotValues(currentBankSettings);
        forceFullRedraw = true;
        drawTopStatusBar(TinyUSBDevice.mounted(), false, 0, 0, false);
        forceFullRedraw = false;
        currentSetupSubMode = MAIN_MENU;
        return;
    }

    static unsigned long lastButtonPressTime[4] = {0};
    const unsigned long debounceDelay = 300;
    const unsigned long rapidChangeThreshold = 500;
    const int rapidChangeMinCount = 5;

    bool up = !digitalRead(NAV_UP_BUTTON), down = !digitalRead(NAV_DOWN_BUTTON), confirm = !digitalRead(NAV_CONFIRM_BUTTON), cancel = !digitalRead(NAV_CANCEL_BUTTON);
    if (up || down || confirm || cancel) setupStartTime = millis();

    switch (currentSetupSubMode) {
        case MAIN_MENU: {
            uint8_t oldMenuCursor = menuCursor;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) { menuCursor = (menuCursor == 0) ? (TOTAL_MAIN_MENU_ITEMS - 1) : menuCursor - 1; lastButtonPressTime[0] = millis(); }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) { menuCursor = (menuCursor == (TOTAL_MAIN_MENU_ITEMS - 1)) ? 0 : menuCursor + 1; lastButtonPressTime[1] = millis(); }
            if (oldMenuCursor != menuCursor) partialRedrawMainMenu(oldMenuCursor, menuCursor);
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastRadioActivityTime = millis(); // MODIFIED: Reset idle timer on exit
                currentAppMode = NORMAL_OPERATION; loadSettings(); activeMidiSettings = &currentBankSettings; lastButtonPressTime[3] = millis();
                tft.fillScreen(THEME_BG);
                drawMainScreenPotentiometerLayout(currentBankSettings, false);
                redrawAllPotValues(currentBankSettings);
                forceFullRedraw = true;
                drawTopStatusBar(TinyUSBDevice.mounted(), false, 0, 0, false);
                forceFullRedraw = false;
                currentSetupSubMode = MAIN_MENU;
                return;
            }
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                uint8_t selectedMenuItem = menuCursor + 1;
                lastSelectedMainMenuIndex = menuCursor;
                menuCursor = 0; subMenuCursor = 0;
                lastButtonPressTime[2] = millis();

                switch (selectedMenuItem) {
                    case 1:
                        currentSetupSubMode = BLUETOOTH_MENU;
                        menuCursor = 0;
                        drawBluetoothMenu();
                        break;
                    case 2:
                        currentSetupSubMode = CC_ASSIGN_LIST;
                        drawMainScreenPotentiometerLayout(currentBankSettings, true);
                        blinkState = true;
                        lastBlinkToggleTime = millis();
                        break;
                    case 3:
                        currentSetupSubMode = LABEL_ASSIGN_LIST;
                        drawMainScreenPotentiometerLayout(currentBankSettings, false);
                        blinkState = true;
                        lastBlinkToggleTime = millis();
                        break;
                    case 4:
                        currentSetupSubMode = BUTTON_MODE_EDIT;
                        subMenuCursor = currentBankSettings.buttonMode;
                        drawButtonModeMenu(subMenuCursor);
                        break;
                    case 5:
                        currentSetupSubMode = MIDI_CHANNEL_EDIT;
                        tempMidiChannel = currentBankSettings.midiChannel;
                        drawMidiChannelEditMenu(tempMidiChannel, true);
                        break;
                    case 6:
                        if (!analogInputReady) {
                            drawADCNotFound();
                            delay(2000);
                            drawSetupMenu();
                        } else {
                            currentSetupSubMode = DEVICE_SETUP_MENU;
                            menuCursor = 0;
                            drawDeviceSetupMenu();
                        }
                        break;
                    case 7:
                        currentSetupSubMode = POWER_OFF_CONFIRM;
                        subMenuCursor = 0;
                        drawSystemOffConfirmMenu();
                        break;
                }
            }
            break;
        }
        case DEVICE_SETUP_MENU: {
            uint8_t oldMenuCursor = menuCursor;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) { menuCursor = (menuCursor == 0) ? (TOTAL_DEVICE_SETUP_ITEMS - 1) : menuCursor - 1; lastButtonPressTime[0] = millis(); }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) { menuCursor = (menuCursor == (TOTAL_DEVICE_SETUP_ITEMS - 1)) ? 0 : menuCursor + 1; lastButtonPressTime[1] = millis(); }
            if (oldMenuCursor != menuCursor) partialRedrawMainMenu(oldMenuCursor, menuCursor);
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = MAIN_MENU;
                menuCursor = lastSelectedMainMenuIndex;
                drawSetupMenu();
            }
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                uint8_t selectedSubMenuItem = menuCursor + 1;
                lastButtonPressTime[2] = millis(); menuCursor = 0;
                switch (selectedSubMenuItem) {
                    case 1:
                        currentSetupSubMode = ESB_CHANNEL_EDIT;
                        for(int i=0; i<NUM_ESB_OPTIONS; i++) { if(esbOptions[i] == globalSettings.esbChannel) tempEsbChannelIndex = i; }
                        drawESBChannelEditMenu(globalSettings.esbChannel, true);
                        break;
                    case 2:
                        currentSetupSubMode = CALIBRATION_MAIN;
                        drawCalibrationMenu();
                        break;
                    case 3:
                        brightnessOnEnterEdit = globalSettings.brightness;
                        tempBrightnessStep = map(globalSettings.brightness, 3, 255, 1, 100);
                        currentSetupSubMode = BRIGHTNESS_EDIT;
                        drawBrightnessMenu(globalSettings.brightness);
                        break;
                    case 4:
                        {
                            currentSetupSubMode = FACTORY_RESET_CONFIRM_1;
                            subMenuCursor = 0;
                            const char* opts[] = {"NO", "YES"};
                            drawSubMenu(F("Factory Reset?"), opts, 0, 2);
                        }
                        break;
                    case 5:
                        currentSetupSubMode = HARDWARE_TEST;
                        drawHardwareTestLayout();
                        for(int i=0; i<NUM_ANALOG_POTS; i++) lastDrawnHwTestValues[i] = -32767;
                        updateHardwareTestValues();
                        break;
                    case 6: // Scan for RX
                        currentAppMode = ESB_SWEEP;
                        currentSetupSubMode = ESB_SWEEP_PROGRESS;
                        foundEsbChannelCount = 0;
                        break;
                }
            }
            break;
        }
        case BLUETOOTH_MENU: {
            uint8_t oldMenuCursor = menuCursor;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) { menuCursor = (menuCursor == 0) ? (TOTAL_BLUETOOTH_MENU_ITEMS - 1) : menuCursor - 1; lastButtonPressTime[0] = millis(); }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) { menuCursor = (menuCursor + 1) % TOTAL_BLUETOOTH_MENU_ITEMS; lastButtonPressTime[1] = millis(); }
            if (oldMenuCursor != menuCursor) partialRedrawBluetoothMenu(oldMenuCursor, menuCursor);

            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis();
                if (menuCursor == 0) {
                    if (bleRunning) {
                        stopBLE();
                    } else {
                        startBLE();
                    }
                    tft.startWrite();
                    tft.setFont(NULL);
                    tft.setTextSize(1.8);
                    int16_t y_start = 85;
                    int16_t line_height = 40;
                    int16_t x1, y1; uint16_t w, h;

                    const char* bleOption1 = bleRunning ? "1. Turn off BLE" : "1. Turn on BLE";
                    int16_t current_y = y_start + 0 * line_height;
                    tft.getTextBounds(bleOption1, 0, 0, &x1, &y1, &w, &h);

                    if (menuCursor == 0) {
                        tft.fillRect(0, current_y - 10, SCREEN_WIDTH, line_height - 4, THEME_ACCENT);
                        tft.setTextColor(THEME_BG);
                    } else {
                        tft.fillRect(0, current_y - 10, SCREEN_WIDTH, line_height - 4, THEME_BG);
                        tft.setTextColor(THEME_FG);
                    }
                    tft.setCursor((SCREEN_WIDTH - w) / 2, current_y);
                    tft.print(bleOption1);

                    drawBleConnectionStatus();
                    tft.endWrite();

                } else if (menuCursor == 1) {
                    if (bleRunning && connHandle != BLE_CONN_HANDLE_INVALID) {
                        Bluefruit.disconnect(connHandle);
                        tft.startWrite();
                        drawBleConnectionStatus();
                        tft.endWrite();
                    }
                }
            }

            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = MAIN_MENU;
                menuCursor = 0;
                drawSetupMenu();
            }
            break;
        }
        case CC_ASSIGN_LIST: {
            uint8_t oldMenuCursor = menuCursor;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) {
                clearBlinkingArea(oldMenuCursor, true);
                menuCursor = (menuCursor == 0) ? (TOTAL_ASSIGNABLE_INPUTS - 1) : menuCursor - 1;
                lastButtonPressTime[0] = millis();
            }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) {
                clearBlinkingArea(oldMenuCursor, true);
                menuCursor = (menuCursor + 1) % TOTAL_ASSIGNABLE_INPUTS;
                lastButtonPressTime[1] = millis();
            }

            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis();
                currentEditingInputIndex = menuCursor;
                currentCCEditValue = currentBankSettings.assignableCCs[currentEditingInputIndex];
                currentSetupSubMode = CC_EDIT_VALUE;
                drawCCAssignmentEditMenu(currentEditingInputIndex, currentCCEditValue, true);
                lastCCButtonPressTime = millis(); ccChangeCounter = 0; ccButtonHeld = false;
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                clearBlinkingArea(menuCursor, true);
                currentSetupSubMode = MAIN_MENU;
                menuCursor = lastSelectedMainMenuIndex;
                drawSetupMenu();
            }
            break;
        }
        case CC_EDIT_VALUE: {
            int oldValue = currentCCEditValue, step = 1;
            bool button_pressed = up || down;
            if (button_pressed) {
                if (!ccButtonHeld) { lastCCButtonPressTime = millis(); ccChangeCounter = 0; ccButtonHeld = true; }
                else if (millis() - lastCCButtonPressTime > rapidChangeThreshold) { if (ccChangeCounter > rapidChangeMinCount) step = 2; }
            } else ccButtonHeld = false;
            if (up && (millis() - lastButtonPressTime[0] > (ccButtonHeld && step > 1 ? 50 : debounceDelay))) { currentCCEditValue = (currentCCEditValue + step) % 128; lastButtonPressTime[0] = millis(); ccChangeCounter++; }
            if (down && (millis() - lastButtonPressTime[1] > (ccButtonHeld && step > 1 ? 50 : debounceDelay))) { currentCCEditValue = (currentCCEditValue == 0) ? 127 : currentCCEditValue - step; if (currentCCEditValue < 0) currentCCEditValue = 127 + currentCCEditValue; lastButtonPressTime[1] = millis(); ccChangeCounter++; }
            if (oldValue != currentCCEditValue) updateCCAssignmentValueOnly(currentCCEditValue);
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis(); currentBankSettings.assignableCCs[currentEditingInputIndex] = currentCCEditValue;
                saveCurrentBankSettings();
                currentSetupSubMode = CC_ASSIGN_LIST;
                drawMainScreenPotentiometerLayout(currentBankSettings, true);
                blinkState = true; lastBlinkToggleTime = millis();
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = CC_ASSIGN_LIST;
                drawMainScreenPotentiometerLayout(currentBankSettings, true);
                blinkState = true; lastBlinkToggleTime = millis();
            }
            break;
        }
        case MIDI_CHANNEL_EDIT: {
            int oldChannel = tempMidiChannel;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) { tempMidiChannel++; if (tempMidiChannel > 16) tempMidiChannel = 1; lastButtonPressTime[0] = millis(); }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) { tempMidiChannel--; if (tempMidiChannel < 1) tempMidiChannel = 16; lastButtonPressTime[1] = millis(); }
            if (oldChannel != tempMidiChannel) updateMidiChannelValueOnly(tempMidiChannel);
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                currentBankSettings.midiChannel = tempMidiChannel; saveCurrentBankSettings();
                currentSetupSubMode = MAIN_MENU; menuCursor = lastSelectedMainMenuIndex; drawSetupMenu();
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) { lastButtonPressTime[3] = millis(); currentSetupSubMode = MAIN_MENU; menuCursor = lastSelectedMainMenuIndex; drawSetupMenu(); }
            break;
        }
        case BUTTON_MODE_EDIT: {
            uint8_t oldSubMenuCursor = subMenuCursor;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) { subMenuCursor = (subMenuCursor == 0) ? 1 : 0; lastButtonPressTime[0] = millis(); }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) { subMenuCursor = (subMenuCursor + 1) % 2; lastButtonPressTime[1] = millis(); }
            if (oldSubMenuCursor != subMenuCursor) partialRedrawButtonModeMenu(oldSubMenuCursor, subMenuCursor);
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis(); currentBankSettings.buttonMode = subMenuCursor; saveCurrentBankSettings();
                currentSetupSubMode = MAIN_MENU; menuCursor = lastSelectedMainMenuIndex; drawSetupMenu();
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) { lastButtonPressTime[3] = millis(); currentSetupSubMode = MAIN_MENU; menuCursor = lastSelectedMainMenuIndex; drawSetupMenu(); }
            break;
        }
        case LABEL_ASSIGN_LIST: {
            uint8_t oldMenuCursor = menuCursor;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) {
                clearBlinkingArea(oldMenuCursor, false);
                menuCursor = (menuCursor == 0) ? (TOTAL_ASSIGNABLE_INPUTS - 1) : menuCursor - 1;
                lastButtonPressTime[0] = millis();
            }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) {
                clearBlinkingArea(oldMenuCursor, false);
                menuCursor = (menuCursor + 1) % TOTAL_ASSIGNABLE_INPUTS;
                lastButtonPressTime[1] = millis();
            }

            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis();
                currentEditingLabelIndex = menuCursor; currentLabelCharIndex = 0;
                if (currentEditingLabelIndex < NUM_ANALOG_POTS) strcpy(currentLabelEditBuffer, currentBankSettings.potLabels[currentEditingLabelIndex]);
                else strcpy(currentLabelEditBuffer, currentBankSettings.buttonLabels[currentEditingLabelIndex - NUM_ANALOG_POTS]);
                currentSetupSubMode = LABEL_EDIT_CHAR;
                drawLabelEditMenu(currentEditingLabelIndex, currentLabelEditBuffer, currentLabelCharIndex);
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                clearBlinkingArea(menuCursor, false);
                currentSetupSubMode = MAIN_MENU;
                menuCursor = lastSelectedMainMenuIndex;
                drawSetupMenu();
            }
            break;
        }
        case LABEL_EDIT_CHAR: {
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) {
                int char_idx = findCharIndexInValidSet(currentLabelEditBuffer[currentLabelCharIndex]);
                char_idx = (char_idx + 1) % VALID_CHARS_LEN;
                currentLabelEditBuffer[currentLabelCharIndex] = VALID_LABEL_CHARS[char_idx];
                updateEditedLabelChar(currentLabelCharIndex); lastButtonPressTime[0] = millis();
            }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) {
                int char_idx = findCharIndexInValidSet(currentLabelEditBuffer[currentLabelCharIndex]);
                char_idx = (char_idx == 0) ? (VALID_CHARS_LEN - 1) : char_idx - 1;
                currentLabelEditBuffer[currentLabelCharIndex] = VALID_LABEL_CHARS[char_idx];
                updateEditedLabelChar(currentLabelCharIndex); lastButtonPressTime[1] = millis();
            }
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis();
                if (currentLabelCharIndex < POT_LABEL_LENGTH - 1) {
                    uint8_t oldIndex = currentLabelCharIndex;
                    currentLabelCharIndex++;
                    updateLabelCursor(oldIndex, currentLabelCharIndex);
                    updateLabelEditInstructions(currentLabelCharIndex);
                } else {
                    if (currentEditingLabelIndex < NUM_ANALOG_POTS) strcpy(currentBankSettings.potLabels[currentEditingLabelIndex], currentLabelEditBuffer);
                    else strcpy(currentBankSettings.buttonLabels[currentEditingLabelIndex - NUM_ANALOG_POTS], currentLabelEditBuffer);
                    saveCurrentBankSettings();
                    currentSetupSubMode = LABEL_ASSIGN_LIST;
                    drawMainScreenPotentiometerLayout(currentBankSettings, false);
                    blinkState = true; lastBlinkToggleTime = millis();
                }
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                if (currentLabelCharIndex > 0) {
                    uint8_t oldIndex = currentLabelCharIndex;
                    currentLabelCharIndex--;
                    updateLabelCursor(oldIndex, currentLabelCharIndex);
                    updateLabelEditInstructions(currentLabelCharIndex);
                } else {
                    currentSetupSubMode = LABEL_ASSIGN_LIST;
                    drawMainScreenPotentiometerLayout(currentBankSettings, false);
                    blinkState = true; lastBlinkToggleTime = millis();
                }
            }
            break;
        }
        case ESB_CHANNEL_EDIT: {
            int oldIndex = tempEsbChannelIndex;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) { tempEsbChannelIndex = (tempEsbChannelIndex + 1) % (sizeof(esbOptions) / sizeof(esbOptions[0])); lastButtonPressTime[0] = millis(); }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) { int numOptions = sizeof(esbOptions) / sizeof(esbOptions[0]); tempEsbChannelIndex = (tempEsbChannelIndex == 0) ? numOptions - 1 : tempEsbChannelIndex - 1; lastButtonPressTime[1] = millis(); }
            if (oldIndex != tempEsbChannelIndex) updateESBChannelValueOnly(esbOptions[tempEsbChannelIndex]);
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                globalSettings.esbChannel = esbOptions[tempEsbChannelIndex]; 
                saveGlobalSettings();
                resetRadio(); // Immediately apply the new channel
                currentSetupSubMode = DEVICE_SETUP_MENU;
                menuCursor = 0;
                drawDeviceSetupMenu();
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = DEVICE_SETUP_MENU;
                menuCursor = 0;
                drawDeviceSetupMenu();
            }
            break;
        }
        case CALIBRATION_MAIN: {
            uint8_t oldSubMenuCursor = subMenuCursor;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) { subMenuCursor = (subMenuCursor == 0) ? 3 : subMenuCursor - 1; lastButtonPressTime[0] = millis(); }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) { subMenuCursor = (subMenuCursor + 1) % 4; lastButtonPressTime[1] = millis(); }
            if (oldSubMenuCursor != subMenuCursor) { partialRedrawCalibrationMenu(oldSubMenuCursor, subMenuCursor); }

            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis();

                if (subMenuCursor == 0) {
                    globalSettings.autoCalEnabled = !globalSettings.autoCalEnabled;
                    if (globalSettings.autoCalEnabled) {
                        readVccReference();
                    }
                    saveGlobalSettings();
                    loadSettings();
                    drawCalibrationMenu();
                }
                else if (subMenuCursor == 1) {
                    menuCursor = 0;
                    currentSetupSubMode = EASY_CAL_POT_SELECT; drawEasyCalPotSelectMenu();
                }
                else if (subMenuCursor == 2) {
                    menuCursor = 0;
                    currentSetupSubMode = CALIBRATION_MANUAL_POT_SELECT; drawManualCalPotSelectMenu();
                }
                else if (subMenuCursor == 3) {
                    currentSetupSubMode = FACTORY_CAL_CONFIRM;
                    subMenuCursor = 0;
                    const char* cal_opts[] = {"NO", "Factory Calibrate"};
                    drawSubMenu(F("Apply Factory Cal?"), cal_opts, 0, 2);
                }
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = DEVICE_SETUP_MENU;
                menuCursor = 1;
                drawDeviceSetupMenu();
            }
            break;
        }
        case EASY_CAL_POT_SELECT: {
            uint8_t oldMenuCursor = menuCursor;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) { menuCursor = (menuCursor == 0) ? NUM_ANALOG_POTS - 1 : menuCursor - 1; lastButtonPressTime[0] = millis(); }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) { menuCursor = (menuCursor + 1) % NUM_ANALOG_POTS; lastButtonPressTime[1] = millis(); }
            if (oldMenuCursor != menuCursor) partialRedrawListMenu(oldMenuCursor, menuCursor, 3);
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis(); selectedPotForCal = menuCursor;
                currentSetupSubMode = EASY_CAL_SET_MIN; drawEasyCalPromptMenu(selectedPotForCal, true);
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = CALIBRATION_MAIN;
                subMenuCursor = 1;
                drawCalibrationMenu();
            }
            break;
        }
        case EASY_CAL_SET_MIN: {
            if (millis() - lastDisplayUpdateTime > 100) { updateEasyCalPromptValueOnly(); lastDisplayUpdateTime = millis(); }
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis(); currentBankSettings.potMinRaw[selectedPotForCal] = capturedRawValue;
                currentSetupSubMode = EASY_CAL_SET_MAX; drawEasyCalPromptMenu(selectedPotForCal, false);
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = EASY_CAL_POT_SELECT; menuCursor = selectedPotForCal; drawEasyCalPotSelectMenu();
            }
            break;
        }
        case EASY_CAL_SET_MAX: {
            if (millis() - lastDisplayUpdateTime > 100) { updateEasyCalPromptValueOnly(); lastDisplayUpdateTime = millis(); }
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis(); currentBankSettings.potMaxRaw[selectedPotForCal] = capturedRawValue;
                if (currentBankSettings.potMinRaw[selectedPotForCal] >= currentBankSettings.potMaxRaw[selectedPotForCal]) {
                    int16_t temp = currentBankSettings.potMinRaw[selectedPotForCal];
                    currentBankSettings.potMinRaw[selectedPotForCal] = currentBankSettings.potMaxRaw[selectedPotForCal];
                    currentBankSettings.potMaxRaw[selectedPotForCal] = temp;
                }
                saveCurrentBankSettings();
                currentSetupSubMode = EASY_CAL_POT_SELECT; menuCursor = selectedPotForCal; drawEasyCalPotSelectMenu();
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = EASY_CAL_POT_SELECT; menuCursor = selectedPotForCal; drawEasyCalPotSelectMenu();
            }
            break;
        }
        case CALIBRATION_MANUAL_POT_SELECT: {
            uint8_t oldMenuCursor = menuCursor;
            if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) { menuCursor = (menuCursor == 0) ? NUM_ANALOG_POTS - 1 : menuCursor - 1; lastButtonPressTime[0] = millis(); }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) { menuCursor = (menuCursor + 1) % NUM_ANALOG_POTS; lastButtonPressTime[1] = millis(); }
            if (oldMenuCursor != menuCursor) partialRedrawListMenu(oldMenuCursor, menuCursor, 3);
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis(); selectedPotForCal = menuCursor;
                currentSetupSubMode = CALIBRATION_MANUAL_MIN_MAX_SELECT; subMenuCursor = 0;
                drawManualCalValueSelectMenu(selectedPotForCal);
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = CALIBRATION_MAIN; subMenuCursor = 2; drawCalibrationMenu();
            }
            break;
        }
        case CALIBRATION_MANUAL_MIN_MAX_SELECT: {
            uint8_t oldSubMenuCursor = subMenuCursor;
            if ((up || down) && (millis() - lastButtonPressTime[0] > debounceDelay)) {
                subMenuCursor = (subMenuCursor + 1) % 2;
                partialRedrawManualCalValueSelectMenu(selectedPotForCal, oldSubMenuCursor, subMenuCursor);
                lastButtonPressTime[0]=millis();
            }
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis(); isEditingMinCal = (subMenuCursor == 0);
                tempCalValue = isEditingMinCal ? currentBankSettings.potMinRaw[selectedPotForCal] : currentBankSettings.potMaxRaw[selectedPotForCal];
                calValDigitIndex = 0; currentSetupSubMode = CALIBRATION_MANUAL_VALUE_EDIT;
                forceFooterRedraw = true;
                drawManualCalValueEditMenu(selectedPotForCal, isEditingMinCal, tempCalValue, calValDigitIndex, true);
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = CALIBRATION_MANUAL_POT_SELECT; menuCursor = selectedPotForCal; drawManualCalPotSelectMenu();
            }
            break;
        }
        case CALIBRATION_MANUAL_VALUE_EDIT: {
            const int16_t ADC_HARDWARE_MAX = 16383;
            bool valueChanged = false;

            if (up && (millis() - lastButtonPressTime[0] > 200)) {
                int powerOf10 = 1;
                for (int k = 0; k < (4 - calValDigitIndex); k++) powerOf10 *= 10;
                int16_t new_val = tempCalValue + powerOf10;

                if (new_val > ADC_HARDWARE_MAX) {
                    if (tempCalValue != ADC_HARDWARE_MAX) {
                        tempCalValue = ADC_HARDWARE_MAX;
                        valueChanged = true;
                    }
                } else {
                    if (tempCalValue != new_val) {
                        tempCalValue = new_val;
                        valueChanged = true;
                    }
                }
                lastButtonPressTime[0] = millis();
            }

            if (down && (millis() - lastButtonPressTime[1] > 200)) {
                int powerOf10 = 1;
                for (int k = 0; k < (4 - calValDigitIndex); k++) powerOf10 *= 10;
                int16_t new_val = tempCalValue - powerOf10;

                if (new_val < 0) {
                    if (tempCalValue != 0) {
                        tempCalValue = 0;
                        valueChanged = true;
                    }
                } else {
                    if (tempCalValue != new_val) {
                        tempCalValue = new_val;
                        valueChanged = true;
                    }
                }
                lastButtonPressTime[1] = millis();
            }

            if (valueChanged) {
                int temp_display_val = tempCalValue;
                for (int i = 4; i >= 0; i--) {
                    updateManualCalDigit(i, temp_display_val % 10);
                    temp_display_val /= 10;
                }
                updateManualCalCursor(calValDigitIndex, calValDigitIndex);
            }

            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis();
                uint8_t oldIndex = calValDigitIndex;
                calValDigitIndex++;
                if (calValDigitIndex >= 5) {
                    tempCalValue = constrain(tempCalValue, 0, ADC_HARDWARE_MAX);
                    if (isEditingMinCal) {
                        currentBankSettings.potMinRaw[selectedPotForCal] = tempCalValue;
                        if (currentBankSettings.potMinRaw[selectedPotForCal] > currentBankSettings.potMaxRaw[selectedPotForCal]) {
                            currentBankSettings.potMaxRaw[selectedPotForCal] = currentBankSettings.potMinRaw[selectedPotForCal];
                        }
                    } else {
                        currentBankSettings.potMaxRaw[selectedPotForCal] = tempCalValue;
                        if (currentBankSettings.potMaxRaw[selectedPotForCal] < currentBankSettings.potMinRaw[selectedPotForCal]) {
                            currentBankSettings.potMinRaw[selectedPotForCal] = currentBankSettings.potMaxRaw[selectedPotForCal];
                        }
                    }
                    saveCurrentBankSettings();
                    currentSetupSubMode = CALIBRATION_MANUAL_MIN_MAX_SELECT;
                    forceFooterRedraw = true;
                    drawManualCalValueSelectMenu(selectedPotForCal);
                } else {
                    updateManualCalCursor(oldIndex, calValDigitIndex);
                    updateManualCalFooterInstructions(calValDigitIndex);
                }
            }

            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                if (calValDigitIndex > 0) {
                    uint8_t oldIndex = calValDigitIndex;
                    calValDigitIndex--;
                    updateManualCalCursor(oldIndex, calValDigitIndex);
                    updateManualCalFooterInstructions(calValDigitIndex);
                } else {
                    currentSetupSubMode = CALIBRATION_MANUAL_MIN_MAX_SELECT;
                    subMenuCursor = isEditingMinCal ? 0 : 1;
                    forceFooterRedraw = true;
                    drawManualCalValueSelectMenu(selectedPotForCal);
                }
            }
            break;
        }
        case FACTORY_CAL_CONFIRM: {
            const char* opts[] = {"NO", "Factory Calibrate"};
            uint8_t oldSubMenuCursor = subMenuCursor;
            if ((up || down) && (millis() - lastButtonPressTime[0] > debounceDelay)) {
                subMenuCursor = (subMenuCursor + 1) % 2;
                lastButtonPressTime[0] = millis();
                partialRedrawSubMenu(opts, oldSubMenuCursor, subMenuCursor, 2);
            }
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis();
                if (subMenuCursor == 1) {
                    factoryCalibrateCurrentBank();
                }
                currentSetupSubMode = CALIBRATION_MAIN;
                subMenuCursor = 3;
                drawCalibrationMenu();
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = CALIBRATION_MAIN;
                subMenuCursor = 3;
                drawCalibrationMenu();
            }
            break;
        }
        case FACTORY_RESET_CONFIRM_1: {
            {
                const char* opts[] = {"NO", "YES"};
                uint8_t oldSubMenuCursor = subMenuCursor;
                if ((up || down) && (millis() - lastButtonPressTime[0] > debounceDelay)) {
                    subMenuCursor = (subMenuCursor + 1) % 2;
                    lastButtonPressTime[0] = millis();
                    partialRedrawSubMenu(opts, oldSubMenuCursor, subMenuCursor, 2);
                }
                if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                    lastButtonPressTime[2] = millis();
                    if (subMenuCursor == 1) {
                        currentSetupSubMode = FACTORY_RESET_CONFIRM_2;
                        subMenuCursor = 0;
                        const char* optsFinal[] = {"NO", "Confirm Factory reset"};
                        drawSubMenu(F("Are you sure?"), optsFinal, 0, 2);
                    } else {
                        currentSetupSubMode = DEVICE_SETUP_MENU;
                        menuCursor = 3;
                        drawDeviceSetupMenu();
                    }
                }
                if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                    lastButtonPressTime[3] = millis();
                    currentSetupSubMode = DEVICE_SETUP_MENU;
                    menuCursor = 3;
                    drawDeviceSetupMenu();
                }
            }
            break;
        }
        case FACTORY_RESET_CONFIRM_2: {
            {
                const char* optsFinal[] = {"NO", "Confirm Factory reset"};
                uint8_t oldSubMenuCursor = subMenuCursor;
                if ((up || down) && (millis() - lastButtonPressTime[0] > debounceDelay)) {
                    subMenuCursor = (subMenuCursor + 1) % 2;
                    lastButtonPressTime[0] = millis();
                    partialRedrawSubMenu(optsFinal, oldSubMenuCursor, subMenuCursor, 2);
                }
                if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                    lastButtonPressTime[2] = millis();
                    if (subMenuCursor == 1) {
                        factoryReset();
                    }
                    currentSetupSubMode = DEVICE_SETUP_MENU;
                    menuCursor = 3;
                    drawDeviceSetupMenu();
                }
                if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                    lastButtonPressTime[3] = millis();
                    currentSetupSubMode = DEVICE_SETUP_MENU;
                    menuCursor = 3;
                    drawDeviceSetupMenu();
                }
            }
            break;
        }
        case BRIGHTNESS_EDIT: {
            uint16_t oldStep = tempBrightnessStep;

            if (up && (millis() - lastButtonPressTime[0] > debounceDelay / 8)) {
                if (tempBrightnessStep < 100) tempBrightnessStep++;
                lastButtonPressTime[0] = millis();
            }
            if (down && (millis() - lastButtonPressTime[1] > debounceDelay / 8)) {
                if (tempBrightnessStep > 1) tempBrightnessStep--;
                lastButtonPressTime[1] = millis();
            }

            if (oldStep != tempBrightnessStep) {
                uint16_t pwmValue = map(tempBrightnessStep, 1, 100, BRIGHTNESS_MIN_14BIT, BRIGHTNESS_MAX_14BIT);
                analogWrite(TFT_BL, pwmValue);
                updateBrightnessBar(tempBrightnessStep, false);
            }

            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis();
                globalSettings.brightness = map(tempBrightnessStep, 1, 100, 3, 255);
                saveGlobalSettings();
                currentSetupSubMode = DEVICE_SETUP_MENU;
                menuCursor = 2;
                drawDeviceSetupMenu();
            }

            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();

                fadeStartTime = micros();
                screenState = FADING_REVERT;

                currentSetupSubMode = DEVICE_SETUP_MENU;
                menuCursor = 2;
                drawDeviceSetupMenu();
            }
            break;
        }
        case HARDWARE_TEST: {
            if (millis() - lastDisplayUpdateTime > 100) { updateHardwareTestValues(); lastDisplayUpdateTime = millis(); }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis();
                currentSetupSubMode = DEVICE_SETUP_MENU;
                menuCursor = 4;
                drawDeviceSetupMenu();
            }
            break;
        }
        case POWER_OFF_CONFIRM: {
            uint8_t oldSubMenuCursor = subMenuCursor;
            const char* opts[] = {"NO", "Power Off"};
            if ((up || down) && (millis() - lastButtonPressTime[0] > debounceDelay)) {
                subMenuCursor = (subMenuCursor + 1) % 2;
                lastButtonPressTime[0] = millis();
                partialRedrawSubMenu(opts, oldSubMenuCursor, subMenuCursor, 2);
            }
            if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                lastButtonPressTime[2] = millis();
                if (subMenuCursor == 1) enterSystemOff(NORMAL);
                else {
                    currentSetupSubMode = MAIN_MENU; menuCursor = lastSelectedMainMenuIndex; drawSetupMenu();
                }
            }
            if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                lastButtonPressTime[3] = millis(); currentSetupSubMode = MAIN_MENU; menuCursor = lastSelectedMainMenuIndex; drawSetupMenu();
            }
            break;
        }
        case ESB_CONNECT_MENU: {
            if (foundEsbChannelCount == 0) {
                if (confirm || cancel) {
                    currentSetupSubMode = DEVICE_SETUP_MENU;
                    menuCursor = 5; 
                    drawDeviceSetupMenu();
                }
            } else {
                uint8_t oldMenuCursor = menuCursor;
                if (up && (millis() - lastButtonPressTime[0] > debounceDelay)) {
                    menuCursor = (menuCursor == 0) ? (foundEsbChannelCount - 1) : menuCursor - 1;
                    lastButtonPressTime[0] = millis();
                }
                if (down && (millis() - lastButtonPressTime[1] > debounceDelay)) {
                    menuCursor = (menuCursor + 1) % foundEsbChannelCount;
                    lastButtonPressTime[1] = millis();
                }
                if (oldMenuCursor != menuCursor) {
                    partialRedrawEsbConnectMenu(oldMenuCursor, menuCursor);
                }

                if (confirm && (millis() - lastButtonPressTime[2] > debounceDelay)) {
                    lastButtonPressTime[2] = millis();
                    globalSettings.esbChannel = foundEsbChannels[menuCursor];
                    saveGlobalSettings();
                    resetRadio();

                    // --- FIX START: Manually redraw the main screen after channel selection ---
                    currentAppMode = NORMAL_OPERATION;
                    activeMidiSettings = &currentBankSettings;
                    tft.fillScreen(THEME_BG);
                    drawMainScreenPotentiometerLayout(currentBankSettings, false);
                    redrawAllPotValues(currentBankSettings);
                    forceFullRedraw = true;
                    drawTopStatusBar(TinyUSBDevice.mounted(), false, 0, 0, false);
                    forceFullRedraw = false;
                    currentSetupSubMode = MAIN_MENU;
                    // --- FIX END ---
                }
                if (cancel && (millis() - lastButtonPressTime[3] > debounceDelay)) {
                    lastButtonPressTime[3] = millis();
                    currentSetupSubMode = DEVICE_SETUP_MENU;
                    menuCursor = 5;
                    drawDeviceSetupMenu();
                }
            }
            break;
        }
    }
}

void partialRedrawMainMenu(uint8_t oldCursor, uint8_t newCursor) {
    int16_t y_start = 50;
    int16_t line_height = 22;

    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.7);

    char buffer[30];

    int16_t old_y = y_start + oldCursor * line_height;
    tft.fillRect(0, old_y - 4, SCREEN_WIDTH, line_height - 2, THEME_BG);
    tft.setTextColor(THEME_FG);
    tft.setCursor(5, old_y);
    if (currentSetupSubMode == MAIN_MENU) {
        strcpy_P(buffer, (char*)pgm_read_ptr(&(menuItems[oldCursor])));
        tft.print(buffer);
    } else if (currentSetupSubMode == DEVICE_SETUP_MENU) {
        strcpy_P(buffer, (char*)pgm_read_ptr(&(deviceSetupMenuItems[oldCursor])));
        tft.print(buffer);
    }


    int16_t new_y = y_start + newCursor * line_height;
    tft.fillRect(0, new_y - 4, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
    tft.setTextColor(THEME_BG);
    tft.setCursor(5, new_y);
    if (currentSetupSubMode == MAIN_MENU) {
        strcpy_P(buffer, (char*)pgm_read_ptr(&(menuItems[newCursor])));
        tft.print(buffer);
    } else if (currentSetupSubMode == DEVICE_SETUP_MENU) {
        strcpy_P(buffer, (char*)pgm_read_ptr(&(deviceSetupMenuItems[newCursor])));
        tft.print(buffer);
    }

    tft.endWrite();
}

void partialRedrawListMenu(uint8_t oldCursor, uint8_t newCursor, uint8_t menuType) {
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.7);
    char line[40];
    int16_t y_start = 50;
    int16_t line_height = (menuType == 3) ? 22 : 18;

    int16_t old_y = y_start + oldCursor * line_height;
    tft.fillRect(0, old_y - 3, SCREEN_WIDTH, line_height - 2, THEME_BG);
    tft.setTextColor(THEME_FG);
    tft.setCursor(5, old_y);
    if(menuType == 1) sprintf(line, "%s: %d", getInputName(oldCursor), currentBankSettings.assignableCCs[oldCursor]);
    else if (menuType == 2) {
        char prefix[5];
        if (oldCursor < NUM_ANALOG_POTS) sprintf(prefix, "P%02d", oldCursor + 1);
        else sprintf(prefix, "B%02d", (oldCursor - NUM_ANALOG_POTS) + 1);
        sprintf(line, "%s: %s", prefix, getInputName(oldCursor));
    } else if (menuType == 3) sprintf(line, "P%d Min:%d Max:%d", oldCursor + 1, currentBankSettings.potMinRaw[oldCursor], currentBankSettings.potMaxRaw[oldCursor]);
    tft.print(line);

    int16_t new_y = y_start + newCursor * line_height;
    tft.fillRect(0, new_y - 3, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
    tft.setTextColor(THEME_BG);
    tft.setCursor(5, new_y);
    if(menuType == 1) sprintf(line, "%s: %d", getInputName(newCursor), currentBankSettings.assignableCCs[newCursor]);
    else if(menuType == 2) {
        char prefix[5];
        if (newCursor < NUM_ANALOG_POTS) sprintf(prefix, "P%02d", newCursor + 1);
        else sprintf(prefix, "B%02d", (newCursor - NUM_ANALOG_POTS) + 1);
        sprintf(line, "%s: %s", prefix, getInputName(newCursor));
    } else if (menuType == 3) sprintf(line, "P%d Min:%d Max:%d", newCursor + 1, currentBankSettings.potMinRaw[newCursor], currentBankSettings.potMaxRaw[newCursor]);
    tft.print(line);
    tft.endWrite();
}

void partialRedrawSubMenu(const char* const options[], uint8_t oldCursor, uint8_t newCursor, uint8_t numOptions) {
    if (!tftConnected) return;
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    int16_t y_start = 65;
    int16_t line_height = 30;

    int16_t old_y = y_start + oldCursor * line_height;
    tft.fillRect(0, old_y - 6, SCREEN_WIDTH, line_height - 2, THEME_BG);
    tft.setTextColor(THEME_FG);
    tft.setCursor(15, old_y);
    tft.print(options[oldCursor]);

    int16_t new_y = y_start + newCursor * line_height;
    tft.fillRect(0, new_y - 6, SCREEN_WIDTH, line_height - 2, THEME_ACCENT);
    tft.setTextColor(THEME_BG);
    tft.setCursor(15, new_y);
    tft.print(options[newCursor]);
    tft.endWrite();
}

void partialRedrawButtonModeMenu(uint8_t oldCursor, uint8_t newCursor) {
    if (!tftConnected) return;
    const char* options[] = {"Momentary", "Toggle"};
    tft.startWrite();
    tft.setFont(NULL);
    tft.setTextSize(1.8);
    int16_t y_start = 85;
    int16_t line_height = 40;
    int16_t x1, y1; uint16_t w, h;

    int16_t old_y = y_start + oldCursor * line_height;
    tft.getTextBounds(options[oldCursor], 0, 0, &x1, &y1, &w, &h);
    tft.fillRect(0, old_y - 10, SCREEN_WIDTH, line_height - 4, THEME_BG);
    tft.setTextColor(THEME_FG);
    tft.setCursor((SCREEN_WIDTH - w) / 2, old_y);
    tft.print(options[oldCursor]);

    int16_t new_y = y_start + newCursor * line_height;
    tft.getTextBounds(options[newCursor], 0, 0, &x1, &y1, &w, &h);
    tft.fillRect(0, new_y - 10, SCREEN_WIDTH, line_height - 4, THEME_ACCENT);
    tft.setTextColor(THEME_BG);
    tft.setCursor((SCREEN_WIDTH - w) / 2, new_y);
    tft.print(options[newCursor]);
    tft.endWrite();
}

void sendCC(uint8_t cc, uint8_t value) {
    if (currentAppMode == SETUP_MENU) return;

    uint8_t msg[3] = {uint8_t(0xB0 + (activeMidiSettings->midiChannel - 1)), cc, value};

    if (TinyUSBDevice.mounted()) {
        uint8_t packet[4] = {0x0B, msg[0], msg[1], msg[2]};
        usb_midi.writePacket(packet);
        lastMidiActivityTime = millis();
    } else if (bleRunning) {
        blemidi.write(msg, sizeof(msg));
        lastMidiActivityTime = millis();
    } else {
        resetRadio();
        uint8_t nrf_payload[3] = {msg[0], msg[1], msg[2]};
        nrf.write(nrf_payload, 3);
        lastRadioActivityTime = millis();
        lastMidiActivityTime = millis();
    }

    if (webusb.connected()) {
        webusb.print("cc:" + String(cc) + "," + String(value) + "\n");
        webusb.flush();
    }
}

// --- Modified sendPotCC Function ---
void sendPotCC(uint8_t potIndex, uint8_t value) {
    lastSentCCValue[potIndex] = value;
    value = constrain(value, 0, 127);
    sendCC(activeMidiSettings->assignableCCs[potIndex], value);

    lastMidiActivityTime = millis();
    lastPotActivityTime = millis();
    isAwaitingPostUnplugShutdown = false;

    if (screenState == DIM || screenState == FADING_DOWN) {
        screenState = FADING_UP;
        fadeStartTime = micros();
    }
}

void setup() {
    delay(10);
    usb_midi.begin();
    // WebUSB Initialization
    webusb.setLandingPage(&landingPage);
    webusb.setLineStateCallback(line_state_callback);
    webusb.begin();

    customSPI.begin();
    analogWriteResolution(14);
    analogReadResolution(14);
    analogOversampling(256);
    analogSampleTime(3);
    pinMode(S0_PIN, OUTPUT); pinMode(S1_PIN, OUTPUT); pinMode(S2_PIN, OUTPUT); pinMode(S3_PIN, OUTPUT);
    pinMode(SIG_PIN, INPUT);
    digitalWrite(S0_PIN, LOW); digitalWrite(S1_PIN, LOW); digitalWrite(S2_PIN, LOW); digitalWrite(S3_PIN, LOW);
    analogInputReady = true;

    NRF_POWER->DCDCEN = 1;
    NRF_POWER->TASKS_LOWPWR = 1;
    nrf.begin(); nrf.setPALevel(NRF_PA_MAX);
    nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_POS8DBM);
    nrf.setAutoAck(true); nrf.setRetries(0, 5);
    nrf.setChannel(globalSettings.esbChannel);
    nrf.stopListening(); nrf.powerDown();
    radioActive = false;
    pinMode(LED_BUILTIN, OUTPUT); digitalWrite(LED_BUILTIN, LOW);
    pinMode(D0, INPUT_PULLUP);
    for (uint8_t i = 0; i < BUTTON_COUNT; i++) pinMode(buttonPins[i], INPUT_PULLUP);
    pinMode(EXT_VCC, OUTPUT); digitalWrite(EXT_VCC, HIGH);

    #if NRF_POWER_HAS_RESETREAS
    uint32_t resetReason = nrf_power_resetreas_get(NRF_POWER);
    nrf_power_resetreas_clear(NRF_POWER, resetReason);
    #endif

    lastUSBState = TinyUSBDevice.mounted();

    for(int i = 0; i < NUM_ANALOG_POTS; ++i) { lastRawValueForDisplay[i] = -32768;
    prevSentADCEquivalent[i] = -1.0; }
    for(int i = 0; i < 4; ++i) lastDisplayedCCValueForPot[i] = -1;
    for(int i = 0; i < TOTAL_ASSIGNABLE_INPUTS; ++i) lastSentCCValue[i] = 255;
    lastMidiActivityTime = millis();
    lastPotActivityTime = millis();
    currentAppMode = NORMAL_OPERATION;
    tftConnected = true;

    long rawSum = 0;
    for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) {
        rawSum += BATT_PIN_READ;
        delayMicroseconds(50);
    }
    float adc_buf_avg = (float)rawSum / BATTERY_SAMPLE_COUNT;
    averagedBatteryVoltage = (adc_buf_avg / VBAT_ADC_RES) * VBAT_REF_VOLTAGE * VBAT_GAIN_DIVIDER * VBAT_CORRECTION_FACTOR;
    lastBatterySampleTime = millis();
    analogOversampling(64);

    readVccReference();
    int16_t newlyReadAutoCalMaxValue = globalSettings.autoCalMaxValue;
    loadSettings();

    if (globalSettings.autoCalEnabled) {
        for (int i = 0; i < NUM_ANALOG_POTS; i++) {
            currentBankSettings.potMaxRaw[i] = newlyReadAutoCalMaxValue;
        }
    }
    activeMidiSettings = &currentBankSettings;

    tft.setSPISpeed(32000000);
    tft.init(SCREEN_WIDTH, SCREEN_HEIGHT);
    tft.setRotation(0);
    tft.fillScreen(THEME_BG);
    tft.invertDisplay(1);
    if (!TinyUSBDevice.mounted() && averagedBatteryVoltage < 3.38f) {
        // This initial fade is temporary for the low battery warning, so direct analogWrite is ok.
        uint16_t targetBrightness_14bit = map(globalSettings.brightness, 3, 255, BRIGHTNESS_MIN_14BIT, BRIGHTNESS_MAX_14BIT);
        if (targetBrightness_14bit < BRIGHTNESS_MIN_14BIT) targetBrightness_14bit = BRIGHTNESS_MIN_14BIT;
        unsigned long fadeSetupStartTime = micros();
        while(micros() - fadeSetupStartTime < FADE_DURATION_US) {
            float progress = (float)(micros() - fadeSetupStartTime) / FADE_DURATION_US;
            uint16_t currentBrightness = mapFloat(progress, 0.0, 1.0, 0, targetBrightness_14bit);
            analogWrite(TFT_BL, currentBrightness);
            // Direct write is fine here
        }
        analogWrite(TFT_BL, targetBrightness_14bit);
        // Direct write is fine here
        enterSystemOff(WAKE_LOW_BATTERY);
    }

    drawMainScreenPotentiometerLayout(currentBankSettings, false);
    redrawAllPotValues(currentBankSettings);
    forceFullRedraw = true;
    drawTopStatusBar(TinyUSBDevice.mounted(), false, 0, 0, false);
    forceFullRedraw = false;
    fadeStartTime = micros();
}

void loop() {
    handleWebUSBCommands();
    handleScreenFade();

    unsigned long currentTime = millis();

    // D0 button, USB, and battery logic
    bool d0_isPressed_now = !digitalRead(0);
    bool currentUSB = TinyUSBDevice.mounted();
    if (currentUSB) {
        isAwaitingPostUnplugShutdown = false;
        if (currentTime - lastMidiActivityTime > AUTO_SYSTEM_OFF_TIMEOUT_MS) { wasPluggedInLong = true; }
    }

    if (lastUSBState && !currentUSB) {
        webusb.disconnect(); 
        if (wasPluggedInLong) { isAwaitingPostUnplugShutdown = true; lastMidiActivityTime = currentTime; wasPluggedInLong = false; }
        lastBatterySampleTime = currentTime;
        analogOversampling(256);
        long rawSum = 0;
        for (int i = 0; i < BATTERY_SAMPLE_COUNT ; i++) { rawSum += BATT_PIN_READ; delayMicroseconds(50); }
        float adc_buf_avg = (float)rawSum / BATTERY_SAMPLE_COUNT;
        
        // --- FIX START ---
        // Restored the correct, more robust battery voltage calculation formula.
        averagedBatteryVoltage = (adc_buf_avg / VBAT_ADC_RES) * VBAT_REF_VOLTAGE * VBAT_GAIN_DIVIDER * VBAT_CORRECTION_FACTOR; 
        // --- FIX END ---
        
        analogOversampling(64);
        forceFullRedraw = true;
    }

    if (!d0_wasPressed && d0_isPressed_now) d0_pressTime = currentTime;
    else if (d0_wasPressed && d0_isPressed_now) {
        if (!isBankSwitching && (currentTime - d0_pressTime > BANK_SWITCH_HOLD_MS) && currentAppMode == NORMAL_OPERATION) {
            isBankSwitching = true;
            tempBankSelection = globalSettings.activeBank;
            loadSpecificBankSettings(tempBankSelection, previewBankSettings);
            activeMidiSettings = &previewBankSettings;
            isD13HeldForCCPreview = !digitalRead(NAV_CANCEL_BUTTON);
            lastD13State = isD13HeldForCCPreview;
            drawTopStatusBar(TinyUSBDevice.mounted(), true, tempBankSelection, previewBankSettings.midiChannel, isD13HeldForCCPreview);
            if (isD13HeldForCCPreview) updateMainScreenLabelsOnly(previewBankSettings, true);
        }
    } else if (d0_wasPressed && !d0_isPressed_now) {
        if (isBankSwitching) {
            isBankSwitching = false;
            bool bankActuallyChanged = (globalSettings.activeBank != tempBankSelection);
            bool wasInCCPreview = isD13HeldForCCPreview;
            isD13HeldForCCPreview = false;
            lastD13State = false;
            if (bankActuallyChanged) {
                globalSettings.activeBank = tempBankSelection;
                saveGlobalSettings();
                loadSettings();
                activeMidiSettings = &currentBankSettings;
                if (wasInCCPreview) { updateMainScreenLabelsOnly(currentBankSettings, false); }
                if (webusb.connected()) { webusb.print("bank_changed:" + String(globalSettings.activeBank) + "\n"); webusb.flush(); }
            } else if (wasInCCPreview) {
                activeMidiSettings = &currentBankSettings;
                updateMainScreenLabelsOnly(currentBankSettings, false);
            }
            forceFullRedraw = true;
            drawTopStatusBar(TinyUSBDevice.mounted(), false, 0, 0, false);
            forceFullRedraw = false;
        } else {
            if (currentAppMode == NORMAL_OPERATION) {
                currentAppMode = SETUP_MENU;
                currentSetupSubMode = MAIN_MENU;
                menuCursor = lastSelectedMainMenuIndex;
                drawSetupMenu();
                setupStartTime = currentTime;
                return;
            }
        }
    }
    d0_wasPressed = d0_isPressed_now;
    if (!TinyUSBDevice.mounted()) {
        unsigned long currentTimeout = isAwaitingPostUnplugShutdown ? POST_UNPLUG_ALIVE_MS : AUTO_SYSTEM_OFF_TIMEOUT_MS;
        if (currentTime - lastMidiActivityTime > currentTimeout) { enterSystemOff(NORMAL); }
    }
    if (currentAppMode == NORMAL_OPERATION && screenState == BRIGHT && (currentTime - lastPotActivityTime > DIM_INACTIVITY_TIMEOUT_MS)) {
        if (globalSettings.brightness > DIM_BRIGHTNESS) {
            screenState = FADING_DOWN;
            brightnessBeforeDim = globalSettings.brightness;
            fadeStartTime = micros();
        }
    }
    if (currentAppMode == NORMAL_OPERATION && !isBankSwitching) {
        if (currentTime - lastBatterySampleTime > BATTERY_UPDATE_INTERVAL_MS || currentUSB != lastUSBState || forceFullRedraw) {
            analogOversampling(256);
            long rawSum = 0;
            for (int i = 0; i < BATTERY_SAMPLE_COUNT; i++) { rawSum += BATT_PIN_READ; delayMicroseconds(50); }
            float adc_buf_avg = (float)rawSum / BATTERY_SAMPLE_COUNT;

            // --- FIX START ---
            // Restored the correct, more robust battery voltage calculation formula here as well.
            averagedBatteryVoltage = (adc_buf_avg / VBAT_ADC_RES) * VBAT_REF_VOLTAGE * VBAT_GAIN_DIVIDER * VBAT_CORRECTION_FACTOR;
            // --- FIX END ---
            
            lastBatterySampleTime = currentTime;
            analogOversampling(64);
            if (!currentUSB && averagedBatteryVoltage < 3.38f) { enterSystemOff(LOW_BATTERY_SHUTDOWN); }
        }
        updateNormalStatusBar(forceFullRedraw);
        forceFullRedraw = false;
    }
    lastUSBState = currentUSB;
    if (bleRunning && Bluefruit.connected() && (currentTime - lastBleBatteryUpdateTime > BATTERY_UPDATE_INTERVAL_MS)) {
        lastBleBatteryUpdateTime = currentTime;
        int batteryPercent = (int)roundf(((averagedBatteryVoltage - 3.4f) / (4.15f - 3.4f)) * 100.0f);
        batteryPercent = constrain(batteryPercent, 0, 100);
        blebas.notify(batteryPercent);
    }

    switch (currentAppMode) {
        case NORMAL_OPERATION: {
            if (isBankSwitching) {
                uint8_t oldTempBankSelection = tempBankSelection;
                bool up_nav = !digitalRead(NAV_DOWN_BUTTON), down_nav = !digitalRead(NAV_UP_BUTTON), d13_current_state = !digitalRead(NAV_CANCEL_BUTTON);
                static unsigned long lastBankSwitchButtonTime = 0;
                if (currentTime - lastBankSwitchButtonTime > 200) {
                    if (down_nav) { tempBankSelection = (tempBankSelection + 1) % NUM_BANKS;
                    lastBankSwitchButtonTime = currentTime; }
                    if (up_nav) { tempBankSelection = (tempBankSelection == 0) ?
                    NUM_BANKS - 1 : tempBankSelection - 1; lastBankSwitchButtonTime = currentTime;
                    }
                }
                if (d13_current_state != lastD13State) {
                    isD13HeldForCCPreview = d13_current_state;
                    lastD13State = d13_current_state;
                    updateMainScreenLabelsOnly(previewBankSettings, isD13HeldForCCPreview);
                    drawTopStatusBar(TinyUSBDevice.mounted(), true, tempBankSelection, previewBankSettings.midiChannel, isD13HeldForCCPreview);
                }
                if (oldTempBankSelection != tempBankSelection) {
                    BankSettings previousPreviewSettings = previewBankSettings;
                    loadSpecificBankSettings(tempBankSelection, previewBankSettings);
                    bool contentChanged = false;
                    if (isD13HeldForCCPreview) {
                        for(int i=0; i<TOTAL_ASSIGNABLE_INPUTS; i++) if(previousPreviewSettings.assignableCCs[i] != previewBankSettings.assignableCCs[i]) { contentChanged = true;
                        break; }
                    } else {
                        for(int i=0; i<NUM_ANALOG_POTS; i++) if(strcmp(previousPreviewSettings.potLabels[i], previewBankSettings.potLabels[i]) != 0) { contentChanged = true;
                        break; }
                        if (!contentChanged) for(int i=0; i<NUM_DIGITAL_MIDI_BUTTONS; i++) if(strcmp(previousPreviewSettings.buttonLabels[i], previewBankSettings.buttonLabels[i]) != 0) { contentChanged = true;
                        break; }
                    }
                    if(contentChanged) updateMainScreenLabelsOnly(previewBankSettings, isD13HeldForCCPreview);
                    drawTopStatusBar_BankSwitchPreview(tempBankSelection, previewBankSettings.midiChannel, isD13HeldForCCPreview);
                }
            }

            if (analogInputReady) {
                int16_t currentRawPotValues[NUM_ANALOG_POTS];
                for (int i = 0; i < NUM_ANALOG_POTS; i++) { selectChannel(i); delayMicroseconds(5); currentRawPotValues[i] = readMedianADC(SIG_PIN);
                }

                for (uint8_t i = 0; i < NUM_ANALOG_POTS; i++) {
                    uint8_t current_mapped_val;
                    int16_t minRaw = activeMidiSettings->potMinRaw[i];
                    int16_t maxRaw = activeMidiSettings->potMaxRaw[i];
                    int16_t rawValue = currentRawPotValues[i];
                    if (rawValue >= maxRaw - SNAP_THRESHOLD_RAW) { current_mapped_val = 127;
                    }
                    else if (rawValue <= minRaw + SNAP_THRESHOLD_RAW) { current_mapped_val = 0;
                    }
                    else { current_mapped_val = map(rawValue, minRaw, maxRaw, 0, 128);
                    }
                    current_mapped_val = constrain(current_mapped_val, 0, 127);
                    bool shouldSend = false;
                    if (lastSentCCValue[i] == 255 || prevSentADCEquivalent[i] < 0) { shouldSend = true;
                    }
                    else {
                        float hysteresisWidth = (16383 * ADC_HYSTERESIS_PERCENT) / 100.0f;
                        float threshold = hysteresisWidth / 2.0f;
                        float rawDifference = rawValue - prevSentADCEquivalent[i];
                        if (abs(rawDifference) > threshold) {
                            if (current_mapped_val != lastSentCCValue[i]) { shouldSend = true;
                            }
                        }
                    }
                    if (shouldSend) {
                        sendPotCC(i, current_mapped_val);
                        prevSentADCEquivalent[i] = rawValue;
                        updatePotVisual(i, current_mapped_val, *activeMidiSettings);
                    }
                }
            }

            for (uint8_t i = 0; i < BUTTON_COUNT; i++) {
                bool state = !digitalRead(buttonPins[i]);
                uint8_t buttonIndex = i, assignable_cc_array_index = NUM_ANALOG_POTS + buttonIndex;
                if (activeMidiSettings->buttonMode == 0) {
                    if (state != lastButtonState[i]) {
                        if (!(isBankSwitching && isD13HeldForCCPreview)) {
                            drawButtonPressIndicator(buttonIndex, state ? THEME_INDICATOR_ON : THEME_INDICATOR_OFF);
                        }
                        uint8_t value_to_send = state ?
                        127 : 0;
                        if (lastSentCCValue[assignable_cc_array_index] != value_to_send) {
                            lastSentCCValue[assignable_cc_array_index] = value_to_send;
                            sendCC(activeMidiSettings->assignableCCs[assignable_cc_array_index], value_to_send);

                            lastPotActivityTime = millis();
                            isAwaitingPostUnplugShutdown = false;
                            if (screenState == DIM || screenState == FADING_DOWN) {
                                screenState = FADING_UP;
                                fadeStartTime = micros();
                            }
                        }
                    }
                } else {
                    if (state && !lastButtonState[i]) {
           
                             buttonToggles[i] = !buttonToggles[i];
                        if (!(isBankSwitching && isD13HeldForCCPreview)) {
                            drawButtonPressIndicator(buttonIndex, buttonToggles[i] ? THEME_INDICATOR_ON : THEME_INDICATOR_OFF);
                        }
                        uint8_t value_to_send = buttonToggles[i] ?
                        127 : 0;
                        if (lastSentCCValue[assignable_cc_array_index] != value_to_send) {
                            lastSentCCValue[assignable_cc_array_index] = value_to_send;
                            sendCC(activeMidiSettings->assignableCCs[assignable_cc_array_index], value_to_send);

                            lastPotActivityTime = millis();
                            isAwaitingPostUnplugShutdown = false;
                            if (screenState == DIM || screenState == FADING_DOWN) {
                                screenState = FADING_UP;
                                fadeStartTime = micros();
                            }
                        }
                    }
                }
                lastButtonState[i] = state;
            }
            if (!TinyUSBDevice.mounted()) idleRadioCheck();
        } break;

        case SETUP_MENU: {
            handleSetupInput();
            if (currentSetupSubMode == CC_ASSIGN_LIST || currentSetupSubMode == LABEL_ASSIGN_LIST) {
                blinkAssignedInputLabelAndIndicator();
            }
        } break;

        case ESB_SWEEP: {
            performEsbSweep();
        } break;
    }
}
