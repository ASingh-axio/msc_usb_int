/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_hidh.h"
#include "esp_hid_gap.h"


//Added libraries of ble above this comment


#include "dirent.h"
#include "esp_log.h"
#include "esp_err.h"
//#include "esp_vfs.h"
#include <sys/stat.h>
#include "sdkconfig.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_partition.h"
#include "freertos/semphr.h"


//Added libraries of display above this comment


#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "OTA.h"
//Added OTA libraries above this comment

#include <stdbool.h>
#include <unistd.h>
#include "freertos/event_groups.h"
#include "esp_err.h"
#include "esp_log.h"
#include "usb/usb_host.h"
#include "errno.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/adc.h"
#include "hid_host.h"
#include "hid_usage_keyboard.h"


#define EXAMPLE_ESP_WIFI_SSID               "Axio Electronics"
#define EXAMPLE_ESP_WIFI_PASS               "axio2929"
#define EXAMPLE_ESP_MAXIMUM_RETRY           5

#define BATT_STAT_OP_PIN            GPIO_NUM_48
#define BATT_STAT_IP_PIN            GPIO_NUM_1
#define MAX_BATTERY_VOLTAGE 4.2  // Maximum battery voltage (100%)
#define MIN_BATTERY_VOLTAGE 3.0  // Minimum battery voltage (0%)
#define ADC_INPUT_CHANNEL           ADC1_CHANNEL_0
#define APP_QUIT_PIN                GPIO_NUM_0
#define APP_QUIT_PIN_POLL_MS        500

#define READY_TO_UNINSTALL          (HOST_NO_CLIENT | HOST_ALL_FREE)

/* Main char symbol for ENTER key */
#define KEYBOARD_ENTER_MAIN_CHAR    '\r'
/* When set to 1 pressing ENTER will be extending with LineFeed during serial debug output */
#define KEYBOARD_ENTER_LF_EXTEND    1

#define SCAN_DURATION_SECONDS 5

#define MAX_SCAN_INPUT_LENGTH 128


//defines of display begin

#define LCM_CS        21
#define LCM_RESET     47
#define D7            4
#define D6            5
#define D5            6
#define D4            7
#define D3            15
#define D2            16
#define D1            17
#define D0            18
#define LCM_ENABLE    3
#define LCM_A0        10

#define downBtn     11
#define selectBtn   14
#define upBtn       13
#define BLBtn       12
#define BL_PWM      2
#define POWER_LED   38
#define POWER_OFF   41
#define powerOffBtn 40

#define IDLE_DELAY  3600*1000*1000 // in microseconds

//defines of display end


//variable declarations display start

int btntmr = 0, mainMenuCursor = 0, fontID = 0, BLLevel = 0, duty = 0;
int start, end, totalLen, currPos, currRow, currCol, currFileID, chars = 100;
int MAX_CHARS_IN_A_LINE = 39, MAX_LINES = 8, PIXELS_IN_A_LINE = 8;
long FSstats[] = {0, 0, 0, 0}; // File Count, Occupied Space, Free Space, Word Count
bool syncStatus = false, wifiStatus = false, btStatus = false, btWaitStatus = false;
bool charInput = false, WMcharInput = false, powerOffStatus = false;
bool gDriveStatus = false, oneDriveStatus = false, dropBoxStatus = false;
char gDriveUsername[38]   = "", gDrivePwd[38]   = "";
char oneDriveUsername[38] = "", oneDrivePwd[38] = "";
char dropBoxUsername[38]  = "", dropBoxPwd[38]  = "";
char files[500][30], *buffer = NULL, btDevName[36];
time_t filesModTime[500];
static esp_timer_handle_t periodic_timer;
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
static int64_t press_start_time = 0;
static volatile bool button_pressed = false;
uint8_t font5x7[240][5] = {
	{ 0x00, 0x7F, 0x3E, 0x1C, 0x08 },  // 10  16
	{ 0x08, 0x1C, 0x3E, 0x7F, 0x00 },  // 11  17
	{ 0x44, 0x66, 0x77, 0x66, 0x44 },  // 12  18
	{ 0x11, 0x33, 0x77, 0x33, 0x11 },  // 13  19
	{ 0x08, 0x14, 0x2A, 0x14, 0x22 },  // 14  20
	{ 0x22, 0x14, 0x2A, 0x14, 0x08 },  // 15  21
	{ 0x07, 0x03, 0x05, 0x08, 0x10 },  // 16  22
	{ 0x10, 0x08, 0x05, 0x03, 0x07 },  // 17  23
	{ 0x70, 0x60, 0x50, 0x08, 0x04 },  // 18  24
	{ 0x04, 0x08, 0x50, 0x60, 0x70 },  // 19  25
	{ 0x20, 0x38, 0x3E, 0x38, 0x20 },  // 1A  26
	{ 0x02, 0x0E, 0x3E, 0x0E, 0x02 },  // 1B  27
	{ 0x10, 0x38, 0x54, 0x10, 0x1F },  // 1C  28
	{ 0x04, 0x02, 0x01, 0x02, 0x04 },  // 1D  29
	{ 0x01, 0x02, 0x04, 0x02, 0x01 },  // 1E  30
	{ 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },  // 1F  31
	{ 0x00, 0x00, 0x00, 0x00, 0x00 },  // 20  32
	{ 0x00, 0x00, 0x5F, 0x00, 0x00 },  // 21  33  !
	{ 0x00, 0x07, 0x00, 0x07, 0x00 },  // 22  34  "
	{ 0x14, 0x7F, 0x14, 0x7F, 0x14 },  // 23  35  #
	{ 0x00, 0x26, 0x6B, 0x32, 0x00 },  // 24  36  $
	{ 0x23, 0x13, 0x08, 0x64, 0x62 },  // 25  37  %
	{ 0x36, 0x49, 0x55, 0x22, 0x50 },  // 26  38  &
	{ 0x00, 0x05, 0x03, 0x00, 0x00 },  // 27  39  '
	{ 0x00, 0x1C, 0x22, 0x41, 0x00 },  // 28  40  (
	{ 0x00, 0x41, 0x22, 0x1C, 0x00 },  // 29  41  )
	{ 0x0A, 0x04, 0x1F, 0x04, 0x0A },  // 2A  42  *
	{ 0x08, 0x08, 0x3E, 0x08, 0x08 },  // 2B  43  +
	{ 0x00, 0x50, 0x30, 0x00, 0x00 },  // 2C  44  ,
	{ 0x08, 0x08, 0x08, 0x08, 0x08 },  // 2D  45  -
	{ 0x00, 0x60, 0x60, 0x00, 0x00 },  // 2E  46  .
	{ 0x40, 0x30, 0x08, 0x06, 0x01 },  // 2F  47  /
	{ 0x3E, 0x51, 0x49, 0x45, 0x3E },  // 30  48  0
	{ 0x00, 0x42, 0x7F, 0x40, 0x00 },  // 31  49  1
	{ 0x42, 0x61, 0x51, 0x49, 0x46 },  // 32  50  2
	{ 0x21, 0x41, 0x45, 0x4B, 0x31 },  // 33  51  3
	{ 0x18, 0x14, 0x12, 0x7F, 0x10 },  // 34  52  4
	{ 0x27, 0x45, 0x45, 0x45, 0x39 },  // 35  53  5
	{ 0x3C, 0x4A, 0x49, 0x49, 0x30 },  // 36  54  6
	{ 0x01, 0x71, 0x09, 0x05, 0x03 },  // 37  55  7
	{ 0x36, 0x49, 0x49, 0x49, 0x36 },  // 38  56  8
	{ 0x06, 0x49, 0x49, 0x29, 0x1E },  // 39  57  9
	{ 0x00, 0x36, 0x36, 0x00, 0x00 },  // 3A  58  :
	{ 0x00, 0x56, 0x36, 0x00, 0x00 },  // 3B  59  ;
	{ 0x08, 0x14, 0x22, 0x41, 0x00 },  // 3C  60  <
	{ 0x14, 0x14, 0x14, 0x14, 0x14 },  // 3D  61  =
	{ 0x00, 0x41, 0x22, 0x14, 0x08 },  // 3E  62  >
	{ 0x02, 0x01, 0x51, 0x09, 0x06 },  // 3F  63  ?
	{ 0x00, 0x3C, 0x42, 0x5A, 0x5E },  // 40  64  @
	{ 0x7E, 0x11, 0x11, 0x11, 0x7E },  // 41  65  A
	{ 0x7F, 0x49, 0x49, 0x49, 0x36 },  // 42  66  B
	{ 0x3E, 0x41, 0x41, 0x41, 0x22 },  // 43  67  C
	{ 0x7F, 0x41, 0x41, 0x22, 0x1C },  // 44  68  D
	{ 0x7F, 0x49, 0x49, 0x49, 0x41 },  // 45  69  E
	{ 0x7F, 0x09, 0x09, 0x09, 0x01 },  // 46  70  F
	{ 0x3E, 0x41, 0x49, 0x49, 0x7A },  // 47  71  G
	{ 0x7F, 0x08, 0x08, 0x08, 0x7F },  // 48  72  H
	{ 0x00, 0x41, 0x7F, 0x41, 0x00 },  // 49  73  I
	{ 0x20, 0x40, 0x41, 0x3F, 0x01 },  // 4A  74  J
	{ 0x7F, 0x08, 0x14, 0x22, 0x41 },  // 4B  75  K
	{ 0x7F, 0x40, 0x40, 0x40, 0x40 },  // 4C  76  L
	{ 0x7F, 0x02, 0x0C, 0x02, 0x7F },  // 4D  77  M
	{ 0x7F, 0x04, 0x08, 0x10, 0x7F },  // 4E  78  N
	{ 0x3E, 0x41, 0x41, 0x41, 0x3E },  // 4F  79  O
	{ 0x7F, 0x09, 0x09, 0x09, 0x06 },  // 50  80  P
	{ 0x3E, 0x41, 0x51, 0x21, 0x5E },  // 51  81  Q
	{ 0x7F, 0x09, 0x19, 0x29, 0x46 },  // 52  82  R
	{ 0x46, 0x49, 0x49, 0x49, 0x31 },  // 53  83  S
	{ 0x01, 0x01, 0x7F, 0x01, 0x01 },  // 54  84  T
	{ 0x3F, 0x40, 0x40, 0x40, 0x3F },  // 55  85  U
	{ 0x1F, 0x20, 0x40, 0x20, 0x1F },  // 56  86  V
	{ 0x3F, 0x40, 0x38, 0x40, 0x3F },  // 57  87  W
	{ 0x63, 0x14, 0x08, 0x14, 0x63 },  // 58  88  X
	{ 0x07, 0x08, 0x70, 0x08, 0x07 },  // 59  89  Y
	{ 0x61, 0x59, 0x49, 0x4D, 0x43 },  // 5A  90  Z
	{ 0x00, 0x7F, 0x41, 0x00, 0x00 },  // 5B  91  [
	{ 0x01, 0x06, 0x08, 0x30, 0x40 },  // 5C  92  '\'
	{ 0x00, 0x00, 0x41, 0x7F, 0x00 },  // 5D  93  ]
	{ 0x04, 0x02, 0x01, 0x02, 0x04 },  // 5E  94  ^
	{ 0x40, 0x40, 0x40, 0x40, 0x40 },  // 5F  95  _
	{ 0x00, 0x01, 0x02, 0x04, 0x00 },  // 60  96  `
	{ 0x20, 0x54, 0x54, 0x54, 0x78 },  // 61  97  a
	{ 0x7F, 0x48, 0x44, 0x44, 0x38 },  // 62  98  b
	{ 0x38, 0x44, 0x44, 0x44, 0x20 },  // 63  99  c
	{ 0x38, 0x44, 0x44, 0x48, 0x7F },  // 64 100  d
	{ 0x38, 0x54, 0x54, 0x54, 0x18 },  // 65 101  e
	{ 0x08, 0x7E, 0x09, 0x01, 0x02 },  // 66 102  f
	{ 0x0C, 0x52, 0x52, 0x52, 0x3E },  // 67 103  g
	{ 0x7F, 0x08, 0x04, 0x04, 0x78 },  // 68 104  h
	{ 0x00, 0x44, 0x7D, 0x40, 0x00 },  // 69 105  i
	{ 0x20, 0x40, 0x44, 0x3D, 0x00 },  // 6A 106  j
	{ 0x7F, 0x10, 0x28, 0x44, 0x00 },  // 6B 107  k
	{ 0x00, 0x41, 0x7F, 0x40, 0x00 },  // 6C 108  l
	{ 0x78, 0x04, 0x78, 0x04, 0x78 },  // 6D 109  m
	{ 0x7C, 0x08, 0x04, 0x04, 0x78 },  // 6E 110  n
	{ 0x38, 0x44, 0x44, 0x44, 0x38 },  // 6F 111  o
	{ 0x7C, 0x14, 0x14, 0x14, 0x08 },  // 70 112  p
	{ 0x08, 0x14, 0x14, 0x7C, 0x40 },  // 71 113  q
	{ 0x7C, 0x08, 0x04, 0x04, 0x08 },  // 72 114  r
	{ 0x48, 0x54, 0x54, 0x54, 0x20 },  // 73 115  s
	{ 0x04, 0x3F, 0x44, 0x40, 0x20 },  // 74 116  t
	{ 0x3C, 0x40, 0x40, 0x20, 0x7C },  // 75 117  u
	{ 0x1C, 0x20, 0x40, 0x20, 0x1C },  // 76 118  v
	{ 0x3C, 0x40, 0x30, 0x40, 0x3C },  // 77 119  w
	{ 0x44, 0x28, 0x10, 0x28, 0x44 },  // 78 120  x
	{ 0x0C, 0x50, 0x50, 0x50, 0x3C },  // 79 121  y
	{ 0x44, 0x64, 0x54, 0x4C, 0x44 },  // 7A 122  z
	{ 0x00, 0x08, 0x3E, 0x41, 0x00 },  // 7B 123  {
	{ 0x00, 0x00, 0x7F, 0x00, 0x00 },  // 7C 124  |
	{ 0x00, 0x41, 0x3E, 0x08, 0x00 },  // 7D 125  }
	{ 0x0C, 0x02, 0x04, 0x08, 0x06 },  // 7E 126  ~
	{ 0x20, 0x55, 0x56, 0x54, 0x78 },  // 7F 127
	{ 0x0E, 0x11, 0x11, 0x0E, 0x00 },  // 80 128
	{ 0x12, 0x1F, 0x10, 0x00, 0x00 },  // 81 129
	{ 0x12, 0x19, 0x15, 0x12, 0x00 },  // 82 130
	{ 0x11, 0x15, 0x15, 0x0A, 0x00 },  // 83 131
	{ 0x0C, 0x0A, 0x1F, 0x08, 0x00 },  // 84 132
	{ 0x17, 0x15, 0x15, 0x09, 0x00 },  // 85 133
	{ 0x0E, 0x15, 0x15, 0x09, 0x00 },  // 86 134
	{ 0x01, 0x19, 0x05, 0x03, 0x00 },  // 87 135
	{ 0x0A, 0x15, 0x15, 0x0A, 0x00 },  // 88 136
	{ 0x02, 0x15, 0x15, 0x0E, 0x00 },  // 89 137
	{ 0x17, 0x08, 0x94, 0xCA, 0xB1 },  // 8A 138
	{ 0x17, 0x68, 0x54, 0xFA, 0x41 },  // 8B 139
	{ 0x44, 0x44, 0x5F, 0x44, 0x44 },  // 8C 140
	{ 0x40, 0x51, 0x4A, 0x44, 0x40 },  // 8D 141
	{ 0x40, 0x44, 0x4A, 0x51, 0x40 },  // 8E 142
	{ 0x7F, 0x10, 0x10, 0x08, 0x1F },  // 8F 143
	{ 0xC0, 0xC0, 0xFE, 0x04, 0x18 },  // 90 144
	{ 0x60, 0x60, 0x3F, 0xC5, 0xFE },  // 91 145
	{ 0x20, 0x3E, 0x7F, 0x3E, 0x20 },  // 92 146
	{ 0x0C, 0x1E, 0x3C, 0x1E, 0x0C },  // 93 147
	{ 0x08, 0x1C, 0x3E, 0x1C, 0x08 },  // 94 148
	{ 0x01, 0x03, 0x7F, 0x03, 0x01 },  // 95 149
	{ 0x07, 0x01, 0x01, 0x00, 0x00 },  // 96 150
	{ 0x00, 0x00, 0x40, 0x40, 0x70 },  // 97 151
	{ 0x06, 0x05, 0x00, 0x06, 0x05 },  // 98 152
	{ 0x05, 0x03, 0x00, 0x05, 0x03 },  // 99 153
	{ 0x00, 0x1C, 0x22, 0x41, 0x00 },  // 9A 154
	{ 0x00, 0x41, 0x22, 0x1C, 0x00 },  // 9B 155
	{ 0x38, 0x44, 0x48, 0x30, 0x4C },  // 9C 156
	{ 0x28, 0x54, 0x54, 0x44, 0x20 },  // 9D 157
	{ 0x30, 0x4A, 0x45, 0x49, 0x32 },  // 9E 158
	{ 0x18, 0x14, 0x08, 0x14, 0x0C },  // 9F 159
	{ 0x3E, 0x41, 0x59, 0x55, 0x1E },  // A0 160
	{ 0x48, 0x3E, 0x49, 0x41, 0x20 },  // A1 161
	{ 0x24, 0x2A, 0x7F, 0x2A, 0x12 },  // A2 162
	{ 0x15, 0x16, 0x7C, 0x16, 0x15 },  // A3 163
	{ 0x38, 0x55, 0x56, 0x54, 0x18 },  // A4 164
	{ 0x38, 0x54, 0x56, 0x55, 0x18 },  // A5 165
	{ 0x3C, 0x41, 0x42, 0x20, 0x7C },  // A6 166
	{ 0x00, 0x49, 0x7A, 0x40, 0x00 },  // A7 167
	{ 0x38, 0x45, 0x46, 0x44, 0x38 },  // A8 168
	{ 0x0E, 0x51, 0x71, 0x11, 0x12 },  // A9 169
	{ 0x0F, 0x05, 0x05, 0x02, 0x00 },  // AA 170
	{ 0x3E, 0x61, 0x5D, 0x43, 0x3E },  // AB 171
	{ 0x38, 0x64, 0x54, 0x4C, 0x38 },  // AC 172
	{ 0x0F, 0x02, 0x01, 0x02, 0x00 },  // AD 173
	{ 0x78, 0x14, 0x15, 0x14, 0x78 },  // AE 174
	{ 0x20, 0x54, 0x55, 0x54, 0x78 },  // AF 175
	{ 0x38, 0x24, 0x22, 0x24, 0x38 },  // B0 176
	{ 0x1C, 0x22, 0x7F, 0x22, 0x10 },  // B1 177
	{ 0x08, 0x55, 0x7F, 0x55, 0x08 },  // B2 178
	{ 0x04, 0x04, 0x7C, 0x44, 0x04 },  // B3 179
	{ 0x41, 0x32, 0x0C, 0x30, 0x40 },  // B4 180
	{ 0x5C, 0x62, 0x02, 0x62, 0x5C },  // B5 181
	{ 0x04, 0x7C, 0x04, 0x7C, 0x04 },  // B6 182
	{ 0x07, 0x08, 0x7F, 0x08, 0x07 },  // B7 183
	{ 0x63, 0x55, 0x49, 0x41, 0x41 },  // B8 184
	{ 0x1C, 0x2A, 0x49, 0x2A, 0x1C },  // B9 185
	{ 0x22, 0x2A, 0x2A, 0x2A, 0x22 },  // BA 186
	{ 0x1C, 0x3E, 0x3E, 0x3E, 0x1C },  // BB 187
	{ 0x7C, 0x0A, 0x09, 0x7F, 0x49 },  // BC 188
	{ 0x64, 0x54, 0x78, 0x54, 0x58 },  // BD 189
	{ 0x7E, 0x15, 0x25, 0x25, 0x1A },  // BE 190
	{ 0x7C, 0x54, 0x56, 0x45, 0x00 },  // BF 191
	{ 0x7F, 0x01, 0x01, 0x01, 0x03 },  // C0 192
	{ 0x70, 0x0C, 0x03, 0x0C, 0x70 },  // C1 193
	{ 0x7F, 0x01, 0x01, 0x01, 0x7F },  // C2 194
	{ 0x06, 0x01, 0x7E, 0x01, 0x06 },  // C3 195
	{ 0x40, 0x40, 0x40, 0x40, 0x40 },  // C4 196
	{ 0x7C, 0x55, 0x56, 0x44, 0x00 },  // C5 197
	{ 0x7C, 0x56, 0x55, 0x46, 0x00 },  // C6 198
	{ 0x38, 0x56, 0x55, 0x56, 0x18 },  // C7 199
	{ 0x0C, 0x52, 0x72, 0x12, 0x00 },  // C8 200
	{ 0x08, 0x55, 0x56, 0x55, 0x3C },  // C9 201
	{ 0x22, 0x25, 0x65, 0x25, 0x19 },  // CA 202
	{ 0x24, 0x2A, 0x6A, 0x2A, 0x10 },  // CB 203
	{ 0x00, 0x44, 0x7D, 0x44, 0x00 },  // CC 204
	{ 0x00, 0x44, 0x7C, 0x40, 0x00 },  // CD 205
	{ 0x04, 0x02, 0x04, 0x08, 0x04 },  // CE 206
	{ 0x08, 0x14, 0x22, 0x14, 0x08 },  // CF 207
	{ 0x3E, 0x3E, 0x3E, 0x3E, 0x3E },  // D0 208
	{ 0x3E, 0x3E, 0x3E, 0x3E, 0x00 },  // D1 209
	{ 0x3E, 0x3E, 0x3E, 0x00, 0x00 },  // D2 210
	{ 0x3E, 0x3E, 0x00, 0x00, 0x00 },  // D3 211
	{ 0x3E, 0x00, 0x00, 0x00, 0x00 },  // D4 212
	{ 0x28, 0x48, 0x3E, 0x09, 0x0A },  // D5 213
	{ 0x7F, 0x7F, 0x7F, 0x7F, 0x7F },  // D6 214
	{ 0x7F, 0x7F, 0x7F, 0x7F, 0x00 },  // D7 215
	{ 0x7F, 0x7F, 0x7F, 0x00, 0x00 },  // D8 216
	{ 0x7F, 0x7F, 0x00, 0x00, 0x00 },  // D9 217
	{ 0x7F, 0x00, 0x00, 0x00, 0x00 },  // DA 218
	{ 0x7F, 0x05, 0x25, 0xF2, 0xA0 },  // DB 219
	{ 0x3C, 0x24, 0x24, 0x3C, 0x00 },  // DC 220
	{ 0x00, 0x18, 0x18, 0x00, 0x00 },  // DD 221
	{ 0x04, 0x02, 0x7F, 0x02, 0x04 },  // DE 222
	{ 0x08, 0x08, 0x2A, 0x1C, 0x08 },  // DF 223
	{ 0x10, 0x20, 0x7F, 0x20, 0x10 },  // E0 224
	{ 0x08, 0x1C, 0x2A, 0x08, 0x08 },  // E1 225
	{ 0x78, 0x14, 0x16, 0x15, 0x78 },  // E2 226
	{ 0x00, 0x44, 0x7E, 0x45, 0x00 },  // E3 227
	{ 0x38, 0x44, 0x46, 0x45, 0x38 },  // E4 228
	{ 0x3C, 0x40, 0x42, 0x41, 0x3C },  // E5 229
	{ 0x04, 0x08, 0x72, 0x09, 0x04 },  // E6 230
	{ 0x20, 0x54, 0x56, 0x55, 0x78 },  // E7 231
	{ 0x00, 0x48, 0x7A, 0x41, 0x00 },  // E8 232
	{ 0x30, 0x48, 0x4C, 0x4A, 0x30 },  // E9 233
	{ 0x3C, 0x40, 0x42, 0x21, 0x7C },  // EA 234
	{ 0x0C, 0x50, 0x52, 0x51, 0x3C },  // EB 235
	{ 0x38, 0x46, 0x45, 0x46, 0x38 },  // EC 236
	{ 0x30, 0x4C, 0x4A, 0x4C, 0x30 },  // ED 237
	{ 0x3C, 0x40, 0x41, 0x40, 0x3C },  // EE 238
	{ 0x3C, 0x40, 0x41, 0x20, 0x7C },  // EF 239
	{ 0x38, 0x45, 0x46, 0x45, 0x28 },  // F0 240
	{ 0x7C, 0x55, 0x56, 0x45, 0x44 },  // F1 241
	{ 0x7C, 0x15, 0x16, 0x35, 0x48 },  // F2 242
	{ 0x48, 0x55, 0x56, 0x55, 0x24 },  // F3 243
	{ 0x64, 0x55, 0x56, 0x55, 0x4C },  // F4 244
	{ 0x30, 0x4A, 0x4C, 0x4A, 0x00 },  // F5 245
	{ 0x38, 0x55, 0x56, 0x55, 0x18 },  // F6 246
	{ 0x7C, 0x09, 0x06, 0x05, 0x08 },  // F7 247
	{ 0x48, 0x55, 0x56, 0x55, 0x20 },  // F8 248
	{ 0x44, 0x65, 0x56, 0x4D, 0x44 },  // F9 249
	{ 0x00, 0x7F, 0x41, 0x41, 0x00 },  // FA 250
	{ 0x02, 0x04, 0x08, 0x10, 0x20 },  // FB 251
	{ 0x00, 0x41, 0x41, 0x7F, 0x00 },  // FC 252
	{ 0x00, 0x08, 0x36, 0x41, 0x00 },  // FD 253
	{ 0x00, 0x00, 0x77, 0x00, 0x00 },  // FE 254
	{ 0x00, 0x41, 0x36, 0x08, 0x00 }   // FF 255
};

uint8_t font9x12[96][12] = {
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},    // Code for char num 32
  {0x00,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00,0x10,0x00,0x00},    // Code for char num 33
  {0x00,0x28,0x28,0x28,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},    // Code for char num 34
  {0x00,0x14,0x14,0x7E,0x28,0x28,0x28,0xFC,0x50,0x50,0x00,0x00},    // Code for char num 35
  {0x00,0x10,0x38,0x44,0x40,0x38,0x04,0x44,0x38,0x10,0x00,0x00},    // Code for char num 36
  {0x00,0x40,0xA2,0x44,0x08,0x10,0x20,0x44,0x8A,0x04,0x00,0x00},    // Code for char num 37
  {0x00,0x30,0x40,0x40,0x20,0x60,0x92,0x94,0x88,0x76,0x00,0x00},    // Code for char num 38
  {0x00,0x10,0x10,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},    // Code for char num 39
  {0x00,0x08,0x10,0x10,0x20,0x20,0x20,0x20,0x20,0x10,0x10,0x08},    // Code for char num 40
  {0x00,0x20,0x10,0x10,0x08,0x08,0x08,0x08,0x08,0x10,0x10,0x20},    // Code for char num 41
  {0x00,0x00,0x00,0x6C,0x38,0xFE,0x38,0x6C,0x00,0x00,0x00,0x00},    // Code for char num 42
  {0x00,0x00,0x00,0x10,0x10,0x10,0xFE,0x10,0x10,0x10,0x00,0x00},    // Code for char num 43
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x20,0x00},    // Code for char num 44
  {0x00,0x00,0x00,0x00,0x00,0x00,0xFE,0x00,0x00,0x00,0x00,0x00},    // Code for char num 45
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,0x00},    // Code for char num 46
  {0x00,0x00,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x00,0x00,0x00},    // Code for char num 47
  {0x00,0x38,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00,0x00},    // Code for char num 48
  {0x00,0x10,0x70,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00},    // Code for char num 49
  {0x00,0x38,0x44,0x04,0x04,0x08,0x10,0x20,0x40,0x7C,0x00,0x00},    // Code for char num 50
  {0x00,0x38,0x44,0x04,0x04,0x18,0x04,0x04,0x44,0x38,0x00,0x00},    // Code for char num 51
  {0x00,0x08,0x18,0x18,0x28,0x28,0x48,0x7C,0x08,0x1C,0x00,0x00},    // Code for char num 52
  {0x00,0x7C,0x40,0x40,0x40,0x7C,0x04,0x04,0x44,0x38,0x00,0x00},    // Code for char num 53
  {0x00,0x18,0x20,0x40,0x40,0x78,0x44,0x44,0x44,0x38,0x00,0x00},    // Code for char num 54
  {0x00,0x7C,0x44,0x04,0x08,0x08,0x10,0x10,0x20,0x20,0x00,0x00},    // Code for char num 55
  {0x00,0x38,0x44,0x44,0x44,0x38,0x44,0x44,0x44,0x38,0x00,0x00},    // Code for char num 56
  {0x00,0x38,0x44,0x44,0x44,0x3C,0x04,0x04,0x08,0x30,0x00,0x00},    // Code for char num 57
  {0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x10,0x00,0x00},    // Code for char num 58
  {0x00,0x00,0x00,0x00,0x10,0x00,0x00,0x00,0x00,0x10,0x20,0x00},    // Code for char num 59
  {0x00,0x00,0x00,0x08,0x10,0x20,0x40,0x20,0x10,0x08,0x00,0x00},    // Code for char num 60
  {0x00,0x00,0x00,0x00,0x7C,0x00,0x7C,0x00,0x00,0x00,0x00,0x00},    // Code for char num 61
  {0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x08,0x10,0x20,0x00,0x00},    // Code for char num 62
  {0x00,0x38,0x44,0x04,0x04,0x08,0x10,0x10,0x00,0x10,0x00,0x00},    // Code for char num 63
  {0x00,0x3C,0x42,0x9A,0xAA,0xAA,0xAA,0x9C,0x40,0x38,0x00,0x00},    // Code for char num 64
  {0x00,0x30,0x10,0x38,0x28,0x6C,0x44,0x7C,0x44,0xEE,0x00,0x00},    // Code for char num 65
  {0x00,0xFC,0x42,0x42,0x42,0x7C,0x42,0x42,0x42,0xFC,0x00,0x00},    // Code for char num 66
  {0x00,0x3C,0x42,0x80,0x80,0x80,0x80,0x80,0x42,0x3C,0x00,0x00},    // Code for char num 67
  {0x00,0xF8,0x44,0x42,0x42,0x42,0x42,0x42,0x44,0xF8,0x00,0x00},    // Code for char num 68
  {0x00,0xFE,0x42,0x40,0x48,0x78,0x48,0x40,0x42,0xFE,0x00,0x00},    // Code for char num 69
  {0x00,0xFE,0x42,0x40,0x48,0x78,0x48,0x40,0x40,0xF0,0x00,0x00},    // Code for char num 70
  {0x00,0x3C,0x42,0x80,0x80,0x80,0x8E,0x82,0x42,0x3C,0x00,0x00},    // Code for char num 71
  {0x00,0xEE,0x44,0x44,0x44,0x7C,0x44,0x44,0x44,0xEE,0x00,0x00},    // Code for char num 72
  {0x00,0x7C,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00},    // Code for char num 73
  {0x00,0x3C,0x08,0x08,0x08,0x08,0x08,0x88,0x88,0x70,0x00,0x00},    // Code for char num 74
  {0x00,0xE6,0x44,0x48,0x48,0x50,0x70,0x48,0x44,0xE6,0x00,0x00},    // Code for char num 75
  {0x00,0x78,0x20,0x20,0x20,0x20,0x20,0x20,0x22,0x7E,0x00,0x00},    // Code for char num 76
  {0x00,0xC6,0x44,0x6C,0x6C,0x54,0x54,0x44,0x44,0xEE,0x00,0x00},    // Code for char num 77
  {0x00,0xCE,0x44,0x64,0x64,0x54,0x4C,0x4C,0x44,0xE4,0x00,0x00},    // Code for char num 78
  {0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x44,0x38,0x00,0x00},    // Code for char num 79
  {0x00,0xFC,0x42,0x42,0x42,0x7C,0x40,0x40,0x40,0xF0,0x00,0x00},    // Code for char num 80
  {0x00,0x38,0x44,0x82,0x82,0x82,0x82,0x82,0x44,0x38,0x36,0x00},    // Code for char num 81
  {0x00,0xFC,0x42,0x42,0x42,0x7C,0x48,0x48,0x44,0xE6,0x00,0x00},    // Code for char num 82
  {0x00,0x7C,0x82,0x80,0x80,0x7C,0x02,0x02,0x82,0x7C,0x00,0x00},    // Code for char num 83
  {0x00,0xFE,0x92,0x10,0x10,0x10,0x10,0x10,0x10,0x38,0x00,0x00},    // Code for char num 84
  {0x00,0xEE,0x44,0x44,0x44,0x44,0x44,0x44,0x44,0x38,0x00,0x00},    // Code for char num 85
  {0x00,0xEE,0x44,0x44,0x44,0x28,0x28,0x28,0x10,0x10,0x00,0x00},    // Code for char num 86
  {0x00,0xEE,0x44,0x44,0x44,0x54,0x54,0x54,0x28,0x28,0x00,0x00},    // Code for char num 87
  {0x00,0xEE,0x44,0x28,0x28,0x10,0x28,0x28,0x44,0xEE,0x00,0x00},    // Code for char num 88
  {0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x10,0x10,0x38,0x00,0x00},    // Code for char num 89
  {0x00,0xFE,0x84,0x08,0x08,0x10,0x20,0x20,0x42,0xFE,0x00,0x00},    // Code for char num 90
  {0x38,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x20,0x38,0x00},    // Code for char num 91
  {0x00,0x00,0x80,0x40,0x20,0x10,0x08,0x04,0x02,0x00,0x00,0x00},    // Code for char num 92
  {0x38,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x08,0x38,0x00},    // Code for char num 93
  {0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},    // Code for char num 94
  {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00},    // Code for char num 95
  {0x00,0x20,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},    // Code for char num 96
  {0x00,0x00,0x00,0x00,0x78,0x04,0x7C,0x84,0x84,0x7A,0x00,0x00},    // Code for char num 97
  {0x00,0xC0,0x40,0x40,0x7C,0x42,0x42,0x42,0x42,0xFC,0x00,0x00},    // Code for char num 98
  {0x00,0x00,0x00,0x00,0x78,0x84,0x80,0x80,0x84,0x78,0x00,0x00},    // Code for char num 99
  {0x00,0x0C,0x04,0x04,0x7C,0x84,0x84,0x84,0x84,0x7E,0x00,0x00},    // Code for char num 100
  {0x00,0x00,0x00,0x00,0x78,0x84,0xFC,0x80,0x84,0x78,0x00,0x00},    // Code for char num 101
  {0x00,0x18,0x20,0x20,0x78,0x20,0x20,0x20,0x20,0x78,0x00,0x00},    // Code for char num 102
  {0x00,0x00,0x00,0x7E,0x84,0x84,0x84,0x7C,0x04,0x04,0x78,0x00},    // Code for char num 103
  {0x00,0xC0,0x40,0x40,0x58,0x64,0x44,0x44,0x44,0xEE,0x00,0x00},    // Code for char num 104
  {0x00,0x10,0x00,0x00,0x70,0x10,0x10,0x10,0x10,0x7C,0x00,0x00},    // Code for char num 105
  {0x00,0x08,0x00,0x78,0x08,0x08,0x08,0x08,0x08,0x08,0x70,0x00},    // Code for char num 106
  {0x00,0xC0,0x40,0x40,0x4C,0x48,0x50,0x70,0x48,0xC6,0x00,0x00},    // Code for char num 107
  {0x00,0x30,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x7C,0x00,0x00},    // Code for char num 108
  {0x00,0x00,0x00,0x00,0xE8,0x54,0x54,0x54,0x54,0xD6,0x00,0x00},    // Code for char num 109
  {0x00,0x00,0x00,0x00,0xD8,0x64,0x44,0x44,0x44,0xEE,0x00,0x00},    // Code for char num 110
  {0x00,0x00,0x00,0x00,0x7C,0x82,0x82,0x82,0x82,0x7C,0x00,0x00},    // Code for char num 111
  {0x00,0x00,0x00,0xFC,0x42,0x42,0x42,0x7C,0x40,0x40,0xE0,0x00},    // Code for char num 112
  {0x00,0x00,0x00,0x7E,0x84,0x84,0x84,0x7C,0x04,0x04,0x0E,0x00},    // Code for char num 113
  {0x00,0x00,0x00,0x00,0xEC,0x32,0x20,0x20,0x20,0xF8,0x00,0x00},    // Code for char num 114
  {0x00,0x00,0x00,0x00,0x7C,0x82,0x70,0x0C,0x82,0x7C,0x00,0x00},    // Code for char num 115
  {0x00,0x00,0x20,0x20,0x78,0x20,0x20,0x20,0x24,0x18,0x00,0x00},    // Code for char num 116
  {0x00,0x00,0x00,0x00,0xCC,0x44,0x44,0x44,0x4C,0x36,0x00,0x00},    // Code for char num 117
  {0x00,0x00,0x00,0x00,0xEE,0x44,0x44,0x28,0x28,0x10,0x00,0x00},    // Code for char num 118
  {0x00,0x00,0x00,0x00,0xEE,0x44,0x54,0x54,0x28,0x28,0x00,0x00},    // Code for char num 119
  {0x00,0x00,0x00,0x00,0xEE,0x44,0x38,0x38,0x44,0xEE,0x00,0x00},    // Code for char num 120
  {0x00,0x00,0x00,0xEE,0x44,0x44,0x28,0x38,0x10,0x10,0x60,0x00},    // Code for char num 121
  {0x00,0x00,0x00,0x00,0xFC,0x88,0x10,0x20,0x44,0xFC,0x00,0x00},    // Code for char num 122
  {0x0C,0x10,0x10,0x10,0x10,0x60,0x10,0x10,0x10,0x10,0x0C,0x00},    // Code for char num 123
  {0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x00},    // Code for char num 124
  {0x60,0x10,0x10,0x10,0x10,0x0C,0x10,0x10,0x10,0x10,0x60,0x00},    // Code for char num 125
  {0x00,0x00,0x62,0x92,0x8C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},    // Code for char num 126
  {0x00,0x00,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x38,0x00}     // Code for char num 127
};

//vairable declarations display end



//function definitions display start

void displayText(char *str, int cursorX, int cursorY, int onOff);
void clearDisplay();
void mainMenu();
void drafts();
void newDoc();
void newDocCreate(char *docName);
void openDoc();
void delDoc();
void BT();
void connectNewBT();
void WIFI();
void connectNewWIFI();
void docSync();
void syncOnOff();
void setupSync();
void gDriveConnect();
void gDriveConnectPwd(char *gDriveUsername);
void oneDriveConnect();
void oneDriveConnectPwd(char *oneDriveUsername);
void dropBoxConnect();
void dropBoxConnectPwd(char *dropBoxUsername);
void fontSize();
void statusMenu();
void list_files(const char *path);
void write_dat(unsigned char data);
void write_cmd(unsigned char data);
void LCD_initial();
void writeMode(int fileID);
void displayPageChange(int upDown);
void displayChar1(char c, int x, int y);
void displayChar2(char c, int x, int y);
void setCursor(bool onOff);
void bufferLeftShift();
void bufferRightShift(char c);

//function defintions display end


static char scan_input_buffer[MAX_SCAN_INPUT_LENGTH] = {0};
static size_t scan_input_length = 0;

TaskHandle_t hid_demo_task_handle;
TaskHandle_t usb_events_task_handle;
TaskHandle_t repeated_keys_task_handle;
TaskHandle_t monitor_keyboard_input_task_handle;
TaskHandle_t display_task_handle;

//mod by as open
typedef struct {
    uint8_t key_code;
    uint8_t counter;
    bool initial_delay_passed;
    uint8_t modifier;
} key_state_t;

#define MAX_KEYS 6
static key_state_t key_states[MAX_KEYS] = {0};
static uint8_t num_keys = 0;
static bool caps_lock_state = false;
volatile bool hid_demo_task_running = false;
static int btnID = 0;
static uint8_t hid_batt_pct = 0;
static uint8_t byok_batt_pct = 100;

esp_hidh_dev_t *connected_hid_dev = NULL;

static esp_hid_scan_result_t *scan_results = NULL;

static size_t results_len = 0;

static SemaphoreHandle_t xbtnIDMutex = NULL;

static SemaphoreHandle_t xHidDemoSemaphore = NULL;

static SemaphoreHandle_t xMonitorKeyboardSemaphore = NULL;

static int8_t selected_index = -1;

QueueHandle_t key_print_queue;

//mod by as close

/**
 * @brief Application Event from USB Host driver
 *
 */
typedef enum {
    HOST_NO_CLIENT = 0x1,
    HOST_ALL_FREE = 0x2,
    DEVICE_CONNECTED = 0x4,
    DEVICE_DISCONNECTED = 0x8,
    DEVICE_ADDRESS_MASK = 0xFF0,
    APP_QUIT_EVENT = 0x10000,

} app_event_t;

/**
 * @brief Key event
 *
 */
typedef struct {
    enum key_state {
        KEY_STATE_PRESSED = 0x00,
        KEY_STATE_RELEASED = 0x01
    } state;
    uint8_t modifier;
    uint8_t key_code;
} key_event_t;

#define USB_EVENTS_TO_WAIT      (DEVICE_CONNECTED | DEVICE_ADDRESS_MASK | DEVICE_DISCONNECTED)

static const char *TAG = "example";

static const char *TAG2 = "ESP_HIDH_DEMO";


static EventGroupHandle_t usb_flags;
static hid_host_device_handle_t hid_device = NULL;
static bool hid_device_connected = false;

hid_host_interface_handle_t keyboard_handle = NULL;

/**
 * @brief Scancode to ascii table
 */
const uint8_t keycode2ascii [113][2] = {
    {0, 0}, /* HID_KEY_NO_PRESS        */
    {0, 0}, /* HID_KEY_ROLLOVER        */
    {0, 0}, /* HID_KEY_POST_FAIL       */
    {0, 0}, /* HID_KEY_ERROR_UNDEFINED */
    {'a', 'A'}, /* HID_KEY_A               */
    {'b', 'B'}, /* HID_KEY_B               */
    {'c', 'C'}, /* HID_KEY_C               */
    {'d', 'D'}, /* HID_KEY_D               */
    {'e', 'E'}, /* HID_KEY_E               */
    {'f', 'F'}, /* HID_KEY_F               */
    {'g', 'G'}, /* HID_KEY_G               */
    {'h', 'H'}, /* HID_KEY_H               */
    {'i', 'I'}, /* HID_KEY_I               */
    {'j', 'J'}, /* HID_KEY_J               */
    {'k', 'K'}, /* HID_KEY_K               */
    {'l', 'L'}, /* HID_KEY_L               */
    {'m', 'M'}, /* HID_KEY_M               */
    {'n', 'N'}, /* HID_KEY_N               */
    {'o', 'O'}, /* HID_KEY_O               */
    {'p', 'P'}, /* HID_KEY_P               */
    {'q', 'Q'}, /* HID_KEY_Q               */
    {'r', 'R'}, /* HID_KEY_R               */
    {'s', 'S'}, /* HID_KEY_S               */
    {'t', 'T'}, /* HID_KEY_T               */
    {'u', 'U'}, /* HID_KEY_U               */
    {'v', 'V'}, /* HID_KEY_V               */
    {'w', 'W'}, /* HID_KEY_W               */
    {'x', 'X'}, /* HID_KEY_X               */
    {'y', 'Y'}, /* HID_KEY_Y               */
    {'z', 'Z'}, /* HID_KEY_Z               */
    {'1', '!'}, /* HID_KEY_1               */
    {'2', '@'}, /* HID_KEY_2               */
    {'3', '#'}, /* HID_KEY_3               */
    {'4', '$'}, /* HID_KEY_4               */
    {'5', '%'}, /* HID_KEY_5               */
    {'6', '^'}, /* HID_KEY_6               */
    {'7', '&'}, /* HID_KEY_7               */
    {'8', '*'}, /* HID_KEY_8               */
    {'9', '('}, /* HID_KEY_9               */
    {'0', ')'}, /* HID_KEY_0               */
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /* HID_KEY_ENTER           */
    {0, 0}, /* HID_KEY_ESC             */
    {0, 0}, /* HID_KEY_DEL             */
    {0, 0}, /* HID_KEY_TAB             */
    {' ', ' '}, /* HID_KEY_SPACE           */
    {'-', '_'}, /* HID_KEY_MINUS           */
    {'=', '+'}, /* HID_KEY_EQUAL           */
    {'[', '{'}, /* HID_KEY_OPEN_BRACKET    */
    {']', '}'}, /* HID_KEY_CLOSE_BRACKET   */
    {'\\', '|'}, /* HID_KEY_BACK_SLASH      */
    {'\\', '|'}, /* HID_KEY_SHARP           */  // HOTFIX: for NonUS Keyboards repeat HID_KEY_BACK_SLASH
    {';', ':'}, /* HID_KEY_COLON           */
    {'\'', '"'}, /* HID_KEY_QUOTE           */
    {'`', '~'}, /* HID_KEY_TILDE           */
    {',', '<'}, /* HID_KEY_LESS            */
    {'.', '>'}, /* HID_KEY_GREATER         */
    {'/', '?'}, /* HID_KEY_SLASH           */
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}, /*HID_KEY_DELETE*/
    {0, 0},
    {0, 0},
    {'R', 'R'}, /*HID_KEY_RIGHT*/
    {'L', 'L'}, /*HID_KEY_LEFT*/
    {'D', 'D'}, /*HID_KEY_DOWN*/
    {'U', 'U'}, /*HID_KEY_UP*/
    {0, 0},
    {'/', '/'}, /*HID_KEY_KEYPAD_DIV*/
    {'*', '*'}, /*HID_KEY_KEYPAD_MUL*/
    {'-', '-'}, /*HID_KEY_KEYPAD_SUB*/
    {'+', '+'}, /*HID_KEY_KEYPAD_ADD*/
    {KEYBOARD_ENTER_MAIN_CHAR, KEYBOARD_ENTER_MAIN_CHAR}, /*HID_KEY_KEYPAD_ENTER*/
    {'1', '1'},
    {'2', '2'},
    {'3', '3'},
    {'4', '4'},
    {'5', '5'},
    {'6', '6'},
    {'7', '7'},
    {'8', '8'},
    {'9', '9'},
    {'0', '0'},
    {0, '.'},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0},
    {0, 0}      /* HID_KEY_F21*/
};

//OTA Code in main.c open
/* FreeRTOS event group to signal when we are connected*/
char running_fw_version[32] = "invalid";
EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
const int WIFI_CONNECTED_BIT = BIT0;
#define WIFI_FAIL_BIT BIT1

//static const char *TAG = "MAIN";

static int s_retry_num = 0;

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
                  wifiStatus = true;
                 
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    }
    else
    {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}
//OTA Code in main.c close









/**
 * @brief Makes new line depending on report output protocol type
 *
 * @param[in] proto Current protocol to output
 */
static void hid_print_new_device_report_header(hid_protocol_t proto)
{
    static hid_protocol_t prev_proto_output = HID_PROTOCOL_NONE;

    if (prev_proto_output != proto) {
        prev_proto_output = proto;
        printf("\r\n");
        /*
        if (proto == HID_PROTOCOL_MOUSE) {
            printf("Mouse\r\n");
        }
        */
        if (proto == HID_PROTOCOL_KEYBOARD) {
            printf("Keyboard\r\n");
        }
        fflush(stdout);
    }
}

/**
 * @brief HID Keyboard modifier verification for capitalization application (right or left shift)
 *
 * @param[in] modifier
 * @return true  Modifier was pressed (left or right shift)
 * @return false Modifier was not pressed (left or right shift)
 *
 */
static inline bool hid_keyboard_is_modifier_shift(uint8_t modifier)
{
    if ((modifier == HID_LEFT_SHIFT) ||
            (modifier == HID_RIGHT_SHIFT)) {
        return true;
    }
    return false;
}

/**
 * @brief HID Keyboard get char symbol from key code
 *
 * @param[in] modifier  Keyboard modifier data
 * @param[in] key_code  Keyboard key code
 * @param[in] key_char  Pointer to key char data
 *
 * @return true  Key scancode converted successfully
 * @return false Key scancode unknown
 */
static inline bool hid_keyboard_get_char(uint8_t modifier,
        uint8_t key_code,
        unsigned char *key_char)
{
    if(!(modifier == 0 || modifier == HID_LEFT_SHIFT || modifier == HID_RIGHT_SHIFT)){
      return false;
    }
    uint8_t mod = (hid_keyboard_is_modifier_shift(modifier)) ? 1 : 0;
    if(key_code == HID_KEY_CAPS_LOCK){
        caps_lock_state = !caps_lock_state;
    }

    if((key_code >= HID_KEY_1) && (key_code <= HID_KEY_F21)){
        *key_char = keycode2ascii[key_code][mod];
    }
    else if((key_code >= HID_KEY_A) && (key_code <= HID_KEY_Z)){
        if(!!caps_lock_state){
            *key_char = keycode2ascii[key_code][!mod];
        }
        else
            *key_char = keycode2ascii[key_code][mod];
    }
    else{
        // All other key pressed

        return false;

    }
    /*
    if ((key_code >= HID_KEY_A) && (key_code <= HID_KEY_F21)) {       //modified by as
        if(!!caps_lock_state)
        *key_char = keycode2ascii[key_code][mod];
    } else {
        // All other key pressed

        return false;
    }*/

    return true;
}

/**
 * @brief HID Keyboard print char symbol
 *
 * @param[in] key_char  Keyboard char to stdout
 */
static inline void hid_keyboard_print_char(unsigned int key_char)
{
    if (!!key_char) {
        //edited code
        switch (key_char) {
            default:
              if (charInput) {
                if (xQueueSend(key_print_queue, &key_char, portMAX_DELAY) != pdPASS) {
                    printf("Failed to send to queue\n");
                }
              }
              putchar(key_char);
                
        }
        //putchar(key_char);
#if (KEYBOARD_ENTER_LF_EXTEND)
        if (KEYBOARD_ENTER_MAIN_CHAR == key_char) {
            putchar('\n');
        }
#endif // KEYBOARD_ENTER_LF_EXTEND
        fflush(stdout);
    }
}

/**
 * @brief Key Event. Key event with the key code, state and modifier.
 *
 * @param[in] key_event Pointer to Key Event structure
 *
 */
static void key_event_callback(key_event_t *key_event)
{
    esp_timer_stop(periodic_timer);
    esp_timer_start_periodic(periodic_timer, IDLE_DELAY);
    unsigned char key_char;

    hid_print_new_device_report_header(HID_PROTOCOL_KEYBOARD);
    printf("key_modifier: %d\n", key_event->modifier);
    printf("key_code: %d\n", key_event->key_code);
    if (KEY_STATE_PRESSED == key_event->state) {
        if (hid_keyboard_get_char(key_event->modifier,
                                  key_event->key_code, &key_char)) {
            if((key_event->key_code == HID_KEY_TAB) || (key_event->key_code == HID_KEY_ESC) || (key_event->key_code == HID_KEY_DEL) || (key_event->key_code == HID_KEY_RIGHT) || (key_event->key_code == HID_KEY_LEFT) || (key_event->key_code == HID_KEY_DOWN) ||(key_event->key_code == HID_KEY_UP)){
                switch(key_event->key_code){
                    case HID_KEY_TAB:
                        for(uint8_t i = 0; i<4; i++)
                        {
                            hid_keyboard_print_char(' ');
                        }
                        break;
                    case HID_KEY_ESC:
                        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE)
                            btnID = 2;
                        xSemaphoreGive(xbtnIDMutex);
                        break;
                    case HID_KEY_DEL:
                        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE)
                            btnID = -4;
                        xSemaphoreGive(xbtnIDMutex);
                        break;
                    case HID_KEY_RIGHT:
                        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE)
                            btnID = 3;
                        xSemaphoreGive(xbtnIDMutex);
                        break;
                    case HID_KEY_LEFT:
                        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE)
                            btnID = -3;
                        xSemaphoreGive(xbtnIDMutex);
                        break;
                    case HID_KEY_UP:
                        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE)
                            btnID = -1;
                        xSemaphoreGive(xbtnIDMutex);
                        break;
                    case HID_KEY_DOWN:
                        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE)
                            btnID = 1;
                        xSemaphoreGive(xbtnIDMutex);
                        break;
                    default : 
                        break;
                }
            }
            /*else if(((key_event->modifier && HID_LEFT_CONTROL) || (key_event->modifier && HID_RIGHT_CONTROL)) && (key_event->key_code == HID_KEY_SPACE)){
                printf("Status Menu\n");
                if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE){
                    btnID = -2;
                    xSemaphoreGive(xbtnIDMutex);
                    }                 
            }*/
            else if(key_event->key_code == HID_KEY_ENTER){
                if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE){
                    if (WMcharInput) btnID = -2;
                    else btnID = 2;
                    xSemaphoreGive(xbtnIDMutex);
                    }                 
            }
            else{
                hid_keyboard_print_char(key_char);
            }
            // Add the character to the input buffer
            if (scan_input_length < MAX_SCAN_INPUT_LENGTH - 1) {
                scan_input_buffer[scan_input_length++] = key_char;
                scan_input_buffer[scan_input_length] = '\0'; // Null-terminate the string
            } else {
                // Reset buffer if it overflows
                scan_input_length = 0;
                memset(scan_input_buffer, 0, MAX_SCAN_INPUT_LENGTH);
            }
        }
    }
}

/**
 * @brief Key buffer scan code search.
 *
 * @param[in] src       Pointer to source buffer where to search
 * @param[in] key       Key scancode to search
 * @param[in] length    Size of the source buffer
 */
static inline bool key_found(const uint8_t *const src,
                             uint8_t key,
                             unsigned int length)
{
    for (unsigned int i = 0; i < length; i++) {
        if (src[i] == key) {
            return true;
        }
    }
    return false;
}

/**
 * @brief USB HID Host Keyboard Interface report callback handler
 *
 * @param[in] data    Pointer to input report data buffer
 * @param[in] length  Length of input report data buffer
 */
static void hid_host_keyboard_report_callback(const uint8_t *const data, const int length) {
    hid_keyboard_input_report_boot_t *kb_report = (hid_keyboard_input_report_boot_t *)data;

    if (length < sizeof(hid_keyboard_input_report_boot_t)) {
        return;
    }

    static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
    key_event_t key_event;
    num_keys = 0;

    for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {
        if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
            !key_found(kb_report->key, prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
            key_event.key_code = prev_keys[i];
            key_event.modifier = 0;
            key_event.state = KEY_STATE_RELEASED;
            key_event_callback(&key_event);
            key_states[i].key_code = 0;
            key_states[i].counter = 0;
            key_states[i].initial_delay_passed = false;
            key_states[i].modifier = 0;
        }

        if (kb_report->key[i] > HID_KEY_ERROR_UNDEFINED) {
            key_states[num_keys].key_code = kb_report->key[i];
            key_states[num_keys].counter = 0;
            key_states[num_keys].initial_delay_passed = false;
            key_states[num_keys].modifier = kb_report->modifier.val;
            num_keys++;
            if (!key_found(prev_keys, kb_report->key[i], HID_KEYBOARD_KEY_MAX)) {
                key_event.key_code = kb_report->key[i];
                key_event.modifier = kb_report->modifier.val;
                key_event.state = KEY_STATE_PRESSED;
                key_event_callback(&key_event);
            }
        }
    }

    memcpy(prev_keys, &kb_report->key, HID_KEYBOARD_KEY_MAX);
    
}


void hidh_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidh_event_t event = (esp_hidh_event_t)id;
    esp_hidh_event_data_t *param = (esp_hidh_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDH_OPEN_EVENT: {
        if (param->open.status == ESP_OK) {
            const uint8_t *bda = esp_hidh_dev_bda_get(param->open.dev);
            ESP_LOGI(TAG, ESP_BD_ADDR_STR " OPEN: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->open.dev));
            esp_hidh_dev_dump(param->open.dev, stdout);
        } else {
            ESP_LOGE(TAG, " OPEN failed!");
        }
        break;
    }
    case ESP_HIDH_BATTERY_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->battery.dev);
        hid_batt_pct = param->battery.level;
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " BATTERY: %d%%", ESP_BD_ADDR_HEX(bda), param->battery.level);
        break;
    }
    case ESP_HIDH_INPUT_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->input.dev);
        static uint8_t prev_keys[HID_KEYBOARD_KEY_MAX] = {0};
        key_event_t key_event;
        num_keys = 0;

        for (int i = 0; i < HID_KEYBOARD_KEY_MAX; i++) {
            if (prev_keys[i] > HID_KEY_ERROR_UNDEFINED &&
                !key_found((&param->input.data[i+1]), prev_keys[i], HID_KEYBOARD_KEY_MAX)) {
                key_event.key_code = prev_keys[i];
                key_event.modifier = 0;
                key_event.state = KEY_STATE_RELEASED;
                key_event_callback(&key_event);
                key_states[i].key_code = 0;
                key_states[i].counter = 0;
                key_states[i].initial_delay_passed = false;
                key_states[i].modifier = 0;
            }

            if (param->input.data[i+1] > HID_KEY_ERROR_UNDEFINED) {
                key_states[num_keys].key_code = param->input.data[i+1];
                key_states[num_keys].counter = 0;
                key_states[num_keys].initial_delay_passed = false;
                key_states[num_keys].modifier = param->input.data[0];
                num_keys++;
                if (!key_found(prev_keys, param->input.data[i+1], HID_KEYBOARD_KEY_MAX)) {
                    key_event.key_code = param->input.data[i+1];
                    key_event.modifier = param->input.data[0];
                    key_event.state = KEY_STATE_PRESSED;
                    key_event_callback(&key_event);
                }
            }
        }

        memcpy(prev_keys, &param->input.data[1], HID_KEYBOARD_KEY_MAX);

    /*
        int length_id = param->input.length;
        int mod = param->input.data[0];
        int key = param->input.data[1];
        unsigned int key_char; */
        //ESP_LOGI(TAG, ESP_BD_ADDR_STR " INPUT: %8s, MAP: %2u, ID: %3u, Len: %d, Data:", ESP_BD_ADDR_HEX(bda), esp_hid_usage_str(param->input.usage), param->input.map_index, param->input.report_id, param->input.length);
        //ESP_LOG_BUFFER_HEX(TAG, param->input.data, param->input.length);
        //printf("Input event\n");
       /* for(int i = 0; i<length_id; i++){
            printf("%d: ", i);
            printf(" %d ", param->input.data[i]);
        }*/
        //printf("\n");
        /*
        if(param->input.data[0] > 0 && param->input.data[1] > 0){
            //printf(" %d :", param->input.data[1]);
            //printf(" %c \n", param->input.data[1]);
            key_char = keycode2ascii[param->input.data[1]][1];
            //printf("mod %d\n", key_char);
            putchar((unsigned int)key_char);
        }
        else if(param->input.data[1] > 0){
            key_char = keycode2ascii[param->input.data[1]][0];
            //printf("no mod %d\n", key_char);
            putchar((unsigned int)key_char);
        } */
        //printf("")
        //printf("\n");
        //fflush(stdout); 
        break;
    }
    case ESP_HIDH_FEATURE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->feature.dev);
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " FEATURE: %8s, MAP: %2u, ID: %3u, Len: %d", ESP_BD_ADDR_HEX(bda),
                 esp_hid_usage_str(param->feature.usage), param->feature.map_index, param->feature.report_id,
                 param->feature.length);
        ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
        break;
    }
    case ESP_HIDH_CLOSE_EVENT: {
        const uint8_t *bda = esp_hidh_dev_bda_get(param->close.dev);
        //avlog_o: added so that hid_demo can be notified to stop running and give semaphore
        //xTaskNotifyGive(hid_demo_task_handle);
        //avlog_c
        ESP_LOGI(TAG, ESP_BD_ADDR_STR " CLOSE: %s", ESP_BD_ADDR_HEX(bda), esp_hidh_dev_name_get(param->close.dev));
        break;
    }
    default:
        ESP_LOGI(TAG, "EVENT: %d", event);
        break;
    }
}


#if 0 //modified code for using scan functionality
void hid_demo_task(void *pvParameters)
{
    size_t results_len = 0;
    esp_hid_scan_result_t *results = NULL;
    ESP_LOGI(TAG, "SCAN...");
    //start scan for HID devices
    esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &results);
    ESP_LOGI(TAG, "SCAN: %u results", results_len);
    if (results_len) {
        esp_hid_scan_result_t *r = results;
        esp_hid_scan_result_t *cr = NULL;
        while (r) {
            printf("  %s: " ESP_BD_ADDR_STR ", ", (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
            printf("RSSI: %d, ", r->rssi);
            printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BLE) {
                cr = r;
                printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
            }
            
           
#endif /* CONFIG_BT_BLE_ENABLED */

#if CONFIG_BT_HID_HOST_ENABLED
            if (r->transport == ESP_HID_TRANSPORT_BT) {
                cr = r;
                printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                printf("] srv 0x%03x, ", r->bt.cod.service);
                print_uuid(&r->bt.uuid);
                printf(", ");
            }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
            printf("NAME: %s ", r->name ? r->name : "");
            printf("\n");
            r = r->next;
        }
        if (cr) {
            //open the last result
            esp_hidh_dev_open(cr->bda, cr->transport, cr->ble.addr_type);
        }
        
        //free the results
        esp_hid_scan_results_free(results);
    }
    vTaskDelete(NULL);
}

#endif


void hid_demo_task(void *pvParameters) {
    //SemaphoreHandle_t xHidDemoSemaphore = (SemaphoreHandle_t)pvParameters;

    while (1) {
        // Wait for the semaphore to be given by another task
        //if (xSemaphoreTake(xHidDemoSemaphore, portMAX_DELAY) == pdTRUE) {
            ESP_LOGI("kuch bhi", "Waiting in HID\n");
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
            ESP_LOGI("kuch bhi", "got notif in HID\n");
            hid_demo_task_running = true;
            //size_t results_len = 0;
            //esp_hid_scan_result_t *results = NULL;
            ESP_LOGI(TAG, "SCAN...");
            // Start scan for HID devices
            esp_hid_scan(SCAN_DURATION_SECONDS, &results_len, &scan_results);
            ESP_LOGI(TAG, "SCAN: %u results", results_len);
            if (results_len) {
                esp_hid_scan_result_t *r = scan_results;
                int index = 1;
                //esp_hid_scan_result_t *cr = NULL;
                while (r) {
                    printf("%d:  %s: " ESP_BD_ADDR_STR ", ", index, (r->transport == ESP_HID_TRANSPORT_BLE) ? "BLE" : "BT ", ESP_BD_ADDR_HEX(r->bda));
                    printf("RSSI: %d, ", r->rssi);
                    printf("USAGE: %s, ", esp_hid_usage_str(r->usage));
#if CONFIG_BT_BLE_ENABLED
                    if (r->transport == ESP_HID_TRANSPORT_BLE) {
                        //cr = r;
                        printf("APPEARANCE: 0x%04x, ", r->ble.appearance);
                        printf("ADDR_TYPE: '%s', ", ble_addr_type_str(r->ble.addr_type));
                    }
#endif /* CONFIG_BT_BLE_ENABLED */
#if CONFIG_BT_HID_HOST_ENABLED
                    if (r->transport == ESP_HID_TRANSPORT_BT) {
                        //cr = r;
                        printf("COD: %s[", esp_hid_cod_major_str(r->bt.cod.major));
                        esp_hid_cod_minor_print(r->bt.cod.minor, stdout);
                        printf("] srv 0x%03x, ", r->bt.cod.service);
                        print_uuid(&r->bt.uuid);
                        printf(", ");
                    }
#endif /* CONFIG_BT_HID_HOST_ENABLED */
                    printf("NAME: %s ", r->name ? r->name : "");
                    printf("\n");
                    r = r->next;
                    index++;
                }
            }

            // Notify the monitor_keyboard_input task to wake up and get user input

            //xSemaphoreGive(xMonitorKeyboardSemaphore);
            
            xTaskNotifyGive(display_task_handle);

            // Wait for the monitor_keyboard_input task to complete

            if (results_len>0) ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            // Open the selected device based on user input

            if (selected_index >= 0 && selected_index < results_len) {

                esp_hid_scan_result_t *r = scan_results;

                for (int i = 0; i < selected_index; i++) {

                    r = r->next;
                }
                 if (r) {

                    connected_hid_dev = esp_hidh_dev_open(r->bda, r->transport, r->ble.addr_type);

                }
                if(connected_hid_dev != NULL) {
                  btStatus = true;
                  xTaskNotifyGive(display_task_handle);
                  // Free the results
                  esp_hid_scan_results_free(scan_results);
                  //wait to disconnect
                  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
                  esp_err_t hid_dev_disconnection_status = esp_hidh_dev_close(connected_hid_dev);
                  if(!hid_dev_disconnection_status) {
                    btStatus = false;
                    selected_index = -1;
                  }
                }
            }
            hid_demo_task_running = false;
           // xSemaphoreGive(xHidDemoSemaphore);
        //}
    }
}

static void handle_repeated_keys(void *arg) {
    const TickType_t xInitialDelay = pdMS_TO_TICKS(300);  // Initial delay of 300 ms
    const TickType_t xRepeatDelay = pdMS_TO_TICKS(50);  // Repeat delay of 50 ms

    while (1) {
        for (int i = 0; i < num_keys; i++) {
            if (key_states[i].key_code > HID_KEY_ERROR_UNDEFINED) {
                if (!key_states[i].initial_delay_passed) {
                    // Wait for the initial delay before starting repeated key presses
                    vTaskDelay(xInitialDelay);
                    key_states[i].initial_delay_passed = true;
                } else {
                    key_states[i].counter++;
                    if (key_states[i].counter >= 5) {  // Adjust the counter threshold as needed
                        key_event_t key_event;
                        key_event.key_code = key_states[i].key_code;
                        key_event.modifier = key_states[i].modifier;
                        key_event.state = KEY_STATE_PRESSED;
                        key_event_callback(&key_event);
                        key_states[i].counter = 0;  // Reset counter after printing
                    }
                }
            }
        }
        vTaskDelay(xRepeatDelay);
    }
}

/**
 * @brief USB HID Host event callback. Handle such event as device connection and removing
 *
 * @param[in] event  HID device event
 * @param[in] arg    Pointer to arguments, does not used
 */
static void hid_host_event_callback(const hid_host_event_t *event, void *arg)
{
    if (event->event == HID_DEVICE_CONNECTED) {
        // Obtained USB device address is placed after application events
        xEventGroupSetBits(usb_flags, DEVICE_CONNECTED | (event->device.address << 4));
    } else if (event->event == HID_DEVICE_DISCONNECTED) {
        xEventGroupSetBits(usb_flags, DEVICE_DISCONNECTED);
    }
}

/**
 * @brief USB HID Host interface callback
 *
 * @param[in] event  HID interface event
 * @param[in] arg    Pointer to arguments, does not used
 */
static void hid_host_interface_event_callback(const hid_host_interface_event_t *event, void *arg)
{
    switch (event->event) {
    case HID_DEVICE_INTERFACE_INIT:
        ESP_LOGI(TAG, "Interface number %d, protocol %s",
                 event->interface.num,
                 (event->interface.proto == HID_PROTOCOL_KEYBOARD)
                 ? "Keyboard"
                 : "Mouse");

        if (event->interface.proto == HID_PROTOCOL_KEYBOARD) {
            const hid_host_interface_config_t hid_keyboard_config = {
                .proto = HID_PROTOCOL_KEYBOARD,
                .callback = hid_host_keyboard_report_callback,
            };

            hid_host_claim_interface(&hid_keyboard_config, &keyboard_handle);
        }

        break;
    case HID_DEVICE_INTERFACE_TRANSFER_ERROR:
        ESP_LOGD(TAG, "Interface number %d, transfer error",
                 event->interface.num);
        break;

    case HID_DEVICE_INTERFACE_CLAIM:
    case HID_DEVICE_INTERFACE_RELEASE:
        // ... do nothing here for now
        break;

    default:
        ESP_LOGI(TAG, "%s Unhandled event %X, Interface number %d",
                 __FUNCTION__,
                 event->event,
                 event->interface.num);
        break;
    }
}

/**
 * @brief Handle common USB host library events
 *
 * @param[in] args  Pointer to arguments, does not used
 */
static void handle_usb_events(void *args)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        // Release devices once all clients has deregistered
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
            xEventGroupSetBits(usb_flags, HOST_NO_CLIENT);
        }
        // Give ready_to_uninstall_usb semaphore to indicate that USB Host library
        // can be deinitialized, and terminate this task.
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            xEventGroupSetBits(usb_flags, HOST_ALL_FREE);
        }
    }

    vTaskDelete(NULL);
}

static bool wait_for_event(EventBits_t event, TickType_t timeout)
{
    return xEventGroupWaitBits(usb_flags, event, pdTRUE, pdTRUE, timeout) & event;
}

void vUSBEventHandlerTask(void *pvParameters) {
    while (gpio_get_level(APP_QUIT_PIN) != 0) {
        EventBits_t event = xEventGroupWaitBits(
            usb_flags, 
            USB_EVENTS_TO_WAIT, 
            pdTRUE, 
            pdFALSE, 
            pdMS_TO_TICKS(APP_QUIT_PIN_POLL_MS)
        );

        if (event & DEVICE_CONNECTED) {
            xEventGroupClearBits(usb_flags, DEVICE_CONNECTED);
            hid_device_connected = true;
        }

        if (event & DEVICE_ADDRESS_MASK) {
            xEventGroupClearBits(usb_flags, DEVICE_ADDRESS_MASK);

            const hid_host_device_config_t hid_host_device_config = {
                .dev_addr = (event & DEVICE_ADDRESS_MASK) >> 4,
                .iface_event_cb = hid_host_interface_event_callback,
                .iface_event_arg = NULL,
            };

            ESP_ERROR_CHECK(hid_host_install_device(&hid_host_device_config, &hid_device));
        }

        if (event & DEVICE_DISCONNECTED) {
            xEventGroupClearBits(usb_flags, DEVICE_DISCONNECTED);

            hid_host_release_interface(keyboard_handle);

            ESP_ERROR_CHECK(hid_host_uninstall_device(hid_device));

            hid_device_connected = false;
        }

        vTaskDelay(pdMS_TO_TICKS(APP_QUIT_PIN_POLL_MS)); // Optional: Add a small delay to yield CPU time
    }
    xEventGroupSetBits(usb_flags, APP_QUIT_EVENT);
    vTaskDelete(NULL); // Delete the task if the loop exits
}

static void monitor_keyboard_input(void *pvParameters) {
    //SemaphoreHandle_t xHidDemoSemaphore = (SemaphoreHandle_t)pvParameters;
    const char *trigger_word = "scan";

    while (1) {
        if (strstr(scan_input_buffer, trigger_word) != NULL) {
            // Clear the buffer after detecting the word
            memset(scan_input_buffer, 0, MAX_SCAN_INPUT_LENGTH);
            scan_input_length = 0;

            // Give the semaphore to trigger the hid_demo_task
            if (!hid_demo_task_running) {
                xSemaphoreGive(xHidDemoSemaphore);
            }
            

            // Wait for the hid_demo_task to complete scanning

            xSemaphoreTake(xMonitorKeyboardSemaphore, portMAX_DELAY);



            // Wait for user input

            printf("Enter the number of the device to connect: ");
            fflush(stdout);

            while (1) {

                if (scan_input_length > 0 && scan_input_buffer[scan_input_length - 1] >= '1' && scan_input_buffer[scan_input_length - 1] <= '9') {

                    selected_index = scan_input_buffer[scan_input_length - 1] - '0';

                    // Clear the buffer after selecting a device

                    memset(scan_input_buffer, 0, MAX_SCAN_INPUT_LENGTH);

                    scan_input_length = 0;

                    break;

                }

                vTaskDelay(pdMS_TO_TICKS(100)); // Check the buffer every 100 ms

            }

            // Notify the hid_demo_task to continue

            xTaskNotifyGive(hid_demo_task_handle);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Check the buffer every 100 ms
    }
}

static void byok_battery_calc(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(1000);  // Task frequency in milliseconds (e.g., 1000ms = 1 second)
    // Initialize the last wake time
  xLastWakeTime = xTaskGetTickCount();
  while (1) {
      // Turn GPIO high
      gpio_set_level(BATT_STAT_OP_PIN, 1);
      //ESP_LOGI(TAG, "GPIO %d set to HIGH", BATT_STAT_OP_PIN);
       // Delay for stabilization or any specific duration
      vTaskDelay(pdMS_TO_TICKS(100));  // Adjust as needed
      // Read ADC value
      uint32_t adc_raw = adc1_get_raw(ADC_INPUT_CHANNEL);
      //ESP_LOGI(TAG, "ADC value: %lu", adc_raw);
      float adc_voltage = (float)(adc_raw * 3.3)/ 4095.0;  // Assuming ESP32 operates on 3.3V
      if (adc_voltage < 2.37) byok_batt_pct = 0;
      else if (adc_voltage < 2.57) byok_batt_pct = 20;
      else if (adc_voltage < 2.77) byok_batt_pct = 40;
      else if (adc_voltage < 2.97) byok_batt_pct = 60;
      else if (adc_voltage < 3.17) byok_batt_pct = 80;
      else byok_batt_pct = 100;

      /*float battery_voltage = adc_voltage*1.25;
      float battery_percentage = ((battery_voltage - MIN_BATTERY_VOLTAGE) / (MAX_BATTERY_VOLTAGE - MIN_BATTERY_VOLTAGE)) * 100.0;
      if (battery_percentage < 0.0) {
        battery_percentage = 0.0;
      } else if (battery_percentage > 100.0) {
        battery_percentage = 100.0;
      }
      byok_batt_pct = (int)battery_percentage;*/

      // Turn GPIO low
      gpio_set_level(BATT_STAT_OP_PIN, 0);
      //ESP_LOGI(TAG, "GPIO %d set to LOW", BATT_STAT_OP_PIN);
      // Delay until the next execution
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

static void display_task(void *pvParameters){
    const esp_vfs_fat_mount_config_t mount_config = {
      .max_files = 4,
      .format_if_mount_failed = true,
      .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    esp_err_t err = esp_vfs_fat_spiflash_mount_rw_wl("/spiflash", "storage", &mount_config, &s_wl_handle);
    if (err != ESP_OK) printf("Failed to mount FAT filesystem: %s\n", esp_err_to_name(err));
    else printf("FAT filesystem mounted successfully\n");
    const char* dir_path = "/spiflash/myfiles"; struct stat st = {0};
    if (stat(dir_path, &st) == -1) {
      if (mkdir(dir_path, 0777) == 0) printf("Directory created: %s\n", dir_path);
      else printf("Failed to create directory: %s\n", dir_path);
    } else printf("Directory already exists: %s\n", dir_path);
    list_files("/spiflash/myfiles"); esp_timer_start_periodic(periodic_timer, IDLE_DELAY);
    while(1){
      (FSstats[0] <= 0)? newDoc(): writeMode(0);
      mainMenu();
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    esp_vfs_fat_spiflash_unmount_rw_wl("/spiflash", s_wl_handle);
}

static void power_off_task(void *pvParameters) {
  while (1) {
    while (byok_batt_pct > 20 && !powerOffStatus)
      vTaskDelay(200/portTICK_PERIOD_MS);
    if (WMcharInput && charInput) {
      char path[50];
      snprintf(path, sizeof(path), "/spiflash/myfiles/%s", files[currFileID]);
      charInput = false; WMcharInput = false; clearDisplay();
      displayText("Saving file...", 0, 22, 1);
      printf("Saving file...\n");
      FILE *file3 = fopen(path, "w");
      if (file3 != NULL) {
        fprintf(file3, buffer);
      fflush(file3);
      fsync(fileno(file3));
        fclose(file3);
      } else {
        clearDisplay();
        displayText("Failed to save file", 0, 22, 1);
      } free(buffer);
      list_files("/spiflash/myfiles");
    }
    clearDisplay();
    if (byok_batt_pct <= 20)
      displayText("Battery Low", 0, 22, 1);
    displayText("Shutting down...", 0, 32, 1);
    printf("Powering off...\n");
    esp_vfs_fat_spiflash_unmount_rw_wl("/spiflash", s_wl_handle);
    UBaseType_t uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    printf("Stack high-water mark: %u bytes\n", uxHighWaterMark);
    gpio_set_level(POWER_LED, 0);
    vTaskDelay(1000/portTICK_PERIOD_MS); gpio_set_level(POWER_OFF, 1);
  }
}

//display functions start

static void IRAM_ATTR powerOffBtnFn(void* arg) {
  powerOffStatus = true;
}

static void IRAM_ATTR down(void* arg) {
  if (esp_timer_get_time()/1000-btntmr>200) {
    btnID = 1;
  } btntmr = esp_timer_get_time()/1000;
  esp_timer_stop(periodic_timer);
  esp_timer_start_periodic(periodic_timer, IDLE_DELAY);
}

static void IRAM_ATTR selectFn(void* arg) {
  if (esp_timer_get_time()/1000-btntmr>200) {
    btnID = 2;
  } btntmr = esp_timer_get_time()/1000;
  esp_timer_stop(periodic_timer);
  esp_timer_start_periodic(periodic_timer, IDLE_DELAY);
}

static void IRAM_ATTR up(void* arg) {
  if (esp_timer_get_time()/1000-btntmr>200) {
    btnID = -1;
  } btntmr = esp_timer_get_time()/1000;
  esp_timer_stop(periodic_timer);
  esp_timer_start_periodic(periodic_timer, IDLE_DELAY);
}

static void IRAM_ATTR BL(void* arg) {
  if (esp_timer_get_time()/1000-btntmr>200) {
    BLLevel++;
    if (BLLevel>3) BLLevel = 0;
    if (BLLevel == 0) duty = 0;
    else if (BLLevel == 1) duty = 1638;
    else if (BLLevel == 2) duty = 4586;
    else if (BLLevel == 3) duty = 8191;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
  } btntmr = esp_timer_get_time()/1000;
  esp_timer_stop(periodic_timer);
  esp_timer_start_periodic(periodic_timer, IDLE_DELAY);
}

void list_files(const char *path) {
  long fileCount = 0, size = 0;
  DIR *dir = opendir(path);
  if (!dir) {
    printf("Failed to open directory\n");
    FSstats[0] = -1; return;
  } struct dirent *entry;
  struct stat file_info;
  while ((entry = readdir(dir)) != NULL) {
    char file_path[50];
    snprintf(file_path, sizeof(file_path), "%.19s/%.29s", path, entry->d_name);
    if (stat(file_path, &file_info) == 0) {
        snprintf(files[fileCount], sizeof(files[fileCount]), "%.29s", entry->d_name);
        filesModTime[fileCount++] = file_info.st_mtim.tv_sec; size+=file_info.st_size;
    } printf("%s\n", file_path);
  } closedir(dir); FSstats[0] = fileCount;
  FSstats[1] = size; FSstats[2] = 3538944-size;
  printf("%ld %ld %ld\n", FSstats[0], FSstats[1], FSstats[2]);
  for(int i=0;i<fileCount-1;i++) {
    for (int j=0;j<fileCount-i-1;j++) {
      if (filesModTime[j]<filesModTime[j+1]) {
        char temp[30]; strcpy(temp, files[j]);
        temp[strlen(files[j])] = '\0';
        strcpy(files[j], files[j+1]);
        files[j][strlen(files[j+1])] = '\0';
        strcpy(files[j+1], temp);
        files[j+1][strlen(temp)] = '\0';
        time_t tempModTime = filesModTime[j];
        filesModTime[j] = filesModTime[j+1];
        filesModTime[j+1] = tempModTime;
      }
    }
  }
}

void displayText(char *str, int cursorX, int cursorY, int onOff) {
  write_cmd(0x2B); // Set Row Address
  write_dat(0x00); write_dat(0x60+cursorY);
  write_dat(0x00); write_dat(0x67+cursorY);
  cursorX /= 3; cursorX += 48; int colCtr = 0;
  if (onOff) {
    while(*str) {
      write_cmd(0x2A); // Set Column Address
      write_dat(0x00); write_dat(cursorX+colCtr);
      write_dat(0x00); write_dat(cursorX+colCtr+1);
      write_cmd(0x2C); unsigned char temp1, temp2;
      uint8_t *bitmap = font5x7[*str++ - 0x10];
      for (int i=0;i<7;i++) {
        temp1 = 36, temp2 = 36;
        for (int j=0;j<5;j++) {
          if ((bitmap[j] >> i) & 0x01) {
            if (j==0)
              temp1+=192;
            else if (j==1)
              temp1+=24;
            else if (j==2)
              temp1+=3;
            else if (j==3)
              temp2+=192;
            else
              temp2+=24;
          }
        } write_dat(temp1); write_dat(temp2);
      } colCtr+=2;
    }
  } else {
    write_cmd(0x2A); // Set Column Address
    write_dat(0x00); write_dat(cursorX);
    write_dat(0x00); write_dat(cursorX+1);
    write_cmd(0x2C);
    for (int i=0;i<16;i++) write_dat(0);
  }
}

void clearDisplay() {
  write_cmd(0x2A); // Set Column Address
  write_dat(0x00); write_dat(48);
  write_dat(0x00); write_dat(127);
  write_cmd(0x2B); // Set Row Address
  write_dat(0x00); write_dat(96);
  write_dat(0x00); write_dat(159);
  write_cmd(0x2C); // write display data
  for (int i = 0; i < 64; i++) {
    for (int j = 0; j < 80; j++)
      write_dat(0);
  }
}

void write_dat(unsigned char data) {
  gpio_set_level(LCM_CS, 0);
  gpio_set_level(LCM_A0, 1);
  gpio_set_level(D0, (data >> 0) & 0x01);
  gpio_set_level(D1, (data >> 1) & 0x01);
  gpio_set_level(D2, (data >> 2) & 0x01);
  gpio_set_level(D3, (data >> 3) & 0x01);
  gpio_set_level(D4, (data >> 4) & 0x01);
  gpio_set_level(D5, (data >> 5) & 0x01);
  gpio_set_level(D6, (data >> 6) & 0x01);
  gpio_set_level(D7, (data >> 7) & 0x01);
  gpio_set_level(LCM_ENABLE, 0);
  esp_rom_delay_us(2);
  gpio_set_level(LCM_ENABLE, 1);
  gpio_set_level(LCM_A0, 1);
  gpio_set_level(LCM_CS, 1);
}

void write_cmd(unsigned char cmd) {
  gpio_set_level(LCM_CS, 0);
  gpio_set_level(LCM_A0, 0);
  gpio_set_level(D0, (cmd >> 0) & 0x01);
  gpio_set_level(D1, (cmd >> 1) & 0x01);
  gpio_set_level(D2, (cmd >> 2) & 0x01);
  gpio_set_level(D3, (cmd >> 3) & 0x01);
  gpio_set_level(D4, (cmd >> 4) & 0x01);
  gpio_set_level(D5, (cmd >> 5) & 0x01);
  gpio_set_level(D6, (cmd >> 6) & 0x01);
  gpio_set_level(D7, (cmd >> 7) & 0x01);
  gpio_set_level(LCM_ENABLE, 0);
  esp_rom_delay_us(2);
  gpio_set_level(LCM_ENABLE, 1);
  gpio_set_level(LCM_A0, 1);
  gpio_set_level(LCM_CS, 1);
}

void LCD_initial() {
    gpio_set_level(LCM_RESET, 1);
    vTaskDelay(120/portTICK_PERIOD_MS);
    gpio_set_level(LCM_RESET, 0);
    vTaskDelay(10/portTICK_PERIOD_MS);
    gpio_set_level(LCM_RESET, 1);
    vTaskDelay(120/portTICK_PERIOD_MS);
    write_cmd(0xD7); // Disable Auto Read
    write_dat(0x9F);
    write_cmd(0xE0); // Enable OTP Read
    write_dat(0x00);
    vTaskDelay(10/portTICK_PERIOD_MS);
    write_cmd(0xE3); // OTP Up-Load
    vTaskDelay(20/portTICK_PERIOD_MS);
    write_cmd(0xE1); // OTP Control Out
    write_cmd(0x11); // Sleep Out
    write_cmd(0x28); // Display OFF
    vTaskDelay(50/portTICK_PERIOD_MS);
    write_cmd(0xC0); // Set Vop
    write_dat(0x9E);
    write_dat(0x00);
    write_cmd(0xC3); // BIAS System
    write_dat(0x05);
    write_cmd(0xC4); // Booster Level
    write_dat(0x05);
    write_cmd(0xD0); // Enable Analog Circuit
    write_dat(0x1D);
    write_cmd(0xB5); // N-Line Inversion
    write_dat(0x00);
    write_cmd(0x39); // Display Mode
    write_cmd(0xF1); // Frame Rate (Monochrome Mode)
    write_dat(0x06);
    write_dat(0x0B);
    write_dat(0x0D);
    write_dat(0x10);
    write_cmd(0x3A); // Enable DDRAM Interface
    write_dat(0x02);
    write_cmd(0x36); // Display Control
    write_dat(0xC8);
    write_cmd(0xB0); // Display Duty
    write_dat(0x3F);
    write_cmd(0x20); // Inverse Display
    write_cmd(0x37); // Start Line
    write_dat(0x00);
    write_cmd(0xB1); // First Output COM
    write_dat(0x00);
    write_cmd(0xB3); // FOSC Divider
    write_dat(0x01);
    write_cmd(0x2A); // Set Column Address
    write_dat(0x00);
    write_dat(0x30);
    write_dat(0x00);
    write_dat(0x7f);
    write_cmd(0x2B); // Set Row Address
    write_dat(0x00);
    write_dat(0x60);
    write_dat(0x00);
    write_dat(0x9f);
    write_cmd(0xC4); // Booster Level
    write_dat(0x07);
    write_cmd(0x29); // Display ON
}

void mainMenu() {
  while(1) {
    clearDisplay();
    displayText("Drafts", 9, 2, 1);
    displayText("Bluetooth", 9, 12, 1);
    displayText("WiFi", 9, 22, 1);
    displayText("Document Sync", 9, 32, 1);
    displayText("Font Size", 9, 42, 1);
    displayText("Return", 9, 52, 1);
    int ctr = mainMenuCursor;
    if (ctr == 5) ctr = 0;
    displayText(">", 0, (ctr*10)+2, 1);
    while (btnID!=2) {
      if (btnID==-1 || btnID==1) {
        if (btnID == 1) printf("ISR down triggered\n");
        else if (btnID == -1) printf("ISR up triggered\n");
        displayText(">", 0, (ctr*10)+2, 0);
        ctr += btnID; 
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (ctr>5) ctr = 0;
        else if (ctr<0) ctr = 5;
        displayText(">", 0, (ctr*10)+2, 1);
      } else vTaskDelay(100/portTICK_PERIOD_MS);
    } 
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    }
    mainMenuCursor = ctr;
    if (ctr == 0)
      drafts();
    else if (ctr == 1)
      BT();
    else if (ctr == 2)
      WIFI();
    else if (ctr == 3)
      docSync();
    else if (ctr == 4)
      fontSize();
    else if (ctr == 5)
      writeMode(0);
  }
}

void drafts() {
  while(1) {
    clearDisplay();
    displayText("Load Draft", 9, 12, 1);
    displayText("New Draft", 9, 22, 1);
    displayText("Delete Draft", 9, 32, 1);
    displayText("Return", 9, 42, 1);
    int ctr = 0; displayText(">", 0, 12, 1);
    while (btnID!=2) {
        if (btnID==-1 || btnID==1) {
          displayText(">", 0, (ctr*10)+12, 0);
          ctr += btnID; 
          if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
            btnID = 0; xSemaphoreGive(xbtnIDMutex);
          }
          if (ctr>3) ctr = 0;
          else if (ctr<0) ctr = 3;
          displayText(">", 0, (ctr*10)+12, 1);
        } else vTaskDelay(100/portTICK_PERIOD_MS);
    } 
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    }
    if (ctr==0)
      openDoc();
    else if (ctr==1)
      newDoc();
    else if (ctr==2)
      delDoc();
    else
      return;
  }
}

void newDoc() {
  clearDisplay();
  displayText("Enter Draft name", 9, 12, 1);
  displayText("OK", 9, 32, 1);
  displayText("Cancel", 9, 42, 1);
  int ctr = 0; char newDocName[26] = "";
  displayText(">", 0, 32, 1);
  unsigned int received_value = 0;
  charInput = true;
  while (1) {
    if (xQueueReceive(key_print_queue, &received_value, 0) == pdTRUE) {
      unsigned int c = received_value; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if ((c >=48 && c <=57) || (c >=65 && c <=90) || (c >=97 && c <=122)) {
        if (strlen(newDocName)<25) {
          newDocName[strlen(newDocName)] = (char)c;
          newDocName[strlen(newDocName)+1] = '\0';
          displayText(newDocName, 9, 22, 1);
        }
      }
    } else if (btnID==-4) {
      if (strlen(newDocName)>0) {
        write_cmd(0x2B); // Set Row Address
        write_dat(0x00); write_dat(0x60+22);
        write_dat(0x00); write_dat(0x66+28);
        write_cmd(0x2A); // Set Column Address
        write_dat(0x00); write_dat(49+strlen(newDocName)*2);
        write_dat(0x00); write_dat(50+strlen(newDocName)*2);
        write_cmd(0x2C); for(int i=0;i<14;i++) write_dat(0);
        newDocName[strlen(newDocName)-1] = '\0';
        displayText(newDocName, 9, 22, 1);
      } 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
    } else if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+32, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }

      if (ctr>1) ctr = 0;
      else if (ctr<0) ctr = 1;
      displayText(">", 0, (ctr*10)+32, 1);
    } else if (btnID==2) {
      if (ctr==0 && strlen(newDocName)<3) {
        displayText("Enter file name of min. length 3", 27, 52, 1);
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
      } else break;
    } vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  if (ctr==0) newDocCreate(newDocName);
  else return;
}

void newDocCreate(char *docName) {
  charInput = false;
  char path[50];
  snprintf(path, sizeof(path), "/spiflash/myfiles/%s.TXT", docName);
  clearDisplay(); struct stat st;
  if (stat(path, &st) == 0) {
    displayText(docName, 0, 12, 1);
    displayText(".TXT already exists", strlen(docName)*6, 12, 1);
  } else {
    if (FSstats[2]<=4096) {
      displayText("Failed to create file", 9, 12, 1);
      displayText("Not enough storage", 9, 22, 1);
      displayText("Please delete some files", 9, 32, 1);
    } else {
      FILE *file = fopen(path, "w");
      if (file == NULL)
        displayText("Failed to create file", 9, 22, 1);
      else {
        printf("File created\n"); fclose(file);
        displayText("File created", 0, 22, 1);
        displayText("Opening file...", 0, 32, 1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        list_files("/spiflash/myfiles");
        vTaskDelay(100/portTICK_PERIOD_MS);
        list_files("/spiflash/myfiles");
        writeMode(0);
      }
    }
  } vTaskDelay(3000/portTICK_PERIOD_MS); return;
}

void openDoc() {
  list_files("/spiflash/myfiles"); clearDisplay();
  int filesCnt = FSstats[0], pageCnt = 0, pageNo = 0;
  int curFileCnt = 0, ctr = 0; bool pageChange = true;
  if (filesCnt<=0) {
    displayText("No drafts available", 9, 12, 1);
    displayText("Return", 9, 52, 1);
    displayText(">", 0, 52, 1);
    while (btnID!=2) vTaskDelay(100/portTICK_PERIOD_MS);
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    } return;
  } pageCnt = filesCnt/5 -1;
  if (filesCnt%5!=0) pageCnt += 1;
  while (btnID!=2 || (pageNo!=pageCnt && ctr==curFileCnt)) {
    if (pageChange) {
      pageChange = false; clearDisplay();
      curFileCnt = (pageNo==pageCnt)?((filesCnt%5==0)?5:filesCnt%5):5;
      printf("Loading %d files\n", curFileCnt);
      for (int i=0;i<curFileCnt;i++)
        displayText(files[(pageNo*5)+i], 9, (10*i)+2, 1);
      if (pageNo==pageCnt)
        displayText("Return", 9, (curFileCnt*10)+2, 1);
      else
        displayText("vvvvvvvvvvvvvvv", 9, 52, 1);
      displayText(">", 0, (ctr*10)+2, 1);
    }
    if (btnID!=0) {
      if (btnID==-1 || btnID==1) {
        displayText(">", 0, (ctr*10)+2, 0);
        ctr += btnID; 
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (ctr>curFileCnt) {
          ctr = 0; pageNo+=1; pageChange = true;
          if (pageNo>pageCnt) pageNo = 0;
        } else if (ctr<0) {
          ctr = 5; pageNo-=1; pageChange = true;
          if (pageNo<0) {
            pageNo = pageCnt;
            ctr = (filesCnt%5==0)?5:filesCnt%5;
          }
        }
      } else {
        if (pageNo!=pageCnt && ctr==curFileCnt) {
          ctr = 0; 
          if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
            btnID = 0; xSemaphoreGive(xbtnIDMutex);
          }
          pageNo+=1; pageChange = true;
          if (pageNo>pageCnt) pageNo = 0;
        }
      } displayText(">", 0, (ctr*10)+2, 1);
    } vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  if ((pageNo*5)+ctr==filesCnt)
    printf("Return to main menu\n");
  else {
    printf("Opening %s...\n\n", files[(pageNo*5)+ctr]);
    writeMode((pageNo*5)+ctr);
  } return;
}

void delDoc() {
  list_files("/spiflash/myfiles"); clearDisplay();
  int filesCnt = FSstats[0], pageCnt = 0, pageNo = 0;
  int curFileCnt = 0, ctr = 0; bool pageChange = true;
  if (filesCnt<=0) {
    displayText("No drafts available", 9, 12, 1);
    displayText("Return", 9, 52, 1);
    displayText(">", 0, 52, 1);
    while (btnID!=2) vTaskDelay(100/portTICK_PERIOD_MS);
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    } return;
  } pageCnt = filesCnt/5 -1;
  if (filesCnt%5!=0) pageCnt += 1;
  while (btnID!=2 || (pageNo!=pageCnt && ctr==curFileCnt)) {
    if (pageChange) {
      pageChange = false; clearDisplay();
      curFileCnt = (pageNo==pageCnt)?((filesCnt%5==0)?5:filesCnt%5):5;
      printf("Loading %d files\n", curFileCnt);
      for (int i=0;i<curFileCnt;i++)
        displayText(files[(pageNo*5)+i], 9, (10*i)+2, 1);
      if (pageNo==pageCnt)
        displayText("Return", 9, (curFileCnt*10)+2, 1);
      else
        displayText("vvvvvvvvvvvvvvv", 9, 52, 1);
      displayText(">", 0, (ctr*10)+2, 1);
    }
    if (btnID!=0) {
      if (btnID==-1 || btnID==1) {
        displayText(">", 0, (ctr*10)+2, 0);
        ctr += btnID; 
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (ctr>curFileCnt) {
          ctr = 0; pageNo+=1; pageChange = true;
          if (pageNo>pageCnt) pageNo = 0;
        }
        else if (ctr<0) {
          ctr = 5; pageNo-=1; pageChange = true;
          if (pageNo<0) {
            pageNo = pageCnt;
            ctr = (filesCnt%5==0)?5:filesCnt%5;
          }
        }
      } else {
        if (pageNo!=pageCnt && ctr==curFileCnt) {
          ctr = 0; 
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        pageNo+=1; pageChange = true;
        if (pageNo>pageCnt) pageNo = 0;
        }
      } displayText(">", 0, (ctr*10)+2, 1);
    } vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  if ((pageNo*5)+ctr==filesCnt) printf("Return to main menu\n");
  else {
    int xctr = 0; clearDisplay();
    displayText("Are you sure you want to", 9, 12, 1);
    displayText("delete this file?", 9, 22, 1);
    displayText("Yes", 9, 32, 1);
    displayText("No", 9, 42, 1);
    displayText(">", 0, 32, 1);
    while (btnID!=2) {
      if (btnID==-1 || btnID==1) {
        displayText(">", 0, (xctr*10)+32, 0);
        xctr += btnID; 
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (xctr>1) xctr = 0;
        else if (xctr<0) xctr = 1;
        displayText(">", 0, (xctr*10)+32, 1);
      } else vTaskDelay(100/portTICK_PERIOD_MS);
    } 
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    }
    if (xctr==0) {
      char delFile[50]; clearDisplay();
      snprintf(delFile, sizeof(delFile), "/spiflash/myfiles/%s", files[(pageNo*5)+ctr]);
      if (remove(delFile)==0) {
        displayText(files[(pageNo*5)+ctr], 0, 22, 1);
        displayText("deleted", 3+strlen(files[(pageNo*5)+ctr])*6, 22, 1);
        printf("Deleted %s\n", files[(pageNo*5)+ctr]);
      } else {
        displayText("Failed to delete", 9, 12, 1);
        displayText(files[(pageNo*5)+ctr], 9, 22, 1);
      } vTaskDelay(2000/portTICK_PERIOD_MS);
    }
  } return;
}

void BT() {
  int ctr = 0;
  while(ctr!=2) {
    clearDisplay();
    if (btStatus) {
      displayText(btDevName, 9, 12, 1);
      displayText("Disconnect device", 9, 22, 1);
    } else {
      displayText("No device connected", 9, 12, 1);
      displayText("Connect device", 9, 22, 1);
    }
    displayText("Return", 9, 32, 1);
    ctr = 0; displayText(">", 0, 22, 1);
    while (btnID!=2) {
      if (btnID==-1 || btnID==1) {
        displayText(">", 0, (ctr*10)+22, 0);
        ctr += btnID; 
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (ctr>1) ctr = 0;
        else if (ctr<0) ctr = 1;
        displayText(">", 0, (ctr*10)+22, 1);
      } else vTaskDelay(100/portTICK_PERIOD_MS);
    } 
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    }
    if (ctr==0)
      if (btStatus) xTaskNotifyGive(hid_demo_task_handle);
      else connectNewBT();
    else
      return;
  } return;
}

void connectNewBT() {
  xTaskNotifyGive(hid_demo_task_handle);
  clearDisplay();
  displayText("Scanning...", 0, 2, 1);
  displayText("Return", 9, 52, 1);
  displayText(">", 0, 52, 1);
  btWaitStatus = true;
  ESP_LOGI("Kuch bhi", "BT scan waiting for 15sec\n");
  ulTaskNotifyTake(pdTRUE, 15000/portTICK_PERIOD_MS);
  btWaitStatus = false;
  if (btnID!=2) {
    int pageCnt = 0, pageNo = 0, curDevCnt = 0, ctr = 0;
    int devCount = results_len;
    bool pageChange = true;
    esp_hid_scan_result_t *devList;
    clearDisplay();
    if (devCount<=0) {
      displayText("No devices available", 9, 12, 1);
      displayText("Return", 9, 52, 1);
      displayText(">", 0, 52, 1);
      while (btnID!=2) vTaskDelay(100/portTICK_PERIOD_MS);
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      } return;
    } pageCnt = devCount/5 -1;
    if (devCount%5!=0) pageCnt += 1;
    while (btnID!=2 || (pageNo!=pageCnt && ctr==curDevCnt)) {
      if (pageChange) {
        pageChange = false; clearDisplay();
        curDevCnt = (pageNo==pageCnt)?((devCount%5==0)?5:devCount%5):5;
        printf("Loading %d devices\n", curDevCnt);
        devList = scan_results; //List of BT Devices
        for (int i=0;i<curDevCnt;i++) {
          char devName[81];
          for (int i=0;i<(pageNo*5);i++) devList = devList->next;
          snprintf(devName, sizeof(devName), "%.20s "ESP_BD_ADDR_STR, devList->name?devList->name:" ", ESP_BD_ADDR_HEX(devList->bda));
          devList = devList->next;
          displayText(devName, 9, (10*i)+2, 1);
        }
        if (pageNo==pageCnt)
          displayText("Return", 9, (curDevCnt*10)+2, 1);
        else
          displayText("vvvvvvvvvvvvvvv", 9, 52, 1);
        displayText(">", 0, (ctr*10)+2, 1);
      }
      if (btnID!=0) {
        if (btnID==-1 || btnID==1) {
          displayText(">", 0, (ctr*10)+2, 0);
          ctr += btnID; 
          if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
            btnID = 0; xSemaphoreGive(xbtnIDMutex);
          }
          if (ctr>curDevCnt) {
            ctr = 0; pageNo+=1; pageChange = true;
            if (pageNo>pageCnt) pageNo = 0;
          } else if (ctr<0) {
            ctr = 5; pageNo-=1; pageChange = true;
            if (pageNo<0) {
              pageNo = pageCnt;
              ctr = (devCount%5==0)?5:devCount%5;
            }
          }
        } else {
          if (pageNo!=pageCnt && ctr==curDevCnt) {
            ctr = 0; 
            if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
              btnID = 0; xSemaphoreGive(xbtnIDMutex);
            }
            pageNo+=1; pageChange = true;
            if (pageNo>pageCnt) pageNo = 0;
          }
        } displayText(">", 0, (ctr*10)+2, 1);
      } vTaskDelay(100/portTICK_PERIOD_MS);
    } 
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    }
    if ((pageNo*5)+ctr==devCount) {
      xTaskNotifyGive(hid_demo_task_handle);
      printf("Return to main menu\n");
    } else {
      devList = scan_results;
      for (int i=0;i<(pageNo*5)+ctr;i++) devList = devList->next;
      snprintf(btDevName, sizeof(btDevName), "%.35s", devList->name?devList->name:"Unnamed device");
      printf("Device %d is selected\n\n", (pageNo*5)+ctr+1);
      selected_index = (pageNo*5)+ctr;
      xTaskNotifyGive(hid_demo_task_handle);
      clearDisplay();
      displayText("Generating passkey...", 9, 22, 1);
      displayText(">", 0, 32, 1);
      displayText("Return", 9, 32, 1);
      btWaitStatus = true;
      ulTaskNotifyTake(pdTRUE, 15000/portTICK_PERIOD_MS);
      btWaitStatus = false;
      if (btnID!=2) {
        clearDisplay();
        displayText("Enter this passkey on the keyboard", 9, 22, 1);
        char PK[7]; 
        snprintf(PK, sizeof(PK), "%06ld", ble_passkey);
        displayText(PK, 9, 32, 1);
        displayText(">", 0, 42, 1);
        displayText("Return", 9, 42, 1);
        btWaitStatus = true;
        ulTaskNotifyTake(pdTRUE, 20000/portTICK_PERIOD_MS);
        btWaitStatus = false;
        if (btnID!=2) {
          // btStatus = true;
        }
        else {
          if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
            btnID = 0; xSemaphoreGive(xbtnIDMutex);
          }
        }
      }
      else {
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        } return;
      }
    } return;
  }
  else {
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    } return;
  }
}

void WIFI() {
  int ctr = 0;
  while(ctr!=2) {
    clearDisplay();
    if (wifiStatus) {
      displayText("WiFi connected to ", 9, 12, 1);
      displayText(EXAMPLE_ESP_WIFI_SSID, 117, 12, 1);
      displayText("Disconnect network", 9, 22, 1);
    } else {
      displayText("No network connected", 9, 12, 1);
      displayText("Connect network", 9, 22, 1);
    } displayText("Return", 9, 32, 1);
    displayText("Firmare Version: ", 0, 52, 1);
    displayText(running_fw_version, 102, 52, 1);
    ctr = 0; displayText(">", 0, (ctr*10)+22, 1);
    while (btnID!=2) {
      if (btnID==-1 || btnID==1) {
        displayText(">", 0, (ctr*10)+22, 0);
        ctr += btnID; 
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (ctr>1) ctr = 0;
        else if (ctr<0) ctr = 1;
        displayText(">", 0, (ctr*10)+22, 1);
      } else vTaskDelay(100/portTICK_PERIOD_MS);
    } 
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    }
    if (ctr==0) {
      if (wifiStatus) wifiStatus = false; // Disconnect WIFI
      else connectNewWIFI();
    } else
      return;
  } return;
}

void connectNewWIFI() {
  clearDisplay();
  displayText("Scanning...", 0, 2, 1);
  vTaskDelay(3000/portTICK_PERIOD_MS);
  clearDisplay();
  displayText("No available networks", 9, 22, 1);
  displayText("Return", 9, 52, 1);
  displayText(">", 0, 52, 1);
  while (btnID!=2) vTaskDelay(100/portTICK_PERIOD_MS);
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  } wifiStatus = true; return;
}
 
void docSync() {
  int ctr = 0;
  while(ctr!=2) {
    clearDisplay();
    if (syncStatus) {
      displayText("Syncing to ", 9, 22, 1);
      if (gDriveStatus)
        displayText("Google Drive", 75, 22, 1);
      else if (oneDriveStatus)
        displayText("One Drive", 75, 22, 1);
      else if (dropBoxStatus)
        displayText("Dropbox", 75, 22, 1);
      else
        displayText("nothing lol", 75, 22, 1);
    } else
      displayText("Not syncing", 9, 22, 1);
    displayText("Setup sync ", 9, 32, 1);
    displayText("Return", 9, 42, 1);
    ctr = 0; displayText(">", 0, 22, 1);
    while (btnID!=2) {
      if (btnID==-1 || btnID==1) {
        displayText(">", 0, (ctr*10)+22, 0);
        ctr += btnID; 
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (ctr>2) ctr = 0;
        else if (ctr<0) ctr = 2;
        displayText(">", 0, (ctr*10)+22, 1);
      } else vTaskDelay(100/portTICK_PERIOD_MS);
    } 
    if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
      btnID = 0; xSemaphoreGive(xbtnIDMutex);
    }
    if (ctr==0)
      syncOnOff();
    else if (ctr==1)
      setupSync();
    else
      return;
  } return;
}

void syncOnOff() {
  clearDisplay();
  displayText("Cancel", 9, 32, 1);
  if (syncStatus)
    displayText("Turn OFF Sync", 9, 22, 1);
  else
    displayText("Turn ON Sync", 9, 22, 1);
  int ctr = 0;
  displayText(">", 0, 22, 1);
  while (btnID!=2) {
    if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+22, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if (ctr>1) ctr = 0;
      else if (ctr<0) ctr = 1;
      displayText(">", 0, (ctr*10)+22, 1);
    } else vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  if (ctr==0) {
    syncStatus = !syncStatus; clearDisplay();
    // Set sync status accordingly
    if (syncStatus)
      displayText("Syncing turned ON", 0, 22, 1);
    else {
      displayText("Syncing turned OFF", 0, 22, 1);
      gDriveStatus = false;
      oneDriveStatus = false;
      dropBoxStatus = false;
    } vTaskDelay(1000/portTICK_PERIOD_MS);
  } return;
}

void setupSync() {
  clearDisplay();
  displayText("Google Drive", 9, 12, 1);
  displayText("Microsoft Onedrive", 9, 22, 1);
  displayText("Dropbox", 9, 32, 1);
  displayText("Return", 9, 42, 1);
  int ctr = 0; displayText(">", 0, 12, 1);
  while (btnID!=2) {
    if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+12, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if (ctr>3) ctr = 0;
      else if (ctr<0) ctr = 3;
      displayText(">", 0, (ctr*10)+12, 1);
    } else vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  if (ctr==0)
    gDriveConnect();
  else if (ctr==1)
    oneDriveConnect();
  else if (ctr==2)
    dropBoxConnect();
  return;
}

void gDriveConnect() {
  clearDisplay();
  displayText("Google Drive", 9, 2, 1);
  displayText("Enter Email ID", 9, 12, 1);
  displayText("OK", 9, 32, 1);
  displayText("Return", 9, 42, 1);
  int ctr = 0; displayText(">", 0, 32, 1);
  memset(gDriveUsername, 0, sizeof(gDriveUsername));
  unsigned int received_value = 0; charInput = true;
  while (1) {
    if (xQueueReceive(key_print_queue, &received_value, 0) == pdTRUE) {
      unsigned int c = received_value; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if ((c==46) || (c >=48 && c <=57) || (c >=64 && c <=90) || (c >=97 && c <=122)) {
        if (strlen(gDriveUsername)<37) {
          gDriveUsername[strlen(gDriveUsername)] = (char)c;
          gDriveUsername[strlen(gDriveUsername)+1] = '\0';
          displayText(gDriveUsername, 9, 22, 1);
        }
      }
    } else if (btnID==-4) {
      if (strlen(gDriveUsername)>0) {
        write_cmd(0x2B); // Set Row Address
        write_dat(0x00); write_dat(0x60+22);
        write_dat(0x00); write_dat(0x66+28);
        write_cmd(0x2A); // Set Column Address
        write_dat(0x00); write_dat(49+strlen(gDriveUsername)*2);
        write_dat(0x00); write_dat(50+strlen(gDriveUsername)*2);
        write_cmd(0x2C); for(int i=0;i<14;i++) write_dat(0);
        gDriveUsername[strlen(gDriveUsername)-1] = '\0';
        displayText(gDriveUsername, 9, 22, 1);
      } 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
    } else if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+32, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if (ctr>1) ctr = 0;
      else if (ctr<0) ctr = 1;
      displayText(">", 0, (ctr*10)+32, 1);
    } else if (btnID==2) {
      if (ctr==0 && strlen(gDriveUsername)<10) {
        displayText("Enter valid Email ID", 60, 52, 1);
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
      } else break;
    } vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  charInput = false;
  if (ctr==0) {
    vTaskDelay(500/portTICK_PERIOD_MS);
    gDriveConnectPwd(gDriveUsername);
  } return;
}

void gDriveConnectPwd(char *username) {
  clearDisplay();
  displayText("Google Drive", 9, 2, 1);
  displayText("Enter Password", 9, 12, 1);
  displayText("OK", 9, 32, 1);
  displayText("Return", 9, 42, 1);
  int ctr = 0; displayText(">", 0, 32, 1);
  memset(gDrivePwd, 0, sizeof(gDrivePwd));
  unsigned int received_value = 0;
  charInput = true;
  while (1) {
    if (xQueueReceive(key_print_queue, &received_value, 0) == pdTRUE) {
      unsigned int c = received_value; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if ((c==46) || (c >=48 && c <=57) || (c >=64 && c <=90) || (c >=97 && c <=122)) {
        if (strlen(gDrivePwd)<37) {
          gDrivePwd[strlen(gDrivePwd)] = (char)c;
          gDrivePwd[strlen(gDrivePwd)+1] = '\0';
          displayText(gDrivePwd, 9, 22, 1);
        }
      }
    } else if (btnID==-4) {
      if (strlen(gDrivePwd)>0) {
        write_cmd(0x2B); // Set Row Address
        write_dat(0x00); write_dat(0x60+22);
        write_dat(0x00); write_dat(0x66+28);
        write_cmd(0x2A); // Set Column Address
        write_dat(0x00); write_dat(49+strlen(gDrivePwd)*2);
        write_dat(0x00); write_dat(50+strlen(gDrivePwd)*2);
        write_cmd(0x2C); for(int i=0;i<14;i++) write_dat(0);
        gDrivePwd[strlen(gDrivePwd)-1] = '\0';
        displayText(gDrivePwd, 9, 22, 1);
      } 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
    } else if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+32, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if (ctr>1) ctr = 0;
      else if (ctr<0) ctr = 1;
      displayText(">", 0, (ctr*10)+32, 1);
    } else if (btnID==2) {
      if (ctr==0 && strlen(gDrivePwd)<6) {
        displayText("Password should be atleast 6 letters", 12, 52, 1);
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
      } else break;
    } vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  charInput = false;
  if (ctr==0) {
    // Code to connect to gDrive
    vTaskDelay(500/portTICK_PERIOD_MS);
    clearDisplay();
    displayText("Logged in to Google Drive", 0, 22, 1);
    syncStatus = true;
    gDriveStatus = true;
    oneDriveStatus = false;
    dropBoxStatus = false;
    vTaskDelay(1000/portTICK_PERIOD_MS);
  } return;
}

void oneDriveConnect() {
  clearDisplay();
  displayText("One Drive", 9, 2, 1);
  displayText("Enter Email ID", 9, 12, 1);
  displayText("OK", 9, 32, 1);
  displayText("Return", 9, 42, 1);
  int ctr = 0; displayText(">", 0, 32, 1);
  memset(oneDriveUsername, 0, sizeof(oneDriveUsername));
  unsigned int received_value = 0;
  charInput = true;
  while (1) {
    if (xQueueReceive(key_print_queue, &received_value, 0) == pdTRUE) {
      unsigned int c = received_value; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if ((c==46) || (c >=48 && c <=57) || (c >=64 && c <=90) || (c >=97 && c <=122)) {
        if (strlen(oneDriveUsername)<37) {
          oneDriveUsername[strlen(oneDriveUsername)] = (char)c;
          oneDriveUsername[strlen(oneDriveUsername)+1] = '\0';
          displayText(oneDriveUsername, 9, 22, 1);
        }
      }
    } else if (btnID==-4) {
      if (strlen(oneDriveUsername)>0) {
        write_cmd(0x2B); // Set Row Address
        write_dat(0x00); write_dat(0x60+22);
        write_dat(0x00); write_dat(0x66+28);
        write_cmd(0x2A); // Set Column Address
        write_dat(0x00); write_dat(49+strlen(oneDriveUsername)*2);
        write_dat(0x00); write_dat(50+strlen(oneDriveUsername)*2);
        write_cmd(0x2C); for(int i=0;i<14;i++) write_dat(0);
        oneDriveUsername[strlen(oneDriveUsername)-1] = '\0';
        displayText(oneDriveUsername, 9, 22, 1);
      } 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
    } else if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+32, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if (ctr>1) ctr = 0;
      else if (ctr<0) ctr = 1;
      displayText(">", 0, (ctr*10)+32, 1);
    } else if (btnID==2) {
      if (ctr==0 && strlen(oneDriveUsername)<10) {
        displayText("Enter valid Email ID", 60, 52, 1);
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
      } else break;
    } vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  charInput = false;
  if (ctr==0) {
    vTaskDelay(500/portTICK_PERIOD_MS);
    oneDriveConnectPwd(oneDriveUsername);
  } return;
}

void oneDriveConnectPwd(char *username) {
  clearDisplay();
  displayText("One Drive", 9, 2, 1);
  displayText("Enter Password", 9, 12, 1);
  displayText("OK", 9, 32, 1);
  displayText("Return", 9, 42, 1);
  int ctr = 0; displayText(">", 0, 32, 1);
  memset(oneDrivePwd, 0, sizeof(oneDrivePwd));
  unsigned int received_value = 0;
  charInput = true;
  while (1) {
    if (xQueueReceive(key_print_queue, &received_value, 0) == pdTRUE) {
      unsigned int c = received_value; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if ((c==46) || (c >=48 && c <=57) || (c >=64 && c <=90) || (c >=97 && c <=122)) {
        if (strlen(oneDrivePwd)<37) {
          oneDrivePwd[strlen(oneDrivePwd)] = (char)c;
          oneDrivePwd[strlen(oneDrivePwd)+1] = '\0';
          displayText(oneDrivePwd, 9, 22, 1);
        }
      }
    } else if (btnID==-4) {
      if (strlen(oneDrivePwd)>0) {
        write_cmd(0x2B); // Set Row Address
        write_dat(0x00); write_dat(0x60+22);
        write_dat(0x00); write_dat(0x66+28);
        write_cmd(0x2A); // Set Column Address
        write_dat(0x00); write_dat(49+strlen(oneDrivePwd)*2);
        write_dat(0x00); write_dat(50+strlen(oneDrivePwd)*2);
        write_cmd(0x2C); for(int i=0;i<14;i++) write_dat(0);
        oneDrivePwd[strlen(oneDrivePwd)-1] = '\0';
        displayText(oneDrivePwd, 9, 22, 1);
      } 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
    } else if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+32, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if (ctr>1) ctr = 0;
      else if (ctr<0) ctr = 1;
      displayText(">", 0, (ctr*10)+32, 1);
    } else if (btnID==2) {
      if (ctr==0 && strlen(oneDrivePwd)<6) {
        displayText("Password should be atleast 6 letters", 12, 52, 1);
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
      } else break;
    } vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  charInput = false;
  if (ctr==0) {
    // Code to connect to oneDrive
    vTaskDelay(500/portTICK_PERIOD_MS);
    clearDisplay();
    displayText("Logged in to One Drive", 0, 22, 1);
    syncStatus = true;
    gDriveStatus = false;
    oneDriveStatus = true;
    dropBoxStatus = false;
    vTaskDelay(1000/portTICK_PERIOD_MS);
  } return;
}

void dropBoxConnect() {
  clearDisplay();
  displayText("Drop Box Drive", 9, 2, 1);
  displayText("Enter Email ID", 9, 12, 1);
  displayText("OK", 9, 32, 1);
  displayText("Return", 9, 42, 1);
  int ctr = 0; displayText(">", 0, 32, 1);
  memset(dropBoxUsername, 0, sizeof(dropBoxUsername));
  unsigned int received_value = 0;
  charInput = true;
  while (1) {
    if (xQueueReceive(key_print_queue, &received_value, 0) == pdTRUE) {
      unsigned int c = received_value; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if ((c==46) || (c >=48 && c <=57) || (c >=64 && c <=90) || (c >=97 && c <=122)) {
        if (strlen(dropBoxUsername)<37) {
          dropBoxUsername[strlen(dropBoxUsername)] = (char)c;
          dropBoxUsername[strlen(dropBoxUsername)+1] = '\0';
          displayText(dropBoxUsername, 9, 22, 1);
        }
      }
    } else if (btnID==-4) {
      if (strlen(dropBoxUsername)>0) {
        write_cmd(0x2B); // Set Row Address
        write_dat(0x00); write_dat(0x60+22);
        write_dat(0x00); write_dat(0x66+28);
        write_cmd(0x2A); // Set Column Address
        write_dat(0x00); write_dat(49+strlen(dropBoxUsername)*2);
        write_dat(0x00); write_dat(50+strlen(dropBoxUsername)*2);
        write_cmd(0x2C); for(int i=0;i<14;i++) write_dat(0);
        dropBoxUsername[strlen(dropBoxUsername)-1] = '\0';
        displayText(dropBoxUsername, 9, 22, 1);
      } 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
    } else if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+32, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if (ctr>1) ctr = 0;
      else if (ctr<0) ctr = 1;
      displayText(">", 0, (ctr*10)+32, 1);
    } else if (btnID==2) {
      if (ctr==0 && strlen(dropBoxUsername)<10) {
        displayText("Enter valid Email ID", 60, 52, 1);
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
      } else break;
    } vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  charInput = false;
  if (ctr==0) {
    vTaskDelay(500/portTICK_PERIOD_MS);
    dropBoxConnectPwd(dropBoxUsername);
  } return;
}

void dropBoxConnectPwd(char *username) {
  clearDisplay();
  displayText("Drop Box Drive", 9, 2, 1);
  displayText("Enter Password", 9, 12, 1);
  displayText("OK", 9, 32, 1);
  displayText("Return", 9, 42, 1);
  int ctr = 0; displayText(">", 0, 32, 1);
  memset(dropBoxPwd, 0, sizeof(dropBoxPwd));
  unsigned int received_value = 0;
  charInput = true;
  while (1) {
    if (xQueueReceive(key_print_queue, &received_value, 0) == pdTRUE) {
      unsigned int c = received_value; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if ((c==46) || (c >=48 && c <=57) || (c >=64 && c <=90) || (c >=97 && c <=122)) {
        if (strlen(dropBoxPwd)<37) {
          dropBoxPwd[strlen(dropBoxPwd)] = (char)c;
          dropBoxPwd[strlen(dropBoxPwd)+1] = '\0';
          displayText(dropBoxPwd, 9, 22, 1);
        }
      }
    } else if (btnID==-4) {
      if (strlen(dropBoxPwd)>0) {
        write_cmd(0x2B); // Set Row Address
        write_dat(0x00); write_dat(0x60+22);
        write_dat(0x00); write_dat(0x66+28);
        write_cmd(0x2A); // Set Column Address
        write_dat(0x00); write_dat(49+strlen(dropBoxPwd)*2);
        write_dat(0x00); write_dat(50+strlen(dropBoxPwd)*2);
        write_cmd(0x2C); for(int i=0;i<14;i++) write_dat(0);
        dropBoxPwd[strlen(dropBoxPwd)-1] = '\0';
        displayText(dropBoxPwd, 9, 22, 1);
      } 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
    } else if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+32, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if (ctr>1) ctr = 0;
      else if (ctr<0) ctr = 1;
      displayText(">", 0, (ctr*10)+32, 1);
    } else if (btnID==2) {
      if (ctr==0 && strlen(dropBoxPwd)<6) {
        displayText("Password should be atleast 6 letters", 12, 52, 1);
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
      } else break;
    } vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  charInput = false;
  if (ctr==0) {
    // Code to connect to dropBox
    vTaskDelay(500/portTICK_PERIOD_MS);
    clearDisplay();
    displayText("Logged in to Drop Box", 0, 22, 1);
    syncStatus = true;
    gDriveStatus = false;
    oneDriveStatus = false;
    dropBoxStatus = true;
    vTaskDelay(1000/portTICK_PERIOD_MS);
  } return;
}

void fontSize() {
  clearDisplay(); int ctr = fontID;
  displayText("Small", 15, 12, 1);
  displayText("Large", 15, 22, 1);
  displayText("*", 9, (ctr*10)+12, 1);
  displayText("Return", 15, 32, 1);
  displayText(">", 0, (ctr*10)+12, 1);
  while (btnID!=2) {
    if (btnID==-1 || btnID==1) {
      displayText(">", 0, (ctr*10)+12, 0);
      ctr += btnID; 
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      if (ctr>2) ctr = 0;
      else if (ctr<0) ctr = 2;
      displayText(">", 0, (ctr*10)+12, 1);
    } else vTaskDelay(100/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  if (ctr!=3) {
    displayText("*", 9, (fontID*10)+12, 0);
    fontID = ctr;
    displayText("*", 9, (fontID*10)+12, 1);
    vTaskDelay(500/portTICK_PERIOD_MS);
  } return;
}

void statusMenu() {
  WMcharInput = false; charInput = false; FSstats[3] = 0;
  for(int i=0;i<totalLen;i++) {
    if (buffer[i] == ' ' || buffer[i] == '\n') 
      FSstats[3]++;
  }
  clearDisplay();
  displayText("File Name:", 0, 2, 1);
  displayText(files[currFileID], 102, 2, 1);
  char WC[8], BL[4], BTBL[4], FS[20];
  itoa(FSstats[3], WC, 10);
  displayText("Word Count:", 0, 12, 1);
  displayText(WC, 102, 12, 1);
  itoa(byok_batt_pct, BL, 10);
  displayText("Battery Level:", 0, 22, 1);
  displayText(BL, 102, 22, 1);
  displayText("KB Battery Level:", 0, 32, 1);
  if (btStatus) {
    itoa(hid_batt_pct, BTBL, 10);
    displayText(BTBL, 102, 32, 1);
  } else displayText("NA", 102, 32, 1);
  displayText("Sync Status:", 0, 42, 1);
  if (syncStatus) {
    displayText("Syncing to ", 102, 42, 1);
      if (gDriveStatus)
        displayText("Google Drive", 168, 42, 1);
      else if (oneDriveStatus)
        displayText("One Drive", 168, 42, 1);
      else if (dropBoxStatus)
        displayText("Dropbox", 168, 42, 1);
      else
        displayText("nothing lol", 168, 42, 1);
  } else displayText("Not Syncing", 102, 42, 1);
  if (FSstats[2]<1000)
    snprintf(FS, sizeof(FS), "%ld Bytes", FSstats[2]);
  else if (FSstats[2]<1024*1000)
    snprintf(FS, sizeof(FS), "%.2f KB", (float)FSstats[2]/1024);
  else
    snprintf(FS, sizeof(FS), "%.2f MB", (float)FSstats[2]/(1024*1024));
  displayText("Free Space: ", 0, 52, 1);
  displayText(FS, 102, 52, 1);
  while (btnID!=2) vTaskDelay(10/portTICK_PERIOD_MS);
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  } WMcharInput = true; charInput = true; return;
}

void writeMode(int fileID) {
  currPos = 0; currFileID = fileID; int c; char path[50];
  struct stat file_info; long currFileSize = 0;
  snprintf(path, sizeof(path), "/spiflash/myfiles/%s", files[fileID]);
  if (stat(path, &file_info) == 0) currFileSize = file_info.st_size;
  FILE *file = fopen(path, "r");
  buffer = (char*)malloc(sizeof(char)*chars);
  if (file == NULL) return;
  else {
    while ((c = fgetc(file)) != EOF) {
      if (currPos >= chars-1) {
        chars += 100;
        buffer = realloc(buffer, sizeof(char)*chars);
      } char temp = (char)c;
      if (temp == '\n') buffer[currPos++] = ' ';
      else buffer[currPos++] = temp;
      printf("%c", temp);
    } fclose(file);
  } buffer[currPos] = '\0'; totalLen = currPos;
  if (fontID == 0) {
    // Small font
    MAX_CHARS_IN_A_LINE = 40;
    MAX_LINES = 7;
    PIXELS_IN_A_LINE = 9;
  } else if (fontID == 1) {
    // Large font
    MAX_CHARS_IN_A_LINE = 26;
    MAX_LINES = 5;
    PIXELS_IN_A_LINE = 13;
  }
  if (totalLen<=MAX_CHARS_IN_A_LINE) start = 0;
  else {
    start = (totalLen/MAX_CHARS_IN_A_LINE)*MAX_CHARS_IN_A_LINE;
    if (totalLen%MAX_CHARS_IN_A_LINE == 0) 
      start -= MAX_CHARS_IN_A_LINE;
  } end = totalLen-1;
  currRow = 0; currCol = end%MAX_CHARS_IN_A_LINE + 1;
  currPos = start + currRow*MAX_CHARS_IN_A_LINE + currCol;
  clearDisplay(); displayPageChange(0);
  charInput = true; WMcharInput = true;
  bool cursor = false; unsigned int received_value = 0;
  long prev_time = esp_timer_get_time()/1000;
  long curr_time = esp_timer_get_time()/1000;
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  while (btnID!=2) {
    if (curr_time - prev_time >=500) { // cursor
      setCursor(cursor=!cursor);
      prev_time = curr_time;
    } curr_time = esp_timer_get_time()/1000;
    if (btnID!=2 && btnID!=-2) {
      if (totalLen >= chars-1) {
        chars += 100;
        buffer = realloc(buffer, sizeof(char)*chars);
      }
      if (btnID == -4) { // back
        setCursor(0); cursor = false;
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (currPos>0) {
          bufferLeftShift();
          displayPageChange(0);
          if (currCol>0) currCol-=1;
          else {
            currCol=MAX_CHARS_IN_A_LINE-1;
            if (currRow>0) currRow-=1;
            else displayPageChange(-1);
          } printf("Left: Row: %d | Col: %d\n", currRow, currCol);
          currPos = start+currRow*MAX_CHARS_IN_A_LINE + currCol;
          printf("Curr Pos: %d\n", currPos);
        }
      } 
      if (btnID == -3) { // left
        setCursor(0); cursor = false;
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (currCol>0) currCol-=1;
        else if (currPos>0) {
          currCol=MAX_CHARS_IN_A_LINE-1;
          if (currRow>0) currRow-=1;
          else displayPageChange(-1);
        } printf("Left: Row: %d | Col: %d\n", currRow, currCol);
        printf("Curr Pos: %d\n", currPos = start+currRow*MAX_CHARS_IN_A_LINE + currCol);
      }
      if (xQueueReceive(key_print_queue, &received_value, 0) == pdTRUE) { // key
        setCursor(0); cursor = false;
        if (totalLen+4096>=FSstats[2]+currFileSize) {
          clearDisplay();
          displayText("Not enough storage", 0, 22, 1);
          displayText("Please delete some files", 0, 32, 1);
          vTaskDelay(1000/portTICK_PERIOD_MS); break;
        }
        else {
          bufferRightShift((char)received_value);
          displayPageChange(0);
          if (currPos<totalLen) {
            if (currCol<MAX_CHARS_IN_A_LINE-1) currCol+=1;
            else {
              currCol=0;
              if (currRow<(MAX_LINES-1)) currRow+=1;
              else displayPageChange(1);
            }
          } printf("Right: Row: %d | Col: %d\n", currRow, currCol);
          printf("Curr Pos: %d\n", currPos = start+currRow*MAX_CHARS_IN_A_LINE + currCol);
        }
      }
      if (btnID == 3) { // right
        setCursor(0); cursor = false;
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (currPos<totalLen) {
          if (currCol<MAX_CHARS_IN_A_LINE-1) currCol+=1;
          else {
            currCol=0;
            if (currRow<(MAX_LINES-1)) currRow+=1;
            else displayPageChange(1);
          }
        } printf("Right: Row: %d | Col: %d\n", currRow, currCol);
        printf("Curr Pos: %d\n", currPos = start+currRow*MAX_CHARS_IN_A_LINE + currCol);
      }
      if (btnID == -1) { // prev line
        setCursor(0); cursor = false;
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (currRow>0) currRow-=1;
        else if (start>0) displayPageChange(-1);
        printf("Prev: Row: %d | Col: %d\n", currRow, currCol);
        printf("Curr Pos: %d\n", currPos = start+currRow*MAX_CHARS_IN_A_LINE + currCol);
      }
      if (btnID == 1) { // next line
        setCursor(0); cursor = false;
        if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
          btnID = 0; xSemaphoreGive(xbtnIDMutex);
        }
        if (start+(currRow+1)*MAX_CHARS_IN_A_LINE<totalLen) {
          if (currRow<(MAX_LINES-1)) currRow+=1;
          else displayPageChange(1);
          currPos = start+currRow*MAX_CHARS_IN_A_LINE + currCol;
          if (currPos>totalLen) currCol = totalLen%MAX_CHARS_IN_A_LINE;
        } printf("Next: Row: %d | Col: %d\n", currRow, currCol);
        printf("Curr Pos: %d\n", currPos = start+currRow*MAX_CHARS_IN_A_LINE + currCol);
      }
    } else if (btnID == -2) { // status menu
      if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
        btnID = 0; xSemaphoreGive(xbtnIDMutex);
      }
      statusMenu(); clearDisplay();
      displayPageChange(0);
    } vTaskDelay(10/portTICK_PERIOD_MS);
  } 
  if (xSemaphoreTake(xbtnIDMutex, portMAX_DELAY) == pdTRUE) {
    btnID = 0; xSemaphoreGive(xbtnIDMutex);
  }
  charInput = false; clearDisplay();
  displayText("Saving file...", 0, 22, 1);
  FILE *file2 = fopen(path, "w");
  if (file2 != NULL) {
    fprintf(file2, buffer);
    fflush(file2);
    fsync(fileno(file2));
    fclose(file2);
  } else {
    clearDisplay();
    displayText("Failed to save file", 0, 22, 1);
  } free(buffer);
  WMcharInput = false;
  list_files("/spiflash/myfiles");
  return;
}

void displayPageChange(int upDown) {
  start += upDown*MAX_CHARS_IN_A_LINE;
  if (upDown == 1) {
    if (end+MAX_CHARS_IN_A_LINE>=totalLen) end=totalLen-1;
    else end+=MAX_CHARS_IN_A_LINE;
  } else if (upDown == -1) {
    if (end-start>=MAX_LINES*MAX_CHARS_IN_A_LINE) end-=((end%MAX_CHARS_IN_A_LINE)+1);
  } printf("\nWindow start: %d | end: %d\n\n", start, end);
  int ctr = -1;
  for (int i=start;i<=end;i++) {
    if (i%MAX_CHARS_IN_A_LINE == 0) ctr++;
    if (fontID == 0)
      displayChar1(buffer[i], (i-ctr*MAX_CHARS_IN_A_LINE-start)*2, ctr*PIXELS_IN_A_LINE);
    else if (fontID == 1)
      displayChar2(buffer[i], (i-ctr*MAX_CHARS_IN_A_LINE-start)*3, ctr*PIXELS_IN_A_LINE);
  }
  for (int i=end-start+1;i<MAX_LINES*MAX_CHARS_IN_A_LINE;i++) {
    if (i%MAX_CHARS_IN_A_LINE == 0) ctr++;
    if (fontID == 0)
      displayChar1(' ', (i-ctr*MAX_CHARS_IN_A_LINE)*2, ctr*PIXELS_IN_A_LINE);
    else if (fontID == 1)
      displayChar2(' ', (i-ctr*MAX_CHARS_IN_A_LINE)*3, ctr*PIXELS_IN_A_LINE);
  }
}

void displayChar1(char c, int x, int y) {
  write_cmd(0x2B); // Set Row Address
  write_dat(0x00); write_dat(0x60+y);
  write_dat(0x00); write_dat(0x67+y);
  write_cmd(0x2A); // Set Column Address
  write_dat(0x00); write_dat(0x30+x);
  write_dat(0x00); write_dat(0x31+x);
  write_cmd(0x2C);
  uint8_t *bitmap = font5x7[(int)c - 0x10];
  for (int i=0;i<7;i++) {
    unsigned char temp1 = 36, temp2 = 36;
    for (int j=0;j<5;j++) {
      if ((bitmap[j] >> i) & 0x01) {
        if (j==0)
          temp1+=192;
        else if (j==1)
          temp1+=24;
        else if (j==2)
          temp1+=3;
        else if (j==3)
          temp2+=192;
        else
          temp2+=24;
      }
    } write_dat(temp1); write_dat(temp2);
  }
}

void displayChar2(char c, int x, int y) {
  write_cmd(0x2B); // Set Row Address
  write_dat(0x00); write_dat(0x60+y);
  write_dat(0x00); write_dat(0x6B+y);
  write_cmd(0x2A); // Set Column Address
  write_dat(0x00); write_dat(0x30+x);
  write_dat(0x00); write_dat(0x32+x);
  write_cmd(0x2C);
  uint8_t *bitmap = font9x12[(int)c - 0x20];
  for (int i=0;i<12;i++) {
    unsigned char temp1 = 36, temp2 = 36, temp3 = 36;
    for (int j=0;j<8;j++) {
      if ((bitmap[i] >> j) & 0x01) {
        if (j==0)
          temp3+=3;
        else if (j==1)
          temp3+=24;
        else if (j==2)
          temp3+=192;
        else if (j==3)
          temp2+=3;
        else if (j==4)
          temp2+=24;
        else if (j==5)
          temp2+=192;
        else if (j==6)
          temp1+=3;
        else if (j==7)
          temp1+=24;
      }
    } write_dat(temp1); write_dat(temp2); write_dat(temp3);
  }
}

void setCursor(bool onOff) {
  write_cmd(0x2A); // Set Column Address
  write_dat(0x00); write_dat(0x30+currCol*(2+fontID));
  write_dat(0x00); write_dat(0x31+fontID+currCol*(2+fontID));
  write_cmd(0x2B); // Set Row Address
  write_dat(0x00); write_dat(0x60+(PIXELS_IN_A_LINE-2)+currRow*PIXELS_IN_A_LINE);
  write_dat(0x00); write_dat(0x60+(PIXELS_IN_A_LINE-2)+currRow*PIXELS_IN_A_LINE);
  write_cmd(0x2C);
  if (fontID == 0) {
    onOff? write_dat(0xff):write_dat(0);
    onOff? write_dat(0xfc):write_dat(0);
  } else if (fontID == 1) {
    onOff? write_dat(0x3f):write_dat(0);
    onOff? write_dat(0xff):write_dat(0);
    onOff? write_dat(0xfc):write_dat(0);
  }
}

void bufferRightShift(char c) {
  for (int i=totalLen-1;i>=currPos;i--)
    buffer[i+1] = buffer[i];
  buffer[totalLen+1] = '\0';
  buffer[currPos] = c;
  totalLen++;
  if (end-start<MAX_LINES*MAX_CHARS_IN_A_LINE) end++;
}

void bufferLeftShift() {
  for (int i=currPos;i<totalLen;i++)
    buffer[i-1] = buffer[i];
  buffer[--totalLen] = '\0';
  if (end>=totalLen) end--;
}

//display functions end

static void periodic_timer_callback(void* arg) {
  powerOffStatus = true;
}

void app_main(void) {
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << POWER_LED);
    gpio_config(&io_conf); gpio_set_level(POWER_LED, 1);
    ledc_timer_config_t ledc_timer = {
      .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
      .freq_hz = 2000,                      // frequency of PWM signal
      .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
      .timer_num = LEDC_TIMER_1,            // timer index
      .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    }; ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    ledc_channel_config_t ledc_channel = {
      .speed_mode     = LEDC_LOW_SPEED_MODE,
      .channel        = LEDC_CHANNEL_1,
      .timer_sel      = LEDC_TIMER_1,
      .flags.output_invert = 0,
      .gpio_num       = BL_PWM,
      .duty           = 0,                    // Set duty initially to 0%
      .hpoint         = 0
    }; ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    io_conf.pin_bit_mask = (1ULL << POWER_OFF);  gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << LCM_CS);     gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << LCM_RESET);  gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << LCM_ENABLE); gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << LCM_A0);     gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << D0); gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << D1); gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << D2); gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << D3); gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << D4); gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << D5); gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << D6); gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << D7); gpio_config(&io_conf);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pin_bit_mask = (1ULL << downBtn);   gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << upBtn);     gpio_config(&io_conf);
    io_conf.pin_bit_mask = (1ULL << BLBtn);     gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(downBtn,   down,  NULL);
    gpio_isr_handler_add(upBtn,     up,    NULL);
    gpio_isr_handler_add(BLBtn,     BL,    NULL);
    io_conf.intr_type = GPIO_INTR_ANYEDGE; io_conf.pin_bit_mask = (1ULL<<selectBtn);
    gpio_config(&io_conf); gpio_isr_handler_add(selectBtn, selectFn, NULL);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; io_conf.pin_bit_mask = (1ULL<<powerOffBtn);
    gpio_config(&io_conf); gpio_isr_handler_add(powerOffBtn, powerOffBtnFn, NULL);

    esp_timer_create_args_t timer_args = {
      .callback = &periodic_timer_callback, // Timer callback function
      .arg = "Periodic Timer",              // Argument to pass to callback
      .name = "MyPeriodicTimer"             // Optional timer name
    };
    esp_err_t err = esp_timer_create(&timer_args, &periodic_timer);
    if (err != ESP_OK) ESP_LOGE(TAG, "Failed to create timer: %s", esp_err_to_name(err));

/*

    const esp_vfs_fat_mount_config_t mount_config = {
      .max_files = 4,
      .format_if_mount_failed = false,
      .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    }; static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
    esp_vfs_fat_spiflash_mount_rw_wl("/spiflash", "storage", &mount_config, &s_wl_handle);
    list_files("/spiflash/myfiles");


    TaskHandle_t usb_events_task_handle;
    TaskHandle_t repeated_keys_task_handle;
    TaskHandle_t monitor_keyboard_input_task_handle;
    TaskHandle_t display_task_handle;
    //TaskHandle_t hid_demo_task_handle;
*/

    printf("Hello_world\n");
    
    LCD_initial();

    BaseType_t task_created;

    esp_err_t ret;
#if HID_HOST_MODE == HIDH_IDLE_MODE
    ESP_LOGE(TAG, "Please turn on BT HID host or BLE!");
    return;
#endif
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    ESP_LOGI(TAG, "setting hid gap, mode:%d", HID_HOST_MODE);
    ESP_ERROR_CHECK( esp_hid_gap_init(HID_HOST_MODE) );
#if CONFIG_BT_BLE_ENABLED
    ESP_ERROR_CHECK( esp_ble_gattc_register_callback(esp_hidh_gattc_event_handler) );
#endif /* CONFIG_BT_BLE_ENABLED */
    esp_hidh_config_t config = {
        .callback = hidh_callback,
        .event_stack_size = 4096,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK( esp_hidh_init(&config) );

    //Creating queue for printing characters:

    key_print_queue = xQueueCreate(10, sizeof(unsigned int));
    assert(key_print_queue != NULL);

    // Declare and create the binary semaphore

   // SemaphoreHandle_t xHidDemoSemaphore = xSemaphoreCreateBinary();

    xbtnIDMutex = xSemaphoreCreateMutex();

    xHidDemoSemaphore = xSemaphoreCreateBinary();

    xMonitorKeyboardSemaphore = xSemaphoreCreateBinary();

    assert(xHidDemoSemaphore != NULL);

    assert(xMonitorKeyboardSemaphore != NULL);

    assert(xbtnIDMutex != NULL);

    // Create the hid_demo_task

    //task_created = xTaskCreate(hid_demo_task, "hid_demo_task", 6 * 1024, (void *)xHidDemoSemaphore, 2, &hid_demo_task_handle);
    task_created = xTaskCreate(hid_demo_task, "hid_demo_task", 6 * 1024, NULL, 2, &hid_demo_task_handle);
    assert(task_created);

    // Create the monitor_keyboard_input_task

   // task_created = xTaskCreate(monitor_keyboard_input, "monitor_keyboard_input", 2048, (void *)xHidDemoSemaphore, 2, &monitor_keyboard_input_task_handle);
   // task_created = xTaskCreate(monitor_keyboard_input, "monitor_keyboard_input", 2048, NULL, 2, &monitor_keyboard_input_task_handle);
   // assert(task_created);

//    xTaskCreate(&hid_demo_task, "hid_task", 6 * 1024, NULL, 2, NULL);    

    const gpio_config_t input_pin = {
        .pin_bit_mask = BIT64(APP_QUIT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&input_pin));
    const gpio_config_t batt_op_pin = {
        .pin_bit_mask = BIT64(BATT_STAT_OP_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&batt_op_pin));

    // const gpio_config_t batt_ip_pin = {
    //     .pin_bit_mask = BIT64(BATT_STAT_IP_PIN),
    //     .mode = GPIO_MODE_INPUT,
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_DISABLE
    // };
    // ESP_ERROR_CHECK(gpio_config(&batt_ip_pin));

    adc1_config_width(ADC_WIDTH_BIT_12);  // ADC resolution
    adc1_config_channel_atten(ADC_INPUT_CHANNEL, ADC_ATTEN_DB_11);
    task_created = xTaskCreate(byok_battery_calc, "byok_battery_calc", 1024, NULL, 2, NULL);
    assert(task_created);
    ESP_LOGI(TAG, "HID HOST example");

    usb_flags = xEventGroupCreate();
    assert(usb_flags);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1
    };

    ESP_ERROR_CHECK(usb_host_install(&host_config));
    task_created = xTaskCreate(handle_usb_events, "usb_events", 4096, NULL, 2, &usb_events_task_handle);
    assert(task_created);

    task_created = xTaskCreate(handle_repeated_keys, "repeated_keys", 4096, NULL, 2, &repeated_keys_task_handle);
    assert(task_created);

    const hid_host_driver_config_t hid_host_config = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = hid_host_event_callback,
        .callback_arg = NULL
    };

    ESP_ERROR_CHECK(hid_host_install(&hid_host_config));
    
    // Create the USB event handler task
    task_created = xTaskCreate(
        vUSBEventHandlerTask, 
        "USBEventHandler", 
        4096, 
        NULL, 
        2, 
        NULL
    );
    assert(task_created == pdPASS);

    //avlog_o: task for display
    task_created = xTaskCreate(display_task, "display_task", 4096, NULL, 2, &display_task_handle);
    assert(task_created == pdPASS);

    task_created = xTaskCreate(power_off_task, "power_off_task", 4096, NULL, 2, NULL);
    assert(task_created == pdPASS);

    //OTA Code in app_main open
    const esp_partition_t *running_part = esp_ota_get_running_partition();
    esp_app_desc_t running_part_app_info;
    if (esp_ota_get_partition_description(running_part, &running_part_app_info) == ESP_OK) {
        ESP_LOGI(TAG, "Running firmware version: %s", running_part_app_info.version);
        memcpy(running_fw_version, running_part_app_info.version, sizeof(running_fw_version));
        ESP_LOGI(running_fw_version, "version variable");
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    ota_start();
    //OTA Code in app_main close

    xEventGroupWaitBits(usb_flags, APP_QUIT_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);

    if (hid_device_connected) {
        ESP_LOGI(TAG, "Uninitializing HID Device");
        hid_host_release_interface(keyboard_handle);
       // hid_host_release_interface(mouse_handle);
        ESP_ERROR_CHECK(hid_host_uninstall_device(hid_device));
        hid_device_connected = false;
    }

    ESP_LOGI(TAG, "Uninitializing USB");
    ESP_ERROR_CHECK(hid_host_uninstall());
    wait_for_event(READY_TO_UNINSTALL, portMAX_DELAY);
    ESP_ERROR_CHECK(usb_host_uninstall());
    vTaskDelete(usb_events_task_handle);
    vTaskDelete(repeated_keys_task_handle);
    vEventGroupDelete(usb_flags);
    ESP_LOGI(TAG, "Done");
    
    //end of code
}
