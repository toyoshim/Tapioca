// Copyright (c) 2020, Takashi Toyoshima <toyoshim@gmail.com>. All rights
// reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
//    * Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
//    * Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
//    * Neither the name of Google Inc. nor the names of its
// contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "ui.h"

#include <memory.h>

#include "file.h"
#include "main.h"
#include "lui.h"
#include "tape.h"

extern ADC_HandleTypeDef hadc;

static const char pcb_ver[] = "1.00";
static const char frm_ver[] = "0.60";
static char line_buf[16];

enum {
  MODE_RAW,
  MODE_MENU,
  MODE_EDIT,
};
static uint8_t next_scene;
static uint8_t scene;
static uint8_t mode;
static union {
  LUI_MENU menu;
  LUI_EDIT edit;
  struct {
    uint8_t count;
    uint8_t mic_h;
    uint8_t mic_l;
    uint8_t remote;
  } raw;
} _;

static void led_on(uint8_t id) {
  switch (id) {
    case 0:
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
      break;
    case 1:
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
      break;
  }
}

static void led_off(uint8_t id) {
  switch (id) {
    case 0:
      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
      break;
    case 1:
      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
      break;
  }
}

static uint8_t sd_is_inserted() {
  return
      (HAL_GPIO_ReadPin(DET_B_GPIO_Port, DET_B_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
}

static const char* list_filename(uint8_t n) {
  const char* listname = NULL;
  if (file_listname(n, &listname))
    ui_go(UI_ERROR);
  return listname;
}

void ui_init() {
  lui_init();
  next_scene = 0;
  mode = MODE_RAW;
  ui_go(UI_TITLE);
}

void ui_go(uint8_t next_scene) {
  scene = next_scene;
  mode = MODE_RAW;
  _.raw.count = 0;
  switch (scene) {
    case UI_TITLE:
      lui_set(LUI_WINDOW_TITLE, "TapiOca! Ready");
      lui_set(LUI_WINDOW_ICON, "");
      lui_set(LUI_WINDOW_MAIN, " 2020 Mellow PCB");
      break;
    case UI_NO_MEDIA:
      lui_set(LUI_WINDOW_TITLE, "No microSD");
      lui_set(LUI_WINDOW_ICON, "");
      lui_set(LUI_WINDOW_MAIN, "");
      break;
    case UI_NO_MEDIA_MENU:
      mode = MODE_MENU;
      _.menu.mode = LUI_MENU_STATIC_FULL;
      _.menu.title = "TapiOca!";
      _.menu.n = 5;
      _.menu.items = "Adjust Mic Lv\0Remote Check\0LED Test\0Version\0Cancel";
      lui_menu_set(&_.menu);
      break;
    case UI_ADJUST_MIC:
      lui_set(LUI_WINDOW_TITLE, "Adjust Mic Lv");
      lui_set(LUI_WINDOW_ICON, "");
      _.raw.mic_l = 255;
      _.raw.mic_h = 0;
      break;
    case UI_REMOTE_CHECK:
      lui_set(LUI_WINDOW_TITLE, "Remote Control");
      lui_set(LUI_WINDOW_ICON, "");
      _.raw.remote = 2;
      break;
    case UI_LED_TEST:
      lui_set(LUI_WINDOW_TITLE, "LED Test");
      lui_set(LUI_WINDOW_ICON, "");
      lui_set(LUI_WINDOW_MAIN, "Press Buttons");
      break;
    case UI_VERSION:
      snprintf(line_buf, 16, "PCB: %s", pcb_ver);
      lui_set(LUI_WINDOW_TITLE, line_buf);
      lui_set(LUI_WINDOW_ICON, "");
      snprintf(line_buf, 16, "FRM: %s", frm_ver);
      lui_set(LUI_WINDOW_MAIN, line_buf);
      break;
    case UI_MAIN:
      mode = MODE_MENU;
      _.menu.mode = LUI_MENU_STATIC_HALF;
      _.menu.title = "microSD Ready";
      _.menu.n = 2;
      _.menu.items = "Open\0Create";
      lui_menu_set(&_.menu);
      break;
    case UI_OPEN:
      mode = MODE_MENU;
      _.menu.mode = LUI_MENU_DYNAMIC_FULL;
      _.menu.title = "Open T77";
      _.menu.item = list_filename;
      if (file_opendir())
        ui_go(UI_ERROR);
      else
        lui_menu_set(&_.menu);
      break;
    case UI_CREATE:
      mode = MODE_EDIT;
      _.edit.title = "Create";
      strcpy(line_buf, "        .T77");
      _.edit.data = line_buf;
      _.edit.chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ_0123456789";
      lui_edit_set(&_.edit);
      break;
    case UI_PLAY:
      lui_set(LUI_WINDOW_TITLE, line_buf);
      lui_set(LUI_WINDOW_ICON, "");
      lui_set(LUI_WINDOW_MAIN, "");
      break;
    case UI_CANCEL:
      lui_set(LUI_WINDOW_TITLE, "Cancelled");
      lui_set(LUI_WINDOW_ICON, "");
      lui_set(LUI_WINDOW_MAIN, "");
      break;
    case UI_ERROR:
      lui_set(LUI_WINDOW_TITLE, "Media Error!");
      lui_set(LUI_WINDOW_ICON, "");
      lui_set(LUI_WINDOW_MAIN, "");
      break;
  }
}

void ui_sync() {
  static uint8_t previous_pressed = 0;
  uint8_t pressed = (
      HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) ? 0 : LUI_BUTTON_LEFT)
      | (HAL_GPIO_ReadPin(B2_GPIO_Port, B2_Pin) ? 0 : LUI_BUTTON_RIGHT)
      | (HAL_GPIO_ReadPin(B3_GPIO_Port, B3_Pin) ? 0 : LUI_BUTTON_OK);
  uint8_t buttons = (previous_pressed ^ pressed) & pressed;
  previous_pressed = pressed;

  if (mode == MODE_MENU) {
    int16_t select = lui_menu_run(&_.menu, buttons);
    if (select >= 0) {
      switch (scene) {
        case UI_MAIN:
          next_scene = UI_MAIN;
          switch (select) {
            case 0:  // Open
              ui_go(UI_OPEN);
              break;
            case 1:  // Create
              ui_go(UI_CREATE);
              break;
          }
          break;
        case UI_NO_MEDIA_MENU:
          next_scene = UI_NO_MEDIA;
          switch (select) {
            case 0:  // Adjust Mic Lv
              ui_go(UI_ADJUST_MIC);
              break;
            case 1:  // Remote Check
              ui_go(UI_REMOTE_CHECK);
              break;
            case 2:  // LED Test
              ui_go(UI_LED_TEST);
              break;
            case 3:  // Version
              ui_go(UI_VERSION);
              break;
            case 4:  // Cancel
              ui_go(UI_NO_MEDIA);
              break;
          }
          break;
        case UI_OPEN:
          switch (file_type(select)) {
            case FILE_TYPE_DIR:
              if (file_chdir(file_name(select)))
                ui_go(UI_ERROR);
              else
                lui_menu_set(&_.menu);
              break;
            case FILE_TYPE_FILE:
              if (file_closedir()) {
                ui_go(UI_ERROR);
              } else {
                const char* listname;
                file_listname(select, &listname);
                snprintf(line_buf, 16, "%s", listname);
                if (file_open(file_name(select)))
                  ui_go(UI_ERROR);
                else
                  ui_go(UI_PLAY);
              }
              break;
            case FILE_TYPE_COMMAND:
              ui_go(UI_MAIN);
              break;
          }
          break;
      }
    }
  } else if (mode == MODE_EDIT) {
    int8_t result = lui_edit_run(&_.edit, buttons);
    if (result < 0)
      return;
    if (result > 0) {
      switch (scene) {
        case UI_CREATE:
          if (line_buf[0] == 0x20) {
            ui_go(UI_CANCEL);
          } else {
            printf("create: %s\n", line_buf);
            // TODO
            ui_go(UI_CANCEL);
          }
          break;
      }
    }
  } else {
    _.raw.count++;
    switch (scene) {
      case UI_TITLE:
        if (_.raw.count == 50) {
          if (sd_is_inserted())
            ui_go(UI_MAIN);
          else
            ui_go(UI_NO_MEDIA);
          led_off(0);
          led_off(1);
        }
        break;
      case UI_NO_MEDIA:
        if (buttons & LUI_BUTTON_OK)
          ui_go(UI_NO_MEDIA_MENU);
        else if (sd_is_inserted())
          ui_go(UI_MAIN);
        break;
      case UI_ADJUST_MIC: {
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 10);
        uint32_t val = HAL_ADC_GetValue(&hadc);  // 0-4096
        HAL_ADC_Stop(&hadc);
        if (val > 255)
          val = 255;
        if (val < _.raw.mic_l)
          _.raw.mic_l = val;
        if (_.raw.mic_h < val)
          _.raw.mic_h = val;
        snprintf(line_buf, 16, "min:%3d/max:%3d", _.raw.mic_l, _.raw.mic_h);
        lui_set(LUI_WINDOW_MAIN, line_buf);
        if (buttons & LUI_BUTTON_LEFT)
          _.raw.mic_l = 255;
        if (buttons & LUI_BUTTON_RIGHT)
          _.raw.mic_h = 0;
        if (buttons & LUI_BUTTON_OK)
          ui_go(next_scene);
        break;
      }
      case UI_REMOTE_CHECK: {
        uint8_t current_remote = tape_is_remote_on();
        if (_.raw.remote != current_remote) {
          _.raw.remote = current_remote;
          lui_set(LUI_WINDOW_MAIN, _.raw.remote ? "ON" : "OFF");
        }
        if (buttons)
          ui_go(next_scene);
        break;
      }
      case UI_LED_TEST:
        if (pressed & LUI_BUTTON_LEFT)
          led_on(0);
        else
          led_off(0);
        if (pressed & LUI_BUTTON_RIGHT)
          led_on(1);
        else
          led_off(1);
        if (buttons & LUI_BUTTON_OK)
          ui_go(next_scene);
        break;
      case UI_VERSION:
        if (buttons)
          ui_go(next_scene);
        break;
      case UI_PLAY: {
        uint16_t sec_x4 = file_tick() * 9 / 250000;
        uint8_t state = file_state();
        uint8_t
        c = state == FILE_STATE_IDLE ? ':' :
                    state == FILE_STATE_PAUSE_TO_PLAY ? 0xfa :
                    (sec_x4 & 1) ? ' ' : 0xfc;
        snprintf(line_buf, 17, "%cPlay      %02d:%02d", c, sec_x4 / 240,
                 (sec_x4 % 240) >> 2);
        lui_set(LUI_WINDOW_MAIN, line_buf);
        break;
      }
      case UI_CANCEL:
      case UI_ERROR:
        if (_.raw.count == 50)
          ui_go(next_scene);
        break;
    }
  }
  switch (scene) {
    case UI_MAIN:
    case UI_OPEN:
    case UI_ERROR:
      if (!sd_is_inserted())
        ui_go(UI_NO_MEDIA);
      break;
  }
  lui_sync();
}

void ui_idle() {
  file_dispatch_async_operations();
}
