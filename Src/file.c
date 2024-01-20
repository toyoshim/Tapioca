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

#include "file.h"

#include <memory.h>

#include "cmt.h"
#include "fatfs.h"
#include "t77.h"

struct fileinfo {
  char display_name[16];
  char fname[13];
  uint8_t type;
  uint8_t n;
};
// Reuse the TapIO buffer to reduce memory consumption.
static TapIO io;
static T77 t77;
static CMT cmt;

static struct fileinfo* files = (struct fileinfo*) io.buffer;
//static struct fileinfo files[16];
static DIR dir;
static volatile uint8_t state = FILE_STATE_IDLE;
static uint8_t dir_opened = 0;
static uint8_t dir_index = 0;
static uint8_t dir_level = 0;
static const char root[] = "0:/";

FIL tape_file;
uint8_t* volatile async_buffer = 0;
volatile uint32_t async_size = 0;
volatile int* async_status = 0;

static int sync_read(uint8_t* buffer, uint32_t size) {
  UINT byte;
  FRESULT result = f_read(&tape_file, buffer, size, &byte);
  if (result != FR_OK)
    return -1;
  return byte;
}

static void async_read(uint8_t* buffer, uint32_t size, volatile int* status) {
  if (async_status != 0) {
    *status = TAPE_STATUS_ERROR;
  } else {
    async_buffer = buffer;
    async_size = size;
    async_status = status;
  }
}

static uint8_t file_list_until(uint8_t n) {
  FILINFO info;
  char lfn[_MAX_LFN + 1];
  info.lfname = lfn;
  info.lfsize = sizeof(lfn);
  FRESULT result;

  if (!dir_opened)
    return 0;

  if (n == 0) {
    if (dir_level) {
      strcpy(files[0].display_name, "../");
      strcpy(files[0].fname, "..");
      files[0].type = FILE_TYPE_DIR;
    } else {
      strcpy(files[0].display_name, "[Cancel]");
      strcpy(files[0].fname, "");
      files[0].type = FILE_TYPE_COMMAND;
    }
    files[0].n = 0;
    return 0;
  }

  if (n < dir_index) {
    result = f_closedir(&dir);
    dir_opened = 0;
    if (result != FR_OK)
      return 1;
    result = f_opendir(&dir, ".");
    if (result != FR_OK)
      return 1;
    dir_opened = 1;
    dir_index = 0;
  }
  do {
    result = f_readdir(&dir, &info);
    if (result != FR_OK)
      return 1;
    if (info.fname[0] == 0)  // EOD
      break;
    uint8_t index = dir_index % 16;
    const char* fname = info.lfname[0] ? info.lfname : info.fname;
    uint8_t len = strlen(fname);
    if (info.fattrib & AM_DIR) {
      if (fname[0] == '.')
        continue;
      snprintf(files[index].display_name, 15, fname);
      if (len > 14)
        files[index].display_name[13] = '$';
      files[index].display_name[(len > 14) ? 14 : len] = '/';
      files[index].display_name[(len > 14) ? 15 : len + 1] = 0;
      files[index].type = FILE_TYPE_DIR;
    } else {
      if ((len < 4) || fname[0] == '.'
          || (strncasecmp(&fname[len - 4], ".t77", 4)
              && strncasecmp(&fname[len - 4], ".cmt", 4))) {
        continue;
      }
      memset(files[index].display_name, 0x20, 11);
      strncpy(files[index].display_name, fname, (len < 15) ? len - 4 : 11);
      if (len > 15)
        files[index].display_name[10] = '$';
      strncpy(&files[index].display_name[11], &fname[len - 4], 5);
      files[index].type = FILE_TYPE_FILE;
    }
    snprintf(files[index].fname, 13, info.fname);
    files[index].n = dir_index++;
  } while (dir_index <= n);
  return 0;
}

uint8_t file_opendir() {
  if (dir_opened)
    f_closedir(&dir);
  dir_opened = 0;
  f_mount(NULL, root, 0);
  FRESULT result = f_mount(&USERFatFS, root, 0);
  if (result != FR_OK)
    return 1;
  for (uint8_t i = 0; i < 16; ++i)
    files[i].type = FILE_TYPE_INVALID;
  result = f_chdir(root);
  if (result != FR_OK)
    return 1;
  result = f_opendir(&dir, ".");
  if (result != FR_OK)
    return 1;
  dir_opened = 1;
  dir_index = 1;  // 0 is for "[Cancel]" and "../"
  dir_level = 0;
  return 0;
}

uint8_t file_chdir(const char* name) {
  FRESULT result = f_closedir(&dir);
  dir_opened = 0;
  dir_index = 1;
  if (result != FR_OK)
    return 1;
  result = f_chdir(name);
  if (result != FR_OK)
    return 1;
  result = f_opendir(&dir, ".");
  if (result != FR_OK)
    return 1;
  dir_opened = 1;
  for (uint8_t i = 0; i < 16; ++i)
    files[i].type = FILE_TYPE_INVALID;
  if (strcmp(name, ".."))
    dir_level++;
  else
    dir_level--;
  return 0;
}

uint8_t file_listname(uint8_t n, const char** name) {
  uint8_t index = n % 16;
  if (files[index].type == FILE_TYPE_INVALID || files[index].n != n) {
    if (file_list_until(n))
      return 1;
  }
  if (files[index].type && files[index].n == n)
    *name = files[index].display_name;
  else
    *name = NULL;
  return 0;
}

const char* file_name(uint8_t n) {
  uint8_t index = n % 16;
  if (files[index].type == FILE_TYPE_INVALID || files[index].n != n)
    return NULL;
  return files[index].fname;
}

uint8_t file_type(uint8_t n) {
  uint8_t index = n % 16;
  if (files[index].type == FILE_TYPE_INVALID || files[index].n != n)
    return FILE_TYPE_INVALID;
  return files[index].type;
}

uint8_t file_closedir() {
  FRESULT result = f_closedir(&dir);
  dir_opened = 0;
  if (result != FR_OK)
    return 1;
  return 0;
}

uint8_t file_open(const char* name) {
  io.sync_read = &sync_read;
  io.async_read = &async_read;
  FILINFO fin;
  FRESULT result = f_stat(name, &fin);
  if (result != FR_OK)
    return 1;
  io.size = fin.fsize;
  result = f_open(&tape_file, name, FA_OPEN_EXISTING | FA_READ);
  if (result != FR_OK)
    return 1;
  int tr = T77_Open(&io, &t77);
  if (tr) {
    f_lseek(&tape_file, 0);
    tr = CMT_Open(&io, &cmt);
    if (tr) {
      return 1;
    }
  }
  state = FILE_STATE_PLAY;  // TODO: check remote state
  return 0;
}

uint8_t file_get_signal() {
  static uint8_t signal = 0;
  if (state != FILE_STATE_PLAY)
    return 0;
  int result = io.get(&io);
  if (result == TAPE_GET_EOF) {
    state = FILE_STATE_IDLE;
    return 0;
  }
  if (result < 0)
    return signal;
  signal = result;
  return signal;
}

void file_dispatch_async_operations() {
  if (!async_status || *async_status != TAPE_STATUS_READING)
    return;

  UINT byte;
  FRESULT result = f_read(&tape_file, async_buffer, async_size, &byte);
  if (result != FR_OK || byte != async_size)
    *async_status = TAPE_STATUS_ERROR;
  else
    *async_status = TAPE_STATUS_OK;
  async_status = 0;
}

uint32_t file_tick() {
  return io.tick;
}

uint8_t file_state() {
  return state;
}
