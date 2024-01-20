// Copyright (c) 2024, Takashi Toyoshima <toyoshim@gmail.com>. All rights
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

#include "cmt.h"

static uint8_t phase[2][8] = {
    { 0, 0, 1, 1, 0, 0, 1, 1 },  // 1200Hz x2
    { 0, 1, 0, 1, 0, 1, 0, 1 },  // 2400Hz x4
};

static int Get(TapIO *io) {
  CMT* cmt = io->ext;
  cmt->count--;
  if (cmt->count == 0) {
    cmt->count = 23;
    cmt->phase++;
    if ((cmt->phase & 3) == 3) {
      cmt->count++;
    }
    if (cmt->phase == 8) {
      cmt->phase = 0;
      if (cmt->lead) {
        cmt->lead--;
      } else {
        if (io->offset == io->size) {
          cmt->mark = 1;
          return TAPE_GET_EOF;
        } else {
          if (cmt->bit == 0) {
            cmt->mark = 0;  // Start Bit
          } else if (cmt->bit > 8) {
            cmt->mark = 1;  // Stop Bit
          } else {
            cmt->mark =
                (io->buffer[io->offset & 0x3ff] & (1 << (cmt->bit - 1))) ?
                    1 : 0;
          }
          cmt->bit++;
          if (cmt->bit == 11) {
            cmt->bit = 0;
            io->offset++;
            // TODO
            if ((io->offset & 0x1ff) == 0 && io->status == TAPE_STATUS_OK) {
              int buffer_offset = (io->offset & 0x200) ^ 0x200;
              uint32_t read_start_offset = io->offset + 0x200;
              uint32_t remaining_size = io->size - read_start_offset;
              uint32_t size_to_read =
                  (remaining_size < 512) ? remaining_size : 512;
              io->status = TAPE_STATUS_READING;
              io->async_read(&io->buffer[buffer_offset], size_to_read,
                             &io->status);
            }
          }
        }
      }
    }
  }
  io->tick++;
  return phase[cmt->mark][cmt->phase];
}

int CMT_Open(TapIO* io, CMT* cmt) {
  io->offset = 0;
  io->tick = 0;
  io->status = TAPE_STATUS_OK;
  io->request = 0;
  cmt->mark = 1;
  cmt->lead = 1000;
  cmt->count = 23;
  cmt->phase = 0;
  cmt->bit = 0;
  uint32_t size_to_read = (io->size < 1024) ? io->size : 1024;
  int read_size = io->sync_read(io->buffer, size_to_read);
  if (read_size != size_to_read) {
    return -1;
  }

  io->ext = cmt;
  io->get = Get;
  return 0;
}
