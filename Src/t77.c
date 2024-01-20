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

#include "t77.h"

#include <memory.h>

static int Get(TapIO* io) {
  T77* t77 = io->ext;
  if (!t77->remain && !t77->request) {
    if ((io->offset + 2) > io->size)
      return TAPE_GET_EOF;
    int buffer_offset = io->offset & 0x3ff;
    uint16_t data = (io->buffer[buffer_offset] << 8)
        | io->buffer[buffer_offset + 1];
    io->offset += 2;

    t77->level = data >> 15;
    t77->remain = data & 0x7fff;

    if ((io->offset & 0x1ff) == 0)
      t77->request = 1;
  }
  if (t77->request && t77->status == TAPE_STATUS_OK) {
    int buffer_offset = (io->offset & 0x200) ^ 0x200;
    uint32_t read_start_offset = io->offset + 0x200;
    uint32_t remaining_size = io->size - read_start_offset;
    uint32_t size_to_read = (remaining_size < 512) ? remaining_size : 512;
    t77->status = TAPE_STATUS_READING;
    io->async_read(&io->buffer[buffer_offset], size_to_read, &t77->status);
    t77->request = 0;
  }
  if (!t77->remain)
    return TAPE_GET_BUSY;

  t77->remain--;
  io->tick++;
  return t77->level;
}

int T77_Open(TapIO* io, T77* t77) {
  io->offset = 0;
  io->tick = 0;
  t77->remain = 0;
  t77->request = 0;
  t77->status = TAPE_STATUS_OK;
  uint32_t size_to_read = (io->size < 1024) ? io->size : 1024;
  int read_size = io->sync_read(io->buffer, size_to_read);
  if (read_size != size_to_read)
    return -1;

  if (io->size < 16 || memcmp(io->buffer, "XM7 TAPE IMAGE 0", 16) != 0)
    return -2;
  io->offset = 16;

  io->ext = t77;
  io->get = Get;
  return 0;
}
