#ifndef STANDALONE
#include <Python.h>
#endif
#include "../bitstream.h"

/********************************************************
 Audio Tools, a module and set of tools for manipulating audio data
 Copyright (C) 2007-2013  Brian Langenberger

 This program is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*******************************************************/

typedef enum {OGG_OK = 0,
              OGG_STREAM_FINISHED = 1,
              OGG_INVALID_MAGIC_NUMBER = -1,
              OGG_INVALID_STREAM_VERSION = -2,
              OGG_CHECKSUM_MISMATCH = -3,
              OGG_PREMATURE_EOF = -4} ogg_status;

struct ogg_page_header {
    uint32_t magic_number;
    uint8_t version;
    uint8_t type;
    uint64_t granule_position;
    uint32_t bitstream_serial_number;
    uint32_t page_sequence_number;
    uint32_t checksum;
    uint8_t page_segment_count;
    uint8_t page_segment_lengths[0x100];
    uint32_t segment_length_total;
};

typedef struct OggReader_s {
    BitstreamReader *ogg_stream;
    uint32_t checksum;
    struct ogg_page_header current_header;
    uint8_t current_segment;
} OggReader;

OggReader*
oggreader_open(FILE *stream);

void
oggreader_close(OggReader *reader);

ogg_status
oggreader_read_page_header(BitstreamReader *ogg_stream,
                           struct ogg_page_header *header);

/*appends the next segment in the stream to "packet"
  and places the segment's size in "segment_size"*/
ogg_status
oggreader_next_segment(OggReader *reader,
                       BitstreamReader *packet,
                       uint8_t *segment_size);

/*places the next packet in Ogg stream in "packet",
  where "packet" is cleared out beforehand

  handles read errors automatically,
  so the bitstream need not be wrapped in a check for them

  an error in the stream may result in a partially filled packet*/
ogg_status
oggreader_next_packet(OggReader *reader, BitstreamReader *packet);

char *
ogg_strerror(ogg_status err);

#ifndef STANDALONE
PyObject*
ogg_exception(ogg_status err);
#endif
