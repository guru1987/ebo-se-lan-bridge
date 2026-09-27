#include "ebo_audio.h"

#include <string.h>

uint8_t ebo_linear_to_alaw(int16_t sample) {
  /* Byte-for-byte transcription of _Z11linear2alawi at virtual address
   * 0x21154 in the owned ROLA arm64 libAACEncode.so (GNU build ID
   * 803a493fd0fdcd5b11bf5dfce01121269a8225dc).  Its negative path is
   * magnitude=-8-sample with mask 0x55, so -1 intentionally encodes as 0x5a
   * rather than being normalized to another library's A-law convention. */
  int32_t magnitude;
  uint8_t mask;
  unsigned segment;
  uint32_t quantized;

  if (sample >= 0) {
    magnitude = sample;
    mask = 0xd5u;
  } else {
    magnitude = -8 - (int32_t)sample;
    mask = 0x55u;
  }

  if (magnitude >= 32768) return (uint8_t)(mask ^ 0x7fu);

  if (magnitude < 256) {
    segment = 0;
    quantized = ((uint32_t)magnitude >> 4) & 0x0fu;
  } else if (magnitude < 512) {
    segment = 1;
    quantized = ((uint32_t)magnitude >> 4) & 0x0fu;
  } else {
    if (magnitude < 1024) segment = 2;
    else if (magnitude < 2048) segment = 3;
    else if (magnitude < 4096) segment = 4;
    else if (magnitude < 8192) segment = 5;
    else if (magnitude < 16384) segment = 6;
    else segment = 7;
    quantized = ((uint32_t)magnitude >> (segment + 3u)) & 0x0fu;
  }

  return (uint8_t)(((segment << 4u) | quantized) ^ mask);
}

size_t ebo_pcm16le_to_alaw(const uint8_t *pcm, size_t pcm_len,
                           uint8_t *alaw, size_t alaw_capacity) {
  size_t sample_count;
  size_t i;

  if (!pcm || !alaw || (pcm_len & 1u) != 0u) return 0;
  sample_count = pcm_len / 2u;
  if (sample_count > alaw_capacity) return 0;

  for (i = 0; i < sample_count; ++i) {
    uint16_t raw = (uint16_t)pcm[i * 2u] |
                   ((uint16_t)pcm[i * 2u + 1u] << 8u);
    alaw[i] = ebo_linear_to_alaw((int16_t)raw);
  }
  return sample_count;
}

void ebo_make_audio_frame_info(uint8_t frame_info[EBO_AUDIO_FRAME_INFO_SIZE],
                               uint32_t timestamp_ms) {
  memset(frame_info, 0, EBO_AUDIO_FRAME_INFO_SIZE);
  frame_info[0] = 0x8au;
  frame_info[1] = 0x00u;
  frame_info[2] = 0x0eu;
  frame_info[12] = (uint8_t)timestamp_ms;
  frame_info[13] = (uint8_t)(timestamp_ms >> 8u);
  frame_info[14] = (uint8_t)(timestamp_ms >> 16u);
  frame_info[15] = (uint8_t)(timestamp_ms >> 24u);
}

void ebo_make_speaker_start_payload(uint8_t payload[8], int reuse_channel) {
  memset(payload, 0, 8);
  payload[0] = 2; /* channel 2 as a little-endian 32-bit integer */
  payload[4] = reuse_channel ? 1u : 0u;
}

void ebo_make_speaker_stop_payload(uint8_t payload[8], int had_channel) {
  memset(payload, 0, 8); /* ROLA sends a zero little-endian value on stop */
  payload[4] = had_channel ? 1u : 0u;
}
