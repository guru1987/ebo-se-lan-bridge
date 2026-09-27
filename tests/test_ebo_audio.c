#include "../app/ebo_audio.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static void test_known_alaw_values(void) {
  /* Values are derived from the owned ROLA libAACEncode.so exported
   * _Z11linear2alawi at 0x21154; -1 => 0x5a is intentional ROLA behavior. */
  assert(ebo_linear_to_alaw(0) == 0xd5u);
  assert(ebo_linear_to_alaw(16) == 0xd4u);
  assert(ebo_linear_to_alaw(256) == 0xc5u);
  assert(ebo_linear_to_alaw(32767) == 0xaau);
  assert(ebo_linear_to_alaw(-1) == 0x5au);
  assert(ebo_linear_to_alaw(-8) == 0x55u);
  assert(ebo_linear_to_alaw(-32768) == 0x2au);
}

static void test_pcm16le_conversion(void) {
  const uint8_t pcm[] = {
    0x00, 0x00, /* 0 */
    0x10, 0x00, /* 16 */
    0xff, 0xff, /* -1 */
    0xf8, 0xff  /* -8 */
  };
  const uint8_t expected[] = {0xd5u, 0xd4u, 0x5au, 0x55u};
  uint8_t encoded[sizeof expected];

  assert(ebo_pcm16le_to_alaw(pcm, sizeof pcm, encoded, sizeof encoded) ==
         sizeof expected);
  assert(memcmp(encoded, expected, sizeof expected) == 0);
  assert(ebo_pcm16le_to_alaw(pcm, sizeof pcm - 1u, encoded,
                             sizeof encoded) == 0u);
  assert(ebo_pcm16le_to_alaw(pcm, sizeof pcm, encoded,
                             sizeof encoded - 1u) == 0u);
}

static void test_frame_info(void) {
  const uint8_t expected[EBO_AUDIO_FRAME_INFO_SIZE] = {
    0x8a, 0x00, 0x0e, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x78, 0x56, 0x34, 0x12
  };
  uint8_t frame_info[EBO_AUDIO_FRAME_INFO_SIZE];

  ebo_make_audio_frame_info(frame_info, 0x12345678u);
  assert(memcmp(frame_info, expected, sizeof expected) == 0);
}

static void test_speaker_control_payloads(void) {
  const uint8_t start_new[8] = {2, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t start_reuse[8] = {2, 0, 0, 0, 1, 0, 0, 0};
  const uint8_t stop_inactive[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  const uint8_t stop_active[8] = {0, 0, 0, 0, 1, 0, 0, 0};
  uint8_t payload[8];

  ebo_make_speaker_start_payload(payload, 0);
  assert(memcmp(payload, start_new, sizeof payload) == 0);
  ebo_make_speaker_start_payload(payload, 1);
  assert(memcmp(payload, start_reuse, sizeof payload) == 0);
  ebo_make_speaker_stop_payload(payload, 0);
  assert(memcmp(payload, stop_inactive, sizeof payload) == 0);
  ebo_make_speaker_stop_payload(payload, 1);
  assert(memcmp(payload, stop_active, sizeof payload) == 0);
}

int main(void) {
  test_known_alaw_values();
  test_pcm16le_conversion();
  test_frame_info();
  test_speaker_control_payloads();
  puts("ebo audio tests: ok");
  return 0;
}
