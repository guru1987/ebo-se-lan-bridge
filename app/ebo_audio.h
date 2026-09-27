#ifndef EBO_AUDIO_H
#define EBO_AUDIO_H

#include <stddef.h>
#include <stdint.h>

#define EBO_PCM16_MAX_BYTES 1280u
#define EBO_AUDIO_FRAME_INFO_SIZE 16u

/* ROLA's pcm2G711a JNI consumes signed 16-bit native PCM and emits one
 * G.711 A-law byte per sample.  The fd3 protocol is explicitly PCM16_LE. */
uint8_t ebo_linear_to_alaw(int16_t sample);
size_t ebo_pcm16le_to_alaw(const uint8_t *pcm, size_t pcm_len,
                           uint8_t *alaw, size_t alaw_capacity);

/* TUTK audio frameinfo used by ROLA: codec 0x008a, flags 0x0e, and the low
 * 32 bits of wall-clock milliseconds at bytes 12..15, all multibyte fields LE. */
void ebo_make_audio_frame_info(uint8_t frame_info[EBO_AUDIO_FRAME_INFO_SIZE],
                               uint32_t timestamp_ms);

/* Eight-byte IOCTRL payloads paired with 0x350 (start) and 0x351 (stop). */
void ebo_make_speaker_start_payload(uint8_t payload[8], int reuse_channel);
void ebo_make_speaker_stop_payload(uint8_t payload[8], int had_channel);

#endif
