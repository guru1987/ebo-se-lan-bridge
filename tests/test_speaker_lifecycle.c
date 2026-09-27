#define main ebo_bridge_program_main
#include "../app/ebo_bridge.c"
#undef main

#include <assert.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>

static pthread_mutex_t mock_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t mock_cond = PTHREAD_COND_INITIALIZER;
static int mock_start_entered;
static int mock_start_released;
static int mock_stop_calls;
static int mock_exit_calls;
static int mock_off_calls;
static int mock_send_without_state_lock;

static int mock_serv_start(AvServStartInConfig* in, AvServStartOutConfig* out) {
  char token[10];
  char too_small[9];
  assert(in->iotc_channel_id == 2);
  assert(in->token_auth != NULL);
  assert(in->token_auth("identity", token, sizeof token) == 0);
  assert(memcmp(token, "test-token", sizeof token) == 0);
  assert(in->token_auth("identity", too_small, sizeof too_small) < 0);
  assert(out->cb == sizeof *out);
  pthread_mutex_lock(&mock_lock);
  mock_start_entered = 1;
  pthread_cond_broadcast(&mock_cond);
  while (!mock_start_released) pthread_cond_wait(&mock_cond, &mock_lock);
  pthread_mutex_unlock(&mock_lock);
  return 7;
}

static int mock_send_io(int av, unsigned int type, const char* payload, int len) {
  (void)av;
  assert((type == 0x350u || type == 0x351u) && len == 8);
  assert(payload != NULL);
  return 0;
}

static int mock_send_audio(int av, const char* data, int len,
                           const char* frame_info, int frame_info_len) {
  int lock_result;
  (void)av;
  assert(data != NULL && len == 1);
  assert(frame_info != NULL && frame_info_len == 16);
  lock_result = pthread_mutex_trylock(&speaker.lock);
  if (lock_result == 0) {
    mock_send_without_state_lock = 1;
    pthread_mutex_unlock(&speaker.lock);
  }
  return 0;
}

static void mock_serv_stop(int av) {
  assert(av == 7 || av == 9);
  mock_stop_calls++;
}

static void mock_serv_exit(int sid, int channel) {
  assert(sid == 42 && channel == 2);
  mock_exit_calls++;
  pthread_mutex_lock(&mock_lock);
  mock_start_released = 1;
  pthread_cond_broadcast(&mock_cond);
  pthread_mutex_unlock(&mock_lock);
}

static int mock_channel_off(int sid, int channel) {
  assert(sid == 42 && channel == 2);
  mock_off_calls++;
  return 0;
}

static void wait_for_start(void) {
  pthread_mutex_lock(&mock_lock);
  while (!mock_start_entered) pthread_cond_wait(&mock_cond, &mock_lock);
  pthread_mutex_unlock(&mock_lock);
}

static void test_cancel_joins_worker(void) {
  assert(speaker_start_session(42, 11) == 0);
  wait_for_start();
  assert(speaker_stop_session(42, 11, 1) == 0);

  pthread_mutex_lock(&speaker.lock);
  assert(speaker.state == SPEAKER_IDLE);
  assert(speaker.worker_joinable == 0);
  assert(speaker.speaker_av == -1);
  pthread_mutex_unlock(&speaker.lock);
  assert(mock_exit_calls == 1);
  assert(mock_off_calls == 1);
  assert(mock_stop_calls == 1);
}

static void test_send_does_not_hold_state_lock(void) {
  const unsigned char pcm[2] = {0, 0};

  pthread_mutex_lock(&speaker.lock);
  speaker.state = SPEAKER_READY;
  speaker.sid = 42;
  speaker.main_av = 11;
  speaker.speaker_av = 9;
  pthread_mutex_unlock(&speaker.lock);

  assert(speaker_send_pcm16le(pcm, sizeof pcm) == 0);
  assert(mock_send_without_state_lock == 1);
  assert(speaker_stop_session(42, 11, 0) == 0);
}

int main(void) {
  avservstartex = mock_serv_start;
  avsendaudio = mock_send_audio;
  avservstop = mock_serv_stop;
  avservexit = mock_serv_exit;
  channeloff = mock_channel_off;
  sendio = mock_send_io;
  speaker_auth_token = "test-token";

  test_cancel_joins_worker();
  test_send_does_not_hold_state_lock();
  puts("speaker lifecycle tests: ok");
  return 0;
}
