/*
 * EBO-SE TUTK bridge (native, Android/bionic) for the Home Assistant appliance.
 *
 * Loads the official EBO TUTK libraries and talks to the robot directly:
 *   - connects via Kalay P2P + DTLS-PSK using the extracted credentials (from env)
 *   - emits H.265 video frames on stdout:           [u32 len LE][u8 codec][payload]
 *     (codec 80 = HEVC, 78 = H264, 0xFF = inbound MAVLink status e.g. battery,
 *      0xA0 = audio: payload is [codec_id:1][flags:1][audio data] for listen-only)
 *   - reads control commands from fd 3:             [u32 len LE][u8 kind][payload]
 *     kind 0 = MAVLink over RDT (motor / dock / lights), 1 = avSendIOCtrl,
 *          2 = speaker start, 3 = PCM16_LE/8 kHz/mono, 4 = speaker stop
 *   - auto-reconnects, keeps the video stream alive with a keepalive.
 *
 * IMPORTANT (control): RDT_Initialize() must be called AFTER the license key is set
 * and GetLicenseKeyState() == 0, otherwise it returns -1005 (NO_LICENSE_KEY).
 *
 * Build (Android NDK):
 *   NDK=/path/to/ndk scripts/build_bridge.sh
 *   EBO_ARCH=arm64 EBO_OUTPUT=/tmp/ebo_bridge-arm64 NDK=... scripts/build_bridge.sh
 * Run (inside a Docker container / PID namespace, see run.sh):
 *   ./bionic/linker /abs/path/ebo_bridge        (fd 3 = control pipe)
 */
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <dlfcn.h>
#include <unistd.h>
#include <fcntl.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>

#include "ebo_audio.h"

typedef int (*fn_token_auth)(const char*, char*, unsigned int);
typedef void (*fn_av_callback)(void);

/* These public SDK layouts were recovered from the owned ARM64 ROLA JNI
 * wrapper.  Keep the assertions: passing the old hard-coded 32-bit layout to
 * an ARM64 libAVAPIs.so corrupts every field after timeout_sec. */
typedef struct {
  uint32_t cb;
  int32_t iotc_session_id;
  int32_t iotc_channel_id;
  int32_t timeout_sec;
  const char* account_or_identity;
  const char* password_or_token;
  int32_t resend;
  int32_t security_mode;
  int32_t auth_type;
  int32_t sync_recv_data;
  const char* dtls_cipher_suites;
} AvClientStartInConfig;

typedef struct {
  uint32_t cb;
  int32_t server_type;
  int32_t resend;
  int32_t two_way_streaming;
  int32_t security_mode;
  int32_t sync_recv_data;
  uint32_t reserved[2];
} AvClientStartOutConfig;

typedef struct {
  uint32_t cb;
  int32_t iotc_session_id;
  uint8_t iotc_channel_id;
  uint8_t reserved0[3];
  int32_t timeout_sec;
  int32_t resend;
  int32_t security_mode;
  int32_t server_type;
  fn_av_callback password_auth;
  fn_token_auth token_auth;
  fn_av_callback token_request;
  fn_av_callback token_delete;
  fn_av_callback identity_array_request;
  fn_av_callback ability_request;
  fn_av_callback change_password_request;
  fn_av_callback json_request;
  const char* dtls_cipher_suites;
  int32_t disable_fec;
  int32_t accept_empty_password;
} AvServStartInConfig;

typedef struct {
  uint32_t cb;
  int32_t resend;
  int32_t two_way_streaming;
  int32_t auth_type;
  char account_or_identity[256];
} AvServStartOutConfig;

#if UINTPTR_MAX == UINT64_MAX
_Static_assert(sizeof(AvClientStartInConfig) == 56, "ARM64 AV client ABI");
_Static_assert(offsetof(AvClientStartInConfig, iotc_channel_id) == 8,
               "ARM64 AV client channel offset");
_Static_assert(offsetof(AvClientStartInConfig, account_or_identity) == 16,
               "ARM64 AV client pointer offset");
_Static_assert(offsetof(AvClientStartInConfig, resend) == 32,
               "ARM64 AV client resend offset");
_Static_assert(offsetof(AvClientStartInConfig, dtls_cipher_suites) == 48,
               "ARM64 AV client cipher offset");
_Static_assert(sizeof(AvServStartInConfig) == 112, "ARM64 AV server ABI");
_Static_assert(offsetof(AvServStartInConfig, iotc_channel_id) == 8,
               "ARM64 AV server channel offset");
_Static_assert(offsetof(AvServStartInConfig, server_type) == 24,
               "ARM64 AV server type offset");
_Static_assert(offsetof(AvServStartInConfig, token_auth) == 40,
               "ARM64 AV server token callback offset");
_Static_assert(offsetof(AvServStartInConfig, dtls_cipher_suites) == 96,
               "ARM64 AV server cipher offset");
#elif UINTPTR_MAX == UINT32_MAX
/* Natural 32-bit projection of the same public SDK fields.  It compiles for
 * XU4/armv7, but cannot be runtime-verified until owned ARM32 SDK libs exist. */
_Static_assert(sizeof(AvClientStartInConfig) == 44, "ARM32 AV client ABI");
_Static_assert(sizeof(AvServStartInConfig) == 76, "ARM32 AV server ABI");
_Static_assert(offsetof(AvServStartInConfig, token_auth) == 32,
               "ARM32 AV server token callback offset");
#else
#error Unsupported pointer size
#endif
/* The owned SDK accepts the 24-byte known prefix, while the pre-existing
 * bridge and SDK-facing callers allocate 32 bytes.  Preserve that full public
 * size and keep the unknown tail zeroed. */
_Static_assert(sizeof(AvClientStartOutConfig) == 32, "AV client output ABI");
_Static_assert(offsetof(AvClientStartOutConfig, reserved) == 24,
               "AV client output known-prefix size");
_Static_assert(sizeof(AvServStartOutConfig) == 272, "AV server output ABI");

typedef int (*fn_s)(const char*);
typedef int (*fn_i2)(unsigned short);
typedef int (*fn_av)(int);
typedef int (*fn_g)(void);
typedef int (*fn_cx)(const char*, int, void*);
typedef int (*fn_client_sx)(AvClientStartInConfig*, AvClientStartOutConfig*);
typedef int (*fn_serv_sx)(AvServStartInConfig*, AvServStartOutConfig*);
typedef int (*fn_rv)(int, char*, int, int*, int*, char*, int, int*, int*);
typedef int (*fn_ra)(int, char*, int, char*, int, unsigned int*);   /* avRecvAudioData */
typedef int (*fn_io)(int, unsigned int, const char*, int);
typedef int (*fn_stop)(int);
typedef int (*fn_send_audio)(int, const char*, int, const char*, int);
typedef void (*fn_serv_stop)(int);
typedef void (*fn_serv_exit)(int, int);
typedef int (*fn_channel_off)(int, int);
typedef int (*fn_rdti)(void);
typedef int (*fn_rdtc)(int, int, int);
typedef int (*fn_rdtw)(int, const char*, unsigned int);
typedef int (*fn_rdtr)(int, char*, unsigned int, unsigned int);

static fn_s setlic; static fn_i2 init2; static fn_av avinit; static fn_g iotc_getsid;
static fn_cx connectex; static fn_client_sx avstartex; static fn_rv recv2; static fn_ra recva; static fn_io sendio; static fn_stop avstop;
static fn_serv_sx avservstartex; static fn_send_audio avsendaudio;
static fn_serv_stop avservstop; static fn_serv_exit avservexit; static fn_channel_off channeloff;
static fn_rdti rdt_init; static fn_rdtc rdt_create; static fn_rdtw rdt_write; static fn_rdtr rdt_read;

enum { CMD_RDT=0, CMD_IOCTRL=1, CMD_SPEAKER_START=2, CMD_SPEAKER_PCM16LE=3, CMD_SPEAKER_STOP=4 };
enum { SPEAKER_IDLE=0, SPEAKER_STARTING=1, SPEAKER_READY=2 };

typedef struct {
  pthread_mutex_t lock;
  pthread_mutex_t send_lock;
  int state;
  int sid;
  int main_av;
  int speaker_av;
  int worker_joinable;
  pthread_t worker;
  unsigned generation;
} SpeakerState;

typedef struct {
  unsigned generation;
  int sid;
} SpeakerStartArgs;

static SpeakerState speaker = {
  PTHREAD_MUTEX_INITIALIZER, PTHREAD_MUTEX_INITIALIZER,
  SPEAKER_IDLE, -1, -1, -1, 0, (pthread_t)0, 0
};
static const char* speaker_auth_token;
static volatile sig_atomic_t shutting_down;

static void* L(const char* p){ void* h=dlopen(p, RTLD_NOW|RTLD_GLOBAL); if(!h){ fprintf(stderr,"[bridge] dlopen %s FAILED: %s\n",p,dlerror()); exit(2);} return h; }
static void* S(const char* n){ void* p=dlsym(RTLD_DEFAULT,n); if(!p) fprintf(stderr,"[bridge] dlsym %s = NULL\n",n); return p; }

static void load_blob(const char* path, unsigned char** out, int* outlen){
  *out=NULL; *outlen=0; if(!path) return;
  FILE* f=fopen(path,"rb"); if(!f) return;
  unsigned char* d=malloc(4096); int n=fread(d,1,4096,f); fclose(f); *out=d; *outlen=n;
}

static int writeall(int fd, const void* buf, int len){
  const char* p=buf; int left=len;
  while(left>0){ int w=write(fd,p,left); if(w<=0){ if(errno==EINTR) continue; return -1; } p+=w; left-=w; }
  return 0;
}

static uint16_t get_le16(const unsigned char* p){
  return (uint16_t)p[0] | ((uint16_t)p[1]<<8);
}
static uint32_t get_le32(const unsigned char* p){
  return (uint32_t)p[0] | ((uint32_t)p[1]<<8) | ((uint32_t)p[2]<<16) | ((uint32_t)p[3]<<24);
}
static void put_le32(unsigned char* p, uint32_t v){
  p[0]=(unsigned char)v; p[1]=(unsigned char)(v>>8); p[2]=(unsigned char)(v>>16); p[3]=(unsigned char)(v>>24);
}

static uint32_t wallclock_ms32(void){
  struct timespec ts;
  if(clock_gettime(CLOCK_REALTIME,&ts)!=0) return 0;
  return (uint32_t)((uint64_t)ts.tv_sec*1000u+(uint64_t)ts.tv_nsec/1000000u);
}

static int speaker_token_auth(const char* identity, char* token, unsigned int capacity){
  size_t len;
  const char* expected=speaker_auth_token;
  (void)identity;
  if(!expected||!token||capacity==0) return -1;
  len=strlen(expected);
  /* The JNI wrapper copies exactly the Java UTF-8 byte count (no terminator)
   * and accepts a token whose byte count equals capacity. */
  if(len>capacity) return -1;
  memcpy(token,expected,len);
  return 0;
}

static void speaker_join_worker(pthread_t worker, int joinable){
  if(!joinable) return;
  pthread_join(worker,NULL);
  pthread_mutex_lock(&speaker.lock);
  if(speaker.worker_joinable && pthread_equal(speaker.worker,worker)){
    speaker.worker_joinable=0;
  }
  pthread_mutex_unlock(&speaker.lock);
}

static void* speaker_start_thread(void* opaque){
  SpeakerStartArgs* args=(SpeakerStartArgs*)opaque;
  AvServStartInConfig in;
  AvServStartOutConfig out;
  int speak_av, cleanup_channel=0, canceled=0;

  memset(&in,0,sizeof in); memset(&out,0,sizeof out);
  in.cb=sizeof in; in.iotc_session_id=args->sid; in.iotc_channel_id=2;
  in.timeout_sec=10; in.resend=1; in.security_mode=0; in.server_type=0;
  in.token_auth=speaker_token_auth;
  out.cb=sizeof out;
  speak_av=avservstartex(&in,&out);

  pthread_mutex_lock(&speaker.lock);
  if(speaker.generation==args->generation && speaker.state==SPEAKER_STARTING){
    if(speak_av>=0){
      speaker.speaker_av=speak_av; speaker.state=SPEAKER_READY;
      fprintf(stderr,"[bridge] speaker ready av=%d channel=2\n",speak_av);
    } else {
      speaker.state=SPEAKER_IDLE; speaker.speaker_av=-1;
      speaker.sid=-1; speaker.main_av=-1; cleanup_channel=1;
      fprintf(stderr,"[bridge] avServStartEx speaker err=%d\n",speak_av);
    }
    speak_av=-1;
  } else {
    canceled=1;
  }
  pthread_mutex_unlock(&speaker.lock);

  /* A canceled worker is joined by stop_session before reconnect.  Its AV
   * index, if one materialized during cancellation, still needs stopping. */
  if(speak_av>=0){
    pthread_mutex_lock(&speaker.send_lock);
    if(avservstop) avservstop(speak_av);
    pthread_mutex_unlock(&speaker.send_lock);
  }
  /* On an ordinary start failure there is no stop_session to tear channel 2
   * down.  Complete that cleanup before this joinable worker exits. */
  if(cleanup_channel && !canceled && args->sid>=0){
    pthread_mutex_lock(&speaker.send_lock);
    if(channeloff) channeloff(args->sid,2);
    if(avservexit) avservexit(args->sid,2);
    pthread_mutex_unlock(&speaker.send_lock);
  }
  free(args);
  return NULL;
}

static int speaker_stop_session(int fallback_sid, int fallback_av, int notify_robot){
  unsigned char payload[8]={0};
  int sid, main_av, speak_av, was_ready, was_active, joinable;
  pthread_t worker=(pthread_t)0;

  pthread_mutex_lock(&speaker.send_lock);
  pthread_mutex_lock(&speaker.lock);
  was_ready=(speaker.state==SPEAKER_READY);
  was_active=(speaker.state!=SPEAKER_IDLE);
  sid=speaker.sid>=0?speaker.sid:fallback_sid;
  main_av=speaker.main_av>=0?speaker.main_av:fallback_av;
  speak_av=speaker.speaker_av;
  joinable=speaker.worker_joinable;
  if(joinable) worker=speaker.worker;
  speaker.generation++; speaker.state=SPEAKER_IDLE;
  speaker.sid=-1; speaker.main_av=-1; speaker.speaker_av=-1;
  pthread_mutex_unlock(&speaker.lock);

  ebo_make_speaker_stop_payload(payload,was_ready);
  if(notify_robot && main_av>=0 && sendio) sendio(main_av,0x351,(const char*)payload,sizeof payload);
  if(speak_av>=0 && avservstop) avservstop(speak_av);
  if(was_active && sid>=0){
    if(channeloff) channeloff(sid,2);
    if(avservexit) avservexit(sid,2);
  }
  pthread_mutex_unlock(&speaker.send_lock);
  /* avServExit above releases a worker blocked in avServStartEx.  Never join
   * under either mutex: the worker may briefly acquire send_lock to dispose
   * of an AV index returned concurrently with cancellation. */
  speaker_join_worker(worker,joinable);
  if(was_active) fprintf(stderr,"[bridge] speaker stopped\n");
  return 0;
}

static int speaker_start_session(int sid, int main_av){
  unsigned char payload[8]={0};
  SpeakerStartArgs* args;
  pthread_t thread;
  pthread_t old_worker=(pthread_t)0;
  int state, rc, old_joinable=0;

  if(!avservstartex||!avsendaudio||!avservstop||!avservexit||!channeloff) return -ENOSYS;
  pthread_mutex_lock(&speaker.lock);
  state=speaker.state;
  if(state==SPEAKER_IDLE && speaker.worker_joinable){
    old_worker=speaker.worker; old_joinable=1;
    pthread_mutex_unlock(&speaker.lock);
    speaker_join_worker(old_worker,old_joinable);
    pthread_mutex_lock(&speaker.lock);
    state=speaker.state;
  }
  if(state==SPEAKER_IDLE){
    args=malloc(sizeof *args);
    if(!args){ pthread_mutex_unlock(&speaker.lock); return -ENOMEM; }
    speaker.generation++; speaker.state=SPEAKER_STARTING;
    speaker.sid=sid; speaker.main_av=main_av; speaker.speaker_av=-1;
    args->generation=speaker.generation; args->sid=sid;
    rc=pthread_create(&thread,NULL,speaker_start_thread,args);
    if(rc!=0){
      speaker.state=SPEAKER_IDLE; speaker.sid=-1; speaker.main_av=-1;
      free(args); pthread_mutex_unlock(&speaker.lock); return -rc;
    }
    speaker.worker=thread; speaker.worker_joinable=1;
  }
  pthread_mutex_unlock(&speaker.lock);

  ebo_make_speaker_start_payload(payload,state==SPEAKER_READY);
  rc=sendio(main_av,0x350,(const char*)payload,sizeof payload);
  if(rc<0 && state==SPEAKER_IDLE) speaker_stop_session(sid,main_av,0);
  return rc;
}

static int speaker_send_pcm16le(const unsigned char* pcm, size_t pcm_len){
  unsigned char encoded[EBO_PCM16_MAX_BYTES/2u];
  unsigned char frame_info[EBO_AUDIO_FRAME_INFO_SIZE];
  size_t encoded_len;
  int rc;

  if(pcm_len==0 || pcm_len>EBO_PCM16_MAX_BYTES || (pcm_len&1u)!=0u) return -EINVAL;
  encoded_len=ebo_pcm16le_to_alaw(pcm,pcm_len,encoded,sizeof encoded);
  if(encoded_len==0) return -EINVAL;
  ebo_make_audio_frame_info(frame_info,wallclock_ms32());

  pthread_mutex_lock(&speaker.send_lock);
  pthread_mutex_lock(&speaker.lock);
  if(speaker.state!=SPEAKER_READY || speaker.speaker_av<0){
    pthread_mutex_unlock(&speaker.lock);
    pthread_mutex_unlock(&speaker.send_lock);
    return -EAGAIN;
  }
  {
    int speak_av=speaker.speaker_av;
    pthread_mutex_unlock(&speaker.lock);
    rc=avsendaudio(speak_av,(const char*)encoded,(int)encoded_len,
                 (const char*)frame_info,sizeof frame_info);
  }
  pthread_mutex_unlock(&speaker.send_lock);
  return rc;
}

static void cleanup_speaker(void){ speaker_stop_session(-1,-1,0); }
static void handle_signal(int signum){ (void)signum; shutting_down=1; }

int main(void){
  setvbuf(stderr, NULL, _IONBF, 0);
  const char *LIB=getenv("EBO_LIB_DIR"); if(!LIB) LIB="/opt/ebo/lib";
  const char *LIC=getenv("EBO_LICENSE"), *UID=getenv("EBO_UID"), *AK=getenv("EBO_AUTHKEY"),
             *ID=getenv("EBO_IDENTITY"), *TK=getenv("EBO_TOKEN"), *I9=getenv("EBO_IOCTL9930");
  if(!LIC||!UID||!AK||!ID||!TK){ fprintf(stderr,"[bridge] missing one of EBO_LICENSE/UID/AUTHKEY/IDENTITY/TOKEN\n"); return 1; }
  speaker_auth_token=TK;

  char path[512];
  snprintf(path,sizeof path,"%s/libTUTKGlobalAPIs.so",LIB); L(path);
  snprintf(path,sizeof path,"%s/libIOTCAPIs.so",LIB); L(path);
  snprintf(path,sizeof path,"%s/libRDTAPIs.so",LIB); L(path);
  snprintf(path,sizeof path,"%s/libAVAPIs.so",LIB); L(path);
  setlic=S("TUTK_SDK_Set_License_Key"); init2=S("IOTC_Initialize2"); avinit=S("avInitialize");
  iotc_getsid=S("IOTC_Get_SessionID"); connectex=S("IOTC_Connect_ByUIDEx"); avstartex=S("avClientStartEx");
  recv2=S("avRecvFrameData2"); recva=S("avRecvAudioData"); sendio=S("avSendIOCtrl"); avstop=S("avClientStop");
  avservstartex=S("avServStartEx"); avsendaudio=S("avSendAudioData"); avservstop=S("avServStop"); avservexit=S("avServExit");
  channeloff=S("IOTC_Session_Channel_OFF");
  rdt_init=S("RDT_Initialize"); rdt_create=S("RDT_Create"); rdt_write=S("RDT_Write"); rdt_read=S("RDT_Read");
  if(!setlic||!init2||!avinit||!iotc_getsid||!connectex||!avstartex||!recv2||!sendio){ fprintf(stderr,"[bridge] missing symbols\n"); return 3; }

  unsigned char* d9930; int l9930; load_blob(I9,&d9930,&l9930);

  int (*licstate)(void) = (int(*)(void))S("GetLicenseKeyState");
  if(setlic(LIC)!=0){ fprintf(stderr,"[bridge] license rejected\n"); return 4; }
  init2(0); avinit(64);
  /* Wait for the license to validate (state 0), THEN init RDT (control channel).
   * RDT_Initialize before license validation returns -1005 NO_LICENSE_KEY. */
  if(licstate){ for(int k=0;k<10 && licstate()!=0; k++) sleep(1); }
  if(rdt_init){ int ri=rdt_init(); fprintf(stderr,"[bridge] RDT_Initialize=%d (control %s)\n",ri, ri>=0?"ready":"failed"); }
  fprintf(stderr,"[bridge] init done\n");

  atexit(cleanup_speaker);
  signal(SIGINT,handle_signal); signal(SIGTERM,handle_signal);

  int cfd=3; fcntl(cfd, F_SETFL, fcntl(cfd,F_GETFL,0)|O_NONBLOCK);   /* fd 3 = control input */
  unsigned char cbuf[4096]; int cpos=0;

  while(!shutting_down){ /* (re)connect loop */
    int slot=iotc_getsid();
    unsigned char cin[152]; memset(cin,0,sizeof cin); put_le32(cin,152); strncpy((char*)cin+8,AK,120); put_le32(cin+144,15);
    int sid=connectex(UID, slot, cin);
    if(sid<0){ fprintf(stderr,"[bridge] IOTC connect %d, retry in 3s\n",sid); sleep(3); continue; }
    AvClientStartInConfig inc; AvClientStartOutConfig out;
    memset(&inc,0,sizeof inc); memset(&out,0,sizeof out);
    inc.cb=sizeof inc; inc.iotc_session_id=sid; inc.iotc_channel_id=0; inc.timeout_sec=10;
    inc.account_or_identity=ID; inc.password_or_token=TK; inc.resend=1; inc.security_mode=2; inc.auth_type=1;
    out.cb=sizeof out;
    int av=avstartex(&inc,&out);
    if(av<0){ fprintf(stderr,"[bridge] avClientStartEx %d, retry in 3s\n",av); sleep(3); continue; }
    int rdtch = rdt_create ? rdt_create(sid, 5000, 1) : -1;   /* control channel (MAVLink over RDT) */
    fprintf(stderr,"[bridge] connected SID=%d av=%d rdt=%d\n",sid,av,rdtch);

    /* request the video stream */
    unsigned char z8[8]; memset(z8,0,8);
    sendio(av,0xff,(const char*)z8,4);
    if(d9930) sendio(av,0x9930,(const char*)d9930,l9930);
    sendio(av,0x32a,(const char*)z8,8); sendio(av,0x1ff,(const char*)z8,8); sendio(av,0x9936,(const char*)z8,8);
    sendio(av,0x300,(const char*)z8,8);   /* IOTYPE_USER_IPCAM_AUDIOSTART (best-effort) */

    char* buf=malloc(1024*1024); char fi[64]; int fa,fb,fc,fd2;
    char* abuf=malloc(65536); char afi[64];     /* audio frame + frameinfo */
    char rbuf[4096];   /* inbound MAVLink (battery / status) via RDT_Read */
    time_t last_ka=time(NULL);
    int audlog=0;      /* log the first few audio frames' codec/rate */
    int alive=1;
    while(alive && !shutting_down){
      int n=recv2(av, buf, 1024*1024, &fa,&fb, fi, 64, &fc,&fd2);
      if(n>0){
        unsigned char hdr[5]; put_le32(hdr,(uint32_t)n); hdr[4]=(unsigned char)(fi[0]); /* codec id = frameinfo byte 0 */
        if(writeall(1,hdr,5)<0 || writeall(1,buf,n)<0){ alive=0; break; }
      } else if(n==-20012){ usleep(2000); }            /* AV_ER_DATA_NOREADY */
      else if(n<=-20015 && n>=-20020){ fprintf(stderr,"[bridge] session lost %d\n",n); alive=0; }
      else usleep(2000);

      /* audio (listen-only) -> stdout: [u32 len][0xA0][codec_id:1][flags:1][data] */
      if(recva){
        unsigned int aidx=0;
        int an=recva(av, abuf, 65536, afi, 64, &aidx);
        if(an>0){
          unsigned char acodec=(unsigned char)afi[0], aflags=(unsigned char)afi[2];
          if(audlog<6){ fprintf(stderr,"[bridge] AUDIO codec=0x%02x flags=0x%02x size=%d\n",acodec,aflags,an); audlog++; }
          unsigned char ah[7]; put_le32(ah,(uint32_t)(an+2)); ah[4]=0xA0; ah[5]=acodec; ah[6]=aflags;
          if(writeall(1,ah,7)<0 || writeall(1,abuf,an)<0){ alive=0; break; }
        }
      }

      time_t now=time(NULL);
      if(now-last_ka>=10){ sendio(av,0x1ff,(const char*)z8,8); last_ka=now; }   /* stream keepalive */

      /* inbound status (battery, etc.) -> stdout status frame, codec 0xFF */
      if(rdt_read && rdtch>=0){
        int rn=rdt_read(rdtch, rbuf, sizeof(rbuf), 0);
        if(rn>0){ unsigned char shdr[5]; put_le32(shdr,(uint32_t)rn); shdr[4]=0xFF;
          if(writeall(1,shdr,5)<0 || writeall(1,rbuf,rn)<0){ alive=0; break; } }
      }

      /* control commands from fd 3 */
      int r=read(cfd, cbuf+cpos, sizeof(cbuf)-cpos);
      if(r>0){ cpos+=r;
        while(cpos>=4){ unsigned clen=get_le32(cbuf);
          if(clen>sizeof(cbuf)-4){
            fprintf(stderr,"[bridge] invalid fd3 frame length=%u; dropping buffered input\n",clen);
            cpos=0; break;
          }
          if(cpos<4+(int)clen) break;
          if(clen>=1){ unsigned char kind=cbuf[4]; int wr=0;
            if(kind==CMD_RDT && rdt_write && rdtch>=0){ wr=rdt_write(rdtch, (const char*)(cbuf+5), clen-1); }
            else if(kind==CMD_IOCTRL && clen>=3){ unsigned io=get_le16(cbuf+5); wr=sendio(av, io, (const char*)(cbuf+7), clen-3); }
            else if(kind==CMD_SPEAKER_START){ wr=speaker_start_session(sid,av); }
            else if(kind==CMD_SPEAKER_PCM16LE){ wr=speaker_send_pcm16le(cbuf+5,clen-1); }
            else if(kind==CMD_SPEAKER_STOP){ wr=speaker_stop_session(sid,av,1); }
            else { fprintf(stderr,"[bridge] cmd kind=%d not handled (rdt=%d)\n",kind,rdtch); }
            if(wr<0) fprintf(stderr,"[bridge] send command err=%d\n",wr);
          }
          memmove(cbuf, cbuf+4+clen, cpos-4-clen); cpos-=4+clen;
        }
      }
    }
    speaker_stop_session(sid,av,1);
    if(avstop) avstop(av);
    free(buf); free(abuf);
    if(!shutting_down) sleep(2);
  }
  return 0;
}
