/*
 * EBO-SE TUTK bridge (native, Android/bionic) for the Home Assistant appliance.
 *
 * Loads the official EBO TUTK libraries and talks to the robot directly:
 *   - connects via Kalay P2P + DTLS-PSK using the extracted credentials (from env)
 *   - emits H.265 video frames on stdout:           [u32 len LE][u8 codec][payload]
 *     (codec 80 = HEVC, 78 = H264, 0xFF = inbound MAVLink status e.g. battery,
 *      0xA0 = audio: payload is [codec_id:1][flags:1][audio data] for listen-only)
 *   - reads control commands from fd 3:             [u32 len LE][u8 kind][payload]
 *     kind 0 = MAVLink over RDT (motor / dock / lights), 1 = avSendIOCtrl
 *   - auto-reconnects, keeps the video stream alive with a keepalive.
 *
 * IMPORTANT (control): RDT_Initialize() must be called AFTER the license key is set
 * and GetLicenseKeyState() == 0, otherwise it returns -1005 (NO_LICENSE_KEY).
 *
 * Build (Android NDK):
 *   clang --target=armv7a-linux-androideabi24 -O2 ebo_bridge.c -o ebo_bridge -ldl
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

typedef int (*fn_s)(const char*);
typedef int (*fn_i2)(unsigned short);
typedef int (*fn_av)(int);
typedef int (*fn_g)(void);
typedef int (*fn_cx)(const char*, int, void*);
typedef int (*fn_sx)(void*, void*);
typedef int (*fn_rv)(int, char*, int, int*, int*, char*, int, int*, int*);
typedef int (*fn_ra)(int, char*, int, char*, int, unsigned int*);   /* avRecvAudioData */
typedef int (*fn_io)(int, unsigned int, const char*, int);
typedef int (*fn_stop)(int);
typedef int (*fn_rdti)(void);
typedef int (*fn_rdtc)(int, int, int);
typedef int (*fn_rdtw)(int, const char*, unsigned int);
typedef int (*fn_rdtr)(int, char*, unsigned int, unsigned int);

static fn_s setlic; static fn_i2 init2; static fn_av avinit; static fn_g iotc_getsid;
static fn_cx connectex; static fn_sx avstartex; static fn_rv recv2; static fn_ra recva; static fn_io sendio; static fn_stop avstop;
static fn_rdti rdt_init; static fn_rdtc rdt_create; static fn_rdtw rdt_write; static fn_rdtr rdt_read;

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

int main(void){
  setvbuf(stderr, NULL, _IONBF, 0);
  const char *LIB=getenv("EBO_LIB_DIR"); if(!LIB) LIB="/opt/ebo/lib";
  const char *LIC=getenv("EBO_LICENSE"), *UID=getenv("EBO_UID"), *AK=getenv("EBO_AUTHKEY"),
             *ID=getenv("EBO_IDENTITY"), *TK=getenv("EBO_TOKEN"), *I9=getenv("EBO_IOCTL9930");
  if(!LIC||!UID||!AK||!ID||!TK){ fprintf(stderr,"[bridge] missing one of EBO_LICENSE/UID/AUTHKEY/IDENTITY/TOKEN\n"); return 1; }

  char path[512];
  snprintf(path,sizeof path,"%s/libTUTKGlobalAPIs.so",LIB); L(path);
  snprintf(path,sizeof path,"%s/libIOTCAPIs.so",LIB); L(path);
  snprintf(path,sizeof path,"%s/libRDTAPIs.so",LIB); L(path);
  snprintf(path,sizeof path,"%s/libAVAPIs.so",LIB); L(path);
  setlic=S("TUTK_SDK_Set_License_Key"); init2=S("IOTC_Initialize2"); avinit=S("avInitialize");
  iotc_getsid=S("IOTC_Get_SessionID"); connectex=S("IOTC_Connect_ByUIDEx"); avstartex=S("avClientStartEx");
  recv2=S("avRecvFrameData2"); recva=S("avRecvAudioData"); sendio=S("avSendIOCtrl"); avstop=S("avClientStop");
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

  int cfd=3; fcntl(cfd, F_SETFL, fcntl(cfd,F_GETFL,0)|O_NONBLOCK);   /* fd 3 = control input */
  unsigned char cbuf[4096]; int cpos=0;

  for(;;){ /* (re)connect loop */
    int slot=iotc_getsid();
    unsigned char cin[152]; memset(cin,0,sizeof cin); *(unsigned*)cin=152; strncpy((char*)cin+8,AK,120); *(unsigned*)(cin+144)=15;
    int sid=connectex(UID, slot, cin);
    if(sid<0){ fprintf(stderr,"[bridge] IOTC connect %d, retry in 3s\n",sid); sleep(3); continue; }
    unsigned char inc[44]; memset(inc,0,sizeof inc); *(unsigned*)inc=44; *(int*)(inc+4)=sid; *(unsigned*)(inc+12)=10;
    *(const char**)(inc+16)=ID; *(const char**)(inc+20)=TK; *(unsigned*)(inc+24)=1; *(unsigned*)(inc+28)=2; *(unsigned*)(inc+32)=1;
    unsigned char out[32]; memset(out,0,sizeof out); *(unsigned*)out=32;
    int av=avstartex(inc,out);
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
    while(alive){
      int n=recv2(av, buf, 1024*1024, &fa,&fb, fi, 64, &fc,&fd2);
      if(n>0){
        unsigned char hdr[5]; *(unsigned*)hdr=(unsigned)n; hdr[4]=(unsigned char)(fi[0]); /* codec id = frameinfo byte 0 */
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
          unsigned char ah[7]; *(unsigned*)ah=(unsigned)(an+2); ah[4]=0xA0; ah[5]=acodec; ah[6]=aflags;
          if(writeall(1,ah,7)<0 || writeall(1,abuf,an)<0){ alive=0; break; }
        }
      }

      time_t now=time(NULL);
      if(now-last_ka>=10){ sendio(av,0x1ff,(const char*)z8,8); last_ka=now; }   /* stream keepalive */

      /* inbound status (battery, etc.) -> stdout status frame, codec 0xFF */
      if(rdt_read && rdtch>=0){
        int rn=rdt_read(rdtch, rbuf, sizeof(rbuf), 0);
        if(rn>0){ unsigned char shdr[5]; *(unsigned*)shdr=(unsigned)rn; shdr[4]=0xFF;
          if(writeall(1,shdr,5)<0 || writeall(1,rbuf,rn)<0){ alive=0; break; } }
      }

      /* control commands from fd 3 */
      int r=read(cfd, cbuf+cpos, sizeof(cbuf)-cpos);
      if(r>0){ cpos+=r;
        while(cpos>=4){ unsigned clen=*(unsigned*)cbuf; if(clen>sizeof(cbuf)-4||cpos<4+(int)clen) break;
          if(clen>=1){ unsigned char kind=cbuf[4]; int wr=0;
            if(kind==0 && rdt_write && rdtch>=0){ wr=rdt_write(rdtch, (const char*)(cbuf+5), clen-1); }
            else if(kind==1 && clen>=3){ unsigned io=*(unsigned short*)(cbuf+5); wr=sendio(av, io, (const char*)(cbuf+7), clen-3); }
            else { fprintf(stderr,"[bridge] cmd kind=%d not handled (rdt=%d)\n",kind,rdtch); }
            if(wr<0) fprintf(stderr,"[bridge] send command err=%d\n",wr);
          }
          memmove(cbuf, cbuf+4+clen, cpos-4-clen); cpos-=4+clen;
        }
      }
    }
    if(avstop) avstop(av);
    free(buf); free(abuf);
    sleep(2);
  }
  return 0;
}
