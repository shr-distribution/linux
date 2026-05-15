/* Decompiled from: v4l2-qcamera-app */
/* Architecture: ARM:LE:32:v8 */

/* ========================================
 * Function: __libc_init
 * Address: 00010f54
 * ======================================== */

void __libc_init(void)

{
  (*(code *)&LAB_00000f40)();
  return;
}



/* ========================================
 * Function: ioctl
 * Address: 00010f60
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int ioctl(int __fd,ulong __request,...)

{
  int iVar1;
  
  iVar1 = ioctl(__fd,__request);
  return iVar1;
}



/* ========================================
 * Function: __stack_chk_fail
 * Address: 00010f6c
 * ======================================== */

void __stack_chk_fail(void)

{
                    /* WARNING: Subroutine does not return */
  __stack_chk_fail();
}



/* ========================================
 * Function: puts
 * Address: 00010f78
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int puts(char *__s)

{
  int iVar1;
  
  iVar1 = puts(__s);
  return iVar1;
}



/* ========================================
 * Function: printf
 * Address: 00010f84
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int printf(char *__format,...)

{
  int iVar1;
  
  iVar1 = printf(__format);
  return iVar1;
}



/* ========================================
 * Function: memset
 * Address: 00010f90
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * memset(void *__s,int __c,size_t __n)

{
  void *pvVar1;
  
  pvVar1 = memset(__s,__c,__n);
  return pvVar1;
}



/* ========================================
 * Function: __errno
 * Address: 00010f9c
 * ======================================== */

void __errno(void)

{
  __errno();
  return;
}



/* ========================================
 * Function: perror
 * Address: 00010fa8
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void perror(char *__s)

{
  perror(__s);
  return;
}



/* ========================================
 * Function: exit
 * Address: 00010fb4
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void exit(int __status)

{
                    /* WARNING: Subroutine does not return */
  exit(__status);
}



/* ========================================
 * Function: fputs
 * Address: 00010fc0
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int fputs(char *__s,FILE *__stream)

{
  int iVar1;
  
  iVar1 = fputs(__s,__stream);
  return iVar1;
}



/* ========================================
 * Function: munmap
 * Address: 00010fcc
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int munmap(void *__addr,size_t __len)

{
  int iVar1;
  
  iVar1 = munmap(__addr,__len);
  return iVar1;
}



/* ========================================
 * Function: do_munmap_ion
 * Address: 00010fd8
 * ======================================== */

void do_munmap_ion(void)

{
  do_munmap_ion();
  return;
}



/* ========================================
 * Function: free
 * Address: 00010fe4
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void free(void *__ptr)

{
  free(__ptr);
  return;
}



/* ========================================
 * Function: __android_log_print
 * Address: 00010ff0
 * ======================================== */

void __android_log_print(void)

{
  __android_log_print();
  return;
}



/* ========================================
 * Function: strerror
 * Address: 00010ffc
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * strerror(int __errnum)

{
  char *pcVar1;
  
  pcVar1 = strerror(__errnum);
  return pcVar1;
}



/* ========================================
 * Function: malloc
 * Address: 00011008
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * malloc(size_t __size)

{
  void *pvVar1;
  
  pvVar1 = malloc(__size);
  return pvVar1;
}



/* ========================================
 * Function: mmap
 * Address: 00011014
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * mmap(void *__addr,size_t __len,int __prot,int __flags,int __fd,__off_t __offset)

{
  void *pvVar1;
  
  pvVar1 = mmap(__addr,__len,__prot,__flags,__fd,__offset);
  return pvVar1;
}



/* ========================================
 * Function: do_mmap_ion
 * Address: 00011020
 * ======================================== */

void do_mmap_ion(void)

{
  do_mmap_ion();
  return;
}



/* ========================================
 * Function: open
 * Address: 0001102c
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int open(char *__file,int __oflag,...)

{
  int iVar1;
  
  iVar1 = open(__file,__oflag);
  return iVar1;
}



/* ========================================
 * Function: close
 * Address: 00011038
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int close(int __fd)

{
  int iVar1;
  
  iVar1 = close(__fd);
  return iVar1;
}



/* ========================================
 * Function: use_overlay_fb_display_driver
 * Address: 00011044
 * ======================================== */

void use_overlay_fb_display_driver(void)

{
  use_overlay_fb_display_driver();
  return;
}



/* ========================================
 * Function: launch_camframe_fb_thread
 * Address: 00011050
 * ======================================== */

void launch_camframe_fb_thread(void)

{
  launch_camframe_fb_thread();
  return;
}



/* ========================================
 * Function: launch_camframe_v4l2_thread
 * Address: 0001105c
 * ======================================== */

void launch_camframe_v4l2_thread(void)

{
  launch_camframe_v4l2_thread();
  return;
}



/* ========================================
 * Function: release_camframe_v4l2_thread
 * Address: 00011068
 * ======================================== */

void release_camframe_v4l2_thread(void)

{
  release_camframe_v4l2_thread();
  return;
}



/* ========================================
 * Function: release_camframe_fb_thread
 * Address: 00011074
 * ======================================== */

void release_camframe_fb_thread(void)

{
  release_camframe_fb_thread();
  return;
}



/* ========================================
 * Function: cam_get_snapshot_images
 * Address: 00011080
 * ======================================== */

void cam_get_snapshot_images(void)

{
  cam_get_snapshot_images();
  return;
}



/* ========================================
 * Function: cam_get_raw_images
 * Address: 0001108c
 * ======================================== */

void cam_get_raw_images(void)

{
  cam_get_raw_images();
  return;
}



/* ========================================
 * Function: jpeg_encoder_hw_inline_config
 * Address: 00011098
 * ======================================== */

void jpeg_encoder_hw_inline_config(void)

{
  jpeg_encoder_hw_inline_config();
  return;
}



/* ========================================
 * Function: cam_get_thumbnail_images
 * Address: 000110a4
 * ======================================== */

void cam_get_thumbnail_images(void)

{
  cam_get_thumbnail_images();
  return;
}



/* ========================================
 * Function: jpeg_encoder_hw_inline_wait_done
 * Address: 000110b0
 * ======================================== */

void jpeg_encoder_hw_inline_wait_done(void)

{
  jpeg_encoder_hw_inline_wait_done();
  return;
}



/* ========================================
 * Function: usleep
 * Address: 000110bc
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int usleep(__useconds_t __useconds)

{
  int iVar1;
  
  iVar1 = usleep(__useconds);
  return iVar1;
}



/* ========================================
 * Function: putchar
 * Address: 000110c8
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int putchar(int __c)

{
  int iVar1;
  
  iVar1 = putchar(__c);
  return iVar1;
}



/* ========================================
 * Function: getopt
 * Address: 000110d4
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int getopt(int ___argc,char **___argv,char *__shortopts)

{
  int iVar1;
  
  iVar1 = getopt(___argc,___argv,__shortopts);
  return iVar1;
}



/* ========================================
 * Function: dlopen
 * Address: 000110e0
 * ======================================== */

void dlopen(void)

{
  dlopen();
  return;
}



/* ========================================
 * Function: dlsym
 * Address: 000110ec
 * ======================================== */

void dlsym(void)

{
  dlsym();
  return;
}



/* ========================================
 * Function: gettimeofday
 * Address: 000110f8
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int gettimeofday(timeval *__tv,__timezone_ptr_t __tz)

{
  int iVar1;
  
  iVar1 = gettimeofday(__tv,__tz);
  return iVar1;
}



/* ========================================
 * Function: __srget
 * Address: 00011104
 * ======================================== */

void __srget(void)

{
  __srget();
  return;
}



/* ========================================
 * Function: getc
 * Address: 00011110
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int getc(FILE *__stream)

{
  int iVar1;
  
  iVar1 = getc(__stream);
  return iVar1;
}



/* ========================================
 * Function: snprintf
 * Address: 0001111c
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int snprintf(char *__s,size_t __maxlen,char *__format,...)

{
  int iVar1;
  
  iVar1 = snprintf(__s,__maxlen,__format);
  return iVar1;
}



/* ========================================
 * Function: fgets
 * Address: 00011128
 * ======================================== */

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * fgets(char *__s,int __n,FILE *__stream)

{
  char *pcVar1;
  
  pcVar1 = fgets(__s,__n,__stream);
  return pcVar1;
}



/* ========================================
 * Function: dlclose
 * Address: 00011134
 * ======================================== */

void dlclose(void)

{
  dlclose();
  return;
}



/* ========================================
 * Function: entry
 * Address: 00011140
 * ======================================== */

void processEntry entry(void)

{
  __libc_init();
  (*(code *)0x0)();
  return;
}



/* ========================================
 * Function: snapshot_init
 * Address: 00012e8c
 * ======================================== */

int snapshot_init(int param_1)

{
  undefined4 uVar1;
  undefined4 uVar2;
  int iVar3;
  int iVar4;
  undefined4 local_30;
  undefined4 local_2c;
  
  iVar3 = FUN_00012b54();
  if (iVar3 == 0) {
    DAT_00019068 = open(&DAT_0001ac7c,0x802);
    if (-1 < DAT_00019068) {
      DAT_00019064 = open(&DAT_0001ac7c,0x802);
      if (-1 < DAT_00019064) {
        iVar3 = FUN_000111e4(DAT_00019064,4);
        if (iVar3 < 0) {
          return iVar3;
        }
        iVar3 = FUN_000111e4(DAT_00019068,3);
        if (iVar3 < 0) {
          return iVar3;
        }
        local_30 = 0x8000007;
        local_2c = 3;
        iVar3 = ioctl(DAT_00019068,0xc008561c,&local_30);
        if (iVar3 < 0) {
          return iVar3;
        }
        FUN_0001281c(4);
        FUN_000127c8(DAT_00019064,1,0x1a4,DAT_00019084);
        iVar3 = ioctl(DAT_00019064,0xc0cc5605,&DAT_0001a9fc);
        if (iVar3 < 0) {
          return iVar3;
        }
        iVar3 = FUN_00012454(DAT_00019064,DAT_00019004,&DAT_0001a55c,DAT_0001902c,4);
        if (iVar3 < 0) {
          return iVar3;
        }
        FUN_0001281c(3);
        FUN_000127c8(DAT_00019068,1,0x1a4,DAT_00019084);
        iVar3 = ioctl(DAT_00019068,0xc0cc5605,&DAT_0001a9fc);
        if (iVar3 < 0) {
          return iVar3;
        }
        iVar4 = FUN_00012454(DAT_00019068,DAT_00019004,&DAT_0001a1e4,DAT_0001903c,3);
        iVar3 = DAT_00019068;
        if (iVar4 < 0) {
          return iVar4;
        }
        *(int *)(param_1 + 0x18) = DAT_00019064;
        uVar1 = DAT_0001902c;
        *(int *)(param_1 + 0x14) = iVar3;
        uVar2 = DAT_0001903c;
        *(undefined4 *)(param_1 + 0x20) = uVar1;
        *(undefined4 *)(param_1 + 0x1c) = uVar2;
        *(undefined **)(param_1 + 0x2c) = &DAT_0001a55c;
        *(undefined **)(param_1 + 0x24) = &DAT_0001a1e4;
        *(undefined4 *)(param_1 + 0x34) = 1;
        return 0;
      }
      close(DAT_00019068);
    }
  }
  else {
    __android_log_print(6,"mm-camera","Invalid formats\n");
  }
  return -1;
}



/* ========================================
 * Function: __aeabi_unwind_cpp_pr0
 * Address: 0001b000
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void __aeabi_unwind_cpp_pr0(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: malloc
 * Address: 0001b004
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * malloc(size_t __size)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: free
 * Address: 0001b008
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void free(void *__ptr)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: __android_log_print
 * Address: 0001b00c
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void __android_log_print(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: memset
 * Address: 0001b010
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * memset(void *__s,int __c,size_t __n)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: ioctl
 * Address: 0001b014
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int ioctl(int __fd,ulong __request,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: open
 * Address: 0001b018
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int open(char *__file,int __oflag,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: usleep
 * Address: 0001b01c
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int usleep(__useconds_t __useconds)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: __errno
 * Address: 0001b020
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void __errno(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: strerror
 * Address: 0001b024
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * strerror(int __errnum)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: snprintf
 * Address: 0001b028
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int snprintf(char *__s,size_t __maxlen,char *__format,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: close
 * Address: 0001b030
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int close(int __fd)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: __stack_chk_fail
 * Address: 0001b034
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void __stack_chk_fail(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: __aeabi_unwind_cpp_pr1
 * Address: 0001b038
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void __aeabi_unwind_cpp_pr1(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: gettimeofday
 * Address: 0001b03c
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int gettimeofday(timeval *__tv,__timezone_ptr_t __tz)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: launch_camframe_v4l2_thread
 * Address: 0001b040
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void launch_camframe_v4l2_thread(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: release_camframe_v4l2_thread
 * Address: 0001b044
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void release_camframe_v4l2_thread(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: do_mmap_ion
 * Address: 0001b048
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void do_mmap_ion(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: do_munmap_ion
 * Address: 0001b04c
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void do_munmap_ion(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: v4l2_test_app_jpeg_fwrite
 * Address: 0001b050
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void v4l2_test_app_jpeg_fwrite(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: v4l2_test_app_jpeg_fclose
 * Address: 0001b054
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void v4l2_test_app_jpeg_fclose(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: cam_get_raw_images
 * Address: 0001b058
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void cam_get_raw_images(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: cam_get_snapshot_images
 * Address: 0001b05c
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void cam_get_snapshot_images(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: cam_get_thumbnail_images
 * Address: 0001b060
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void cam_get_thumbnail_images(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: jpeg_encoder_hw_inline_config
 * Address: 0001b064
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void jpeg_encoder_hw_inline_config(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: jpeg_encoder_hw_inline_wait_done
 * Address: 0001b068
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void jpeg_encoder_hw_inline_wait_done(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: use_overlay_fb_display_driver
 * Address: 0001b06c
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void use_overlay_fb_display_driver(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: launch_camframe_fb_thread
 * Address: 0001b074
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void launch_camframe_fb_thread(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: release_camframe_fb_thread
 * Address: 0001b078
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void release_camframe_fb_thread(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: mmap
 * Address: 0001b07c
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * mmap(void *__addr,size_t __len,int __prot,int __flags,int __fd,__off_t __offset)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: munmap
 * Address: 0001b080
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int munmap(void *__addr,size_t __len)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: dlopen
 * Address: 0001b084
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void dlopen(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: dlsym
 * Address: 0001b088
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void dlsym(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: dlclose
 * Address: 0001b08c
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void dlclose(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: perror
 * Address: 0001b090
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void perror(char *__s)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: exit
 * Address: 0001b094
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void exit(int __status)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: fputs
 * Address: 0001b0a0
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int fputs(char *__s,FILE *__stream)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: fgets
 * Address: 0001b0a4
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * fgets(char *__s,int __n,FILE *__stream)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: getopt
 * Address: 0001b0ac
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int getopt(int ___argc,char **___argv,char *__shortopts)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: __srget
 * Address: 0001b0b0
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */

void __srget(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: getc
 * Address: 0001b0b4
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int getc(FILE *__stream)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: printf
 * Address: 0001b0b8
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int printf(char *__format,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: putchar
 * Address: 0001b0bc
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int putchar(int __c)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



/* ========================================
 * Function: puts
 * Address: 0001b0c0
 * ======================================== */

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int puts(char *__s)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



