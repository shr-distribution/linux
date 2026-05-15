// Function: memcpy @ 0001cd70

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * memcpy(void *__dest,void *__src,size_t __n)

{
  void *pvVar1;
  
  pvVar1 = (void *)(*(code *)PTR_memcpy_00037a3c)(__dest);
  return pvVar1;
}



// Function: CameraParameters @ 0001cd7c

void __thiscall android::CameraParameters::CameraParameters(CameraParameters *this)

{
  (*(code *)PTR_CameraParameters_00037a40)(this);
  return;
}



// Function: __aeabi_atexit @ 0001cd88

void __aeabi_atexit(void)

{
  (*(code *)PTR___aeabi_atexit_00037a44)();
  return;
}



// Function: String8 @ 0001cd94

void __thiscall android::String8::String8(String8 *this)

{
  (*(code *)PTR_String8_00037a48)(this);
  return;
}



// Function: __android_log_print @ 0001cda0

void __android_log_print(void)

{
  (*(code *)PTR___android_log_print_00037a4c)();
  return;
}



// Function: operator= @ 0001cdac

void __thiscall
android::SortedVectorImpl::operator=(SortedVectorImpl *this,SortedVectorImpl *param_1)

{
  (*(code *)PTR_operator__00037a50)(this);
  return;
}



// Function: setTo @ 0001cdb8

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::String8::setTo(String8 *param_1)

{
  (*(code *)PTR_setTo_00037a54)(param_1);
  return;
}



// Function: ~CameraParameters @ 0001cdc4

void __thiscall android::CameraParameters::~CameraParameters(CameraParameters *this)

{
  (*(code *)PTR__CameraParameters_00037a58)(this);
  return;
}



// Function: flatten @ 0001cdd0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::flatten(void)

{
  (*(code *)PTR_flatten_00037a5c)();
  return;
}



// Function: ~String8 @ 0001cddc

void __thiscall android::String8::~String8(String8 *this)

{
  (*(code *)PTR__String8_00037a60)(this);
  return;
}



// Function: String8 @ 0001cde8

void __thiscall android::String8::String8(String8 *this,char *param_1)

{
  (*(code *)PTR_String8_00037a64)(this);
  return;
}



// Function: unflatten @ 0001cdf4

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::unflatten(String8 *param_1)

{
  (*(code *)PTR_unflatten_00037a68)(param_1);
  return;
}



// Function: free @ 0001ce00

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void free(void *__ptr)

{
  (*(code *)PTR_free_00037a6c)(__ptr);
  return;
}



// Function: atoi @ 0001ce0c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int atoi(char *__nptr)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_atoi_00037a70)(__nptr);
  return iVar1;
}



// Function: strcmp @ 0001ce18

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int strcmp(char *__s1,char *__s2)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_strcmp_00037a74)(__s1);
  return iVar1;
}



// Function: malloc @ 0001ce24

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * malloc(size_t __size)

{
  void *pvVar1;
  
  pvVar1 = (void *)(*(code *)PTR_malloc_00037a78)(__size);
  return pvVar1;
}



// Function: memset @ 0001ce30

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * memset(void *__s,int __c,size_t __n)

{
  void *pvVar1;
  
  pvVar1 = (void *)(*(code *)PTR_memset_00037a7c)(__s);
  return pvVar1;
}



// Function: __aeabi_idiv @ 0001ce3c

void __aeabi_idiv(void)

{
  (*(code *)PTR___aeabi_idiv_00037a80)();
  return;
}



// Function: decWeak @ 0001ce48

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::weakref_type::decWeak(void *param_1)

{
  (*(code *)PTR_decWeak_00037a84)(param_1);
  return;
}



// Function: pthread_cond_destroy @ 0001ce54

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_cond_destroy(pthread_cond_t *__cond)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_cond_destroy_00037a88)(__cond);
  return iVar1;
}



// Function: pthread_mutex_destroy @ 0001ce60

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_destroy(pthread_mutex_t *__mutex)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_mutex_destroy_00037a8c)(__mutex);
  return iVar1;
}



// Function: property_get @ 0001ce6c

void property_get(void)

{
  (*(code *)PTR_property_get_00037a90)();
  return;
}



// Function: __aeabi_uidivmod @ 0001ce78

void __aeabi_uidivmod(void)

{
  (*(code *)PTR___aeabi_uidivmod_00037a94)();
  return;
}



// Function: __stack_chk_fail @ 0001ce84

void __stack_chk_fail(void)

{
  (*(code *)PTR___stack_chk_fail_00037a98)();
  return;
}



// Function: dlopen @ 0001ce90

void dlopen(void)

{
  (*(code *)PTR_dlopen_00037a9c)();
  return;
}



// Function: dlerror @ 0001ce9c

void dlerror(void)

{
  (*(code *)PTR_dlerror_00037aa0)();
  return;
}



// Function: dlsym @ 0001cea8

void dlsym(void)

{
  (*(code *)PTR_dlsym_00037aa4)();
  return;
}



// Function: dlclose @ 0001ceb4

void dlclose(void)

{
  (*(code *)PTR_dlclose_00037aa8)();
  return;
}



// Function: pthread_mutex_lock @ 0001cec0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_lock(pthread_mutex_t *__mutex)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_mutex_lock_00037aac)(__mutex);
  return iVar1;
}



// Function: pthread_mutex_unlock @ 0001cecc

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_unlock(pthread_mutex_t *__mutex)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_mutex_unlock_00037ab0)(__mutex);
  return iVar1;
}



// Function: pthread_cond_signal @ 0001ced8

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_cond_signal(pthread_cond_t *__cond)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_cond_signal_00037ab4)(__cond);
  return iVar1;
}



// Function: incStrong @ 0001cee4

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::incStrong(void *param_1)

{
  (*(code *)PTR_incStrong_00037ab8)(param_1);
  return;
}



// Function: decStrong @ 0001cef0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::decStrong(void *param_1)

{
  (*(code *)PTR_decStrong_00037abc)(param_1);
  return;
}



// Function: __errno @ 0001cefc

void __errno(void)

{
  (*(code *)PTR___errno_00037ac0)();
  return;
}



// Function: strerror @ 0001cf08

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * strerror(int __errnum)

{
  char *pcVar1;
  
  pcVar1 = (char *)(*(code *)PTR_strerror_00037ac4)(__errnum);
  return pcVar1;
}



// Function: __aeabi_idivmod @ 0001cf14

void __aeabi_idivmod(void)

{
  (*(code *)PTR___aeabi_idivmod_00037ac8)();
  return;
}



// Function: append @ 0001cf20

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::String8::append(char *param_1)

{
  (*(code *)PTR_append_00037acc)(param_1);
  return;
}



// Function: snprintf @ 0001cf2c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int snprintf(char *__s,size_t __maxlen,char *__format,...)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_snprintf_00037ad0)(__s);
  return iVar1;
}



// Function: getDevice @ 0001cf38

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::MemoryHeapBase::getDevice(void)

{
  (*(code *)PTR_getDevice_00037ad4)();
  return;
}



// Function: write @ 0001cf44

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

ssize_t write(int __fd,void *__buf,size_t __n)

{
  ssize_t sVar1;
  
  sVar1 = (*(code *)PTR_write_00037ad8)(__fd);
  return sVar1;
}



// Function: operator.delete[] @ 0001cf50

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void operator_delete__(void *param_1)

{
  (*(code *)PTR_operator_delete___00037adc)(param_1);
  return;
}



// Function: ~RefBase @ 0001cf5c

void __thiscall android::RefBase::~RefBase(RefBase *this)

{
  (*(code *)PTR__RefBase_00037ae0)(this);
  return;
}



// Function: operator.delete @ 0001cf68

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void operator_delete(void *param_1)

{
  (*(code *)PTR_operator_delete_00037ae4)(param_1);
  return;
}



// Function: operator.new[] @ 0001cf74

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * operator_new__(uint param_1)

{
  void *pvVar1;
  
  pvVar1 = (void *)(*(code *)PTR_operator_new___00037ae8)(param_1);
  return pvVar1;
}



// Function: operator.new @ 0001cf80

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * operator_new(uint param_1)

{
  void *pvVar1;
  
  pvVar1 = (void *)(*(code *)PTR_operator_new_00037aec)(param_1);
  return pvVar1;
}



// Function: MemoryBase @ 0001cf8c

void __thiscall
android::MemoryBase::MemoryBase(MemoryBase *this,sp *param_1,long param_2,uint param_3)

{
  (*(code *)PTR_MemoryBase_00037af0)(this);
  return;
}



// Function: RefBase @ 0001cf98

void __thiscall android::RefBase::RefBase(RefBase *this)

{
  (*(code *)PTR_RefBase_00037af4)(this);
  return;
}



// Function: MemoryHeapBase @ 0001cfa4

void __thiscall
android::MemoryHeapBase::MemoryHeapBase
          (MemoryHeapBase *this,uint param_1,uint param_2,char *param_3)

{
  (*(code *)PTR_MemoryHeapBase_00037af8)(this);
  return;
}



// Function: attemptIncStrong @ 0001cfb0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::weakref_type::attemptIncStrong(void *param_1)

{
  (*(code *)PTR_attemptIncStrong_00037afc)(param_1);
  return;
}



// Function: createWeak @ 0001cfbc

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::createWeak(void *param_1)

{
  (*(code *)PTR_createWeak_00037b00)(param_1);
  return;
}



// Function: MemoryHeapBase @ 0001cfc8

void __thiscall
android::MemoryHeapBase::MemoryHeapBase
          (MemoryHeapBase *this,char *param_1,uint param_2,uint param_3)

{
  (*(code *)PTR_MemoryHeapBase_00037b04)(this);
  return;
}



// Function: MemoryHeapPmem @ 0001cfd4

void __thiscall
android::MemoryHeapPmem::MemoryHeapPmem(MemoryHeapPmem *this,sp *param_1,uint param_2)

{
  (*(code *)PTR_MemoryHeapPmem_00037b08)(this);
  return;
}



// Function: ioctl @ 0001cfe0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int ioctl(int __fd,ulong __request,...)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_ioctl_00037b0c)(__fd);
  return iVar1;
}



// Function: get @ 0001cfec

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::get(char *param_1)

{
  (*(code *)PTR_get_00037b10)(param_1);
  return;
}



// Function: set @ 0001cff8

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::set(char *param_1,char *param_2)

{
  (*(code *)PTR_set_00037b14)(param_1);
  return;
}



// Function: getInt @ 0001d004

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getInt(char *param_1)

{
  (*(code *)PTR_getInt_00037b18)(param_1);
  return;
}



// Function: set @ 0001d010

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::set(char *param_1,int param_2)

{
  (*(code *)PTR_set_00037b1c)(param_1);
  return;
}



// Function: pthread_cond_wait @ 0001d01c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_cond_wait(pthread_cond_t *__cond,pthread_mutex_t *__mutex)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_cond_wait_00037b20)(__cond);
  return iVar1;
}



// Function: strncmp @ 0001d028

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int strncmp(char *__s1,char *__s2,size_t __n)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_strncmp_00037b24)(__s1);
  return iVar1;
}



// Function: remove @ 0001d034

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::remove(char *param_1)

{
  (*(code *)PTR_remove_00037b28)(param_1);
  return;
}



// Function: getPreviewSize @ 0001d040

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewSize(int *param_1,int *param_2)

{
  (*(code *)PTR_getPreviewSize_00037b2c)(param_1);
  return;
}



// Function: getMeteringAreaCenter @ 0001d04c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getMeteringAreaCenter(int *param_1,int *param_2)

{
  (*(code *)PTR_getMeteringAreaCenter_00037b30)(param_1);
  return;
}



// Function: setTouchIndexAec @ 0001d058

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setTouchIndexAec(int param_1,int param_2)

{
  (*(code *)PTR_setTouchIndexAec_00037b34)(param_1);
  return;
}



// Function: setTouchIndexAf @ 0001d064

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setTouchIndexAf(int param_1,int param_2)

{
  (*(code *)PTR_setTouchIndexAf_00037b38)(param_1);
  return;
}



// Function: pthread_attr_init @ 0001d070

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_attr_init(pthread_attr_t *__attr)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_attr_init_00037b3c)(__attr);
  return iVar1;
}



// Function: pthread_attr_setdetachstate @ 0001d07c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_attr_setdetachstate(pthread_attr_t *__attr,int __detachstate)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_attr_setdetachstate_00037b40)(__attr);
  return iVar1;
}



// Function: pthread_create @ 0001d088

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_create(pthread_t *__newthread,pthread_attr_t *__attr,__start_routine *__start_routine,
                  void *__arg)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_create_00037b44)(__newthread);
  return iVar1;
}



// Function: getPreviewFormat @ 0001d094

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewFormat(void)

{
  (*(code *)PTR_getPreviewFormat_00037b48)();
  return;
}



// Function: getPictureSize @ 0001d0a0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPictureSize(int *param_1,int *param_2)

{
  (*(code *)PTR_getPictureSize_00037b4c)(param_1);
  return;
}



// Function: setPictureSize @ 0001d0ac

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPictureSize(int param_1,int param_2)

{
  (*(code *)PTR_setPictureSize_00037b50)(param_1);
  return;
}



// Function: getPreviewFrameRateMode @ 0001d0b8

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewFrameRateMode(void)

{
  (*(code *)PTR_getPreviewFrameRateMode_00037b54)();
  return;
}



// Function: getPreviewFrameRate @ 0001d0c4

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewFrameRate(void)

{
  (*(code *)PTR_getPreviewFrameRate_00037b58)();
  return;
}



// Function: setPreviewFrameRateMode @ 0001d0d0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewFrameRateMode(char *param_1)

{
  (*(code *)PTR_setPreviewFrameRateMode_00037b5c)(param_1);
  return;
}



// Function: setPreviewFrameRate @ 0001d0dc

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewFrameRate(int param_1)

{
  (*(code *)PTR_setPreviewFrameRate_00037b60)(param_1);
  return;
}



// Function: getPreviewFpsRange @ 0001d0e8

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewFpsRange(int *param_1,int *param_2)

{
  (*(code *)PTR_getPreviewFpsRange_00037b64)(param_1);
  return;
}



// Function: setPreviewFpsRange @ 0001d0f4

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewFpsRange(int param_1,int param_2)

{
  (*(code *)PTR_setPreviewFpsRange_00037b68)(param_1);
  return;
}



// Function: setPreviewSize @ 0001d100

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewSize(int param_1,int param_2)

{
  (*(code *)PTR_setPreviewSize_00037b6c)(param_1);
  return;
}



// Function: strtol @ 0001d10c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

long strtol(char *__nptr,char **__endptr,int __base)

{
  long lVar1;
  
  lVar1 = (*(code *)PTR_strtol_00037b70)(__nptr);
  return lVar1;
}



// Function: strcpy @ 0001d118

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * strcpy(char *__dest,char *__src)

{
  char *pcVar1;
  
  pcVar1 = (char *)(*(code *)PTR_strcpy_00037b74)(__dest);
  return pcVar1;
}



// Function: log @ 0001d124

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

double log(double __x)

{
  undefined4 uVar1;
  undefined4 extraout_s1;
  
  uVar1 = (*(code *)PTR_log_00037b78)(SUB84(__x,0));
  return (double)CONCAT44(extraout_s1,uVar1);
}



// Function: setFloat @ 0001d130

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setFloat(char *param_1,float param_2)

{
  (*(code *)PTR_setFloat_00037b7c)(param_1);
  return;
}



// Function: strlen @ 0001d13c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

size_t strlen(char *__s)

{
  size_t sVar1;
  
  sVar1 = (*(code *)PTR_strlen_00037b80)(__s);
  return sVar1;
}



// Function: time @ 0001d148

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

time_t time(time_t *__timer)

{
  time_t tVar1;
  
  tVar1 = (*(code *)PTR_time_00037b84)(__timer);
  return tVar1;
}



// Function: localtime @ 0001d154

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

tm * localtime(time_t *__timer)

{
  tm *ptVar1;
  
  ptVar1 = (tm *)(*(code *)PTR_localtime_00037b88)(__timer);
  return ptVar1;
}



// Function: strftime @ 0001d160

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

size_t strftime(char *__s,size_t __maxsize,char *__format,tm *__tp)

{
  size_t sVar1;
  
  sVar1 = (*(code *)PTR_strftime_00037b8c)(__s);
  return sVar1;
}



// Function: pow @ 0001d16c

/* WARNING: Heritage AFTER dead removal. Example location: s1 : 0x0001d174 */
/* WARNING: Restarted to delay deadcode elimination for space: register */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

double pow(double __x,double __y)

{
  (*(code *)PTR_pow_00037b90)(SUB84(__x,0),(int)((ulonglong)__x >> 0x20),SUB84(__y,0));
  return __x;
}



// Function: getFloat @ 0001d178

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getFloat(char *param_1)

{
  (*(code *)PTR_getFloat_00037b94)(param_1);
  return;
}



// Function: strtod @ 0001d184

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

double strtod(char *__nptr,char **__endptr)

{
  undefined4 uVar1;
  undefined4 extraout_s1;
  
  uVar1 = (*(code *)PTR_strtod_00037b98)(__nptr);
  return (double)CONCAT44(extraout_s1,uVar1);
}



// Function: lround @ 0001d190

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

long lround(double __x)

{
  long lVar1;
  
  lVar1 = (*(code *)PTR_lround_00037b9c)(SUB84(__x,0));
  return lVar1;
}



// Function: atol @ 0001d19c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

long atol(char *__nptr)

{
  long lVar1;
  
  lVar1 = (*(code *)PTR_atol_00037ba0)(__nptr);
  return lVar1;
}



// Function: gmtime_r @ 0001d1a8

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

tm * gmtime_r(time_t *__timer,tm *__tp)

{
  tm *ptVar1;
  
  ptVar1 = (tm *)(*(code *)PTR_gmtime_r_00037ba4)(__timer);
  return ptVar1;
}



// Function: strncpy @ 0001d1b4

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * strncpy(char *__dest,char *__src,size_t __n)

{
  char *pcVar1;
  
  pcVar1 = (char *)(*(code *)PTR_strncpy_00037ba8)(__dest);
  return pcVar1;
}



// Function: JpegEncoder @ 0001d1c0

void __thiscall android::JpegEncoder::JpegEncoder(JpegEncoder *this)

{
  (*(code *)PTR_JpegEncoder_00037bac)(this);
  return;
}



// Function: makeExif @ 0001d1cc

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::JpegEncoder::makeExif
               (uchar *param_1,uchar *param_2,exif_attribute_t *param_3,uint *param_4,long param_5,
               bool param_6)

{
  (*(code *)PTR_makeExif_00037bb0)(param_1);
  return;
}



// Function: ~JpegEncoder @ 0001d1d8

void __thiscall android::JpegEncoder::~JpegEncoder(JpegEncoder *this)

{
  (*(code *)PTR__JpegEncoder_00037bb4)(this);
  return;
}



// Function: genlock_unlock_buffer @ 0001d1e4

void genlock_unlock_buffer(void)

{
  (*(code *)PTR_genlock_unlock_buffer_00037bb8)();
  return;
}



// Function: native_handle_delete @ 0001d1f0

void native_handle_delete(void)

{
  (*(code *)PTR_native_handle_delete_00037bbc)();
  return;
}



// Function: open @ 0001d1fc

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int open(char *__file,int __oflag,...)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_open_00037bc0)(__file);
  return iVar1;
}



// Function: mmap @ 0001d208

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * mmap(void *__addr,size_t __len,int __prot,int __flags,int __fd,__off_t __offset)

{
  void *pvVar1;
  
  pvVar1 = (void *)(*(code *)PTR_mmap_00037bc4)(__addr);
  return pvVar1;
}



// Function: close @ 0001d214

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int close(int __fd)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_close_00037bc8)(__fd);
  return iVar1;
}



// Function: systemTime @ 0001d220

void systemTime(void)

{
  (*(code *)PTR_systemTime_00037bcc)();
  return;
}



// Function: __aeabi_l2f @ 0001d22c

void __aeabi_l2f(void)

{
  (*(code *)PTR___aeabi_l2f_00037bd0)();
  return;
}



// Function: String8 @ 0001d238

void __thiscall android::String8::String8(String8 *this,String8 *param_1)

{
  (*(code *)PTR_String8_00037bd4)(this);
  return;
}



// Function: finish_vector @ 0001d244

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::finish_vector(void)

{
  (*(code *)PTR_finish_vector_00037bd8)();
  return;
}



// Function: ~SortedVectorImpl @ 0001d250

void __thiscall android::SortedVectorImpl::~SortedVectorImpl(SortedVectorImpl *this)

{
  (*(code *)PTR__SortedVectorImpl_00037bdc)(this);
  return;
}



// Function: VectorImpl @ 0001d25c

void __thiscall android::VectorImpl::VectorImpl(VectorImpl *this,VectorImpl *param_1)

{
  (*(code *)PTR_VectorImpl_00037be0)(this);
  return;
}



// Function: pthread_mutex_trylock @ 0001d268

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_trylock(pthread_mutex_t *__mutex)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_mutex_trylock_00037be4)(__mutex);
  return iVar1;
}



// Function: usleep @ 0001d274

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int usleep(__useconds_t __useconds)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_usleep_00037be8)(__useconds);
  return iVar1;
}



// Function: munmap @ 0001d280

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int munmap(void *__addr,size_t __len)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_munmap_00037bec)(__addr);
  return iVar1;
}



// Function: fopen @ 0001d28c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

FILE * fopen(char *__filename,char *__modes)

{
  FILE *pFVar1;
  
  pFVar1 = (FILE *)(*(code *)PTR_fopen_00037bf0)(__filename);
  return pFVar1;
}



// Function: fscanf @ 0001d298

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int fscanf(FILE *__stream,char *__format,...)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_fscanf_00037bf4)(__stream);
  return iVar1;
}



// Function: fclose @ 0001d2a4

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int fclose(FILE *__stream)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_fclose_00037bf8)(__stream);
  return iVar1;
}



// Function: genlock_lock_buffer @ 0001d2b0

void genlock_lock_buffer(void)

{
  (*(code *)PTR_genlock_lock_buffer_00037bfc)();
  return;
}



// Function: pthread_join @ 0001d2bc

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_join(pthread_t __th,void **__thread_return)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_join_00037c00)(__th);
  return iVar1;
}



// Function: sprintf @ 0001d2c8

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int sprintf(char *__s,char *__format,...)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_sprintf_00037c04)(__s);
  return iVar1;
}



// Function: clear @ 0001d2d4

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::clear(void)

{
  (*(code *)PTR_clear_00037c08)();
  return;
}



// Function: removeItemsAt @ 0001d2e0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::removeItemsAt(uint param_1,uint param_2)

{
  (*(code *)PTR_removeItemsAt_00037c0c)(param_1);
  return;
}



// Function: add @ 0001d2ec

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::add(void *param_1)

{
  (*(code *)PTR_add_00037c10)(param_1);
  return;
}



// Function: ~VectorImpl @ 0001d2f8

void __thiscall android::VectorImpl::~VectorImpl(VectorImpl *this)

{
  (*(code *)PTR__VectorImpl_00037c14)(this);
  return;
}



// Function: memmove @ 0001d304

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * memmove(void *__dest,void *__src,size_t __n)

{
  void *pvVar1;
  
  pvVar1 = (void *)(*(code *)PTR_memmove_00037c18)(__dest);
  return pvVar1;
}



// Function: pthread_mutex_init @ 0001d310

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_init(pthread_mutex_t *__mutex,pthread_mutexattr_t *__mutexattr)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_mutex_init_00037c1c)(__mutex);
  return iVar1;
}



// Function: pthread_cond_init @ 0001d31c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_cond_init(pthread_cond_t *__cond,pthread_condattr_t *__cond_attr)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_pthread_cond_init_00037c20)(__cond);
  return iVar1;
}



// Function: VectorImpl @ 0001d328

void __thiscall android::VectorImpl::VectorImpl(VectorImpl *this,uint param_1,uint param_2)

{
  (*(code *)PTR_VectorImpl_00037c24)(this);
  return;
}



// Function: gmtime @ 0001d334

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

tm * gmtime(time_t *__timer)

{
  tm *ptVar1;
  
  ptVar1 = (tm *)(*(code *)PTR_gmtime_00037c28)(__timer);
  return ptVar1;
}



// Function: sscanf @ 0001d340

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int sscanf(char *__s,char *__format,...)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_sscanf_00037c2c)(__s);
  return iVar1;
}



// Function: getPictureFormat @ 0001d34c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPictureFormat(void)

{
  (*(code *)PTR_getPictureFormat_00037c30)();
  return;
}



// Function: native_handle_create @ 0001d358

void native_handle_create(void)

{
  (*(code *)PTR_native_handle_create_00037c34)();
  return;
}



// Function: setPreviewFormat @ 0001d364

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewFormat(char *param_1)

{
  (*(code *)PTR_setPreviewFormat_00037c38)(param_1);
  return;
}



// Function: setPictureFormat @ 0001d370

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPictureFormat(char *param_1)

{
  (*(code *)PTR_setPictureFormat_00037c3c)(param_1);
  return;
}



// Function: stat @ 0001d37c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int stat(char *__file,stat *__buf)

{
  int iVar1;
  
  iVar1 = (*(code *)PTR_stat_00037c40)(__file);
  return iVar1;
}



// Function: onFirstRef @ 0001d388

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::onFirstRef(void)

{
  (*(code *)PTR_onFirstRef_00037c44)();
  return;
}



// Function: onLastStrongRef @ 0001d394

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::onLastStrongRef(void *param_1)

{
  (*(code *)PTR_onLastStrongRef_00037c48)(param_1);
  return;
}



// Function: onIncStrongAttempted @ 0001d3a0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::onIncStrongAttempted(uint param_1,void *param_2)

{
  (*(code *)PTR_onIncStrongAttempted_00037c4c)(param_1);
  return;
}



// Function: onLastWeakRef @ 0001d3ac

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::onLastWeakRef(void *param_1)

{
  (*(code *)PTR_onLastWeakRef_00037c50)(param_1);
  return;
}



// Function: reservedVectorImpl1 @ 0001d3b8

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl1(void)

{
  (*(code *)PTR_reservedVectorImpl1_00037c54)();
  return;
}



// Function: reservedVectorImpl2 @ 0001d3c4

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl2(void)

{
  (*(code *)PTR_reservedVectorImpl2_00037c58)();
  return;
}



// Function: reservedVectorImpl3 @ 0001d3d0

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl3(void)

{
  (*(code *)PTR_reservedVectorImpl3_00037c5c)();
  return;
}



// Function: reservedVectorImpl4 @ 0001d3dc

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl4(void)

{
  (*(code *)PTR_reservedVectorImpl4_00037c60)();
  return;
}



// Function: reservedVectorImpl5 @ 0001d3e8

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl5(void)

{
  (*(code *)PTR_reservedVectorImpl5_00037c64)();
  return;
}



// Function: reservedVectorImpl6 @ 0001d3f4

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl6(void)

{
  (*(code *)PTR_reservedVectorImpl6_00037c68)();
  return;
}



// Function: reservedVectorImpl7 @ 0001d400

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl7(void)

{
  (*(code *)PTR_reservedVectorImpl7_00037c6c)();
  return;
}



// Function: reservedVectorImpl8 @ 0001d40c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl8(void)

{
  (*(code *)PTR_reservedVectorImpl8_00037c70)();
  return;
}



// Function: reservedSortedVectorImpl1 @ 0001d418

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl1(void)

{
  (*(code *)PTR_reservedSortedVectorImpl1_00037c74)();
  return;
}



// Function: reservedSortedVectorImpl2 @ 0001d424

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl2(void)

{
  (*(code *)PTR_reservedSortedVectorImpl2_00037c78)();
  return;
}



// Function: reservedSortedVectorImpl3 @ 0001d430

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl3(void)

{
  (*(code *)PTR_reservedSortedVectorImpl3_00037c7c)();
  return;
}



// Function: reservedSortedVectorImpl4 @ 0001d43c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl4(void)

{
  (*(code *)PTR_reservedSortedVectorImpl4_00037c80)();
  return;
}



// Function: reservedSortedVectorImpl5 @ 0001d448

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl5(void)

{
  (*(code *)PTR_reservedSortedVectorImpl5_00037c84)();
  return;
}



// Function: reservedSortedVectorImpl6 @ 0001d454

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl6(void)

{
  (*(code *)PTR_reservedSortedVectorImpl6_00037c88)();
  return;
}



// Function: reservedSortedVectorImpl7 @ 0001d460

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl7(void)

{
  (*(code *)PTR_reservedSortedVectorImpl7_00037c8c)();
  return;
}



// Function: reservedSortedVectorImpl8 @ 0001d46c

/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl8(void)

{
  (*(code *)PTR_reservedSortedVectorImpl8_00037c90)();
  return;
}



// Function: __cxa_finalize @ 0001d478

void __cxa_finalize(void)

{
  (*(code *)PTR___cxa_finalize_00037c94)();
  return;
}



// Function: util_get_Hal_obj @ 0001d498

/* android::util_get_Hal_obj(camera_device*) */

undefined4 android::util_get_Hal_obj(camera_device *param_1)

{
  undefined4 uVar1;
  
  if ((param_1 == (camera_device *)0x0) || (*(undefined4 **)(param_1 + 0x44) == (undefined4 *)0x0))
  {
    uVar1 = 0;
  }
  else {
    uVar1 = **(undefined4 **)(param_1 + 0x44);
  }
  return uVar1;
}



// Function: util_get_HAL_parameter @ 0001d4a8

/* android::util_get_HAL_parameter(camera_device*) */

int android::util_get_HAL_parameter(camera_device *param_1)

{
  int iVar1;
  
  if ((param_1 == (camera_device *)0x0) || (*(int *)(param_1 + 0x44) == 0)) {
    iVar1 = 0;
  }
  else {
    iVar1 = *(int *)(param_1 + 0x44) + 8;
  }
  return iVar1;
}



// Function: enable_msg_type @ 0001d4b8

void enable_msg_type(camera_device *param_1,undefined4 param_2)

{
  int *piVar1;
  
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 != (int *)0x0) {
    (**(code **)(*piVar1 + 0x18))(piVar1,param_2);
  }
  return;
}



// Function: _INIT_0 @ 0001d4cc

void _INIT_0(void)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  CameraParameters *this;
  String8 *this_00;
  
  iVar1 = DAT_0001d524;
  iVar2 = DAT_0001d518 + 0x1d4da;
  memcpy(*(void **)(iVar2 + DAT_0001d51c),(void *)(DAT_0001d520 + 0x1d4e4),0x80);
  this = *(CameraParameters **)(iVar2 + DAT_0001d528);
  android::CameraParameters::CameraParameters(this);
  uVar3 = *(undefined4 *)(iVar2 + DAT_0001d52c);
  __aeabi_atexit(this,*(undefined4 *)(iVar2 + iVar1),uVar3);
  this_00 = *(String8 **)(iVar2 + DAT_0001d530);
  android::String8::String8(this_00);
  __aeabi_atexit(this_00,*(undefined4 *)(iVar2 + DAT_0001d534),uVar3);
  return;
}



// Function: dump @ 0001d538

undefined4 dump(camera_device *param_1)

{
  int iVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d560 + 0x1d542,DAT_0001d564 + 0x1d546,DAT_0001d568 + 0x1d54a);
  iVar1 = android::util_get_Hal_obj(param_1);
  if (iVar1 == 0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = 0;
  }
  return uVar2;
}



// Function: release @ 0001d56c

void release(camera_device *param_1)

{
  int *piVar1;
  int iVar2;
  
  __android_log_print(6,DAT_0001d59c + 0x1d576,DAT_0001d5a0 + 0x1d57a,DAT_0001d5a4 + 0x1d57e);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 != (int *)0x0) {
    iVar2 = *(int *)(param_1 + 0x44);
    (**(code **)(*piVar1 + 0x84))();
    *(undefined4 *)(iVar2 + 4) = 1;
  }
  return;
}



// Function: send_command @ 0001d5a8

undefined4
send_command(camera_device *param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d5e4 + 0x1d5c0,DAT_0001d5e8 + 0x1d5c2,DAT_0001d5ec + 0x1d5c4);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 100))(piVar1,param_2,param_3,param_4);
  }
  return uVar2;
}



// Function: put_parameters @ 0001d5f0

void put_parameters(camera_device *param_1)

{
  int iVar1;
  
  iVar1 = DAT_0001d61c + 0x1d5fe;
  __android_log_print(6,iVar1,DAT_0001d620 + 0x1d602,DAT_0001d624 + 0x1d604);
  android::util_get_Hal_obj(param_1);
  __android_log_print(6,iVar1,DAT_0001d628 + 0x1d618);
  return;
}



// Function: cancel_picture @ 0001d62c

undefined4 cancel_picture(camera_device *param_1)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d658 + 0x1d636,DAT_0001d65c + 0x1d63a,DAT_0001d660 + 0x1d63e);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 0x58))();
  }
  return uVar2;
}



// Function: take_picture @ 0001d664

undefined4 take_picture(camera_device *param_1)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d690 + 0x1d66e,DAT_0001d694 + 0x1d672,DAT_0001d698 + 0x1d676);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 0x4c))();
  }
  return uVar2;
}



// Function: cancel_auto_focus @ 0001d69c

undefined4 cancel_auto_focus(camera_device *param_1)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d6c8 + 0x1d6a6,DAT_0001d6cc + 0x1d6aa,DAT_0001d6d0 + 0x1d6ae);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 0x48))();
  }
  return uVar2;
}



// Function: auto_focus @ 0001d6d4

undefined4 auto_focus(camera_device *param_1)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d700 + 0x1d6de,DAT_0001d704 + 0x1d6e2,DAT_0001d708 + 0x1d6e6);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 0x44))();
  }
  return uVar2;
}



// Function: release_recording_frame @ 0001d70c

void release_recording_frame(camera_device *param_1,undefined4 param_2)

{
  int *piVar1;
  
  __android_log_print(6,DAT_0001d740 + 0x1d720,DAT_0001d738 + 0x1d71c,DAT_0001d73c + 0x1d71e);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 != (int *)0x0) {
    (**(code **)(*piVar1 + 0x40))(piVar1,param_2);
  }
  return;
}



// Function: recording_enabled @ 0001d744

undefined4 recording_enabled(camera_device *param_1)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d770 + 0x1d74e,DAT_0001d774 + 0x1d752,DAT_0001d778 + 0x1d756);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 0x3c))();
  }
  return uVar2;
}



// Function: stop_recording @ 0001d77c

void stop_recording(camera_device *param_1)

{
  int *piVar1;
  
  __android_log_print(6,DAT_0001d7a4 + 0x1d786,DAT_0001d7a8 + 0x1d78a,DAT_0001d7ac + 0x1d78e);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 != (int *)0x0) {
    (**(code **)(*piVar1 + 0x38))();
  }
  return;
}



// Function: start_recording @ 0001d7b0

undefined4 start_recording(camera_device *param_1)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d7dc + 0x1d7ba,DAT_0001d7e0 + 0x1d7be,DAT_0001d7e4 + 0x1d7c2);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 0x34))();
  }
  return uVar2;
}



// Function: preview_enabled @ 0001d7e8

undefined4 preview_enabled(camera_device *param_1)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d814 + 0x1d7f2,DAT_0001d818 + 0x1d7f6,DAT_0001d81c + 0x1d7fa);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 0x30))();
  }
  return uVar2;
}



// Function: stop_preview @ 0001d820

void stop_preview(camera_device *param_1)

{
  int *piVar1;
  
  __android_log_print(6,DAT_0001d848 + 0x1d82a,DAT_0001d84c + 0x1d82e,DAT_0001d850 + 0x1d832);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 != (int *)0x0) {
    (**(code **)(*piVar1 + 0x2c))();
  }
  return;
}



// Function: start_preview @ 0001d854

undefined4 start_preview(camera_device *param_1)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d894 + 0x1d85e,DAT_0001d898 + 0x1d862,DAT_0001d89c + 0x1d866);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  uVar2 = 0xffffffff;
  if (piVar1 != (int *)0x0) {
    uVar2 = (**(code **)(*piVar1 + 0x28))();
  }
  __android_log_print(6,DAT_0001d8a0 + 0x1d88a,DAT_0001d8a4 + 0x1d88c,DAT_0001d8a8 + 0x1d88e);
  return uVar2;
}



// Function: msg_type_enabled @ 0001d8ac

undefined4 msg_type_enabled(camera_device *param_1,undefined4 param_2)

{
  int *piVar1;
  undefined4 uVar2;
  
  __android_log_print(6,DAT_0001d8e4 + 0x1d8c0,DAT_0001d8dc + 0x1d8bc,DAT_0001d8e0 + 0x1d8be);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 0x20))(piVar1,param_2);
  }
  return uVar2;
}



// Function: disable_msg_type @ 0001d8e8

void disable_msg_type(camera_device *param_1,undefined4 param_2)

{
  int *piVar1;
  
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  __android_log_print(6,DAT_0001d914 + 0x1d8fa,DAT_0001d918 + 0x1d8fc,DAT_0001d91c + 0x1d8fe);
  if (piVar1 != (int *)0x0) {
    (**(code **)(*piVar1 + 0x1c))(piVar1,param_2);
  }
  return;
}



// Function: set_preview_window @ 0001d920

undefined8 set_preview_window(camera_device *param_1,undefined4 param_2,undefined4 param_3)

{
  int *piVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  
  uVar3 = param_2;
  __android_log_print(6,DAT_0001d958 + 0x1d934,DAT_0001d954 + 0x1d930,DAT_0001d950 + 0x1d932,param_2
                      ,param_2,param_3);
  piVar1 = (int *)android::util_get_Hal_obj(param_1);
  if (piVar1 == (int *)0x0) {
    uVar2 = 0xffffffff;
  }
  else {
    uVar2 = (**(code **)(*piVar1 + 0x78))(piVar1,param_2);
  }
  return CONCAT44(uVar3,uVar2);
}



// Function: close_Hal_obj @ 0001d95c

/* android::close_Hal_obj(camera_device*) */

void android::close_Hal_obj(camera_device *param_1)

{
  int iVar1;
  int iVar2;
  int *piVar3;
  
  iVar2 = DAT_0001d9a8 + 0x1d96a;
  iVar1 = DAT_0001d9ac + 0x1d96c;
  __android_log_print(4,iVar2,DAT_0001d9b0 + 0x1d970,iVar1);
  if ((param_1 != (camera_device *)0x0) &&
     (piVar3 = *(int **)(param_1 + 0x44), piVar3 != (int *)0x0)) {
    __android_log_print(4,iVar2,DAT_0001d9b4 + 0x1d988,iVar1);
    piVar3 = (int *)*piVar3;
    if (piVar3 != (int *)0x0) {
      (**(code **)(*piVar3 + 4))();
    }
  }
  __android_log_print(4,DAT_0001d9b8 + 0x1d9a0,DAT_0001d9bc + 0x1d9a2,DAT_0001d9c0 + 0x1d9a4);
  return;
}



// Function: get_mem @ 0001d9c4

/* android::get_mem(int, unsigned int, unsigned int, void*) */

undefined4 android::get_mem(int param_1,uint param_2,uint param_3,void *param_4)

{
  undefined4 uVar1;
  int iVar2;
  
  __android_log_print(6,DAT_0001da00 + 0x1d9dc,DAT_0001da04 + 0x1d9de,DAT_0001da08 + 0x1d9e0);
  if ((param_4 != (void *)0x0) && (iVar2 = *(int *)((int)param_4 + 0x44), iVar2 != 0)) {
    if (*(code **)(iVar2 + 0x2c) != (code *)0x0) {
      uVar1 = (**(code **)(iVar2 + 0x2c))(param_1,param_2,param_3,*(undefined4 *)(iVar2 + 0x30));
      return uVar1;
    }
  }
  return 0;
}



// Function: cam_notify_callback @ 0001da0c

/* android::cam_notify_callback(int, int, int, void*) */

void android::cam_notify_callback(int param_1,int param_2,int param_3,void *param_4)

{
  int iVar1;
  
  __android_log_print(6,DAT_0001da44 + 0x1da24,DAT_0001da48 + 0x1da26,DAT_0001da4c + 0x1da28);
  if ((param_4 != (void *)0x0) && (iVar1 = *(int *)((int)param_4 + 0x44), iVar1 != 0)) {
    if (*(code **)(iVar1 + 0x20) != (code *)0x0) {
      (**(code **)(iVar1 + 0x20))(param_1,param_2,param_3,*(undefined4 *)(iVar1 + 0x30));
    }
  }
  return;
}



// Function: get_parameters @ 0001da50

int get_parameters(camera_device *param_1)

{
  int iVar1;
  int *piVar2;
  int iVar3;
  int iVar4;
  String8 *pSVar5;
  SortedVectorImpl *this;
  CameraParameters aCStack_4c [24];
  CameraParameters aCStack_34 [24];
  String8 aSStack_1c [8];
  
  iVar3 = DAT_0001daf0 + 0x1da5e;
  __android_log_print(6,iVar3,DAT_0001daf4 + 0x1da64,DAT_0001daf8 + 0x1da66);
  android::CameraParameters::CameraParameters(aCStack_34);
  piVar2 = (int *)android::util_get_Hal_obj(param_1);
  iVar1 = DAT_0001dafc;
  iVar4 = 0;
  if (piVar2 != (int *)0x0) {
    (**(code **)(*piVar2 + 0x60))(aCStack_4c,piVar2);
    this = *(SortedVectorImpl **)(iVar1 + 0x1da8e);
    android::SortedVectorImpl::operator=(this,(SortedVectorImpl *)aCStack_4c);
    android::String8::setTo((String8 *)(this + 0x14));
    iVar4 = DAT_0001db00;
    android::CameraParameters::~CameraParameters(aCStack_4c);
    android::CameraParameters::flatten();
    pSVar5 = *(String8 **)(iVar4 + 0x1dab6);
    android::String8::setTo(pSVar5);
    android::String8::~String8(aSStack_1c);
    iVar4 = *(int *)pSVar5;
    if (iVar4 == 0) {
      __android_log_print(6,iVar3,DAT_0001db04 + 0x1dad2);
    }
  }
  __android_log_print(6,DAT_0001db08 + 0x1dade,DAT_0001db0c + 0x1dae0);
  android::CameraParameters::~CameraParameters(aCStack_34);
  return iVar4;
}



// Function: set_parameters @ 0001db10

undefined8 set_parameters(camera_device *param_1,char *param_2,undefined4 param_3)

{
  int iVar1;
  int iVar2;
  int *piVar3;
  undefined4 uVar4;
  String8 *pSVar5;
  char *pcStack_1c;
  undefined4 uStack_18;
  
  pcStack_1c = param_2;
  uStack_18 = param_3;
  __android_log_print(6,DAT_0001db88 + 0x1db24,DAT_0001db84 + 0x1db22,DAT_0001db80 + 0x1db20);
  piVar3 = (int *)android::util_get_Hal_obj(param_1);
  iVar1 = DAT_0001db8c;
  uVar4 = 0xffffffff;
  if (piVar3 != (int *)0x0 && param_2 != (char *)0x0) {
    android::String8::String8((String8 *)&pcStack_1c,param_2);
    iVar2 = DAT_0001db90;
    android::String8::setTo(*(String8 **)(iVar1 + 0x1db56));
    android::String8::~String8((String8 *)&pcStack_1c);
    pSVar5 = *(String8 **)(iVar2 + 0x1db6a);
    android::CameraParameters::unflatten(pSVar5);
    uVar4 = (**(code **)(*piVar3 + 0x5c))(piVar3,pSVar5);
  }
  return CONCAT44(param_1,uVar4);
}



// Function: store_meta_data_in_buffers @ 0001db94

undefined4 store_meta_data_in_buffers(camera_device *param_1,int param_2)

{
  QualcommCameraHardware *this;
  undefined4 uVar1;
  
  __android_log_print(6,DAT_0001dbc8 + 0x1dba8,DAT_0001dbc0 + 0x1dba4,DAT_0001dbc4 + 0x1dba6);
  this = (QualcommCameraHardware *)android::util_get_Hal_obj(param_1);
  if (this == (QualcommCameraHardware *)0x0) {
    uVar1 = 0xffffffff;
  }
  else {
    uVar1 = android::QualcommCameraHardware::storeMetaDataInBuffers(this,param_2);
  }
  return uVar1;
}



// Function: set_callbacks @ 0001dbcc

void set_callbacks(camera_device *param_1,_func_void_int_int_int_void_ptr *param_2,
                  _func_void_int_camera_memory_ptr_uint_camera_frame_metadata_ptr_void_ptr *param_3,
                  _func_void_longlong_int_camera_memory_ptr_uint_void_ptr *param_4,
                  _func_camera_memory_ptr_int_uint_uint_void_ptr *param_5,void *param_6)

{
  int iVar1;
  QualcommCameraHardware *this;
  int iVar2;
  int iVar3;
  _func_void_int_camera_memory_ptr_uint_camera_frame_metadata_ptr_void_ptr *p_Var4;
  
  iVar3 = DAT_0001dc40 + 0x1dbdc;
  p_Var4 = param_3;
  __android_log_print(6,iVar3,DAT_0001dc44 + 0x1dbe2,DAT_0001dc48 + 0x1dbe6,param_1,param_2,param_3)
  ;
  this = (QualcommCameraHardware *)android::util_get_Hal_obj(param_1);
  iVar1 = DAT_0001dc4c;
  if ((this != (QualcommCameraHardware *)0x0) && (iVar2 = *(int *)(param_1 + 0x44), iVar2 != 0)) {
    *(_func_void_int_int_int_void_ptr **)(iVar2 + 0x20) = param_2;
    *(_func_void_int_camera_memory_ptr_uint_camera_frame_metadata_ptr_void_ptr **)(iVar2 + 0x24) =
         param_3;
    *(_func_void_longlong_int_camera_memory_ptr_uint_void_ptr **)(iVar2 + 0x28) = param_4;
    *(void **)(iVar2 + 0x30) = param_6;
    *(_func_camera_memory_ptr_int_uint_uint_void_ptr **)(iVar2 + 0x2c) = param_5;
    __android_log_print(6,iVar3,iVar1 + 0x1dc12,0,0,0,p_Var4);
    android::QualcommCameraHardware::setCallbacks(this,param_2,param_3,param_4,param_5,param_6);
  }
  return;
}



// Function: close_camera_device @ 0001dc50

undefined8 close_camera_device(camera_device *param_1,undefined4 param_2,undefined4 param_3)

{
  undefined4 uVar1;
  int *piVar2;
  int iVar3;
  camera_device *pcVar4;
  
  pcVar4 = param_1;
  __android_log_print(6,DAT_0001dca8 + 0x1dc62,DAT_0001dcac + 0x1dc64,DAT_0001dca4 + 0x1dc5e,param_1
                      ,param_2,param_3);
  uVar1 = 0xffffffff;
  if (param_1 != (camera_device *)0x0) {
    iVar3 = *(int *)(param_1 + 0x44);
    if (iVar3 != 0) {
      piVar2 = (int *)android::util_get_Hal_obj(param_1);
      if ((piVar2 != (int *)0x0) && (*(int *)(iVar3 + 4) != 1)) {
        (**(code **)(*piVar2 + 0x84))();
      }
      android::close_Hal_obj(param_1);
      free(*(void **)(param_1 + 0x44));
      *(undefined4 *)(param_1 + 0x44) = 0;
    }
    free(param_1);
    uVar1 = 0;
  }
  return CONCAT44(pcVar4,uVar1);
}



// Function: camera_device_open @ 0001dcb0

undefined4 camera_device_open(int param_1,char *param_2,undefined4 *param_3)

{
  int iVar1;
  int iVar2;
  int *__s;
  undefined4 uVar3;
  undefined4 uVar4;
  void *__ptr;
  
  __android_log_print(6,DAT_0001dd84 + 0x1dcc6,DAT_0001dd88 + 0x1dcca,DAT_0001dd80 + 0x1dcc4);
  if (param_2 == (char *)0x0 || param_1 == 0) {
    if (param_3 == (undefined4 *)0x0) goto LAB_0001dd64;
    __ptr = (void *)0x0;
  }
  else {
    if (param_3 == (undefined4 *)0x0) {
LAB_0001dd64:
      __android_log_print(6,DAT_0001dd98 + 0x1dd70,DAT_0001dd9c + 0x1dd72,DAT_0001dda0 + 0x1dd74);
      return 0xffffffff;
    }
    iVar1 = atoi(param_2);
    iVar2 = strcmp(*(char **)(param_1 + 0xc),*(char **)(DAT_0001dd8c + 0x1dcfc));
    if (iVar2 == 0) {
      __ptr = malloc(0x48);
      if (__ptr != (void *)0x0) {
        __s = malloc(0x34);
        if (__s == (int *)0x0) {
          free(__ptr);
          __ptr = (void *)0x0;
        }
        else {
          memset(__s,0,0x34);
          iVar2 = HAL_openCameraHardware(iVar1);
          *__s = iVar2;
          iVar1 = DAT_0001dd94;
          if (iVar2 != 0) {
            uVar4 = *(undefined4 *)(DAT_0001dd90 + 0x1dd30);
            *(int **)((int)__ptr + 0x44) = __s;
            uVar3 = *(undefined4 *)(iVar1 + 0x1dd36);
            *(undefined4 *)((int)__ptr + 0x3c) = uVar4;
            *(undefined4 *)((int)__ptr + 0x40) = uVar3;
            uVar3 = 0;
            goto LAB_0001dd60;
          }
          free(__s);
          free(__ptr);
          __ptr = (void *)0x0;
        }
      }
    }
    else {
      __ptr = (void *)0x0;
    }
  }
  uVar3 = 0xffffffff;
LAB_0001dd60:
  *param_3 = __ptr;
  return uVar3;
}



// Function: get_camera_info @ 0001dda4

undefined4 get_camera_info(undefined4 param_1,int *param_2,int param_3,undefined4 param_4)

{
  int *local_1c;
  int local_18;
  undefined4 uStack_14;
  
  local_1c = param_2;
  local_18 = param_3;
  uStack_14 = param_4;
  __android_log_print(6,DAT_0001ddf0 + 0x1ddb8,DAT_0001ddec + 0x1ddb6,DAT_0001ddf4 + 0x1ddba,param_1
                     );
  if (param_2 != (int *)0x0) {
    memset(&local_1c,-1,0xc);
    HAL_getCameraInfo(param_1,&local_1c);
    if (-1 < (int)local_1c) {
      *param_2 = (int)local_1c;
      param_2[1] = local_18;
      return 0;
    }
  }
  return 0xffffffff;
}



// Function: get_number_of_cameras @ 0001ddf8

void get_number_of_cameras(void)

{
  __android_log_print(6,DAT_0001de14 + 0x1de04,DAT_0001de18 + 0x1de08,DAT_0001de1c + 0x1de0a);
  HAL_getNumberOfCameras();
  return;
}



// Function: FUN_0001de20 @ 0001de20

void FUN_0001de20(int *param_1)

{
  (**(code **)(*param_1 + 0x14))();
  return;
}



// Function: setContinuousAf @ 0001de2c

/* android::QualcommCameraHardware::setContinuousAf(android::CameraParameters const&) */

undefined4 android::QualcommCameraHardware::setContinuousAf(CameraParameters *param_1)

{
  return 0;
}



// Function: filterPictureSizes @ 0001de30

/* android::QualcommCameraHardware::filterPictureSizes() */

void __thiscall android::QualcommCameraHardware::filterPictureSizes(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  undefined4 *puVar3;
  undefined4 uVar4;
  
  iVar2 = DAT_0001deb0;
  iVar1 = DAT_0001dea0;
  if (*(int *)(DAT_0001de90 + 0x1de36) == 0) {
    puVar3 = *(undefined4 **)(DAT_0001de98 + 0x1de44);
    *(int *)(DAT_0001de94 + 0x1de40) = DAT_0001de9c + 0x1de4c;
    uVar4 = 0x990;
    *(undefined4 *)(iVar1 + 0x1de52) = *puVar3;
    *(undefined4 *)(this + 0xd20) = 0xcc0;
  }
  else {
    puVar3 = *(undefined4 **)(DAT_0001dea8 + 0x1de6e);
    *(int *)(DAT_0001dea4 + 0x1de6a) = DAT_0001deac + 0x1de76;
    *(undefined4 *)(iVar2 + 0x1de7c) = *puVar3;
    *(undefined4 *)(this + 0xd20) = 0x640;
    uVar4 = 0x4b0;
  }
  *(undefined4 *)(this + 0xd24) = uVar4;
  return;
}



// Function: supportsSceneDetection @ 0001deb4

/* android::QualcommCameraHardware::supportsSceneDetection() */

undefined4 android::QualcommCameraHardware::supportsSceneDetection(void)

{
  int *piVar1;
  int *piVar2;
  
  piVar2 = *(int **)(DAT_0001dedc + 0x1debc);
  piVar1 = piVar2 + 0x15;
  while ((*(int *)(DAT_0001dee0 + 0x1dec0) != *piVar2 || ((char)piVar2[2] == '\0'))) {
    piVar2 = piVar2 + 3;
    if (piVar2 == piVar1) {
      return 0;
    }
  }
  return 1;
}



// Function: supportsSelectableZoneAf @ 0001dee4

/* android::QualcommCameraHardware::supportsSelectableZoneAf() */

undefined4 android::QualcommCameraHardware::supportsSelectableZoneAf(void)

{
  int *piVar1;
  int *piVar2;
  
  piVar2 = *(int **)(DAT_0001df0c + 0x1deec);
  piVar1 = piVar2 + 0x15;
  while ((*(int *)(DAT_0001df10 + 0x1def0) != *piVar2 || (*(char *)((int)piVar2 + 9) == '\0'))) {
    piVar2 = piVar2 + 3;
    if (piVar2 == piVar1) {
      return 0;
    }
  }
  return 1;
}



// Function: supportsFaceDetection @ 0001df14

/* android::QualcommCameraHardware::supportsFaceDetection() */

undefined4 android::QualcommCameraHardware::supportsFaceDetection(void)

{
  int *piVar1;
  int *piVar2;
  
  piVar2 = *(int **)(DAT_0001df3c + 0x1df1c);
  piVar1 = piVar2 + 0x15;
  while ((*(int *)(DAT_0001df40 + 0x1df20) != *piVar2 || (*(char *)((int)piVar2 + 10) == '\0'))) {
    piVar2 = piVar2 + 3;
    if (piVar2 == piVar1) {
      return 0;
    }
  }
  return 1;
}



// Function: mapBuffer @ 0001df44

/* android::QualcommCameraHardware::mapBuffer(msm_frame*) */

int __thiscall
android::QualcommCameraHardware::mapBuffer(QualcommCameraHardware *this,msm_frame *param_1)

{
  int iVar1;
  QualcommCameraHardware *pQVar2;
  
  iVar1 = 0;
  pQVar2 = this + 0xc60;
  while( true ) {
    if (*(int *)(this + 0xd5c) <= iVar1) {
      return -1;
    }
    if (*(int *)(*(int *)pQVar2 + 0x10) == *(int *)(param_1 + 0x10)) break;
    iVar1 = iVar1 + 1;
    pQVar2 = pQVar2 + 0x10;
  }
  return iVar1;
}



// Function: mapFrame @ 0001df6c

/* android::QualcommCameraHardware::mapFrame(native_handle const**) */

int __thiscall
android::QualcommCameraHardware::mapFrame(QualcommCameraHardware *this,native_handle **param_1)

{
  int iVar1;
  QualcommCameraHardware *pQVar2;
  
  iVar1 = 0;
  pQVar2 = this + 0xc64;
  while( true ) {
    if (*(int *)(this + 0xd5c) <= iVar1) {
      return -1;
    }
    if (*(native_handle ***)pQVar2 == param_1) break;
    iVar1 = iVar1 + 1;
    pQVar2 = pQVar2 + 0x10;
  }
  return iVar1;
}



// Function: startInitialPreview @ 0001df90

/* android::QualcommCameraHardware::startInitialPreview() */

undefined4 __thiscall
android::QualcommCameraHardware::startInitialPreview(QualcommCameraHardware *this)

{
  this[0x34] = (QualcommCameraHardware)0x1;
  return 0;
}



// Function: stopInitialPreview @ 0001df9c

/* android::QualcommCameraHardware::stopInitialPreview() */

void __thiscall android::QualcommCameraHardware::stopInitialPreview(QualcommCameraHardware *this)

{
  this[0x34] = (QualcommCameraHardware)0x0;
  return;
}



// Function: runFaceDetection @ 0001dfa4

/* android::QualcommCameraHardware::runFaceDetection() */

undefined4 android::QualcommCameraHardware::runFaceDetection(void)

{
  return 0xffffffea;
}



// Function: native_zoom_image @ 0001dfac

/* android::QualcommCameraHardware::native_zoom_image(int, int, int, common_crop_t*) */

undefined4 __thiscall
android::QualcommCameraHardware::native_zoom_image
          (QualcommCameraHardware *this,int param_1,int param_2,int param_3,common_crop_t *param_4)

{
  int iVar1;
  int iVar2;
  undefined4 *puVar3;
  undefined4 uVar4;
  
  iVar2 = DAT_0001e054 + 0x1dfb8;
  puVar3 = *(undefined4 **)(iVar2 + DAT_0001e058);
  *puVar3 = 1;
  puVar3[1] = *(undefined4 *)(this + 0xd10);
  uVar4 = *(undefined4 *)(this + 0xd14);
  puVar3[5] = param_1;
  puVar3[4] = param_2;
  puVar3[2] = uVar4;
  puVar3[3] = 2;
  puVar3[7] = *(undefined4 *)(this + 0xd10);
  uVar4 = *(undefined4 *)(this + 0xd14);
  puVar3[0xb] = param_1;
  puVar3[10] = param_3;
  puVar3[8] = uVar4;
  puVar3[0x17] = 0;
  puVar3[9] = 2;
  puVar3[0x16] = 0xffffffff;
  puVar3[0x15] = 0xff;
  if ((*(int *)param_4 == 0) || (*(int *)(param_4 + 8) == 0)) {
    iVar1 = *(int *)(iVar2 + DAT_0001e058);
    *(undefined4 *)(iVar1 + 0x34) = 0;
    *(undefined4 *)(iVar1 + 0x38) = 0;
    *(undefined4 *)(iVar1 + 0x3c) = *(undefined4 *)(this + 0xd10);
    *(undefined4 *)(iVar1 + 0x40) = *(undefined4 *)(this + 0xd14);
  }
  else {
    puVar3[0xd] = ((uint)((*(int *)(param_4 + 4) + 1) - *(int *)param_4) >> 1) - 1;
    puVar3[0xe] = ((uint)((*(int *)(param_4 + 0xc) + 1) - *(int *)(param_4 + 8)) >> 1) - 1;
    puVar3[0xf] = *(undefined4 *)param_4;
    puVar3[0x10] = *(undefined4 *)(param_4 + 8);
  }
  iVar2 = *(int *)(iVar2 + DAT_0001e058);
  *(undefined4 *)(iVar2 + 0x44) = 0;
  *(undefined4 *)(iVar2 + 0x48) = 0;
  *(undefined4 *)(iVar2 + 0x4c) = *(undefined4 *)(this + 0xd10);
  *(undefined4 *)(iVar2 + 0x50) = *(undefined4 *)(this + 0xd14);
  return 1;
}



// Function: recordingEnabled @ 0001e05c

/* android::QualcommCameraHardware::recordingEnabled() */

uint __thiscall android::QualcommCameraHardware::recordingEnabled(QualcommCameraHardware *this)

{
  uint uVar1;
  
  if ((this[0x34] == (QualcommCameraHardware)0x0) || (*(int *)(this + 0xcfc) == 0)) {
    uVar1 = 0;
  }
  else {
    uVar1 = (uint)(*(int *)(this + 0xcf0) << 0x1a) >> 0x1f;
  }
  return uVar1;
}



// Function: YUY2toNV21 @ 0001e078

/* android::QualcommCameraHardware::YUY2toNV21(void*, void*, unsigned int, unsigned int) */

undefined4 __thiscall
android::QualcommCameraHardware::YUY2toNV21
          (QualcommCameraHardware *this,void *param_1,void *param_2,uint param_3,uint param_4)

{
  int iVar1;
  void *pvVar2;
  uint uVar3;
  uint uVar4;
  uint uVar5;
  undefined1 *puVar6;
  int iVar7;
  
  iVar7 = 0;
  iVar1 = param_3 * param_4;
  uVar4 = param_3 * 2;
  pvVar2 = param_1;
  for (uVar5 = 0; uVar5 != param_4; uVar5 = uVar5 + 1) {
    puVar6 = (undefined1 *)((int)param_2 + iVar7);
    for (uVar3 = 1; uVar3 < uVar4; uVar3 = uVar3 + 2) {
      *puVar6 = *(undefined1 *)((int)pvVar2 + uVar3);
      puVar6 = puVar6 + 1;
    }
    iVar7 = iVar7 + (param_3 & 0x7fffffff);
    pvVar2 = (void *)((int)pvVar2 + uVar4);
  }
  pvVar2 = param_1;
  for (uVar5 = 0; uVar5 < param_4; uVar5 = uVar5 + 2) {
    puVar6 = (undefined1 *)((int)param_2 + iVar1);
    for (uVar3 = 0; uVar3 < uVar4; uVar3 = uVar3 + 4) {
      iVar1 = iVar1 + 1;
      *puVar6 = *(undefined1 *)((int)pvVar2 + uVar3);
      puVar6 = puVar6 + 1;
    }
    pvVar2 = (void *)((int)pvVar2 + param_3 * 4);
  }
  for (uVar5 = 0; uVar5 < param_4; uVar5 = uVar5 + 2) {
    puVar6 = (undefined1 *)((int)param_2 + iVar1);
    for (uVar3 = 0; uVar3 < uVar4; uVar3 = uVar3 + 4) {
      iVar1 = iVar1 + 1;
      *puVar6 = *(undefined1 *)((int)param_1 + uVar3 + 2);
      puVar6 = puVar6 + 1;
    }
    param_1 = (void *)((int)param_1 + param_3 * 4);
  }
  return 1;
}



// Function: setDropFrame @ 0001e108

/* android::QualcommCameraHardware::setDropFrame(int) */

void __thiscall
android::QualcommCameraHardware::setDropFrame(QualcommCameraHardware *this,int param_1)

{
  if (*(int *)(this + 0x67c) < param_1) {
    *(int *)(this + 0x67c) = param_1;
  }
  return;
}



// Function: pointer @ 0001e118

/* android::QualcommCameraHardware::MMCameraDL::pointer() */

undefined4 __thiscall android::QualcommCameraHardware::MMCameraDL::pointer(MMCameraDL *this)

{
  return *(undefined4 *)(this + 8);
}



// Function: getVideoBuffer @ 0001e11c

/* android::QualcommCameraHardware::getVideoBuffer(int) */

void android::QualcommCameraHardware::getVideoBuffer(int param_1)

{
  *(undefined4 *)param_1 = 0;
  return;
}



// Function: msgTypeEnabled @ 0001e124

/* android::QualcommCameraHardware::msgTypeEnabled(int) */

bool __thiscall
android::QualcommCameraHardware::msgTypeEnabled(QualcommCameraHardware *this,int param_1)

{
  return (param_1 & *(uint *)(this + 0xcf0)) != 0;
}



// Function: isValidDimension @ 0001e134

/* android::QualcommCameraHardware::isValidDimension(int, int) */

undefined4 __thiscall
android::QualcommCameraHardware::isValidDimension
          (QualcommCameraHardware *this,int param_1,int param_2)

{
  int iVar1;
  int iVar2;
  
  if ((((param_1 == (param_1 + 0xfU & 0xfff0)) && (param_2 == (param_2 + 0xfU & 0xfff0))) &&
      (param_1 <= *(int *)(this + 0xd20))) && (param_2 <= *(int *)(this + 0xd24))) {
    iVar1 = __aeabi_idiv(param_1 << 0xc,param_2);
    iVar2 = 0;
    do {
      if (*(int *)(DAT_0001e188 + 0x1e176 + iVar2) == iVar1) {
        return 1;
      }
      iVar2 = iVar2 + 0xc;
    } while (iVar2 != 0x48);
  }
  return 0;
}



// Function: HAL_isIn3DMode @ 0001e18c

bool HAL_isIn3DMode(void)

{
  return *(int *)(DAT_0001e19c + 0x1e192) == 2;
}



// Function: do_destroy @ 0001e1a0

/* android::Vector<msm_frame*>::do_destroy(void*, unsigned int) const */

void * android::Vector<msm_frame*>::do_destroy(void *param_1,uint param_2)

{
  return param_1;
}



// Function: do_splat @ 0001e1a4

/* android::Vector<msm_frame*>::do_splat(void*, void const*, unsigned int) const */

Vector<msm_frame*> * __thiscall
android::Vector<msm_frame*>::do_splat
          (Vector<msm_frame*> *this,void *param_1,void *param_2,uint param_3)

{
  for (; param_3 != 0; param_3 = param_3 - 1) {
    this = *(Vector<msm_frame*> **)param_2;
    *(Vector<msm_frame*> **)param_1 = this;
    param_1 = (undefined4 *)((int)param_1 + 4);
  }
  return this;
}



// Function: FUN_0001e1ae @ 0001e1ae

void FUN_0001e1ae(undefined4 param_1,undefined4 *param_2,undefined4 *param_3,int param_4)

{
  for (; param_4 != 0; param_4 = param_4 + -1) {
    *param_2 = *param_3;
    param_2 = param_2 + 1;
  }
  return;
}



// Function: do_construct @ 0001e1b4

/* android::SortedVector<android::key_value_pair_t<android::String8, android::String8>
   >::do_construct(void*, unsigned int) const */

SortedVector<android::key_value_pair_t<android::String8,android::String8>> * __thiscall
android::SortedVector<android::key_value_pair_t<android::String8,android::String8>>::do_construct
          (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *this,
          void *param_1,uint param_2)

{
  for (; param_2 != 0; param_2 = param_2 - 1) {
    android::String8::String8(param_1);
    this = (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *)
           android::String8::String8((String8 *)((int)param_1 + 4));
    param_1 = (String8 *)((int)param_1 + 8);
  }
  return this;
}



// Function: do_destroy @ 0001e1d8

/* android::SortedVector<android::key_value_pair_t<android::String8, android::String8>
   >::do_destroy(void*, unsigned int) const */

SortedVector<android::key_value_pair_t<android::String8,android::String8>> * __thiscall
android::SortedVector<android::key_value_pair_t<android::String8,android::String8>>::do_destroy
          (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *this,
          void *param_1,uint param_2)

{
  for (; param_2 != 0; param_2 = param_2 - 1) {
    android::String8::~String8((String8 *)((int)param_1 + 4));
    this = (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *)
           android::String8::~String8(param_1);
    param_1 = (void *)((int)param_1 + 8);
  }
  return this;
}



// Function: dump @ 0001e1f8

/* android::QualcommCameraHardware::dump(int, android::Vector<android::String16> const&) const */

undefined4 android::QualcommCameraHardware::dump(int param_1,Vector *param_2)

{
  Vector *pVStack_c;
  
  pVStack_c = param_2;
  android::String8::String8((String8 *)&pVStack_c);
  android::String8::~String8((String8 *)&pVStack_c);
  return 0;
}



// Function: ~wp @ 0001e20c

/* android::wp<android::QualcommCameraHardware::MMCameraDL>::~wp() */

wp<android::QualcommCameraHardware::MMCameraDL> * __thiscall
android::wp<android::QualcommCameraHardware::MMCameraDL>::~wp
          (wp<android::QualcommCameraHardware::MMCameraDL> *this)

{
  if (*(int *)this != 0) {
    android::RefBase::weakref_type::decWeak(*(void **)(this + 4));
  }
  return this;
}



// Function: FUN_0001e220 @ 0001e220

pthread_cond_t * FUN_0001e220(pthread_cond_t *param_1)

{
  pthread_cond_destroy(param_1);
  return param_1;
}



// Function: FUN_0001e22c @ 0001e22c

pthread_mutex_t * FUN_0001e22c(pthread_mutex_t *param_1)

{
  pthread_mutex_destroy(param_1);
  return param_1;
}



// Function: storePreviewFrameForPostview @ 0001e238

/* android::QualcommCameraHardware::storePreviewFrameForPostview() */

undefined4 __thiscall
android::QualcommCameraHardware::storePreviewFrameForPostview(QualcommCameraHardware *this)

{
  int iVar1;
  
  iVar1 = DAT_0001e26c + 0x1e244;
  __android_log_print(2,iVar1,DAT_0001e270 + 0x1e24a);
  __android_log_print(2,iVar1,DAT_0001e274 + 0x1e258,*(undefined4 *)(this + 0x420));
  __android_log_print(2,iVar1,DAT_0001e278 + 0x1e266);
  return 1;
}



// Function: getNumberOfVideoBuffers @ 0001e27c

/* android::QualcommCameraHardware::getNumberOfVideoBuffers() */

undefined4 android::QualcommCameraHardware::getNumberOfVideoBuffers(void)

{
  undefined4 *puVar1;
  
  puVar1 = (undefined4 *)(DAT_0001e298 + 0x1e288);
  __android_log_print(6,DAT_0001e29c + 0x1e28c,DAT_0001e2a0 + 0x1e28e,*puVar1);
  return *puVar1;
}



// Function: storeMetaDataInBuffers @ 0001e2a4

/* android::QualcommCameraHardware::storeMetaDataInBuffers(int) */

undefined4 __thiscall
android::QualcommCameraHardware::storeMetaDataInBuffers(QualcommCameraHardware *this,int param_1)

{
  __android_log_print(4,DAT_0001e2c8 + 0x1e2b4,DAT_0001e2c4 + 0x1e2b2,param_1);
  *(int *)(this + 0x3ec) = param_1;
  return 0;
}



// Function: register_record_buffers @ 0001e2cc

/* android::QualcommCameraHardware::register_record_buffers(bool) */

undefined8 android::QualcommCameraHardware::register_record_buffers(bool param_1)

{
  undefined4 in_r1;
  
  __android_log_print(4,DAT_0001e2f0 + 0x1e2e0,DAT_0001e2ec + 0x1e2de,DAT_0001e2e8 + 0x1e2da);
  return CONCAT44(in_r1,1);
}



// Function: previewEnabled @ 0001e2f4

/* android::QualcommCameraHardware::previewEnabled() */

undefined1 android::QualcommCameraHardware::previewEnabled(void)

{
  int in_r0;
  
  __android_log_print(6,DAT_0001e318 + 0x1e30a,DAT_0001e31c + 0x1e30c,*(undefined1 *)(in_r0 + 0x34),
                      *(undefined4 *)(in_r0 + 0xcd0));
  return *(undefined1 *)(in_r0 + 0x34);
}



// Function: receiveLiveSnapshot @ 0001e320

/* android::QualcommCameraHardware::receiveLiveSnapshot(unsigned int) */

void android::QualcommCameraHardware::receiveLiveSnapshot(uint param_1)

{
  __android_log_print(2,DAT_0001e334 + 0x1e32c,DAT_0001e338 + 0x1e32e);
  return;
}



// Function: initLiveSnapshot @ 0001e33c

/* android::QualcommCameraHardware::initLiveSnapshot(int, int) */

bool __thiscall
android::QualcommCameraHardware::initLiveSnapshot
          (QualcommCameraHardware *this,int param_1,int param_2)

{
  int iVar1;
  int iVar2;
  
  iVar1 = DAT_0001e3e4 + 0x1e34c;
  __android_log_print(2,iVar1,DAT_0001e3e8 + 0x1e350);
  if (*(int *)(this + 0x7d8) != 0) {
    __android_log_print(2,iVar1,DAT_0001e3ec + 0x1e366);
    (**(code **)(*(int *)(this + 0x7d8) + 0xc))();
    *(undefined4 *)(this + 0x7d8) = 0;
  }
  iVar2 = DAT_0001e3f0 + 0x1e38a;
  *(int *)(this + 0x430) = (int)(longlong)((double)(longlong)(param_1 * param_2) * 1.5);
  __android_log_print(2,iVar2,DAT_0001e3f4 + 0x1e3a4);
  iVar1 = (**(code **)(this + 0xd00))
                    (0xffffffff,*(undefined4 *)(this + 0x430),1,*(undefined4 *)(this + 0xd04));
  *(int *)(this + 0x7d8) = iVar1;
  if (iVar1 != 0) {
    __android_log_print(2,iVar2,DAT_0001e3fc + 0x1e3dc);
  }
  else {
    __android_log_print(6,iVar2,DAT_0001e3f8 + 0x1e3cc);
  }
  return iVar1 != 0;
}



// Function: set_PreviewWindow @ 0001e400

/* android::QualcommCameraHardware::set_PreviewWindow(void*) */

void __thiscall
android::QualcommCameraHardware::set_PreviewWindow(QualcommCameraHardware *this,void *param_1)

{
  __android_log_print(6,DAT_0001e424 + 0x1e410,DAT_0001e420 + 0x1e40e);
  (**(code **)(*(int *)this + 0x7c))(this,param_1);
  return;
}



// Function: mapJpegBuffer @ 0001e428

/* android::QualcommCameraHardware::mapJpegBuffer(mm_camera_buffer_t*) */

int __thiscall
android::QualcommCameraHardware::mapJpegBuffer
          (QualcommCameraHardware *this,mm_camera_buffer_t *param_1)

{
  int iVar1;
  QualcommCameraHardware *pQVar2;
  int iVar3;
  
  pQVar2 = this + 0x760;
  iVar3 = 0;
  while( true ) {
    if (*(int *)(this + 0xd64) == 0) {
      iVar1 = *(int *)(this + 0xc);
    }
    else {
      iVar1 = 5;
    }
    if (iVar1 <= iVar3) break;
    pQVar2 = pQVar2 + 4;
    if (**(int **)pQVar2 == *(int *)param_1) {
      __android_log_print(6,DAT_0001e470 + 0x1e44e,DAT_0001e474 + 0x1e450,DAT_0001e478 + 0x1e454,
                          iVar3,param_1);
      return iVar3;
    }
    iVar3 = iVar3 + 1;
  }
  return -1;
}



// Function: mapThumbnailBuffer @ 0001e47c

/* android::QualcommCameraHardware::mapThumbnailBuffer(msm_frame*) */

int __thiscall
android::QualcommCameraHardware::mapThumbnailBuffer(QualcommCameraHardware *this,msm_frame *param_1)

{
  int iVar1;
  QualcommCameraHardware *pQVar2;
  int iVar3;
  
  pQVar2 = this + 0x6b8;
  iVar3 = 0;
  while( true ) {
    if (*(int *)(this + 0xd64) == 0) {
      iVar1 = *(int *)(this + 0xc);
    }
    else {
      iVar1 = 5;
    }
    if (iVar1 <= iVar3) break;
    pQVar2 = pQVar2 + 4;
    if (*(int *)pQVar2 == *(int *)(param_1 + 0x10)) {
      __android_log_print(6,DAT_0001e4bc + 0x1e4a0,DAT_0001e4c0 + 0x1e4a2,iVar3);
      return iVar3;
    }
    iVar3 = iVar3 + 1;
  }
  return -1;
}



// Function: mapRawBuffer @ 0001e4c4

/* android::QualcommCameraHardware::mapRawBuffer(msm_frame*) */

int __thiscall
android::QualcommCameraHardware::mapRawBuffer(QualcommCameraHardware *this,msm_frame *param_1)

{
  int iVar1;
  QualcommCameraHardware *pQVar2;
  int iVar3;
  
  pQVar2 = this + 0x74c;
  iVar3 = 0;
  while( true ) {
    if (*(int *)(this + 0xd64) == 0) {
      iVar1 = *(int *)(this + 0xc);
    }
    else {
      iVar1 = 5;
    }
    if (iVar1 <= iVar3) break;
    pQVar2 = pQVar2 + 4;
    if (**(int **)pQVar2 == *(int *)(param_1 + 0x10)) {
      __android_log_print(6,DAT_0001e508 + 0x1e4ec,DAT_0001e50c + 0x1e4ee,iVar3);
      return iVar3;
    }
    iVar3 = iVar3 + 1;
  }
  return -1;
}



// Function: mapvideoBuffer @ 0001e510

/* android::QualcommCameraHardware::mapvideoBuffer(msm_frame*) */

int __thiscall
android::QualcommCameraHardware::mapvideoBuffer(QualcommCameraHardware *this,msm_frame *param_1)

{
  QualcommCameraHardware *pQVar1;
  int iVar2;
  
  pQVar1 = this + 0x788;
  iVar2 = 0;
  while( true ) {
    if (*(int *)(DAT_0001e54c + 0x1e51e) <= iVar2) {
      return -1;
    }
    pQVar1 = pQVar1 + 4;
    if (**(int **)pQVar1 == *(int *)(param_1 + 0x10)) break;
    iVar2 = iVar2 + 1;
  }
  __android_log_print(6,DAT_0001e550 + 0x1e538,DAT_0001e554 + 0x1e53a,iVar2);
  return iVar2;
}



// Function: filterVideoSizes @ 0001e558

/* android::QualcommCameraHardware::filterVideoSizes() */

void android::QualcommCameraHardware::filterVideoSizes(void)

{
  int iVar1;
  undefined4 *puVar2;
  int iVar3;
  
  iVar1 = DAT_0001e5cc;
  iVar3 = DAT_0001e5c0;
  if (*(int *)(DAT_0001e5b0 + 0x1e560) == 0) {
    puVar2 = (undefined4 *)(DAT_0001e5b8 + 0x1e572);
    *(int *)(DAT_0001e5b4 + 0x1e56e) = DAT_0001e5bc + 0x1e576;
    *puVar2 = 6;
    iVar3 = iVar3 + 0x1e57e;
  }
  else {
    *(int *)(DAT_0001e5c4 + 0x1e588) = DAT_0001e5c8 + 0x1e58e;
    iVar3 = DAT_0001e5d0;
    *(undefined4 *)(iVar1 + 0x1e592) = 3;
    iVar3 = iVar3 + 0x1e59a;
  }
  __android_log_print(2,DAT_0001e5d4 + 0x1e5a2,DAT_0001e5d8 + 0x1e5a4,iVar3,
                      *(undefined4 *)(DAT_0001e5dc + 0x1e5a6));
  return;
}



// Function: filterPreviewSizes @ 0001e5e0

/* android::QualcommCameraHardware::filterPreviewSizes() */

void android::QualcommCameraHardware::filterPreviewSizes(void)

{
  int iVar1;
  int *piVar2;
  int iVar3;
  undefined4 *puVar4;
  
  iVar1 = DAT_0001e64c;
  if (*(int *)(DAT_0001e640 + 0x1e5e8) == 0) {
    piVar2 = (int *)(DAT_0001e648 + 0x1e5f8);
    *(undefined4 *)(DAT_0001e644 + 0x1e5f4) = 7;
    iVar3 = DAT_0001e650;
    *piVar2 = iVar1 + 0x1e5fc;
    iVar3 = iVar3 + 0x1e602;
  }
  else {
    if (*(int *)(DAT_0001e640 + 0x1e5e8) == 1) {
      puVar4 = (undefined4 *)(DAT_0001e658 + 0x1e618);
      *(int *)(DAT_0001e654 + 0x1e614) = DAT_0001e65c + 0x1e61a;
      *puVar4 = 3;
    }
    iVar3 = (int)&DAT_0001e640 + DAT_0001e66c;
  }
  __android_log_print(2,DAT_0001e660 + 0x1e62c,DAT_0001e664 + 0x1e62e,iVar3,
                      *(undefined4 *)(DAT_0001e668 + 0x1e630));
  return;
}



// Function: hasAutoFocusSupport @ 0001e670

/* android::QualcommCameraHardware::hasAutoFocusSupport() */

void __thiscall android::QualcommCameraHardware::hasAutoFocusSupport(QualcommCameraHardware *this)

{
  QualcommCameraHardware QVar1;
  
  QVar1 = (QualcommCameraHardware)(*(int *)(DAT_0001e6a4 + 0x1e678) != 1);
  if (!(bool)QVar1) {
    __android_log_print(4,DAT_0001e6a8 + 0x1e688,DAT_0001e6ac + 0x1e68a);
  }
  this[0xd28] = QVar1;
  if (*(int *)(this + 0xd64) != 0) {
    this[0xd28] = (QualcommCameraHardware)0x0;
  }
  return;
}



// Function: storeTargetType @ 0001e6b0

/* android::QualcommCameraHardware::storeTargetType() */

void android::QualcommCameraHardware::storeTargetType(void)

{
  int iVar1;
  
  iVar1 = DAT_0001e6d0;
  *(undefined4 *)(DAT_0001e6cc + 0x1e6ba) = 6;
  __android_log_print(2,iVar1 + 0x1e6c4,DAT_0001e6d4 + 0x1e6c6);
  return;
}



// Function: HAL_getCameraInfo @ 0001e6d8

void HAL_getCameraInfo(int param_1,uint *param_2)

{
  uint extraout_r1;
  int iVar1;
  int iVar2;
  int iVar3;
  uint uVar4;
  undefined1 auStack_78 [92];
  int local_1c;
  
  local_1c = **(int **)(DAT_0001e79c + 0x1e6e6);
  if (param_2 == (uint *)0x0) {
    __android_log_print(6,DAT_0001e7a0 + 0x1e6f6,DAT_0001e7a4 + 0x1e6f8);
  }
  else {
    property_get(DAT_0001e7a8 + 0x1e708,auStack_78,DAT_0001e7ac + 0x1e70a);
    for (iVar1 = 0; iVar1 < *(int *)(DAT_0001e7b0 + 0x1e712); iVar1 = iVar1 + 1) {
      if (iVar1 == param_1) {
        iVar3 = DAT_0001e7b4 + 0x1e724;
        __android_log_print(4,iVar3,DAT_0001e7b8 + 0x1e728,iVar1);
        iVar2 = iVar1 * 2 + 1;
        iVar1 = DAT_0001e7bc + 0x1e738;
        uVar4 = (uint)(*(int *)(iVar1 + iVar2 * 8) != 0);
        *param_2 = uVar4;
        __aeabi_uidivmod(0x1c2 - *(int *)(iVar1 + iVar2 * 8 + 4),0x168);
        iVar1 = DAT_0001e7c0 + 0x1e75c;
        iVar2 = DAT_0001e7c4 + 0x1e760;
        param_2[1] = extraout_r1;
        __android_log_print(4,iVar3,iVar1,iVar2,uVar4,extraout_r1);
        param_2[2] = 1;
        goto LAB_0001e784;
      }
    }
    __android_log_print(6,DAT_0001e7c8 + 0x1e780,DAT_0001e7cc + 0x1e782,param_1);
  }
LAB_0001e784:
  if (local_1c != **(int **)(DAT_0001e7d0 + 0x1e78c)) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail();
  }
  return;
}



// Function: getCameraInfo @ 0001e7d4

/* android::QualcommCameraHardware::getCameraInfo() */

void android::QualcommCameraHardware::getCameraInfo(void)

{
  int iVar1;
  undefined4 uVar2;
  uint uVar3;
  int iVar4;
  int *piVar5;
  undefined4 *puVar6;
  int iVar7;
  int *piVar8;
  int iVar9;
  int iVar10;
  int *piVar11;
  
  iVar4 = DAT_0001e908 + 0x1e7e2;
  __android_log_print(4,iVar4,DAT_0001e90c + 0x1e7e4);
  iVar1 = dlopen(DAT_0001e910 + 0x1e7f0,0);
  __android_log_print(4,iVar4,DAT_0001e914 + 0x1e7fa,iVar1);
  if (iVar1 == 0) {
    uVar2 = dlerror();
    __android_log_print(6,iVar4,DAT_0001e918 + 0x1e810,uVar2);
  }
  iVar7 = DAT_0001e924;
  iVar4 = DAT_0001e920;
  uVar2 = dlsym(iVar1,DAT_0001e91c + 0x1e822);
  puVar6 = *(undefined4 **)(iVar4 + 0x1e82e);
  piVar11 = (int *)(DAT_0001e928 + 0x1e834);
  piVar8 = (int *)(DAT_0001e92c + 0x1e842);
  iVar9 = DAT_0001e930 + 0x1e844;
  iVar10 = DAT_0001e934 + 0x1e846;
  *puVar6 = uVar2;
  storeTargetType();
  (*(code *)*puVar6)(iVar7 + 0x1e832,piVar11);
  iVar4 = DAT_0001e940;
  __android_log_print(4,DAT_0001e938 + 0x1e85e,DAT_0001e93c + 0x1e860,*piVar11);
  piVar5 = (int *)(iVar7 + 0x1e83a);
  iVar4 = iVar4 + 0x1e86c;
  for (iVar7 = 0; iVar7 < *piVar11; iVar7 = iVar7 + 1) {
    if ((*piVar5 == 0) && (*piVar8 == 6)) {
      uVar3 = piVar5[-2] | 8;
    }
    else {
      uVar3 = piVar5[-2] | 4;
    }
    piVar5[-2] = uVar3;
    __android_log_print(4,iVar4,iVar9,iVar7);
    __android_log_print(4,iVar4,iVar10,(int)(char)piVar5[-1]);
    __android_log_print(4,iVar4,DAT_0001e944 + 0x1e8b6,piVar5[-2]);
    __android_log_print(4,iVar4,DAT_0001e948 + 0x1e8c4,*piVar5);
    __android_log_print(4,iVar4,DAT_0001e94c + 0x1e8d2,piVar5[1]);
    piVar5 = piVar5 + 4;
  }
  if (iVar1 != 0) {
    dlclose(iVar1);
    __android_log_print(2,DAT_0001e950 + 0x1e8f2,DAT_0001e954 + 0x1e8f4);
  }
  __android_log_print(4,DAT_0001e958 + 0x1e900,DAT_0001e95c + 0x1e902);
  return;
}



// Function: HAL_getNumberOfCameras @ 0001e960

undefined4 HAL_getNumberOfCameras(void)

{
  android::QualcommCameraHardware::getCameraInfo();
  return *(undefined4 *)(DAT_0001e970 + 0x1e96c);
}



// Function: receive_camframe_error_timeout @ 0001e974

/* android::QualcommCameraHardware::receive_camframe_error_timeout() */

void __thiscall
android::QualcommCameraHardware::receive_camframe_error_timeout(QualcommCameraHardware *this)

{
  int iVar1;
  
  iVar1 = DAT_0001e9d4 + 0x1e982;
  __android_log_print(4,iVar1,DAT_0001e9d8 + 0x1e98a);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f8));
  __android_log_print(6,iVar1,DAT_0001e9dc + 0x1e99e);
  *(int *)(DAT_0001e9e0 + 0x1e9aa) = *(int *)(DAT_0001e9e0 + 0x1e9aa) + 1;
  (**(code **)(this + 0xcf4))(1,0x3e9,0,*(undefined4 *)(this + 0xd04));
  __android_log_print(4,iVar1,DAT_0001e9e4 + 0x1e9c6);
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f8));
  return;
}



// Function: setCallbacks @ 0001e9e8

/* android::QualcommCameraHardware::setCallbacks(void (*)(int, int, int, void*), void (*)(int,
   camera_memory const*, unsigned int, camera_frame_metadata*, void*), void (*)(long long, int,
   camera_memory const*, unsigned int, void*), camera_memory* (*)(int, unsigned int, unsigned int,
   void*), void*) */

void __thiscall
android::QualcommCameraHardware::setCallbacks
          (QualcommCameraHardware *this,_func_void_int_int_int_void_ptr *param_1,
          _func_void_int_camera_memory_ptr_uint_camera_frame_metadata_ptr_void_ptr *param_2,
          _func_void_longlong_int_camera_memory_ptr_uint_void_ptr *param_3,
          _func_camera_memory_ptr_int_uint_uint_void_ptr *param_4,void *param_5)

{
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  *(_func_void_int_int_int_void_ptr **)(this + 0xcf4) = param_1;
  *(_func_camera_memory_ptr_int_uint_uint_void_ptr **)(this + 0xd00) = param_4;
  *(_func_void_int_camera_memory_ptr_uint_camera_frame_metadata_ptr_void_ptr **)(this + 0xcf8) =
       param_2;
  *(_func_void_longlong_int_camera_memory_ptr_uint_void_ptr **)(this + 0xcfc) = param_3;
  *(void **)(this + 0xd04) = param_5;
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return;
}



// Function: takeLiveSnapshot @ 0001ea20

/* android::QualcommCameraHardware::takeLiveSnapshot() */

undefined4 __thiscall
android::QualcommCameraHardware::takeLiveSnapshot(QualcommCameraHardware *this)

{
  undefined4 uVar1;
  int iVar2;
  
  iVar2 = DAT_0001ea60 + 0x1ea2c;
  __android_log_print(2,iVar2,DAT_0001ea64 + 0x1ea34);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  __android_log_print(2,iVar2,DAT_0001ea68 + 0x1ea48);
  uVar1 = (**(code **)(*(int *)this + 0x54))(this);
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return uVar1;
}



// Function: isInitialized @ 0001ea6c

/* android::QualcommCameraHardware::FrameQueue::isInitialized() */

FrameQueue __thiscall android::QualcommCameraHardware::FrameQueue::isInitialized(FrameQueue *this)

{
  FrameQueue FVar1;
  
  pthread_mutex_lock((pthread_mutex_t *)(this + 8));
  FVar1 = this[0x10];
  pthread_mutex_unlock((pthread_mutex_t *)(this + 8));
  return FVar1;
}



// Function: deinit @ 0001ea88

/* android::QualcommCameraHardware::FrameQueue::deinit() */

void __thiscall android::QualcommCameraHardware::FrameQueue::deinit(FrameQueue *this)

{
  pthread_mutex_lock((pthread_mutex_t *)(this + 8));
  this[0x10] = (FrameQueue)0x0;
  pthread_cond_signal((pthread_cond_t *)(this + 0xc));
  pthread_mutex_unlock((pthread_mutex_t *)(this + 8));
  return;
}



// Function: deinitPreview @ 0001eaac

/* android::QualcommCameraHardware::deinitPreview() */

void __thiscall android::QualcommCameraHardware::deinitPreview(QualcommCameraHardware *this)

{
  int iVar1;
  
  iVar1 = DAT_0001eae0 + 0x1eab8;
  __android_log_print(4,iVar1,DAT_0001eae4 + 0x1eabe);
  FrameQueue::deinit((FrameQueue *)(this + 0x330));
  (*(code *)**(undefined4 **)(DAT_0001eae8 + 0x1eace))();
  __android_log_print(4,iVar1,DAT_0001eaec + 0x1eadc);
  return;
}



// Function: init @ 0001eaf0

/* android::QualcommCameraHardware::FrameQueue::init() */

void __thiscall android::QualcommCameraHardware::FrameQueue::init(FrameQueue *this)

{
  pthread_mutex_lock((pthread_mutex_t *)(this + 8));
  this[0x10] = (FrameQueue)0x1;
  pthread_cond_signal((pthread_cond_t *)(this + 0xc));
  pthread_mutex_unlock((pthread_mutex_t *)(this + 8));
  return;
}



// Function: notifyShutter @ 0001eb14

/* android::QualcommCameraHardware::notifyShutter(bool) */

void __thiscall
android::QualcommCameraHardware::notifyShutter(QualcommCameraHardware *this,bool param_1)

{
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3a8));
  if (param_1) {
    (**(code **)(this + 0xcf4))(2,0,1,*(undefined4 *)(this + 0xd04));
  }
  else if (((this[0x3a4] != (QualcommCameraHardware)0x0) &&
           (*(code **)(this + 0xcf4) != (code *)0x0)) && (*(int *)(this + 0xcf0) << 0x1e < 0)) {
    (**(code **)(this + 0xcf4))(2,0,param_1,*(undefined4 *)(this + 0xd04));
    this[0x3a4] = (QualcommCameraHardware)param_1;
  }
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3a8));
  return;
}



// Function: releaseRecordingFrame @ 0001eb68

/* android::QualcommCameraHardware::releaseRecordingFrame(void const*) */

void __thiscall
android::QualcommCameraHardware::releaseRecordingFrame(QualcommCameraHardware *this,void *param_1)

{
  int iVar1;
  undefined4 *puVar2;
  int iVar3;
  void *pvVar4;
  QualcommCameraHardware *pQVar5;
  int iVar6;
  int iVar7;
  int *piVar8;
  int iVar9;
  void *pvVar10;
  void *pvVar11;
  
  pvVar10 = param_1;
  pvVar11 = param_1;
  __android_log_print(6,DAT_0001ecb8 + 0x1eb7e,DAT_0001ecb0 + 0x1eb7c,DAT_0001ecb4 + 0x1eb7a,param_1
                      ,param_1);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x410));
  this[0x3fd] = (QualcommCameraHardware)0x1;
  pthread_cond_signal((pthread_cond_t *)(this + 0x414));
  if (*(int *)(DAT_0001ecbc + 0x1eba2) - 4U < 3) {
    iVar7 = 0;
    pQVar5 = this;
    for (iVar6 = 0; iVar6 < *(int *)(DAT_0001ecc0 + 0x1ebb4); iVar6 = iVar6 + 1) {
      if (*(int *)(this + 0x3ec) == 0) {
        pvVar4 = *(void **)(*(int *)(this + 0xcc0) + iVar7 + 0x10);
        if ((pvVar4 == (void *)0x0) || (param_1 != pvVar4)) goto LAB_0001ec0c;
        iVar1 = DAT_0001eccc + 0x1ec00;
        iVar3 = DAT_0001ecd0 + 0x1ec02;
LAB_0001ec00:
        __android_log_print(2,iVar1,iVar3,param_1,pvVar10,pvVar11);
        iVar7 = *(int *)(this + 0xcc0) + iVar7;
        goto LAB_0001ec18;
      }
      if ((*(int **)(pQVar5 + 0x7b4) != (int *)0x0) &&
         ((void *)**(int **)(pQVar5 + 0x7b4) == param_1)) {
        iVar7 = iVar6 * 0x68;
        iVar1 = DAT_0001ecc4 + 0x1ebdc;
        iVar3 = DAT_0001ecc8 + 0x1ebde;
        param_1 = *(void **)(*(int *)(this + 0xcc0) + iVar7 + 0x10);
        goto LAB_0001ec00;
      }
LAB_0001ec0c:
      iVar7 = iVar7 + 0x68;
      pQVar5 = pQVar5 + 4;
    }
    iVar7 = 0;
LAB_0001ec18:
    iVar3 = DAT_0001ece8;
    iVar1 = DAT_0001ece4;
    piVar8 = (int *)(DAT_0001ecd4 + 0x1ec20);
    if (iVar6 < *piVar8) {
      pthread_mutex_lock((pthread_mutex_t *)(this + 0x35c));
      if (this[0x358] != (QualcommCameraHardware)0x0) {
        puVar2 = *(undefined4 **)(DAT_0001ecd8 + 0x1ec42);
        *(undefined1 *)(*(int *)(this + 0xcc8) + iVar6) = 0;
        (*(code *)*puVar2)(0,iVar7);
      }
      pthread_mutex_unlock((pthread_mutex_t *)(this + 0x35c));
    }
    else {
      iVar7 = 0;
      __android_log_print(6,DAT_0001ecdc + 0x1ec60,DAT_0001ece0 + 0x1ec66);
      for (iVar6 = 0; iVar6 < *piVar8; iVar6 = iVar6 + 1) {
        iVar9 = *(int *)(this + 0xcc0) + iVar7;
        iVar7 = iVar7 + 0x68;
        __android_log_print(6,iVar1 + 0x1ec6e,iVar3 + 0x1ec70,iVar6,*(undefined4 *)(iVar9 + 0x10));
      }
    }
  }
  __android_log_print(2,DAT_0001ecec + 0x1eca0,DAT_0001ecf0 + 0x1eca2);
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x410));
  return;
}



// Function: setHistogramOff @ 0001ecf4

/* android::QualcommCameraHardware::setHistogramOff() */

undefined4 __thiscall android::QualcommCameraHardware::setHistogramOff(QualcommCameraHardware *this)

{
  QualcommCameraHardware *pQVar1;
  pthread_mutex_t *__mutex;
  int iVar2;
  
  __mutex = (pthread_mutex_t *)(this + 0x390);
  __android_log_print(2,DAT_0001ed60 + 0x1ed02,DAT_0001ed64 + 0x1ed04);
  pthread_mutex_lock(__mutex);
  if (*(int *)(this + 900) != 0) {
    iVar2 = 0;
    *(undefined4 *)(this + 900) = 0;
    pthread_mutex_unlock(__mutex);
    (**(code **)(DAT_0001ed68 + 0x1ed38))(2,this + 900);
    pthread_mutex_lock(__mutex);
    pQVar1 = this + 0x780;
    do {
      if (*(int *)pQVar1 != 0) {
        (**(code **)(*(int *)pQVar1 + 0xc))();
        *(undefined4 *)pQVar1 = 0;
      }
      iVar2 = iVar2 + 1;
      pQVar1 = pQVar1 + 4;
    } while (iVar2 != 3);
  }
  pthread_mutex_unlock(__mutex);
  return 0;
}



// Function: setHistogramOn @ 0001ed6c

/* android::QualcommCameraHardware::setHistogramOn() */

undefined4 android::QualcommCameraHardware::setHistogramOn(void)

{
  int in_r0;
  undefined4 *puVar1;
  pthread_mutex_t *__mutex;
  int iVar2;
  int iVar3;
  int iVar4;
  int *piVar5;
  
  __mutex = (pthread_mutex_t *)(in_r0 + 0x390);
  __android_log_print(2,DAT_0001ee38 + 0x1ed7a,DAT_0001ee3c + 0x1ed7e);
  pthread_mutex_lock(__mutex);
  *(undefined1 *)(in_r0 + 0x38c) = 1;
  if (*(int *)(in_r0 + 900) == 1) {
LAB_0001edea:
    pthread_mutex_unlock(__mutex);
  }
  else {
    piVar5 = (int *)(in_r0 + 0x77c);
    iVar2 = 0;
    iVar3 = DAT_0001ee40 + 0x1edae;
    iVar4 = DAT_0001ee44 + 0x1edb0;
    *(undefined4 *)(in_r0 + 0x434) = 0x404;
    *(undefined4 *)(in_r0 + 0x388) = 0xffffffff;
    do {
      puVar1 = (undefined4 *)
               (**(code **)(in_r0 + 0xd00))
                         (0xffffffff,*(undefined4 *)(in_r0 + 0x434),1,*(undefined4 *)(in_r0 + 0xd04)
                         );
      piVar5 = piVar5 + 1;
      *piVar5 = (int)puVar1;
      if (puVar1 == (undefined4 *)0x0) {
        __android_log_print(6,DAT_0001ee48 + 0x1ede6,DAT_0001ee4c + 0x1ede8,iVar2);
        goto LAB_0001edea;
      }
      iVar2 = iVar2 + 1;
      __android_log_print(2,iVar3,iVar4,*puVar1,puVar1[2],puVar1[1],puVar1[3]);
    } while (iVar2 != 3);
    *(undefined4 *)(in_r0 + 900) = 1;
    pthread_mutex_unlock(__mutex);
    (**(code **)(DAT_0001ee50 + 0x1ee2e))(2,in_r0 + 900);
  }
  return 0;
}



// Function: sp @ 0001ee54

/* android::sp<android::MemoryHeapBase>::sp(android::MemoryHeapBase*) */

sp<android::MemoryHeapBase> * __thiscall
android::sp<android::MemoryHeapBase>::sp(sp<android::MemoryHeapBase> *this,MemoryHeapBase *param_1)

{
  *(MemoryHeapBase **)this = param_1;
  if (param_1 != (MemoryHeapBase *)0x0) {
    android::RefBase::incStrong(param_1 + *(int *)(*(int *)param_1 + -0x10));
  }
  return this;
}



// Function: clear @ 0001ee70

/* android::sp<android::MemoryHeapBase>::clear() */

void __thiscall android::sp<android::MemoryHeapBase>::clear(sp<android::MemoryHeapBase> *this)

{
  int *piVar1;
  
  piVar1 = *(int **)this;
  if (piVar1 != (int *)0x0) {
    android::RefBase::decStrong((void *)((int)piVar1 + *(int *)(*piVar1 + -0x10)));
    *(undefined4 *)this = 0;
  }
  return;
}



// Function: ~sp @ 0001ee8c

/* android::sp<android::MemoryHeapPmem>::~sp() */

sp<android::MemoryHeapPmem> * __thiscall
android::sp<android::MemoryHeapPmem>::~sp(sp<android::MemoryHeapPmem> *this)

{
  int *piVar1;
  
  piVar1 = *(int **)this;
  if (piVar1 != (int *)0x0) {
    android::RefBase::decStrong((void *)((int)piVar1 + *(int *)(*piVar1 + -0x10)));
  }
  return this;
}



// Function: ~sp @ 0001eea8

/* android::sp<android::QualcommCameraHardware::IonPool>::~sp() */

sp<android::QualcommCameraHardware::IonPool> * __thiscall
android::sp<android::QualcommCameraHardware::IonPool>::~sp
          (sp<android::QualcommCameraHardware::IonPool> *this)

{
  if (*(void **)this != (void *)0x0) {
    android::RefBase::decStrong(*(void **)this);
  }
  return this;
}



// Function: operator= @ 0001eebc

/* android::sp<android::QualcommCameraHardware::MMCameraDL>::TEMPNAMEPLACEHOLDERVALUE(android::sp<android::QualcommCameraHardware::MMCameraDL>
   const&) */

sp<android::QualcommCameraHardware::MMCameraDL> * __thiscall
android::sp<android::QualcommCameraHardware::MMCameraDL>::operator=
          (sp<android::QualcommCameraHardware::MMCameraDL> *this,sp *param_1)

{
  void *pvVar1;
  
  pvVar1 = *(void **)param_1;
  if (pvVar1 != (void *)0x0) {
    android::RefBase::incStrong(pvVar1);
  }
  if (*(void **)this != (void *)0x0) {
    android::RefBase::decStrong(*(void **)this);
  }
  *(void **)this = pvVar1;
  return this;
}



// Function: operator= @ 0001eedc

/* android::sp<android::MemoryHeapBase>&
   android::sp<android::MemoryHeapBase>::TEMPNAMEPLACEHOLDERVALUE(android::sp<android::MemoryHeapPmem>
   const&) */

sp * __thiscall
android::sp<android::MemoryHeapBase>::operator=(sp<android::MemoryHeapBase> *this,sp *param_1)

{
  int *piVar1;
  int *piVar2;
  
  piVar2 = *(int **)param_1;
  if (piVar2 != (int *)0x0) {
    android::RefBase::incStrong((void *)((int)piVar2 + *(int *)(*piVar2 + -0x10)));
  }
  piVar1 = *(int **)this;
  if (piVar1 != (int *)0x0) {
    android::RefBase::decStrong((void *)((int)piVar1 + *(int *)(*piVar1 + -0x10)));
  }
  *(int **)this = piVar2;
  return (sp *)this;
}



// Function: operator= @ 0001ef0c

/* android::sp<android::MemoryHeapBase>::TEMPNAMEPLACEHOLDERVALUE(android::MemoryHeapBase*) */

sp<android::MemoryHeapBase> * __thiscall
android::sp<android::MemoryHeapBase>::operator=
          (sp<android::MemoryHeapBase> *this,MemoryHeapBase *param_1)

{
  int *piVar1;
  
  if (param_1 != (MemoryHeapBase *)0x0) {
    android::RefBase::incStrong(param_1 + *(int *)(*(int *)param_1 + -0x10));
  }
  piVar1 = *(int **)this;
  if (piVar1 != (int *)0x0) {
    android::RefBase::decStrong((void *)((int)piVar1 + *(int *)(*piVar1 + -0x10)));
  }
  *(MemoryHeapBase **)this = param_1;
  return this;
}



// Function: operator= @ 0001ef3c

/* android::sp<android::MemoryBase>::TEMPNAMEPLACEHOLDERVALUE(android::MemoryBase*) */

sp<android::MemoryBase> * __thiscall
android::sp<android::MemoryBase>::operator=(sp<android::MemoryBase> *this,MemoryBase *param_1)

{
  int *piVar1;
  
  if (param_1 != (MemoryBase *)0x0) {
    android::RefBase::incStrong(param_1 + *(int *)(*(int *)param_1 + -0xc));
  }
  piVar1 = *(int **)this;
  if (piVar1 != (int *)0x0) {
    android::RefBase::decStrong((void *)((int)piVar1 + *(int *)(*piVar1 + -0xc)));
  }
  *(MemoryBase **)this = param_1;
  return this;
}



// Function: clear @ 0001ef6c

/* android::sp<android::QualcommCameraHardware::IonPool>::clear() */

void __thiscall
android::sp<android::QualcommCameraHardware::IonPool>::clear
          (sp<android::QualcommCameraHardware::IonPool> *this)

{
  if (*(void **)this != (void *)0x0) {
    android::RefBase::decStrong(*(void **)this);
    *(undefined4 *)this = 0;
  }
  return;
}



// Function: FUN_0001ef80 @ 0001ef80

bool FUN_0001ef80(undefined4 param_1,undefined4 param_2)

{
  int iVar1;
  int *piVar2;
  char *pcVar3;
  code *pcVar4;
  
  pcVar4 = *(code **)(*(int *)(DAT_0001efb8 + 0x1ef8c) + 4);
  iVar1 = (*pcVar4)(param_1,param_2,0,pcVar4,param_1,param_2);
  if (iVar1 != 0) {
    piVar2 = (int *)__errno();
    pcVar3 = strerror(*piVar2);
    __android_log_print(6,DAT_0001efbc + 0x1efac,DAT_0001efc0 + 0x1efae,param_1,pcVar3);
  }
  return iVar1 == 0;
}



// Function: enableMsgType @ 0001efc4

/* android::QualcommCameraHardware::enableMsgType(int) */

void __thiscall
android::QualcommCameraHardware::enableMsgType(QualcommCameraHardware *this,int param_1)

{
  int iVar1;
  uint uVar2;
  
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  iVar1 = DAT_0001f004;
  uVar2 = *(uint *)(this + 0xcf0);
  *(uint *)(this + 0xcf0) = param_1 | uVar2;
  if ((2 < *(int *)(iVar1 + 0x1efe4) - 4U) && ((int)((param_1 | uVar2) << 0x1a) < 0)) {
    FUN_0001ef80(10,0);
    *(undefined4 *)(this + 0xd94) = 1;
  }
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return;
}



// Function: FUN_0001f008 @ 0001f008

bool FUN_0001f008(undefined4 param_1,undefined4 param_2,undefined4 param_3)

{
  int iVar1;
  int *piVar2;
  char *pcVar3;
  int iVar4;
  undefined4 local_20;
  undefined4 local_1c;
  undefined4 local_18;
  
  iVar4 = DAT_0001f058 + 0x1f01a;
  local_20 = param_1;
  local_1c = param_2;
  local_18 = param_3;
  __android_log_print(2,iVar4,DAT_0001f05c + 0x1f022,param_1);
  iVar1 = (**(code **)(DAT_0001f060 + 0x1f040))(&local_20);
  if (iVar1 != 0) {
    piVar2 = (int *)__errno();
    pcVar3 = strerror(*piVar2);
    __android_log_print(6,iVar4,DAT_0001f064 + 0x1f04a,param_1,pcVar3);
  }
  return iVar1 == 0;
}



// Function: setAutoFocusStartStop @ 0001f068

/* android::QualcommCameraHardware::setAutoFocusStartStop(int) */

void __thiscall
android::QualcommCameraHardware::setAutoFocusStartStop(QualcommCameraHardware *this,int param_1)

{
  undefined4 uVar1;
  int iVar2;
  
  if (param_1 == 1) {
    iVar2 = DAT_0001f0a0 + 0x1f07c;
  }
  else {
    iVar2 = DAT_0001f09c + 0x1f076;
  }
  __android_log_print(6,DAT_0001f0a4 + 0x1f084,DAT_0001f0a8 + 0x1f086,iVar2);
  if (param_1 == 0) {
    uVar1 = 0xd;
  }
  else {
    uVar1 = 0xc;
  }
  FUN_0001f008(uVar1,0,0);
  return;
}



// Function: sendCommand @ 0001f0ac

/* android::QualcommCameraHardware::sendCommand(int, int, int) */

undefined4 __thiscall
android::QualcommCameraHardware::sendCommand
          (QualcommCameraHardware *this,int param_1,int param_2,int param_3)

{
  int iVar1;
  undefined4 uVar2;
  int iVar3;
  int iVar4;
  int iVar5;
  int iVar6;
  int iVar7;
  
  iVar3 = DAT_0001f144 + 0x1f0bc;
  iVar4 = param_1;
  iVar5 = param_2;
  iVar6 = param_3;
  iVar7 = param_3;
  __android_log_print(2,iVar3,DAT_0001f148 + 0x1f0ca,DAT_0001f14c + 0x1f0c6,param_1,param_2,param_3,
                      param_3);
  iVar1 = DAT_0001f150;
  if (param_1 == 0x44f) {
    uVar2 = 0x11;
    *(int *)(this + 0x40) = param_2;
    *(int *)(this + 0x44) = param_3;
  }
  else {
    if (param_1 < 0x450) {
      if (param_1 != 3) {
        return 0;
      }
      *(undefined4 *)(this + 0x6a4) = 1;
      __android_log_print(6,iVar3,iVar1 + 0x1f102,1,iVar4,iVar5,iVar6,iVar7);
      return 0;
    }
    if (param_1 != 0x451) {
      if (param_1 != 0x456) {
        return 0;
      }
      this[0x6b8] = (QualcommCameraHardware)(param_2 != 0);
      return 0;
    }
    __android_log_print(2,iVar3,DAT_0001f154 + 0x1f114,param_2,iVar4,iVar5,iVar6,iVar7);
    *(int *)(this + 0x48) = param_2;
    uVar2 = 0xf;
    param_3 = 0;
  }
  FUN_0001f008(uVar2,param_2,param_3);
  return 0;
}



// Function: cancelAutoFocusDefault @ 0001f158

/* android::QualcommCameraHardware::cancelAutoFocusDefault() */

undefined4 __thiscall
android::QualcommCameraHardware::cancelAutoFocusDefault(QualcommCameraHardware *this)

{
  undefined4 uVar1;
  
  if (this[0xd28] == (QualcommCameraHardware)0x0) {
    uVar1 = 0;
  }
  else {
    __android_log_print(2,DAT_0001f1b8 + 0x1f170,DAT_0001f1bc + 0x1f176,
                        *(undefined4 *)(this + 0x670));
    if (this[0x5cc] == (QualcommCameraHardware)0x0) {
      if (*(int *)(this + 0x670) == 0) {
        uVar1 = 0;
      }
      else {
        uVar1 = FUN_0001f008(0xb,*(uint *)(this + 0x668) | 0x100);
      }
    }
    else {
      uVar1 = setAutoFocusStartStop(this,0);
    }
    __android_log_print(2,DAT_0001f1c0 + 0x1f1ae,DAT_0001f1c4 + 0x1f1b0,uVar1);
  }
  return uVar1;
}



// Function: FUN_0001f1c8 @ 0001f1c8

bool FUN_0001f1c8(undefined4 param_1,undefined4 param_2,undefined4 *param_3)

{
  int iVar1;
  int *piVar2;
  char *pcVar3;
  int iVar4;
  undefined4 local_20;
  undefined4 local_1c;
  undefined4 local_18;
  
  iVar4 = DAT_0001f21c + 0x1f1d8;
  local_20 = param_1;
  local_1c = param_2;
  __android_log_print(2,iVar4,DAT_0001f220 + 0x1f1e0,param_1);
  iVar1 = (**(code **)(DAT_0001f224 + 0x1f200))(&local_20);
  if (iVar1 == 0) {
    *param_3 = local_18;
  }
  else {
    piVar2 = (int *)__errno();
    pcVar3 = strerror(*piVar2);
    __android_log_print(6,iVar4,DAT_0001f228 + 0x1f206,param_1,pcVar3);
  }
  return iVar1 == 0;
}



// Function: native_set_parms @ 0001f22c

/* android::QualcommCameraHardware::native_set_parms(mm_camera_parm_type_t, unsigned short, void*,
   int*) */

bool __thiscall
android::QualcommCameraHardware::native_set_parms
          (undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4,uint *param_5
          )

{
  uint uVar1;
  int *piVar2;
  char *pcVar3;
  int iVar4;
  uint uVar5;
  
  uVar1 = (**(code **)(DAT_0001f2a4 + 0x1f23a))(param_2,param_4);
  iVar4 = DAT_0001f2a8 + 0x1f24e;
  uVar5 = 1 - uVar1;
  if (1 < uVar1) {
    uVar5 = 0;
  }
  if (uVar1 == 5) {
    uVar5 = uVar5 | 1;
  }
  __android_log_print(2,iVar4,DAT_0001f2ac + 0x1f250,uVar1);
  if (uVar5 == 0) {
    piVar2 = (int *)__errno();
    pcVar3 = strerror(*piVar2);
    __android_log_print(6,iVar4,DAT_0001f2b0 + 0x1f288,DAT_0001f2b4 + 0x1f28c,param_2,param_3,pcVar3
                        ,uVar1);
    *param_5 = uVar1;
  }
  else {
    *param_5 = uVar1;
  }
  return uVar5 != 0;
}



// Function: native_set_parms @ 0001f2b8

/* android::QualcommCameraHardware::native_set_parms(mm_camera_parm_type_t, unsigned short, void*)
    */

undefined8 __thiscall
android::QualcommCameraHardware::native_set_parms
          (QualcommCameraHardware *this,undefined4 param_2,QualcommCameraHardware *param_3,
          undefined4 param_4)

{
  int iVar1;
  int *piVar2;
  char *pcVar3;
  QualcommCameraHardware *pQVar4;
  
  pQVar4 = param_3;
  iVar1 = (**(code **)(DAT_0001f2f4 + 0x1f2c8))(param_2,param_4);
  if (iVar1 != 0) {
    piVar2 = (int *)__errno();
    pcVar3 = strerror(*piVar2);
    __android_log_print(6,DAT_0001f2f8 + 0x1f2e4,DAT_0001f2fc + 0x1f2e8,param_2,param_3,pcVar3,
                        pQVar4);
    this = param_3;
  }
  return CONCAT44(this,(uint)(iVar1 == 0));
}



// Function: setVpeParameters @ 0001f300

/* android::QualcommCameraHardware::setVpeParameters() */

undefined4 android::QualcommCameraHardware::setVpeParameters(void)

{
  int in_r0;
  int extraout_r1;
  undefined4 uVar1;
  int iVar2;
  int iVar3;
  bool bVar4;
  undefined4 local_14;
  
  iVar2 = DAT_0001f3e0 + 0x1f30c;
  __android_log_print(2,iVar2,DAT_0001f3e4 + 0x1f310);
  uVar1 = *(undefined4 *)(in_r0 + 0xd30);
  __android_log_print(2,iVar2,DAT_0001f3e8 + 0x1f322,*(undefined4 *)(in_r0 + 0xd2c),uVar1);
  __aeabi_idivmod(*(undefined4 *)(in_r0 + 0xd38),0x168);
  local_14 = 0;
  if (extraout_r1 != 0) {
    if (extraout_r1 == 0x5a) {
      local_14 = 1;
    }
    else if (extraout_r1 == 0xb4) {
      local_14 = 6;
    }
    else {
      local_14 = 7;
    }
  }
  iVar2 = *(int *)(in_r0 + 0xd2c);
  if (iVar2 == 0x500) {
    bVar4 = *(int *)(in_r0 + 0xd30) == 0x2d0;
  }
  else {
    if (iVar2 != 800) goto LAB_0001f3a0;
    bVar4 = *(int *)(in_r0 + 0xd30) == 0x1e0;
  }
  if ((bVar4) && (extraout_r1 == 0x10e || extraout_r1 == 0x5a)) {
    uVar1 = *(undefined4 *)(in_r0 + 0xd30);
    __android_log_print(4,DAT_0001f3ec + 0x1f394,DAT_0001f3f0 + 0x1f396,iVar2,uVar1,extraout_r1);
    local_14 = 0;
  }
LAB_0001f3a0:
  iVar3 = DAT_0001f3f4 + 0x1f3ac;
  __android_log_print(2,iVar3,DAT_0001f3f8 + 0x1f3b4,local_14,uVar1);
  iVar2 = native_set_parms();
  __android_log_print(2,iVar3,DAT_0001f3fc + 0x1f3ca,iVar2);
  if (iVar2 == 0) {
    uVar1 = 0x80000000;
  }
  else {
    uVar1 = 0;
  }
  return uVar1;
}



// Function: FUN_0001f400 @ 0001f400

String8 * FUN_0001f400(String8 *param_1,undefined4 param_2,int param_3)

{
  int iVar1;
  
  android::String8::String8(param_1);
  if (0 < param_3) {
    android::String8::append((char *)param_1);
  }
  for (iVar1 = 1; iVar1 < param_3; iVar1 = iVar1 + 1) {
    android::String8::append((char *)param_1);
    android::String8::append((char *)param_1);
  }
  return param_1;
}



// Function: dump @ 0001f448

/* android::QualcommCameraHardware::MemPool::dump(int, android::Vector<android::String16> const&)
   const */

void android::QualcommCameraHardware::MemPool::dump(int param_1,Vector *param_2)

{
  undefined4 uVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  undefined4 uVar4;
  void *local_128;
  char acStack_124 [256];
  int local_24;
  
  local_24 = **(int **)(DAT_0001f52c + 0x1f45a);
  android::String8::String8((String8 *)&local_128);
  snprintf(acStack_124,0xff,(char *)(DAT_0001f530 + 0x1f470));
  android::String8::append((char *)&local_128);
  if (*(int *)(param_1 + 0x20) != 0) {
    snprintf(acStack_124,0xff,(char *)(DAT_0001f534 + 0x1f488));
    android::String8::append((char *)&local_128);
  }
  if (*(int **)(param_1 + 0x18) != (int *)0x0) {
    uVar1 = (**(code **)(**(int **)(param_1 + 0x18) + 0xc))();
    uVar2 = (**(code **)(**(int **)(param_1 + 0x18) + 0x10))();
    uVar3 = (**(code **)(**(int **)(param_1 + 0x18) + 0x14))();
    uVar4 = android::MemoryHeapBase::getDevice();
    snprintf(acStack_124,0xff,(char *)(DAT_0001f538 + 0x1f4c4),uVar1,uVar2,uVar3,uVar4);
    android::String8::append((char *)&local_128);
  }
  snprintf(acStack_124,0xff,(char *)(DAT_0001f53c + 0x1f4e8),*(undefined4 *)(param_1 + 8),
           *(undefined4 *)(param_1 + 0x10),*(undefined4 *)(param_1 + 0x14));
  android::String8::append((char *)&local_128);
  write((int)param_2,local_128,*(int *)((int)local_128 + -0xc) - 1);
  android::String8::~String8((String8 *)&local_128);
  if (local_24 != **(int **)(DAT_0001f540 + 0x1f51a)) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail(0);
  }
  return;
}



// Function: ~MemPool @ 0001f544

/* android::QualcommCameraHardware::MemPool::~MemPool() */

MemPool * __thiscall android::QualcommCameraHardware::MemPool::~MemPool(MemPool *this)

{
  int iVar1;
  int *piVar2;
  
  iVar1 = DAT_0001f5d0 + 0x1f556;
  *(int *)this = *(int *)(DAT_0001f5c8 + 0x1f550 + DAT_0001f5cc) + 8;
  __android_log_print(2,iVar1,DAT_0001f5d4 + 0x1f562,*(undefined4 *)(this + 0x20));
  if ((0 < *(int *)(this + 0x14)) && (iVar1 = *(int *)(this + 0x1c), iVar1 != 0)) {
    iVar1 = iVar1 + *(int *)(iVar1 + -4) * 4;
    while (iVar1 != *(int *)(this + 0x1c)) {
      piVar2 = *(int **)(iVar1 + -4);
      iVar1 = iVar1 + -4;
      if (piVar2 != (int *)0x0) {
        android::RefBase::decStrong((void *)((int)piVar2 + *(int *)(*piVar2 + -0xc)));
      }
    }
    operator_delete__((void *)(iVar1 + -8));
  }
  sp<android::MemoryHeapBase>::clear((sp<android::MemoryHeapBase> *)(this + 0x18));
  __android_log_print(2,DAT_0001f5d8 + 0x1f5b2,DAT_0001f5dc + 0x1f5b6,*(undefined4 *)(this + 0x20));
  sp<android::MemoryHeapPmem>::~sp((sp<android::MemoryHeapPmem> *)(this + 0x18));
  android::RefBase::~RefBase((RefBase *)this);
  return this;
}



// Function: ~MemPool @ 0001f5e0

/* android::QualcommCameraHardware::MemPool::~MemPool() */

MemPool * __thiscall android::QualcommCameraHardware::MemPool::~MemPool(MemPool *this)

{
  ~MemPool(this);
  operator_delete(this);
  return this;
}



// Function: FUN_0001f5f4 @ 0001f5f4

MemPool * FUN_0001f5f4(MemPool *param_1)

{
  *(int *)param_1 = *(int *)(DAT_0001f610 + 0x1f600 + DAT_0001f614) + 8;
  android::QualcommCameraHardware::MemPool::~MemPool(param_1);
  return param_1;
}



// Function: FUN_0001f618 @ 0001f618

void * FUN_0001f618(void *param_1)

{
  FUN_0001f5f4();
  operator_delete(param_1);
  return param_1;
}



// Function: ~DispMemPool @ 0001f62c

/* android::QualcommCameraHardware::DispMemPool::~DispMemPool() */

DispMemPool * __thiscall
android::QualcommCameraHardware::DispMemPool::~DispMemPool(DispMemPool *this)

{
  int iVar1;
  int iVar2;
  
  iVar2 = DAT_0001f670 + 0x1f63e;
  *(int *)this = *(int *)(DAT_0001f668 + 0x1f638 + DAT_0001f66c) + 8;
  __android_log_print(2,iVar2,DAT_0001f674 + 0x1f64a);
  iVar1 = DAT_0001f678;
  *(undefined4 *)(this + 0x24) = 0xffffffff;
  __android_log_print(2,iVar2,iVar1 + 0x1f65a);
  MemPool::~MemPool((MemPool *)this);
  return this;
}



// Function: ~DispMemPool @ 0001f67c

/* android::QualcommCameraHardware::DispMemPool::~DispMemPool() */

DispMemPool * __thiscall
android::QualcommCameraHardware::DispMemPool::~DispMemPool(DispMemPool *this)

{
  ~DispMemPool(this);
  operator_delete(this);
  return this;
}



// Function: ~MMCameraDL @ 0001f690

/* android::QualcommCameraHardware::MMCameraDL::~MMCameraDL() */

MMCameraDL * __thiscall android::QualcommCameraHardware::MMCameraDL::~MMCameraDL(MMCameraDL *this)

{
  int iVar1;
  int iVar2;
  
  iVar1 = DAT_0001f6f0;
  iVar2 = DAT_0001f6e8 + 0x1f69c;
  *(int *)this = *(int *)(iVar2 + DAT_0001f6ec) + 8;
  __android_log_print(2,iVar1 + 0x1f6a8,DAT_0001f6f4 + 0x1f6ae);
  (*(code *)**(undefined4 **)(iVar2 + DAT_0001f6f8))();
  if (*(int *)(this + 8) != 0) {
    dlclose();
    __android_log_print(2,iVar1 + 0x1f6a8,DAT_0001f6fc + 0x1f6ca);
  }
  iVar2 = DAT_0001f704;
  iVar1 = DAT_0001f700;
  *(undefined4 *)(this + 8) = 0;
  __android_log_print(2,iVar1 + 0x1f6da,iVar2 + 0x1f6dc);
  android::RefBase::~RefBase((RefBase *)this);
  return this;
}



// Function: ~MMCameraDL @ 0001f708

/* android::QualcommCameraHardware::MMCameraDL::~MMCameraDL() */

MMCameraDL * __thiscall android::QualcommCameraHardware::MMCameraDL::~MMCameraDL(MMCameraDL *this)

{
  ~MMCameraDL(this);
  operator_delete(this);
  return this;
}



// Function: FUN_0001f71c @ 0001f71c

undefined4 FUN_0001f71c(undefined4 *param_1,int param_2,char *param_3)

{
  int iVar1;
  int iVar2;
  
  if (param_3 != (char *)0x0) {
    for (iVar2 = 0; iVar2 < param_2; iVar2 = iVar2 + 1) {
      iVar1 = strcmp((char *)*param_1,param_3);
      if (iVar1 == 0) {
        return param_1[1];
      }
      param_1 = param_1 + 2;
    }
  }
  return 0xffffffff;
}



// Function: do_compare @ 0001f74c

/* android::SortedVector<android::key_value_pair_t<android::String8, android::String8>
   >::do_compare(void const*, void const*) const */

int __thiscall
android::SortedVector<android::key_value_pair_t<android::String8,android::String8>>::do_compare
          (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *this,
          void *param_1,void *param_2)

{
  int iVar1;
  int iVar2;
  char *__s1;
  char *__s2;
  
  __s1 = *(char **)param_2;
  __s2 = *(char **)param_1;
  iVar1 = strcmp(__s1,__s2);
  iVar2 = strcmp(__s2,__s1);
  return (iVar2 >> 0x1f) - (iVar1 >> 0x1f);
}



// Function: FUN_0001f76c @ 0001f76c

undefined4
FUN_0001f76c(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4,
            undefined4 param_5,undefined4 param_6,undefined4 param_7,undefined4 param_8,
            undefined1 param_9,char param_10,char param_11)

{
  undefined4 uVar1;
  undefined4 local_4c;
  undefined4 local_48;
  undefined4 local_44;
  undefined4 local_40;
  undefined4 local_3c;
  undefined4 local_30;
  undefined4 local_2c;
  undefined4 local_28;
  undefined1 local_24;
  
  memset(&local_4c,0,0x2c);
  local_4c = param_8;
  local_48 = param_5;
  local_40 = param_6;
  local_44 = param_7;
  local_2c = param_3;
  local_28 = param_4;
  if (param_11 != '\0') {
    local_2c = *(undefined4 *)(DAT_0001f7dc + 0x1f7b6);
    local_28 = *(undefined4 *)(DAT_0001f7dc + 0x1f7ba);
  }
  if (param_10 == '\0') {
    uVar1 = 0xc;
  }
  else {
    uVar1 = 0xb;
  }
  local_24 = param_9;
  local_3c = param_1;
  local_30 = param_4;
  FUN_0001ef80(uVar1,&local_4c);
  return 1;
}



// Function: ~PmemPool @ 0001f7e0

/* android::QualcommCameraHardware::PmemPool::~PmemPool() */

PmemPool * __thiscall android::QualcommCameraHardware::PmemPool::~PmemPool(PmemPool *this)

{
  int iVar1;
  undefined4 uVar2;
  int iVar3;
  char *__s2;
  int iVar4;
  int iVar5;
  undefined4 uVar6;
  undefined4 uVar7;
  undefined4 uVar8;
  undefined4 uVar9;
  
  *(int *)this = *(int *)(DAT_0001f8d4 + 0x1f7e8 + DAT_0001f8d8) + 8;
  __android_log_print(4,DAT_0001f8dc + 0x1f800,DAT_0001f8e0 + 0x1f804,DAT_0001f8e4 + 0x1f806,
                      *(undefined4 *)(this + 0x20));
  if (*(int *)(this + 0x18) != 0) {
    __s2 = *(char **)(this + 0x20);
    iVar1 = strcmp((char *)(DAT_0001f8e8 + 0x1f81a),__s2);
    if (iVar1 != 0) {
      iVar5 = *(int *)(this + 0x10);
      iVar4 = 0;
      iVar1 = strcmp((char *)(DAT_0001f8ec + 0x1f82c),__s2);
      if (iVar1 == 0) {
        iVar5 = 6;
      }
      for (; iVar4 < iVar5; iVar4 = iVar4 + 1) {
        uVar9 = *(undefined4 *)(this + 8);
        uVar8 = *(undefined4 *)(this + 0x14);
        uVar7 = *(undefined4 *)(this + 0x2c);
        uVar6 = *(undefined4 *)(this + 0x30);
        uVar2 = (**(code **)(**(int **)(this + 0x18) + 8))();
        iVar3 = *(int *)(this + 0xc);
        iVar1 = FUN_0001de20((int)*(int **)(this + 0x18) + *(int *)(**(int **)(this + 0x18) + -0xc))
        ;
        FUN_0001f76c(uVar9,uVar8,uVar7,uVar6,uVar2,iVar3 * iVar4,
                     *(int *)(this + 0xc) * iVar4 + iVar1,*(undefined4 *)(this + 0x28),0,0,0);
      }
    }
  }
  if (*(void **)(this + 0x44) != (void *)0x0) {
    android::RefBase::decStrong(*(void **)(this + 0x44));
    *(undefined4 *)(this + 0x44) = 0;
  }
  __android_log_print(4,DAT_0001f8f0 + 0x1f8b4,DAT_0001f8f4 + 0x1f8b6,DAT_0001f8f8 + 0x1f8ba,
                      *(undefined4 *)(this + 0x20));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x44));
  MemPool::~MemPool((MemPool *)this);
  return this;
}



// Function: ~PmemPool @ 0001f8fc

/* android::QualcommCameraHardware::PmemPool::~PmemPool() */

PmemPool * __thiscall android::QualcommCameraHardware::PmemPool::~PmemPool(PmemPool *this)

{
  ~PmemPool(this);
  operator_delete(this);
  return this;
}



// Function: completeInitialization @ 0001f910

/* android::QualcommCameraHardware::MemPool::completeInitialization() */

void android::QualcommCameraHardware::MemPool::completeInitialization(void)

{
  int in_r0;
  undefined4 *puVar1;
  undefined4 *puVar2;
  MemoryBase *this;
  int iVar3;
  int iVar4;
  int *local_1c;
  
  if (0 < *(int *)(in_r0 + 0x14)) {
    __android_log_print(6,DAT_0001f9bc + 0x1f926,DAT_0001f9c0 + 0x1f92a,
                        *(undefined4 *)(in_r0 + 0x10));
    iVar4 = *(int *)(in_r0 + 0x10);
    puVar1 = operator_new__((iVar4 + 2) * 4);
    *puVar1 = 4;
    puVar1[1] = iVar4;
    puVar2 = puVar1 + 1;
    for (iVar3 = 0; iVar3 != iVar4; iVar3 = iVar3 + 1) {
      puVar2 = puVar2 + 1;
      *puVar2 = 0;
    }
    *(undefined4 **)(in_r0 + 0x1c) = puVar1 + 2;
    for (iVar3 = 0; iVar3 < *(int *)(in_r0 + 0x10); iVar3 = iVar3 + 1) {
      iVar4 = *(int *)(in_r0 + 0x1c);
      local_1c = *(int **)(in_r0 + 0x18);
      if (local_1c != (int *)0x0) {
        local_1c = (int *)((int)local_1c + *(int *)(*local_1c + -0xc));
      }
      if (local_1c != (int *)0x0) {
        android::RefBase::incStrong((void *)((int)local_1c + *(int *)(*local_1c + -0xc)));
      }
      this = operator_new(0x24);
      android::MemoryBase::MemoryBase
                (this,(sp *)&local_1c,*(int *)(in_r0 + 0xc) * iVar3,*(uint *)(in_r0 + 0x14));
      sp<android::MemoryBase>::operator=((sp<android::MemoryBase> *)(iVar4 + iVar3 * 4),this);
      if (local_1c != (int *)0x0) {
        android::RefBase::decStrong((void *)((int)local_1c + *(int *)(*local_1c + -0xc)));
      }
    }
  }
  return;
}



// Function: MemPool @ 0001f9c4

/* android::QualcommCameraHardware::MemPool::MemPool(int, int, int, char const*) */

MemPool * __thiscall
android::QualcommCameraHardware::MemPool::MemPool
          (MemPool *this,int param_1,int param_2,int param_3,char *param_4)

{
  int iVar1;
  uint uVar2;
  int iVar3;
  int *piVar4;
  
  iVar1 = DAT_0001fa0c;
  android::RefBase::RefBase((RefBase *)this);
  iVar3 = *(int *)(iVar1 + 0x1f9e0 + DAT_0001fa10);
  piVar4 = *(int **)(iVar1 + 0x1f9e0 + DAT_0001fa14);
  *(int *)(this + 8) = param_1;
  *(undefined4 *)(this + 0x18) = 0;
  *(undefined4 *)(this + 0x1c) = 0;
  *(char **)(this + 0x20) = param_4;
  *(int *)this = iVar3 + 8;
  *(int *)(this + 0x10) = param_2;
  *(int *)(this + 0x14) = param_3;
  uVar2 = *piVar4 - 1;
  *(uint *)(this + 0xc) = param_1 + uVar2 & ~uVar2;
  return this;
}



// Function: AshmemPool @ 0001fa18

/* android::QualcommCameraHardware::AshmemPool::AshmemPool(int, int, int, char const*) */

AshmemPool * __thiscall
android::QualcommCameraHardware::AshmemPool::AshmemPool
          (AshmemPool *this,int param_1,int param_2,int param_3,char *param_4)

{
  int iVar1;
  MemoryHeapBase *this_00;
  int iVar2;
  uint uVar3;
  int iVar4;
  
  iVar1 = DAT_0001fa90;
  iVar4 = param_3;
  MemPool::MemPool((MemPool *)this,param_1,param_2,param_3,param_4);
  iVar2 = DAT_0001fa98 + 0x1fa44;
  *(int *)this = *(int *)(iVar1 + 0x1fa38 + DAT_0001fa94) + 8;
  __android_log_print(2,iVar2,DAT_0001fa9c + 0x1fa52,*(undefined4 *)(this + 0x20),param_2,param_3,
                      param_1,iVar4);
  uVar3 = **(int **)(iVar1 + 0x1fa38 + DAT_0001faa0) - 1;
  this_00 = operator_new(0x38);
  android::MemoryHeapBase::MemoryHeapBase(this_00,param_1 * param_2 + uVar3 & ~uVar3,0,(char *)0x0);
  sp<android::MemoryHeapBase>::operator=((sp<android::MemoryHeapBase> *)(this + 0x18),this_00);
  MemPool::completeInitialization();
  return this;
}



// Function: DispMemPool @ 0001faa4

/* android::QualcommCameraHardware::DispMemPool::DispMemPool(int, int, int, int, char const*) */

DispMemPool * __thiscall
android::QualcommCameraHardware::DispMemPool::DispMemPool
          (DispMemPool *this,int param_1,int param_2,int param_3,int param_4,char *param_5)

{
  int iVar1;
  int iVar2;
  
  iVar1 = DAT_0001facc;
  MemPool::MemPool((MemPool *)this,param_2,param_3,param_4,param_5);
  iVar2 = DAT_0001fad0;
  *(int *)(this + 0x24) = param_1;
  *(int *)this = *(int *)(iVar1 + 0x1fac0 + iVar2) + 8;
  return this;
}



// Function: MMCameraDL @ 0001fad4

/* android::QualcommCameraHardware::MMCameraDL::MMCameraDL() */

MMCameraDL * __thiscall android::QualcommCameraHardware::MMCameraDL::MMCameraDL(MMCameraDL *this)

{
  undefined4 uVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  
  iVar3 = DAT_0001fb24;
  android::RefBase::RefBase((RefBase *)this);
  iVar4 = DAT_0001fb30;
  iVar2 = DAT_0001fb2c + 0x1faea;
  *(int *)this = *(int *)(iVar3 + 0x1fae4 + DAT_0001fb28) + 8;
  iVar4 = iVar4 + 0x1faf4;
  __android_log_print(2,iVar4,iVar2);
  iVar3 = DAT_0001fb34;
  *(undefined4 *)(this + 8) = 0;
  uVar1 = dlopen(iVar3 + 0x1fb02);
  iVar3 = DAT_0001fb38 + 0x1fb0c;
  *(undefined4 *)(this + 8) = uVar1;
  __android_log_print(2,iVar4,iVar3,uVar1);
  __android_log_print(2,iVar4,DAT_0001fb3c + 0x1fb1e);
  return this;
}



// Function: getInstance @ 0001fb40

/* android::QualcommCameraHardware::MMCameraDL::getInstance() */

MMCameraDL * __thiscall android::QualcommCameraHardware::MMCameraDL::getInstance(MMCameraDL *this)

{
  int iVar1;
  MMCameraDL *this_00;
  undefined4 uVar2;
  undefined4 *puVar3;
  int iVar4;
  int *piVar5;
  undefined4 uVar6;
  
  iVar4 = DAT_0001fbd0 + 0x1fb4e;
  pthread_mutex_lock(*(pthread_mutex_t **)(iVar4 + DAT_0001fbd4));
  iVar1 = DAT_0001fbd8;
  *(undefined4 *)this = 0;
  piVar5 = *(int **)(iVar4 + iVar1);
  if ((*piVar5 != 0) &&
     (iVar1 = android::RefBase::weakref_type::attemptIncStrong((void *)piVar5[1]), iVar1 != 0)) {
    *(int *)this = *piVar5;
  }
  if (*(int *)this == 0) {
    this_00 = operator_new(0xc);
    MMCameraDL(this_00);
    if (this_00 != (MMCameraDL *)0x0) {
      android::RefBase::incStrong(this_00);
    }
    if (*(void **)this != (void *)0x0) {
      android::RefBase::decStrong(*(void **)this);
    }
    *(MMCameraDL **)this = this_00;
    uVar2 = 0;
    if (this_00 != (MMCameraDL *)0x0) {
      uVar2 = android::RefBase::createWeak(this_00);
    }
    uVar6 = *(undefined4 *)this;
    if (**(int **)(iVar4 + DAT_0001fbd8) != 0) {
      android::RefBase::weakref_type::decWeak((void *)(*(int **)(iVar4 + DAT_0001fbd8))[1]);
    }
    puVar3 = *(undefined4 **)(iVar4 + DAT_0001fbd8);
    *puVar3 = uVar6;
    puVar3[1] = uVar2;
  }
  pthread_mutex_unlock(*(pthread_mutex_t **)(iVar4 + DAT_0001fbd4));
  return this;
}



// Function: PmemPool @ 0001fbdc

/* android::QualcommCameraHardware::PmemPool::PmemPool(char const*, int, int, int, int, int, int,
   int, char const*) */

PmemPool * __thiscall
android::QualcommCameraHardware::PmemPool::PmemPool
          (PmemPool *this,char *param_1,int param_2,int param_3,int param_4,int param_5,int param_6,
          int param_7,int param_8,char *param_9)

{
  ulong __request;
  int iVar1;
  MemoryHeapBase *this_00;
  int iVar2;
  MemoryHeapPmem *this_01;
  int *piVar3;
  char *pcVar4;
  undefined4 *puVar5;
  undefined4 uVar6;
  undefined4 uVar7;
  int iVar8;
  undefined4 uVar9;
  undefined4 uVar10;
  int iVar11;
  int iVar12;
  undefined4 uVar13;
  undefined4 uVar14;
  bool bVar15;
  int iVar16;
  int iVar17;
  MemoryHeapPmem *local_34;
  int *local_30;
  MMCameraDL aMStack_2c [8];
  
  iVar2 = DAT_0001fef0;
  MemPool::MemPool((MemPool *)this,param_4,param_5,param_6,param_9);
  iVar1 = *(int *)(iVar2 + 0x1fc0e + DAT_0001fef4);
  *(undefined4 *)(this + 0x44) = 0;
  iVar2 = DAT_0001fefc;
  iVar16 = DAT_0001fef8 + 0x1fc22;
  *(int *)this = iVar1 + 8;
  *(int *)(this + 0x2c) = param_7;
  *(int *)(this + 0x30) = param_8;
  *(int *)(this + 0x28) = param_3;
  __android_log_print(4,iVar16,iVar2 + 0x1fc26,*(undefined4 *)(this + 0x20),param_1,param_5,param_6,
                      param_4);
  MMCameraDL::getInstance(aMStack_2c);
  sp<android::QualcommCameraHardware::MMCameraDL>::operator=
            ((sp<android::QualcommCameraHardware::MMCameraDL> *)(this + 0x44),(sp *)aMStack_2c);
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)aMStack_2c);
  *(int *)(this + 0x38) = *(int *)(this + 0xc) * param_5;
  this_00 = operator_new(0x38);
  android::MemoryHeapBase::MemoryHeapBase(this_00,param_1,*(uint *)(this + 0x38),param_2);
  sp<android::MemoryHeapBase>::sp((sp<android::MemoryHeapBase> *)&local_30,this_00);
  iVar2 = (**(code **)(*local_30 + 8))();
  if (iVar2 < 0) {
    __android_log_print(6,iVar16,DAT_0001ff00 + 0x1fc9a,param_1);
    sp<android::MemoryHeapBase>::clear((sp<android::MemoryHeapBase> *)&local_30);
    goto LAB_0001feda;
  }
  this_01 = operator_new(0x54);
  android::MemoryHeapPmem::MemoryHeapPmem(this_01,(sp *)&local_30,param_2);
  local_34 = this_01;
  if (this_01 != (MemoryHeapPmem *)0x0) {
    android::RefBase::incStrong(this_01 + *(int *)(*(int *)this_01 + -0x10));
  }
  iVar2 = (**(code **)(*(int *)local_34 + 8))();
  if (iVar2 < 0) {
    __android_log_print(6,DAT_0001ff3c + 0x1feba,DAT_0001ff40 + 0x1febc,param_1);
LAB_0001febe:
    __android_log_print(4,DAT_0001ff44 + 0x1feca,DAT_0001ff48 + 0x1fece,DAT_0001ff4c + 0x1fed0,
                        *(undefined4 *)(this + 0x20));
  }
  else {
    (**(code **)(*(int *)local_34 + 0x20))();
    sp<android::MemoryHeapBase>::clear((sp<android::MemoryHeapBase> *)&local_30);
    sp<android::MemoryHeapBase>::operator=
              ((sp<android::MemoryHeapBase> *)(this + 0x18),(sp *)&local_34);
    if (local_34 != (MemoryHeapPmem *)0x0) {
      android::RefBase::decStrong(local_34 + *(int *)(*(int *)local_34 + -0x10));
      local_34 = (MemoryHeapPmem *)0x0;
    }
    iVar2 = (**(code **)(**(int **)(this + 0x18) + 8))();
    __request = DAT_0001feec;
    *(int *)(this + 0x24) = iVar2;
    iVar2 = ioctl(iVar2,__request,this + 0x3c);
    if (iVar2 == 0) {
      uVar6 = *(undefined4 *)(this + 0x40);
      iVar17 = DAT_0001ff0c + 0x1fd5c;
      __android_log_print(2,iVar17,DAT_0001ff10 + 0x1fd60,param_1,*(undefined4 *)(this + 0x24),uVar6
                         );
      uVar14 = *(undefined4 *)(this + 0xc);
      __android_log_print(3,iVar17,DAT_0001ff14 + 0x1fd76,*(undefined4 *)(this + 8),uVar14);
      pcVar4 = *(char **)(this + 0x20);
      iVar16 = strcmp((char *)(DAT_0001ff18 + 0x1fd84),pcVar4);
      iVar1 = DAT_0001ff24;
      iVar2 = DAT_0001ff20;
      if (iVar16 != 0) {
        iVar16 = strcmp((char *)(DAT_0001ff1c + 0x1fd9a),pcVar4);
        if (iVar16 == 0) {
          param_5 = 6;
        }
        __android_log_print(3,iVar17,DAT_0001ff28 + 0x1fdaa,param_5,uVar14,uVar6);
        iVar17 = DAT_0001ff2c;
        pcVar4 = (char *)(DAT_0001ff30 + 0x1fdc6);
        iVar12 = DAT_0001ff34 + 0x1fdca;
        uVar6 = 0;
        for (iVar16 = 0; iVar16 < param_5; iVar16 = iVar16 + 1) {
          if (param_3 == 0xe) {
            bVar15 = iVar16 < 3;
            if (*pcVar4 != '\0') {
              if (iVar16 == *(int *)(DAT_0001ff38 + 0x1fde8) + -1) {
                param_3 = 0x10;
                bVar15 = true;
              }
              else {
                param_3 = 0xe;
              }
            }
            __android_log_print(2,iVar12,iVar17 + 0x1fe02,bVar15);
            if (param_3 == 0xf) {
LAB_0001fe28:
              if ((*(int *)(iVar2 + 0x1fda8) == 4) && (*(int *)(iVar1 + 0x1fdac) != 3)) {
                param_3 = 0xf;
                uVar6 = 1;
              }
              else {
                param_3 = 0xf;
              }
            }
          }
          else {
            if (param_3 == 0xf) {
              bVar15 = iVar16 < 3;
              goto LAB_0001fe28;
            }
            if (param_3 - 3U < 2) {
              bVar15 = iVar16 < 3;
            }
            else {
              bVar15 = true;
            }
          }
          uVar14 = *(undefined4 *)(this + 0x30);
          uVar10 = *(undefined4 *)(this + 0x14);
          uVar13 = *(undefined4 *)(this + 0x2c);
          uVar9 = *(undefined4 *)(this + 8);
          uVar7 = (**(code **)(**(int **)(this + 0x18) + 8))();
          iVar11 = *(int *)(this + 0xc);
          iVar8 = FUN_0001de20((int)*(int **)(this + 0x18) +
                               *(int *)(**(int **)(this + 0x18) + -0xc));
          FUN_0001f76c(uVar9,uVar10,uVar13,uVar14,uVar7,iVar11 * iVar16,
                       *(int *)(this + 0xc) * iVar16 + iVar8,param_3,bVar15,1,uVar6);
        }
      }
      MemPool::completeInitialization();
      goto LAB_0001febe;
    }
    piVar3 = (int *)__errno();
    pcVar4 = strerror(*piVar3);
    puVar5 = (undefined4 *)__errno();
    __android_log_print(6,DAT_0001ff04 + 0x1fd38,DAT_0001ff08 + 0x1fd3a,param_1,pcVar4,*puVar5);
    sp<android::MemoryHeapBase>::clear((sp<android::MemoryHeapBase> *)(this + 0x18));
  }
  sp<android::MemoryHeapPmem>::~sp((sp<android::MemoryHeapPmem> *)&local_34);
LAB_0001feda:
  sp<android::MemoryHeapPmem>::~sp((sp<android::MemoryHeapPmem> *)&local_30);
  return this;
}



// Function: PmemPool @ 0001ff50

/* android::QualcommCameraHardware::PmemPool::PmemPool(char const*, int, int, int, int, int, int,
   int, char const*) */

PmemPool * __thiscall
android::QualcommCameraHardware::PmemPool::PmemPool
          (PmemPool *this,char *param_1,int param_2,int param_3,int param_4,int param_5,int param_6,
          int param_7,int param_8,char *param_9)

{
  ulong __request;
  int iVar1;
  MemoryHeapBase *this_00;
  int iVar2;
  MemoryHeapPmem *this_01;
  int *piVar3;
  char *pcVar4;
  undefined4 *puVar5;
  undefined4 uVar6;
  int iVar7;
  undefined4 uVar8;
  int iVar9;
  undefined4 uVar10;
  undefined4 uVar11;
  undefined4 uVar12;
  bool bVar13;
  int iVar14;
  int iVar15;
  int iVar16;
  undefined4 local_50;
  MemoryHeapPmem *local_34;
  int *local_30;
  MMCameraDL aMStack_2c [8];
  
  iVar2 = DAT_00020268;
  MemPool::MemPool((MemPool *)this,param_4,param_5,param_6,param_9);
  iVar1 = *(int *)(iVar2 + 0x1ff82 + DAT_0002026c);
  *(undefined4 *)(this + 0x44) = 0;
  iVar2 = DAT_00020274;
  iVar14 = DAT_00020270 + 0x1ff96;
  *(int *)this = iVar1 + 8;
  *(int *)(this + 0x2c) = param_7;
  *(int *)(this + 0x30) = param_8;
  *(int *)(this + 0x28) = param_3;
  __android_log_print(4,iVar14,iVar2 + 0x1ff9a,*(undefined4 *)(this + 0x20),param_1,param_5,param_6,
                      param_4);
  MMCameraDL::getInstance(aMStack_2c);
  sp<android::QualcommCameraHardware::MMCameraDL>::operator=
            ((sp<android::QualcommCameraHardware::MMCameraDL> *)(this + 0x44),(sp *)aMStack_2c);
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)aMStack_2c);
  *(int *)(this + 0x38) = *(int *)(this + 0xc) * param_5;
  this_00 = operator_new(0x38);
  android::MemoryHeapBase::MemoryHeapBase(this_00,param_1,*(uint *)(this + 0x38),param_2);
  sp<android::MemoryHeapBase>::sp((sp<android::MemoryHeapBase> *)&local_30,this_00);
  iVar2 = (**(code **)(*local_30 + 8))();
  if (iVar2 < 0) {
    __android_log_print(6,iVar14,DAT_00020278 + 0x2000e,param_1);
    sp<android::MemoryHeapBase>::clear((sp<android::MemoryHeapBase> *)&local_30);
    goto LAB_00020252;
  }
  this_01 = operator_new(0x54);
  android::MemoryHeapPmem::MemoryHeapPmem(this_01,(sp *)&local_30,param_2);
  local_34 = this_01;
  if (this_01 != (MemoryHeapPmem *)0x0) {
    android::RefBase::incStrong(this_01 + *(int *)(*(int *)this_01 + -0x10));
  }
  iVar2 = (**(code **)(*(int *)local_34 + 8))();
  if (iVar2 < 0) {
    __android_log_print(6,DAT_000202b4 + 0x20232,DAT_000202b8 + 0x20234,param_1);
LAB_00020236:
    __android_log_print(4,DAT_000202bc + 0x20242,DAT_000202c0 + 0x20246,DAT_000202c4 + 0x20248,
                        *(undefined4 *)(this + 0x20));
  }
  else {
    (**(code **)(*(int *)local_34 + 0x20))();
    sp<android::MemoryHeapBase>::clear((sp<android::MemoryHeapBase> *)&local_30);
    sp<android::MemoryHeapBase>::operator=
              ((sp<android::MemoryHeapBase> *)(this + 0x18),(sp *)&local_34);
    if (local_34 != (MemoryHeapPmem *)0x0) {
      android::RefBase::decStrong(local_34 + *(int *)(*(int *)local_34 + -0x10));
      local_34 = (MemoryHeapPmem *)0x0;
    }
    iVar2 = (**(code **)(**(int **)(this + 0x18) + 8))();
    __request = DAT_00020264;
    *(int *)(this + 0x24) = iVar2;
    iVar2 = ioctl(iVar2,__request,this + 0x3c);
    if (iVar2 == 0) {
      uVar6 = *(undefined4 *)(this + 0x40);
      iVar15 = DAT_00020284 + 0x200d0;
      __android_log_print(2,iVar15,DAT_00020288 + 0x200d4,param_1,*(undefined4 *)(this + 0x24),uVar6
                         );
      uVar12 = *(undefined4 *)(this + 0xc);
      __android_log_print(3,iVar15,DAT_0002028c + 0x200ea,*(undefined4 *)(this + 8),uVar12);
      pcVar4 = *(char **)(this + 0x20);
      iVar14 = strcmp((char *)(DAT_00020290 + 0x200f8),pcVar4);
      iVar1 = DAT_0002029c;
      iVar2 = DAT_00020298;
      if (iVar14 != 0) {
        iVar14 = strcmp((char *)(DAT_00020294 + 0x2010e),pcVar4);
        if (iVar14 == 0) {
          param_5 = 6;
        }
        __android_log_print(3,iVar15,DAT_000202a0 + 0x2011e,param_5,uVar12,uVar6);
        iVar15 = DAT_000202a4;
        iVar16 = DAT_000202a8 + 0x20140;
        piVar3 = (int *)(DAT_000202ac + 0x20144);
        local_50 = 0;
        for (iVar14 = 0; iVar14 < param_5; iVar14 = iVar14 + 1) {
          if (param_3 == 0xe) {
            bVar13 = iVar14 < 3;
            if (*(char *)(iVar1 + 0x20120) != '\0') {
              if (iVar14 == *(int *)(DAT_000202b0 + 0x20160) + -1) {
                bVar13 = true;
                param_3 = 0x10;
              }
              else {
                param_3 = 0xe;
              }
            }
            __android_log_print(2,iVar16,iVar15 + 0x2017a,bVar13);
            if (param_3 == 0xf) {
LAB_000201a0:
              if ((*piVar3 == 4) && (*(int *)(iVar2 + 0x2011c) != 3)) {
                param_3 = 0xf;
                local_50 = 1;
              }
              else {
                param_3 = 0xf;
              }
            }
          }
          else {
            if (param_3 == 0xf) {
              bVar13 = iVar14 < 3;
              goto LAB_000201a0;
            }
            if (param_3 - 3U < 2) {
              bVar13 = iVar14 < 3;
            }
            else {
              bVar13 = true;
            }
          }
          uVar6 = *(undefined4 *)(this + 0x30);
          uVar8 = *(undefined4 *)(this + 0x14);
          uVar11 = *(undefined4 *)(this + 0x2c);
          uVar10 = *(undefined4 *)(this + 8);
          uVar12 = (**(code **)(**(int **)(this + 0x18) + 8))();
          iVar9 = *(int *)(this + 0xc);
          iVar7 = FUN_0001de20((int)*(int **)(this + 0x18) +
                               *(int *)(**(int **)(this + 0x18) + -0xc));
          FUN_0001f76c(uVar10,uVar8,uVar11,uVar6,uVar12,iVar9 * iVar14,
                       *(int *)(this + 0xc) * iVar14 + iVar7,param_3,bVar13,1,local_50);
        }
      }
      MemPool::completeInitialization();
      goto LAB_00020236;
    }
    piVar3 = (int *)__errno();
    pcVar4 = strerror(*piVar3);
    puVar5 = (undefined4 *)__errno();
    __android_log_print(6,DAT_0002027c + 0x200ac,DAT_00020280 + 0x200ae,param_1,pcVar4,*puVar5);
    sp<android::MemoryHeapBase>::clear((sp<android::MemoryHeapBase> *)(this + 0x18));
  }
  sp<android::MemoryHeapPmem>::~sp((sp<android::MemoryHeapPmem> *)&local_34);
LAB_00020252:
  sp<android::MemoryHeapPmem>::~sp((sp<android::MemoryHeapPmem> *)&local_30);
  return this;
}



// Function: setExpBracketing @ 000202c8

/* android::QualcommCameraHardware::setExpBracketing(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setExpBracketing
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  char cVar1;
  
  cVar1 = (**(code **)(DAT_00020308 + 0x202e0))(0x3c);
  if ((cVar1 == '\0') && (*(int *)(this + 0xd64) != 0)) {
    __android_log_print(4,DAT_0002030c + 0x202ee,DAT_00020310 + 0x202f0);
  }
  else {
    android::CameraParameters::get((char *)param_1);
    this[0xd91] = (QualcommCameraHardware)0x0;
  }
  return 0;
}



// Function: setDIS @ 00020318

/* android::QualcommCameraHardware::setDIS() */

undefined4 __thiscall android::QualcommCameraHardware::setDIS(QualcommCameraHardware *this)

{
  char *__s1;
  undefined4 uVar1;
  int iVar2;
  uint uVar3;
  uint local_20;
  undefined4 local_1c;
  undefined4 local_18;
  uint local_14;
  
  iVar2 = DAT_000203dc + 0x20324;
  __android_log_print(2,iVar2,DAT_000203e0 + 0x2032a);
  __android_log_print(2,iVar2,DAT_000203e4 + 0x2033c,this[0xd34]);
  if (*(int *)(DAT_000203e8 + 0x2034c) == 6) {
    uVar3 = *(int *)(this + 0xd2c) * *(int *)(this + 0xd30) + 0x7ffU & 0xfffff800;
  }
  else {
    uVar3 = *(int *)(this + 0xd2c) * *(int *)(this + 0xd30) + 3U & 0xfffffffc;
  }
  local_20 = (uint)(byte)this[0xd34];
  __s1 = (char *)android::CameraParameters::get((char *)(this + 0x18));
  if ((__s1 != (char *)0x0) && (iVar2 = strcmp(__s1,*(char **)(DAT_000203f0 + 0x20386)), iVar2 != 0)
     ) {
    __android_log_print(4,DAT_000203f4 + 0x20398,DAT_000203f8 + 0x2039a,DAT_000203fc + 0x2039c);
    local_20 = 0;
  }
  local_1c = *(undefined4 *)(this + 0xd2c);
  local_18 = *(undefined4 *)(this + 0xd30);
  local_14 = uVar3;
  iVar2 = native_set_parms(this,0x1b,0x10,&local_20);
  __android_log_print(2,DAT_00020400 + 0x203c4,DAT_00020404 + 0x203c6,iVar2);
  if (iVar2 == 0) {
    uVar1 = 0x80000000;
  }
  else {
    uVar1 = 0;
  }
  return uVar1;
}



// Function: setPictureFormat @ 00020408

/* android::QualcommCameraHardware::setPictureFormat(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setPictureFormat
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  char *pcVar4;
  
  pcVar4 = *(char **)(DAT_00020454 + 0x20414);
  iVar1 = android::CameraParameters::get((char *)param_1);
  uVar3 = 0;
  if (iVar1 != 0) {
    iVar2 = FUN_0001f71c(DAT_00020458 + 0x20428,2,iVar1);
    if (iVar2 == -1) {
      __android_log_print(6,DAT_0002045c + 0x2044a,DAT_00020460 + 0x2044c,iVar1);
      uVar3 = 0xffffffea;
    }
    else {
      android::CameraParameters::set((char *)(this + 0x18),pcVar4);
      uVar3 = 0;
    }
  }
  return uVar3;
}



// Function: setFocusMode @ 00020464

/* android::QualcommCameraHardware::setFocusMode(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setFocusMode
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  char *pcVar1;
  char *__s2;
  int iVar2;
  int iVar3;
  int iVar4;
  QualcommCameraHardware *pQVar5;
  
  iVar4 = DAT_00020510 + 0x20472;
  pQVar5 = this + 0x18;
  pcVar1 = (char *)android::CameraParameters::get((char *)param_1);
  __s2 = (char *)android::CameraParameters::get((char *)pQVar5);
  if ((pcVar1 != (char *)0x0) &&
     ((__s2 == (char *)0x0 || (iVar2 = strcmp(pcVar1,__s2), iVar2 != 0)))) {
    iVar2 = DAT_0002051c;
    iVar3 = FUN_0001f71c(DAT_00020518 + 0x204a8,6,pcVar1);
    __android_log_print(2,iVar2 + 0x204b0,DAT_00020520 + 0x204b6,pcVar1);
    if (iVar3 == -1) {
      __android_log_print(6,iVar2 + 0x204b0,DAT_00020530 + 0x20502,pcVar1);
      return 0xffffffea;
    }
    android::CameraParameters::set((char *)pQVar5,*(char **)(iVar4 + DAT_00020514));
    if (iVar3 == 1) {
      pcVar1 = *(char **)(iVar4 + DAT_00020524);
    }
    else {
      pcVar1 = *(char **)(iVar4 + DAT_00020524);
    }
    android::CameraParameters::set((char *)pQVar5,pcVar1);
    FUN_0001f008(0xb,iVar3,0);
    *(int *)(this + 0x668) = iVar3;
  }
  return 0;
}



// Function: updateFocusDistances @ 00020534

/* android::QualcommCameraHardware::updateFocusDistances(char const*) */

void __thiscall
android::QualcommCameraHardware::updateFocusDistances(QualcommCameraHardware *this,char *param_1)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  int iVar4;
  int iVar5;
  int iVar6;
  double dVar7;
  undefined1 local_54 [4];
  float local_50;
  float local_4c;
  undefined4 local_48;
  char acStack_44 [32];
  int local_24;
  
  iVar4 = DAT_0002065c + 0x20542;
  iVar6 = DAT_00020664 + 0x2054c;
  local_24 = **(int **)(iVar4 + DAT_00020660);
  iVar5 = DAT_00020668 + 0x20554;
  __android_log_print(2,iVar6,DAT_0002066c + 0x2055c,iVar5);
  iVar2 = (**(code **)(DAT_00020670 + 0x20570))(0x25,local_54);
  iVar1 = DAT_00020674;
  if (iVar2 == 0) {
    android::String8::String8((String8 *)&local_48);
    snprintf(acStack_44,0x20,(char *)(DAT_00020678 + 0x20590));
    android::String8::append((char *)&local_48);
    dVar7 = (double)local_50;
    snprintf(acStack_44,0x20,(char *)(iVar1 + 0x20592));
    android::String8::append((char *)&local_48);
    iVar2 = strcmp(param_1,*(char **)(iVar4 + DAT_0002067c));
    if (iVar2 == 0) {
      snprintf(acStack_44,0x20,(char *)(DAT_00020680 + 0x205da),DAT_00020684 + 0x205dc,dVar7);
      uVar3 = (undefined4)((ulonglong)dVar7 >> 0x20);
    }
    else {
      uVar3 = (undefined4)((ulonglong)(double)local_4c >> 0x20);
      snprintf(acStack_44,0x20,(char *)(iVar1 + 0x20592));
    }
    android::String8::append((char *)&local_48);
    __android_log_print(4,DAT_00020688 + 0x2060c,DAT_0002068c + 0x2060e,DAT_00020690 + 0x20612,
                        local_48,uVar3);
    android::CameraParameters::set((char *)(this + 0x18),*(char **)(iVar4 + DAT_00020694));
    android::String8::~String8((String8 *)&local_48);
    uVar3 = 0;
  }
  else {
    __android_log_print(6,iVar6,DAT_00020698 + 0x2063a,iVar5);
    uVar3 = 0xffffffea;
  }
  if (local_24 != **(int **)(iVar4 + DAT_00020660)) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail(uVar3);
  }
  return;
}



// Function: setDenoise @ 0002069c

/* android::QualcommCameraHardware::setDenoise(android::CameraParameters const&) */

undefined8 android::QualcommCameraHardware::setDenoise(CameraParameters *param_1)

{
  char cVar1;
  int iVar2;
  undefined4 uVar3;
  char *in_r1;
  char *pcVar4;
  int local_1c [2];
  
  cVar1 = (**(code **)(DAT_00020730 + 0x206b4))(0x33);
  if (cVar1 == '\0') {
    __android_log_print(6,DAT_00020734 + 0x206ba,DAT_00020738 + 0x206bc);
    uVar3 = 0;
  }
  else {
    pcVar4 = *(char **)(DAT_0002073c + 0x206ca);
    iVar2 = android::CameraParameters::get(in_r1);
    if (iVar2 == 0) {
      __android_log_print(6,DAT_00020744 + 0x20720,DAT_00020748 + 0x20722,DAT_0002074c + 0x20724);
      uVar3 = 0xffffffea;
    }
    else {
      local_1c[0] = FUN_0001f71c(DAT_00020740 + 0x206de,2,iVar2);
      if ((local_1c[0] != -1) && (*(int *)(param_1 + 0xd60) != local_1c[0])) {
        *(int *)(param_1 + 0xd60) = local_1c[0];
        android::CameraParameters::set((char *)(param_1 + 0x18),pcVar4);
        iVar2 = native_set_parms((QualcommCameraHardware *)param_1,0x33,4,local_1c);
        if (iVar2 == 0) {
          uVar3 = 0x80000000;
          goto LAB_0002072e;
        }
      }
      uVar3 = 0;
    }
  }
LAB_0002072e:
  return CONCAT44(param_1,uVar3);
}



// Function: setSceneMode @ 00020750

/* android::QualcommCameraHardware::setSceneMode(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setSceneMode
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  char *pcVar4;
  
  pcVar4 = *(char **)(DAT_000207d4 + 0x2075e);
  iVar1 = android::CameraParameters::get((char *)param_1);
  __android_log_print(2,DAT_000207d8 + 0x2076c,DAT_000207dc + 0x2076e,iVar1);
  if (iVar1 == 0) {
    iVar1 = DAT_000207e4 + 0x207ba;
LAB_000207b8:
    __android_log_print(6,DAT_000207e8 + 0x207c4,DAT_000207ec + 0x207c6,iVar1);
    uVar3 = 0xffffffea;
  }
  else {
    iVar2 = FUN_0001f71c(DAT_000207e0 + 0x20782,0xf,iVar1);
    if (iVar2 != *(int *)(this + 0x664)) {
      if (iVar2 == -1) goto LAB_000207b8;
      android::CameraParameters::set((char *)(this + 0x18),pcVar4);
      iVar1 = FUN_0001f008(1,iVar2,0);
      *(int *)(this + 0x664) = iVar2;
      if (iVar1 == 0) {
        return 0x80000000;
      }
    }
    uVar3 = 0;
  }
  return uVar3;
}



// Function: setSceneDetect @ 000207f0

/* android::QualcommCameraHardware::setSceneDetect(android::CameraParameters const&) */

undefined8 android::QualcommCameraHardware::setSceneDetect(CameraParameters *param_1)

{
  char cVar1;
  int iVar2;
  undefined4 uVar3;
  int iVar4;
  char *in_r1;
  char *pcVar5;
  bool bVar6;
  char *local_1c [2];
  
  local_1c[0] = in_r1;
  iVar2 = supportsSceneDetection();
  iVar4 = DAT_000208d4;
  if (iVar2 == 0) {
LAB_000208ba:
    uVar3 = 0;
  }
  else {
    cVar1 = (**(code **)(DAT_000208d4 + 0x20812))(0x17);
    if ((cVar1 == '\0') && (cVar1 = (**(code **)(iVar4 + 0x20812))(0x18), cVar1 == '\0')) {
      __android_log_print(6,DAT_000208d8 + 0x20824,DAT_000208dc + 0x20826);
      uVar3 = 0;
      goto LAB_000208bc;
    }
    pcVar5 = *(char **)(DAT_000208f0 + 0x208c6);
    iVar4 = android::CameraParameters::get(in_r1);
    if (iVar4 == 0) {
      iVar4 = DAT_000208e4 + 0x208a0;
    }
    else {
      local_1c[0] = (char *)FUN_0001f71c(DAT_000208e0 + 0x20836,2,iVar4);
      if (local_1c[0] != (char *)0xffffffff) {
        android::CameraParameters::set((char *)(param_1 + 0x18),pcVar5);
        iVar4 = native_set_parms((QualcommCameraHardware *)param_1,0x17,4,local_1c);
        iVar2 = native_set_parms((QualcommCameraHardware *)param_1,0x18,4,local_1c);
        if ((iVar4 == 0) || (iVar2 == 0)) {
          bVar6 = (char *)0x1 < local_1c[0];
          local_1c[0] = (char *)(1 - (int)local_1c[0]);
          if (bVar6) {
            local_1c[0] = (char *)0x0;
          }
          iVar4 = native_set_parms((QualcommCameraHardware *)param_1,0x17,4,local_1c);
          iVar2 = native_set_parms((QualcommCameraHardware *)param_1,0x18,4,local_1c);
          if ((iVar4 == 0) || (iVar2 == 0)) {
            uVar3 = 0x80000000;
            goto LAB_000208bc;
          }
        }
        goto LAB_000208ba;
      }
    }
    __android_log_print(6,DAT_000208e8 + 0x208aa,DAT_000208ec + 0x208ac,iVar4);
    uVar3 = 0xffffffea;
  }
LAB_000208bc:
  return CONCAT44(param_1,uVar3);
}



// Function: setRedeyeReduction @ 000208f4

/* android::QualcommCameraHardware::setRedeyeReduction(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setRedeyeReduction
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  char cVar1;
  int iVar2;
  int iVar3;
  undefined4 uVar4;
  char *pcVar5;
  undefined1 local_19 [5];
  
  cVar1 = (**(code **)(DAT_00020990 + 0x2090e))(0x32);
  if (cVar1 == '\0') {
    __android_log_print(4,DAT_00020994 + 0x20914,DAT_00020998 + 0x20916);
    uVar4 = 0;
  }
  else {
    pcVar5 = *(char **)(DAT_0002099c + 0x20924);
    iVar2 = android::CameraParameters::get((char *)param_1);
    if (iVar2 == 0) {
      iVar2 = DAT_000209b0 + 0x2097a;
    }
    else {
      iVar3 = FUN_0001f71c(DAT_000209a0 + 0x20938,2,iVar2);
      if (iVar3 != -1) {
        local_19[0] = (undefined1)iVar3;
        __android_log_print(4,DAT_000209a4 + 0x2094a,DAT_000209a8 + 0x20950,DAT_000209ac + 0x20952,
                            iVar2);
        android::CameraParameters::set((char *)(this + 0x18),pcVar5);
        native_set_parms(this,0x32,1,local_19);
        return 0;
      }
    }
    __android_log_print(6,DAT_000209b4 + 0x20984,DAT_000209b8 + 0x20986,iVar2);
    uVar4 = 0xffffffea;
  }
  return uVar4;
}



// Function: setISOValue @ 000209bc

/* android::QualcommCameraHardware::setISOValue(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setISOValue(QualcommCameraHardware *this,CameraParameters *param_1)

{
  char *__s1;
  char *__s2;
  int iVar1;
  
  __s1 = (char *)android::CameraParameters::get((char *)param_1);
  __s2 = (char *)android::CameraParameters::get((char *)(this + 0x18));
  if (((__s1 != (char *)0x0) && (__s2 != (char *)0x0)) && (iVar1 = strcmp(__s1,__s2), iVar1 != 0)) {
    iVar1 = FUN_0001f71c(DAT_00020a34 + 0x209f4,7,__s1);
    if (iVar1 == -1) {
      __android_log_print(6,DAT_00020a3c + 0x20a26,DAT_00020a40 + 0x20a28,__s1);
    }
    else {
      android::CameraParameters::set((char *)(this + 0x18),*(char **)(DAT_00020a38 + 0x20a06));
      FUN_0001f008(5,iVar1,0);
      *(int *)(this + 0x654) = iVar1;
    }
  }
  return 0;
}



// Function: setFaceDetection @ 00020a44

/* android::QualcommCameraHardware::setFaceDetection(char const*) */

undefined4 __thiscall
android::QualcommCameraHardware::setFaceDetection(QualcommCameraHardware *this,char *param_1)

{
  int iVar1;
  undefined4 uVar2;
  
  iVar1 = supportsFaceDetection();
  if (iVar1 == 0) {
    __android_log_print(4,DAT_00020ac0 + 0x20a5e,DAT_00020ac4 + 0x20a60);
    uVar2 = 0;
  }
  else {
    if (param_1 == (char *)0x0) {
      param_1 = (char *)(DAT_00020ad0 + 0x20aa8);
    }
    else {
      iVar1 = FUN_0001f71c(DAT_00020ac8 + 0x20a72,2,param_1);
      if (iVar1 != -1) {
        pthread_mutex_lock((pthread_mutex_t *)(this + 0x3a0));
        *(int *)(this + 0x398) = iVar1;
        pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3a0));
        android::CameraParameters::set((char *)(this + 0x18),*(char **)(DAT_00020acc + 0x20a9a));
        return 0;
      }
    }
    __android_log_print(6,DAT_00020ad4 + 0x20ab2,DAT_00020ad8 + 0x20ab4,param_1);
    uVar2 = 0xffffffea;
  }
  return uVar2;
}



// Function: setSelectableZoneAf @ 00020adc

/* android::QualcommCameraHardware::setSelectableZoneAf(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setSelectableZoneAf
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  undefined4 uVar2;
  char *pcVar3;
  QualcommCameraHardware *pQVar4;
  CameraParameters *local_14;
  
  if ((this[0xd28] == (QualcommCameraHardware)0x0) ||
     (pQVar4 = this, local_14 = param_1, iVar1 = supportsSelectableZoneAf(), iVar1 == 0)) {
LAB_00020b52:
    uVar2 = 0;
  }
  else {
    pcVar3 = *(char **)(DAT_00020b58 + 0x20afa);
    iVar1 = android::CameraParameters::get((char *)param_1);
    if (iVar1 == 0) {
      iVar1 = DAT_00020b60 + 0x20b3e;
    }
    else {
      local_14 = (CameraParameters *)FUN_0001f71c(DAT_00020b5c + 0x20b0e,4,iVar1);
      if (local_14 != (CameraParameters *)0xffffffff) {
        android::CameraParameters::set((char *)(this + 0x18),pcVar3);
        iVar1 = native_set_parms(this,0x12,4,&local_14);
        if (iVar1 == 0) {
          return 0x80000000;
        }
        goto LAB_00020b52;
      }
    }
    __android_log_print(6,DAT_00020b64 + 0x20b48,DAT_00020b68 + 0x20b4a,iVar1,pQVar4);
    uVar2 = 0xffffffea;
  }
  return uVar2;
}



// Function: setLensshadeValue @ 00020b6c

/* android::QualcommCameraHardware::setLensshadeValue(android::CameraParameters const&) */

undefined8 __thiscall
android::QualcommCameraHardware::setLensshadeValue
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  char cVar1;
  int iVar2;
  int iVar3;
  undefined4 uVar4;
  char *pcVar5;
  QualcommCameraHardware *pQVar6;
  CameraParameters *pCVar7;
  
  pQVar6 = this;
  pCVar7 = param_1;
  cVar1 = (**(code **)(DAT_00020bf0 + 0x20b84))(0x10);
  if (cVar1 == '\0') {
    __android_log_print(4,DAT_00020bf4 + 0x20b8a,DAT_00020bf8 + 0x20b8c);
    uVar4 = 0;
  }
  else {
    pcVar5 = *(char **)(DAT_00020bfc + 0x20b9a);
    iVar2 = android::CameraParameters::get((char *)param_1);
    if (iVar2 == 0) {
      iVar2 = DAT_00020c04 + 0x20bdc;
    }
    else {
      iVar3 = FUN_0001f71c(DAT_00020c00 + 0x20bae,2,iVar2);
      if (iVar3 != -1) {
        android::CameraParameters::set((char *)(this + 0x18),pcVar5);
        native_set_parms(this,0x10,1,&stack0xffffffe7);
        uVar4 = 0;
        goto LAB_00020bee;
      }
    }
    __android_log_print(6,DAT_00020c08 + 0x20be6,DAT_00020c0c + 0x20be8,iVar2,pQVar6,pCVar7);
    uVar4 = 0xffffffea;
  }
LAB_00020bee:
  return CONCAT44(pQVar6,uVar4);
}



// Function: setHDRImaging @ 00020c10

/* android::QualcommCameraHardware::setHDRImaging(android::CameraParameters const&) */

void __thiscall
android::QualcommCameraHardware::setHDRImaging
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  char cVar1;
  int iVar2;
  undefined4 uVar3;
  int iVar4;
  int iVar5;
  char *pcVar6;
  undefined4 local_54;
  int local_50;
  undefined4 local_4c;
  undefined4 local_48;
  int local_24;
  
  iVar5 = DAT_00020cfc + 0x20c1e;
  local_24 = **(int **)(iVar5 + DAT_00020d00);
  cVar1 = (**(code **)(DAT_00020d04 + 0x20c34))(0x3c);
  if ((cVar1 == '\0') && (*(int *)(this + 0xd64) != 0)) {
    __android_log_print(4,DAT_00020d08 + 0x20c48,DAT_00020d0c + 0x20c4a);
    uVar3 = 0;
  }
  else {
    pcVar6 = *(char **)(iVar5 + DAT_00020d2c);
    iVar4 = android::CameraParameters::get((char *)param_1);
    if (iVar4 == 0) {
      iVar4 = DAT_00020d20 + 0x20cb6;
    }
    else {
      iVar2 = FUN_0001f71c(DAT_00020d10 + 0x20c5a,2,iVar4);
      if (iVar2 != -1) {
        memset(&local_54,0,0x30);
        local_4c = 3;
        local_48 = 2;
        local_54 = 1;
        local_50 = iVar2;
        __android_log_print(4,DAT_00020d18 + 0x20c84,DAT_00020d14 + 0x20c80,DAT_00020d1c + 0x20c86,
                            iVar4);
        android::CameraParameters::set((char *)(this + 0x18),pcVar6);
        *(undefined4 *)(this + 0xc) = 1;
        native_set_parms(this,0x3c,0x30,&local_54);
        uVar3 = 0;
        goto LAB_00020cc8;
      }
    }
    __android_log_print(6,DAT_00020d24 + 0x20cc0,DAT_00020d28 + 0x20cc2,iVar4);
    uVar3 = 0xffffffea;
  }
LAB_00020cc8:
  if (local_24 != **(int **)(iVar5 + DAT_00020d00)) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail(uVar3);
  }
  return;
}



// Function: setMCEValue @ 00020d30

/* android::QualcommCameraHardware::setMCEValue(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setMCEValue(QualcommCameraHardware *this,CameraParameters *param_1)

{
  char cVar1;
  int iVar2;
  int iVar3;
  undefined4 uVar4;
  char *pcVar5;
  undefined1 local_19 [5];
  
  cVar1 = (**(code **)(DAT_00020dcc + 0x20d4a))(0x2e);
  if (cVar1 == '\0') {
    __android_log_print(4,DAT_00020dd0 + 0x20d50,DAT_00020dd4 + 0x20d52);
    uVar4 = 0;
  }
  else {
    pcVar5 = *(char **)(DAT_00020dd8 + 0x20d60);
    iVar2 = android::CameraParameters::get((char *)param_1);
    if (iVar2 == 0) {
      iVar2 = DAT_00020dec + 0x20db6;
    }
    else {
      iVar3 = FUN_0001f71c(DAT_00020ddc + 0x20d74,2,iVar2);
      if (iVar3 != -1) {
        local_19[0] = (undefined1)iVar3;
        __android_log_print(4,DAT_00020de0 + 0x20d86,DAT_00020de4 + 0x20d8c,DAT_00020de8 + 0x20d8e,
                            iVar2);
        android::CameraParameters::set((char *)(this + 0x18),pcVar5);
        native_set_parms(this,0x2e,1,local_19);
        return 0;
      }
    }
    __android_log_print(6,DAT_00020df0 + 0x20dc0,DAT_00020df4 + 0x20dc2,iVar2);
    uVar4 = 0xffffffea;
  }
  return uVar4;
}



// Function: setFlash @ 00020df8

/* android::QualcommCameraHardware::setFlash(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setFlash(QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  char *pcVar4;
  QualcommCameraHardware *pQVar5;
  
  pcVar4 = *(char **)(DAT_00020ed0 + 0x20e06);
  pQVar5 = this + 0x18;
  iVar1 = android::CameraParameters::get((char *)param_1);
  android::CameraParameters::get((char *)pQVar5);
  if (iVar1 != 0) {
    iVar3 = DAT_00020ed4 + 0x20e2a;
    __android_log_print(2,iVar3,DAT_00020ed8 + 0x20e2c,iVar1);
    iVar2 = FUN_0001f71c(DAT_00020edc + 0x20e3a,4,iVar1);
    if ((iVar2 != *(int *)(this + 0x638)) && (iVar2 != 2 || *(int *)(this + 0x638) != 3)) {
      if (iVar2 == -1) {
        __android_log_print(6,iVar3,DAT_00020ee8 + 0x20ec2,iVar1);
        return 0xffffffea;
      }
      android::CameraParameters::set((char *)pQVar5,pcVar4);
      *(int *)(this + 0x638) = iVar2;
      iVar1 = FUN_0001f008(0,iVar2,0);
      if (iVar2 != 0 && *(int *)(this + 0xd64) != 0) {
        android::CameraParameters::set((char *)pQVar5,(char *)(DAT_00020ee0 + 0x20ea6));
        *(undefined4 *)(this + 0xc) = 1;
      }
      if (iVar1 == 0) {
        return 0x80000000;
      }
    }
  }
  return 0;
}



// Function: setFirmwareUpdate @ 00020eec

/* android::QualcommCameraHardware::setFirmwareUpdate(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setFirmwareUpdate
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  char *pcVar4;
  
  pcVar4 = (char *)(DAT_00020f64 + 0x20efa);
  iVar1 = android::CameraParameters::get((char *)param_1);
  if (iVar1 != 0) {
    iVar3 = DAT_00020f68 + 0x20f0e;
    __android_log_print(2,iVar3,DAT_00020f6c + 0x20f10,iVar1);
    iVar2 = FUN_0001f71c(DAT_00020f70 + 0x20f1e,4,iVar1);
    if (iVar2 == -1) {
      __android_log_print(6,iVar3,DAT_00020f74 + 0x20f56,iVar1);
      return 0xffffffea;
    }
    android::CameraParameters::set((char *)(this + 0x18),pcVar4);
    iVar1 = FUN_0001f008(0x20,iVar2,0);
    *(int *)(this + 0x6b4) = iVar2;
    if (iVar1 == 0) {
      return 0x80000000;
    }
  }
  return 0;
}



// Function: setWhiteBalance @ 00020f78

/* android::QualcommCameraHardware::setWhiteBalance(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setWhiteBalance
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  undefined4 uVar2;
  int iVar3;
  int iVar4;
  
  iVar3 = *(int *)(this + 0x664);
  iVar4 = DAT_00021024 + 0x20f8a;
  if (iVar3 == 1) {
    iVar3 = android::CameraParameters::get((char *)param_1);
  }
  else {
    iVar1 = DAT_0002102c;
    if ((iVar3 != 10) && ((iVar3 == 0xe || (iVar1 = DAT_00021034, iVar3 == 9)))) {
      iVar1 = DAT_00021030;
    }
    iVar3 = *(int *)(iVar4 + iVar1);
  }
  __android_log_print(2,DAT_00021038 + 0x20fc0,DAT_0002103c + 0x20fc2,iVar3);
  if (iVar3 == 0) {
    iVar3 = DAT_00021044 + 0x2100a;
LAB_00021008:
    __android_log_print(6,DAT_00021048 + 0x21014,DAT_0002104c + 0x21016,iVar3);
    uVar2 = 0xffffffea;
  }
  else {
    iVar1 = FUN_0001f71c(DAT_00021040 + 0x20fd0,5,iVar3);
    if (iVar1 != *(int *)(this + 0x650)) {
      if (iVar1 == -1) goto LAB_00021008;
      android::CameraParameters::set((char *)(this + 0x18),*(char **)(iVar4 + DAT_00021028));
      iVar3 = FUN_0001f008(6,iVar1,0);
      *(int *)(this + 0x650) = iVar1;
      if (iVar3 == 0) {
        return 0x80000000;
      }
    }
    uVar2 = 0;
  }
  return uVar2;
}



// Function: setAutoExposure @ 00021050

/* android::QualcommCameraHardware::setAutoExposure(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setAutoExposure
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  char *__s1;
  char *__s2;
  int iVar1;
  int iVar2;
  
  iVar2 = *(int *)(this + 0x664);
  if (iVar2 == 1) {
    __s1 = (char *)android::CameraParameters::get((char *)param_1);
  }
  else if (iVar2 == 4) {
    if (*(int *)(this + 0x638) == 0) {
      __s1 = (char *)(DAT_0002112c + 0x21094);
    }
    else {
      __s1 = (char *)(DAT_00021128 + 0x2108e);
    }
  }
  else if (iVar2 == 5) {
    __s1 = (char *)(DAT_00021124 + 0x21082);
  }
  else {
    __s1 = (char *)(DAT_00021120 + 0x2107c);
  }
  __s2 = (char *)android::CameraParameters::get((char *)(this + 0x18));
  if ((__s1 != (char *)0x0) &&
     (((__s2 == (char *)0x0 || (iVar2 = strcmp(__s1,__s2), iVar2 != 0)) &&
      (iVar2 = FUN_0001f71c(DAT_00021134 + 0x210bc,3,__s1), iVar2 != *(int *)(this + 0x658))))) {
    if (iVar2 == -1) {
      __android_log_print(6,DAT_00021144 + 0x2110a,DAT_00021148 + 0x2110c,__s1);
      return 0xffffffea;
    }
    android::CameraParameters::set((char *)(this + 0x18),(char *)(DAT_00021138 + 0x210d6));
    __android_log_print(2,DAT_0002113c + 0x210e2,DAT_00021140 + 0x210e6,iVar2);
    iVar1 = FUN_0001f008(0x12,iVar2,0);
    *(int *)(this + 0x658) = iVar2;
    if (iVar1 == 0) {
      return 0x80000000;
    }
  }
  return 0;
}



// Function: setRecordingHint @ 0002114c

/* android::QualcommCameraHardware::setRecordingHint(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setRecordingHint
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  undefined4 uVar2;
  char *pcVar3;
  QualcommCameraHardware *pQVar4;
  CameraParameters *local_14;
  
  pcVar3 = *(char **)(DAT_000211a8 + 0x21158);
  pQVar4 = this;
  local_14 = param_1;
  iVar1 = android::CameraParameters::get((char *)param_1);
  uVar2 = 0;
  if (iVar1 != 0) {
    local_14 = (CameraParameters *)FUN_0001f71c(DAT_000211ac + 0x2116c,2,iVar1);
    if (local_14 == (CameraParameters *)0xffffffff) {
      __android_log_print(6,DAT_000211b0 + 0x2119e,DAT_000211b4 + 0x211a0,iVar1,pQVar4);
      uVar2 = 0xffffffea;
    }
    else {
      native_set_parms(this,0x42,4,&local_14);
      android::CameraParameters::set((char *)(this + 0x18),pcVar3);
      uVar2 = 0;
    }
  }
  return uVar2;
}



// Function: setEffect @ 000211b8

/* android::QualcommCameraHardware::setEffect(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setEffect(QualcommCameraHardware *this,CameraParameters *param_1)

{
  char *__s1;
  char *__s2;
  int iVar1;
  int iVar2;
  
  __s1 = (char *)android::CameraParameters::get((char *)param_1);
  __s2 = (char *)android::CameraParameters::get((char *)(this + 0x18));
  if ((__s1 != (char *)0x0) && ((__s2 == (char *)0x0 || (iVar1 = strcmp(__s1,__s2), iVar1 != 0)))) {
    iVar2 = DAT_00021254 + 0x211f6;
    __android_log_print(6,iVar2,DAT_00021258 + 0x211f8,__s1);
    iVar1 = FUN_0001f71c(DAT_0002125c + 0x21206,5,__s1);
    if (iVar1 == -1) {
      __android_log_print(6,iVar2,DAT_00021264 + 0x21240,__s1);
      return 0xffffffea;
    }
    android::CameraParameters::set((char *)(this + 0x18),*(char **)(DAT_00021260 + 0x2121a));
    *(int *)(this + 0x63c) = iVar1;
    iVar1 = FUN_0001f008(3,iVar1,0);
    if (iVar1 == 0) {
      return 0x80000000;
    }
  }
  return 0;
}



// Function: setAntibanding @ 00021268

/* android::QualcommCameraHardware::setAntibanding(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setAntibanding
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  android::CameraParameters::getInt((char *)param_1);
  android::CameraParameters::getInt((char *)(this + 0x18));
  return 0;
}



// Function: setOrientation @ 0002128c

/* android::QualcommCameraHardware::setOrientation(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setOrientation
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  if ((iVar1 != -1) && (*(int *)(this + 0x6a4) != iVar1)) {
    __android_log_print(6,DAT_000212e4 + 0x212b6,DAT_000212e8 + 0x212b8,iVar1);
    if (iVar1 - 1U < 2) {
      *(int *)(this + 0x6a4) = iVar1;
    }
    else {
      *(undefined4 *)(this + 0x6a4) = 0;
    }
    android::CameraParameters::set((char *)(this + 0x18),DAT_000212ec + 0x212da);
  }
  return 0;
}



// Function: setZoom @ 000212f0

/* android::QualcommCameraHardware::setZoom(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setZoom(QualcommCameraHardware *this,CameraParameters *param_1)

{
  uint uVar1;
  uint uVar2;
  undefined4 uVar3;
  int iVar4;
  int iVar5;
  int iVar6;
  
  iVar6 = *(int *)(DAT_00021388 + 0x212fe);
  uVar1 = android::CameraParameters::getInt((char *)param_1);
  uVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if (uVar2 == uVar1 || uVar1 == 0xffffffff) {
    uVar3 = 0;
  }
  else {
    iVar5 = DAT_0002138c + 0x2133a;
    __android_log_print(2,iVar5,DAT_00021390 + 0x2133c,uVar1);
    iVar4 = android::CameraParameters::getInt((char *)param_1);
    uVar2 = uVar1 >> 0x1f;
    if (iVar4 < (int)uVar1) {
      uVar2 = 1;
    }
    if (uVar2 == 0) {
      android::CameraParameters::set((char *)(this + 0x18),iVar6);
      FUN_0001f008(9,uVar1,0);
      uVar3 = 0;
    }
    else {
      __android_log_print(6,iVar5,DAT_00021398 + 0x21366,uVar1);
      uVar3 = 0xffffffea;
    }
  }
  return uVar3;
}



// Function: setRotation @ 0002139c

/* android::QualcommCameraHardware::setRotation(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setRotation(QualcommCameraHardware *this,CameraParameters *param_1)

{
  uint uVar1;
  undefined4 extraout_r1;
  uint uVar2;
  
  uVar1 = android::CameraParameters::getInt((char *)param_1);
  if (uVar1 != 0xffffffff) {
    uVar2 = 1 - uVar1;
    if (1 < uVar1) {
      uVar2 = 0;
    }
    if (uVar1 == 0x5a) {
      uVar2 = uVar2 | 1;
    }
    if ((uVar2 == 0) && (uVar1 != 0x10e && uVar1 != 0xb4)) {
      __android_log_print(6,DAT_00021418 + 0x21406,DAT_0002141c + 0x21408);
      return 0xffffffea;
    }
    __aeabi_idivmod(uVar1,0x168);
    android::CameraParameters::set((char *)(this + 0x18),*(int *)(DAT_00021414 + 0x213f0));
    *(undefined4 *)(this + 0xd38) = extraout_r1;
  }
  return 0;
}



// Function: setAppShutterSound @ 00021420

/* android::QualcommCameraHardware::setAppShutterSound(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setAppShutterSound
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  
  iVar3 = DAT_00021484 + 0x2142c;
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  iVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if ((iVar2 != iVar1 && iVar1 != -1) && (iVar1 != 0 || iVar2 != -1)) {
    __android_log_print(2,DAT_00021488 + 0x21472,DAT_0002148c + 0x21474,iVar1);
    android::CameraParameters::set((char *)(this + 0x18),iVar3);
  }
  return 0;
}



// Function: setHdrMode @ 00021490

/* android::QualcommCameraHardware::setHdrMode(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setHdrMode(QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  iVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if ((iVar2 != iVar1 && iVar1 != -1) && (iVar1 != 0 || iVar2 != -1)) {
    __android_log_print(2,DAT_000214fc + 0x214e2,DAT_00021500 + 0x214e4,iVar1);
    android::CameraParameters::set((char *)(this + 0x18),DAT_00021504 + 0x214f0);
  }
  return 0;
}



// Function: setFaceBeauty @ 00021508

/* android::QualcommCameraHardware::setFaceBeauty(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setFaceBeauty
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  
  iVar3 = DAT_00021574 + 0x21516;
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  iVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if ((iVar2 != iVar1 && iVar1 != -1) && (iVar1 != 0 || iVar2 != -1)) {
    android::CameraParameters::set((char *)(this + 0x18),iVar3);
    iVar1 = FUN_0001f008(0x1c,iVar1,0);
    if (iVar1 == 0) {
      return 0x80000000;
    }
  }
  return 0;
}



// Function: setBlur @ 00021578

/* android::QualcommCameraHardware::setBlur(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setBlur(QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  
  iVar3 = DAT_00021608 + 0x21586;
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  iVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if ((iVar2 != iVar1 && iVar1 != -1) && (iVar1 != 0 || iVar2 != -1)) {
    __android_log_print(2,DAT_0002160c + 0x215d8,DAT_00021610 + 0x215da,iVar1);
    android::CameraParameters::set((char *)(this + 0x18),iVar3);
    setDropFrame(this,3);
    iVar1 = FUN_0001f008(0x1d,iVar1,0);
    if (iVar1 == 0) {
      return 0x80000000;
    }
  }
  return 0;
}



// Function: setSkinToneEnhancement @ 00021614

/* android::QualcommCameraHardware::setSkinToneEnhancement(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setSkinToneEnhancement
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  char cVar1;
  int iVar2;
  int iVar3;
  
  cVar1 = (**(code **)(DAT_00021690 + 0x2162e))(0x1d);
  if (cVar1 == '\0') {
    __android_log_print(4,DAT_00021694 + 0x21634,DAT_00021698 + 0x21636);
  }
  else {
    iVar3 = DAT_0002169c + 0x21644;
    iVar2 = android::CameraParameters::getInt((char *)param_1);
    if (*(int *)(this + 0x688) != iVar2) {
      __android_log_print(2,DAT_000216a0 + 0x2165e,DAT_000216a4 + 0x21660,iVar2);
      *(int *)(this + 0x688) = iVar2;
      android::CameraParameters::set((char *)(this + 0x18),iVar3);
      iVar2 = native_set_parms(this,0x1d,4,this + 0x688);
      if (iVar2 == 0) {
        return 0x80000000;
      }
    }
  }
  return 0;
}



// Function: setBrightness @ 000216a8

/* android::QualcommCameraHardware::setBrightness(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setBrightness
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  iVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if (iVar1 != iVar2) {
    __android_log_print(2,DAT_00021700 + 0x216d4,DAT_00021704 + 0x216d6,iVar1);
    iVar1 = FUN_0001f008(8,iVar1 + 4,0);
    android::CameraParameters::set((char *)(this + 0x18),*(int *)(DAT_00021708 + 0x216ea));
    if (iVar1 == 0) {
      return 0x80000000;
    }
  }
  return 0;
}



// Function: setAutoContrast @ 0002170c

/* android::QualcommCameraHardware::setAutoContrast(android::CameraParameters const&) */

undefined8 android::QualcommCameraHardware::setAutoContrast(CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  char *in_r1;
  int iVar4;
  
  iVar4 = *(int *)(DAT_00021764 + 0x21718);
  iVar1 = android::CameraParameters::getInt(in_r1);
  iVar2 = android::CameraParameters::getInt((char *)(param_1 + 0x18));
  __android_log_print(2,DAT_00021768 + 0x21734,DAT_0002176c + 0x21736,iVar1);
  if (iVar1 != iVar2) {
    iVar1 = FUN_0001f008(0x18,iVar1,0);
    android::CameraParameters::set((char *)(param_1 + 0x18),iVar4);
    uVar3 = 0x80000000;
    if (iVar1 == 0) goto LAB_00021760;
  }
  uVar3 = 0;
LAB_00021760:
  return CONCAT44(iVar2,uVar3);
}



// Function: setSaturation @ 00021770

/* android::QualcommCameraHardware::setSaturation(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setSaturation
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  uint uVar1;
  uint uVar2;
  int iVar3;
  
  iVar3 = *(int *)(DAT_000217bc + 0x2177c);
  uVar1 = android::CameraParameters::getInt((char *)param_1);
  uVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if (uVar1 != uVar2) {
    if (10 < uVar1) {
      return 0x80000000;
    }
    __android_log_print(2,DAT_000217c0 + 0x217a8,DAT_000217c4 + 0x217aa,uVar1);
    android::CameraParameters::set((char *)(this + 0x18),iVar3);
  }
  return 0;
}



// Function: setContrast @ 000217c8

/* android::QualcommCameraHardware::setContrast(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setContrast(QualcommCameraHardware *this,CameraParameters *param_1)

{
  uint uVar1;
  uint uVar2;
  int iVar3;
  
  iVar3 = *(int *)(DAT_00021814 + 0x217d4);
  uVar1 = android::CameraParameters::getInt((char *)param_1);
  uVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if (uVar1 != uVar2) {
    __android_log_print(2,DAT_00021818 + 0x217f6,DAT_0002181c + 0x217f8,uVar1);
    if (10 < uVar1) {
      return 0x80000000;
    }
    android::CameraParameters::set((char *)(this + 0x18),iVar3);
  }
  return 0;
}



// Function: setSharpness @ 00021820

/* android::QualcommCameraHardware::setSharpness(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setSharpness
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  uint uVar1;
  uint uVar2;
  int iVar3;
  
  iVar3 = *(int *)(DAT_0002186c + 0x2182c);
  uVar1 = android::CameraParameters::getInt((char *)param_1);
  uVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if (uVar1 != uVar2) {
    __android_log_print(2,DAT_00021870 + 0x2184e,DAT_00021874 + 0x21850,uVar1);
    if (0x1e < uVar1) {
      return 0x80000000;
    }
    android::CameraParameters::set((char *)(this + 0x18),iVar3);
  }
  return 0;
}



// Function: setAntiShakeMode @ 00021878

/* android::QualcommCameraHardware::setAntiShakeMode(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setAntiShakeMode
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  
  iVar3 = *(int *)(DAT_000218cc + 0x21884);
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  iVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if (iVar1 != iVar2) {
    __android_log_print(2,DAT_000218d0 + 0x218a6,DAT_000218d4 + 0x218a8,iVar1);
    android::CameraParameters::set((char *)(this + 0x18),iVar3);
    iVar1 = FUN_0001f008(0x17,iVar1,0);
    if (iVar1 == 0) {
      return 0x80000000;
    }
  }
  return 0;
}



// Function: setDTPMode @ 000218d8

/* android::QualcommCameraHardware::setDTPMode(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setDTPMode(QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  
  iVar2 = DAT_0002193c + 0x218e4;
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  if (iVar1 != -1) {
    if (*(uint *)(this + 0x678) != (uint)(iVar1 != 0)) {
      __android_log_print(6,DAT_00021940 + 0x21902,DAT_00021944 + 0x21906,*(uint *)(this + 0x678),
                          iVar1,param_1);
      *(uint *)(this + 0x678) = (uint)(iVar1 != 0);
      android::CameraParameters::set((char *)(this + 0x18),iVar2);
      if (0 < *(int *)(this + 0x678)) {
        setDropFrame(this,5);
      }
      FUN_0001f008(0x19,*(undefined4 *)(this + 0x678),0);
    }
  }
  return 0;
}



// Function: setExposureCompensation @ 00021948

/* android::QualcommCameraHardware::setExposureCompensation(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setExposureCompensation
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  iVar2 = android::CameraParameters::getInt((char *)(this + 0x18));
  if (iVar1 != iVar2) {
    __android_log_print(2,DAT_000219a0 + 0x21974,DAT_000219a4 + 0x21976,iVar1);
    iVar1 = FUN_0001f008(8,iVar1 + 4,0);
    android::CameraParameters::set((char *)(this + 0x18),*(int *)(DAT_000219a8 + 0x2198a));
    if (iVar1 == 0) {
      return 0x80000000;
    }
  }
  return 0;
}



// Function: setJpegQuality @ 000219ac

/* android::QualcommCameraHardware::setJpegQuality(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setJpegQuality
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  uint uVar1;
  uint uVar2;
  undefined4 uVar3;
  int iVar4;
  QualcommCameraHardware *pQVar5;
  int iVar6;
  CameraParameters *pCVar7;
  
  iVar6 = *(int *)(DAT_00021a60 + 0x219ba);
  pQVar5 = this + 0x18;
  pCVar7 = param_1;
  uVar1 = android::CameraParameters::getInt((char *)param_1);
  uVar2 = android::CameraParameters::getInt((char *)pQVar5);
  if (uVar2 != uVar1) {
    iVar4 = DAT_00021a64 + 0x219de;
    uVar2 = uVar1;
    __android_log_print(2,iVar4,DAT_00021a68 + 0x219e0,DAT_00021a6c + 0x219e4,uVar1,pCVar7);
    if (*(int *)(DAT_00021a70 + 0x219ee) == 1) {
      if (uVar1 < 0x65) {
        android::CameraParameters::set((char *)pQVar5,iVar6);
      }
      else {
        __android_log_print(6,iVar4,DAT_00021a74 + 0x21a0e,uVar1,uVar2,pCVar7);
      }
      iVar6 = *(int *)(DAT_00021a78 + 0x21a18);
      uVar1 = android::CameraParameters::getInt((char *)param_1);
      if (uVar1 < 0x65) {
        android::CameraParameters::set((char *)pQVar5,iVar6);
      }
      else {
        __android_log_print(6,DAT_00021a7c + 0x21a3a,DAT_00021a80 + 0x21a3c,uVar1,uVar2);
      }
    }
    else {
      if ((int)uVar1 < 0x4b) {
        if ((int)uVar1 < 0x32) {
          uVar3 = 3;
        }
        else {
          uVar3 = 2;
        }
      }
      else {
        uVar3 = 1;
      }
      FUN_0001f008(0x15,uVar3,0);
    }
  }
  return 0;
}



// Function: setJpegThumbnailSize @ 00021a84

/* android::QualcommCameraHardware::setJpegThumbnailSize(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setJpegThumbnailSize
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  int iVar4;
  int iVar5;
  bool bVar6;
  
  iVar5 = DAT_00021b1c + 0x21a92;
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  iVar2 = android::CameraParameters::getInt((char *)param_1);
  if (*(int *)(DAT_00021b28 + 0x21aac) == 0) {
    iVar4 = 0;
    do {
      if ((iVar1 == *(int *)(DAT_00021b2c + 0x21ad0 + iVar4)) &&
         (iVar2 == *(int *)(DAT_00021b2c + 0x21ad0 + iVar4 + 4))) goto LAB_00021ae8;
      iVar4 = iVar4 + 8;
    } while (iVar4 != 0x20);
LAB_00021b14:
    uVar3 = 0xffffffea;
  }
  else {
    if (iVar1 == 0x140) {
      bVar6 = iVar2 == 0xf0;
LAB_00021ac0:
      if (!bVar6) goto LAB_00021b14;
    }
    else {
      if (iVar1 == 0xa0) {
        bVar6 = iVar2 == 0x78;
        goto LAB_00021ac0;
      }
      if ((iVar1 != 0) || (iVar2 != 0)) goto LAB_00021b14;
    }
LAB_00021ae8:
    android::CameraParameters::set((char *)(this + 0x18),*(int *)(iVar5 + DAT_00021b20));
    android::CameraParameters::set((char *)(this + 0x18),*(int *)(iVar5 + DAT_00021b24));
    *(int *)(this + 0xd40) = iVar1;
    *(int *)(this + 0xd44) = iVar2;
    uVar3 = 0;
  }
  return uVar3;
}



// Function: setMovieMode @ 00021b30

/* android::QualcommCameraHardware::setMovieMode(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setMovieMode
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  
  iVar2 = DAT_00021b94 + 0x21b3c;
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  if ((iVar1 != -1) && (this[0x6b0] != (QualcommCameraHardware)(iVar1 != 0))) {
    __android_log_print(2,DAT_00021b98 + 0x21b60,DAT_00021b9c + 0x21b62,iVar1);
    this[0x6b0] = (QualcommCameraHardware)(iVar1 != 0);
    android::CameraParameters::set((char *)(this + 0x18),iVar2);
    iVar1 = FUN_0001f008(0x1f,this[0x6b0],0);
    if (iVar1 == 0) {
      return 0x80000000;
    }
  }
  return 0;
}



// Function: setVtSurface @ 00021ba0

/* android::QualcommCameraHardware::setVtSurface(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setVtSurface
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  
  iVar2 = DAT_00021be4 + 0x21bac;
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  if ((iVar1 != -1) && (*(int *)(this + 0x6a0) != iVar1)) {
    __android_log_print(2,DAT_00021be8 + 0x21bca,DAT_00021bec + 0x21bcc,iVar1);
    *(int *)(this + 0x6a0) = iVar1;
    android::CameraParameters::set((char *)(this + 0x18),iVar2);
  }
  return 0;
}



// Function: setVtMode @ 00021bf0

/* android::QualcommCameraHardware::setVtMode(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setVtMode(QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  
  iVar2 = DAT_00021c40 + 0x21bfc;
  iVar1 = android::CameraParameters::getInt((char *)param_1);
  if ((iVar1 != -1) && (*(int *)(this + 0x69c) != iVar1)) {
    __android_log_print(2,DAT_00021c44 + 0x21c1a,DAT_00021c48 + 0x21c1c,iVar1);
    *(int *)(this + 0x69c) = iVar1;
    android::CameraParameters::set((char *)(this + 0x18),iVar2);
    FUN_0001f008(0x22,*(undefined4 *)(this + 0x69c),0);
  }
  return 0;
}



// Function: setCameraMode @ 00021c4c

/* android::QualcommCameraHardware::setCameraMode(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setCameraMode
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  pthread_mutex_t *__mutex;
  
  iVar4 = *(int *)(DAT_00021ce8 + 0x21c5a);
  iVar3 = android::CameraParameters::getInt((char *)param_1);
  android::CameraParameters::set((char *)(this + 0x18),iVar4);
  __android_log_print(4,DAT_00021cec + 0x21c7a,DAT_00021cf0 + 0x21c7c,iVar3);
  iVar2 = DAT_00021cfc;
  iVar1 = DAT_00021cf8;
  iVar4 = DAT_00021cf4;
  if (iVar3 != *(int *)(this + 0xd64)) {
    __mutex = (pthread_mutex_t *)(this + 0x35c);
    pthread_mutex_lock(__mutex);
    while (this[0x358] != (QualcommCameraHardware)0x0) {
      __android_log_print(4,iVar4 + 0x21ca2,iVar1 + 0x21ca4);
      pthread_cond_wait((pthread_cond_t *)(this + 0x360),__mutex);
      __android_log_print(4,iVar4 + 0x21ca2,iVar2 + 0x21ca6);
    }
    pthread_mutex_unlock(__mutex);
  }
  if (iVar3 == 1) {
    *(undefined4 *)(this + 0xd64) = 1;
  }
  else {
    *(undefined4 *)(this + 0xd64) = 0;
  }
  return 0;
}



// Function: setSnapshotCount @ 00021d00

/* android::QualcommCameraHardware::setSnapshotCount(android::CameraParameters const&) */

undefined4 android::QualcommCameraHardware::setSnapshotCount(CameraParameters *param_1)

{
  int iVar1;
  char *__nptr;
  char *in_r1;
  int iVar2;
  CameraParameters *pCVar3;
  char acStack_18 [8];
  
  pCVar3 = param_1;
  if (*(int *)(param_1 + 0xd64) == 0) {
    iVar2 = *(int *)(param_1 + 0xc);
  }
  else {
    __nptr = (char *)android::CameraParameters::get(in_r1);
    if (__nptr == (char *)0x0) {
      iVar2 = 1;
      goto LAB_00021d34;
    }
    iVar2 = atoi(__nptr);
  }
  if (iVar2 < 4) {
    if (iVar2 < 1) {
      iVar2 = 1;
    }
  }
  else {
    iVar2 = 3;
  }
LAB_00021d34:
  snprintf(acStack_18,5,(char *)(DAT_00021d74 + 0x21d40),iVar2,pCVar3);
  iVar1 = DAT_00021d78;
  *(int *)(param_1 + 0xc) = iVar2;
  android::CameraParameters::set((char *)(param_1 + 0x18),(char *)(iVar1 + 0x21d52));
  __android_log_print(4,DAT_00021d7c + 0x21d60,DAT_00021d80 + 0x21d62,DAT_00021d84 + 0x21d66,
                      acStack_18);
  return 0;
}



// Function: setZslParam @ 00021d88

/* android::QualcommCameraHardware::setZslParam(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setZslParam(QualcommCameraHardware *this,CameraParameters *param_1)

{
  char *__s1;
  int iVar1;
  
  if (*(int *)(this + 0xd64) == 0) {
    __android_log_print(2,DAT_00021dd0 + 0x21d9c,DAT_00021dd4 + 0x21d9e);
  }
  else {
    __s1 = (char *)android::CameraParameters::get((char *)param_1);
    if (__s1 == (char *)0x0) {
      *(undefined4 *)(this + 0xd68) = 0;
    }
    else {
      iVar1 = strncmp(__s1,(char *)(DAT_00021ddc + 0x21db8),8);
      *(uint *)(this + 0xd68) = (uint)(iVar1 == 0);
    }
  }
  return 0;
}



// Function: setStrTextures @ 00021de0

/* android::QualcommCameraHardware::setStrTextures(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setStrTextures
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  char *__s1;
  int iVar1;
  QualcommCameraHardware QVar2;
  char *pcVar3;
  
  pcVar3 = (char *)(DAT_00021e58 + 0x21dec);
  __s1 = (char *)android::CameraParameters::get((char *)param_1);
  if (__s1 != (char *)0x0) {
    __android_log_print(2,DAT_00021e5c + 0x21e00,DAT_00021e60 + 0x21e02,__s1);
    android::CameraParameters::set((char *)(this + 0x18),pcVar3);
    iVar1 = strncmp(__s1,(char *)(DAT_00021e64 + 0x21e1a),2);
    if ((iVar1 == 0) || (iVar1 = strncmp(__s1,(char *)(DAT_00021e68 + 0x21e28),2), iVar1 == 0)) {
      QVar2 = (QualcommCameraHardware)0x1;
    }
    else {
      iVar1 = strncmp(__s1,(char *)(DAT_00021e6c + 0x21e3a),3);
      if ((iVar1 != 0) && (iVar1 = strncmp(__s1,(char *)(DAT_00021e70 + 0x21e48),3), iVar1 != 0)) {
        return 0;
      }
      QVar2 = (QualcommCameraHardware)0x0;
    }
    this[0xd48] = QVar2;
  }
  return 0;
}



// Function: setGpsLocation @ 00021e74

/* android::QualcommCameraHardware::setGpsLocation(android::CameraParameters const&) */

longlong __thiscall
android::QualcommCameraHardware::setGpsLocation
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  char *pcVar3;
  int iVar4;
  QualcommCameraHardware *pQVar5;
  char *pcVar6;
  char *pcVar7;
  QualcommCameraHardware *local_30;
  
  iVar4 = DAT_00021f64 + 0x21e82;
  pcVar7 = *(char **)(iVar4 + DAT_00021f68);
  iVar1 = android::CameraParameters::get((char *)param_1);
  pcVar6 = *(char **)(iVar4 + DAT_00021f6c);
  iVar2 = android::CameraParameters::get((char *)param_1);
  pcVar3 = *(char **)(iVar4 + DAT_00021f70);
  local_30 = (QualcommCameraHardware *)android::CameraParameters::get((char *)param_1);
  if ((iVar2 == 0 || iVar1 == 0) || (local_30 == (QualcommCameraHardware *)0x0)) {
    pQVar5 = this + 0x18;
    android::CameraParameters::remove((char *)pQVar5);
    android::CameraParameters::remove((char *)pQVar5);
    android::CameraParameters::remove((char *)pQVar5);
    local_30 = this;
  }
  else {
    pQVar5 = this + 0x18;
    android::CameraParameters::set((char *)pQVar5,pcVar7);
    android::CameraParameters::set((char *)pQVar5,pcVar6);
    android::CameraParameters::set((char *)pQVar5,pcVar3);
  }
  pcVar3 = *(char **)(iVar4 + DAT_00021f74);
  iVar1 = android::CameraParameters::get((char *)param_1);
  if (iVar1 == 0) {
    android::CameraParameters::remove((char *)(this + 0x18));
  }
  else {
    android::CameraParameters::set((char *)(this + 0x18),pcVar3);
  }
  pcVar3 = *(char **)(iVar4 + DAT_00021f78);
  iVar1 = android::CameraParameters::get((char *)param_1);
  if (iVar1 == 0) {
    android::CameraParameters::remove((char *)(this + 0x18));
  }
  else {
    android::CameraParameters::set((char *)(this + 0x18),pcVar3);
  }
  return ZEXT48(local_30) << 0x20;
}



// Function: updatePictureDimension @ 00021f7c

/* android::QualcommCameraHardware::updatePictureDimension(android::CameraParameters const&, int&,
   int&) */

void __thiscall
android::QualcommCameraHardware::updatePictureDimension
          (QualcommCameraHardware *this,CameraParameters *param_1,int *param_2,int *param_3)

{
  QualcommCameraHardware QVar1;
  int iVar2;
  uint uVar3;
  int iVar4;
  int local_20;
  uint local_1c [2];
  
  android::CameraParameters::getPreviewSize((int *)param_1,(int *)local_1c);
  __android_log_print(2,DAT_00022014 + 0x21f9e,DAT_00022018 + 0x21fa6,*param_2,*param_3,local_1c[0],
                      local_20);
  if ((*param_2 < (int)local_1c[0]) && (*param_3 < local_20)) {
    *(int *)(this + 0xd7c) = *param_2;
    uVar3 = local_1c[0] + 7 & (int)local_1c[0] >> 0x20;
    if (local_1c[0] < 0xfffffff9) {
      uVar3 = local_1c[0];
    }
    *(int *)(this + 0xd80) = *param_3;
    iVar4 = *param_2;
    if (iVar4 < (int)uVar3 >> 3) {
      iVar2 = __aeabi_idiv(local_1c[0],iVar4);
      for (uVar3 = 0; ((int)uVar3 < iVar2 && (7 < iVar2 >> (uVar3 & 0xff))); uVar3 = uVar3 + 1) {
      }
      *param_2 = uVar3 * iVar4 * 2;
      local_20 = uVar3 * *param_3 * 2;
    }
    else {
      *param_2 = local_1c[0];
    }
    *param_3 = local_20;
    QVar1 = (QualcommCameraHardware)0x1;
  }
  else {
    QVar1 = (QualcommCameraHardware)0x0;
  }
  this[0xd84] = QVar1;
  return;
}



// Function: setTouchAfAec @ 0002201c

/* android::QualcommCameraHardware::setTouchAfAec(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setTouchAfAec
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  char *pcVar2;
  uint uVar3;
  int iVar4;
  int iVar5;
  QualcommCameraHardware *pQVar6;
  uint uVar7;
  int iVar8;
  undefined1 auStack_60 [4];
  undefined1 local_5c;
  undefined2 local_5a;
  undefined2 local_58;
  undefined2 local_56;
  undefined2 local_54;
  QualcommCameraHardware local_4a;
  int local_48;
  int local_44;
  uint local_40;
  int local_3c;
  int local_38;
  int local_34;
  int local_30;
  uint local_2c [2];
  
  iVar4 = DAT_000221ec + 0x2202c;
  __android_log_print(6,iVar4,DAT_000221f0 + 0x22034,DAT_000221f4 + 0x22032);
  if (this[0xd28] != (QualcommCameraHardware)0x0) {
    pQVar6 = this + 0x18;
    android::CameraParameters::getMeteringAreaCenter((int *)param_1,(int *)local_2c);
    android::CameraParameters::getPreviewSize((int *)pQVar6,&local_34);
    pcVar2 = *(char **)(DAT_000221f8 + 0x2208c);
    uVar7 = (uint)((float)(longlong)(int)(local_2c[0] + 1000) *
                  ((float)(longlong)local_34 / DAT_000221e8));
    iVar8 = (int)((float)(longlong)(local_30 + 1000) * ((float)(longlong)local_38 / DAT_000221e8));
    local_30 = iVar8;
    local_2c[0] = uVar7;
    iVar1 = android::CameraParameters::get((char *)param_1);
    if (iVar1 == 0) {
      __android_log_print(6,iVar4,DAT_00022208 + 0x221d6,DAT_0002220c + 0x221d8);
      return 0xffffffea;
    }
    local_48 = FUN_0001f71c(DAT_000221fc + 0x220d4,2,iVar1);
    if (local_48 != -1) {
      iVar4 = android::CameraParameters::getInt((char *)param_1);
      iVar1 = android::CameraParameters::getInt((char *)param_1);
      android::CameraParameters::set((char *)pQVar6,pcVar2);
      android::CameraParameters::setTouchIndexAec((int)pQVar6,uVar7);
      android::CameraParameters::setTouchIndexAf((int)pQVar6,uVar7);
      memset(auStack_60,0,0x18);
      local_5c = (undefined1)local_48;
      if (local_48 == 1) {
        uVar3 = ~uVar7 >> 0x1f;
        if (iVar8 < 0) {
          uVar3 = 0;
        }
        if (uVar3 != 0) {
          local_4a = this[0xd92];
          iVar5 = uVar7 - iVar4 / 2;
          if (iVar5 < 0) {
            local_5a = 1;
          }
          else {
            local_5a = (undefined2)iVar5;
          }
          iVar5 = iVar8 - iVar1 / 2;
          if (iVar5 < 0) {
            local_58 = 1;
          }
          else {
            local_58 = (undefined2)iVar5;
          }
          local_56 = (undefined2)iVar4;
          local_54 = (undefined2)iVar1;
          local_44 = local_48;
          local_40 = uVar7;
          local_3c = iVar8;
          native_set_parms(this,0x13,0x10,&local_48);
          native_set_parms(this,0x14,0x18,auStack_60);
        }
      }
      else if (local_48 == 0) {
        local_40 = 0xffffffff;
        local_44 = 1;
        local_3c = 0xffffffff;
        native_set_parms(this,0x13,0x10,&local_48);
        native_set_parms(this,0x14,0x18,auStack_60);
        return 0;
      }
    }
  }
  return 0;
}



// Function: setHighFrameRate @ 00022210

/* android::QualcommCameraHardware::setHighFrameRate(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setHighFrameRate
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  char cVar1;
  char *__s1;
  int iVar2;
  uint uVar3;
  undefined4 uVar4;
  char *__s2;
  int iVar5;
  int iVar6;
  char *pcVar7;
  QualcommCameraHardware QVar8;
  undefined4 local_2c [2];
  
  cVar1 = (**(code **)(DAT_00022340 + 0x2222c))(0x31);
  if ((cVar1 == '\0') || (this[0xcec] != (QualcommCameraHardware)0x0)) {
    __android_log_print(4,DAT_00022344 + 0x2223e,DAT_00022348 + 0x22240);
LAB_00022302:
    uVar4 = 0;
  }
  else {
    pcVar7 = *(char **)(DAT_00022370 + 0x2232e);
    __s2 = (char *)android::CameraParameters::get((char *)param_1);
    if (__s2 == (char *)0x0) {
      __s2 = (char *)(DAT_00022364 + 0x2230c);
    }
    else {
      local_2c[0] = FUN_0001f71c(DAT_0002234c + 0x2224e,4,__s2);
      if (local_2c[0] != -1) {
        iVar6 = DAT_00022350 + 0x22262;
        iVar5 = DAT_00022354 + 0x22264;
        __android_log_print(4,iVar6,DAT_00022358 + 0x22268,iVar5,__s2,local_2c[0]);
        __s1 = (char *)android::CameraParameters::get((char *)(this + 0x18));
        iVar2 = strcmp(__s1,__s2);
        if (iVar2 != 0) {
          __android_log_print(4,iVar6,DAT_0002235c + 0x22294,iVar5,__s1,__s2);
          android::CameraParameters::set((char *)(this + 0x18),pcVar7);
          this[0xd75] = (QualcommCameraHardware)0x1;
          if (this[0x34] != (QualcommCameraHardware)0x0) {
            pthread_mutex_lock((pthread_mutex_t *)(this + 0x32c));
            pthread_attr_init((pthread_attr_t *)&stack0xffffffbc);
            pthread_attr_setdetachstate((pthread_attr_t *)&stack0xffffffbc,1);
            uVar3 = pthread_create((pthread_t *)(this + 0x5f0),(pthread_attr_t *)&stack0xffffffbc,
                                   *(__start_routine **)(DAT_00022360 + 0x222d6),(void *)0x0);
            QVar8 = (QualcommCameraHardware)('\x01' - (char)uVar3);
            if (1 < uVar3) {
              QVar8 = (QualcommCameraHardware)0x0;
            }
            this[0x328] = QVar8;
            pthread_mutex_unlock((pthread_mutex_t *)(this + 0x32c));
            return 0;
          }
        }
        native_set_parms(this,0x31,4,local_2c);
        goto LAB_00022302;
      }
    }
    __android_log_print(6,DAT_00022368 + 0x22316,DAT_0002236c + 0x22318,__s2);
    uVar4 = 0xffffffea;
  }
  return uVar4;
}



// Function: autoFocus @ 00022374

/* android::QualcommCameraHardware::autoFocus() */

undefined4 __thiscall android::QualcommCameraHardware::autoFocus(QualcommCameraHardware *this)

{
  uint uVar1;
  int iVar2;
  int iVar3;
  undefined4 uVar4;
  pthread_mutex_t *__mutex;
  pthread_t apStack_24 [2];
  
  iVar3 = DAT_00022418 + 0x22382;
  __android_log_print(2,iVar3,DAT_0002241c + 0x22386);
  __mutex = (pthread_mutex_t *)(this + 0x5d0);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  pthread_mutex_lock(__mutex);
  if (this[0x5cc] == (QualcommCameraHardware)0x0) {
    pthread_attr_init((pthread_attr_t *)&stack0xffffffc4);
    pthread_attr_setdetachstate((pthread_attr_t *)&stack0xffffffc4,1);
    uVar1 = pthread_create(apStack_24,(pthread_attr_t *)&stack0xffffffc4,
                           *(__start_routine **)(DAT_00022420 + 0x223c6),(void *)0x0);
    iVar2 = 1 - uVar1;
    if (1 < uVar1) {
      iVar2 = 0;
    }
    this[0x5cc] = SUB41(iVar2,0);
    if (iVar2 == 0) {
      uVar4 = 0x80000000;
      __android_log_print(6,iVar3,DAT_00022424 + 0x223e8);
      pthread_mutex_unlock(__mutex);
      goto LAB_00022408;
    }
  }
  uVar4 = 0;
  pthread_mutex_unlock(__mutex);
  __android_log_print(2,DAT_00022428 + 0x22404,DAT_0002242c + 0x22406);
LAB_00022408:
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return uVar4;
}



// Function: setPreviewFormat @ 00022430

/* android::QualcommCameraHardware::setPreviewFormat(android::CameraParameters const&) */

undefined4 android::QualcommCameraHardware::setPreviewFormat(CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int *piVar3;
  int iVar4;
  int iVar5;
  
  iVar2 = DAT_000224b4;
  iVar1 = android::CameraParameters::getPreviewFormat();
  piVar3 = (int *)(iVar2 + 0x22446);
  iVar5 = DAT_000224b8 + 0x2244a;
  iVar4 = DAT_000224bc + 0x2244c;
  iVar2 = FUN_0001f71c(DAT_000224c0 + 0x22454,3,iVar1);
  __android_log_print(2,iVar5,iVar4,*piVar3);
  if (*piVar3 != iVar2) {
    if (iVar2 == -1) {
      if (iVar1 == 0) {
        iVar1 = DAT_000224c8 + 0x2249c;
      }
      __android_log_print(6,DAT_000224cc + 0x224a6,DAT_000224d0 + 0x224a8,iVar1);
      return 0xffffffea;
    }
    android::CameraParameters::set((char *)(param_1 + 0x18),*(char **)(DAT_000224c4 + 0x2247c));
    *piVar3 = iVar2;
    __android_log_print(6,iVar5,iVar4,iVar2);
  }
  return 0;
}



// Function: initZslParameter @ 000224d4

/* android::QualcommCameraHardware::initZslParameter() */

undefined4 android::QualcommCameraHardware::initZslParameter(void)

{
  int iVar1;
  int iVar2;
  QualcommCameraHardware *in_r0;
  undefined4 uVar3;
  int iVar4;
  undefined4 *__s;
  
  iVar4 = DAT_0002259c + 0x224e4;
  __android_log_print(2,iVar4,DAT_000225a0 + 0x224ea,DAT_000225a4 + 0x224ec);
  android::CameraParameters::getPictureSize((int *)(in_r0 + 0x18),(int *)(in_r0 + 0xd4c));
  uVar3 = *(undefined4 *)(in_r0 + 0xd50);
  __android_log_print(2,iVar4,DAT_000225a8 + 0x22514,*(undefined4 *)(in_r0 + 0xd4c),uVar3);
  iVar4 = updatePictureDimension
                    (in_r0,(CameraParameters *)(in_r0 + 0x18),(int *)(in_r0 + 0xd4c),
                     (int *)(in_r0 + 0xd50));
  if (iVar4 != 0) {
    *(undefined2 *)(in_r0 + 0x43c) = *(undefined2 *)(in_r0 + 0xd4c);
    *(undefined2 *)(in_r0 + 0x43e) = *(undefined2 *)(in_r0 + 0xd50);
  }
  iVar4 = DAT_000225ac;
  *(undefined4 *)(DAT_000225ac + 0x22546) = *(undefined4 *)(in_r0 + 0xd4c);
  *(undefined4 *)(iVar4 + 0x2254a) = *(undefined4 *)(in_r0 + 0xd50);
  *(uint *)(iVar4 + 0x2254e) = (uint)*(ushort *)(in_r0 + 0x440);
  iVar1 = DAT_000225b0;
  __s = (undefined4 *)(DAT_000225b0 + 0x22566);
  *(uint *)(iVar4 + 0x22552) = (uint)*(ushort *)(in_r0 + 0x442);
  *(undefined1 *)(iVar4 + 0x22556) = 1;
  memset(__s,0,0xc);
  iVar2 = DAT_000225b8;
  iVar4 = DAT_000225b4;
  *(undefined4 *)(iVar1 + 0x2256e) = *(undefined4 *)(in_r0 + 0xd58);
  *(undefined4 *)(iVar1 + 0x2256a) = *(undefined4 *)(in_r0 + 0xd54);
  __android_log_print(2,iVar4 + 0x22582,iVar2 + 0x22588,*(undefined4 *)(in_r0 + 0xc),uVar3);
  *__s = *(undefined4 *)(in_r0 + 0xc);
  return 1;
}



// Function: setPictureSize @ 000225bc

/* android::QualcommCameraHardware::setPictureSize(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setPictureSize
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  QualcommCameraHardware *pQVar3;
  int local_28;
  int local_24;
  int local_20;
  int local_1c [2];
  
  android::CameraParameters::getPictureSize((int *)param_1,local_1c);
  pQVar3 = this + 0x18;
  __android_log_print(2,DAT_000226b8 + 0x225dc,DAT_000226bc + 0x225de,local_1c[0],local_20);
  android::CameraParameters::getPictureSize((int *)pQVar3,&local_24);
  if ((local_1c[0] != local_24) || (local_20 != local_28)) {
    iVar2 = 0;
    for (iVar1 = 0; iVar1 < *(int *)(DAT_000226c0 + 0x2260a); iVar1 = iVar1 + 1) {
      if ((local_1c[0] == *(int *)(*(int *)(DAT_000226c4 + 0x22610) + iVar2)) &&
         (local_20 == *(int *)(*(int *)(DAT_000226c4 + 0x22610) + iVar2 + 4))) {
        android::CameraParameters::setPictureSize((int)pQVar3,local_1c[0]);
        iVar2 = DAT_000226cc;
        iVar1 = DAT_000226c8;
        *(short *)(this + 0x43e) = (short)local_20;
        *(short *)(this + 0x43c) = (short)local_1c[0];
        __android_log_print(2,iVar1 + 0x2263c,iVar2 + 0x22640,local_1c[0],local_20);
        FUN_0001f008(0x14,local_1c[0],local_20);
        return 0;
      }
      iVar2 = iVar2 + 8;
    }
    iVar1 = isValidDimension(this,local_1c[0],local_20);
    if (iVar1 == 0) {
      iVar1 = DAT_000226d0 + 0x22670;
      __android_log_print(6,iVar1,DAT_000226d4 + 0x22674,local_1c[0],local_20);
      local_1c[0] = **(int **)(DAT_000226d8 + 0x22682);
      local_20 = (*(int **)(DAT_000226d8 + 0x22682))[1];
      __android_log_print(6,iVar1,DAT_000226dc + 0x22690,local_1c[0],local_20);
    }
    android::CameraParameters::setPictureSize((int)pQVar3,local_1c[0]);
    *(undefined2 *)(this + 0x43c) = (undefined2)local_1c[0];
    *(undefined2 *)(this + 0x43e) = (undefined2)local_20;
  }
  return 0;
}



// Function: setPreviewFrameRateMode @ 000226e0

/* android::QualcommCameraHardware::setPreviewFrameRateMode(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setPreviewFrameRateMode
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  short sVar1;
  short sVar2;
  char *__s1;
  char *__s2;
  int iVar3;
  uint uVar4;
  uint uVar5;
  undefined4 uVar6;
  int iVar7;
  char *pcVar8;
  QualcommCameraHardware *pQVar9;
  
  pQVar9 = this + 0x18;
  __s1 = (char *)android::CameraParameters::getPreviewFrameRateMode();
  __s2 = (char *)android::CameraParameters::getPreviewFrameRateMode();
  iVar3 = FUN_0001f71c(DAT_000227b8 + 0x22704,2,__s2);
  if (iVar3 == -1) {
    pcVar8 = __s2;
    if (__s2 == (char *)0x0) {
      pcVar8 = (char *)(DAT_000227cc + 0x2279c);
    }
    iVar3 = DAT_000227d0 + 0x227a6;
    iVar7 = DAT_000227d4 + 0x227a8;
LAB_000227a6:
    __android_log_print(6,iVar3,iVar7,pcVar8,this,param_1);
    uVar6 = 0xffffffea;
  }
  else {
    sVar1 = android::CameraParameters::getPreviewFrameRate();
    sVar2 = android::CameraParameters::getPreviewFrameRate();
    pcVar8 = (char *)(int)sVar1;
    uVar4 = strcmp(__s1,__s2);
    uVar5 = 1 - uVar4;
    if (1 < uVar4) {
      uVar5 = 0;
    }
    if (pcVar8 == (char *)(int)sVar2) {
      uVar5 = uVar5 & 1;
    }
    else {
      uVar5 = 0;
    }
    if (uVar5 == 0) {
      android::CameraParameters::setPreviewFrameRateMode((char *)pQVar9);
      if (0x1a < (ushort)(sVar1 - 5U)) {
        iVar3 = DAT_000227c4 + 0x22792;
        iVar7 = DAT_000227c8 + 0x22794;
        goto LAB_000227a6;
      }
      android::CameraParameters::setPreviewFrameRate((int)pQVar9);
      __android_log_print(2,DAT_000227bc + 0x2276c,DAT_000227c0 + 0x22770,__s2,pcVar8);
      iVar3 = FUN_0001f008(10,iVar3,pcVar8);
      if (iVar3 == 0) {
        return 0x80000000;
      }
    }
    uVar6 = 0;
  }
  return uVar6;
}



// Function: setPreviewFrameRate @ 000227d8

/* android::QualcommCameraHardware::setPreviewFrameRate(android::CameraParameters const&) */

undefined4 android::QualcommCameraHardware::setPreviewFrameRate(CameraParameters *param_1)

{
  int iVar1;
  short sVar2;
  short sVar3;
  undefined4 uVar4;
  int *in_r1;
  int in_r2;
  int in_r3;
  int iVar5;
  char *pcVar6;
  CameraParameters *pCVar7;
  int local_1c;
  
  pCVar7 = param_1;
  local_1c = in_r3;
  sVar2 = android::CameraParameters::getPreviewFrameRate();
  sVar3 = android::CameraParameters::getPreviewFrameRate();
  android::CameraParameters::getPreviewFpsRange(in_r1,&local_1c);
  if (((local_1c < 0) || (in_r2 < 0)) || (in_r2 < local_1c)) {
    __android_log_print(6,DAT_000228ac + 0x2281a,DAT_000228b0 + 0x2281c,local_1c,in_r2);
LAB_0002285e:
    uVar4 = 0xffffffea;
  }
  else {
    if ((param_1[0x618] == (CameraParameters)0x0) || (sVar3 != sVar2)) {
      iVar5 = DAT_000228bc + 0x22850;
      __android_log_print(2,iVar5,DAT_000228c0 + 0x22852,sVar3,pCVar7);
      if (0x1a < (ushort)(sVar3 - 5U)) goto LAB_0002285e;
      android::CameraParameters::setPreviewFrameRate((int)(param_1 + 0x18));
      iVar1 = DAT_000228c8;
      __android_log_print(2,iVar5,DAT_000228c4 + 0x2287a,sVar3,local_1c,in_r2);
      pcVar6 = *(char **)(iVar1 + 0x22888);
      android::CameraParameters::get((char *)in_r1);
      android::CameraParameters::set((char *)(param_1 + 0x18),pcVar6);
      FUN_0001f008(10,sVar3,0);
    }
    else {
      __android_log_print(2,DAT_000228b4 + 0x2283e,DAT_000228b8 + 0x22840,sVar3,pCVar7);
    }
    uVar4 = 0;
  }
  return uVar4;
}



// Function: setPreviewFpsRange @ 000228cc

/* android::QualcommCameraHardware::setPreviewFpsRange(android::CameraParameters const&) */

undefined4 android::QualcommCameraHardware::setPreviewFpsRange(CameraParameters *param_1)

{
  undefined4 uVar1;
  int *in_r1;
  int in_r2;
  int in_r3;
  int local_c;
  
  local_c = in_r3;
  android::CameraParameters::getPreviewFpsRange(in_r1,&local_c);
  __android_log_print(6,DAT_00022918 + 0x228e6,DAT_0002291c + 0x228ea,local_c,in_r2);
  if ((local_c == *(int *)(DAT_00022920 + 0x228f6)) && (in_r2 == *(int *)(DAT_00022920 + 0x228fa)))
  {
    android::CameraParameters::setPreviewFpsRange((int)(param_1 + 0x18),local_c);
    uVar1 = 0;
  }
  else {
    uVar1 = 0xffffffea;
  }
  return uVar1;
}



// Function: setPreviewSize @ 00022924

/* android::QualcommCameraHardware::setPreviewSize(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setPreviewSize
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  undefined4 uVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  QualcommCameraHardware *pQVar5;
  uint uVar6;
  uint uVar7;
  uint uVar8;
  uint local_28;
  uint local_24;
  uint local_20;
  uint local_1c;
  
  android::CameraParameters::getPreviewSize((int *)param_1,(int *)&local_1c);
  uVar8 = local_20;
  __android_log_print(6,DAT_00022b58 + 0x22944,DAT_00022b5c + 0x2294e,*(undefined4 *)(this + 0x69c),
                      local_1c,local_20);
  if (((int)local_1c < 0) || ((int)local_20 < 0)) {
    __android_log_print(2,DAT_00022b60 + 0x22968,DAT_00022b64 + 0x2296a,local_1c,local_20,uVar8);
    uVar1 = 0xffffffea;
  }
  else {
    if (*(int *)(this + 0x69c) != 0) {
      *(uint *)(this + 0x6a8) = local_1c;
      *(uint *)(this + 0x6ac) = local_20;
    }
    pQVar5 = this + 0x18;
    android::CameraParameters::getPreviewSize((int *)pQVar5,(int *)&local_24);
    if ((local_1c == local_24) && (local_20 == local_28)) {
      *(uint *)(this + 0xd10) = local_1c;
      *(uint *)(this + 0xd14) = local_20;
      *(short *)(this + 0x440) = (short)local_1c;
      *(short *)(this + 0x442) = (short)local_20;
      *(uint *)(this + 0x690) = local_1c;
      *(uint *)(this + 0x694) = local_20;
    }
    else {
      iVar4 = 0;
      iVar3 = 0;
      if (*(int *)(DAT_00022b68 + 0x229c6) == 0) {
        iVar2 = 7;
      }
      else {
        iVar2 = 3;
      }
      do {
        if ((local_1c == *(uint *)(*(int *)(DAT_00022b6c + 0x229d6) + iVar4)) &&
           (local_20 == *(uint *)(*(int *)(DAT_00022b6c + 0x229d6) + iVar4 + 4))) {
          uVar6 = local_1c;
          uVar7 = local_20;
          if ((*(int *)(this + 0x69c) == 1) && ((local_1c == 0xb0 || (local_20 == 0x90)))) {
            uVar6 = 0x140;
            uVar7 = 0xf0;
          }
          android::CameraParameters::setPreviewSize((int)pQVar5,uVar6);
          *(short *)(this + 0x440) = (short)uVar6;
          *(short *)(this + 0x442) = (short)uVar7;
          *(uint *)(this + 0xd10) = uVar6 & 0xffff;
          *(uint *)(this + 0xd14) = uVar7 & 0xffff;
          iVar4 = DAT_00022b74;
          iVar3 = DAT_00022b70;
          if ((*(uint *)(this + 0x690) == uVar6) && (*(uint *)(this + 0x694) == uVar7))
          goto LAB_00022b4c;
          *(uint *)(this + 0x690) = uVar6;
          iVar3 = iVar3 + 0x22a44;
          *(uint *)(this + 0x694) = uVar7;
          iVar4 = iVar4 + 0x22a4a;
          goto LAB_00022b42;
        }
        iVar3 = iVar3 + 1;
        iVar4 = iVar4 + 8;
      } while (iVar3 != iVar2);
      iVar3 = DAT_00022b78 + 0x22a62;
      __android_log_print(2,iVar3,DAT_00022b7c + 0x22a68,local_1c,local_20,uVar8);
      __android_log_print(2,iVar3,DAT_00022b80 + 0x22a76);
      if ((local_1c != (local_1c + 0xf & 0xfff0)) || (local_20 != (local_20 + 0xf & 0xfff0))) {
        uVar6 = local_1c & DAT_00022b54;
        if ((int)uVar6 < 0) {
          uVar6 = ~(~((uVar6 - 1) * 0x10000000) >> 0x1c) + 1;
        }
        local_1c = local_1c - uVar6;
        uVar6 = local_20 & DAT_00022b54;
        if ((int)uVar6 < 0) {
          uVar6 = ~(~((uVar6 - 1) * 0x10000000) >> 0x1c) + 1;
        }
        local_20 = local_20 - uVar6;
      }
      if ((0x280 < (int)local_1c) || (0x1e0 < (int)local_20)) {
        local_1c = 0x280;
        local_20 = 0x1e0;
      }
      android::CameraParameters::setPreviewSize((int)pQVar5,local_1c);
      *(undefined2 *)(this + 0x440) = (undefined2)local_1c;
      *(undefined2 *)(this + 0x442) = (undefined2)local_20;
      *(uint *)(this + 0xd10) = local_1c & 0xffff;
      *(uint *)(this + 0xd14) = local_20 & 0xffff;
      iVar4 = DAT_00022b88;
      iVar3 = DAT_00022b84;
      if ((*(int *)(this + 0x690) != 0x280) || (*(int *)(this + 0x694) != 0x1e0)) {
        uVar6 = 0x280;
        *(undefined4 *)(this + 0x694) = 0x1e0;
        iVar3 = iVar3 + 0x22b38;
        uVar7 = 0x1e0;
        iVar4 = iVar4 + 0x22b3c;
        *(undefined4 *)(this + 0x690) = 0x280;
LAB_00022b42:
        __android_log_print(2,iVar3,iVar4,uVar6,uVar7,uVar8);
        this[0x698] = (QualcommCameraHardware)0x1;
      }
    }
LAB_00022b4c:
    uVar1 = 0;
  }
  return uVar1;
}



// Function: FUN_00022b8c @ 00022b8c

undefined8 FUN_00022b8c(char *param_1,long *param_2,long *param_3)

{
  long lVar1;
  undefined4 uVar2;
  long lVar3;
  long *local_1c;
  long *plStack_18;
  
  local_1c = param_2;
  plStack_18 = param_3;
  lVar1 = strtol(param_1,(char **)&local_1c,10);
  if ((char)*local_1c == 'X' || (char)*local_1c == 'x') {
    lVar3 = strtol((char *)((int)local_1c + 1),(char **)0x0,10);
    *param_2 = lVar1;
    *param_3 = lVar3;
    uVar2 = 0;
  }
  else {
    uVar2 = 0xffffffff;
  }
  return CONCAT44(param_1,uVar2);
}



// Function: setRecordSize @ 00022bcc

/* android::QualcommCameraHardware::setRecordSize(android::CameraParameters const&) */

undefined4 __thiscall
android::QualcommCameraHardware::setRecordSize
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  QualcommCameraHardware *pQVar4;
  int iVar5;
  int iVar6;
  bool bVar7;
  
  iVar5 = DAT_00022e14 + 0x22bda;
  iVar1 = android::CameraParameters::get((char *)param_1);
  iVar3 = iVar1;
  if (iVar1 == 0) {
    iVar3 = DAT_00022e1c + 0x22bf0;
  }
  __android_log_print(6,DAT_00022e20 + 0x22bf8,DAT_00022e24 + 0x22bfa,iVar3);
  iVar2 = android::CameraParameters::get((char *)param_1);
  iVar3 = iVar2;
  if (iVar2 == 0) {
    iVar3 = DAT_00022e2c + 0x22c12;
  }
  __android_log_print(6,DAT_00022e30 + 0x22c1e,DAT_00022e34 + 0x22c20,iVar3);
  if (iVar2 == 0) {
    android::CameraParameters::set((char *)(this + 0x18),(char *)(DAT_00022e38 + 0x22c32));
  }
  else {
    android::CameraParameters::set((char *)(this + 0x18),(char *)(DAT_00022e40 + 0x22c42));
    iVar1 = android::CameraParameters::get((char *)param_1);
  }
  if (iVar1 == 0) {
    iVar3 = *(int *)(this + 0xd10);
    iVar1 = DAT_00022e44 + 0x22c64;
    __android_log_print(4,iVar1,DAT_00022e48 + 0x22c66,DAT_00022e4c + 0x22c6a,iVar3,
                        *(undefined4 *)(this + 0xd14));
    android::CameraParameters::set((char *)(this + 0x18),*(char **)(iVar5 + DAT_00022e18));
    __android_log_print(2,iVar1,DAT_00022e54 + 0x22c8c);
    *(undefined4 *)(this + 0xd2c) = *(undefined4 *)(this + 0xd10);
    *(undefined4 *)(this + 0xd30) = *(undefined4 *)(this + 0xd14);
  }
  else {
    pQVar4 = this + 0x18;
    iVar6 = DAT_00022e58 + 0x22cb2;
    iVar3 = iVar1;
    __android_log_print(4,iVar6,DAT_00022e5c + 0x22cb4,DAT_00022e60 + 0x22cb8,iVar1);
    iVar2 = FUN_00022b8c(iVar1,this + 0xd2c,this + 0xd30);
    if (iVar2 != 0) {
      android::CameraParameters::set((char *)pQVar4,*(char **)(iVar5 + DAT_00022e18));
      __android_log_print(6,iVar6,DAT_00022e74 + 0x22d7c,iVar1,iVar3);
      return 0xffffffea;
    }
    android::CameraParameters::set((char *)pQVar4,*(char **)(iVar5 + DAT_00022e18));
    if ((*(int *)(this + 0xd2c) < *(int *)(this + 0xd10)) ||
       (*(int *)(this + 0xd30) < *(int *)(this + 0xd14))) {
      iVar3 = *(int *)(this + 0xd14);
      __android_log_print(4,DAT_00022e64 + 0x22d06,DAT_00022e68 + 0x22d0a,*(int *)(this + 0xd10),
                          iVar3,*(int *)(this + 0xd2c),*(undefined4 *)(this + 0xd30));
      *(int *)(this + 0xd10) = *(int *)(this + 0xd2c);
      *(undefined4 *)(this + 0xd14) = *(undefined4 *)(this + 0xd30);
      android::CameraParameters::setPreviewSize((int)pQVar4,*(int *)(this + 0xd2c));
    }
    if (2 < *(int *)(DAT_00022e6c + 0x22d2c) - 4U) {
      *(int *)(this + 0xd10) = *(int *)(this + 0xd2c);
      *(undefined4 *)(this + 0xd14) = *(undefined4 *)(this + 0xd30);
      android::CameraParameters::setPreviewSize((int)pQVar4,*(int *)(this + 0xd2c));
    }
    if (this[0xcec] != (QualcommCameraHardware)0x0) {
      *(int *)(this + 0xd10) = *(int *)(this + 0xd2c);
      *(undefined4 *)(this + 0xd14) = *(undefined4 *)(this + 0xd30);
      android::CameraParameters::setPreviewSize((int)pQVar4,*(int *)(this + 0xd2c));
    }
  }
  iVar1 = DAT_00022e78;
  iVar5 = *(int *)(this + 0xd2c);
  if (iVar5 == 0x500) {
    bVar7 = *(int *)(this + 0xd30) == 0x2d0;
  }
  else {
    if (iVar5 != 0x780) goto LAB_00022dde;
    bVar7 = *(int *)(this + 0xd30) == 0x438;
  }
  if ((bVar7) &&
     ((*(int *)(this + 0x690) != iVar5 || (*(int *)(this + 0x694) != *(int *)(this + 0xd30))))) {
    *(int *)(this + 0x690) = iVar5;
    *(undefined4 *)(this + 0x694) = *(undefined4 *)(this + 0xd30);
    __android_log_print(2,iVar1 + 0x22dd2,DAT_00022e7c + 0x22dd6,iVar5,iVar3);
    this[0x698] = (QualcommCameraHardware)0x1;
  }
LAB_00022dde:
  __android_log_print(4,DAT_00022e80 + 0x22dec,DAT_00022e84 + 0x22df2,DAT_00022e88 + 0x22df8,
                      *(undefined4 *)(this + 0xd2c),*(undefined4 *)(this + 0xd30));
  *(undefined2 *)(this + 0x440) = *(undefined2 *)(this + 0xd10);
  *(undefined2 *)(this + 0x442) = *(undefined2 *)(this + 0xd14);
  return 0;
}



// Function: setParameters @ 00022e8c

/* android::QualcommCameraHardware::setParameters(android::CameraParameters const&) */

int __thiscall
android::QualcommCameraHardware::setParameters
          (QualcommCameraHardware *this,CameraParameters *param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  
  iVar4 = DAT_00023110 + 0x22e9a;
  __android_log_print(2,iVar4,DAT_00023114 + 0x22e9e);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x400));
  if (this[0x3ac] == (QualcommCameraHardware)0x0) {
    iVar1 = setCameraMode(this,param_1);
    iVar4 = setFirmwareUpdate(this,param_1);
    if (iVar4 == 0) {
      iVar4 = iVar1;
    }
    iVar1 = setVtMode(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setVtSurface(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setPreviewFrameRate((CameraParameters *)this);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setMovieMode(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setPreviewSize(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setRecordSize(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setPreviewFormat((CameraParameters *)this);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setPictureSize(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setPictureFormat(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setRotation(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setJpegQuality(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setOrientation(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setJpegThumbnailSize(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setFocusMode(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    if (*(int *)(DAT_00023130 + 0x2301c) == 0) {
      iVar1 = setSceneMode(this,param_1);
      if (iVar1 != 0) {
        iVar4 = iVar1;
      }
      iVar1 = setISOValue(this,param_1);
      if (iVar1 != 0) {
        iVar4 = iVar1;
      }
      iVar1 = setWhiteBalance(this,param_1);
      if (iVar1 != 0) {
        iVar4 = iVar1;
      }
      iVar1 = setFlash(this,param_1);
      if (iVar1 != 0) {
        iVar4 = iVar1;
      }
      iVar1 = setAutoExposure(this,param_1);
      if (iVar1 != 0) {
        iVar4 = iVar1;
      }
      iVar1 = setEffect(this,param_1);
      if (iVar1 != 0) {
        iVar4 = iVar1;
      }
      iVar1 = setZoom(this,param_1);
      if (iVar1 != 0) {
        iVar4 = iVar1;
      }
      iVar1 = setAutoContrast((CameraParameters *)this);
      if (iVar1 != 0) {
        iVar4 = iVar1;
      }
      iVar1 = setAntiShakeMode(this,param_1);
      if (iVar1 != 0) {
        iVar4 = iVar1;
      }
      iVar1 = setFaceBeauty(this,param_1);
    }
    else {
      iVar1 = setBlur(this,param_1);
    }
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setExposureCompensation(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setGpsLocation(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setAppShutterSound(this,param_1);
    if (iVar1 == 0) {
      if (iVar4 == 0) {
        iVar3 = DAT_0002313c + 0x230f2;
      }
      else {
        iVar3 = DAT_00023138 + 0x230ec;
      }
    }
    else {
      iVar3 = DAT_00023134 + 0x230e4;
      iVar4 = iVar1;
    }
    iVar1 = DAT_00023140 + 0x230f8;
    iVar2 = DAT_00023144 + 0x230fa;
  }
  else {
    __android_log_print(2,iVar4,DAT_00023118 + 0x22eca);
    iVar1 = setCameraMode(this,param_1);
    iVar4 = setPreviewSize(this,param_1);
    if (iVar4 == 0) {
      iVar4 = iVar1;
    }
    iVar1 = setRecordSize(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setPictureSize(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setJpegThumbnailSize(this,param_1);
    if (iVar1 != 0) {
      iVar4 = iVar1;
    }
    iVar1 = setJpegQuality(this,param_1);
    if (iVar1 == 0) {
      if (iVar4 == 0) {
        iVar3 = DAT_00023124 + 0x22f30;
      }
      else {
        iVar3 = DAT_00023120 + 0x22f2a;
      }
    }
    else {
      iVar3 = DAT_0002311c + 0x22f22;
      iVar4 = iVar1;
    }
    iVar1 = DAT_00023128 + 0x22f36;
    iVar2 = DAT_0002312c + 0x22f38;
  }
  __android_log_print(2,iVar1,iVar2,iVar3);
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x400));
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return iVar4;
}



// Function: receiveJpegPicture @ 00023148

/* android::QualcommCameraHardware::receiveJpegPicture(int, mm_camera_buffer_t*) */

void __thiscall
android::QualcommCameraHardware::receiveJpegPicture
          (QualcommCameraHardware *this,int param_1,mm_camera_buffer_t *param_2)

{
  int iVar1;
  uint uVar2;
  undefined4 uVar3;
  int iVar4;
  uint uVar5;
  mm_camera_buffer_t *pmVar6;
  
  uVar5 = 1 - param_1;
  if (1 < (uint)param_1) {
    uVar5 = 0;
  }
  pmVar6 = param_2;
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x404));
  if (param_2 == (mm_camera_buffer_t *)0x0) {
    uVar2 = 0;
  }
  else {
    uVar2 = uVar5 & 1;
  }
  *(int *)(this + 0x10) = *(int *)(this + 0x10) + 1;
  if (uVar2 == 0) {
LAB_000231b0:
    __android_log_print(6,DAT_000232c0 + 0x231ba,DAT_000232c4 + 0x231bc);
    if ((*(code **)(this + 0xcf8) != (code *)0x0) && (*(int *)(this + 0xcf0) << 0x17 < 0)) {
      (**(code **)(this + 0xcf8))(0x100,0,0,0,*(undefined4 *)(this + 0xd04));
    }
  }
  else {
    uVar3 = *(undefined4 *)(this + 0x430);
    iVar4 = DAT_000232b4 + 0x23188;
    __android_log_print(2,iVar4,DAT_000232b8 + 0x2318c,*(undefined4 *)(param_2 + 4),uVar3,param_1,
                        pmVar6);
    uVar2 = mapJpegBuffer(this,param_2);
    __android_log_print(6,iVar4,DAT_000232bc + 0x231a2,uVar2);
    if (2 < uVar2) goto LAB_000231b0;
    __android_log_print(2,iVar4,DAT_000232c8 + 0x231ee,uVar2,uVar3,param_1,pmVar6);
    if ((*(int *)(this + 0xcf8) == 0) || (-1 < *(int *)(this + 0xcf0) << 0x17)) {
      __android_log_print(6,DAT_000232d8 + 0x23272,DAT_000232dc + 0x23274);
    }
    else if (uVar5 != 0) {
      __android_log_print(6,iVar4,DAT_000232cc + 0x2320e,*(int *)(this + 0xcf0),uVar3);
      iVar1 = (**(code **)(this + 0xd00))
                        (0xffffffff,*(undefined4 *)(param_2 + 4),1,*(undefined4 *)(this + 0xd04));
      *(int *)(this + 0x7b0) = iVar1;
      if (iVar1 == 0) {
        __android_log_print(6,iVar4,DAT_000232d0 + 0x23234,DAT_000232d4 + 0x23236);
      }
      memcpy((void *)**(undefined4 **)(this + 0x7b0),
             (void *)**(undefined4 **)(this + uVar2 * 4 + 0x764),*(size_t *)(param_2 + 4));
      (**(code **)(this + 0xcf8))
                (0x100,*(undefined4 *)(this + 0x7b0),0,0,*(undefined4 *)(this + 0xd04));
    }
    if (*(int *)(this + 0x10) != *(int *)(this + 0xc)) goto LAB_0002329c;
  }
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3c0));
  this[0x3bc] = (QualcommCameraHardware)0x0;
  pthread_cond_signal((pthread_cond_t *)(this + 0x3c4));
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3c0));
LAB_0002329c:
  __android_log_print(2,DAT_000232e0 + 0x232a6,DAT_000232e4 + 0x232a8);
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x404));
  return;
}



// Function: receiveCameraStats @ 000232e8

/* android::QualcommCameraHardware::receiveCameraStats(camstats_type,
   camera_preview_histogram_info*) */

void __thiscall
android::QualcommCameraHardware::receiveCameraStats
          (QualcommCameraHardware *this,undefined4 param_2,void *param_3)

{
  int extraout_r1;
  code *pcVar1;
  pthread_mutex_t *__mutex;
  undefined4 uVar2;
  uint uVar3;
  
  if (this[0x34] == (QualcommCameraHardware)0x0) {
    __android_log_print(6,DAT_000233b0 + 0x23300,DAT_000233b4 + 0x23302,0,this,param_2);
  }
  else {
    pthread_mutex_lock((pthread_mutex_t *)(this + 0x404));
    __mutex = (pthread_mutex_t *)(this + 0x390);
    uVar3 = *(uint *)(this + 0xcf0);
    pcVar1 = *(code **)(this + 0xcf8);
    uVar2 = *(undefined4 *)(this + 0xd04);
    pthread_mutex_unlock((pthread_mutex_t *)(this + 0x404));
    pthread_mutex_lock(__mutex);
    if ((*(int *)(this + 900) == 0) || (this[0x38c] == (QualcommCameraHardware)0x0)) {
      pthread_mutex_unlock(__mutex);
    }
    else {
      this[0x38c] = (QualcommCameraHardware)0x0;
      __aeabi_idivmod(*(int *)(this + 0x388) + 1,3);
      *(int *)(this + 0x388) = extraout_r1;
      *(undefined4 *)**(undefined4 **)(this + (extraout_r1 + 0x1e0) * 4) =
           *(undefined4 *)((int)param_3 + 0x400);
      memcpy((void *)(**(int **)(this + (*(int *)(this + 0x388) + 0x1e0) * 4) + 4),param_3,0x400);
      pthread_mutex_unlock(__mutex);
      if ((pcVar1 != (code *)0x0) && ((uVar3 & 0x800) != 0)) {
        (*pcVar1)(0x800,*(undefined4 *)(this + (*(int *)(this + 0x388) + 0x1e0) * 4),0,0,uVar2);
      }
    }
  }
  return;
}



// Function: do_copy @ 000233b8

/* android::Vector<msm_frame*>::do_copy(void*, void const*, unsigned int) const */

void __thiscall
android::Vector<msm_frame*>::do_copy
          (Vector<msm_frame*> *this,void *param_1,void *param_2,uint param_3)

{
  memcpy(param_1,param_2,param_3 << 2);
  return;
}



// Function: setExifFixedAttribute @ 000233c8

/* android::QualcommCameraHardware::setExifFixedAttribute() */

void __thiscall android::QualcommCameraHardware::setExifFixedAttribute(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  int *piVar3;
  char *pcVar4;
  undefined4 extraout_r0;
  undefined4 extraout_r0_00;
  undefined4 extraout_r0_01;
  undefined4 extraout_r1;
  undefined4 extraout_r1_00;
  undefined4 extraout_r1_01;
  undefined4 uVar5;
  int iVar6;
  float fVar7;
  undefined4 extraout_s1;
  double dVar8;
  double dVar9;
  double dVar10;
  undefined4 local_28 [3];
  QualcommCameraHardware *local_1c;
  
  strcpy((char *)(this + 0x4f),(char *)(DAT_00023614 + 0x233d8));
  property_get(DAT_00023618 + 0x233ea,this + 0x6f,DAT_0002361c + 0x233ec);
  property_get(DAT_00023620 + 0x233fa,this + 0x8f,DAT_00023624 + 0x233fc);
  iVar2 = DAT_00023628;
  this[0x4e] = (QualcommCameraHardware)0x0;
  *(undefined2 *)(this + 0x176) = 1;
  iVar1 = DAT_00023630;
  if (*(int *)(iVar2 + 0x23412) == 0) {
    local_1c = this + 0x15d;
    iVar6 = DAT_0002362c + 0x23428;
    this[0x4e] = (QualcommCameraHardware)0x1;
    local_28[0] = 0x21;
    __android_log_print(2,iVar6,iVar1 + 0x23430,0x21);
    iVar2 = (**(code **)(DAT_00023634 + 0x23450))(local_28);
    if (iVar2 != 0) {
      piVar3 = (int *)__errno();
      pcVar4 = strerror(*piVar3);
      __android_log_print(6,iVar6,DAT_00023638 + 0x23456,0x21,pcVar4);
    }
  }
  if (*(int *)(DAT_0002363c + 0x23462) == 0) {
    uVar5 = 100;
    *(undefined4 *)(this + 400) = 0x109;
  }
  else {
    uVar5 = 10;
    *(undefined4 *)(this + 400) = 0x1c;
  }
  iVar2 = DAT_00023640;
  *(undefined4 *)(this + 0x194) = uVar5;
  *(undefined2 *)(this + 0x178) = 3;
  memcpy(this + 0xaf,(void *)(iVar2 + 0x2348c),4);
  dVar9 = DAT_00023600;
  dVar8 = log((double)CONCAT44(extraout_s1,*(undefined4 *)(this + 400)));
  dVar10 = (double)CONCAT44(extraout_r1,extraout_r0) / dVar9;
  if (dVar10 + dVar10 < 0.0) {
    dVar8 = log(dVar8);
    fVar7 = SUB84(dVar8,0);
    dVar9 = (double)CONCAT44(extraout_r1_01,extraout_r0_01) / dVar9;
    dVar9 = (dVar9 + dVar9) * DAT_00023608 - 0.5;
  }
  else {
    dVar8 = log(dVar8);
    fVar7 = SUB84(dVar8,0);
    dVar9 = (double)CONCAT44(extraout_r1_00,extraout_r0_00) / dVar9;
    dVar9 = (dVar9 + dVar9) * DAT_00023608 + 0.5;
  }
  iVar1 = DAT_00023644;
  dVar8 = DAT_00023608;
  dVar9 = dVar9 / DAT_00023608;
  *(undefined4 *)(this + 0x19c) = 100;
  *(undefined4 *)(this + 0x1a4) = 100;
  uVar5 = 0x10e;
  dVar9 = dVar9 * dVar8;
  iVar2 = (uint)(0.0 < dVar9) * (int)(longlong)dVar9;
  *(int *)(this + 0x198) = iVar2;
  *(int *)(this + 0x1a0) = iVar2;
  if (*(int *)(iVar1 + 0x23556) == 0) {
    if (this[0x15d] == (QualcommCameraHardware)0x4f) {
      uVar5 = 0x18d;
    }
    else {
      uVar5 = 0x193;
    }
  }
  android::CameraParameters::setFloat((char *)(this + 0x18),fVar7);
  *(undefined4 *)(this + 0x1ac) = 100;
  iVar2 = DAT_0002364c;
  *(undefined4 *)(this + 0x1a8) = uVar5;
  strcpy((char *)(this + 199),(char *)(iVar2 + 0x235a4));
  *(undefined2 *)(this + 0x180) = 1;
  *(undefined2 *)(this + 0x2ca) = 6;
  *(undefined4 *)(this + 700) = 1;
  *(undefined4 *)(this + 0x2c4) = 1;
  *(undefined4 *)(this + 0x170) = 0xf0;
  iVar1 = DAT_00023654;
  iVar2 = DAT_00023650;
  *(undefined2 *)(this + 0x182) = 0;
  *(undefined4 *)(this + 0x2b8) = 0x48;
  *(undefined4 *)(this + 0x2c0) = 0x48;
  *(undefined2 *)(this + 0x2c8) = 2;
  *(undefined4 *)(this + 0x16c) = 0x140;
  __android_log_print(2,iVar2 + 0x235d8,iVar1 + 0x235e6);
  return;
}



// Function: checkAreaParameters @ 00023658

/* android::checkAreaParameters(char const*) */

undefined4 android::checkAreaParameters(char *param_1)

{
  char cVar1;
  size_t sVar2;
  long lVar3;
  char *pcVar4;
  int iVar6;
  int iVar7;
  int iVar8;
  long *plVar9;
  long lStack_38;
  int local_34 [4];
  int local_24;
  char *local_1c [2];
  char *pcVar5;
  
  iVar7 = 0;
  pcVar5 = param_1;
  while( true ) {
    pcVar4 = pcVar5 + 1;
    cVar1 = *pcVar5;
    if (cVar1 == '\0') break;
    pcVar5 = pcVar4;
    if (cVar1 == ',') {
      iVar7 = iVar7 + 1;
    }
  }
  if (iVar7 < 5) {
    if ((*param_1 == '(') && (sVar2 = strlen(param_1), param_1[sVar2 - 1] == ')')) {
      plVar9 = &lStack_38;
      iVar7 = 0;
      pcVar5 = param_1;
      while( true ) {
        lVar3 = strtol(pcVar5 + 1,local_1c,10);
        plVar9 = plVar9 + 1;
        *plVar9 = lVar3;
        if ((*local_1c[0] != ',') && (iVar7 != 4)) break;
        iVar7 = iVar7 + 1;
        pcVar5 = local_1c[0];
        if (iVar7 == 5) {
          __android_log_print(2,DAT_00023784 + 0x236f8,DAT_00023788 + 0x23706,DAT_0002378c + 0x2370c
                              ,local_34[0],local_34[1],local_34[2],local_34[3],local_24);
          if (local_34[0] < -1000) {
            return 0xffffffff;
          }
          if (local_34[1] < -1000) {
            return 0xffffffff;
          }
          if (1000 < local_34[2]) {
            return 0xffffffff;
          }
          if (1000 < local_34[3]) {
            return 0xffffffff;
          }
          if (999 < local_24 - 1U) {
            return 0xffffffff;
          }
          if (local_34[2] <= local_34[0]) {
            return 0xffffffff;
          }
          if (local_34[3] <= local_34[1]) {
            return 0xffffffff;
          }
          return 0;
        }
      }
      __android_log_print(6,DAT_0002377c + 0x236d6,DAT_00023780 + 0x236d8,0x2c,param_1,*local_1c[0])
      ;
    }
    else {
      __android_log_print(6,DAT_00023774 + 0x236a6,DAT_00023778 + 0x236a8,param_1);
    }
    iVar7 = DAT_00023790 + 0x23756;
    iVar6 = DAT_00023794 + 0x23758;
    iVar8 = DAT_00023798 + 0x2375a;
  }
  else {
    iVar7 = DAT_00023768 + 0x23680;
    iVar6 = DAT_0002376c + 0x23682;
    iVar8 = DAT_00023770 + 0x23684;
  }
  __android_log_print(6,iVar7,iVar6,iVar8,param_1);
  return 0xffffffff;
}



// Function: setFocusAreas @ 0002379c

/* android::QualcommCameraHardware::setFocusAreas(android::CameraParameters const&) */

undefined8 android::QualcommCameraHardware::setFocusAreas(CameraParameters *param_1)

{
  int iVar1;
  undefined4 uVar2;
  char *in_r1;
  char *pcVar3;
  CameraParameters *local_20;
  
  pcVar3 = *(char **)(DAT_00023830 + 0x237a8);
  local_20 = (CameraParameters *)android::CameraParameters::get(in_r1);
  if ((local_20 == (CameraParameters *)0x0) ||
     (iVar1 = strcmp((char *)local_20,(char *)(DAT_00023834 + 0x237b8)), iVar1 == 0)) {
    __android_log_print(6,DAT_00023838 + 0x237c8,DAT_0002383c + 0x237ca,DAT_00023840 + 0x237cc);
  }
  else {
    iVar1 = strcmp((char *)(DAT_00023844 + 0x237d8),(char *)local_20);
    if ((iVar1 != 0) &&
       (iVar1 = strcmp((char *)(DAT_00023848 + 0x237e4),(char *)local_20), iVar1 != 0)) {
      iVar1 = checkAreaParameters((char *)local_20);
      if (iVar1 == 0) {
        android::CameraParameters::set((char *)(param_1 + 0x18),pcVar3);
        uVar2 = 0;
        local_20 = param_1;
      }
      else {
        __android_log_print(6,DAT_00023850 + 0x23812,DAT_00023854 + 0x23814,DAT_00023858 + 0x23818);
        uVar2 = 0xffffffea;
      }
      goto LAB_0002382e;
    }
    android::CameraParameters::set((char *)(param_1 + 0x18),*(int *)(DAT_0002384c + 0x237f4));
  }
  uVar2 = 0;
  local_20 = param_1;
LAB_0002382e:
  return CONCAT44(local_20,uVar2);
}



// Function: setMeteringAreas @ 0002385c

/* android::QualcommCameraHardware::setMeteringAreas(android::CameraParameters const&) */

undefined8 android::QualcommCameraHardware::setMeteringAreas(CameraParameters *param_1)

{
  int iVar1;
  undefined4 uVar2;
  char *in_r1;
  char *pcVar3;
  CameraParameters *local_20;
  
  pcVar3 = *(char **)(DAT_000238f0 + 0x23868);
  local_20 = (CameraParameters *)android::CameraParameters::get(in_r1);
  if ((local_20 == (CameraParameters *)0x0) ||
     (iVar1 = strcmp((char *)local_20,(char *)(DAT_000238f4 + 0x23878)), iVar1 == 0)) {
    __android_log_print(6,DAT_000238f8 + 0x23888,DAT_000238fc + 0x2388a,DAT_00023900 + 0x2388c);
  }
  else {
    iVar1 = strcmp((char *)(DAT_00023904 + 0x23898),(char *)local_20);
    if ((iVar1 != 0) &&
       (iVar1 = strcmp((char *)(DAT_00023908 + 0x238a4),(char *)local_20), iVar1 != 0)) {
      iVar1 = checkAreaParameters((char *)local_20);
      if (iVar1 == 0) {
        android::CameraParameters::set((char *)(param_1 + 0x18),pcVar3);
        uVar2 = 0;
        local_20 = param_1;
      }
      else {
        __android_log_print(6,DAT_00023910 + 0x238d2,DAT_00023914 + 0x238d4,DAT_00023918 + 0x238d8);
        uVar2 = 0xffffffea;
      }
      goto LAB_000238ee;
    }
    android::CameraParameters::set((char *)(param_1 + 0x18),*(int *)(DAT_0002390c + 0x238b4));
  }
  uVar2 = 0;
  local_20 = param_1;
LAB_000238ee:
  return CONCAT44(local_20,uVar2);
}



// Function: setExifChangedAttribute @ 00023920

/* android::QualcommCameraHardware::setExifChangedAttribute() */

void __thiscall
android::QualcommCameraHardware::setExifChangedAttribute(QualcommCameraHardware *this)

{
  int iVar1;
  tm *__tp;
  undefined4 extraout_r0;
  undefined4 extraout_r0_00;
  undefined4 extraout_r0_01;
  float fVar2;
  char *pcVar3;
  char *__nptr;
  char *__nptr_00;
  undefined4 extraout_r0_02;
  undefined4 extraout_r0_03;
  undefined4 extraout_r0_04;
  long lVar4;
  size_t __n;
  undefined4 uVar5;
  undefined4 extraout_r1;
  undefined4 extraout_r1_00;
  undefined4 extraout_r1_01;
  undefined4 extraout_r1_02;
  char *pcVar6;
  undefined4 extraout_r1_03;
  undefined4 extraout_r1_04;
  QualcommCameraHardware QVar7;
  undefined2 uVar8;
  int iVar9;
  QualcommCameraHardware *pQVar10;
  undefined4 uVar11;
  undefined4 extraout_s1;
  double dVar12;
  undefined4 extraout_s2;
  undefined4 extraout_s2_00;
  undefined4 extraout_s3;
  undefined4 extraout_s3_00;
  tm local_78;
  time_t local_4c;
  undefined4 local_48;
  undefined2 local_44 [2];
  int local_40;
  time_t tStack_3c;
  undefined4 local_38;
  int local_34 [2];
  
  pQVar10 = this + 0x18;
  __android_log_print(2,DAT_00023bf8 + 0x23934,DAT_00023bfc + 0x23936);
  android::CameraParameters::getPictureSize((int *)pQVar10,local_34);
  *(int *)(this + 0x164) = local_34[0];
  *(undefined4 *)(this + 0x168) = local_38;
  iVar1 = android::CameraParameters::getInt((char *)pQVar10);
  if (iVar1 == 0x5a) {
    uVar8 = 6;
LAB_00023984:
    *(undefined2 *)(this + 0x174) = uVar8;
  }
  else {
    if (iVar1 < 0x5b) {
LAB_0002398a:
      uVar8 = 1;
    }
    else {
      if (iVar1 != 0xb4) {
        if (iVar1 == 0x10e) {
          uVar8 = 8;
          goto LAB_00023984;
        }
        goto LAB_0002398a;
      }
      uVar8 = 3;
    }
    *(undefined2 *)(this + 0x174) = uVar8;
  }
  time(&tStack_3c);
  __tp = localtime(&tStack_3c);
  strftime((char *)(this + 0xb3),0x14,(char *)(DAT_00023c08 + 0x239ac),__tp);
  uVar11 = FUN_0001f1c8(0x24,1,&local_40);
  iVar9 = DAT_00023c0c;
  *(undefined4 *)(this + 0x188) = 1;
  iVar1 = local_40;
  if (*(int *)(iVar9 + 0x239cc) == 0) {
    dVar12 = pow((double)CONCAT44(extraout_s1,uVar11),(double)CONCAT44(extraout_s3,extraout_s2));
    if ((double)CONCAT44(extraout_r1,extraout_r0) < 0.0) {
      pow(dVar12,(double)CONCAT44(extraout_s3_00,extraout_s2_00));
      dVar12 = (double)CONCAT44(extraout_r1_01,extraout_r0_01) - 0.5;
    }
    else {
      pow(dVar12,(double)CONCAT44(extraout_s3_00,extraout_s2_00));
      dVar12 = (double)CONCAT44(extraout_r1_00,extraout_r0_00) + 0.5;
    }
    iVar1 = (uint)(0.0 < dVar12) * (int)(longlong)dVar12;
  }
  *(int *)(this + 0x18c) = iVar1;
  FUN_0001f1c8(0x24,5,local_44);
  iVar1 = DAT_00023c10;
  *(undefined2 *)(this + 0x17a) = local_44[0];
  if (*(int *)(iVar1 + 0x23a62) == 0) {
    *(undefined4 *)(this + 0x1b4) = 100;
    *(int *)(this + 0x1b0) = local_40;
    FUN_0001f1c8(0x24,3,&local_4c);
    *(undefined4 *)(this + 0x1bc) = 100;
    *(time_t *)(this + 0x1b8) = local_4c;
    FUN_0001f1c8(0x24,6,local_44);
    *(undefined2 *)(this + 0x17e) = local_44[0];
  }
  iVar1 = android::CameraParameters::getInt((char *)pQVar10);
  fVar2 = (float)android::CameraParameters::getFloat((char *)pQVar10);
  iVar9 = *(int *)(this + 0x658);
  *(undefined4 *)(this + 0x1c4) = 100;
  if (iVar9 == 1) {
    *(undefined2 *)(this + 0x17c) = 2;
  }
  *(int *)(this + 0x1c0) = (int)((float)(longlong)iVar1 * fVar2) * 100;
  if (iVar9 != 1) {
    if (iVar9 == 2) {
      uVar8 = 3;
    }
    else {
      uVar8 = 1;
    }
    *(undefined2 *)(this + 0x17c) = uVar8;
  }
  iVar1 = *(int *)(this + 0x664);
  *(ushort *)(this + 0x184) = (ushort)(*(int *)(this + 0x650) != 1);
  if (iVar1 != 3) {
    if (iVar1 == 5) {
      *(undefined2 *)(this + 0x186) = 1;
      goto LAB_00023b22;
    }
    if (iVar1 != 2) {
      iVar1 = 0;
    }
  }
  *(short *)(this + 0x186) = (short)iVar1;
LAB_00023b22:
  local_48 = *(undefined4 *)(DAT_00023c20 + 0x23b34);
  memcpy(this + 0x1cc,&local_48,4);
  pcVar3 = (char *)android::CameraParameters::get((char *)pQVar10);
  __nptr = (char *)android::CameraParameters::get((char *)pQVar10);
  __nptr_00 = (char *)android::CameraParameters::get((char *)pQVar10);
  if ((__nptr == (char *)0x0 || pcVar3 == (char *)0x0) || (__nptr_00 == (char *)0x0)) {
    QVar7 = (QualcommCameraHardware)0x0;
  }
  else {
    strtod(pcVar3,(char **)0x0);
    if ((double)CONCAT44(extraout_r1_02,extraout_r0_02) <= 0.0) {
      pcVar6 = (char *)(DAT_00023c30 + 0x23baa);
    }
    else {
      pcVar6 = (char *)(DAT_00023c2c + 0x23ba4);
    }
    strcpy((char *)(this + 0x1c8),pcVar6);
    strtod(__nptr,(char **)0x0);
    if ((double)CONCAT44(extraout_r1_03,extraout_r0_03) <= 0.0) {
      pcVar6 = (char *)(DAT_00023c38 + 0x23bd2);
    }
    else {
      pcVar6 = (char *)(DAT_00023c34 + 0x23bcc);
    }
    strcpy((char *)(this + 0x1ca),pcVar6);
    strtod(__nptr_00,(char **)0x0);
    this[0x1d0] = (QualcommCameraHardware)((double)CONCAT44(extraout_r1_04,extraout_r0_04) <= 0.0);
    dVar12 = strtod(pcVar3,(char **)0x0);
    lVar4 = lround(dVar12);
    dVar12 = strtod(__nptr,(char **)0x0);
    iVar1 = (int)(longlong)(double)(longlong)lVar4;
    lVar4 = lround(dVar12);
    dVar12 = strtod(__nptr_00,(char **)0x0);
    if (iVar1 < 0) {
      iVar1 = -iVar1;
    }
    iVar9 = (int)(longlong)(double)(longlong)lVar4;
    lVar4 = lround(dVar12);
    uVar11 = DAT_00023de8;
    *(undefined4 *)(this + 0x1e0) = 1;
    *(undefined4 *)(this + 0x1e8) = 1;
    *(undefined4 *)(this + 0x1f8) = 1;
    *(undefined4 *)(this + 0x200) = 1;
    *(int *)(this + 0x1d4) = iVar1;
    *(undefined4 *)(this + 0x1d8) = uVar11;
    *(undefined4 *)(this + 0x1dc) = 0;
    *(undefined4 *)(this + 0x1e4) = 0;
    *(undefined4 *)(this + 0x1f0) = uVar11;
    *(undefined4 *)(this + 500) = 0;
    *(undefined4 *)(this + 0x1fc) = 0;
    if (iVar9 < 0) {
      iVar9 = -iVar9;
    }
    *(int *)(this + 0x1ec) = iVar9;
    *(undefined4 *)(this + 0x208) = 100;
    iVar1 = (int)(longlong)(double)(longlong)lVar4;
    if (iVar1 < 0) {
      iVar1 = -iVar1;
    }
    *(int *)(this + 0x204) = iVar1;
    pcVar3 = (char *)android::CameraParameters::get((char *)pQVar10);
    local_4c = 0;
    if (pcVar3 != (char *)0x0) {
      local_4c = atol(pcVar3);
    }
    gmtime_r(&local_4c,&local_78);
    *(int *)(this + 0x214) = local_78.tm_min;
    iVar1 = DAT_00023df0;
    *(int *)(this + 0x20c) = local_78.tm_hour;
    *(undefined4 *)(this + 0x210) = 1;
    *(undefined4 *)(this + 0x218) = 1;
    *(int *)(this + 0x21c) = local_78.tm_sec;
    *(undefined4 *)(this + 0x220) = 1;
    strftime((char *)(this + 0x224),0x14,(char *)(iVar1 + 0x23d4e),&local_78);
    pcVar3 = (char *)android::CameraParameters::get((char *)pQVar10);
    if (pcVar3 != (char *)0x0) {
      __n = strlen(pcVar3);
      memset(this + 0x238,0,0x80);
      if (0x7e < __n) {
        __n = 0x7f;
      }
      strncpy((char *)(this + 0x238),pcVar3,__n);
    }
    QVar7 = (QualcommCameraHardware)0x1;
  }
  this[0x4c] = QVar7;
  uVar11 = android::CameraParameters::getInt((char *)pQVar10);
  uVar5 = android::CameraParameters::getInt((char *)pQVar10);
  *(undefined4 *)(this + 0x16c) = uVar11;
  *(undefined4 *)(this + 0x170) = uVar5;
  return;
}



// Function: getExif @ 00023e00

/* android::QualcommCameraHardware::getExif(unsigned char*, unsigned char*, long) */

undefined4 __thiscall
android::QualcommCameraHardware::getExif
          (QualcommCameraHardware *this,uchar *param_1,uchar *param_2,long param_3)

{
  uint uVar1;
  QualcommCameraHardware QVar2;
  JpegEncoder aJStack_68 [76];
  undefined4 local_1c [2];
  
  android::JpegEncoder::JpegEncoder(aJStack_68);
  if ((*(int *)(this + 0xd40) != 0) && (*(int *)(this + 0xd44) != 0)) {
    uVar1 = 1 - (int)param_2;
    if ((uchar *)0x1 < param_2) {
      uVar1 = 0;
    }
    if (param_3 == 0) {
      uVar1 = uVar1 | 1;
    }
    if (uVar1 == 0) {
      QVar2 = (QualcommCameraHardware)0x1;
      goto LAB_00023e36;
    }
  }
  QVar2 = (QualcommCameraHardware)0x0;
LAB_00023e36:
  this[0x4d] = QVar2;
  setExifChangedAttribute(this);
  if (this[0x4d] == (QualcommCameraHardware)0x0) {
    param_2 = (uchar *)0x0;
    param_3 = 0;
  }
  android::JpegEncoder::makeExif
            ((uchar *)aJStack_68,param_1,(exif_attribute_t *)param_2,(uint *)(this + 0x4c),
             (long)local_1c,SUB41(param_3,0));
  android::JpegEncoder::~JpegEncoder(aJStack_68);
  return local_1c[0];
}



// Function: relinquishBuffers @ 00023e78

/* android::QualcommCameraHardware::relinquishBuffers() */

void __thiscall android::QualcommCameraHardware::relinquishBuffers(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  QualcommCameraHardware *pQVar5;
  QualcommCameraHardware *pQVar6;
  QualcommCameraHardware *pQVar7;
  int iVar8;
  int iVar9;
  int iVar10;
  int iVar11;
  
  iVar4 = DAT_00023fbc + 0x23e88;
  __android_log_print(2,iVar4,DAT_00023fc0 + 0x23e8c,DAT_00023fc4 + 0x23e8e);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f4));
  iVar1 = DAT_00023fc8;
  if (*(int *)(this + 0xcd0) == 0) {
    __android_log_print(2,iVar4,DAT_00023ff4 + 0x23f9a);
  }
  else {
    pQVar7 = this + 0xa1c;
    pQVar5 = this + 0xc64;
    iVar3 = DAT_00023fd4 + 0x23eca;
    iVar4 = DAT_00023fd8 + 0x23ece;
    iVar10 = DAT_00023fcc + 0x23ed4;
    iVar11 = DAT_00023fd0 + 0x23ed6;
    pQVar6 = this + 0x734;
    for (iVar8 = 0; iVar8 < *(int *)(this + 0xd5c); iVar8 = iVar8 + 1) {
      if (*(int *)(pQVar5 + 8) == 1) {
        __android_log_print(6,iVar10,iVar11,*(undefined4 *)pQVar7);
        iVar2 = genlock_unlock_buffer(**(undefined4 **)pQVar5);
        if (iVar2 == 2) {
          __android_log_print(6,iVar10,iVar1 + 0x23f06,iVar3);
        }
        else {
          *(undefined4 *)(pQVar5 + 8) = 0;
        }
      }
      iVar2 = (**(code **)(*(int *)(this + 0xcd0) + 8))
                        (*(int *)(this + 0xcd0),*(undefined4 *)pQVar5);
      (**(code **)(*(int *)(pQVar6 + 4) + 0xc))();
      if ((*(int *)(this + 0x3ec) != 0) && (*(int **)(pQVar6 + 0x80) != (int *)0x0)) {
        native_handle_delete(*(undefined4 *)(**(int **)(pQVar6 + 0x80) + 4));
        __android_log_print(4,iVar4,DAT_00023fdc + 0x23f3e,DAT_00023fe0 + 0x23f42,iVar8);
        (**(code **)(*(int *)(pQVar6 + 0x80) + 0xc))();
        *(undefined4 *)(pQVar6 + 0x80) = 0;
      }
      iVar9 = DAT_00023fe4 + 0x23f60;
      __android_log_print(6,iVar9,DAT_00023fe8 + 0x23f62,iVar8);
      if (iVar2 != 0) {
        __android_log_print(6,iVar9,DAT_00023fec + 0x23f78,DAT_00023ff0 + 0x23f7a,
                            *(undefined4 *)pQVar7);
      }
      pQVar7 = pQVar7 + 0x68;
      pQVar5 = pQVar5 + 0x10;
      pQVar6 = pQVar6 + 4;
    }
  }
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f4));
  __android_log_print(2,DAT_00023ff8 + 0x23fae,DAT_00023ffc + 0x23fb0,DAT_00024000 + 0x23fb2);
  return;
}



// Function: mm_camera_do_mmap @ 00024004

/* android::mm_camera_do_mmap(unsigned int, int*) */

void * __thiscall android::mm_camera_do_mmap(android *this,uint param_1,int *param_2)

{
  char *pcVar1;
  int __fd;
  void *pvVar2;
  int *piVar3;
  undefined4 *puVar4;
  
  if (*(int *)(DAT_000240b4 + 0x2400e) == 6) {
    pcVar1 = (char *)(DAT_000240b8 + 0x2401c);
  }
  else {
    pcVar1 = (char *)(DAT_000240bc + 0x24022);
  }
  __fd = open(pcVar1,DAT_000240b0,param_2,*(int *)(DAT_000240b4 + 0x2400e),this,param_1);
  if (__fd < 1) {
    __android_log_print(6,DAT_000240c0 + 0x24036,DAT_000240c4 + 0x24038);
    pvVar2 = (void *)0x0;
  }
  else {
    pvVar2 = mmap((void *)0x0,(uint)(this + 0xfff) & 0xfffff000,3,1,__fd,0);
    if (pvVar2 == (void *)0xffffffff) {
      piVar3 = (int *)__errno();
      pcVar1 = strerror(*piVar3);
      puVar4 = (undefined4 *)__errno();
      __android_log_print(6,DAT_000240c8 + 0x2407c,DAT_000240cc + 0x2407e,pcVar1,*puVar4);
      close(__fd);
      pvVar2 = (void *)0x0;
    }
    else {
      __android_log_print(6,DAT_000240d0 + 0x2409e,DAT_000240d4 + 0x240a0,__fd,pvVar2,
                          (uint)(this + 0xfff) & 0xfffff000);
      *(int *)param_1 = __fd;
    }
  }
  return pvVar2;
}



// Function: deallocate_ion_memory @ 000240d8

/* android::QualcommCameraHardware::deallocate_ion_memory(int*, ion_fd_data*) */

undefined4 __thiscall
android::QualcommCameraHardware::deallocate_ion_memory
          (QualcommCameraHardware *this,int *param_1,ion_fd_data *param_2)

{
  undefined4 local_c;
  
  local_c = *(undefined4 *)param_2;
  ioctl(*param_1,DAT_000240f8,&local_c,local_c,this);
  close(*param_1);
  return 0;
}



// Function: deinitZslBuffers @ 000240fc

/* android::QualcommCameraHardware::deinitZslBuffers() */

undefined4 __thiscall
android::QualcommCameraHardware::deinitZslBuffers(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  QualcommCameraHardware *pQVar3;
  int iVar4;
  int iVar5;
  
  iVar2 = DAT_00024204;
  iVar4 = DAT_00024200;
  iVar5 = 0;
  __android_log_print(6,DAT_000241f8 + 0x24114,DAT_000241fc + 0x24116);
  pQVar3 = this + 0x750;
  while( true ) {
    if (*(int *)(this + 0xd64) == 0) {
      iVar1 = *(int *)(this + 0xc);
    }
    else {
      iVar1 = 5;
    }
    if (iVar1 <= iVar5) break;
    if (*(int *)pQVar3 != 0) {
      __android_log_print(6,iVar4 + 0x24124,iVar2 + 0x24126);
      FUN_0001f76c(*(undefined4 *)(this + 0x430),*(undefined4 *)(this + 0x428),
                   *(undefined4 *)(this + 0x42c),0,*(undefined4 *)(pQVar3 + -0x6c),0,
                   **(undefined4 **)pQVar3,4,0,0,0);
      (**(code **)(*(int *)pQVar3 + 0xc))();
      *(undefined4 *)pQVar3 = 0;
      close(*(int *)(pQVar3 + -0x6c));
      deallocate_ion_memory
                (this,(int *)(this + (iVar5 + 0x1f8) * 4),
                 (ion_fd_data *)(this + (iVar5 + 0x12e) * 8));
    }
    iVar5 = iVar5 + 1;
    pQVar3 = pQVar3 + 4;
  }
  iVar4 = 0;
  pQVar3 = this + 0x764;
  while( true ) {
    if (*(int *)(this + 0xd64) == 0) {
      iVar2 = *(int *)(this + 0xc);
    }
    else {
      iVar2 = 5;
    }
    if (iVar2 <= iVar4) break;
    if (*(int *)pQVar3 != 0) {
      (**(code **)(*(int *)pQVar3 + 0xc))();
      *(undefined4 *)pQVar3 = 0;
    }
    iVar4 = iVar4 + 1;
    pQVar3 = pQVar3 + 4;
  }
  if (*(int *)(this + 0x7b0) != 0) {
    (**(code **)(*(int *)(this + 0x7b0) + 0xc))();
    *(undefined4 *)(this + 0x7b0) = 0;
  }
  __android_log_print(6,DAT_00024208 + 0x241ea,DAT_0002420c + 0x241ec);
  return 1;
}



// Function: allocate_ion_memory @ 00024210

/* android::QualcommCameraHardware::allocate_ion_memory(int*, ion_allocation_data*, ion_fd_data*,
   int, int, int*) */

undefined4 __thiscall
android::QualcommCameraHardware::allocate_ion_memory
          (QualcommCameraHardware *this,int *param_1,ion_allocation_data *param_2,
          ion_fd_data *param_3,int param_4,int param_5,int *param_6)

{
  ulong uVar1;
  int iVar2;
  int *piVar3;
  char *pcVar4;
  int *local_14;
  
  local_14 = param_1;
  iVar2 = open((char *)(DAT_000242fc + 0x2421c),DAT_000242ec,param_2,param_3,this);
  *param_1 = iVar2;
  if (iVar2 < 0) {
    iVar2 = DAT_00024300 + 0x24232;
    __android_log_print(6,iVar2,DAT_00024304 + 0x24236);
    piVar3 = (int *)__errno();
    pcVar4 = strerror(*piVar3);
    __android_log_print(6,iVar2,DAT_00024308 + 0x2424a,pcVar4);
  }
  else {
    *(undefined4 *)(param_2 + 4) = 0x1000;
    *(uint *)param_2 = param_5 + 0xfffU & 0xfffff000;
    uVar1 = DAT_000242f0;
    *(uint *)(param_2 + 8) = 1 << (param_4 & 0xffU) | 0x10;
    iVar2 = ioctl(*param_1,uVar1,param_2);
    uVar1 = DAT_000242f4;
    if (iVar2 < 0) {
      __android_log_print(6,DAT_0002430c + 0x24290,DAT_00024310 + 0x24292);
    }
    else {
      *(undefined4 *)param_3 = *(undefined4 *)(param_2 + 0xc);
      iVar2 = ioctl(*param_1,uVar1,param_3);
      if (-1 < iVar2) {
        *param_6 = *(int *)(param_3 + 4);
        return 0;
      }
      piVar3 = (int *)__errno();
      pcVar4 = strerror(*piVar3);
      __android_log_print(6,DAT_00024314 + 0x242ba,DAT_00024318 + 0x242bc,pcVar4);
      local_14 = *(int **)param_3;
      ioctl(*param_1,DAT_000242f8,&local_14);
    }
    close(*param_1);
  }
  return 0xffffffff;
}



// Function: createRawToJpegMemory @ 0002431c

/* android::QualcommCameraHardware::createRawToJpegMemory() */

undefined4 __thiscall
android::QualcommCameraHardware::createRawToJpegMemory(QualcommCameraHardware *this)

{
  int iVar1;
  undefined4 uVar2;
  undefined4 *puVar3;
  
  iVar1 = allocate_ion_memory(this,(int *)(this + 0x7f4),(ion_allocation_data *)(this + 0x870),
                              (ion_fd_data *)(this + 0x9a0),8,*(int *)(this + 0x7dc),
                              (int *)(this + 0x710));
  if (iVar1 < 0) {
    __android_log_print(6,DAT_0002440c + 0x24356,DAT_00024410 + 0x24358,DAT_00024414 + 0x2435a);
    uVar2 = 0;
  }
  else {
    iVar1 = DAT_00024418 + 0x2436e;
    __android_log_print(6,iVar1,DAT_0002441c + 0x24374,DAT_00024420 + 0x2437a,
                        *(undefined4 *)(this + 0x710),*(undefined4 *)(this + 0x7dc));
    puVar3 = (undefined4 *)
             (**(code **)(this + 0xd00))
                       (*(undefined4 *)(this + 0x710),*(undefined4 *)(this + 0x7dc),1,
                        *(undefined4 *)(this + 0xd04));
    *(undefined4 **)(this + 0x77c) = puVar3;
    if (puVar3 == (undefined4 *)0x0) {
      __android_log_print(6,iVar1,DAT_00024424 + 0x243a6);
      uVar2 = 0;
    }
    else {
      __android_log_print(6,iVar1,DAT_00024428 + 0x243b6,*puVar3,puVar3[2],puVar3[1],puVar3[3]);
      __android_log_print(6,iVar1,DAT_0002442c + 0x243d4,*(undefined4 *)(this + 0x710),
                          *(undefined4 *)(this + 0x7dc));
      FUN_0001f76c(*(undefined4 *)(this + 0x7dc),*(undefined4 *)(this + 0x7dc),0,0,
                   *(undefined4 *)(this + 0x710),0,**(undefined4 **)(this + 0x77c),5,1,1,0);
      uVar2 = 1;
    }
  }
  return uVar2;
}



// Function: debugShowVideoFPS @ 00024430

/* android::QualcommCameraHardware::debugShowVideoFPS() const */

void android::QualcommCameraHardware::debugShowVideoFPS(void)

{
  uint uVar1;
  uint uVar2;
  float fVar3;
  int iVar4;
  int iVar5;
  float *pfVar6;
  int *piVar7;
  int *piVar8;
  undefined8 *puVar9;
  float fVar10;
  undefined8 uVar11;
  
  *(int *)(DAT_000244d0 + 0x2443c) = *(int *)(DAT_000244d0 + 0x2443c) + 1;
  uVar11 = systemTime(1);
  uVar1 = (int)((ulonglong)uVar11 >> 0x20) - *(int *)(DAT_000244d4 + 0x24452);
  uVar2 = (uint)((uint)uVar11 < *(uint *)(DAT_000244d4 + 0x2444e));
  if ((0 < (int)(uVar1 - uVar2)) ||
     ((uVar1 == uVar2 && (DAT_000244c8 < (uint)uVar11 - *(uint *)(DAT_000244d4 + 0x2444e))))) {
    piVar8 = (int *)(DAT_000244d8 + 0x24472);
    piVar7 = (int *)(DAT_000244dc + 0x24474);
    fVar10 = (float)(longlong)(*piVar8 - *piVar7) * DAT_000244cc;
    fVar3 = (float)__aeabi_l2f();
    pfVar6 = (float *)(DAT_000244e0 + 0x24494);
    iVar4 = DAT_000244e4 + 0x24496;
    iVar5 = DAT_000244e8 + 0x24498;
    fVar10 = fVar10 / fVar3;
    *pfVar6 = fVar10;
    __android_log_print(4,iVar4,iVar5,pfVar6,(double)fVar10);
    puVar9 = (undefined8 *)(DAT_000244ec + 0x244ba);
    *piVar7 = *piVar8;
    *puVar9 = uVar11;
  }
  return;
}



// Function: debugShowPreviewFPS @ 000244f0

/* android::QualcommCameraHardware::debugShowPreviewFPS() const */

void android::QualcommCameraHardware::debugShowPreviewFPS(void)

{
  uint uVar1;
  uint uVar2;
  float fVar3;
  int iVar4;
  int iVar5;
  float *pfVar6;
  int *piVar7;
  int *piVar8;
  undefined8 *puVar9;
  float fVar10;
  undefined8 uVar11;
  
  *(int *)(DAT_00024590 + 0x244fc) = *(int *)(DAT_00024590 + 0x244fc) + 1;
  uVar11 = systemTime(1);
  uVar1 = (int)((ulonglong)uVar11 >> 0x20) - *(int *)(DAT_00024594 + 0x24512);
  uVar2 = (uint)((uint)uVar11 < *(uint *)(DAT_00024594 + 0x2450e));
  if ((0 < (int)(uVar1 - uVar2)) ||
     ((uVar1 == uVar2 && (DAT_00024588 < (uint)uVar11 - *(uint *)(DAT_00024594 + 0x2450e))))) {
    piVar8 = (int *)(DAT_00024598 + 0x24532);
    piVar7 = (int *)(DAT_0002459c + 0x24534);
    fVar10 = (float)(longlong)(*piVar8 - *piVar7) * DAT_0002458c;
    fVar3 = (float)__aeabi_l2f();
    pfVar6 = (float *)(DAT_000245a0 + 0x24554);
    iVar4 = DAT_000245a4 + 0x24556;
    iVar5 = DAT_000245a8 + 0x24558;
    fVar10 = fVar10 / fVar3;
    *pfVar6 = fVar10;
    __android_log_print(4,iVar4,iVar5,pfVar6,(double)fVar10);
    puVar9 = (undefined8 *)(DAT_000245ac + 0x2457a);
    *piVar7 = *piVar8;
    *puVar9 = uVar11;
  }
  return;
}



// Function: receiveRecordingFrame @ 000245b0

/* android::QualcommCameraHardware::receiveRecordingFrame(msm_frame*) */

void __thiscall
android::QualcommCameraHardware::receiveRecordingFrame
          (QualcommCameraHardware *this,msm_frame *param_1)

{
  int iVar1;
  undefined4 *puVar2;
  int iVar3;
  
  iVar3 = DAT_00024654 + 0x245bc;
  __android_log_print(2,iVar3,DAT_00024658 + 0x245c0);
  iVar1 = DAT_0002465c;
  if (param_1 == (msm_frame *)0x0) {
    __android_log_print(6,iVar3,DAT_00024678 + 0x24642);
  }
  else {
    puVar2 = (undefined4 *)(DAT_0002465c + 0x245ce);
    pthread_mutex_lock((pthread_mutex_t *)(DAT_0002465c + 0x245da));
    __android_log_print(2,iVar3,DAT_00024660 + 0x245de,*puVar2);
    puVar2 = malloc(8);
    if (puVar2 == (undefined4 *)0x0) {
      __android_log_print(6,iVar3,DAT_00024670 + 0x24620);
    }
    else {
      puVar2[1] = param_1;
      *puVar2 = 0;
      if (*(undefined4 **)(iVar1 + 0x245d6) == (undefined4 *)0x0) {
        *(undefined4 **)(iVar1 + 0x245d2) = puVar2;
      }
      else {
        **(undefined4 **)(iVar1 + 0x245d6) = puVar2;
      }
      iVar3 = DAT_00024664;
      *(undefined4 **)(iVar1 + 0x245d6) = puVar2;
      iVar1 = DAT_00024668 + 0x24608;
      *(int *)(iVar3 + 0x24602) = *(int *)(iVar3 + 0x24602) + 1;
      __android_log_print(2,iVar1,DAT_0002466c + 0x24612);
    }
    iVar1 = DAT_00024674;
    pthread_mutex_unlock((pthread_mutex_t *)(DAT_00024674 + 0x24634));
    pthread_cond_signal((pthread_cond_t *)(iVar1 + 0x24638));
  }
  __android_log_print(2,DAT_0002467c + 0x2464e,DAT_00024680 + 0x24650);
  return;
}



// Function: runSmoothzoomThread @ 00024684

/* android::QualcommCameraHardware::runSmoothzoomThread(void*) */

void android::QualcommCameraHardware::runSmoothzoomThread(void *param_1)

{
  undefined4 uVar1;
  int iVar2;
  undefined4 uVar3;
  int iVar4;
  int iVar5;
  int iVar6;
  int iVar7;
  pthread_mutex_t *__mutex;
  pthread_mutex_t *__mutex_00;
  int unaff_r11;
  CameraParameters aCStack_40 [28];
  
  uVar1 = android::CameraParameters::getInt((char *)((int)param_1 + 0x18));
  uVar3 = *(undefined4 *)((int)param_1 + 0x370);
  iVar7 = DAT_00024814 + 0x246a8;
  __android_log_print(2,iVar7,DAT_00024818 + 0x246aa,uVar1,uVar3);
  iVar2 = android::CameraParameters::getInt((char *)((int)param_1 + 0x18));
  iVar5 = DAT_00024828;
  iVar4 = DAT_00024824;
  iVar6 = *(int *)((int)param_1 + 0x370);
  if (iVar2 != iVar6 && iVar6 <= iVar2) {
    unaff_r11 = -1;
  }
  if (iVar2 <= iVar6) {
    if (iVar2 == iVar6) {
      __android_log_print(2,iVar7,DAT_0002481c + 0x246d8,iVar6,uVar3);
      if (*(char *)((int)param_1 + 0xd85) == '\0') {
        (**(code **)((int)param_1 + 0xcf4))(8,iVar2,1,*(undefined4 *)((int)param_1 + 0xd04));
        return;
      }
      __android_log_print(2,iVar7,DAT_00024820 + 0x246fc);
      return;
    }
    unaff_r11 = 1;
  }
  __mutex_00 = (pthread_mutex_t *)((int)param_1 + 0x37c);
  (**(code **)(*(int *)param_1 + 0x60))(aCStack_40,param_1);
  __mutex = (pthread_mutex_t *)((int)param_1 + 0x378);
  pthread_mutex_lock(__mutex);
  *(undefined1 *)((int)param_1 + 0x375) = 1;
  pthread_mutex_unlock(__mutex);
  do {
    pthread_mutex_lock(__mutex_00);
    if (*(char *)((int)param_1 + 0x374) != '\0') {
      __android_log_print(2,DAT_0002482c + 0x24754,DAT_00024830 + 0x24756);
      pthread_mutex_unlock(__mutex_00);
LAB_000247e2:
      pthread_mutex_lock(__mutex);
      *(undefined1 *)((int)param_1 + 0x375) = 0;
      pthread_mutex_unlock(__mutex);
      __android_log_print(2,DAT_00024844 + 0x247fe,DAT_00024848 + 0x24800);
      android::CameraParameters::~CameraParameters(aCStack_40);
      return;
    }
    pthread_mutex_unlock(__mutex_00);
    if ((iVar2 < 0) || (*(int *)(iVar4 + 0x24726) < iVar2)) {
      uVar1 = 6;
      iVar4 = DAT_00024834 + 0x2477c;
      iVar5 = DAT_00024838 + 0x2477e;
LAB_000247c4:
      __android_log_print(uVar1,iVar4,iVar5);
      goto LAB_000247e2;
    }
    android::CameraParameters::set((char *)aCStack_40,iVar5 + 0x24728);
    setZoom(param_1,aCStack_40);
    if (*(char *)((int)param_1 + 0xd85) != '\0') {
      uVar1 = 2;
      iVar4 = DAT_0002483c + 0x247c4;
      iVar5 = DAT_00024840 + 0x247c6;
      goto LAB_000247c4;
    }
    (**(code **)((int)param_1 + 0xcf4))
              (8,iVar2,*(int *)((int)param_1 + 0x370) == iVar2,*(undefined4 *)((int)param_1 + 0xd04)
              );
    if (iVar2 == *(int *)((int)param_1 + 0x370)) goto LAB_000247e2;
    iVar2 = iVar2 + unaff_r11;
    pthread_mutex_lock(__mutex);
    pthread_cond_wait((pthread_cond_t *)((int)param_1 + 0x380),__mutex);
    pthread_mutex_unlock(__mutex);
  } while( true );
}



// Function: do_move_backward @ 0002484c

/* android::SortedVector<android::key_value_pair_t<android::String8, android::String8>
   >::do_move_backward(void*, void const*, unsigned int) const */

SortedVector<android::key_value_pair_t<android::String8,android::String8>> * __thiscall
android::SortedVector<android::key_value_pair_t<android::String8,android::String8>>::
do_move_backward(SortedVector<android::key_value_pair_t<android::String8,android::String8>> *this,
                void *param_1,void *param_2,uint param_3)

{
  int iVar1;
  String8 *this_00;
  String8 *this_01;
  
  iVar1 = 0;
  while( true ) {
    this_00 = (String8 *)((int)param_2 + iVar1);
    this_01 = (String8 *)((int)param_1 + iVar1);
    if (param_3 == 0) break;
    iVar1 = iVar1 + 8;
    android::String8::String8(this_01,this_00);
    param_3 = param_3 - 1;
    android::String8::String8(this_01 + 4,this_00 + 4);
    android::String8::~String8(this_00 + 4);
    this = (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *)
           android::String8::~String8(this_00);
  }
  return this;
}



// Function: do_move_forward @ 00024890

/* android::SortedVector<android::key_value_pair_t<android::String8, android::String8>
   >::do_move_forward(void*, void const*, unsigned int) const */

SortedVector<android::key_value_pair_t<android::String8,android::String8>> * __thiscall
android::SortedVector<android::key_value_pair_t<android::String8,android::String8>>::do_move_forward
          (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *this,
          void *param_1,void *param_2,uint param_3)

{
  String8 *this_00;
  String8 *pSVar1;
  String8 *pSVar2;
  
  pSVar1 = (String8 *)((int)param_2 + param_3 * 8);
  pSVar2 = (String8 *)((int)param_1 + param_3 * 8);
  for (; param_3 != 0; param_3 = param_3 - 1) {
    this_00 = pSVar1 + -8;
    android::String8::String8(pSVar2 + -8,this_00);
    android::String8::String8(pSVar2 + -4,pSVar1 + -4);
    android::String8::~String8(pSVar1 + -4);
    this = (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *)
           android::String8::~String8(this_00);
    pSVar1 = this_00;
    pSVar2 = pSVar2 + -8;
  }
  return this;
}



// Function: do_splat @ 000248cc

/* android::SortedVector<android::key_value_pair_t<android::String8, android::String8>
   >::do_splat(void*, void const*, unsigned int) const */

SortedVector<android::key_value_pair_t<android::String8,android::String8>> * __thiscall
android::SortedVector<android::key_value_pair_t<android::String8,android::String8>>::do_splat
          (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *this,
          void *param_1,void *param_2,uint param_3)

{
  String8 *this_00;
  
  for (; param_3 != 0; param_3 = param_3 - 1) {
    android::String8::String8(param_1,param_2);
    this_00 = (String8 *)((int)param_1 + 4);
    param_1 = (void *)((int)param_1 + 8);
    this = (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *)
           android::String8::String8(this_00,(String8 *)((int)param_2 + 4));
  }
  return this;
}



// Function: do_copy @ 000248f8

/* android::SortedVector<android::key_value_pair_t<android::String8, android::String8>
   >::do_copy(void*, void const*, unsigned int) const */

SortedVector<android::key_value_pair_t<android::String8,android::String8>> * __thiscall
android::SortedVector<android::key_value_pair_t<android::String8,android::String8>>::do_copy
          (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *this,
          void *param_1,void *param_2,uint param_3)

{
  int iVar1;
  String8 *this_00;
  String8 *pSVar2;
  
  iVar1 = 0;
  while( true ) {
    pSVar2 = (String8 *)((int)param_2 + iVar1);
    this_00 = (String8 *)((int)param_1 + iVar1);
    if (param_3 == 0) break;
    android::String8::String8(this_00,pSVar2);
    iVar1 = iVar1 + 8;
    this = (SortedVector<android::key_value_pair_t<android::String8,android::String8>> *)
           android::String8::String8(this_00 + 4,pSVar2 + 4);
    param_3 = param_3 - 1;
  }
  return this;
}



// Function: FUN_00024930 @ 00024930

SortedVectorImpl * FUN_00024930(SortedVectorImpl *param_1)

{
  *(int *)param_1 = *(int *)(DAT_00024950 + 0x2493c + DAT_00024954) + 8;
  android::VectorImpl::finish_vector();
  android::SortedVectorImpl::~SortedVectorImpl(param_1);
  return param_1;
}



// Function: FUN_00024958 @ 00024958

void * FUN_00024958(void *param_1)

{
  FUN_00024930();
  operator_delete(param_1);
  return param_1;
}



// Function: getParameters @ 0002496c

/* android::QualcommCameraHardware::getParameters() const */

void android::QualcommCameraHardware::getParameters(void)

{
  int iVar1;
  VectorImpl *in_r0;
  int in_r1;
  
  __android_log_print(2,DAT_000249ac + 0x2497c,DAT_000249a8 + 0x2497a);
  iVar1 = DAT_000249b0;
  android::VectorImpl::VectorImpl(in_r0,(VectorImpl *)(in_r1 + 0x18));
  *(int *)in_r0 = *(int *)(iVar1 + 0x24992 + DAT_000249b4) + 8;
  android::String8::String8((String8 *)(in_r0 + 0x14),(String8 *)(in_r1 + 0x2c));
  return;
}



// Function: cancelAutoFocusInternal @ 000249b8

/* android::QualcommCameraHardware::cancelAutoFocusInternal() */

undefined4 __thiscall
android::QualcommCameraHardware::cancelAutoFocusInternal(QualcommCameraHardware *this)

{
  QualcommCameraHardware QVar1;
  int iVar2;
  undefined4 uVar3;
  int iVar4;
  pthread_mutex_t *__mutex;
  int iVar5;
  
  iVar4 = DAT_00024a7c + 0x249c6;
  __android_log_print(2,iVar4,DAT_00024a80 + 0x249ca);
  if (this[0xd28] == (QualcommCameraHardware)0x0) {
    __android_log_print(2,iVar4,DAT_00024a84 + 0x249de);
    uVar3 = 0;
  }
  else {
    __mutex = (pthread_mutex_t *)(this + 0x5d0);
    iVar5 = DAT_00024a88 + 0x249f2;
    iVar4 = DAT_00024a8c + 0x249f4;
    while (iVar2 = pthread_mutex_trylock((pthread_mutex_t *)(this + 0x5d4)), iVar2 == 0) {
      __android_log_print(2,iVar5,iVar4);
      pthread_mutex_unlock((pthread_mutex_t *)(this + 0x5d4));
      pthread_mutex_lock(__mutex);
      QVar1 = this[0x5cc];
      pthread_mutex_unlock(__mutex);
      if (QVar1 == (QualcommCameraHardware)0x0) {
        uVar3 = FUN_0001f008(0xb,*(uint *)(this + 0x668) | 0x100,0);
        goto LAB_00024a2c;
      }
      usleep(5000);
    }
    __android_log_print(2,DAT_00024a98 + 0x24a4c,DAT_00024a9c + 0x24a4e);
    uVar3 = setAutoFocusStartStop(this,0);
    pthread_mutex_lock(__mutex);
    pthread_mutex_unlock(__mutex);
LAB_00024a2c:
    __android_log_print(2,DAT_00024a90 + 0x24a38,DAT_00024a94 + 0x24a3a,uVar3);
  }
  return uVar3;
}



// Function: cancelAutoFocus @ 00024aa0

/* android::QualcommCameraHardware::cancelAutoFocus() */

undefined4 __thiscall android::QualcommCameraHardware::cancelAutoFocus(QualcommCameraHardware *this)

{
  __android_log_print(2,DAT_00024af0 + 0x24aac,DAT_00024af4 + 0x24aae);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  if (((this[0x34] != (QualcommCameraHardware)0x0) && (*(int *)(this + 0xcf4) != 0)) &&
     (*(int *)(this + 0xcf0) << 0x1d < 0)) {
    cancelAutoFocusInternal(this);
  }
  __android_log_print(2,DAT_00024af8 + 0x24ae0,DAT_00024afc + 0x24ae2);
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return 0;
}



// Function: runAutoFocus @ 00024b00

/* android::QualcommCameraHardware::runAutoFocus() */

void __thiscall android::QualcommCameraHardware::runAutoFocus(QualcommCameraHardware *this)

{
  char *pcVar1;
  int iVar2;
  undefined4 uVar3;
  int *piVar4;
  undefined1 uVar5;
  int iVar6;
  pthread_mutex_t *__mutex;
  code *pcVar7;
  QualcommCameraHardware *pQVar8;
  uint uVar9;
  undefined4 local_30;
  int local_2c;
  
  __mutex = (pthread_mutex_t *)(this + 0x5d0);
  pthread_mutex_lock(__mutex);
  pQVar8 = this + 0x18;
  pcVar1 = (char *)android::CameraParameters::get((char *)pQVar8);
  iVar2 = android::CameraParameters::get((char *)pQVar8);
  if (((iVar2 == 0) || (iVar2 = strcmp(pcVar1,*(char **)(DAT_00024cf8 + 0x24b40)), iVar2 == 0)) ||
     (iVar2 = strcmp(pcVar1,*(char **)(DAT_00024cfc + 0x24b52)), iVar2 == 0)) {
    uVar5 = 1;
  }
  else {
    if (**(int **)(DAT_00024d3c + 0x24ce2) == 0) {
      uVar3 = dlerror();
      __android_log_print(6,DAT_00024d00 + 0x24b6a,DAT_00024d04 + 0x24b6c,uVar3);
      this[0x5cc] = (QualcommCameraHardware)0x0;
      pthread_mutex_unlock(__mutex);
      return;
    }
    uVar3 = android::CameraParameters::get((char *)pQVar8);
    iVar6 = DAT_00024d08 + 0x24b92;
    uVar3 = FUN_0001f71c(DAT_00024d0c + 0x24b98,6,uVar3);
    __android_log_print(2,iVar6,DAT_00024d10 + 0x24ba2,uVar3);
    iVar2 = pthread_mutex_trylock((pthread_mutex_t *)(this + 0x5d4));
    if (iVar2 == 0) {
      pthread_mutex_lock((pthread_mutex_t *)(this + 0x38));
      if (this[0x34] == (QualcommCameraHardware)0x0) {
        __android_log_print(2,iVar6,DAT_00024d2c + 0x24c58);
        uVar5 = 0;
      }
      else {
        __android_log_print(2,iVar6,DAT_00024d14 + 0x24bd6);
        *(undefined4 *)(this + 0x670) = 0;
        setAutoFocusStartStop(this,1);
        local_30 = 0x10;
        iVar2 = (**(code **)(DAT_00024d18 + 0x24c00))(&local_30);
        if (iVar2 == 0) {
          __android_log_print(2,iVar6,DAT_00024d20 + 0x24c1e,local_2c);
          iVar2 = local_2c;
        }
        else {
          piVar4 = (int *)__errno();
          pcVar1 = strerror(*piVar4);
          __android_log_print(6,iVar6,DAT_00024d1c + 0x24c08,0x10,pcVar1);
          iVar2 = 0;
        }
        __android_log_print(2,DAT_00024d24 + 0x24c2c,DAT_00024d28 + 0x24c2e,
                            *(undefined4 *)(this + 0x670),iVar2);
        if (iVar2 == 2) {
          uVar5 = 1;
          *(undefined4 *)(this + 0x670) = 0;
        }
        else {
          uVar5 = 0;
          *(undefined4 *)(this + 0x670) = 1;
        }
      }
      pthread_mutex_unlock((pthread_mutex_t *)(this + 0x38));
      pthread_mutex_unlock((pthread_mutex_t *)(this + 0x5d4));
    }
    else {
      uVar5 = 0;
      __android_log_print(2,iVar6,DAT_00024d30 + 0x24c74);
    }
    pthread_mutex_lock((pthread_mutex_t *)(this + 0x400));
    pthread_mutex_unlock((pthread_mutex_t *)(this + 0x400));
    __android_log_print(2,DAT_00024d34 + 0x24c90,DAT_00024d38 + 0x24c94,uVar5);
  }
  this[0x5cc] = (QualcommCameraHardware)0x0;
  pthread_mutex_unlock(__mutex);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x404));
  pcVar7 = *(code **)(this + 0xcf4);
  uVar9 = 0;
  if (pcVar7 != (code *)0x0) {
    uVar9 = (uint)(*(int *)(this + 0xcf0) << 0x1d) >> 0x1f;
  }
  uVar3 = *(undefined4 *)(this + 0xd04);
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x404));
  if (uVar9 != 0) {
    (*pcVar7)(4,uVar5,0,uVar3);
  }
  return;
}



// Function: deinitRaw @ 00024d40

/* android::QualcommCameraHardware::deinitRaw() */

void __thiscall android::QualcommCameraHardware::deinitRaw(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  int extraout_r3;
  int extraout_r3_00;
  int iVar5;
  int iVar6;
  int iVar7;
  QualcommCameraHardware *pQVar8;
  int iVar9;
  int iVar10;
  int iVar11;
  int iVar12;
  int iVar13;
  
  iVar7 = DAT_00024ffc + 0x24d4e;
  __android_log_print(2,iVar7,DAT_00025000 + 0x24d52);
  iVar11 = DAT_00025014;
  iVar9 = DAT_00025010;
  if (*(int *)(this + 0xcd0) != 0) {
    __android_log_print(6,iVar7,DAT_0002500c + 0x24d86);
    iVar7 = DAT_00025018;
    pQVar8 = this + 0x6d0;
    iVar12 = 0;
    iVar6 = DAT_00025020 + 0x24da6;
    iVar13 = DAT_0002501c + 0x24dac;
    while( true ) {
      if (*(int *)(this + 0xd64) == 0) {
        iVar5 = *(int *)(this + 0xc);
      }
      else {
        iVar5 = 3;
      }
      if (iVar5 <= iVar12) break;
      if (*(int **)(pQVar8 + 0x608) != (int *)0x0) {
        iVar5 = **(int **)(pQVar8 + 0x608);
        __android_log_print(6,iVar9 + 0x24da8,iVar11 + 0x24daa);
        pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f4));
        if (*(int *)pQVar8 == 1) {
          iVar1 = genlock_unlock_buffer(iVar5);
          if (iVar1 == 2) {
            __android_log_print(6,iVar9 + 0x24da8,DAT_00025024 + 0x24de8,DAT_00025028 + 0x24dea);
          }
          else {
            *(undefined4 *)pQVar8 = 0;
          }
        }
        iVar1 = *(int *)(iVar5 + 0xc);
        __android_log_print(6,iVar13,iVar6,iVar7 + 0x24dfe,iVar1);
        iVar2 = (**(code **)(*(int *)(this + 0xcd0) + 8))
                          (*(int *)(this + 0xcd0),*(undefined4 *)(pQVar8 + 0x608));
        if (iVar2 != 0) {
          iVar1 = *(int *)(iVar5 + 0xc);
          __android_log_print(6,iVar13,DAT_0002502c + 0x24e22,iVar7 + 0x24dfe,iVar1);
        }
        if ((*(int *)(this + 0x3ec) != 0) && (*(int **)(pQVar8 + 0xe4) != (int *)0x0)) {
          native_handle_delete(*(undefined4 *)(**(int **)(pQVar8 + 0xe4) + 4));
          iVar1 = iVar12;
          __android_log_print(4,DAT_00025030 + 0x24e46,DAT_00025034 + 0x24e48,DAT_00025038 + 0x24e4e
                              ,iVar12);
          (**(code **)(*(int *)(pQVar8 + 0xe4) + 0xc))();
          *(undefined4 *)(pQVar8 + 0xe4) = 0;
        }
        iVar4 = *(int *)(this + 0xd10);
        iVar2 = *(int *)(this + 0xd14);
        if (*(int *)(pQVar8 + -0x14) != 0) {
          iVar10 = DAT_0002503c + 0x24e80;
          __android_log_print(6,iVar10,DAT_00025040 + 0x24e82,DAT_00025044 + 0x24e86,
                              *(undefined4 *)(iVar5 + 0xc));
          iVar1 = *(int *)(iVar5 + 0xc);
          iVar3 = (iVar4 * iVar2 * 3) / 2;
          FUN_0001f76c(iVar3,iVar3,iVar4 * iVar2 + 3U & 0xfffffffc,0,iVar1,0,
                       *(undefined4 *)(pQVar8 + -0x14),3,0,0,0);
          iVar5 = munmap(*(void **)(pQVar8 + -0x14),*(size_t *)(iVar5 + 0x1c));
          iVar2 = extraout_r3;
          if (iVar5 == -1) {
            __android_log_print(6,iVar10,DAT_00025048 + 0x24edc,
                                *(undefined4 *)(DAT_0002504c + 0x24ede));
            iVar2 = extraout_r3_00;
          }
          *(undefined4 *)(pQVar8 + 0x608) = 0;
          *(undefined4 *)(pQVar8 + -0x14) = 0;
        }
        __android_log_print(6,DAT_00025050 + 0x24ef6,DAT_00025054 + 0x24ef8,iVar2,iVar1);
        pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f4));
      }
      iVar12 = iVar12 + 1;
      pQVar8 = pQVar8 + 4;
    }
  }
  iVar9 = 0;
  iVar11 = DAT_00025004 + 0x24d70;
  iVar7 = DAT_00025008 + 0x24d74;
  pQVar8 = this + 0x750;
  while( true ) {
    if (*(int *)(this + 0xd64) == 0) {
      iVar6 = *(int *)(this + 0xc);
    }
    else {
      iVar6 = 5;
    }
    if (iVar6 <= iVar9) break;
    if (*(int *)pQVar8 != 0) {
      __android_log_print(6,iVar11,iVar7);
      FUN_0001f76c(*(undefined4 *)(this + 0x430),*(undefined4 *)(this + 0x428),
                   *(undefined4 *)(this + 0x42c),0,*(undefined4 *)(pQVar8 + -0x6c),0,
                   **(undefined4 **)pQVar8,4,0,0,0);
      (**(code **)(*(int *)pQVar8 + 0xc))();
      *(undefined4 *)pQVar8 = 0;
      close(*(int *)(pQVar8 + -0x6c));
      deallocate_ion_memory
                (this,(int *)(this + (iVar9 + 0x1f8) * 4),
                 (ion_fd_data *)(this + (iVar9 + 0x12e) * 8));
    }
    iVar9 = iVar9 + 1;
    pQVar8 = pQVar8 + 4;
  }
  iVar9 = 0;
  iVar7 = DAT_00025058 + 0x24fa4;
  iVar11 = DAT_0002505c + 0x24fa6;
  pQVar8 = this + 0x764;
  while( true ) {
    if (*(int *)(this + 0xd64) == 0) {
      iVar6 = *(int *)(this + 0xc);
    }
    else {
      iVar6 = 5;
    }
    if (iVar6 <= iVar9) break;
    if (*(int *)pQVar8 != 0) {
      __android_log_print(6,iVar7,iVar11);
      (**(code **)(*(int *)pQVar8 + 0xc))();
      *(undefined4 *)pQVar8 = 0;
    }
    iVar9 = iVar9 + 1;
    pQVar8 = pQVar8 + 4;
  }
  if (*(int *)(this + 0x7b0) != 0) {
    (**(code **)(*(int *)(this + 0x7b0) + 0xc))();
    *(undefined4 *)(this + 0x7b0) = 0;
  }
  __android_log_print(2,DAT_00025060 + 0x24ff0,DAT_00025064 + 0x24ff2);
  return;
}



// Function: encodeData @ 00025068

/* android::QualcommCameraHardware::encodeData() */

void __thiscall android::QualcommCameraHardware::encodeData(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  int iVar3;
  pthread_mutex_t *__mutex;
  
  __android_log_print(2,DAT_00025150 + 0x25076,DAT_00025154 + 0x25078);
  iVar2 = DAT_0002515c;
  iVar1 = DAT_00025158;
  if ((*(int *)(this + 0xcf8) == 0) || (-1 < *(int *)(this + 0xcf0) << 0x17)) {
    __android_log_print(2,DAT_00025170 + 0x25106,DAT_00025174 + 0x25108);
  }
  else {
    __mutex = (pthread_mutex_t *)(this + 0x3c0);
    pthread_mutex_lock(__mutex);
    this[0x3bc] = (QualcommCameraHardware)0x1;
    pthread_mutex_unlock(__mutex);
    (**(code **)(*(int *)(DAT_00025160 + 0x250b4) + 4))
              (0xf,DAT_00025164 + 0x250ba,DAT_00025168 + 0x250c0);
    iVar3 = DAT_0002516c;
    pthread_mutex_lock(__mutex);
    while (this[0x3bc] != (QualcommCameraHardware)0x0) {
      __android_log_print(2,iVar3 + 0x250d0,iVar1 + 0x250bc);
      pthread_cond_wait((pthread_cond_t *)(this + 0x3c4),__mutex);
      __android_log_print(2,iVar3 + 0x250d0,iVar2 + 0x250c4);
    }
    pthread_mutex_unlock(__mutex);
  }
  (**(code **)(*(int *)(DAT_00025178 + 0x25114) + 0xc))(3,0,0);
  deinitRaw(this);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3d8));
  this[0x3d4] = (QualcommCameraHardware)0x0;
  pthread_cond_signal((pthread_cond_t *)(this + 0x3dc));
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3d8));
  __android_log_print(2,DAT_0002517c + 0x25148,DAT_00025180 + 0x2514a);
  return;
}



// Function: deinitRawSnapshot @ 00025184

/* android::QualcommCameraHardware::deinitRawSnapshot() */

void __thiscall android::QualcommCameraHardware::deinitRawSnapshot(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  int iVar5;
  
  iVar3 = DAT_000253c0 + 0x25190;
  __android_log_print(2,iVar3,DAT_000253c4 + 0x25194);
  if (((*(int *)(this + 0xcd0) != 0) &&
      (__android_log_print(6,iVar3,DAT_000253c8 + 0x251ae), *(int *)(this + 0xd64) == 0)) &&
     (*(int *)(this + 0xc) == 1)) {
    if (*(int **)(this + 0xcd8) != (int *)0x0) {
      iVar2 = **(int **)(this + 0xcd8);
      __android_log_print(6,iVar3,DAT_000253cc + 0x251d6);
      pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f4));
      if (*(int *)(this + 0x6d0) == 1) {
        iVar1 = genlock_unlock_buffer(iVar2);
        if (iVar1 == 2) {
          __android_log_print(6,iVar3,DAT_000253d0 + 0x25204,DAT_000253d4 + 0x25206);
        }
        else {
          *(undefined4 *)(this + 0x6d0) = 0;
        }
      }
      iVar5 = DAT_000253d8 + 0x25216;
      iVar1 = DAT_000253dc + 0x25218;
      __android_log_print(6,iVar5,DAT_000253e0 + 0x2521c,iVar1,*(undefined4 *)(iVar2 + 0xc));
      iVar3 = (**(code **)(*(int *)(this + 0xcd0) + 8))
                        (*(int *)(this + 0xcd0),*(undefined4 *)(this + 0xcd8));
      if (iVar3 != 0) {
        __android_log_print(6,iVar5,DAT_000253e4 + 0x2523e,iVar1,*(undefined4 *)(iVar2 + 0xc));
      }
      iVar1 = *(int *)(this + 0xd10);
      iVar3 = *(int *)(this + 0xd14);
      if (*(int *)(this + 0x6bc) != 0) {
        iVar4 = DAT_000253e8 + 0x25262;
        __android_log_print(6,iVar4,DAT_000253ec + 0x25264,DAT_000253f0 + 0x25268,
                            *(undefined4 *)(iVar2 + 0xc));
        iVar5 = (iVar1 * iVar3 * 3) / 2;
        FUN_0001f76c(iVar5,iVar5,iVar1 * iVar3 + 3U & 0xfffffffc,0,*(undefined4 *)(iVar2 + 0xc),0,
                     *(undefined4 *)(this + 0x6bc),3,0,0,0);
        iVar3 = munmap(*(void **)(this + 0x6bc),*(size_t *)(iVar2 + 0x1c));
        if (iVar3 == -1) {
          __android_log_print(6,iVar4,DAT_000253f4 + 0x252b8,*(undefined4 *)(DAT_000253f8 + 0x252ba)
                             );
        }
        *(undefined4 *)(this + 0xcd8) = 0;
        *(undefined4 *)(this + 0x6bc) = 0;
      }
    }
    __android_log_print(6,DAT_000253fc + 0x252d2,DAT_00025400 + 0x252d4);
    pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f4));
  }
  __android_log_print(6,DAT_00025404 + 0x252e8,DAT_00025408 + 0x252ea);
  if (*(undefined4 **)(this + 0x778) != (undefined4 *)0x0) {
    FUN_0001f76c((uint)*(ushort *)(this + 0x45c) * (uint)*(ushort *)(this + 0x45e),
                 (uint)*(ushort *)(this + 0x45c) * (uint)*(ushort *)(this + 0x45e),0,0,
                 *(undefined4 *)(this + 0x6f8),0,**(undefined4 **)(this + 0x778),5,0,0,0);
    (**(code **)(*(int *)(this + 0x778) + 0xc))();
    *(undefined4 *)(this + 0x778) = 0;
    close(*(int *)(this + 0x6f8));
    deallocate_ion_memory(this,(int *)(this + 0x7f8),(ion_fd_data *)(this + 0x998));
  }
  if (*(undefined4 **)(this + 0x77c) != (undefined4 *)0x0) {
    FUN_0001f76c(*(undefined4 *)(this + 0x7dc),*(undefined4 *)(this + 0x7dc),0,0,
                 *(undefined4 *)(this + 0x710),0,**(undefined4 **)(this + 0x77c),5,0,0,0);
    (**(code **)(*(int *)(this + 0x77c) + 0xc))();
    iVar3 = DAT_0002540c + 0x25382;
    iVar2 = DAT_00025410 + 0x25388;
    *(undefined4 *)(this + 0x77c) = 0;
    __android_log_print(2,iVar3,iVar2,*(undefined4 *)(this + 0x7dc));
    *(undefined4 *)(this + 0x7dc) = 0;
    close(*(int *)(this + 0x710));
    deallocate_ion_memory(this,(int *)(this + 0x7f4),(ion_fd_data *)(this + 0x9a0));
  }
  __android_log_print(2,DAT_00025414 + 0x253b4,DAT_00025418 + 0x253ba,*(undefined4 *)(this + 0x7dc))
  ;
  return;
}



// Function: receiveRawPicture @ 0002541c

/* android::QualcommCameraHardware::receiveRawPicture(int, msm_frame*, msm_frame*) */

void android::QualcommCameraHardware::receiveRawPicture
               (int param_1,msm_frame *param_2,msm_frame *param_3)

{
  undefined4 uVar1;
  AshmemPool *this;
  void *pvVar2;
  char *pcVar3;
  MemoryHeapBase *pMVar4;
  uchar *puVar5;
  uchar *puVar6;
  size_t __n;
  undefined1 *puVar7;
  MemoryBase *this_00;
  char *pcVar8;
  int iVar9;
  char *pcVar10;
  undefined4 uVar11;
  void *__dest;
  int iVar12;
  pthread_mutex_t *__mutex;
  int *piVar13;
  int iVar14;
  int iVar15;
  int *piVar16;
  int *piVar17;
  char *local_40;
  uint local_3c;
  int *local_38;
  int *local_34;
  int *local_30;
  int *local_2c [2];
  
  iVar14 = DAT_00025a50 + 0x25434;
  iVar12 = DAT_00025a54 + 0x25436;
  __android_log_print(6,iVar14,DAT_00025a58 + 0x25438,iVar12);
  iVar9 = *(int *)(param_1 + 0x664);
  if (iVar9 == 0xc || iVar9 == 3) {
    __android_log_print(6,iVar14,DAT_00025a5c + 0x25468,iVar12,iVar9);
    notifyShutter((QualcommCameraHardware *)param_1,true);
  }
  __mutex = (pthread_mutex_t *)(param_1 + 0x3b0);
  pthread_mutex_lock(__mutex);
  if (*(char *)(param_1 + 0x3ac) == '\0') {
    __android_log_print(6,DAT_00025a60 + 0x25496,DAT_00025a64 + 0x25498,DAT_00025a68 + 0x2549a);
  }
  else {
    pthread_mutex_unlock(__mutex);
    if (param_2 == (msm_frame *)0x0) {
      piVar13 = *(int **)(param_3 + 0x30);
      notifyShutter((QualcommCameraHardware *)param_1,false);
      iVar12 = DAT_00025a88;
      iVar9 = DAT_00025a7c;
      if (*(int *)(param_1 + 0x3e4) == 1) {
        if (piVar13 != (int *)0x0) {
          if ((*piVar13 == 0) || (piVar13[2] == 0)) {
            *(undefined4 *)(DAT_00025a88 + 0x255e6) = 0;
            *(undefined4 *)(iVar12 + 0x255ea) = 0;
            uVar1 = *(undefined4 *)(param_1 + 0xd54);
            *(undefined4 *)(iVar12 + 0x255ee) = uVar1;
            uVar11 = *(undefined4 *)(param_1 + 0xd58);
            *(undefined4 *)(iVar12 + 0x255f2) = uVar11;
            (**(code **)(*(int *)(param_1 + 0xcd0) + 0x14))
                      (*(int *)(param_1 + 0xcd0),0,0,uVar1,uVar11);
          }
          else {
            piVar16 = (int *)(DAT_00025a7c + 0x25574);
            iVar14 = ((uint)((piVar13[1] + 1) - *piVar13) >> 1) - 1;
            *piVar16 = iVar14;
            iVar12 = ((uint)((piVar13[3] + 1) - piVar13[2]) >> 1) - 1;
            if (iVar14 < 0) {
              *piVar16 = 0;
            }
            *(int *)(iVar9 + 0x25578) = iVar12;
            if (iVar12 < 0) {
              *(undefined4 *)(DAT_00025a80 + 0x255a4) = 0;
            }
            iVar9 = DAT_00025a84;
            piVar17 = (int *)(DAT_00025a84 + 0x255ac);
            piVar16 = (int *)(DAT_00025a84 + 0x255b0);
            iVar14 = *piVar17 + *piVar13;
            *(int *)(DAT_00025a84 + 0x255b4) = iVar14;
            iVar12 = *piVar16 + piVar13[2];
            *(int *)(iVar9 + 0x255b8) = iVar12;
            (**(code **)(*(int *)(param_1 + 0xcd0) + 0x14))
                      (*(int *)(param_1 + 0xcd0),*piVar17,*(undefined4 *)(iVar9 + 0x255b0),iVar14,
                       iVar12);
            *(undefined1 *)(param_1 + 0xd3c) = 1;
          }
        }
        __mutex = (pthread_mutex_t *)(param_1 + 0x3f4);
        iVar12 = DAT_00025a8c + 0x25618;
        __android_log_print(6,iVar12,DAT_00025a90 + 0x2561a);
        pthread_mutex_lock(__mutex);
        iVar9 = mapThumbnailBuffer((QualcommCameraHardware *)param_1,param_3);
        __android_log_print(6,iVar12,DAT_00025a94 + 0x25636,iVar9);
        if (iVar9 < 0) {
          __android_log_print(6,iVar12,DAT_00025a98 + 0x2564e);
        }
        else if (((*(int *)(param_1 + 0xcd0) != 0) &&
                 (piVar13 = *(int **)(param_1 + iVar9 * 4 + 0xcd8), piVar13 != (int *)0x0)) &&
                (*(int *)(param_1 + 0xd64) == 0)) {
          iVar15 = *piVar13;
          uVar1 = *(undefined4 *)(iVar15 + 0xc);
          iVar14 = DAT_00025a9c + 0x2568a;
          __android_log_print(2,iVar12,DAT_00025aa0 + 0x2568c,iVar14,uVar1);
          if (*(int *)(param_1 + (iVar9 + 0x1b4) * 4) == 1) {
            iVar15 = genlock_unlock_buffer(iVar15);
            if (iVar15 == 2) {
              __android_log_print(6,iVar12,DAT_00025aa4 + 0x256b8,iVar14,uVar1);
              goto LAB_000256bc;
            }
            *(undefined4 *)(param_1 + (iVar9 + 0x1b4) * 4) = 0;
          }
          iVar14 = DAT_00025aa8 + 0x256d6;
          iVar12 = (**(code **)(*(int *)(param_1 + 0xcd0) + 4))
                             (*(int *)(param_1 + 0xcd0),*(undefined4 *)(param_1 + iVar9 * 4 + 0xcd8)
                             );
          __android_log_print(6,iVar14,DAT_00025aac + 0x256de);
          if (iVar12 != 0) {
            __android_log_print(6,iVar14,DAT_00025ab0 + 0x256f2,DAT_00025ab4 + 0x256f6,iVar12);
          }
        }
        pthread_mutex_unlock(__mutex);
        __android_log_print(6,DAT_00025ab8 + 0x25708,DAT_00025abc + 0x2570a);
        if ((*(code **)(param_1 + 0xcf8) == (code *)0x0) || (-1 < *(int *)(param_1 + 0xcf0) << 0x18)
           ) {
          if ((*(code **)(param_1 + 0xcf4) != (code *)0x0) &&
             (*(int *)(param_1 + 0xcf0) << 0x16 < 0)) {
            (**(code **)(param_1 + 0xcf4))(0x200,0,0,*(undefined4 *)(param_1 + 0xd04));
          }
        }
        else {
          (**(code **)(param_1 + 0xcf8))
                    (0x80,*(undefined4 *)(param_1 + (iVar9 + 0x1d4) * 4),0,0,
                     *(undefined4 *)(param_1 + 0xd04));
        }
      }
      else {
        iVar9 = DAT_00025ac0 + 0x25760;
        __android_log_print(2,iVar9,DAT_00025ac4 + 0x25762);
        if ((*(int *)(param_1 + 0xcf8) != 0) && (*(int *)(param_1 + 0xcf0) << 0x17 < 0)) {
          notifyShutter((QualcommCameraHardware *)param_1,false);
          if (*(int *)(param_1 + 0x30c) != 0) {
            __android_log_print(2,iVar9,DAT_00025ac8 + 0x25792);
            if (*(void **)(param_1 + 0x30c) != (void *)0x0) {
              android::RefBase::decStrong(*(void **)(param_1 + 0x30c));
              *(undefined4 *)(param_1 + 0x30c) = 0;
            }
          }
          this = operator_new(0x24);
          iVar9 = *(int *)(*(int *)(param_1 + 0x778) + 4);
          AshmemPool::AshmemPool(this,iVar9,1,iVar9,(char *)(DAT_00025acc + 0x257c2));
          if (this != (AshmemPool *)0x0) {
            android::RefBase::incStrong(this);
          }
          if (*(void **)(param_1 + 0x30c) != (void *)0x0) {
            android::RefBase::decStrong(*(void **)(param_1 + 0x30c));
          }
          *(AshmemPool **)(param_1 + 0x30c) = this;
          piVar13 = *(int **)(this + 0x18);
          if ((piVar13 == (int *)0x0) ||
             (iVar9 = FUN_0001de20((int)piVar13 + *(int *)(*piVar13 + -0xc)), iVar9 == -1)) {
            __android_log_print(6,DAT_00025d14 + 0x25cc8,DAT_00025d18 + 0x25cca);
            deinitRawSnapshot((QualcommCameraHardware *)param_1);
            return;
          }
          iVar12 = **(int **)(param_1 + 0x778);
          local_40 = (char *)0x0;
          local_3c = 0;
          FUN_0001f1c8(0x16,0,&local_40);
          iVar9 = DAT_00025ad0;
          FUN_0001f1c8(0x16,1,&local_3c);
          __android_log_print(6,iVar9 + 0x25826,DAT_00025ad4 + 0x2582e,local_40,local_3c);
          if (local_40 == (char *)0x0) {
            __android_log_print(6,iVar9 + 0x25826,DAT_00025ad8 + 0x25842);
            piVar13 = *(int **)(*(int *)(param_1 + 0x30c) + 0x18);
            pvVar2 = (void *)FUN_0001de20((int)piVar13 + *(int *)(*piVar13 + -0xc));
            memcpy(pvVar2,(void *)**(undefined4 **)(param_1 + 0x778),
                   (*(undefined4 **)(param_1 + 0x778))[1]);
            piVar13 = *(int **)(*(int *)(param_1 + 0x30c) + 0x18);
            pcVar3 = (char *)FUN_0001de20((int)piVar13 + *(int *)(*piVar13 + -0xc));
            pcVar10 = pcVar3;
            do {
              pcVar8 = pcVar10 + 1;
              if ((*pcVar10 == -1) && (pcVar10[1] == -0x27)) {
                local_40 = pcVar10 + (2 - (int)pcVar3);
                __android_log_print(6,DAT_00025adc + 0x2589e,DAT_00025ae0 + 0x258a2);
                break;
              }
              pcVar10 = pcVar8;
            } while (pcVar8 != pcVar3 + 0x3a0000);
            iVar9 = 0x3a0000;
            pcVar10 = pcVar3 + DAT_00025a48;
            do {
              if ((pcVar10[1] == -1) && (pcVar10[2] == -0x27)) {
                local_3c = iVar9 + DAT_00025a4c;
                __android_log_print(6,DAT_00025ae4 + 0x258d4,DAT_00025ae8 + 0x258d8);
                break;
              }
              iVar9 = iVar9 + 1;
              pcVar10 = pcVar10 + 1;
            } while (iVar9 != 0x3b0000);
          }
          pMVar4 = operator_new(0x38);
          android::MemoryHeapBase::MemoryHeapBase(pMVar4,local_3c,0,(char *)0x0);
          sp<android::MemoryHeapBase>::sp((sp<android::MemoryHeapBase> *)local_2c,pMVar4);
          pvVar2 = (void *)FUN_0001de20((int)local_2c[0] + *(int *)(*local_2c[0] + -0xc));
          memcpy(pvVar2,(void *)(iVar12 + 0x3a0000),local_3c);
          pMVar4 = operator_new(0x38);
          android::MemoryHeapBase::MemoryHeapBase(pMVar4,local_3c + 0xb40,0,(char *)0x0);
          sp<android::MemoryHeapBase>::sp((sp<android::MemoryHeapBase> *)&local_30,pMVar4);
          puVar5 = (uchar *)FUN_0001de20((int)local_30 + *(int *)(*local_30 + -0xc));
          iVar9 = DAT_00025aec;
          puVar6 = (uchar *)FUN_0001de20((int)local_2c[0] + *(int *)(*local_2c[0] + -0xc));
          iVar9 = iVar9 + 0x2595e;
          __n = getExif((QualcommCameraHardware *)param_1,puVar5,puVar6,local_3c);
          __android_log_print(2,iVar9,DAT_00025af0 + 0x2596c,__n);
          if ((int)__n < 1) {
            local_34 = (int *)0x0;
            __android_log_print(2,iVar9,DAT_00025cd8 + 0x25b1c,local_40);
            piVar13 = *(int **)(*(int *)(param_1 + 0x30c) + 0x18);
            pvVar2 = (void *)FUN_0001de20((int)piVar13 + *(int *)(*piVar13 + -0xc));
            memcpy(pvVar2,(void *)**(undefined4 **)(param_1 + 0x778),(size_t)local_40);
            local_38 = *(int **)(*(int *)(param_1 + 0x30c) + 0x18);
            if (local_38 != (int *)0x0) {
              local_38 = (int *)((int)local_38 + *(int *)(*local_38 + -0xc));
            }
            if (local_38 != (int *)0x0) {
              android::RefBase::incStrong((void *)((int)local_38 + *(int *)(*local_38 + -0xc)));
            }
            this_00 = operator_new(0x24);
            android::MemoryBase::MemoryBase(this_00,(sp *)&local_38,0,(uint)local_40);
            sp<android::MemoryBase>::operator=((sp<android::MemoryBase> *)&local_34,this_00);
            if (local_38 != (int *)0x0) {
              android::RefBase::decStrong((void *)((int)local_38 + *(int *)(*local_38 + -0xc)));
            }
            if (local_34 != (int *)0x0) {
              android::RefBase::decStrong((void *)((int)local_34 + *(int *)(*local_34 + -0xc)));
            }
          }
          else {
            piVar13 = *(int **)(*(int *)(param_1 + 0x30c) + 0x18);
            puVar7 = (undefined1 *)FUN_0001de20((int)piVar13 + *(int *)(*piVar13 + -0xc));
            __android_log_print(2,iVar9,DAT_00025af4 + 0x25994);
            iVar14 = DAT_00025af8;
            puVar7[1] = 0xd8;
            *puVar7 = 0xff;
            __android_log_print(2,iVar9,iVar14 + 0x259a8,__n);
            pvVar2 = (void *)FUN_0001de20((int)local_30 + *(int *)(*local_30 + -0xc));
            memcpy(puVar7 + 2,pvVar2,__n);
            __android_log_print(2,iVar9,DAT_00025afc + 0x259d6,__n);
            memcpy(puVar7 + 2 + __n,(void *)(iVar12 + 2),(size_t)local_40);
            __android_log_print(2,iVar9,DAT_00025b00 + 0x259f0,__n);
            *(char **)(param_1 + 0x7dc) = local_40 + __n;
            iVar12 = createRawToJpegMemory((QualcommCameraHardware *)param_1);
            if (iVar12 == 0) {
              __android_log_print(6,iVar9,DAT_00025b04 + 0x25a0e);
            }
            piVar13 = *(int **)(*(int *)(param_1 + 0x30c) + 0x18);
            __dest = (void *)**(undefined4 **)(param_1 + 0x77c);
            pvVar2 = (void *)FUN_0001de20((int)piVar13 + *(int *)(*piVar13 + -0xc));
            memcpy(__dest,pvVar2,*(size_t *)(param_1 + 0x7dc));
            __android_log_print(2,DAT_00025b08 + 0x25a3e,DAT_00025b0c + 0x25a44,
                                *(undefined4 *)(param_1 + 0x7dc));
          }
          if (*(int *)(param_1 + 0xcf0) << 0x16 < 0) {
            (**(code **)(param_1 + 0xcf4))(0x200,0,0,*(undefined4 *)(param_1 + 0xd04));
          }
          iVar12 = DAT_00025cdc + 0x25bc6;
          __android_log_print(2,iVar12,DAT_00025ce0 + 0x25bcc,*(undefined4 *)(param_1 + 0xcf0));
          (**(code **)(param_1 + 0xcf8))
                    (0x100,*(undefined4 *)(param_1 + 0x77c),0,0,*(undefined4 *)(param_1 + 0xd04));
          iVar9 = DAT_00025ce4;
          *(uint *)(param_1 + 0xcf0) = *(uint *)(param_1 + 0xcf0) & 0xfffffeff;
          __android_log_print(2,iVar12,iVar9 + 0x25bfe);
          sp<android::MemoryHeapPmem>::~sp((sp<android::MemoryHeapPmem> *)&local_30);
          sp<android::MemoryHeapPmem>::~sp((sp<android::MemoryHeapPmem> *)local_2c);
        }
        pthread_mutex_lock((pthread_mutex_t *)(param_1 + 0x3c0));
        *(undefined1 *)(param_1 + 0x3bc) = 0;
        pthread_cond_signal((pthread_cond_t *)(param_1 + 0x3c4));
        pthread_mutex_unlock((pthread_mutex_t *)(param_1 + 0x3c0));
        __android_log_print(2,DAT_00025ce8 + 0x25c34,DAT_00025cec + 0x25c36);
      }
      pthread_mutex_lock((pthread_mutex_t *)(param_1 + 0x3cc));
      *(undefined1 *)(param_1 + 0x3c8) = 0;
      pthread_cond_signal((pthread_cond_t *)(param_1 + 0x3d0));
      pthread_mutex_unlock((pthread_mutex_t *)(param_1 + 0x3cc));
      __android_log_print(2,DAT_00025cf0 + 0x25c60,DAT_00025cf4 + 0x25c62);
      if (*(int *)(param_1 + 0x3e4) == 1) {
        pcVar10 = (char *)(param_1 + 0x18);
        android::CameraParameters::set(pcVar10,DAT_00025cf8 + 0x25c78);
        FUN_0001f1c8(0x24,0,&local_3c);
        android::CameraParameters::set(pcVar10,DAT_00025d00 + 0x25c92);
        FUN_0001f1c8(0x24,5,&local_40);
        android::CameraParameters::set(pcVar10,DAT_00025d04 + 0x25ca8);
      }
      __android_log_print(2,DAT_00025d08 + 0x25cb6,DAT_00025d0c + 0x25cb8,DAT_00025d10 + 0x25cba);
      return;
    }
    iVar9 = DAT_00025a6c + 0x254ba;
    __android_log_print(6,iVar9,DAT_00025a70 + 0x254bc,DAT_00025a74 + 0x254c0);
    if ((*(int *)(param_1 + 0xcf8) != 0) && (*(int *)(param_1 + 0xcf0) << 0x17 < 0)) {
      __android_log_print(6,iVar9,DAT_00025a78 + 0x254dc);
      (**(code **)(param_1 + 0xcf8))(0x100,0,0,0,*(undefined4 *)(param_1 + 0xd04));
    }
    pthread_mutex_lock((pthread_mutex_t *)(param_1 + 0x3a8));
    *(undefined1 *)(param_1 + 0x3a4) = 0;
    pthread_mutex_unlock((pthread_mutex_t *)(param_1 + 0x3a8));
    pthread_mutex_lock((pthread_mutex_t *)(param_1 + 0x3c0));
    *(undefined1 *)(param_1 + 0x3bc) = 0;
    pthread_cond_signal((pthread_cond_t *)(param_1 + 0x3c4));
    __mutex = (pthread_mutex_t *)(param_1 + 0x3cc);
    pthread_mutex_unlock((pthread_mutex_t *)(param_1 + 0x3c0));
    pthread_mutex_lock(__mutex);
    *(undefined1 *)(param_1 + 0x3c8) = 0;
    pthread_cond_signal((pthread_cond_t *)(param_1 + 0x3d0));
  }
LAB_000256bc:
  pthread_mutex_unlock(__mutex);
  return;
}



// Function: get_currentTemp @ 00025d1c

/* android::get_currentTemp() */

undefined4 android::get_currentTemp(void)

{
  FILE *__stream;
  undefined4 uVar1;
  char *__filename;
  short local_12 [3];
  
  __filename = (char *)(DAT_00025d8c + 0x25d28);
  local_12[0] = 0;
  __stream = fopen(__filename,(char *)(DAT_00025d90 + 0x25d2e));
  if (__stream == (FILE *)0x0) {
    __android_log_print(6,DAT_00025d94 + 0x25d46,DAT_00025d98 + 0x25d48,__filename);
    uVar1 = 1;
  }
  else {
    fscanf(__stream,(char *)(DAT_00025d9c + 0x25d58),local_12);
    fclose(__stream);
    __android_log_print(6,DAT_00025da0 + 0x25d6e,DAT_00025da4 + 0x25d72,DAT_00025da8 + 0x25d74,
                        (int)local_12[0]);
    if (local_12[0] < -0x1d) {
      uVar1 = 0;
    }
    else {
      uVar1 = 1;
    }
  }
  return uVar1;
}



// Function: runSnapshotThread @ 00025dac

/* android::QualcommCameraHardware::runSnapshotThread(void*) */

void android::QualcommCameraHardware::runSnapshotThread(void *param_1)

{
  int iVar1;
  int iVar2;
  undefined4 uVar3;
  undefined4 *puVar4;
  pthread_cond_t *__cond;
  code *pcVar5;
  int iVar6;
  pthread_mutex_t *__mutex;
  pthread_mutex_t *__mutex_00;
  int iVar7;
  int iVar8;
  
  iVar6 = DAT_000260b8;
  iVar7 = DAT_000260b0 + 0x25dba;
  __android_log_print(4,iVar7,DAT_000260b4 + 0x25dbe);
  iVar2 = get_currentTemp();
  iVar6 = iVar6 + 0x25dce;
  if (iVar2 != 0) {
    __android_log_print(2,iVar7,DAT_000260c0 + 0x25df6,*(undefined4 *)((int)param_1 + 0x638));
    uVar3 = 6;
  }
  else {
    __android_log_print(2,iVar7,DAT_000260bc + 0x25dde,*(undefined4 *)((int)param_1 + 0x638));
    uVar3 = 5;
  }
  FUN_0001f008(0,uVar3,iVar2 != 0);
  if (**(int **)(iVar6 + DAT_000260c4) == 0) {
    uVar3 = dlerror();
    __android_log_print(6,DAT_000260c8 + 0x25e16,DAT_000260cc + 0x25e18,uVar3);
  }
  __mutex = (pthread_mutex_t *)((int)param_1 + 0xd78);
  pthread_mutex_lock(__mutex);
  if (*(char *)((int)param_1 + 0xd74) == '\0') {
    __mutex_00 = (pthread_mutex_t *)((int)param_1 + 0x3c0);
    pthread_mutex_unlock(__mutex);
    pthread_mutex_lock(__mutex_00);
    __cond = (pthread_cond_t *)((int)param_1 + 0x3c4);
    *(undefined1 *)((int)param_1 + 0x3bc) = 1;
    pthread_cond_signal(__cond);
    pthread_mutex_unlock(__mutex_00);
    iVar2 = DAT_000260ec;
    iVar7 = *(int *)((int)param_1 + 0x3e4);
    if (iVar7 == 1) {
      uVar3 = 0xd;
    }
    else {
      uVar3 = 0xe;
    }
    if (*(char *)((int)param_1 + 0xd48) == '\0') {
      if (iVar7 == 1) {
        if ((*(int *)((int)param_1 + 0xd64) == 0) || (*(char *)((int)param_1 + 0xd6c) != '\0')) {
          puVar4 = (undefined4 *)(DAT_000260e4 + 0x25f0a);
          iVar2 = DAT_000260e8 + 0x25f0c;
          pcVar5 = *(code **)(*(int *)(iVar6 + DAT_000260dc) + 4);
        }
        else {
          notifyShutter(param_1,true);
          puVar4 = (undefined4 *)(iVar2 + 0x25f1e);
          initZslParameter();
          iVar7 = DAT_000260dc;
          __android_log_print(6,DAT_000260f0 + 0x25f32,DAT_000260f4 + 0x25f38,
                              *(undefined4 *)(iVar2 + 0x25f22),*(undefined4 *)(iVar2 + 0x25f26),
                              *puVar4);
          pcVar5 = *(code **)(*(int *)(iVar6 + iVar7) + 4);
          iVar2 = DAT_000260f8 + 0x25f50;
        }
        iVar7 = DAT_000260fc;
        (*pcVar5)(uVar3,puVar4,iVar2);
        iVar1 = DAT_00026104;
        iVar2 = DAT_00026100;
        pthread_mutex_lock(__mutex_00);
        iVar8 = DAT_00026108 + 0x25f70;
        while (*(char *)((int)param_1 + 0x3bc) != '\0') {
          __android_log_print(2,iVar7 + 0x25f6a,iVar2 + 0x25f6c,iVar1 + 0x25f6e);
          pthread_cond_wait(__cond,__mutex_00);
          __android_log_print(2,iVar7 + 0x25f6a,iVar8,iVar1 + 0x25f6e);
        }
        pthread_mutex_unlock(__mutex_00);
        if ((*(int *)((int)param_1 + 0xd64) == 0) || (*(char *)((int)param_1 + 0xd6c) != '\0')) {
          deinitRaw(param_1);
        }
      }
      else if (iVar7 == 2) {
        if (*(int *)((int)param_1 + 0x664) != 0xc && *(int *)((int)param_1 + 0x664) != 3) {
          notifyShutter(param_1,true);
        }
        iVar7 = DAT_00026118;
        iVar2 = DAT_00026114;
        iVar8 = DAT_00026110 + 0x25ff4;
        (**(code **)(*(int *)(iVar6 + DAT_000260dc) + 4))(uVar3,DAT_0002610c + 0x25fee,0);
        iVar1 = DAT_00026124;
        __android_log_print(2,DAT_0002611c + 0x26006,DAT_00026120 + 0x2600a);
        pthread_mutex_lock(__mutex_00);
        while (*(char *)((int)param_1 + 0x3bc) != '\0') {
          __android_log_print(2,iVar1 + 0x26018,iVar8,iVar2 + 0x2600c);
          pthread_cond_wait(__cond,__mutex_00);
          __android_log_print(2,iVar1 + 0x26018,iVar7 + 0x2601a,iVar2 + 0x2600c);
        }
        pthread_mutex_unlock(__mutex_00);
        __android_log_print(2,DAT_00026128 + 0x26052,DAT_0002612c + 0x26054);
        deinitRawSnapshot(param_1);
      }
    }
    else {
      uVar3 = 3;
      (**(code **)(*(int *)(iVar6 + DAT_000260dc) + 4))(3,DAT_000260e0 + 0x25ede,0);
    }
    if ((*(int *)((int)param_1 + 0xd64) == 0) || (*(char *)((int)param_1 + 0xd6c) != '\0')) {
      (**(code **)(*(int *)(iVar6 + DAT_000260dc) + 0xc))(uVar3,0,0);
    }
    *(undefined1 *)((int)param_1 + 0xd6c) = 0;
    pthread_mutex_lock((pthread_mutex_t *)((int)param_1 + 0x3b0));
    *(undefined1 *)((int)param_1 + 0x3ac) = 0;
    pthread_cond_signal((pthread_cond_t *)((int)param_1 + 0x3b4));
    pthread_mutex_unlock((pthread_mutex_t *)((int)param_1 + 0x3b0));
    __android_log_print(4,DAT_00026130 + 0x260a6,DAT_00026134 + 0x260a8);
  }
  else {
    *(undefined1 *)((int)param_1 + 0xd74) = 0;
    pthread_mutex_unlock(__mutex);
    __android_log_print(4,DAT_000260d0 + 0x25e4c,DAT_000260d4 + 0x25e4e,DAT_000260d8 + 0x25e50);
    deinitRaw(param_1);
    pthread_mutex_lock((pthread_mutex_t *)((int)param_1 + 0x3cc));
    *(undefined1 *)((int)param_1 + 0x3c8) = 0;
    pthread_cond_signal((pthread_cond_t *)((int)param_1 + 0x3d0));
    pthread_mutex_unlock((pthread_mutex_t *)((int)param_1 + 0x3cc));
    pthread_mutex_lock((pthread_mutex_t *)((int)param_1 + 0x3b0));
    *(undefined1 *)((int)param_1 + 0x3ac) = 0;
    pthread_cond_signal((pthread_cond_t *)((int)param_1 + 0x3b4));
    pthread_mutex_unlock((pthread_mutex_t *)((int)param_1 + 0x3b0));
  }
  return;
}



// Function: createSnapshotMemory @ 00026138

/* android::QualcommCameraHardware::createSnapshotMemory(int, int, bool, int) */

undefined4 __thiscall
android::QualcommCameraHardware::createSnapshotMemory
          (QualcommCameraHardware *this,int param_1,int param_2,bool param_3,int param_4)

{
  bool bVar1;
  int iVar2;
  void *pvVar3;
  undefined4 *puVar4;
  undefined4 uVar5;
  int iVar6;
  int iVar7;
  int iVar8;
  int iVar9;
  QualcommCameraHardware *pQVar10;
  int iVar11;
  int iVar12;
  int iVar13;
  int iVar14;
  int iVar15;
  int iVar16;
  pthread_mutex_t *__mutex;
  
  iVar13 = DAT_000265c8;
  if (param_4 != 1) {
    if (param_1 == 1) {
      iVar12 = (uint)*(ushort *)(this + 0x45c) * (uint)*(ushort *)(this + 0x45e);
      iVar13 = allocate_ion_memory(this,(int *)(this + 0x7f8),(ion_allocation_data *)(this + 0x880),
                                   (ion_fd_data *)(this + 0x998),8,iVar12,(int *)(this + 0x6f8));
      if (-1 < iVar13) {
        iVar13 = DAT_00026650 + 0x26524;
        __android_log_print(6,iVar13,DAT_00026654 + 0x26526,DAT_00026658 + 0x2652a,
                            *(undefined4 *)(this + 0x6f8));
        puVar4 = (undefined4 *)
                 (**(code **)(this + 0xd00))
                           (*(undefined4 *)(this + 0x6f8),iVar12,1,*(undefined4 *)(this + 0xd04));
        *(undefined4 **)(this + 0x778) = puVar4;
        if (puVar4 != (undefined4 *)0x0) {
          __android_log_print(6,iVar13,DAT_00026660 + 0x26562,*puVar4,puVar4[2],puVar4[1],puVar4[3])
          ;
          __android_log_print(6,iVar13,DAT_00026664 + 0x2657e,*(undefined4 *)(this + 0x6f8));
          FUN_0001f76c(iVar12,iVar12,0,0,*(undefined4 *)(this + 0x6f8),0,
                       **(undefined4 **)(this + 0x778),5,1,1,0);
          return 1;
        }
        __android_log_print(6,iVar13,DAT_0002665c + 0x26554);
        return 0;
      }
      iVar13 = DAT_00026644 + 0x2650a;
      iVar12 = DAT_00026648 + 0x2650c;
      iVar6 = DAT_0002664c + 0x2650e;
LAB_0002650c:
      __android_log_print(6,iVar13,iVar12,iVar6);
    }
    else {
      __android_log_print(6,DAT_00026668 + 0x265ba,DAT_0002666c + 0x265bc);
    }
    return 0;
  }
  iVar9 = 0;
  iVar12 = DAT_000265cc + 0x26166;
  iVar14 = DAT_000265d0 + 0x2616c;
  iVar16 = DAT_000265d4 + 0x2616e;
  iVar6 = DAT_000265d8 + 0x26172;
  pQVar10 = this;
  do {
    if (param_1 <= iVar9) {
      if (param_3) {
        iVar12 = DAT_000265ec + 0x26284;
        iVar13 = DAT_000265f0 + 0x2628a;
        iVar6 = DAT_000265f4 + 0x2628e;
        iVar14 = DAT_000265f8 + 0x26290;
        pQVar10 = this + 0x6f8;
        for (iVar9 = 0; iVar9 < param_2; iVar9 = iVar9 + 1) {
          uVar5 = *(undefined4 *)(pQVar10 + 4);
          iVar8 = iVar9;
          __android_log_print(6,iVar12,iVar13,iVar6,iVar9,uVar5);
          puVar4 = (undefined4 *)
                   (**(code **)(this + 0xd00))
                             (0xffffffff,*(undefined4 *)(this + 0x430),1,
                              *(undefined4 *)(this + 0xd04));
          *(undefined4 **)(pQVar10 + 0x6c) = puVar4;
          if (puVar4 == (undefined4 *)0x0) {
            iVar13 = DAT_000265fc + 0x262ce;
LAB_000262d0:
            __android_log_print(6,iVar12,iVar13,iVar9,iVar8,uVar5);
            return 0;
          }
          __android_log_print(6,iVar12,iVar14,*puVar4,puVar4[2],puVar4[1],puVar4[3]);
          pQVar10 = pQVar10 + 4;
        }
      }
      iVar12 = DAT_0002660c;
      iVar13 = DAT_00026608;
      __mutex = (pthread_mutex_t *)(this + 0x3f4);
      __android_log_print(6,DAT_00026600 + 0x26308,DAT_00026604 + 0x2630a);
      iVar6 = DAT_00026610;
      pQVar10 = this + 0x6d0;
      iVar16 = 0;
      iVar14 = DAT_00026614 + 0x26326;
      iVar9 = DAT_00026618 + 0x2632a;
      do {
        if (*(int *)(this + 0xd64) == 0) {
          iVar8 = *(int *)(this + 0xc);
        }
        else {
          iVar8 = 3;
        }
        if (iVar8 <= iVar16) {
          return 1;
        }
        if (*(int *)(this + 0xcd0) != 0) {
          if (*(int *)(pQVar10 + 0x608) == 0) {
            __android_log_print(6,iVar9,iVar6 + 0x263de,iVar13 + 0x26312);
          }
          else {
            __android_log_print(6,iVar14,iVar12 + 0x26316);
            pthread_mutex_lock(__mutex);
            __android_log_print(2,iVar14,DAT_0002661c + 0x2635a,iVar16);
            iVar8 = (**(code **)(*(int *)(this + 0xcd0) + 0x24))
                              (*(int *)(this + 0xcd0),*(undefined4 *)(pQVar10 + 0x608));
            if (iVar8 != 0) {
              __android_log_print(6,iVar14,DAT_00026620 + 0x2637a);
              __android_log_print(6,iVar14,DAT_00026624 + 0x26386);
              pthread_mutex_unlock(__mutex);
              return 0;
            }
            iVar8 = genlock_lock_buffer(**(undefined4 **)(pQVar10 + 0x608),2,1000,0);
            if (iVar8 != 0) {
              __android_log_print(6,iVar14,DAT_00026628 + 0x263ae,DAT_0002662c + 0x263b2);
              pthread_mutex_unlock(__mutex);
              return 1;
            }
            *(undefined4 *)pQVar10 = 1;
            pthread_mutex_unlock(__mutex);
            __android_log_print(6,iVar14,DAT_00026630 + 0x263d0);
          }
        }
        iVar7 = *(int *)(this + 0xd14);
        iVar8 = *(int *)(this + 0xd10);
        if (*(int **)(pQVar10 + 0x608) != (int *)0x0) {
          iVar11 = **(int **)(pQVar10 + 0x608);
          iVar15 = DAT_00026634 + 0x26408;
          __android_log_print(2,iVar15,DAT_00026638 + 0x2640a,*(undefined4 *)(iVar11 + 0xc),
                              *(undefined4 *)(iVar11 + 0x1c));
          pvVar3 = mmap((void *)0x0,*(size_t *)(iVar11 + 0x1c),3,1,*(int *)(iVar11 + 0xc),0);
          *(void **)(pQVar10 + -0x14) = pvVar3;
          if (pvVar3 == (void *)0xffffffff) {
            puVar4 = (undefined4 *)__errno();
            __android_log_print(6,iVar15,DAT_0002663c + 0x2643e,*puVar4);
            return 0;
          }
          iVar2 = (iVar8 * iVar7 * 3) / 2;
          __android_log_print(2,iVar15,DAT_00026640 + 0x2645e,iVar2,iVar2);
          FUN_0001f76c(iVar2,iVar2,iVar8 * iVar7 + 3U & 0xfffffffc,0,*(undefined4 *)(iVar11 + 0xc),0
                       ,*(undefined4 *)(pQVar10 + -0x14),3,iVar16 < 3,1,0);
        }
        iVar16 = iVar16 + 1;
        pQVar10 = pQVar10 + 4;
      } while( true );
    }
    iVar8 = allocate_ion_memory(this,(int *)(this + (iVar9 + 0x1f8) * 4),
                                (ion_allocation_data *)(this + (iVar9 + 0x82) * 0x10),
                                (ion_fd_data *)(this + (iVar9 + 0x12e) * 8),8,*(int *)(this + 0x430)
                                ,(int *)(this + (iVar9 + 0x1b8) * 4 + 4));
    if (iVar8 < 0) {
      iVar13 = DAT_000265dc + 0x261bc;
      iVar12 = DAT_000265e0 + 0x261be;
      iVar6 = DAT_000265e4 + 0x261c0;
      goto LAB_0002650c;
    }
    uVar5 = *(undefined4 *)(pQVar10 + 0x6e4);
    iVar8 = iVar9;
    __android_log_print(6,iVar12,iVar14,iVar16,iVar9,uVar5);
    puVar4 = (undefined4 *)
             (**(code **)(this + 0xd00))
                       (*(undefined4 *)(pQVar10 + 0x6e4),*(undefined4 *)(this + 0x430),1,
                        *(undefined4 *)(this + 0xd04));
    *(undefined4 **)(pQVar10 + 0x750) = puVar4;
    if (puVar4 == (undefined4 *)0x0) {
      iVar13 = DAT_000265e8 + 0x261fe;
      goto LAB_000262d0;
    }
    __android_log_print(6,iVar12,iVar6,*puVar4,puVar4[2],puVar4[1],puVar4[3]);
    __android_log_print(6,iVar12,iVar13 + 0x26226,iVar9,*(undefined4 *)(pQVar10 + 0x6e4));
    bVar1 = iVar9 < 3;
    iVar9 = iVar9 + 1;
    FUN_0001f76c(*(undefined4 *)(this + 0x430),*(undefined4 *)(this + 0x428),
                 *(undefined4 *)(this + 0x42c),0,*(undefined4 *)(pQVar10 + 0x6e4),0,
                 **(undefined4 **)(pQVar10 + 0x750),4,bVar1,1,0);
    pQVar10 = pQVar10 + 4;
  } while( true );
}



// Function: initRawSnapshot @ 00026670

/* android::QualcommCameraHardware::initRawSnapshot() */

bool android::QualcommCameraHardware::initRawSnapshot(void)

{
  undefined2 uVar1;
  undefined2 uVar2;
  QualcommCameraHardware *in_r0;
  int iVar3;
  int iVar4;
  undefined4 uVar5;
  uint uVar6;
  uint *extraout_r3;
  uint *puVar7;
  int iVar8;
  bool bVar9;
  undefined4 uVar10;
  
  iVar8 = DAT_00026718 + 0x2667e;
  __android_log_print(2,iVar8,DAT_0002671c + 0x26682);
  uVar1 = *(undefined2 *)(in_r0 + 0x43c);
  uVar2 = *(undefined2 *)(in_r0 + 0x43e);
  iVar3 = native_set_parms();
  *(undefined2 *)(in_r0 + 0x43c) = uVar1;
  *(undefined2 *)(in_r0 + 0x43e) = uVar2;
  if (iVar3 == 0) {
    __android_log_print(6,iVar8,DAT_00026720 + 0x266b6);
    bVar9 = false;
  }
  else {
    uVar6 = (uint)*(ushort *)(in_r0 + 0x45e);
    __android_log_print(2,iVar8,DAT_00026724 + 0x266d0,*(ushort *)(in_r0 + 0x45c) * uVar6,
                        (uint)*(ushort *)(in_r0 + 0x45c),uVar6);
    uVar10 = 2;
    iVar4 = createSnapshotMemory(in_r0,*(int *)(in_r0 + 0xc),*(int *)(in_r0 + 0xc),false,2);
    iVar3 = DAT_0002672c;
    bVar9 = iVar4 != 0;
    if (bVar9) {
      uVar5 = 2;
      puVar7 = (uint *)(DAT_0002672c + 0x266fc);
      *(undefined4 *)(DAT_0002672c + 0x26704) = 1;
      *puVar7 = (uint)*(ushort *)(in_r0 + 0x45e);
      *(uint *)(iVar3 + 0x26700) = (uint)*(ushort *)(in_r0 + 0x45c);
      iVar3 = DAT_00026730 + 0x2670e;
    }
    else {
      uVar5 = 6;
      iVar3 = DAT_00026728 + 0x266f2;
      puVar7 = extraout_r3;
    }
    __android_log_print(uVar5,iVar8,iVar3,puVar7,uVar10,uVar6);
  }
  return bVar9;
}



// Function: startCamera @ 00026734

/* android::QualcommCameraHardware::startCamera() */

undefined4 android::QualcommCameraHardware::startCamera(void)

{
  int in_r0;
  undefined4 uVar1;
  int *piVar2;
  int iVar3;
  int iVar4;
  int iVar5;
  int iVar6;
  int iVar7;
  int iVar8;
  int *piVar9;
  int iVar10;
  int *piVar11;
  void *pvStack_1c;
  
  iVar10 = DAT_00026a00 + 0x26742;
  __android_log_print(2,iVar10,DAT_00026a04 + 0x26746);
  iVar8 = DAT_00026a0c + 0x26758;
  if (*(int *)(DAT_00026a08 + 0x26752) == 7) {
    iVar8 = DAT_00026a10 + 0x2675e;
  }
  else {
    piVar9 = *(int **)(iVar8 + DAT_00026a14);
    __android_log_print(2,iVar10,DAT_00026a18 + 0x2676a,*piVar9);
    iVar4 = DAT_00026a24;
    if (*piVar9 == 0) {
      uVar1 = dlerror();
      __android_log_print(6,iVar10,DAT_00026a1c + 0x26780,uVar1);
      return 0;
    }
    uVar1 = dlsym(*piVar9,DAT_00026a20 + 0x26792);
    iVar3 = DAT_00026a2c + 0x2679e;
    **(undefined4 **)(iVar8 + DAT_00026a28) = uVar1;
    uVar1 = dlsym(*piVar9,iVar3);
    iVar3 = DAT_00026a30 + 0x267ac;
    **(undefined4 **)(iVar8 + iVar4) = uVar1;
    uVar1 = dlsym(*piVar9,iVar3);
    **(undefined4 **)(iVar8 + DAT_00026a34) = uVar1;
    uVar1 = dlsym(*piVar9,DAT_00026a38 + 0x267c4);
    iVar4 = DAT_00026a40 + 0x267d0;
    **(undefined4 **)(iVar8 + DAT_00026a3c) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    iVar4 = DAT_00026a48;
    **(undefined4 **)(iVar8 + DAT_00026a44) = uVar1;
    uVar1 = dlsym(*piVar9,DAT_00026a4c + 0x267e6);
    iVar3 = DAT_00026a54 + 0x267f6;
    **(undefined4 **)(iVar8 + DAT_00026a50) = uVar1;
    uVar1 = dlsym(*piVar9,iVar3);
    iVar3 = DAT_00026a58 + 0x26804;
    **(undefined4 **)(iVar8 + iVar4) = uVar1;
    uVar1 = dlsym(*piVar9,iVar3);
    iVar4 = DAT_00026a60 + 0x26814;
    **(undefined4 **)(iVar8 + DAT_00026a5c) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    iVar4 = DAT_00026a68 + 0x26828;
    **(undefined4 **)(iVar8 + DAT_00026a64) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    **(undefined4 **)(iVar8 + DAT_00026a6c) = uVar1;
    uVar1 = dlsym(*piVar9,DAT_00026a70 + 0x2683c);
    iVar4 = DAT_00026a78;
    iVar5 = DAT_00026a7c + 0x2684c;
    iVar6 = DAT_00026a80 + 0x26852;
    iVar7 = DAT_00026a84 + 0x26854;
    **(undefined4 **)(iVar8 + DAT_00026a74) = uVar1;
    iVar3 = DAT_00026a88;
    piVar11 = *(int **)(iVar8 + iVar4);
    *piVar11 = iVar5;
    iVar4 = DAT_00026a8c;
    piVar11[2] = iVar3 + 0x26860;
    iVar3 = DAT_00026a90;
    piVar11[3] = iVar7;
    piVar11[4] = iVar6;
    piVar11[1] = iVar3 + 0x2686e;
    uVar1 = dlsym(*piVar9,iVar4 + 0x26868);
    iVar4 = DAT_00026a98 + 0x26882;
    **(undefined4 **)(iVar8 + DAT_00026a94) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    iVar4 = DAT_00026aa0 + 0x26892;
    **(undefined4 **)(iVar8 + DAT_00026a9c) = uVar1;
    piVar2 = (int *)dlsym(*piVar9,iVar4);
    iVar4 = DAT_00026aa8 + 0x268a6;
    **(undefined4 **)(iVar8 + DAT_00026aa4) = piVar2;
    *piVar2 = iVar4;
    uVar1 = dlsym(*piVar9,DAT_00026aac + 0x268b0);
    iVar4 = DAT_00026ab4 + 0x268bc;
    **(undefined4 **)(iVar8 + DAT_00026ab0) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    **(undefined4 **)(iVar8 + DAT_00026ab8) = uVar1;
    uVar1 = dlsym(*piVar9,DAT_00026abc + 0x268d4);
    iVar4 = DAT_00026ac4 + 0x268e0;
    **(undefined4 **)(iVar8 + DAT_00026ac0) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    **(undefined4 **)(iVar8 + DAT_00026ac8) = uVar1;
    uVar1 = dlsym(*piVar9,DAT_00026acc + 0x268f8);
    iVar4 = DAT_00026ad4 + 0x26904;
    **(undefined4 **)(iVar8 + DAT_00026ad0) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    **(undefined4 **)(iVar8 + DAT_00026ad8) = uVar1;
    uVar1 = dlsym(*piVar9,DAT_00026adc + 0x2691c);
    iVar4 = DAT_00026ae4 + 0x26928;
    **(undefined4 **)(iVar8 + DAT_00026ae0) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    **(undefined4 **)(iVar8 + DAT_00026ae8) = uVar1;
    iVar4 = DAT_00026af0;
    piVar11[7] = DAT_00026aec + 0x26940;
    uVar1 = dlsym(*piVar9,iVar4 + 0x26944);
    iVar4 = DAT_00026af8 + 0x26952;
    **(undefined4 **)(iVar8 + DAT_00026af4) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    iVar4 = DAT_00026b00;
    **(undefined4 **)(iVar8 + DAT_00026afc) = uVar1;
    uVar1 = dlsym(*piVar9,DAT_00026b04 + 0x26968);
    iVar3 = DAT_00026b0c + 0x26978;
    **(undefined4 **)(iVar8 + DAT_00026b08) = uVar1;
    uVar1 = dlsym(*piVar9,iVar3);
    iVar3 = DAT_00026b10 + 0x26986;
    **(undefined4 **)(iVar8 + iVar4) = uVar1;
    uVar1 = dlsym(*piVar9,iVar3);
    iVar4 = DAT_00026b18 + 0x26996;
    **(undefined4 **)(iVar8 + DAT_00026b14) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    iVar4 = DAT_00026b20 + 0x269aa;
    **(undefined4 **)(iVar8 + DAT_00026b1c) = uVar1;
    uVar1 = dlsym(*piVar9,iVar4);
    **(undefined4 **)(iVar8 + DAT_00026b24) = uVar1;
    iVar8 = pthread_join(*(pthread_t *)(in_r0 + 0x5e8),&pvStack_1c);
    if (iVar8 == 0) {
      if (*(char *)(DAT_00026b2c + 0x269da) == '\0') {
        __android_log_print(6,iVar10,DAT_00026b30 + 0x269e8);
        return 0;
      }
      __android_log_print(2,iVar10,DAT_00026b34 + 0x269f8);
      return 1;
    }
    iVar8 = DAT_00026b28 + 0x269c8;
  }
  __android_log_print(6,iVar10,iVar8);
  return 0;
}



// Function: FUN_00026b38 @ 00026b38

void FUN_00026b38(String8 *param_1,undefined4 *param_2,int param_3)

{
  int iVar1;
  int iVar2;
  char *__format;
  char acStack_44 [32];
  int local_24;
  
  local_24 = **(int **)(DAT_00026bc0 + 0x26b4a);
  android::String8::String8(param_1);
  if (0 < param_3) {
    sprintf(acStack_44,(char *)(DAT_00026bc4 + 0x26b66),*param_2,param_2[1]);
    android::String8::append((char *)param_1);
  }
  __format = (char *)(DAT_00026bc8 + 0x26b82);
  iVar1 = 0;
  for (iVar2 = 1; iVar2 < param_3; iVar2 = iVar2 + 1) {
    sprintf(acStack_44,__format,*(undefined4 *)((int)param_2 + iVar1 + 8),
            *(undefined4 *)((int)param_2 + iVar1 + 0xc));
    android::String8::append((char *)param_1);
    iVar1 = iVar1 + 8;
  }
  if (local_24 != **(int **)(DAT_00026bcc + 0x26bae)) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail(param_1);
  }
  return;
}



// Function: flush @ 00026bd0

/* android::QualcommCameraHardware::FrameQueue::flush() */

void __thiscall android::QualcommCameraHardware::FrameQueue::flush(FrameQueue *this)

{
  pthread_mutex_lock((pthread_mutex_t *)(this + 8));
  android::VectorImpl::clear();
  pthread_mutex_unlock((pthread_mutex_t *)(this + 8));
  return;
}



// Function: runFrameThread @ 00026bf0

/* android::QualcommCameraHardware::runFrameThread(void*) */

void __thiscall
android::QualcommCameraHardware::runFrameThread(QualcommCameraHardware *this,void *param_1)

{
  ushort uVar1;
  ushort uVar2;
  int iVar3;
  int iVar4;
  int iVar5;
  int iVar6;
  int iVar7;
  QualcommCameraHardware *pQVar8;
  int *piVar9;
  pthread_mutex_t *__mutex;
  int iVar10;
  
  iVar7 = DAT_00026eb0;
  __android_log_print(2,DAT_00026eac + 0x26c02,DAT_00026ea8 + 0x26c06);
  iVar4 = DAT_00026ec4;
  iVar3 = DAT_00026ec0;
  iVar6 = DAT_00026ebc;
  iVar7 = iVar7 + 0x26c10;
  if (**(int **)(iVar7 + DAT_00026eb4) != 0) {
    (*(code *)**(undefined4 **)(iVar7 + DAT_00026eb8))(*(undefined4 *)(this + 0x69c));
    (*(code *)**(undefined4 **)(iVar7 + DAT_00026ec8))(*(undefined4 *)(DAT_00026ecc + 0x26c3e));
    (*(code *)**(undefined4 **)(iVar7 + iVar6))(*(undefined4 *)(this + 0x6a4));
    (*(code *)**(undefined4 **)(iVar7 + iVar3))
              (*(undefined4 *)(this + 0x6a8),*(undefined4 *)(this + 0x6ac));
    (*(code *)**(undefined4 **)(iVar7 + iVar4))(param_1);
  }
  iVar4 = DAT_00026ed8;
  iVar3 = DAT_00026ed4;
  iVar6 = DAT_00026ed0;
  __mutex = (pthread_mutex_t *)(this + 800);
  pthread_mutex_lock(__mutex);
  while (this[0x31c] != (QualcommCameraHardware)0x0) {
    __android_log_print(4,iVar6 + 0x26c84,iVar3 + 0x26c86);
    pthread_cond_wait((pthread_cond_t *)(this + 0x324),__mutex);
    __android_log_print(4,iVar6 + 0x26c84,iVar4 + 0x26c88);
  }
  pthread_mutex_unlock(__mutex);
  relinquishBuffers(this);
  FrameQueue::flush((FrameQueue *)(this + 0x330));
  (*(code *)**(undefined4 **)(iVar7 + DAT_00026edc))(2);
  if (this[0xcec] == (QualcommCameraHardware)0x0) {
    iVar3 = *(int *)(this + 0xd14);
    iVar4 = *(int *)(this + 0xd10);
    __android_log_print(6,DAT_00026ee4 + 0x26cf0,DAT_00026ee0 + 0x26cec);
    iVar7 = (iVar4 * iVar3 * 3) / 2;
    pQVar8 = this + 0xa1c;
    for (iVar6 = 0; iVar6 < *(int *)(this + 0xd5c); iVar6 = iVar6 + 1) {
      FUN_0001f76c(iVar7,iVar7,iVar4 * iVar3 + 3U & 0xfffffffc,0,*(undefined4 *)pQVar8,0,
                   *(undefined4 *)(pQVar8 + -0x1c),0xf,0,0,1);
      pQVar8 = pQVar8 + 0x68;
    }
  }
  if ((*(int *)(this + 0xd64) == 0) && (*(int *)(DAT_00026ee8 + 0x26d58) - 4U < 3)) {
    if (this[0xd75] != (QualcommCameraHardware)0x0) {
      __android_log_print(4,DAT_00026eec + 0x26d72,DAT_00026ef0 + 0x26d74,DAT_00026ef4 + 0x26d76);
      register_record_buffers(SUB41(this,0));
    }
    iVar7 = DAT_00026f0c;
    uVar1 = *(ushort *)(this + 0x43a);
    uVar2 = *(ushort *)(this + 0x438);
    piVar9 = (int *)(DAT_00026ef8 + 0x26d94);
    iVar4 = 0;
    iVar10 = DAT_00026f04 + 0x26dbe;
    __android_log_print(4,DAT_00026f08 + 0x26dc0,DAT_00026efc + 0x26d98,DAT_00026f00 + 0x26da4,
                        *piVar9);
    iVar6 = DAT_00026f10 + 0x26dd0;
    pQVar8 = this;
    for (iVar3 = 0; iVar3 < *piVar9; iVar3 = iVar3 + 1) {
      if (*(int *)(this + 0xcc0) != 0) {
        iVar5 = *(int *)(this + 0xcc0) + iVar4;
        FUN_0001f76c(*(undefined4 *)(this + 0x424),*(undefined4 *)(this + 0x424),
                     (uint)uVar2 * (uint)uVar1 + 0x7ff & 0xfffff800,0,*(undefined4 *)(iVar5 + 0x2c),
                     0,*(undefined4 *)(iVar5 + 0x10),0xe,0,0,0);
        if (*(int *)(pQVar8 + 0x78c) != 0) {
          (**(code **)(*(int *)(pQVar8 + 0x78c) + 0xc))();
          *(undefined4 *)(pQVar8 + 0x78c) = 0;
          close(*(int *)(pQVar8 + 0x714));
          if ((*(int *)(this + 0x3ec) != 0) && (*(int **)(pQVar8 + 0x7b4) != (int *)0x0)) {
            iVar5 = **(int **)(pQVar8 + 0x7b4);
            __android_log_print(4,iVar10,iVar7 + 0x26dd4,iVar6,iVar3);
            native_handle_delete(*(undefined4 *)(iVar5 + 4));
            (**(code **)(*(int *)(pQVar8 + 0x7b4) + 0xc))();
            *(undefined4 *)(pQVar8 + 0x7b4) = 0;
          }
          deallocate_ion_memory
                    (this,(int *)(this + (iVar3 + 0x1fe) * 4 + 4),
                     (ion_fd_data *)(this + (iVar3 + 0x135) * 8));
        }
      }
      iVar4 = iVar4 + 0x68;
      pQVar8 = pQVar8 + 4;
    }
  }
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x35c));
  this[0x358] = (QualcommCameraHardware)0x0;
  pthread_cond_signal((pthread_cond_t *)(this + 0x360));
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x35c));
  __android_log_print(2,DAT_00026f14 + 0x26e9e,DAT_00026f18 + 0x26ea0);
  return;
}



// Function: get @ 00026f1c

/* android::QualcommCameraHardware::FrameQueue::get() */

undefined4 __thiscall android::QualcommCameraHardware::FrameQueue::get(FrameQueue *this)

{
  pthread_mutex_t *__mutex;
  undefined4 uVar1;
  
  __mutex = (pthread_mutex_t *)(this + 8);
  pthread_mutex_lock(__mutex);
  do {
    uVar1 = 0;
    if (this[0x10] == (FrameQueue)0x0) {
LAB_00026f42:
      pthread_mutex_unlock(__mutex);
      return uVar1;
    }
    if (*(int *)(this + 0x1c) != 0) {
      uVar1 = **(undefined4 **)(this + 0x18);
      android::VectorImpl::removeItemsAt((uint)(this + 0x14),0);
      goto LAB_00026f42;
    }
    pthread_cond_wait((pthread_cond_t *)(this + 0xc),__mutex);
  } while( true );
}



// Function: runPreviewThread @ 00026f60

/* android::QualcommCameraHardware::runPreviewThread(void*) */

void android::QualcommCameraHardware::runPreviewThread(void *param_1)

{
  int iVar1;
  void *__dest;
  msm_frame *pmVar2;
  int iVar3;
  int *piVar4;
  undefined4 uVar5;
  int iVar6;
  int *piVar7;
  code *pcVar8;
  int iVar9;
  int iVar10;
  int iVar11;
  int iVar12;
  undefined8 uVar13;
  int local_b0;
  uint local_ac [9];
  int local_88;
  int local_84;
  int local_80;
  FrameQueue *local_7c;
  sp<android::QualcommCameraHardware::IonPool> *local_78;
  sp<android::QualcommCameraHardware::IonPool> *local_74;
  pthread_cond_t *local_70;
  pthread_cond_t *local_6c;
  sp<android::QualcommCameraHardware::IonPool> *local_68;
  int local_64;
  pthread_mutex_t *local_60;
  pthread_mutex_t *local_5c;
  int *local_58;
  code *local_54;
  pthread_mutex_t *local_50;
  pthread_mutex_t *local_4c;
  int *local_48;
  undefined4 *local_44;
  pthread_mutex_t *local_40;
  int local_3c;
  undefined1 auStack_34 [4];
  native_handle **local_30;
  int local_2c;
  
  iVar1 = DAT_00027604;
  local_7c = (FrameQueue *)((int)param_1 + 0x330);
  local_3c = DAT_000275f0 + 0x26f76;
  local_60 = (pthread_mutex_t *)((int)param_1 + 0x404);
  local_2c = **(int **)(local_3c + DAT_000275f4);
  local_5c = (pthread_mutex_t *)((int)param_1 + 0x378);
  local_50 = (pthread_mutex_t *)((int)param_1 + 0x410);
  local_68 = (sp<android::QualcommCameraHardware::IonPool> *)((int)param_1 + 0x2f4);
  local_70 = (pthread_cond_t *)((int)param_1 + 0x380);
  local_6c = (pthread_cond_t *)((int)param_1 + 0x414);
  local_74 = (sp<android::QualcommCameraHardware::IonPool> *)((int)param_1 + 0x310);
  local_40 = (pthread_mutex_t *)((int)param_1 + 0x3f4);
  local_44 = (undefined4 *)(DAT_000275fc + 0x26fc8);
  local_48 = (int *)(DAT_00027600 + 0x26fce);
  local_30 = (native_handle **)0x0;
  piVar7 = (int *)(DAT_00027604 + 0x26fd2);
  local_78 = (sp<android::QualcommCameraHardware::IonPool> *)((int)param_1 + 0x2ec);
  local_58 = (int *)(DAT_00027608 + 0x26fd8);
  local_4c = (pthread_mutex_t *)((int)param_1 + 0x3a0);
  local_80 = DAT_000275f8;
  do {
    pmVar2 = (msm_frame *)FrameQueue::get(local_7c);
    if (pmVar2 == (msm_frame *)0x0) {
      pthread_mutex_lock((pthread_mutex_t *)((int)param_1 + 800));
      *(undefined1 *)((int)param_1 + 0x31c) = 0;
      pthread_cond_signal((pthread_cond_t *)((int)param_1 + 0x324));
      pthread_mutex_unlock((pthread_mutex_t *)((int)param_1 + 800));
      if (local_2c == **(int **)(local_3c + DAT_000275f4)) {
        return;
      }
                    /* WARNING: Subroutine does not return */
      __stack_chk_fail();
    }
    if (*(int *)((int)param_1 + 0xd08) != 0) {
      debugShowPreviewFPS();
    }
    pthread_mutex_lock(local_60);
    local_54 = *(code **)((int)param_1 + 0xcfc);
    local_64 = *(int *)((int)param_1 + 0xcf0);
    pcVar8 = *(code **)((int)param_1 + 0xcf8);
    iVar10 = *(int *)((int)param_1 + 0xd04);
    pthread_mutex_unlock(local_60);
    pthread_mutex_lock(local_5c);
    if (*(char *)((int)param_1 + 0x375) != '\0') {
      pthread_cond_signal(local_70);
    }
    pthread_mutex_unlock(local_5c);
    piVar4 = *(int **)(pmVar2 + 0x30);
    *(undefined1 *)((int)param_1 + 0xccc) = 1;
    if ((*piVar4 == 0) || (piVar4[2] == 0)) {
      *local_44 = 0;
      local_44[1] = 0;
      local_44[2] = *piVar4;
      iVar3 = piVar4[2];
      local_44[3] = iVar3;
      if (*(char *)((int)param_1 + 0xd3c) != '\0') {
        local_88 = iVar3;
        (**(code **)(*(int *)((int)param_1 + 0xcd0) + 0x14))(*(int *)((int)param_1 + 0xcd0),0,0);
        *(undefined1 *)((int)param_1 + 0xd3c) = 0;
      }
    }
    else {
      iVar6 = ((uint)((piVar4[1] + 1) - *piVar4) >> 1) - 1;
      *local_48 = iVar6;
      iVar3 = ((uint)((piVar4[3] + 1) - piVar4[2]) >> 1) - 1;
      local_48[1] = iVar3;
      if (iVar6 < 0) {
        *local_48 = 0;
      }
      if (iVar3 < 0) {
        *(undefined4 *)(local_80 + 0x27078) = 0;
      }
      *(int *)(iVar1 + 0x26fda) = *piVar7 + *piVar4;
      local_88 = *(int *)(iVar1 + 0x26fd6) + piVar4[2];
      *(int *)(iVar1 + 0x26fde) = local_88;
      (**(code **)(*(int *)((int)param_1 + 0xcd0) + 0x14))
                (*(int *)((int)param_1 + 0xcd0),*piVar7,*(undefined4 *)(iVar1 + 0x26fd6));
      *(undefined1 *)((int)param_1 + 0xd3c) = 1;
    }
    if ((*local_58 == 6) && (*(char *)((int)param_1 + 1000) != '\0')) {
      __android_log_print(3,DAT_0002760c + 0x270ea,DAT_00027610 + 0x270ec);
      sp<android::QualcommCameraHardware::IonPool>::clear(local_78);
      sp<android::QualcommCameraHardware::IonPool>::clear(local_68);
      if (*(int *)((int)param_1 + 0xd64) == 0) {
        sp<android::QualcommCameraHardware::IonPool>::clear(local_68);
        sp<android::QualcommCameraHardware::IonPool>::clear(local_74);
      }
      *(undefined1 *)((int)param_1 + 1000) = 0;
    }
    *(undefined4 *)(DAT_00027614 + 0x27122) = *(undefined4 *)(pmVar2 + 0x10);
    iVar3 = mapBuffer(param_1,pmVar2);
    if ((*(int *)((int)param_1 + 0x69c) == 0) || (*(int *)((int)param_1 + 0x6a0) == 0)) {
      if (iVar3 < 0) {
        __android_log_print(6,DAT_00027680 + 0x2736a,DAT_00027684 + 0x2736c);
      }
      else {
        if ((pcVar8 != (code *)0x0) && (local_64 << 0x1b < 0)) {
          if (*(int *)(DAT_00027634 + 0x271fe) == 4 || *(int *)(DAT_00027634 + 0x271fe) == 1) {
            piVar4 = (int *)(**(code **)((int)param_1 + 0xd00))
                                      (*(undefined4 *)((int)param_1 + iVar3 * 0x68 + 0xa1c),
                                       (*(int *)((int)param_1 + 0xd14) *
                                        *(int *)((int)param_1 + 0xd10) * 3) / 2,1,
                                       *(undefined4 *)((int)param_1 + 0xd04));
            if ((piVar4 == (int *)0x0) || (*piVar4 == 0)) {
              __android_log_print(6,DAT_00027638 + 0x27258,DAT_0002763c + 0x2725a,
                                  DAT_00027640 + 0x2725c);
            }
            else {
              local_88 = iVar10;
              (*pcVar8)(0x10,piVar4,0,0);
              (*(code *)piVar4[3])(piVar4);
            }
          }
          else {
            local_88 = iVar10;
            (*pcVar8)(0x10,*(undefined4 *)((int)param_1 + (iVar3 + 0x1ce) * 4),0,0);
          }
        }
        pthread_mutex_lock(local_40);
        if (*(int *)((int)param_1 + 0xcd0) != 0) {
          if (*(int *)((int)param_1 + iVar3 * 0x10 + 0xc6c) == 1) {
            iVar6 = genlock_unlock_buffer(**(undefined4 **)((int)param_1 + iVar3 * 0x10 + 0xc64));
            if (iVar6 == 2) {
              iVar6 = DAT_00027644 + 0x272c4;
              iVar9 = DAT_00027648 + 0x272c6;
              iVar11 = DAT_0002764c + 0x272c8;
              goto LAB_000272dc;
            }
            *(undefined4 *)((int)param_1 + iVar3 * 0x10 + 0xc6c) = 0;
          }
          else {
            iVar6 = DAT_00027650 + 0x272da;
            iVar9 = DAT_00027654 + 0x272dc;
            iVar11 = DAT_00027658 + 0x272de;
LAB_000272dc:
            __android_log_print(6,iVar6,iVar9,iVar11);
            pthread_mutex_unlock(local_40);
          }
          iVar6 = (**(code **)(*(int *)((int)param_1 + 0xcd0) + 4))
                            (*(int *)((int)param_1 + 0xcd0),
                             *(undefined4 *)((int)param_1 + iVar3 * 0x10 + 0xc64));
          if (iVar6 != 0) {
            local_88 = *(int *)((int)param_1 + iVar3 * 0x68 + 0xa1c);
            local_84 = iVar6;
            __android_log_print(6,DAT_0002765c + 0x27308,DAT_00027660 + 0x2730a,
                                DAT_00027664 + 0x2730c);
          }
          iVar6 = (*(code *)**(undefined4 **)((int)param_1 + 0xcd0))
                            (*(undefined4 **)((int)param_1 + 0xcd0),&local_30,auStack_34);
          if (iVar6 == 0) {
            iVar6 = (**(code **)(*(int *)((int)param_1 + 0xcd0) + 0x24))
                              (*(int *)((int)param_1 + 0xcd0),local_30);
            if (iVar6 == 0) goto LAB_00027358;
            iVar9 = DAT_00027674 + 0x2734e;
            iVar11 = DAT_00027678 + 0x27350;
            iVar12 = DAT_0002767c + 0x27352;
          }
          else {
            iVar9 = DAT_00027668 + 0x27334;
            iVar11 = DAT_0002766c + 0x27336;
            iVar12 = DAT_00027670 + 0x27338;
          }
          local_88 = iVar6;
          __android_log_print(6,iVar9,iVar11,iVar12);
        }
LAB_00027358:
        pthread_mutex_unlock(local_40);
      }
    }
    else {
      local_88 = *(int *)((int)param_1 + 0x69c);
      local_84 = *(int *)((int)param_1 + 0x6a0);
      __android_log_print(6,DAT_00027618 + 0x2714a,DAT_0002761c + 0x2714e,DAT_00027620 + 0x27150);
      if (pcVar8 != (code *)0x0) {
        if (*(int *)(DAT_00027624 + 0x27166) == 1) {
          piVar4 = (int *)(**(code **)((int)param_1 + 0xd00))
                                    (*(undefined4 *)((int)param_1 + iVar3 * 0x68 + 0xa1c),
                                     (*(int *)((int)param_1 + 0xd14) *
                                      *(int *)((int)param_1 + 0xd10) * 3) / 2,1,
                                     *(undefined4 *)((int)param_1 + 0xd04));
          if ((piVar4 == (int *)0x0) || (*piVar4 == 0)) {
            __android_log_print(6,DAT_00027628 + 0x271b0,DAT_0002762c + 0x271b2,
                                DAT_00027630 + 0x271b4);
          }
          else {
            local_88 = iVar10;
            (*pcVar8)(0x10,piVar4,0,0);
            (*(code *)piVar4[3])(piVar4);
          }
        }
        else {
          local_88 = iVar10;
          (*pcVar8)(0x10,*(undefined4 *)((int)param_1 + (iVar3 + 0x1ce) * 4),0,0);
        }
      }
    }
    if (((2 < *(int *)(DAT_00027688 + 0x27374) - 4U) && (local_54 != (code *)0x0)) &&
       (local_64 << 0x1a < 0)) {
      if (*(int *)((int)param_1 + 0x3ec) == 0) {
        uVar13 = systemTime(1);
        uVar5 = *(undefined4 *)((int)param_1 + (iVar3 + 0x1ce) * 4);
      }
      else {
        if (*(int *)((int)param_1 + (iVar3 + 0x1ec) * 4 + 4) == 0) goto LAB_000273e6;
        uVar13 = systemTime(1);
        uVar5 = *(undefined4 *)((int)param_1 + (iVar3 + 0x1ec) * 4 + 4);
      }
      local_88 = 0;
      local_84 = iVar10;
      (*local_54)((int)uVar13,(int)((ulonglong)uVar13 >> 0x20),0x20,uVar5);
      pthread_mutex_lock(local_50);
      if (*(char *)((int)param_1 + 0x3fd) == '\0') {
        pthread_cond_wait(local_6c,local_50);
      }
      *(undefined1 *)((int)param_1 + 0x3fd) = 0;
      pthread_mutex_unlock(local_50);
    }
LAB_000273e6:
    if (*(int *)(DAT_0002768c + 0x273ec) == 6) {
      pthread_mutex_lock(local_4c);
      if ((*(int *)((int)param_1 + 0x398) == 1) && (*(char *)((int)param_1 + 0x39c) != '\0')) {
        iVar6 = 0;
        *(undefined1 *)((int)param_1 + 0x39c) = 0;
        iVar9 = *(int *)(pmVar2 + 0x3c);
        iVar11 = (int)*(short *)(iVar9 + 4);
        local_b0 = iVar11 << 2;
        iVar10 = 0;
        piVar4 = &local_b0;
        do {
          if (iVar10 < iVar11) {
            iVar12 = iVar9 + iVar6;
            piVar4[1] = (uint)*(ushort *)(iVar12 + 6);
            piVar4[2] = (uint)*(ushort *)(iVar12 + 8);
            piVar4[3] = (uint)*(ushort *)(iVar12 + 10);
            piVar4[4] = (uint)*(ushort *)(iVar12 + 10);
          }
          else {
            piVar4[1] = -1;
            piVar4[2] = -1;
            piVar4[3] = -1;
            piVar4[4] = -1;
          }
          iVar10 = iVar10 + 1;
          iVar6 = iVar6 + 8;
          piVar4 = piVar4 + 4;
        } while (iVar10 != 2);
        if (*(int *)((int)param_1 + 0x300) == 0) {
          pthread_mutex_unlock(local_4c);
          __android_log_print(6,DAT_00027698 + 0x274ac,DAT_0002769c + 0x274ae);
        }
        else {
          __android_log_print(2,DAT_00027690 + 0x27476,DAT_00027694 + 0x27478);
          piVar4 = *(int **)(*(int *)((int)param_1 + 0x300) + 0x18);
          __dest = (void *)FUN_0001de20((int)piVar4 + *(int *)(*piVar4 + -0xc));
          memcpy(__dest,&local_b0,0x24);
          pthread_mutex_unlock(local_4c);
        }
      }
      else {
        pthread_mutex_unlock(local_4c);
      }
    }
    if ((*(int *)((int)param_1 + 0x69c) == 0) || (*(int *)((int)param_1 + 0x6a0) == 0)) {
      iVar10 = mapFrame(param_1,local_30);
      if (iVar10 < 0) {
        iVar3 = DAT_000276bc + 0x2756c;
        __android_log_print(6,iVar3,DAT_000276c0 + 0x2756e);
        pthread_mutex_lock(local_40);
        __android_log_print(2,iVar3,DAT_000276c4 + 0x27582);
        iVar10 = (**(code **)(*(int *)((int)param_1 + 0xcd0) + 8))
                           (*(int *)((int)param_1 + 0xcd0),local_30);
        if (iVar10 != 0) {
          __android_log_print(6,iVar3,DAT_000276c8 + 0x2759c,DAT_000276cc + 0x2759e);
        }
        pthread_mutex_unlock(local_40);
      }
      else {
        (*(code *)**(undefined4 **)(local_3c + DAT_000276ac))
                  (2,(int)param_1 + iVar10 * 0x68 + 0x9f0);
        iVar3 = genlock_lock_buffer(*local_30,2,1000);
        if (iVar3 == 0) {
          *(undefined4 *)((int)param_1 + iVar10 * 0x10 + 0xc6c) = 1;
        }
        else {
          __android_log_print(6,DAT_000276b0 + 0x27542,DAT_000276b4 + 0x27544,DAT_000276b8 + 0x27546
                             );
          *(undefined4 *)((int)param_1 + iVar10 * 0x10 + 0xc6c) = 0;
        }
      }
    }
    else {
      local_88 = *(int *)((int)param_1 + 0x69c);
      local_84 = *(int *)((int)param_1 + 0x6a0);
      __android_log_print(6,DAT_000276a0 + 0x274d0,DAT_000276a4 + 0x274d4,DAT_000276a8 + 0x274d6);
      (*(code *)**(undefined4 **)(local_3c + DAT_000276ac))(2,(int)param_1 + iVar3 * 0x68 + 0x9f0);
    }
  } while( true );
}



// Function: add @ 000276d0

/* android::QualcommCameraHardware::FrameQueue::add(msm_frame*) */

undefined8 android::QualcommCameraHardware::FrameQueue::add(msm_frame *param_1)

{
  msm_frame mVar1;
  
  pthread_mutex_lock((pthread_mutex_t *)(param_1 + 8));
  mVar1 = param_1[0x10];
  if (mVar1 != (msm_frame)0x0) {
    android::VectorImpl::add(param_1 + 0x14);
    pthread_cond_signal((pthread_cond_t *)(param_1 + 0xc));
  }
  pthread_mutex_unlock((pthread_mutex_t *)(param_1 + 8));
  return CONCAT44(param_1,(uint)(mVar1 != (msm_frame)0x0));
}



// Function: receivePreviewFrame @ 00027708

/* android::QualcommCameraHardware::receivePreviewFrame(msm_frame*) */

void __thiscall
android::QualcommCameraHardware::receivePreviewFrame
          (QualcommCameraHardware *this,msm_frame *param_1)

{
  int iVar1;
  int iVar2;
  code *pcVar3;
  int iVar4;
  
  iVar4 = DAT_00027790 + 0x27716;
  if (this[0x34] == (QualcommCameraHardware)0x0) {
    __android_log_print(6,DAT_00027794 + 0x27722,DAT_00027798 + 0x27724);
  }
  else {
    if ((*(int *)(DAT_0002779c + 0x2772e) == 3) && (*(int *)(DAT_000277a0 + 0x27738) == 1)) {
      (*(code *)**(undefined4 **)(iVar4 + DAT_000277a4))(param_1);
    }
    iVar1 = DAT_000277ac;
    iVar2 = DAT_000277a8;
    if (*(int *)(this + 0x67c) < 1) {
      iVar2 = FrameQueue::add((msm_frame *)(this + 0x330));
      if (iVar2 != 0) {
        return;
      }
      pcVar3 = (code *)**(undefined4 **)(iVar4 + DAT_000277b0);
      goto LAB_00027788;
    }
    *(int *)(this + 0x67c) = *(int *)(this + 0x67c) + -1;
    __android_log_print(4,iVar2 + 0x27762,iVar1 + 0x27764);
  }
  pcVar3 = (code *)**(undefined4 **)(iVar4 + DAT_000277b0);
LAB_00027788:
  (*pcVar3)(2,param_1);
  return;
}



// Function: FUN_000277b4 @ 000277b4

VectorImpl * FUN_000277b4(VectorImpl *param_1)

{
  *(int *)param_1 = *(int *)(DAT_000277d4 + 0x277c0 + DAT_000277d8) + 8;
  android::VectorImpl::finish_vector();
  android::VectorImpl::~VectorImpl(param_1);
  return param_1;
}



// Function: ~FrameQueue @ 000277dc

/* android::QualcommCameraHardware::FrameQueue::~FrameQueue() */

FrameQueue * __thiscall android::QualcommCameraHardware::FrameQueue::~FrameQueue(FrameQueue *this)

{
  *(int *)this = *(int *)(DAT_00027814 + 0x277e8 + DAT_00027818) + 8;
  flush(this);
  FUN_000277b4(this + 0x14);
  pthread_cond_destroy((pthread_cond_t *)(this + 0xc));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 8));
  android::RefBase::~RefBase((RefBase *)this);
  return this;
}



// Function: ~QualcommCameraHardware @ 0002781c

/* android::QualcommCameraHardware::~QualcommCameraHardware() */

QualcommCameraHardware * __thiscall
android::QualcommCameraHardware::~QualcommCameraHardware(QualcommCameraHardware *this)

{
  int iVar1;
  void *pvVar2;
  QualcommCameraHardware *pQVar3;
  QualcommCameraHardware *pQVar4;
  
  iVar1 = DAT_00027a74;
  *(int *)this = *(int *)(DAT_00027a6c + 0x27824 + DAT_00027a70) + 8;
  __android_log_print(4,iVar1 + 0x27836,DAT_00027a78 + 0x27838);
  if (*(int *)(DAT_00027a7c + 0x27840) - 4U < 3) {
    if (*(void **)(this + 0xcc0) != (void *)0x0) {
      operator_delete__(*(void **)(this + 0xcc0));
    }
    *(undefined4 *)(this + 0xcc0) = 0;
    if (*(void **)(this + 0xcc8) != (void *)0x0) {
      operator_delete__(*(void **)(this + 0xcc8));
    }
  }
  if (*(void **)(this + 0x318) != (void *)0x0) {
    android::RefBase::decStrong(*(void **)(this + 0x318));
    *(undefined4 *)(this + 0x318) = 0;
  }
  __android_log_print(4,DAT_00027a80 + 0x27884,DAT_00027a84 + 0x27886);
  pthread_cond_destroy((pthread_cond_t *)(this + 0xd8c));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0xd88));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0xd78));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x5d4));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x5d0));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x418));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x414));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x410));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x40c));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x408));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x404));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x400));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3f8));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3f4));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3f0));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x3dc));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3d8));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x3d0));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3cc));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x3c4));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3c0));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3b8));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x3b4));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3b0));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3a8));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x3a0));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x394));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x390));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x380));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x37c));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x378));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x36c));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x368));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x360));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x35c));
  FrameQueue::~FrameQueue((FrameQueue *)(this + 0x330));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x32c));
  pthread_cond_destroy((pthread_cond_t *)(this + 0x324));
  pthread_mutex_destroy((pthread_mutex_t *)(this + 800));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x318));
  if (*(void **)(this + 0x314) != (void *)0x0) {
    android::RefBase::decStrong(*(void **)(this + 0x314));
  }
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x310));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x30c));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x308));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x304));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x300));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x2fc));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x2f8));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x2f4));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x2f0));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x2ec));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x2e8));
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)(this + 0x2e4));
  pQVar3 = this + 0x2e0;
  while (pQVar3 != this + 0x2c8) {
    pQVar4 = pQVar3 + -4;
    pvVar2 = *(void **)pQVar3;
    pQVar3 = pQVar4;
    if (pvVar2 != (void *)0x0) {
      android::RefBase::decStrong(pvVar2);
    }
  }
  pthread_mutex_destroy((pthread_mutex_t *)(this + 0x38));
  android::CameraParameters::~CameraParameters((CameraParameters *)(this + 0x18));
  android::RefBase::~RefBase((RefBase *)this);
  return this;
}



// Function: ~QualcommCameraHardware @ 00027a88

/* android::QualcommCameraHardware::~QualcommCameraHardware() */

QualcommCameraHardware * __thiscall
android::QualcommCameraHardware::~QualcommCameraHardware(QualcommCameraHardware *this)

{
  ~QualcommCameraHardware(this);
  operator_delete(this);
  return this;
}



// Function: ~FrameQueue @ 00027a9c

/* android::QualcommCameraHardware::FrameQueue::~FrameQueue() */

FrameQueue * __thiscall android::QualcommCameraHardware::FrameQueue::~FrameQueue(FrameQueue *this)

{
  ~FrameQueue(this);
  operator_delete(this);
  return this;
}



// Function: FUN_00027ab0 @ 00027ab0

void * FUN_00027ab0(void *param_1)

{
  FUN_000277b4();
  operator_delete(param_1);
  return param_1;
}



// Function: do_move_forward @ 00027ac4

/* android::Vector<msm_frame*>::do_move_forward(void*, void const*, unsigned int) const */

void __thiscall
android::Vector<msm_frame*>::do_move_forward
          (Vector<msm_frame*> *this,void *param_1,void *param_2,uint param_3)

{
  memmove(param_1,param_2,param_3 << 2);
  return;
}



// Function: FrameQueue @ 00027ad4

/* android::QualcommCameraHardware::FrameQueue::FrameQueue() */

FrameQueue * __thiscall android::QualcommCameraHardware::FrameQueue::FrameQueue(FrameQueue *this)

{
  int iVar1;
  int iVar2;
  
  iVar1 = DAT_00027b1c;
  android::RefBase::RefBase((RefBase *)this);
  *(int *)this = *(int *)(iVar1 + 0x27ae6 + DAT_00027b20) + 8;
  pthread_mutex_init((pthread_mutex_t *)(this + 8),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0xc),(pthread_condattr_t *)0x0);
  android::VectorImpl::VectorImpl((VectorImpl *)(this + 0x14),4,7);
  iVar2 = DAT_00027b24;
  this[0x10] = (FrameQueue)0x0;
  *(int *)(this + 0x14) = *(int *)(iVar1 + 0x27ae6 + iVar2) + 8;
  return this;
}



// Function: QualcommCameraHardware @ 00027b28

/* android::QualcommCameraHardware::QualcommCameraHardware() */

void __thiscall
android::QualcommCameraHardware::QualcommCameraHardware(QualcommCameraHardware *this)

{
  undefined4 uVar1;
  uint uVar2;
  void *pvVar3;
  int iVar4;
  QualcommCameraHardware *pQVar5;
  int iVar6;
  undefined4 *puVar7;
  int *piVar8;
  int iVar9;
  int iVar10;
  uint *puVar11;
  undefined1 *puVar12;
  MMCameraDL aMStack_7c [4];
  char acStack_78 [92];
  int local_1c;
  
  iVar9 = DAT_000280dc + 0x27b38;
  iVar10 = 0;
  local_1c = **(int **)(iVar9 + DAT_000280e0);
  android::RefBase::RefBase((RefBase *)this);
  *(int *)this = *(int *)(iVar9 + DAT_000280e4) + 8;
  android::CameraParameters::CameraParameters((CameraParameters *)(this + 0x18));
  this[0x34] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x38),(pthread_mutexattr_t *)0x0);
  iVar6 = 0;
  this[0x3c] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0x40) = 0;
  *(undefined4 *)(this + 0x44) = 0;
  do {
    *(undefined4 *)(this + iVar6 + 0x2cc) = 0;
    iVar6 = iVar6 + 4;
  } while (iVar6 != 0x18);
  *(undefined4 *)(this + 0x2e4) = 0;
  *(undefined4 *)(this + 0x2e8) = 0;
  *(undefined4 *)(this + 0x2ec) = 0;
  *(undefined4 *)(this + 0x2f0) = 0;
  *(undefined4 *)(this + 0x2f4) = 0;
  *(undefined4 *)(this + 0x2f8) = 0;
  *(undefined4 *)(this + 0x2fc) = 0;
  *(undefined4 *)(this + 0x300) = 0;
  *(undefined4 *)(this + 0x304) = 0;
  *(undefined4 *)(this + 0x308) = 0;
  *(undefined4 *)(this + 0x30c) = 0;
  *(undefined4 *)(this + 0x310) = 0;
  *(undefined4 *)(this + 0x314) = 0;
  *(undefined4 *)(this + 0x318) = 0;
  this[0x31c] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 800),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x324),(pthread_condattr_t *)0x0);
  this[0x328] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x32c),(pthread_mutexattr_t *)0x0);
  FrameQueue::FrameQueue((FrameQueue *)(this + 0x330));
  this[0x358] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x35c),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x360),(pthread_condattr_t *)0x0);
  this[0x365] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x368),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x36c),(pthread_condattr_t *)0x0);
  this[0x374] = (QualcommCameraHardware)0x0;
  this[0x375] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x378),(pthread_mutexattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x37c),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x380),(pthread_condattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x390),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x394),(pthread_condattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3a0),(pthread_mutexattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3a8),(pthread_mutexattr_t *)0x0);
  this[0x3ac] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3b0),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x3b4),(pthread_condattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3b8),(pthread_mutexattr_t *)0x0);
  this[0x3bc] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3c0),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x3c4),(pthread_condattr_t *)0x0);
  this[0x3c8] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3cc),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x3d0),(pthread_condattr_t *)0x0);
  this[0x3d4] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3d8),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x3dc),(pthread_condattr_t *)0x0);
  this[1000] = (QualcommCameraHardware)0x1;
  this[0x3e0] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0x3e4) = 0;
  *(undefined4 *)(this + 0x3ec) = 0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3f0),(pthread_mutexattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3f4),(pthread_mutexattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x3f8),(pthread_mutexattr_t *)0x0);
  this[0x3fd] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x400),(pthread_mutexattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x404),(pthread_mutexattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x408),(pthread_mutexattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x40c),(pthread_mutexattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x410),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x414),(pthread_condattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0x418),(pthread_condattr_t *)0x0);
  *(undefined4 *)(this + 0x420) = 0;
  *(undefined4 *)(this + 0x428) = 0;
  *(undefined4 *)(this + 0x42c) = 0;
  this[0x5cc] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0x5d0),(pthread_mutexattr_t *)0x0);
  pthread_mutex_init((pthread_mutex_t *)(this + 0x5d4),(pthread_mutexattr_t *)0x0);
  this[0x618] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0x63c) = 0;
  *(undefined4 *)(this + 0x670) = 0;
  *(undefined4 *)(this + 0x674) = 0;
  *(undefined4 *)(this + 0x67c) = 0;
  *(undefined4 *)(this + 0x680) = 0;
  *(undefined4 *)(this + 0x688) = 0;
  *(undefined4 *)(this + 0x68c) = 0;
  this[0x698] = (QualcommCameraHardware)0x0;
  this[0xccc] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0xcd0) = 0;
  this[0xcec] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0xcf0) = 0;
  *(undefined4 *)(this + 0xcf4) = 0;
  *(undefined4 *)(this + 0xcf8) = 0;
  *(undefined4 *)(this + 0xcfc) = 0;
  *(undefined4 *)(this + 0xd04) = 0;
  *(undefined4 *)(this + 0xd08) = 0;
  this[0xd1c] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0xd20) = 0;
  *(undefined4 *)(this + 0xd24) = 0;
  this[0xd28] = (QualcommCameraHardware)0x0;
  this[0xd34] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0xd38) = 0;
  this[0xd3c] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0xd40) = 0;
  *(undefined4 *)(this + 0xd44) = 0;
  this[0xd48] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0xd4c) = 0;
  *(undefined4 *)(this + 0xd50) = 0;
  *(undefined4 *)(this + 0xd54) = 0;
  *(undefined4 *)(this + 0xd58) = 0;
  *(undefined4 *)(this + 0xd5c) = 0;
  *(undefined4 *)(this + 0xd60) = 0;
  *(undefined4 *)(this + 0xd64) = 0;
  *(undefined4 *)(this + 0xd68) = 0;
  this[0xd6c] = (QualcommCameraHardware)0x0;
  this[0xd74] = (QualcommCameraHardware)0x0;
  this[0xd75] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0xd78),(pthread_mutexattr_t *)0x0);
  *(undefined4 *)(this + 0xd7c) = 0;
  *(undefined4 *)(this + 0xd80) = 0;
  this[0xd85] = (QualcommCameraHardware)0x0;
  this[0xd86] = (QualcommCameraHardware)0x0;
  pthread_mutex_init((pthread_mutex_t *)(this + 0xd88),(pthread_mutexattr_t *)0x0);
  pthread_cond_init((pthread_cond_t *)(this + 0xd8c),(pthread_condattr_t *)0x0);
  iVar6 = DAT_000280ec;
  iVar4 = DAT_000280e8 + 0x27e42;
  this[0xd90] = (QualcommCameraHardware)0x0;
  this[0xd91] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0xd94) = 0;
  __android_log_print(4,iVar4,iVar6 + 0x27e48);
  MMCameraDL::getInstance(aMStack_7c);
  sp<android::QualcommCameraHardware::MMCameraDL>::operator=
            ((sp<android::QualcommCameraHardware::MMCameraDL> *)(this + 0x318),(sp *)aMStack_7c);
  sp<android::QualcommCameraHardware::IonPool>::~sp
            ((sp<android::QualcommCameraHardware::IonPool> *)aMStack_7c);
  uVar1 = MMCameraDL::pointer(*(MMCameraDL **)(this + 0x318));
  puVar7 = *(undefined4 **)(iVar9 + DAT_000280f0);
  iVar6 = DAT_000280f8 + 0x27e7e;
  *(undefined1 *)(DAT_000280f4 + 0x27e7c) = 0;
  *puVar7 = uVar1;
  property_get(DAT_000280fc + 0x27e8a,acStack_78,iVar6);
  iVar6 = atoi(acStack_78);
  this[0xd92] = (QualcommCameraHardware)(iVar6 != 0);
  storeTargetType();
  pQVar5 = this + 0x6b8;
  iVar6 = 0;
  do {
    iVar10 = iVar10 + 1;
    *(undefined4 *)(pQVar5 + 0x98) = 0;
    *(undefined4 *)(pQVar5 + 0xac) = 0;
    pQVar5 = pQVar5 + 4;
    *(undefined4 *)pQVar5 = 0;
  } while (iVar10 != 5);
  pQVar5 = this + 0x788;
  iVar10 = 0;
  *(undefined4 *)(this + 0x778) = 0;
  *(undefined4 *)(this + 0x7b0) = 0;
  *(undefined4 *)(this + 0x77c) = 0;
  *(undefined4 *)(this + 0x7dc) = 0;
  do {
    iVar6 = iVar6 + 1;
    pQVar5 = pQVar5 + 4;
    *(undefined4 *)pQVar5 = 0;
    iVar4 = DAT_00028100;
  } while (iVar6 != 9);
  pQVar5 = this + 0x7b0;
  *(undefined4 *)(this + 0x780) = 0;
  *(undefined4 *)(this + 0x784) = 0;
  *(undefined4 *)(this + 0x788) = 0;
  for (; iVar6 = DAT_00028104, iVar10 < *(int *)(iVar4 + 0x27ef0); iVar10 = iVar10 + 1) {
    pQVar5 = pQVar5 + 4;
    *(undefined4 *)pQVar5 = 0;
  }
  *(undefined4 *)(this + 0x7d8) = 0;
  if (*(int *)(iVar6 + 0x27f0c) == 2) {
    this[0xcec] = (QualcommCameraHardware)0x1;
  }
  property_get(DAT_00028108 + 0x27f22,acStack_78,DAT_0002810c + 0x27f24);
  iVar10 = atoi(acStack_78);
  iVar6 = DAT_00028110;
  if (iVar10 == 1) {
    this[0xcec] = (QualcommCameraHardware)0x1;
    *(undefined4 *)(iVar6 + 0x27f3e) = 2;
  }
  iVar6 = pthread_create((pthread_t *)(this + 0x5e8),(pthread_attr_t *)0x0,
                         *(__start_routine **)(iVar9 + DAT_00028114),(void *)0x0);
  if (iVar6 != 0) {
    __android_log_print(6,DAT_00028118 + 0x27f60,DAT_0002811c + 0x27f62);
  }
  memset(this + 0x438,0,0x194);
  memset(this + 0x5f4,0,0x24);
  memset((void *)(DAT_00028120 + 0x27f8c),0,0x10);
  property_get(DAT_00028124 + 0x27f98,acStack_78,DAT_00028128 + 0x27f9a);
  iVar6 = atoi(acStack_78);
  piVar8 = (int *)(DAT_0002812c + 0x27fa8);
  *(int *)(this + 0xd08) = iVar6;
  iVar6 = DAT_00028130;
  if (*piVar8 - 5U < 2) {
    *(undefined4 *)(this + 0xd0c) = 4;
    puVar11 = (uint *)(iVar6 + 0x27fc0);
    uVar2 = 0x3a8;
    *puVar11 = 9;
  }
  else {
    *(undefined4 *)(this + 0xd0c) = 6;
    if (*piVar8 != 4) goto LAB_00027ff0;
    uVar2 = 0x340;
    puVar11 = (uint *)(DAT_00028134 + 0x27fde);
    *puVar11 = 8;
  }
  pvVar3 = operator_new__(uVar2);
  *(void **)(this + 0xcc0) = pvVar3;
  pvVar3 = operator_new__(*puVar11);
  *(void **)(this + 0xcc8) = pvVar3;
LAB_00027ff0:
  iVar6 = DAT_00028138;
  *(undefined4 *)(this + 0xd5c) = 6;
  iVar10 = DAT_00028140;
  if (*(int *)(iVar6 + 0x27ffc) - 2U < 5) {
                    /* WARNING: Could not recover jumptable at 0x00028002. Too many branches */
                    /* WARNING: Treating indirect jump as call */
    (*(code *)(&UNK_00028006 + (uint)*(byte *)(*(int *)(iVar6 + 0x27ffc) + 0x28004) * 2))();
    return;
  }
  puVar7 = (undefined4 *)(DAT_0002813c + 0x2801a);
  *(undefined4 *)(this + 0x14) = 0;
  *(undefined4 *)(this + 0x464) = 1;
  *puVar7 = 1;
  *(undefined4 *)(this + 0x468) = 1;
  if (*(int *)(iVar10 + 0x2802a) - 5U < 2) {
    *(undefined4 *)(this + 0x468) = 0;
  }
  *(undefined4 *)(this + 0x690) = 0x280;
  iVar6 = DAT_00028144;
  *(undefined4 *)(this + 0x470) = 1;
  *(undefined4 *)(this + 0x694) = 0x1e0;
  *(undefined4 *)(this + 0x46c) = 1;
  *(undefined4 *)(this + 0x69c) = 0;
  *(undefined4 *)(this + 0x6a0) = 0;
  *(undefined4 *)(this + 0x6a4) = 0;
  *(undefined4 *)(this + 0x6a8) = 0;
  *(undefined4 *)(this + 0x6ac) = 0;
  this[0x6b0] = (QualcommCameraHardware)0x0;
  *(undefined4 *)(this + 0x678) = 0;
  *(undefined4 *)(this + 0x668) = 0;
  this[0x6b8] = (QualcommCameraHardware)0x0;
  this[0x6b9] = (QualcommCameraHardware)0x0;
  iVar4 = DAT_00028150;
  iVar10 = DAT_0002814c;
  if (*(int *)(iVar6 + 0x28080) - 5U < 2) {
    puVar12 = (undefined1 *)(DAT_00028148 + 0x28090);
    this[0xd34] = (QualcommCameraHardware)0x0;
    *puVar12 = 1;
    __android_log_print(2,iVar10 + 0x28098,iVar4 + 0x2809e,this[0xd34],1);
  }
  if (this[0xcec] != (QualcommCameraHardware)0x0) {
    this[0xd34] = (QualcommCameraHardware)0x0;
  }
  __android_log_print(2,DAT_00028154 + 0x280be,DAT_00028158 + 0x280c0);
  if (local_1c != **(int **)(iVar9 + DAT_000280e0)) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail(this);
  }
  return;
}



// Function: getInstance @ 0002815c

/* android::QualcommCameraHardware::getInstance() */

QualcommCameraHardware * android::QualcommCameraHardware::getInstance(void)

{
  QualcommCameraHardware *this;
  
  this = (QualcommCameraHardware *)**(undefined4 **)(DAT_00028188 + 0x28164);
  if (this == (QualcommCameraHardware *)0x0) {
    __android_log_print(2,DAT_0002818c + 0x28172,DAT_00028190 + 0x28174);
    this = operator_new(0xd98);
    QualcommCameraHardware(this);
  }
  return this;
}



// Function: smoothzoom_thread @ 00028194

/* android::smoothzoom_thread(void*) */

undefined4 android::smoothzoom_thread(void *param_1)

{
  void *pvVar1;
  int iVar2;
  
  iVar2 = DAT_000281d4 + 0x281a0;
  __android_log_print(2,iVar2,DAT_000281d8 + 0x281a4);
  pvVar1 = (void *)QualcommCameraHardware::getInstance();
  if (pvVar1 == (void *)0x0) {
    __android_log_print(6,iVar2,DAT_000281dc + 0x281c0);
  }
  else {
    QualcommCameraHardware::runSmoothzoomThread(pvVar1);
  }
  __android_log_print(2,DAT_000281e0 + 0x281cc,DAT_000281e4 + 0x281ce);
  return 0;
}



// Function: snapshot_thread @ 000281e8

/* android::snapshot_thread(void*) */

undefined4 android::snapshot_thread(void *param_1)

{
  void *pvVar1;
  int iVar2;
  
  iVar2 = DAT_00028228 + 0x281f4;
  __android_log_print(3,iVar2,DAT_0002822c + 0x281f8);
  pvVar1 = (void *)QualcommCameraHardware::getInstance();
  if (pvVar1 == (void *)0x0) {
    __android_log_print(5,iVar2,DAT_00028230 + 0x28214);
  }
  else {
    QualcommCameraHardware::runSnapshotThread(pvVar1);
  }
  __android_log_print(3,DAT_00028234 + 0x28220,DAT_00028238 + 0x28222);
  return 0;
}



// Function: auto_focus_thread @ 0002823c

/* android::auto_focus_thread(void*) */

undefined4 android::auto_focus_thread(void *param_1)

{
  QualcommCameraHardware *this;
  int iVar1;
  
  iVar1 = DAT_00028278 + 0x28248;
  __android_log_print(2,iVar1,DAT_0002827c + 0x2824a);
  this = (QualcommCameraHardware *)QualcommCameraHardware::getInstance();
  if (this == (QualcommCameraHardware *)0x0) {
    __android_log_print(5,iVar1,DAT_00028280 + 0x28264);
  }
  else {
    QualcommCameraHardware::runAutoFocus(this);
  }
  __android_log_print(2,DAT_00028284 + 0x28270,DAT_00028288 + 0x28272);
  return 0;
}



// Function: frame_thread @ 0002828c

/* android::frame_thread(void*) */

undefined4 android::frame_thread(void *param_1)

{
  QualcommCameraHardware *this;
  int iVar1;
  
  iVar1 = DAT_000282cc + 0x28298;
  __android_log_print(3,iVar1,DAT_000282d0 + 0x2829c);
  this = (QualcommCameraHardware *)QualcommCameraHardware::getInstance();
  if (this == (QualcommCameraHardware *)0x0) {
    __android_log_print(5,iVar1,DAT_000282d4 + 0x282b8);
  }
  else {
    QualcommCameraHardware::runFrameThread(this,param_1);
  }
  __android_log_print(3,DAT_000282d8 + 0x282c4,DAT_000282dc + 0x282c6);
  return 0;
}



// Function: preview_thread @ 000282e0

/* android::preview_thread(void*) */

undefined4 android::preview_thread(void *param_1)

{
  void *pvVar1;
  int iVar2;
  
  iVar2 = DAT_00028320 + 0x282ec;
  __android_log_print(4,iVar2,DAT_00028324 + 0x282f0);
  pvVar1 = (void *)QualcommCameraHardware::getInstance();
  if (pvVar1 == (void *)0x0) {
    __android_log_print(6,iVar2,DAT_00028328 + 0x2830c);
  }
  else {
    QualcommCameraHardware::runPreviewThread(pvVar1);
  }
  __android_log_print(4,DAT_0002832c + 0x28318,DAT_00028330 + 0x2831a);
  return 0;
}



// Function: FUN_00028334 @ 00028334

void FUN_00028334(int param_1)

{
  uint uVar1;
  
  if (param_1 == 0) {
    uVar1 = android::QualcommCameraHardware::getInstance();
    if (uVar1 != 0) {
      android::QualcommCameraHardware::receiveLiveSnapshot(uVar1);
    }
  }
  else {
    __android_log_print(6,DAT_00028358 + 0x28352,DAT_0002835c + 0x28354);
  }
  return;
}



// Function: FUN_0002839c @ 0002839c

void FUN_0002839c(msm_frame *param_1)

{
  QualcommCameraHardware *this;
  
  this = (QualcommCameraHardware *)android::QualcommCameraHardware::getInstance();
  if (this != (QualcommCameraHardware *)0x0) {
    android::QualcommCameraHardware::receiveRecordingFrame(this,param_1);
  }
  return;
}



// Function: FUN_000283b0 @ 000283b0

void FUN_000283b0(int param_1)

{
  QualcommCameraHardware *this;
  
  this = (QualcommCameraHardware *)android::QualcommCameraHardware::getInstance();
  if ((this != (QualcommCameraHardware *)0x0) && (param_1 - 0x11U < 2)) {
    android::QualcommCameraHardware::receive_camframe_error_timeout(this);
  }
  return;
}



// Function: FUN_000284c0 @ 000284c0

void FUN_000284c0(undefined4 param_1,undefined4 param_2)

{
  QualcommCameraHardware *pQVar1;
  
  pQVar1 = (QualcommCameraHardware *)android::QualcommCameraHardware::getInstance();
  if (pQVar1 != (QualcommCameraHardware *)0x0) {
    android::QualcommCameraHardware::receiveCameraStats(pQVar1,param_1,param_2);
  }
  return;
}



// Function: FUN_000284d8 @ 000284d8

void FUN_000284d8(msm_frame *param_1)

{
  QualcommCameraHardware *this;
  
  this = (QualcommCameraHardware *)android::QualcommCameraHardware::getInstance();
  if (this != (QualcommCameraHardware *)0x0) {
    android::QualcommCameraHardware::receivePreviewFrame(this,param_1);
  }
  return;
}



// Function: openCamera @ 000284ec

/* android::openCamera(void*) */

undefined4 android::openCamera(void *param_1)

{
  undefined4 uVar1;
  int iVar2;
  undefined4 uVar3;
  int iVar4;
  int *piVar5;
  int iVar6;
  undefined4 *puVar7;
  int iVar8;
  undefined1 *puVar9;
  undefined4 *puVar10;
  undefined4 *puVar11;
  undefined4 local_38;
  undefined4 local_34;
  int local_30;
  undefined1 local_29 [5];
  
  iVar4 = DAT_0002867c + 0x284fa;
  iVar8 = DAT_00028684 + 0x28504;
  __android_log_print(2,iVar4,DAT_00028680 + 0x284fe);
  piVar5 = *(int **)(iVar8 + DAT_00028688);
  puVar9 = (undefined1 *)(DAT_0002868c + 0x28516);
  *puVar9 = 0;
  iVar6 = DAT_0002869c;
  iVar2 = DAT_00028698;
  if (*piVar5 == 0) {
    uVar1 = dlerror();
    __android_log_print(6,iVar4,DAT_00028690 + 0x28528,uVar1);
    return 0;
  }
  uVar1 = dlsym(*piVar5,DAT_00028694 + 0x2853a);
  puVar11 = *(undefined4 **)(iVar8 + iVar2);
  iVar2 = DAT_000286a0 + 0x28548;
  *puVar11 = uVar1;
  uVar1 = dlsym(*piVar5,iVar2);
  puVar10 = *(undefined4 **)(iVar8 + iVar6);
  iVar2 = DAT_000286a4 + 0x2855a;
  *puVar10 = uVar1;
  uVar1 = dlsym(*piVar5,iVar2);
  iVar2 = DAT_000286b0;
  puVar7 = *(undefined4 **)(iVar8 + DAT_000286a8);
  iVar6 = DAT_000286b0 + 0x28570;
  uVar3 = *(undefined4 *)(iVar8 + DAT_000286ac);
  *puVar7 = uVar1;
  iVar6 = (*(code *)*puVar11)(iVar6,uVar3,*(undefined4 *)(iVar8 + DAT_000286b4),0);
  if (iVar6 == 0) {
    local_29[0] = (undefined1)*(undefined4 *)(DAT_000286c0 + 0x2859e);
    iVar6 = (**(code **)(iVar2 + 0x28574))(0x21,local_29);
    if (iVar6 == 0) {
      local_30 = 1;
      iVar6 = (**(code **)(iVar2 + 0x28574))(0x1f,&local_30);
      if (iVar6 != 0) {
        __android_log_print(6,iVar4,DAT_000286cc + 0x285de);
        (*(code *)*puVar7)();
        puVar9 = (undefined1 *)(DAT_000286d0 + 0x285ea);
        goto LAB_000285e8;
      }
      iVar6 = (*(code *)*puVar10)();
      if (iVar6 == 0) {
        *puVar9 = 1;
        if (local_30 == 2) {
          local_38 = 1;
          iVar2 = (**(code **)(iVar2 + 0x28578))(0x20,&local_38);
          if (iVar2 != 0) {
            __android_log_print(6,iVar4,DAT_000286dc + 0x28632,DAT_000286e0 + 0x28634);
            (*(code *)*puVar7)();
            return 0;
          }
          iVar8 = QualcommCameraHardware::getInstance();
          iVar6 = DAT_000286e8;
          iVar2 = DAT_000286e4;
          if (iVar8 != 0) {
            *(undefined4 *)(iVar8 + 0xd70) = local_34;
            __android_log_print(4,iVar4,iVar2 + 0x28650,iVar6 + 0x28654,local_34);
          }
        }
        iVar2 = DAT_000286f0 + 0x2866a;
        *(undefined1 *)(DAT_000286ec + 0x28668) = 1;
        __android_log_print(2,iVar2,DAT_000286f4 + 0x28672);
        return 0;
      }
      __android_log_print(6,iVar4,DAT_000286d4 + 0x28600);
      (*(code *)*puVar7)();
      puVar9 = (undefined1 *)(DAT_000286d8 + 0x2860c);
    }
    else {
      __android_log_print(6,iVar4,DAT_000286c4 + 0x285b4);
      (*(code *)*puVar7)();
      puVar9 = (undefined1 *)(DAT_000286c8 + 0x285c0);
    }
    *puVar9 = 0;
  }
  else {
    __android_log_print(6,iVar4,DAT_000286b8 + 0x2858c);
    puVar9 = (undefined1 *)(DAT_000286bc + 0x28594);
LAB_000285e8:
    *puVar9 = 0;
  }
  return 0;
}



// Function: _INIT_1 @ 000286f8

void _INIT_1(void)

{
  int iVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  undefined4 *puVar5;
  undefined4 uVar6;
  undefined4 uVar7;
  int iVar8;
  String8 *pSVar9;
  String8 *pSVar10;
  pthread_mutex_t *ppVar11;
  
  iVar8 = DAT_00028a48;
  iVar1 = DAT_00028a40;
  pSVar9 = (String8 *)(DAT_00028a44 + 0x28710);
  *(undefined4 *)(DAT_00028a40 + 0x2870a) = 5000;
  *(undefined4 *)(iVar1 + 0x2870e) = 31000;
  iVar8 = iVar8 + 0x2871e;
  android::String8::String8(pSVar9);
  uVar7 = *(undefined4 *)(iVar8 + DAT_00028a4c);
  uVar6 = *(undefined4 *)(iVar8 + DAT_00028a50);
  pSVar10 = (String8 *)(DAT_00028a54 + 0x28730);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028a58 + 0x28748);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028a5c + 0x2875c);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028a60 + 0x28770);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028a64 + 0x28784);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028a68 + 0x28798);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028a6c + 0x287ac);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028a70 + 0x287c0);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028a74 + 0x287d4);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028a78 + 0x287e8);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028a7c + 0x287fc);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028a80 + 0x28810);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028a84 + 0x28824);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028a88 + 0x28838);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028a8c + 0x2884c);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028a90 + 0x28860);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028a94 + 0x28874);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028a98 + 0x28888);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028a9c + 0x2889c);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028aa0 + 0x288b0);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028aa4 + 0x288c4);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028aa8 + 0x288d8);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028aac + 0x288ec);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028ab0 + 0x28900);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028ab4 + 0x28914);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028ab8 + 0x28928);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028abc + 0x2893c);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028ac0 + 0x28950);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028ac4 + 0x28964);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028ac8 + 0x28978);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  pSVar10 = (String8 *)(DAT_00028acc + 0x2898c);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  android::String8::String8(pSVar10);
  pSVar9 = (String8 *)(DAT_00028ad0 + 0x289a0);
  __aeabi_atexit(pSVar10,uVar7,uVar6);
  android::String8::String8(pSVar9);
  iVar1 = DAT_00028ad8;
  ppVar11 = (pthread_mutex_t *)(DAT_00028ad4 + 0x289b6);
  __aeabi_atexit(pSVar9,uVar7,uVar6);
  pthread_mutex_init(ppVar11,(pthread_mutexattr_t *)0x0);
  __aeabi_atexit(ppVar11,iVar1 + 0x289bc,uVar6);
  iVar4 = DAT_00028ae4;
  iVar3 = DAT_00028ae0;
  iVar2 = DAT_00028adc;
  uVar7 = DAT_00028a34;
  *(undefined4 *)(DAT_00028adc + 0x289d8) = DAT_00028a30;
  *(undefined4 *)(iVar2 + 0x289dc) = uVar7;
  uVar7 = DAT_00028a3c;
  *(undefined4 *)(iVar3 + 0x289e4) = DAT_00028a38;
  *(undefined4 *)(iVar3 + 0x289e8) = uVar7;
  pthread_cond_init((pthread_cond_t *)(iVar4 + 0x289ec),(pthread_condattr_t *)0x0);
  __aeabi_atexit((pthread_cond_t *)(iVar4 + 0x289ec),DAT_00028ae8 + 0x28a00,uVar6);
  puVar5 = *(undefined4 **)(iVar8 + DAT_00028aec);
  uVar7 = *(undefined4 *)(iVar8 + DAT_00028af0);
  *puVar5 = 0;
  __aeabi_atexit(puVar5,uVar7,uVar6);
  ppVar11 = *(pthread_mutex_t **)(iVar8 + DAT_00028af4);
  pthread_mutex_init(ppVar11,(pthread_mutexattr_t *)0x0);
  __aeabi_atexit(ppVar11,iVar1 + 0x289bc,uVar6);
  return;
}



// Function: FUN_00028af8 @ 00028af8

void FUN_00028af8(void)

{
  int iVar1;
  undefined4 *puVar2;
  undefined4 uVar3;
  
  iVar1 = DAT_00028b20;
  puVar2 = *(undefined4 **)(DAT_00028b20 + 0x28b02);
  if (puVar2 != (undefined4 *)0x0) {
    if (puVar2 == *(undefined4 **)(DAT_00028b20 + 0x28b06)) {
      uVar3 = 0;
      *(undefined4 *)(DAT_00028b20 + 0x28b06) = 0;
    }
    else {
      uVar3 = *puVar2;
    }
    *(undefined4 *)(iVar1 + 0x28b02) = uVar3;
    iVar1 = DAT_00028b24;
    *puVar2 = 0;
    *(int *)(iVar1 + 0x28b1a) = *(int *)(iVar1 + 0x28b1a) + -1;
  }
  return;
}



// Function: cam_frame_flush_video @ 00028b28

/* android::cam_frame_flush_video() */

void android::cam_frame_flush_video(void)

{
  int iVar1;
  int iVar2;
  int iVar3;
  void *__ptr;
  undefined4 *puVar4;
  pthread_mutex_t *__mutex;
  
  iVar2 = DAT_00028b94;
  iVar1 = DAT_00028b88;
  puVar4 = (undefined4 *)(DAT_00028b88 + 0x28b34);
  __mutex = (pthread_mutex_t *)(DAT_00028b88 + 0x28b40);
  __android_log_print(2,DAT_00028b8c + 0x28b3a,DAT_00028b90 + 0x28b3c,*puVar4);
  iVar3 = DAT_00028b98;
  pthread_mutex_lock(__mutex);
  while (*(int *)(iVar1 + 0x28b38) != 0) {
    __ptr = (void *)FUN_00028af8();
    if (__ptr != (void *)0x0) {
      free(__ptr);
    }
    __android_log_print(2,iVar2 + 0x28b48,iVar3 + 0x28b52);
  }
  pthread_mutex_unlock((pthread_mutex_t *)(iVar1 + 0x28b40));
  __android_log_print(2,DAT_00028b9c + 0x28b7e,DAT_00028ba0 + 0x28b82,*puVar4);
  return;
}



// Function: initRecord @ 00028ba4

/* android::QualcommCameraHardware::initRecord() */

undefined4 __thiscall android::QualcommCameraHardware::initRecord(QualcommCameraHardware *this)

{
  bool bVar1;
  QualcommCameraHardware *pQVar2;
  int iVar3;
  int iVar4;
  char *pcVar5;
  QualcommCameraHardware *pQVar6;
  int iVar7;
  undefined4 *puVar8;
  int iVar9;
  int iVar10;
  int iVar11;
  uint uVar12;
  int iVar13;
  int iVar14;
  QualcommCameraHardware *pQVar15;
  pthread_mutex_t *__mutex;
  int *piVar16;
  int iVar17;
  int iVar18;
  int iVar19;
  uint uVar20;
  undefined4 uVar21;
  
  iVar19 = DAT_00028fe0;
  iVar11 = DAT_00028fd8 + 0x28bb6;
  __android_log_print(2,iVar11,DAT_00028fdc + 0x28bba);
  iVar19 = iVar19 + 0x28bcc;
  if (*(int *)(this + 0xd64) == 0) {
    if (*(int *)(DAT_00028fe8 + 0x28be0) == 6) {
      iVar11 = DAT_00028ff0 + 0x28bf4;
    }
    else {
      iVar11 = DAT_00028fec + 0x28bec;
    }
    uVar12 = (uint)*(ushort *)(this + 0x43a);
    __android_log_print(4,DAT_00028ff4 + 0x28c00,DAT_00028ff8 + 0x28c06,
                        *(undefined2 *)(this + 0x438),uVar12);
    if (*(int *)(DAT_00028ffc + 0x28c14) == 6) {
      iVar4 = (uint)*(ushort *)(this + 0x438) * (uint)*(ushort *)(this + 0x43a);
      uVar20 = iVar4 + 0x7ffU & 0xfffff800;
      iVar4 = ((iVar4 >> 1) + 0x7ffU & 0xfffff800) + uVar20;
    }
    else {
      iVar4 = (uint)*(ushort *)(this + 0x438) * (uint)*(ushort *)(this + 0x43a);
      uVar20 = iVar4 + 3U & 0xfffffffc;
      iVar4 = iVar4 * 3 >> 1;
    }
    *(uint *)(this + 0x424) = iVar4 + 0xfffU & 0xfffff000;
    pcVar5 = (char *)android::CameraParameters::get((char *)(this + 0x18));
    if ((pcVar5 == (char *)0x0) ||
       (iVar4 = strcmp(pcVar5,*(char **)(iVar19 + DAT_00029004)), iVar4 == 0)) {
      bVar1 = false;
    }
    else {
      __android_log_print(4,DAT_00029008 + 0x28c96,DAT_0002900c + 0x28c98,DAT_00029010 + 0x28c9a,
                          uVar12);
      bVar1 = true;
    }
    iVar4 = DAT_00029018;
    if ((((*(char *)(DAT_00029014 + 0x28caa) != '\0') &&
         (this[0xd34] != (QualcommCameraHardware)0x0)) && (!bVar1)) ||
       (this[0xcec] != (QualcommCameraHardware)0x0)) {
      iVar9 = *(int *)(this + 0xd30) * *(int *)(this + 0xd2c);
      *(int *)(this + 0x424) = (iVar9 * 3) / 2;
      if (*(int *)(iVar4 + 0x28cde) == 6) {
        *(uint *)(this + 0x424) =
             (iVar9 + 0x7ffU & 0xfffff800) + 0xfff + (iVar9 / 2 + 0x7ffU & 0xfffff800) & 0xfffff000;
      }
    }
    iVar4 = DAT_0002901c + 0x28d2a;
    piVar16 = (int *)(DAT_00029020 + 0x28d2c);
    __android_log_print(2,iVar4,DAT_00029024 + 0x28d30,*(undefined4 *)(this + 0x424),uVar12);
    iVar13 = *piVar16;
    iVar18 = 0;
    __android_log_print(2,iVar4,DAT_00029028 + 0x28d44,DAT_0002902c + 0x28d48,iVar13);
    iVar9 = DAT_00029030 + 0x28d5a;
    iVar10 = DAT_00029034 + 0x28d5e;
    iVar4 = DAT_00029038 + 0x28d60;
    pcVar5 = (char *)(DAT_0002903c + 0x28d64);
    pQVar15 = this;
    for (iVar14 = 0; iVar3 = DAT_00029070, iVar17 = DAT_0002906c, iVar7 = DAT_00029068,
        iVar14 < *piVar16; iVar14 = iVar14 + 1) {
      iVar13 = *(int *)(this + 0x424);
      pQVar6 = this + (iVar14 + 0x1c4) * 4 + 4;
      iVar7 = allocate_ion_memory(this,(int *)(this + (iVar14 + 0x1fe) * 4 + 4),
                                  (ion_allocation_data *)(this + (iVar14 + 0x8e) * 0x10),
                                  (ion_fd_data *)(this + (iVar14 + 0x135) * 8),8,iVar13,
                                  (int *)pQVar6);
      if (iVar7 < 0) {
        __android_log_print(6,DAT_00029040 + 0x28db4,DAT_00029044 + 0x28db6,iVar11);
        return 0;
      }
      puVar8 = (undefined4 *)
               (**(code **)(this + 0xd00))
                         (*(undefined4 *)(pQVar15 + 0x714),*(undefined4 *)(this + 0x424),1,
                          *(undefined4 *)(this + 0xd04));
      *(undefined4 **)(pQVar15 + 0x78c) = puVar8;
      if (puVar8 == (undefined4 *)0x0) {
        __android_log_print(6,iVar4,DAT_00029048 + 0x28de4,iVar14);
      }
      else {
        iVar13 = puVar8[1];
        pQVar6 = (QualcommCameraHardware *)puVar8[3];
        __android_log_print(6,iVar9,iVar10,*puVar8,puVar8[2],iVar13,pQVar6);
      }
      *(undefined4 *)(*(int *)(this + 0xcc0) + iVar18 + 0x10) = **(undefined4 **)(pQVar15 + 0x78c);
      *(undefined4 *)(*(int *)(this + 0xcc0) + iVar18 + 0x2c) = *(undefined4 *)(pQVar15 + 0x714);
      *(undefined4 *)(*(int *)(this + 0xcc0) + iVar18 + 0x20) = 0;
      *(uint *)(*(int *)(this + 0xcc0) + iVar18 + 0x24) = uVar20;
      bVar1 = iVar14 < 3;
      *(undefined4 *)(*(int *)(this + 0xcc0) + iVar18 + 0x28) = 0;
      *(undefined4 *)(*(int *)(this + 0xcc0) + iVar18 + 8) = 8;
      *(undefined1 *)(*(int *)(this + 0xcc8) + iVar14) = 0;
      if ((*pcVar5 == '\0') || (iVar14 != *(int *)(DAT_0002904c + 0x28e70) + -1)) {
        uVar21 = 0xe;
      }
      else {
        uVar21 = 0x10;
        bVar1 = true;
      }
      pQVar2 = pQVar15 + 0x714;
      pQVar15 = pQVar15 + 4;
      iVar17 = DAT_00029050 + 0x28e96;
      __android_log_print(6,iVar17,DAT_00029054 + 0x28e9a,iVar14,*(undefined4 *)pQVar2,iVar13,pQVar6
                         );
      iVar7 = *(int *)(this + 0xcc0) + iVar18;
      iVar18 = iVar18 + 0x68;
      iVar13 = *(int *)(iVar7 + 0x2c);
      FUN_0001f76c(*(undefined4 *)(this + 0x424),*(undefined4 *)(this + 0x424),uVar20,0,iVar13,0,
                   *(undefined4 *)(iVar7 + 0x10),uVar21,bVar1,1,0);
      __android_log_print(6,iVar17,DAT_00029058 + 0x28ee4);
    }
    __mutex = (pthread_mutex_t *)(this + 0x368);
    __android_log_print(2,DAT_0002905c + 0x28f00,DAT_00029060 + 0x28f02,DAT_00029064 + 0x28f06,
                        iVar13);
    cam_frame_flush_video();
    pthread_mutex_lock(__mutex);
    while (iVar11 = DAT_00029074, this[0x365] != (QualcommCameraHardware)0x0) {
      __android_log_print(2,iVar7 + 0x28f20,iVar17 + 0x28f22);
      pthread_cond_wait((pthread_cond_t *)(this + 0x36c),__mutex);
      __android_log_print(2,iVar7 + 0x28f20,iVar3 + 0x28f24);
    }
    pthread_mutex_unlock(__mutex);
    (*(code *)**(undefined4 **)(iVar19 + DAT_00029078))(0);
    iVar4 = DAT_00029080;
    if (*(char *)(iVar11 + 0x28f5c) == '\0') {
      iVar9 = 0x138;
      piVar16 = (int *)(DAT_0002907c + 0x28f7a);
      for (iVar11 = 3; iVar11 < *piVar16; iVar11 = iVar11 + 1) {
        iVar10 = *(int *)(this + 0xcc0) + iVar9;
        iVar9 = iVar9 + 0x68;
        (*(code *)**(undefined4 **)(iVar19 + iVar4))(0,iVar10);
      }
    }
    else {
      iVar9 = 0x138;
      piVar16 = (int *)(DAT_00029084 + 0x28f8a);
      for (iVar11 = 3; iVar11 < *piVar16 + -1; iVar11 = iVar11 + 1) {
        iVar10 = *(int *)(this + 0xcc0) + iVar9;
        iVar9 = iVar9 + 0x68;
        (*(code *)**(undefined4 **)(iVar19 + iVar4))(0,iVar10);
      }
    }
    iVar11 = DAT_00029088 + 0x28fcc;
    iVar19 = DAT_0002908c + 0x28fce;
  }
  else {
    iVar19 = DAT_00028fe4 + 0x28bd8;
  }
  __android_log_print(2,iVar11,iVar19);
  return 1;
}



// Function: initPreview @ 00029090

/* android::QualcommCameraHardware::initPreview() */

int __thiscall android::QualcommCameraHardware::initPreview(QualcommCameraHardware *this)

{
  int iVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  int iVar7;
  int iVar8;
  int iVar9;
  uint *puVar10;
  uint uVar11;
  undefined4 *puVar12;
  CameraParameters *pCVar13;
  pthread_attr_t *__attr;
  int iVar14;
  pthread_mutex_t *__mutex;
  pthread_mutex_t *__mutex_00;
  pthread_attr_t pStack_58;
  
  iVar8 = DAT_000295d4;
  pCVar13 = (CameraParameters *)(this + 0x18);
  android::CameraParameters::getPreviewSize((int *)pCVar13,(int *)(this + 0xd10));
  iVar8 = iVar8 + 0x290ba;
  iVar9 = DAT_000295dc + 0x290c2;
  iVar1 = android::CameraParameters::get((char *)pCVar13);
  uVar3 = *(undefined4 *)(this + 0xd14);
  uVar2 = *(undefined4 *)(this + 0xd10);
  __android_log_print(6,iVar9,DAT_000295e0 + 0x290d6,DAT_000295e4 + 0x290d8,uVar2,uVar3);
  if (iVar1 == 0) {
    __android_log_print(2,iVar9,DAT_000295e8 + 0x290f8);
    *(undefined4 *)(this + 0xd2c) = *(undefined4 *)(this + 0xd10);
    *(undefined4 *)(this + 0xd30) = *(undefined4 *)(this + 0xd14);
  }
  else {
    iVar14 = FUN_00022b8c(iVar1,this + 0xd2c,this + 0xd30);
    if (iVar14 != 0) {
      __android_log_print(6,iVar9,DAT_000295f8 + 0x291a2,iVar1,uVar2,uVar3);
      return 0;
    }
    if ((*(int *)(this + 0xd2c) < *(int *)(this + 0xd10)) ||
       (*(int *)(this + 0xd30) < *(int *)(this + 0xd14))) {
      __android_log_print(4,DAT_000295ec + 0x2914e,DAT_000295f0 + 0x29152,*(int *)(this + 0xd10),
                          *(undefined4 *)(this + 0xd14),*(int *)(this + 0xd2c),
                          *(undefined4 *)(this + 0xd30));
      *(int *)(this + 0xd10) = *(int *)(this + 0xd2c);
      *(undefined4 *)(this + 0xd14) = *(undefined4 *)(this + 0xd30);
      android::CameraParameters::setPreviewSize((int)pCVar13,*(int *)(this + 0xd2c));
    }
    if (2 < *(int *)(DAT_000295f4 + 0x29176) - 4U) {
      *(int *)(this + 0xd10) = *(int *)(this + 0xd2c);
      *(undefined4 *)(this + 0xd14) = *(undefined4 *)(this + 0xd30);
      android::CameraParameters::setPreviewSize((int)pCVar13,*(int *)(this + 0xd2c));
    }
  }
  *(short *)(this + 0x442) = (short)*(undefined4 *)(this + 0xd14);
  iVar9 = DAT_000295fc + 0x291d2;
  iVar1 = DAT_00029600 + 0x291d8;
  *(short *)(this + 0x440) = (short)*(undefined4 *)(this + 0xd10);
  *(undefined2 *)(this + 0x44e) = 0x200;
  *(undefined2 *)(this + 0x44c) = 0x180;
  __android_log_print(2,iVar9,iVar1,*(undefined4 *)(this + 0xd10),*(undefined4 *)(this + 0xd14),
                      *(undefined4 *)(this + 0xd2c),*(undefined4 *)(this + 0xd30));
  iVar1 = DAT_00029608;
  if (*(int *)(DAT_00029604 + 0x291f2) - 4U < 3) {
    uVar5 = *(int *)(this + 0xd2c) + 0xfU & 0xfff0;
    *(short *)(this + 0x438) = (short)uVar5;
    *(uint *)(this + 0xd2c) = uVar5;
    *(undefined2 *)(this + 0x43a) = *(undefined2 *)(this + 0xd30);
    __android_log_print(2,iVar9,iVar1 + 0x2921c,*(undefined4 *)(this + 0xd10),
                        *(undefined4 *)(this + 0xd14),uVar5,*(undefined2 *)(this + 0xd30));
  }
  iVar14 = DAT_00029614;
  iVar9 = DAT_00029610;
  iVar1 = DAT_0002960c;
  __mutex_00 = (pthread_mutex_t *)(this + 0x35c);
  pthread_mutex_lock(__mutex_00);
  while (this[0x358] != (QualcommCameraHardware)0x0) {
    __android_log_print(4,iVar1 + 0x2924c,iVar9 + 0x2924e);
    pthread_cond_wait((pthread_cond_t *)(this + 0x360),__mutex_00);
    __android_log_print(4,iVar1 + 0x2924c,iVar14 + 0x29250);
  }
  __mutex = (pthread_mutex_t *)(this + 0x3cc);
  pthread_mutex_unlock(__mutex_00);
  pthread_mutex_lock(__mutex);
  iVar14 = DAT_00029618 + 0x29296;
  iVar9 = DAT_0002961c + 0x2929a;
  iVar1 = DAT_00029620 + 0x2929c;
  while (iVar7 = DAT_00029624, this[0x3c8] != (QualcommCameraHardware)0x0) {
    __android_log_print(4,iVar1,iVar14);
    pthread_cond_wait((pthread_cond_t *)(this + 0x3d0),__mutex);
    __android_log_print(4,iVar1,iVar9);
  }
  pthread_mutex_unlock(__mutex);
  puVar10 = (uint *)(iVar7 + 0x292d0);
  *puVar10 = 0;
  *(undefined4 *)(iVar7 + 0x292d4) = 0;
  iVar1 = *(int *)(this + 0xd14);
  iVar9 = DAT_00029628 + 0x292f0;
  *(int *)(this + 0x420) = (iVar1 * *(int *)(this + 0xd10) * 3) / 2;
  __android_log_print(6,iVar9,DAT_0002962c + 0x29300,*(int *)(this + 0xd10),iVar1);
  if (*(int *)(DAT_00029630 + 0x29308) == 4) {
    uVar4 = *(int *)(this + 0xd10) * *(int *)(this + 0xd14) + 3U & 0xfffffffc;
    *puVar10 = uVar4;
    uVar6 = *(int *)(this + 0xd10) * *(int *)(this + 0xd14);
    uVar5 = uVar6 & ~((int)uVar6 >> 0x20);
    if ((int)uVar6 < 0) {
      uVar5 = uVar6 + 3;
    }
    *(uint *)(iVar7 + 0x292d4) = uVar4 + (((int)uVar5 >> 2) + 3U & 0xfffffffc);
    *(undefined4 *)(this + 0x464) = 4;
    iVar1 = *(int *)(iVar7 + 0x292d4);
    __android_log_print(6,iVar9,DAT_00029634 + 0x29352,*puVar10,iVar1);
  }
  iVar9 = DAT_00029638;
  iVar7 = *(int *)(this + 0xd10);
  iVar14 = *(int *)(this + 0xd14);
  *(short *)(this + 0x484) = (short)iVar7;
  *(short *)(this + 0x486) = (short)iVar14;
  *(short *)(this + 0x488) = (short)iVar7;
  *(short *)(this + 0x48a) = (short)iVar14;
  if (*(int *)(iVar9 + 0x29378) == 2) {
    uVar4 = iVar14 + 0x1fU & 0xffffffe0;
    uVar6 = iVar7 + 0x1fU & 0xffffffe0;
    uVar11 = iVar7 / 2 + 0x1fU & 0xffffffe0;
    uVar5 = iVar14 / 2 + 0x1fU & 0xffffffe0;
    *(undefined4 *)(this + 0x464) = 2;
    *(short *)(this + 0x484) = (short)uVar6;
    *(short *)(this + 0x486) = (short)uVar4;
    *(short *)(this + 0x488) = (short)(uVar11 << 1);
    *(short *)(this + 0x48a) = (short)uVar5;
    *(uint *)(this + 0x420) = (uVar6 * uVar4 + 0xfff & 0xfffff000) + uVar5 * uVar11 * 2;
  }
  iVar9 = DAT_0002963c + 0x293e2;
  __android_log_print(2,iVar9,DAT_00029640 + 0x293ea,*(undefined4 *)(this + 0x464),iVar1);
  __android_log_print(2,iVar9,DAT_00029644 + 0x293f8,*(undefined2 *)(this + 0x484));
  __android_log_print(2,iVar9,DAT_00029648 + 0x29408,*(undefined2 *)(this + 0x486));
  __android_log_print(2,iVar9,DAT_0002964c + 0x29418,*(undefined2 *)(this + 0x488));
  puVar12 = (undefined4 *)(DAT_00029650 + 0x29428);
  __android_log_print(2,iVar9,DAT_00029654 + 0x2942e,*(undefined2 *)(this + 0x48a));
  *puVar12 = 0;
  *(undefined2 *)(this + 0x444) = *(undefined2 *)(this + 0xd2c);
  *(undefined2 *)(this + 0x446) = *(undefined2 *)(this + 0xd30);
  if (*(int *)(this + 0xd64) != 0) {
    *(undefined2 *)(this + 0x44e) = *(undefined2 *)(this + 0x440);
    *(undefined2 *)(this + 0x44c) = *(undefined2 *)(this + 0x442);
    android::CameraParameters::getPictureSize((int *)pCVar13,(int *)(this + 0xd4c));
    iVar1 = updatePictureDimension(this,pCVar13,(int *)(this + 0xd4c),(int *)(this + 0xd50));
    if (iVar1 != 0) {
      *(undefined2 *)(this + 0x43c) = *(undefined2 *)(this + 0xd4c);
      *(undefined2 *)(this + 0x43e) = *(undefined2 *)(this + 0xd50);
    }
  }
  iVar9 = DAT_00029658 + 0x29498;
  __android_log_print(2,iVar9,DAT_0002965c + 0x2949a);
  iVar1 = native_set_parms(this,3,0x194,this + 0x438);
  if (iVar1 == 0) {
    iVar8 = DAT_00029660 + 0x294b8;
  }
  else {
    if ((2 < *(int *)(DAT_00029664 + 0x294be) - 4U) || (iVar1 = initRecord(this), iVar1 != 0)) {
      iVar1 = DAT_0002966c;
      if (this[0xcec] == (QualcommCameraHardware)0x0) {
        FrameQueue::init((FrameQueue *)(this + 0x330));
        (*(code *)**(undefined4 **)(iVar8 + DAT_00029670))(2);
        __attr = (pthread_attr_t *)((int)&pStack_58 + 0x18);
        (*(code *)**(undefined4 **)(iVar8 + iVar1))(2,this + 0xb28);
        pthread_mutex_lock((pthread_mutex_t *)(this + 800));
        pthread_attr_init(__attr);
        pthread_attr_setdetachstate(__attr,1);
        uVar5 = pthread_create((pthread_t *)(this + 0x5e0),__attr,
                               *(__start_routine **)(iVar8 + DAT_00029674),(void *)0x0);
        iVar1 = 1 - uVar5;
        if (1 < uVar5) {
          iVar1 = 0;
        }
        this[0x31c] = SUB41(iVar1,0);
        pthread_mutex_unlock((pthread_mutex_t *)(this + 800));
        if (iVar1 == 0) {
          return 0;
        }
      }
      pthread_mutex_lock(__mutex_00);
      pthread_attr_init(&pStack_58);
      pthread_attr_setdetachstate(&pStack_58,1);
      iVar1 = DAT_00029678;
      *(undefined4 *)(DAT_00029678 + 0x29572) = 1;
      if (this[0xcec] != (QualcommCameraHardware)0x0) {
        *(undefined4 *)(iVar1 + 0x29572) = 2;
      }
      iVar1 = DAT_00029680;
      (*(code *)**(undefined4 **)(iVar8 + DAT_0002967c))(0);
      uVar5 = pthread_create((pthread_t *)(this + 0x5d8),&pStack_58,
                             *(__start_routine **)(iVar8 + iVar1),(void *)(DAT_00029684 + 0x29592));
      iVar1 = 1 - uVar5;
      if (1 < uVar5) {
        iVar1 = 0;
      }
      this[0x358] = SUB41(iVar1,0);
      pthread_mutex_unlock(__mutex_00);
      (*(code *)**(undefined4 **)(iVar8 + DAT_00029688))();
      iVar8 = DAT_00029690;
      iVar9 = DAT_0002968c + 0x295c0;
      this[1000] = (QualcommCameraHardware)0x1;
      __android_log_print(2,iVar9,iVar8 + 0x295c6,iVar1);
      return iVar1;
    }
    iVar8 = DAT_00029668 + 0x294d4;
  }
  __android_log_print(6,iVar9,iVar8);
  return 0;
}



// Function: FUN_00029694 @ 00029694

int FUN_00029694(void)

{
  void *__ptr;
  int iVar1;
  
  __android_log_print(2,DAT_000296d0 + 0x296a0,DAT_000296d4 + 0x296a2);
  iVar1 = 0;
  if (*(int *)(DAT_000296d8 + 0x296ae) != 0) {
    __ptr = (void *)FUN_00028af8();
    iVar1 = 0;
    if (__ptr != (void *)0x0) {
      iVar1 = *(int *)((int)__ptr + 4);
      free(__ptr);
    }
    __android_log_print(2,DAT_000296dc + 0x296c4,DAT_000296e0 + 0x296c8,
                        *(undefined4 *)(iVar1 + 0x10));
  }
  return iVar1;
}



// Function: startRecordingInternal @ 000296e4

/* android::QualcommCameraHardware::startRecordingInternal() */

undefined4 __thiscall
android::QualcommCameraHardware::startRecordingInternal(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  int iVar3;
  undefined4 uVar4;
  int iVar5;
  int iVar6;
  int *piVar7;
  int iVar8;
  int iVar9;
  
  iVar5 = DAT_0002988c;
  iVar6 = DAT_00029880 + 0x296f4;
  __android_log_print(4,iVar6,DAT_00029884 + 0x296f8,DAT_00029888 + 0x296fa);
  iVar5 = iVar5 + 0x2970a;
  this[0x3fd] = (QualcommCameraHardware)0x0;
  if (this[0x365] == (QualcommCameraHardware)0x0) {
    if (*(char *)(DAT_00029894 + 0x29726) != '\0') {
      __android_log_print(4,iVar6,DAT_00029898 + 0x29732);
      iVar3 = setVpeParameters();
      if (iVar3 != 0) {
        __android_log_print(6,iVar6,DAT_0002989c + 0x29746);
        return 1;
      }
    }
    iVar6 = DAT_000298b0;
    if (*(int *)(DAT_000298a0 + 0x29752) - 4U < 3) {
      piVar7 = (int *)(DAT_000298a4 + 0x29766);
      __android_log_print(2,DAT_000298a8 + 0x29768,DAT_000298ac + 0x2976c,*piVar7);
      while (iVar2 = DAT_000298c4, iVar1 = DAT_000298c0, iVar3 = DAT_000298bc, 0 < *piVar7) {
        uVar4 = FUN_00029694();
        (*(code *)**(undefined4 **)(iVar5 + iVar6))(0,uVar4);
      }
      iVar8 = 0;
      __android_log_print(2,DAT_000298b4 + 0x29796,DAT_000298b8 + 0x2979c);
      iVar6 = DAT_000298b0;
      for (iVar9 = 0; iVar9 < *(int *)(iVar3 + 0x297a8); iVar9 = iVar9 + 1) {
        if (*(char *)(*(int *)(this + 0xcc8) + iVar9) != '\0') {
          __android_log_print(4,iVar1 + 0x297aa,iVar2 + 0x297ac,iVar9,
                              *(undefined4 *)(*(int *)(this + 0xcc0) + iVar8 + 0x10));
          (*(code *)**(undefined4 **)(iVar5 + iVar6))(0,*(int *)(this + 0xcc0) + iVar8);
          *(undefined1 *)(*(int *)(this + 0xcc8) + iVar9) = 0;
        }
        iVar8 = iVar8 + 0x68;
      }
      __android_log_print(6,DAT_000298c8 + 0x2980a,DAT_000298cc + 0x2980c);
      if (this[0xcec] == (QualcommCameraHardware)0x0) {
        FUN_0001ef80(10);
      }
      pthread_mutex_lock((pthread_mutex_t *)(this + 0x368));
      this[0x364] = (QualcommCameraHardware)0x0;
      pthread_attr_init((pthread_attr_t *)&stack0xffffffc0);
      pthread_attr_setdetachstate((pthread_attr_t *)&stack0xffffffc0,1);
      iVar5 = pthread_create((pthread_t *)(this + 0x5dc),(pthread_attr_t *)&stack0xffffffc0,
                             *(__start_routine **)(iVar5 + DAT_000298d0),(void *)0x0);
      this[0x365] = (QualcommCameraHardware)(iVar5 == 0);
      pthread_mutex_unlock((pthread_mutex_t *)(this + 0x368));
    }
    __android_log_print(2,DAT_000298d4 + 0x29870,DAT_000298d8 + 0x29872,DAT_000298dc + 0x29874);
  }
  else {
    __android_log_print(4,iVar6,DAT_00029890 + 0x2971a);
  }
  return 0;
}



// Function: runVideoThread @ 000298e0

/* android::QualcommCameraHardware::runVideoThread(void*) */

void android::QualcommCameraHardware::runVideoThread(void *param_1)

{
  int iVar1;
  int iVar2;
  pthread_mutex_t *__mutex;
  msm_frame *pmVar3;
  int iVar4;
  undefined4 uVar5;
  pthread_mutex_t *__mutex_00;
  int iVar6;
  int iVar7;
  undefined4 uVar8;
  pthread_mutex_t *__mutex_01;
  int iVar9;
  int iVar10;
  int iVar11;
  int *piVar12;
  code *pcVar13;
  undefined8 local_48;
  
  iVar1 = DAT_00029b3c;
  __android_log_print(6,DAT_00029b34 + 0x298f0,DAT_00029b38 + 0x298f2);
  iVar2 = DAT_00029b44;
  __mutex_01 = (pthread_mutex_t *)((int)param_1 + 0x368);
  piVar12 = (int *)(DAT_00029b44 + 0x29912);
  __mutex_00 = (pthread_mutex_t *)(DAT_00029b40 + 0x2991c);
  __mutex = (pthread_mutex_t *)(DAT_00029b44 + 0x2991e);
  iVar4 = DAT_00029b48 + 0x2991a;
  do {
    pthread_mutex_lock(__mutex);
    pthread_mutex_lock(__mutex_01);
    if (*(char *)((int)param_1 + 0x364) != '\0') {
      __android_log_print(2,DAT_00029b4c + 0x2993c,DAT_00029b50 + 0x2993e);
      pthread_mutex_unlock(__mutex_01);
LAB_000299b6:
      pthread_mutex_unlock(__mutex);
      if (*(int *)((int)param_1 + 0x674) != 0) {
        setAutoFocusStartStop(param_1,0);
        *(undefined4 *)((int)param_1 + 0x674) = 0;
      }
      pthread_mutex_lock(__mutex_01);
      *(undefined1 *)((int)param_1 + 0x365) = 0;
      pthread_cond_signal((pthread_cond_t *)((int)param_1 + 0x36c));
      pthread_mutex_unlock(__mutex_01);
      __android_log_print(2,DAT_00029b84 + 0x29b24,DAT_00029b88 + 0x29b26);
      return;
    }
    pthread_mutex_unlock(__mutex_01);
    __android_log_print(2,iVar1 + 0x2990c,iVar4);
    __android_log_print(2,iVar1 + 0x2990c,DAT_00029b54 + 0x29964);
    if (*piVar12 < 1) {
      pthread_cond_wait((pthread_cond_t *)(iVar2 + 0x29922),__mutex);
    }
    iVar9 = DAT_00029b58 + 0x29980;
    __android_log_print(2,iVar9,DAT_00029b5c + 0x29982);
    __android_log_print(2,iVar9,DAT_00029b60 + 0x29990);
    pthread_mutex_lock(__mutex_01);
    if (*(char *)((int)param_1 + 0x364) != '\0') {
      __android_log_print(2,iVar9,DAT_00029b64 + 0x299a8);
      pthread_mutex_unlock(__mutex_01);
      __mutex = (pthread_mutex_t *)(DAT_00029b68 + 0x299c2);
      goto LAB_000299b6;
    }
    pthread_mutex_unlock(__mutex_01);
    pmVar3 = (msm_frame *)FUN_00029694();
    pthread_mutex_unlock(__mutex_00);
    __android_log_print(6,iVar9,DAT_00029b6c + 0x299e2,pmVar3);
    if (pmVar3 == (msm_frame *)0x0) {
      __android_log_print(6,iVar9,DAT_00029b80 + 0x29aee);
    }
    else {
      iVar10 = *(int *)pmVar3;
      iVar9 = *(int *)(DAT_00029b70 + 0x299f8) + 1;
      iVar6 = *(int *)(pmVar3 + 4);
      *(int *)(DAT_00029b70 + 0x299f8) = iVar9;
      if (((0x1e < iVar9) && (*(int *)((int)param_1 + 0x674) == 0)) &&
         (0x2cf < *(int *)((int)param_1 + 0xd30))) {
        setAutoFocusStartStop(param_1,1);
        *(undefined4 *)((int)param_1 + 0x674) = 1;
      }
      iVar11 = DAT_00029b74 + 0x29a34;
      __android_log_print(2,iVar11,DAT_00029b78 + 0x29a36);
      pthread_mutex_lock((pthread_mutex_t *)((int)param_1 + 0x404));
      iVar7 = *(int *)((int)param_1 + 0xcf0);
      uVar5 = *(undefined4 *)((int)param_1 + 0xd04);
      pcVar13 = *(code **)((int)param_1 + 0xcfc);
      pthread_mutex_unlock((pthread_mutex_t *)((int)param_1 + 0x404));
      iVar9 = mapvideoBuffer(param_1,pmVar3);
      if (((*(char *)((int)param_1 + 0xcec) == '\0') &&
          (*(undefined1 *)(*(int *)((int)param_1 + 0xcc8) + iVar9) = 1, pcVar13 != (code *)0x0)) &&
         (iVar7 << 0x1a < 0)) {
        local_48 = (longlong)iVar6;
        local_48 = (longlong)DAT_00029b30 * (longlong)iVar10 + local_48;
        __android_log_print(2,iVar11,DAT_00029b7c + 0x29aa4,iVar9);
        local_48._4_4_ = (undefined4)((ulonglong)local_48 >> 0x20);
        if (*(int *)((int)param_1 + 0x3ec) == 0) {
          uVar8 = *(undefined4 *)((int)param_1 + iVar9 * 4 + 0x78c);
        }
        else {
          uVar8 = *(undefined4 *)((int)param_1 + iVar9 * 4 + 0x7b4);
        }
        (*pcVar13)((undefined4)local_48,local_48._4_4_,0x20,uVar8,0,uVar5);
      }
    }
  } while( true );
}



// Function: video_thread @ 00029b8c

/* android::video_thread(void*) */

undefined4 android::video_thread(void *param_1)

{
  void *pvVar1;
  int iVar2;
  
  iVar2 = DAT_00029bcc + 0x29b98;
  __android_log_print(2,iVar2,DAT_00029bd0 + 0x29b9c);
  pvVar1 = (void *)QualcommCameraHardware::getInstance();
  if (pvVar1 == (void *)0x0) {
    __android_log_print(6,iVar2,DAT_00029bd4 + 0x29bb8);
  }
  else {
    QualcommCameraHardware::runVideoThread(pvVar1);
  }
  __android_log_print(2,DAT_00029bd8 + 0x29bc4,DAT_00029bdc + 0x29bc6);
  return 0;
}



// Function: FUN_00029be0 @ 00029be0

bool FUN_00029be0(undefined4 param_1,undefined4 param_2)

{
  int iVar1;
  int *piVar2;
  char *pcVar3;
  code *pcVar4;
  
  pcVar4 = *(code **)(*(int *)(DAT_00029c1c + 0x29bee) + 8);
  iVar1 = (*pcVar4)(param_1,0,0,pcVar4,param_1,param_2);
  if (iVar1 != 0) {
    piVar2 = (int *)__errno();
    pcVar3 = strerror(*piVar2);
    __android_log_print(6,DAT_00029c20 + 0x29c0e,DAT_00029c24 + 0x29c10,param_1,pcVar3);
  }
  return iVar1 == 0;
}



// Function: stopPreviewInternal @ 00029c28

/* android::QualcommCameraHardware::stopPreviewInternal() */

void android::QualcommCameraHardware::stopPreviewInternal(void)

{
  byte bVar1;
  QualcommCameraHardware *in_r0;
  uint uVar2;
  int iVar3;
  pthread_mutex_t *ppVar4;
  int iVar5;
  int iVar6;
  int iVar7;
  
  iVar3 = DAT_00029f58 + 0x29c36;
  __android_log_print(4,iVar3,DAT_00029f5c + 0x29c3c,in_r0[0x34]);
  in_r0[0xd85] = (QualcommCameraHardware)0x1;
  if ((in_r0[0x34] != (QualcommCameraHardware)0x0) && (*(int *)(in_r0 + 0xcd0) != 0)) {
    if (in_r0[0xcec] != (QualcommCameraHardware)0x0) {
      *(undefined4 *)(in_r0 + 0xd94) = 0;
      pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x368));
      iVar6 = DAT_00029f68;
      __android_log_print(4,iVar3,DAT_00029f60 + 0x29c80,DAT_00029f64 + 0x29c82);
      ppVar4 = (pthread_mutex_t *)(iVar6 + 0x29c90);
      in_r0[0x364] = (QualcommCameraHardware)0x1;
      pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x368));
      pthread_mutex_lock(ppVar4);
      pthread_cond_signal((pthread_cond_t *)(iVar6 + 0x29c94));
      pthread_mutex_unlock(ppVar4);
    }
    pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x37c));
    in_r0[0x374] = (QualcommCameraHardware)0x1;
    pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x37c));
    pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x378));
    if (in_r0[0x375] != (QualcommCameraHardware)0x0) {
      pthread_cond_signal((pthread_cond_t *)(in_r0 + 0x380));
    }
    pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x378));
    pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x3f8));
    pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x38));
    iVar3 = DAT_00029f80;
    if (in_r0[0x3fc] == (QualcommCameraHardware)0x0) {
      if (*(int *)(DAT_00029f6c + 0x29cfe) - 4U < 3) {
        if (*(int *)(in_r0 + 0xd64) == 0) {
          iVar5 = DAT_00029f70 + 0x29d2a;
          iVar6 = DAT_00029f74 + 0x29d2c;
          __android_log_print(6,iVar5,DAT_00029f78 + 0x29d30,iVar6,in_r0[0x34]);
          bVar1 = FUN_00029be0(2);
          iVar3 = DAT_00029f7c + 0x29d44;
          in_r0[0x34] = (QualcommCameraHardware)(bVar1 ^ 1);
          __android_log_print(6,iVar5,iVar3,iVar6,bVar1 ^ 1);
        }
        else {
          in_r0[0x34] = (QualcommCameraHardware)0x1;
          iVar6 = *(int *)(iVar3 + 0x29d66);
          iVar3 = (**(code **)(iVar6 + 8))(1,0,0);
          if (iVar3 == 0) {
            deinitZslBuffers(in_r0);
            iVar3 = (**(code **)(iVar6 + 0xc))(1,DAT_00029f84 + 0x29d80,0);
            if (iVar3 == 0) {
              in_r0[0x34] = (QualcommCameraHardware)0x0;
            }
          }
          if (in_r0[0x34] != (QualcommCameraHardware)0x0) {
            __android_log_print(6,DAT_00029f88 + 0x29d98,DAT_00029f8c + 0x29d9a);
          }
        }
      }
      else {
        bVar1 = FUN_00029be0(0);
        in_r0[0x34] = (QualcommCameraHardware)(bVar1 ^ 1);
      }
    }
    else {
      __android_log_print(6,DAT_00029f90 + 0x29dac,DAT_00029f94 + 0x29dae,DAT_00029f98 + 0x29db0);
      in_r0[0x34] = (QualcommCameraHardware)0x0;
    }
    pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x38));
    pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x3f8));
  }
  iVar5 = DAT_00029fa4;
  iVar6 = DAT_00029fa0;
  iVar3 = DAT_00029f9c;
  if (in_r0[0xcec] != (QualcommCameraHardware)0x0) {
    ppVar4 = (pthread_mutex_t *)(in_r0 + 0x368);
    pthread_mutex_lock(ppVar4);
    iVar7 = DAT_00029fa8 + 0x29dea;
    while (in_r0[0x365] != (QualcommCameraHardware)0x0) {
      __android_log_print(4,iVar3 + 0x29de4,iVar6 + 0x29de6,iVar5 + 0x29de8);
      pthread_cond_wait((pthread_cond_t *)(in_r0 + 0x36c),ppVar4);
      __android_log_print(4,iVar3 + 0x29de4,iVar7,iVar5 + 0x29de8);
    }
    pthread_mutex_unlock(ppVar4);
  }
  uVar2 = (uint)(byte)in_r0[0x34];
  iVar3 = DAT_00029fac + 0x29e26;
  iVar6 = DAT_00029fb0 + 0x29e28;
  __android_log_print(6,iVar3,DAT_00029fb4 + 0x29e2c,iVar6,uVar2);
  if (in_r0[0x34] == (QualcommCameraHardware)0x0) {
    uVar2 = (uint)(byte)in_r0[0x3c];
    __android_log_print(6,iVar3,DAT_00029fb8 + 0x29e4c,iVar6,uVar2);
    if (in_r0[0x3c] != (QualcommCameraHardware)0x0) {
      __android_log_print(6,iVar3,DAT_00029fbc + 0x29e62,in_r0[0x3c],uVar2);
      deinitPreview(in_r0);
      iVar6 = DAT_00029fc4;
      if (*(int *)(DAT_00029fc0 + 0x29e70) - 4U < 3) {
        pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x368));
        __android_log_print(2,iVar3,DAT_00029fc8 + 0x29e8e);
        ppVar4 = (pthread_mutex_t *)(iVar6 + 0x29e98);
        in_r0[0x364] = (QualcommCameraHardware)0x1;
        pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x368));
        pthread_mutex_lock(ppVar4);
        pthread_cond_signal((pthread_cond_t *)(iVar6 + 0x29e9c));
        pthread_mutex_unlock(ppVar4);
        __android_log_print(6,iVar3,DAT_00029fcc + 0x29ec0);
        cam_frame_flush_video();
        (*(code *)**(undefined4 **)(DAT_00029fd0 + 0x29ecc))(0);
      }
      iVar5 = DAT_00029fdc;
      iVar6 = DAT_00029fd8;
      iVar3 = DAT_00029fd4;
      ppVar4 = (pthread_mutex_t *)(in_r0 + 0x35c);
      pthread_mutex_lock(ppVar4);
      while (in_r0[0x358] != (QualcommCameraHardware)0x0) {
        __android_log_print(4,iVar3 + 0x29eec,iVar6 + 0x29eee);
        pthread_cond_wait((pthread_cond_t *)(in_r0 + 0x360),ppVar4);
        __android_log_print(4,iVar3 + 0x29eec,iVar5 + 0x29ef0);
      }
      pthread_mutex_unlock(ppVar4);
      in_r0[0x3c] = (QualcommCameraHardware)0x0;
    }
  }
  else {
    __android_log_print(4,iVar3,DAT_00029fe0 + 0x29f2c);
  }
  if ((*(int *)(in_r0 + 0xcf4) != 0) && (*(int *)(in_r0 + 0xcf0) << 0x1d < 0)) {
    cancelAutoFocusDefault(in_r0);
  }
  __android_log_print(4,DAT_00029fe4 + 0x29f4c,DAT_00029fe8 + 0x29f52,in_r0[0x34],uVar2);
  return;
}



// Function: stopPreview @ 00029fec

/* android::QualcommCameraHardware::stopPreview() */

void __thiscall android::QualcommCameraHardware::stopPreview(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  int iVar5;
  undefined4 uVar6;
  int iVar7;
  int iVar8;
  QualcommCameraHardware *pQVar9;
  int iVar10;
  int iVar11;
  int iVar12;
  int iVar13;
  
  __android_log_print(2,DAT_0002a1f0 + 0x29ff8,DAT_0002a1f4 + 0x29ffe);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  iVar3 = DAT_0002a200;
  if ((*(int *)(this + 0xcfc) == 0) || (-1 < *(int *)(this + 0xcf0) << 0x1a)) {
    if (this[0x3ac] == (QualcommCameraHardware)0x0) {
      if (*(int *)(this + 0xcd0) == 0) {
        __android_log_print(6,DAT_0002a23c + 0x2a1c8,DAT_0002a240 + 0x2a1ca,DAT_0002a244 + 0x2a1cc);
      }
      else {
        iVar10 = DAT_0002a204 + 0x2a04e;
        iVar13 = DAT_0002a208 + 0x2a052;
        iVar12 = DAT_0002a20c + 0x2a056;
        iVar5 = DAT_0002a210 + 0x2a058;
        pQVar9 = this + 0x6d0;
        iVar4 = 0;
        while( true ) {
          if (*(int *)(this + 0xd64) == 0) {
            iVar8 = *(int *)(this + 0xc);
          }
          else {
            iVar8 = 3;
          }
          if (iVar8 <= iVar4) break;
          if ((*(int *)(this + 0xcd0) != 0) && (*(int **)(pQVar9 + 0x608) != (int *)0x0)) {
            iVar8 = **(int **)(pQVar9 + 0x608);
            uVar6 = *(undefined4 *)(iVar8 + 0xc);
            __android_log_print(6,iVar10,iVar13,iVar12,uVar6);
            __android_log_print(6,iVar10,iVar5);
            pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f4));
            if (*(int *)pQVar9 == 1) {
              iVar1 = genlock_unlock_buffer(iVar8);
              if (iVar1 == 2) {
                __android_log_print(6,iVar10,DAT_0002a214 + 0x2a0b2,iVar12,uVar6);
              }
              else {
                *(undefined4 *)pQVar9 = 0;
              }
            }
            iVar1 = (**(code **)(*(int *)(this + 0xcd0) + 8))
                              (*(int *)(this + 0xcd0),*(undefined4 *)(pQVar9 + 0x608));
            __android_log_print(6,iVar3 + 0x2a0d4,DAT_0002a218 + 0x2a0cc);
            if (iVar1 != 0) {
              uVar6 = *(undefined4 *)(iVar8 + 0xc);
              __android_log_print(6,iVar3 + 0x2a0d4,DAT_0002a21c + 0x2a0ee,DAT_0002a220 + 0x2a0f0,
                                  uVar6);
            }
            iVar1 = *(int *)(this + 0xd10);
            iVar7 = *(int *)(this + 0xd14);
            if (((*(int *)(pQVar9 + -0x14) != 0) && (*(int *)(this + 0x3e4) == 1)) ||
               (*(int *)(this + 0xd64) != 0)) {
              iVar11 = DAT_0002a224 + 0x2a126;
              __android_log_print(6,iVar11,DAT_0002a228 + 0x2a128,DAT_0002a22c + 0x2a12c,
                                  *(undefined4 *)(iVar8 + 0xc));
              uVar6 = *(undefined4 *)(iVar8 + 0xc);
              iVar2 = (iVar1 * iVar7 * 3) / 2;
              FUN_0001f76c(iVar2,iVar2,iVar1 * iVar7 + 3U & 0xfffffffc,0,uVar6,0,
                           *(undefined4 *)(pQVar9 + -0x14),3,0,0,0);
              iVar8 = munmap(*(void **)(pQVar9 + -0x14),*(size_t *)(iVar8 + 0x1c));
              if (iVar8 == -1) {
                __android_log_print(6,iVar11,DAT_0002a230 + 0x2a182,iVar4);
              }
              iVar7 = 0;
              *(undefined4 *)(pQVar9 + -0x14) = 0;
              *(undefined4 *)(pQVar9 + 0x608) = 0;
            }
            __android_log_print(6,DAT_0002a234 + 0x2a198,DAT_0002a238 + 0x2a19a,iVar7,uVar6);
            pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f4));
          }
          iVar4 = iVar4 + 1;
          pQVar9 = pQVar9 + 4;
        }
      }
      stopPreviewInternal();
      iVar3 = DAT_0002a248 + 0x2a1dc;
      iVar4 = DAT_0002a24c + 0x2a1de;
    }
    else {
      iVar3 = DAT_0002a1f8 + 0x2a02c;
      iVar4 = DAT_0002a1fc + 0x2a02e;
    }
    __android_log_print(2,iVar3,iVar4);
  }
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return;
}



// Function: stopRecording @ 0002a250

/* android::QualcommCameraHardware::stopRecording() */

void __thiscall android::QualcommCameraHardware::stopRecording(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  int iVar3;
  undefined4 *puVar4;
  QualcommCameraHardware *pQVar5;
  pthread_mutex_t *__mutex;
  int *piVar6;
  int iVar7;
  int iVar8;
  
  iVar3 = DAT_0002a3d4 + 0x2a260;
  puVar4 = (undefined4 *)(DAT_0002a3dc + 0x2a268);
  __android_log_print(2,iVar3,DAT_0002a3d8 + 0x2a264);
  *puVar4 = 0;
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x410));
  this[0x3fd] = (QualcommCameraHardware)0x1;
  pthread_cond_signal((pthread_cond_t *)(this + 0x414));
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x410));
  if (((*(int *)(this + 0xcf8) == 0) || (*(int *)(DAT_0002a3e0 + 0x2a2a4) == 4)) ||
     (-1 < *(int *)(this + 0xcf0) << 0x1b)) {
    if (*(int *)(this + 0x7d8) != 0) {
      __android_log_print(4,DAT_0002a3e8 + 0x2a2cc,DAT_0002a3ec + 0x2a2ce);
      (**(code **)(*(int *)(this + 0x7d8) + 0xc))();
      *(undefined4 *)(this + 0x7d8) = 0;
    }
    piVar6 = (int *)(DAT_0002a3f0 + 0x2a2f0);
    iVar1 = DAT_0002a3f4 + 0x2a2f6;
    iVar7 = DAT_0002a3f8 + 0x2a2f8;
    iVar8 = DAT_0002a3fc + 0x2a2fc;
    pQVar5 = this;
    for (iVar3 = 0; iVar3 < *piVar6; iVar3 = iVar3 + 1) {
      if ((*(int *)(this + 0x3ec) != 0) && (*(int **)(pQVar5 + 0x7b4) != (int *)0x0)) {
        iVar2 = **(int **)(pQVar5 + 0x7b4);
        __android_log_print(4,iVar1,iVar7,iVar8,iVar3);
        native_handle_delete(*(undefined4 *)(iVar2 + 4));
        (**(code **)(*(int *)(pQVar5 + 0x7b4) + 0xc))();
        *(undefined4 *)(pQVar5 + 0x7b4) = 0;
      }
      pQVar5 = pQVar5 + 4;
    }
    if (*(int *)(this + 0x638) == 3) {
      *(undefined4 *)(this + 0x638) = 2;
      FUN_0001f008(0,2,0);
    }
    if (*(int *)(DAT_0002a400 + 0x2a356) - 4U < 3) {
      if (this[0xcec] != (QualcommCameraHardware)0x0) {
        __android_log_print(2,DAT_0002a404 + 0x2a36e,DAT_0002a408 + 0x2a370,DAT_0002a40c + 0x2a372);
        *(undefined4 *)(this + 0xd94) = 0;
        goto LAB_0002a3c8;
      }
      pthread_mutex_lock((pthread_mutex_t *)(this + 0x368));
      this[0x364] = (QualcommCameraHardware)0x1;
      iVar3 = DAT_0002a410;
      pthread_mutex_unlock((pthread_mutex_t *)(this + 0x368));
      FUN_00029be0(10);
      __mutex = (pthread_mutex_t *)(iVar3 + 0x2a3a6);
      pthread_mutex_lock(__mutex);
      pthread_cond_signal((pthread_cond_t *)(iVar3 + 0x2a3aa));
      pthread_mutex_unlock(__mutex);
    }
    iVar3 = DAT_0002a414;
    *(undefined4 *)(this + 0xd94) = 0;
    iVar3 = iVar3 + 0x2a3c4;
    iVar1 = DAT_0002a418 + 0x2a3c6;
  }
  else {
    iVar1 = DAT_0002a3e4 + 0x2a2ba;
  }
  __android_log_print(2,iVar3,iVar1);
LAB_0002a3c8:
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return;
}



// Function: cancelPicture @ 0002a41c

/* android::QualcommCameraHardware::cancelPicture() */

undefined4 __thiscall android::QualcommCameraHardware::cancelPicture(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  int iVar3;
  undefined4 uVar4;
  int iVar5;
  pthread_mutex_t *__mutex;
  
  iVar5 = DAT_0002a4f4 + 0x2a42a;
  __android_log_print(4,iVar5,DAT_0002a4f8 + 0x2a430);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0xd78));
  __android_log_print(4,iVar5,DAT_0002a4fc + 0x2a448,DAT_0002a500 + 0x2a44c);
  this[0xd74] = (QualcommCameraHardware)0x1;
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0xd78));
  iVar2 = DAT_0002a510;
  iVar1 = DAT_0002a50c;
  iVar5 = DAT_0002a508;
  iVar3 = *(int *)(DAT_0002a504 + 0x2a460);
  if ((iVar3 == 2) || (iVar3 == 3 || iVar3 == 1)) {
    __mutex = (pthread_mutex_t *)(this + 0x3b0);
    this[0xd1c] = (QualcommCameraHardware)0x1;
    pthread_mutex_lock(__mutex);
    while (this[0x3ac] != (QualcommCameraHardware)0x0) {
      __android_log_print(2,iVar5 + 0x2a498,iVar1 + 0x2a49a);
      pthread_cond_wait((pthread_cond_t *)(this + 0x3b4),__mutex);
      __android_log_print(2,iVar5 + 0x2a498,iVar2 + 0x2a49c);
    }
    pthread_mutex_unlock(__mutex);
  }
  iVar5 = FUN_00029be0(3);
  iVar1 = DAT_0002a514 + 0x2a4d4;
  iVar2 = DAT_0002a518 + 0x2a4d6;
  if (iVar5 == 0) {
    uVar4 = 0x80000000;
  }
  else {
    uVar4 = 0;
  }
  this[0xd1c] = (QualcommCameraHardware)0x0;
  __android_log_print(4,iVar1,iVar2,uVar4);
  return uVar4;
}



// Function: disableMsgType @ 0002a51c

/* android::QualcommCameraHardware::disableMsgType(int) */

void __thiscall
android::QualcommCameraHardware::disableMsgType(QualcommCameraHardware *this,int param_1)

{
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  if (2 < *(int *)(DAT_0002a560 + 0x2a532) - 4U) {
    if (*(int *)(this + 0xcf0) << 0x1a < 0) {
      FUN_00029be0(10);
      *(undefined4 *)(this + 0xd94) = 0;
    }
  }
  *(uint *)(this + 0xcf0) = *(uint *)(this + 0xcf0) & ~param_1;
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return;
}



// Function: FUN_0002a564 @ 0002a564

void FUN_0002a564(undefined4 param_1,int param_2,uint param_3,undefined4 *param_4)

{
  undefined4 uVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  
  iVar2 = *(int *)(DAT_0002a608 + 0x2a56e);
  if (iVar2 == 0x28) {
    __android_log_print(6,DAT_0002a60c + 0x2a57c,DAT_0002a610 + 0x2a57e);
    return;
  }
  iVar3 = *(int *)(DAT_0002a614 + 0x2a58e);
  iVar4 = iVar3 + iVar2 * 0x18;
  *(undefined4 *)(iVar3 + iVar2 * 0x18) = param_1;
  *(int *)(iVar4 + 4) = param_2;
  iVar3 = iVar3 + (iVar2 * 3 + 1) * 8;
  *(uint *)(iVar3 + 4) = param_3;
  *(undefined1 *)(iVar4 + 8) = 1;
  if (param_3 < 2 || param_2 != 5) {
    if (param_3 == 1 && param_2 == 5) {
      uVar1 = param_4[1];
      *(undefined4 *)(iVar3 + 8) = *param_4;
      *(undefined4 *)(iVar3 + 0xc) = uVar1;
      goto LAB_0002a5fa;
    }
    if (param_2 != 2) {
      if (param_2 == 1) {
        *(undefined1 *)(iVar3 + 8) = *(undefined1 *)param_4;
        goto LAB_0002a5fa;
      }
      if (param_3 < 2 || param_2 != 3) {
        if (param_3 == 1 && param_2 == 3) {
          *(undefined2 *)(iVar3 + 8) = *(undefined2 *)param_4;
        }
        goto LAB_0002a5fa;
      }
    }
  }
  *(undefined4 **)(iVar3 + 8) = param_4;
LAB_0002a5fa:
  *(int *)(DAT_0002a618 + 0x2a602) = iVar2 + 1;
  return;
}



// Function: FUN_0002a620 @ 0002a620

void FUN_0002a620(int param_1,char *param_2)

{
  int iVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  undefined4 extraout_r0;
  undefined4 uVar5;
  undefined4 extraout_r1;
  int *piVar6;
  double dVar7;
  
  strtod(param_2,(char **)0x0);
  iVar4 = DAT_0002a6cc;
  iVar3 = DAT_0002a6c8;
  iVar1 = (int)(longlong)ABS((double)CONCAT44(extraout_r1,extraout_r0));
  dVar7 = (ABS((double)CONCAT44(extraout_r1,extraout_r0)) - (double)(longlong)iVar1) * DAT_0002a6b8;
  iVar2 = (int)(longlong)dVar7;
  uVar5 = (undefined4)(longlong)((dVar7 - (double)(longlong)iVar2) * DAT_0002a6b8 * DAT_0002a6c0);
  if (param_1 == 0x20002) {
    piVar6 = (int *)(DAT_0002a6c8 + 0x2a676);
    *(undefined4 *)(DAT_0002a6c8 + 0x2a686) = uVar5;
    *(int *)(iVar3 + 0x2a67e) = iVar2;
    *piVar6 = iVar1;
    *(undefined4 *)(iVar3 + 0x2a68a) = 1000;
    *(undefined4 *)(iVar3 + 0x2a682) = 1;
    *(undefined4 *)(iVar3 + 0x2a67a) = 1;
    uVar5 = 0x20002;
  }
  else {
    piVar6 = (int *)(DAT_0002a6cc + 0x2a696);
    *(undefined4 *)(DAT_0002a6cc + 0x2a6a6) = uVar5;
    *(int *)(iVar4 + 0x2a69e) = iVar2;
    *piVar6 = iVar1;
    *(undefined4 *)(iVar4 + 0x2a6aa) = 1000;
    uVar5 = 0x40004;
    *(undefined4 *)(iVar4 + 0x2a6a2) = 1;
    *(undefined4 *)(iVar4 + 0x2a69a) = 1;
  }
  FUN_0002a564(uVar5,5,3);
  return;
}



// Function: setGpsParameters @ 0002a6d0

/* android::QualcommCameraHardware::setGpsParameters() */

void __thiscall android::QualcommCameraHardware::setGpsParameters(QualcommCameraHardware *this)

{
  double dVar1;
  int iVar2;
  char *pcVar3;
  undefined4 extraout_r0;
  undefined4 extraout_r0_00;
  undefined4 extraout_r0_01;
  undefined4 extraout_r0_02;
  undefined4 extraout_r0_03;
  size_t sVar4;
  tm *__tp;
  int iVar5;
  undefined4 extraout_r1;
  undefined4 extraout_r1_00;
  undefined4 extraout_r1_01;
  undefined4 extraout_r1_02;
  undefined4 extraout_r1_03;
  int iVar6;
  undefined1 uVar7;
  undefined4 *puVar8;
  int *piVar9;
  QualcommCameraHardware *pQVar10;
  void *__dest;
  char *__dest_00;
  undefined1 *puVar11;
  long local_24;
  
  *(undefined4 *)(this + 0x628) = 0;
  *(undefined4 *)(this + 0x62c) = 0;
  pQVar10 = this + 0x18;
  *(undefined4 *)(this + 0x620) = 0;
  *(undefined4 *)(this + 0x624) = 0;
  pcVar3 = (char *)android::CameraParameters::get((char *)pQVar10);
  iVar2 = DAT_0002a8f0;
  if (pcVar3 != (char *)0x0) {
    puVar11 = (undefined1 *)(DAT_0002a8f0 + 0x2a714);
    FUN_0002a620(0x20002,pcVar3);
    *(undefined1 *)(iVar2 + 0x2a715) = 0;
    strtod(pcVar3,(char **)0x0);
    if ((double)CONCAT44(extraout_r1,extraout_r0) <= 0.0) {
      uVar7 = 0x53;
    }
    else {
      uVar7 = 0x4e;
    }
    *puVar11 = uVar7;
    FUN_0002a564(0x10001,2,2,puVar11);
    strtod(pcVar3,(char **)0x0);
    *(undefined4 *)(this + 0x628) = extraout_r0_00;
    *(undefined4 *)(this + 0x62c) = extraout_r1_00;
  }
  pcVar3 = (char *)android::CameraParameters::get((char *)pQVar10);
  iVar2 = DAT_0002a8f8;
  if (pcVar3 != (char *)0x0) {
    puVar11 = (undefined1 *)(DAT_0002a8f8 + 0x2a774);
    FUN_0002a620(0x40004,pcVar3);
    *(undefined1 *)(iVar2 + 0x2a775) = 0;
    strtod(pcVar3,(char **)0x0);
    if ((double)CONCAT44(extraout_r1_01,extraout_r0_01) <= 0.0) {
      uVar7 = 0x57;
    }
    else {
      uVar7 = 0x45;
    }
    *puVar11 = uVar7;
    FUN_0002a564("aps-per-shutter",2,2,puVar11);
    strtod(pcVar3,(char **)0x0);
    *(undefined4 *)(this + 0x620) = extraout_r0_02;
    *(undefined4 *)(this + 0x624) = extraout_r1_02;
  }
  pcVar3 = (char *)android::CameraParameters::get((char *)pQVar10);
  if (pcVar3 != (char *)0x0) {
    strtod(pcVar3,(char **)0x0);
    dVar1 = DAT_0002a8e0;
    puVar8 = (undefined4 *)(DAT_0002a900 + 0x2a7dc);
    *(undefined4 *)(DAT_0002a900 + 0x2a7e0) = 1000;
    *puVar8 = (int)(longlong)((double)CONCAT44(extraout_r1_03,extraout_r0_03) * dVar1);
    FUN_0002a564(0x60006,5,1);
    *(bool *)(DAT_0002a904 + 0x2a806) = (double)CONCAT44(extraout_r1_03,extraout_r0_03) <= 0.0;
    FUN_0002a564(0x50005,1,1);
  }
  pcVar3 = (char *)android::CameraParameters::get((char *)pQVar10);
  iVar2 = DAT_0002a90c;
  if (pcVar3 != (char *)0x0) {
    __dest = (void *)(DAT_0002a90c + 0x2a834);
    __dest_00 = (char *)(DAT_0002a90c + 0x2a83c);
    memcpy(__dest,(void *)(DAT_0002a910 + 0x2a836),8);
    strncpy(__dest_00,pcVar3,100);
    *(undefined1 *)(iVar2 + 0x2a8a0) = 0;
    sVar4 = strlen(__dest_00);
    FUN_0002a564(0x1b001b,2,sVar4 + 9,__dest);
  }
  pcVar3 = (char *)android::CameraParameters::get((char *)pQVar10);
  if (pcVar3 != (char *)0x0) {
    local_24 = atol(pcVar3);
    __tp = gmtime(&local_24);
    pcVar3 = (char *)(DAT_0002a918 + 0x2a890);
    strftime(pcVar3,0x14,(char *)(DAT_0002a91c + 0x2a892),__tp);
    sVar4 = strlen(pcVar3);
    FUN_0002a564(0x1d001d,2,sVar4 + 1,pcVar3);
    iVar2 = DAT_0002a920;
    iVar6 = __tp->tm_hour;
    piVar9 = (int *)(DAT_0002a920 + 0x2a8b8);
    iVar5 = __tp->tm_min;
    *(int *)(DAT_0002a920 + 0x2a8c8) = __tp->tm_sec;
    *piVar9 = iVar6;
    *(undefined4 *)(iVar2 + 0x2a8cc) = 1;
    *(int *)(iVar2 + 0x2a8c0) = iVar5;
    *(undefined4 *)(iVar2 + 0x2a8c4) = 1;
    *(undefined4 *)(iVar2 + 0x2a8bc) = 1;
    FUN_0002a564(0x70007,5,3);
  }
  return;
}



// Function: set_liveshot_exifinfo @ 0002a928

/* android::QualcommCameraHardware::set_liveshot_exifinfo() */

void __thiscall android::QualcommCameraHardware::set_liveshot_exifinfo(QualcommCameraHardware *this)

{
  undefined4 uVar1;
  int iVar2;
  char *__src;
  char *__dest;
  
  setGpsParameters(this);
  __src = (char *)android::CameraParameters::get((char *)(this + 0x18));
  iVar2 = DAT_0002a968;
  if (__src != (char *)0x0) {
    __dest = (char *)(DAT_0002a968 + 0x2a94a);
    strncpy(__dest,__src,0x13);
    uVar1 = DAT_0002a960;
    *(undefined1 *)(iVar2 + 0x2a95d) = 0;
    FUN_0002a564(uVar1,2,0x14,__dest);
  }
  return;
}



// Function: takeLiveSnapshotInternal @ 0002a96c

/* android::QualcommCameraHardware::takeLiveSnapshotInternal() */

undefined4 __thiscall
android::QualcommCameraHardware::takeLiveSnapshotInternal(QualcommCameraHardware *this)

{
  char cVar1;
  int iVar2;
  int iVar3;
  int *piVar4;
  int *piVar5;
  double dVar6;
  
  iVar3 = DAT_0002aae8 + 0x2a980;
  piVar4 = (int *)(DAT_0002aaec + 0x2a982);
  __android_log_print(2,iVar3,DAT_0002aaf0 + 0x2a986);
  if ((*piVar4 != 1) && (*(int *)(this + 0xd94) != 0)) {
    piVar5 = (int *)(DAT_0002aaf4 + 0x2a9a4);
    iVar2 = *piVar5;
    if (iVar2 - 5U < 2 || iVar2 == 3) {
      *piVar4 = 1;
      iVar2 = initLiveSnapshot(this,*(int *)(this + 0xd2c),*(int *)(this + 0xd30));
      if (iVar2 == 0) {
        __android_log_print(6,iVar3,DAT_0002aafc + 0x2a9f2);
        *piVar4 = 2;
        return 0x80000000;
      }
      dVar6 = (double)(longlong)(*(int *)(this + 0xd2c) * *(int *)(this + 0xd30)) * 1.5;
      set_liveshot_exifinfo(this);
      cVar1 = (*(code *)**(undefined4 **)(DAT_0002ab00 + 0x2aa2a))
                        (*(undefined4 *)(this + 0xd2c),*(undefined4 *)(this + 0xd30),
                         *(undefined4 *)(DAT_0002ab04 + 0x2aa2e),
                         *(undefined4 *)(DAT_0002ab08 + 0x2aa4a),**(undefined4 **)(this + 0x7d8),
                         (uint)(0.0 < dVar6) * (int)(longlong)dVar6);
      if (cVar1 == '\0') {
        __android_log_print(6,iVar3,DAT_0002ab0c + 0x2aa60);
        if (*(int *)(this + 0x7d8) != 0) {
          __android_log_print(2,iVar3,DAT_0002ab10 + 0x2aa74);
          (**(code **)(*(int *)(this + 0x7d8) + 0xc))();
          *(undefined4 *)(this + 0x7d8) = 0;
          return 0;
        }
      }
      else {
        if ((*piVar5 - 5U < 2) && (iVar2 = FUN_0001ef80(8,0), iVar2 == 0)) {
          __android_log_print(6,iVar3,DAT_0002ab14 + 0x2aaa4);
          *piVar4 = 2;
          if (*(int *)(this + 0x7d8) == 0) {
            return 0x80000000;
          }
          __android_log_print(2,iVar3,DAT_0002ab18 + 0x2aab8);
          (**(code **)(*(int *)(this + 0x7d8) + 0xc))();
          *(undefined4 *)(this + 0x7d8) = 0;
          return 0x80000000;
        }
        __android_log_print(2,DAT_0002ab1c + 0x2aad6,DAT_0002ab20 + 0x2aad8);
      }
    }
    else {
      __android_log_print(4,iVar3,DAT_0002aaf8 + 0x2a9cc);
      *piVar4 = 2;
    }
  }
  return 0;
}



// Function: jpeg_set_location @ 0002ab24

/* android::QualcommCameraHardware::jpeg_set_location() */

void android::QualcommCameraHardware::jpeg_set_location(void)

{
  bool bVar1;
  QualcommCameraHardware *in_r0;
  char *pcVar2;
  int iVar3;
  undefined4 in_r1;
  QualcommCameraHardware *pQVar4;
  QualcommCameraHardware *local_20;
  undefined4 uStack_1c;
  
  pQVar4 = in_r0 + 0x18;
  local_20 = in_r0;
  uStack_1c = in_r1;
  pcVar2 = (char *)android::CameraParameters::get((char *)pQVar4);
  if (pcVar2 == (char *)0x0) {
    __android_log_print(2,DAT_0002ac98 + 0x2ab74,DAT_0002ac9c + 0x2ab76);
LAB_0002ac82:
    bVar1 = false;
  }
  else {
    local_20 = (QualcommCameraHardware *)0x0;
    iVar3 = sscanf(pcVar2,(char *)(DAT_0002ac8c + 0x2ab44),&local_20);
    if (iVar3 != 1) {
      __android_log_print(6,DAT_0002ac90 + 0x2ab64,DAT_0002ac94 + 0x2ab66,pcVar2);
      goto LAB_0002ac82;
    }
    bVar1 = true;
    if (local_20 != (QualcommCameraHardware *)0x0) goto LAB_0002ab82;
  }
  time((time_t *)0x0);
LAB_0002ab82:
  pcVar2 = (char *)android::CameraParameters::get((char *)pQVar4);
  if (pcVar2 == (char *)0x0) {
    __android_log_print(2,DAT_0002acb0 + 0x2abcc,DAT_0002acb4 + 0x2abce);
    bVar1 = false;
  }
  else {
    local_20 = (QualcommCameraHardware *)((uint)local_20 & 0xffff0000);
    iVar3 = sscanf(pcVar2,(char *)(DAT_0002aca4 + 0x2aba4),&local_20);
    if (iVar3 != 1) {
      bVar1 = false;
      __android_log_print(6,DAT_0002aca8 + 0x2abb8,DAT_0002acac + 0x2abbc,pcVar2);
    }
  }
  pcVar2 = (char *)android::CameraParameters::get((char *)pQVar4);
  if (pcVar2 == (char *)0x0) {
    __android_log_print(2,DAT_0002acc8 + 0x2ac1a,DAT_0002accc + 0x2ac1c);
    bVar1 = false;
  }
  else {
    local_20 = (QualcommCameraHardware *)0x0;
    uStack_1c = 0;
    iVar3 = sscanf(pcVar2,(char *)(DAT_0002acbc + 0x2abf4),&local_20);
    if (iVar3 != 1) {
      bVar1 = false;
      __android_log_print(6,DAT_0002acc0 + 0x2ac06,DAT_0002acc4 + 0x2ac0a,pcVar2);
    }
  }
  pcVar2 = (char *)android::CameraParameters::get((char *)pQVar4);
  if (pcVar2 == (char *)0x0) {
    __android_log_print(2,DAT_0002ace0 + 0x2ac62,DAT_0002ace4 + 0x2ac64);
  }
  else {
    local_20 = (QualcommCameraHardware *)0x0;
    uStack_1c = 0;
    iVar3 = sscanf(pcVar2,(char *)(DAT_0002acd4 + 0x2ac40),&local_20);
    if (iVar3 == 1) {
      if (bVar1) {
        setGpsParameters(in_r0);
        return;
      }
    }
    else {
      __android_log_print(6,DAT_0002acd8 + 0x2ac52,DAT_0002acdc + 0x2ac54,pcVar2);
    }
  }
  __android_log_print(2,DAT_0002ace8 + 0x2ac7c,DAT_0002acec + 0x2ac7e);
  return;
}



// Function: initImageEncodeParameters @ 0002acf0

/* android::QualcommCameraHardware::initImageEncodeParameters(int) */

void __thiscall
android::QualcommCameraHardware::initImageEncodeParameters(QualcommCameraHardware *this,int param_1)

{
  int iVar1;
  int iVar2;
  float fVar3;
  int extraout_r1;
  int iVar4;
  undefined4 *puVar5;
  undefined4 uVar6;
  int iVar7;
  QualcommCameraHardware *pQVar8;
  int iVar9;
  int local_ac [5];
  undefined4 local_98;
  undefined2 local_92;
  char acStack_90 [92];
  int local_34;
  
  pQVar8 = this + 0x18;
  iVar7 = DAT_0002b0a0 + 0x2ad0a;
  iVar9 = DAT_0002b0a8 + 0x2ad12;
  local_34 = **(int **)(iVar7 + DAT_0002b0a4);
  __android_log_print(2,iVar9,DAT_0002b0ac + 0x2ad14,DAT_0002b0b0 + 0x2ad20);
  memset((void *)(DAT_0002b0b4 + 0x2ad2c),0,0x30);
  iVar2 = android::CameraParameters::getInt((char *)pQVar8);
  if (-1 < iVar2) {
    __android_log_print(2,iVar9,DAT_0002b0bc + 0x2ad4a,iVar2);
    if (iVar2 == 0x46) {
      local_ac[4] = 0x5a;
    }
    else if (iVar2 == 0x28) {
      local_ac[4] = 0x55;
    }
    else {
      local_ac[4] = 0x5f;
    }
    *(int *)(DAT_0002b0c0 + 0x2ad88) = local_ac[4];
    iVar2 = native_set_parms(this,0x28,4,local_ac + 4);
    if (iVar2 == 0) {
      __android_log_print(6,DAT_0002b0c4 + 0x2ad88,DAT_0002b0c8 + 0x2ad8a);
      uVar6 = 0;
      goto LAB_0002b056;
    }
    local_98 = local_ac[4];
    native_set_parms(this,0x29,4,&local_98);
  }
  iVar2 = android::CameraParameters::getInt((char *)pQVar8);
  property_get(DAT_0002b0d4 + 0x2adbe,acStack_90,DAT_0002b0d0 + 0x2adb8);
  iVar9 = strcmp(acStack_90,(char *)(DAT_0002b0d8 + 0x2adc8));
  if (iVar9 == 0) {
    __aeabi_idivmod(iVar2 + 0x5a,0x168);
    iVar2 = extraout_r1;
  }
  iVar9 = DAT_0002b0dc;
  if (this[0xcec] == (QualcommCameraHardware)0x0) {
    if (-1 < iVar2) goto LAB_0002ade8;
  }
  else {
    iVar2 = 0;
LAB_0002ade8:
    __android_log_print(2,DAT_0002b0e0 + 0x2adfa,DAT_0002b0e4 + 0x2adfc,iVar2);
    *(int *)(iVar9 + 0x2ae08) = iVar2;
  }
  jpeg_set_location();
  local_ac[2] = 0;
  local_ac[3] = 0;
  local_ac[0] = 0;
  local_ac[1] = 0;
  FUN_0002a564(DAT_0002b074,2,0xe,DAT_0002b0ec + 0x2ae18);
  local_92 = 1;
  FUN_0002a564(DAT_0002b078,3,1,&local_92);
  fVar3 = (float)android::CameraParameters::getFloat((char *)pQVar8);
  local_ac[3] = 100;
  local_ac[2] = (int)(fVar3 * DAT_0002b07c);
  FUN_0002a564(DAT_0002b080,5,1,local_ac + 2);
  if (*(int *)(DAT_0002b0f0 + 0x2ae6e) == 0) {
    puVar5 = (undefined4 *)(DAT_0002b0f4 + 0x2ae76);
  }
  else {
    puVar5 = (undefined4 *)(DAT_0002b0f8 + 0x2ae7c);
  }
  local_ac[2] = *puVar5;
  local_ac[3] = puVar5[1];
  FUN_0002a564(DAT_0002b084,5,1,local_ac + 2);
  local_92 = 3;
  FUN_0002a564(DAT_0002b088,3,1,&local_92);
  local_92 = 0;
  FUN_0002a564(DAT_0002b08c,3,1,&local_92);
  iVar2 = android::CameraParameters::getInt((char *)pQVar8);
  fVar3 = (float)android::CameraParameters::getFloat((char *)pQVar8);
  local_ac[1] = 100;
  local_ac[0] = (int)((float)(longlong)iVar2 * fVar3 * DAT_0002b07c);
  FUN_0002a564(DAT_0002b090,10,1,local_ac);
  local_92 = *(undefined2 *)(this + 0x43c);
  FUN_0002a564(DAT_0002b094,3,1,&local_92);
  local_92 = *(undefined2 *)(this + 0x43e);
  FUN_0002a564(DAT_0002b098,3,1,&local_92);
  iVar2 = android::CameraParameters::getInt((char *)pQVar8);
  if (iVar2 == 0x5a) {
    local_92 = 6;
  }
  else if (iVar2 < 0x5b) {
LAB_0002af60:
    local_92 = 1;
  }
  else if (iVar2 == 0xb4) {
    local_92 = 3;
  }
  else {
    if (iVar2 != 0x10e) goto LAB_0002af60;
    local_92 = 8;
  }
  FUN_0002a564(DAT_0002b09c,3,1,&local_92);
  if (this[0xd84] != (QualcommCameraHardware)0x0) {
    __android_log_print(2,DAT_0002b108 + 0x2af88,DAT_0002b10c + 0x2af8a,DAT_0002b110 + 0x2af8c);
    iVar2 = DAT_0002b114;
    *(undefined4 *)(DAT_0002b114 + 0x2afbe) = *(undefined4 *)(this + 0xd7c);
    *(undefined4 *)(iVar2 + 0x2afc2) = *(undefined4 *)(this + 0xd80);
  }
  iVar9 = DAT_0002b120;
  iVar2 = DAT_0002b11c;
  pQVar8 = this + 0x760;
  iVar4 = *(int *)(DAT_0002b118 + 0x2afba);
  *(undefined4 *)(DAT_0002b11c + 0x2afda) = *(undefined4 *)(this + 0x42c);
  iVar1 = DAT_0002b124;
  if (iVar4 == 2) {
    *(undefined4 *)(iVar2 + 0x2afda) = *(undefined4 *)(this + 0x42c);
  }
  iVar2 = DAT_0002b128;
  *(undefined4 *)(iVar9 + 0x2afec) = 0;
  puVar5 = (undefined4 *)(iVar1 + 0x2afe4);
  for (iVar9 = 0; iVar4 = DAT_0002b138, iVar1 = DAT_0002b130, iVar9 < param_1; iVar9 = iVar9 + 1) {
    memset((void *)(iVar9 * 0x14 + iVar2 + 0x2afec),0,0x14);
    pQVar8 = pQVar8 + 4;
    *puVar5 = **(undefined4 **)pQVar8;
    puVar5[1] = *(undefined4 *)(this + 0x430);
    puVar5[2] = *(undefined4 *)(this + 0x430);
    puVar5[4] = 0;
    puVar5[3] = 0xffffffff;
    puVar5 = puVar5 + 5;
  }
  uVar6 = *(undefined4 *)(iVar7 + DAT_0002b12c);
  puVar5 = (undefined4 *)(DAT_0002b130 + 0x2b03a);
  *(int *)(DAT_0002b130 + 0x2b042) = DAT_0002b134 + 0x2b03e;
  *puVar5 = uVar6;
  *(undefined4 *)(iVar1 + 0x2b03e) = *(undefined4 *)(iVar4 + 0x2b048);
  *(uint *)(iVar1 + 0x2b066) = (uint)(byte)this[0xcec];
  uVar6 = 1;
LAB_0002b056:
  if (local_34 != **(int **)(iVar7 + DAT_0002b0a4)) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail(uVar6);
  }
  return;
}



// Function: initRaw @ 0002b13c

/* android::QualcommCameraHardware::initRaw(bool) */

void __thiscall android::QualcommCameraHardware::initRaw(QualcommCameraHardware *this,bool param_1)

{
  int iVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  undefined4 uVar4;
  undefined4 uVar5;
  int extraout_r1;
  int iVar6;
  uint uVar7;
  undefined4 *__s;
  int iVar8;
  CameraParameters *pCVar9;
  int iVar10;
  int iVar11;
  undefined1 auStack_a0 [4];
  undefined4 local_9c;
  undefined4 local_98;
  int local_94;
  undefined4 local_90;
  int local_8c;
  char acStack_88 [92];
  int local_2c;
  
  iVar8 = DAT_0002b64c + 0x2b14e;
  local_2c = **(int **)(iVar8 + DAT_0002b650);
  pCVar9 = (CameraParameters *)(this + 0x18);
  android::CameraParameters::getPictureSize((int *)pCVar9,(int *)(this + 0xd4c));
  *(undefined4 *)(this + 0xd7c) = *(undefined4 *)(this + 0xd4c);
  *(undefined4 *)(this + 0xd80) = *(undefined4 *)(this + 0xd50);
  iVar1 = updatePictureDimension(this,pCVar9,(int *)(this + 0xd4c),(int *)(this + 0xd50));
  if (iVar1 != 0) {
    *(undefined2 *)(this + 0x43c) = *(undefined2 *)(this + 0xd4c);
    *(undefined2 *)(this + 0x43e) = *(undefined2 *)(this + 0xd50);
  }
  __android_log_print(2,DAT_0002b654 + 0x2b1b4,DAT_0002b658 + 0x2b1b6,*(undefined4 *)(this + 0xd4c),
                      *(undefined4 *)(this + 0xd50));
  if ((this[0xcec] == (QualcommCameraHardware)0x0) || (*(int *)(this + 0xd70) != 1)) {
    iVar1 = 1;
  }
  else {
    iVar1 = 2;
  }
  iVar10 = DAT_0002b660 + 0x2b1e2;
  uVar2 = android::CameraParameters::getInt((char *)pCVar9);
  uVar3 = android::CameraParameters::getInt((char *)pCVar9);
  __android_log_print(2,iVar10,DAT_0002b668 + 0x2b200,uVar2,uVar3);
  iVar11 = DAT_0002b670;
  iVar6 = DAT_0002b66c;
  *(undefined4 *)(this + 0xd40) = uVar2;
  *(undefined4 *)(this + 0xd44) = uVar3;
  uVar5 = uVar3;
  __android_log_print(6,iVar10,iVar6 + 0x2b21a,*(undefined4 *)(iVar11 + 0x2b22a),uVar2,uVar3);
  iVar6 = *(int *)(this + 0xd14);
  *(undefined4 *)(this + 0xd58) = *(undefined4 *)(this + 0xd44);
  *(undefined4 *)(this + 0xd54) = *(undefined4 *)(this + 0xd40);
  if (((*(int *)(this + 0xd50) < iVar6) || (*(int *)(DAT_0002b674 + 0x2b252) == 2)) ||
     (this[0xcec] != (QualcommCameraHardware)0x0)) {
    if (*(int *)(this + 0xd80) < *(int *)(this + 0xd44)) {
      *(undefined4 *)(this + 0xd58) = 0x90;
      uVar4 = __aeabi_idiv(*(int *)(this + 0xd7c) * 0x90);
      *(undefined4 *)(this + 0xd44) = 0x90;
      *(undefined4 *)(this + 0xd54) = uVar4;
      *(undefined4 *)(this + 0xd40) = uVar4;
    }
  }
  else {
    *(int *)(this + 0xd58) = iVar6;
    uVar4 = __aeabi_idiv(iVar6 * *(int *)(this + 0xd4c));
    *(undefined4 *)(this + 0xd54) = uVar4;
  }
  iVar11 = DAT_0002b67c + 0x2b2b2;
  iVar6 = DAT_0002b680 + 0x2b2b6;
  if (*(int *)(DAT_0002b678 + 0x2b2ae) == 2) {
    *(undefined4 *)(this + 0x470) = 2;
    *(undefined4 *)(this + 0x46c) = 2;
  }
  __android_log_print(6,iVar11,iVar6,*(undefined4 *)(this + 0xd54),*(undefined4 *)(this + 0xd58),
                      uVar5);
  *(undefined2 *)(this + 0x44c) = *(undefined2 *)(this + 0xd58);
  *(undefined2 *)(this + 0x44e) = *(undefined2 *)(this + 0xd54);
  iVar6 = native_set_parms(this,3,0x194,this + 0x438);
  if (iVar6 == 0) {
    __android_log_print(6,iVar11,DAT_0002b684 + 0x2b2fa);
    uVar5 = 0;
  }
  else {
    iVar11 = *(int *)(this + 0xd50);
    iVar10 = iVar11 * *(int *)(this + 0xd4c);
    *(int *)(this + 0x428) = (iVar1 * iVar10 * 3) / 2;
    iVar6 = DAT_0002b688;
    *(uint *)(this + 0x42c) = iVar1 * iVar10 + 3U & 0xfffffffc;
    if (*(int *)(iVar6 + 0x2b334) == 2) {
      iVar6 = *(int *)(this + 0xd4c) * iVar1;
      uVar7 = (iVar11 + 0x1fU & 0xffffffe0) * (iVar6 + 0x1fU & 0xffffffe0) + 0xfff & 0xfffff000;
      *(uint *)(this + 0x42c) = uVar7;
      *(uint *)(this + 0x428) =
           (iVar6 / 2 + 0x1fU & 0xffffffe0) * (iVar11 / 2 + 0x1fU & 0xffffffe0) * 2 + uVar7;
    }
    iVar6 = DAT_0002b6e4;
    iVar11 = *(int *)(DAT_0002b68c + 0x2b38a);
    if ((iVar11 == 2) || (iVar11 == 3 || iVar11 == 1)) {
      *(int *)(this + 0x430) =
           (int)((*(int *)(this + 0xd4c) * iVar1 + 0xfU & 0xfff0) *
                (*(int *)(this + 0xd50) + 0xfU & 0xfff0) * 3) >> 1;
    }
    else {
      iVar11 = *(int *)(this + 0xd50);
      *(int *)(this + 0x430) = (iVar1 * iVar11 * *(int *)(this + 0xd4c) * 3) / 2;
      if (*(int *)(iVar6 + 0x2b63a) == 2) {
        iVar6 = *(int *)(this + 0xd4c) * iVar1;
        *(uint *)(this + 0x430) =
             (iVar6 / 2 + 0x1fU & 0xffffffe0) * (iVar11 / 2 + 0x1fU & 0xffffffe0) * 2 +
             ((iVar11 + 0x1fU & 0xffffffe0) * (iVar6 + 0x1fU & 0xffffffe0) + 0xfff & 0xfffff000);
      }
    }
    local_8c = android::CameraParameters::getInt((char *)pCVar9);
    property_get(DAT_0002b698 + 0x2b440,acStack_88,DAT_0002b694 + 0x2b43a);
    iVar6 = strcmp(acStack_88,(char *)(DAT_0002b69c + 0x2b44a));
    if (iVar6 == 0) {
      __aeabi_idivmod(local_8c + 0x5a,0x168);
      local_8c = extraout_r1;
    }
    if (this[0xcec] != (QualcommCameraHardware)0x0) {
      local_8c = 0;
    }
    if (*(int *)(DAT_0002b6a0 + 0x2b46c) == 1) {
      local_8c = 0;
    }
    iVar6 = native_set_parms(this,0x27,4,&local_8c);
    if (iVar6 == 0) {
      __android_log_print(6,DAT_0002b6a4 + 0x2b48e,DAT_0002b6a8 + 0x2b490);
      uVar5 = 0;
    }
    else {
      if (this[0xcec] == (QualcommCameraHardware)0x0) {
        local_94 = *(int *)(this + 0xd4c) * iVar1;
        local_90 = *(undefined4 *)(this + 0xd50);
        (**(code **)(DAT_0002b6ac + 0x2b4b4))(0x26,auStack_a0);
        *(undefined4 *)(this + 0x428) = local_98;
        *(undefined4 *)(this + 0x430) = local_98;
        *(undefined4 *)(this + 0x42c) = local_9c;
      }
      if (1 < *(int *)(DAT_0002b6b0 + 0x2b4d2) - 2U) {
        android::CameraParameters::getPreviewSize((int *)pCVar9,(int *)(this + 0xd10));
      }
      iVar11 = DAT_0002b6b4 + 0x2b4f4;
      __android_log_print(2,iVar11,DAT_0002b6b8 + 0x2b4f6);
      iVar6 = createSnapshotMemory(this,*(int *)(this + 0xc),*(int *)(this + 0xc),param_1,1);
      iVar1 = DAT_0002b6c0;
      if (iVar6 == 0) {
        __android_log_print(6,iVar11,DAT_0002b6bc + 0x2b518);
        uVar5 = 0;
      }
      else {
        __s = (undefined4 *)(DAT_0002b6c0 + 0x2b528);
        initImageEncodeParameters(this,*(int *)(this + 0xc));
        memset(__s,0,0x1c);
        *(undefined4 *)(iVar1 + 0x2b540) = *(undefined4 *)(this + 0xc);
        *__s = *(undefined4 *)(this + 0xd4c);
        *(undefined4 *)(iVar1 + 0x2b52c) = *(undefined4 *)(this + 0xd50);
        *(undefined4 *)(iVar1 + 0x2b530) = *(undefined4 *)(this + 0xd54);
        *(undefined4 *)(iVar1 + 0x2b534) = *(undefined4 *)(this + 0xd58);
        iVar6 = android::CameraParameters::getInt((char *)pCVar9);
        iVar10 = android::CameraParameters::getInt((char *)pCVar9);
        __android_log_print(2,iVar11,DAT_0002b6c4 + 0x2b57c,uVar2,uVar3);
        if (iVar10 == 0 || iVar6 == 0) {
          *(undefined4 *)(iVar1 + 0x2b538) = 0;
          *(undefined4 *)(iVar1 + 0x2b53c) = 0;
        }
        else {
          *(undefined4 *)(iVar1 + 0x2b538) = *(undefined4 *)(this + 0xd40);
          *(undefined4 *)(iVar1 + 0x2b53c) = *(undefined4 *)(this + 0xd44);
        }
        iVar1 = DAT_0002b6c8;
        iVar6 = DAT_0002b6cc + 0x2b5b6;
        iVar11 = DAT_0002b6d0 + 0x2b5ba;
        __android_log_print(4,iVar6,DAT_0002b6d4 + 0x2b5bc,iVar11,
                            *(undefined4 *)(DAT_0002b6c8 + 0x2b5b0),
                            *(undefined4 *)(DAT_0002b6c8 + 0x2b5b4));
        __android_log_print(4,iVar6,DAT_0002b6d8 + 0x2b5d4,iVar11,*(undefined4 *)(iVar1 + 0x2b5b8),
                            *(undefined4 *)(iVar1 + 0x2b5bc));
        __android_log_print(4,iVar6,DAT_0002b6dc + 0x2b5ea,iVar11,*(undefined4 *)(iVar1 + 0x2b5c0),
                            *(undefined4 *)(iVar1 + 0x2b5c4));
        __android_log_print(2,iVar6,DAT_0002b6e0 + 0x2b5fe);
        uVar5 = 1;
      }
    }
  }
  if (local_2c == **(int **)(iVar8 + DAT_0002b650)) {
    return;
  }
                    /* WARNING: Subroutine does not return */
  __stack_chk_fail(uVar5);
}



// Function: takePicture @ 0002b6e8

/* android::QualcommCameraHardware::takePicture() */

undefined4 __thiscall android::QualcommCameraHardware::takePicture(QualcommCameraHardware *this)

{
  QualcommCameraHardware QVar1;
  int iVar2;
  undefined4 uVar3;
  char *pcVar4;
  int iVar5;
  bool bVar6;
  ushort uVar7;
  int iVar8;
  ushort *puVar9;
  pthread_mutex_t *ppVar10;
  undefined4 local_24;
  
  __android_log_print(6,DAT_0002ba38 + 0x2b6f8,DAT_0002ba3c + 0x2b6fe,*(undefined4 *)(this + 0xcf0))
  ;
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  iVar2 = DAT_0002ba48;
  iVar8 = DAT_0002ba44;
  iVar5 = DAT_0002ba40;
  if (*(int *)(this + 0xd94) != 0) {
    uVar3 = (**(code **)(*(int *)this + 0x54))(this);
    goto LAB_0002ba1c;
  }
  if (this[0xd48] != (QualcommCameraHardware)0x0) {
    ppVar10 = (pthread_mutex_t *)(this + 0x3d8);
    pthread_mutex_lock(ppVar10);
    while (this[0x3d4] != (QualcommCameraHardware)0x0) {
      __android_log_print(6,iVar5 + 0x2b73e,iVar8 + 0x2b740);
      pthread_cond_wait((pthread_cond_t *)(this + 0x3dc),ppVar10);
      __android_log_print(6,iVar5 + 0x2b73e,iVar2 + 0x2b742);
    }
    pthread_mutex_unlock(ppVar10);
  }
  iVar2 = DAT_0002ba54;
  iVar8 = DAT_0002ba50;
  iVar5 = DAT_0002ba4c;
  ppVar10 = (pthread_mutex_t *)(this + 0x3b0);
  pthread_mutex_lock(ppVar10);
  while( true ) {
    QVar1 = this[0x3ac];
    if ((byte)QVar1 == 0) break;
    __android_log_print(2,iVar5 + 0x2b786,iVar8 + 0x2b788);
    pthread_cond_wait((pthread_cond_t *)(this + 0x3b4),ppVar10);
    __android_log_print(2,iVar5 + 0x2b786,iVar2 + 0x2b78a);
  }
  this[0xd6c] = QVar1;
  if ((*(int *)(this + 0xd64) != 0) &&
     (local_24 = (uint)(byte)QVar1, (**(code **)(DAT_0002ba58 + 0x2b7cc))(0x24,&local_24),
     local_24 != 0)) {
    this[0xd6c] = (QualcommCameraHardware)0x1;
  }
  pcVar4 = (char *)android::CameraParameters::get((char *)(this + 0x18));
  if (pcVar4 != (char *)0x0) {
    local_24 = 0;
    iVar5 = (**(code **)(DAT_0002ba60 + 0x2b800))(0x24,&local_24);
    if (iVar5 == 0) {
      iVar5 = strcmp(pcVar4,(char *)(DAT_0002ba68 + 0x2b816));
      if (iVar5 == 0) {
        *(undefined2 *)(DAT_0002ba6c + 0x2b826) = 1;
      }
      iVar5 = strcmp(pcVar4,(char *)(DAT_0002ba70 + 0x2b830));
      if (iVar5 == 0) {
        *(undefined2 *)(DAT_0002ba74 + 0x2b83a) = 0;
      }
      iVar5 = strcmp(pcVar4,(char *)(DAT_0002ba78 + 0x2b842));
      if (iVar5 == 0) {
        puVar9 = (ushort *)(DAT_0002ba7c + 0x2b852);
        *puVar9 = 0x18;
        if (local_24 != 0) {
          uVar7 = (ushort)((int)local_24 >> 1) | 0x18;
          goto LAB_0002b85c;
        }
      }
    }
    else {
      uVar7 = 0x20;
      puVar9 = (ushort *)(DAT_0002ba64 + 0x2b80e);
LAB_0002b85c:
      *puVar9 = uVar7;
    }
    FUN_0002a564(DAT_0002ba34,3,1,DAT_0002ba80 + 0x2b86a);
  }
  iVar5 = android::CameraParameters::getPictureFormat();
  if (iVar5 == 0) {
LAB_0002ba2a:
    *(undefined4 *)(this + 0x3e4) = 1;
  }
  else {
    pcVar4 = (char *)android::CameraParameters::getPictureFormat();
    iVar5 = strcmp(pcVar4,*(char **)(DAT_0002ba84 + 0x2b884));
    if (iVar5 != 0) goto LAB_0002ba2a;
    *(undefined4 *)(this + 0x3e4) = 2;
    this[0xd6c] = (QualcommCameraHardware)0x1;
  }
  if (*(int *)(DAT_0002ba88 + 0x2b8a0) == 0) {
    uVar3 = 2;
  }
  else {
    uVar3 = 1;
  }
  *(undefined4 *)(this + 0x3e4) = uVar3;
  if ((((*(int *)(this + 0xd64) == 0) || (this[0xd6c] != (QualcommCameraHardware)0x0)) &&
      (*(int *)(this + 0x3e4) == 1)) && (iVar5 = FUN_0001ef80(6,0), iVar5 == 0)) {
    pthread_mutex_unlock(ppVar10);
    __android_log_print(6,DAT_0002ba8c + 0x2b8da,DAT_0002ba90 + 0x2b8dc);
LAB_0002b962:
    uVar3 = 0x80000000;
  }
  else {
    if ((*(int *)(this + 0xd64) == 0) || (this[0xd6c] != (QualcommCameraHardware)0x0)) {
      stopPreviewInternal();
    }
    if (*(int *)(this + 0x3e4) == 1) {
      uVar3 = 0xd;
    }
    else {
      uVar3 = 0xe;
    }
    if (this[0xd48] != (QualcommCameraHardware)0x0) {
      uVar3 = 3;
    }
    if ((*(int *)(this + 0xd64) == 0) || (this[0xd6c] != (QualcommCameraHardware)0x0)) {
      (*(code *)**(undefined4 **)(DAT_0002ba94 + 0x2b91c))(uVar3,0,0);
    }
    if (*(int *)(this + 0x3e4) == 1) {
      if ((*(int *)(this + 0xd64) == 0) || (this[0xd6c] != (QualcommCameraHardware)0x0)) {
        bVar6 = false;
        if (*(int *)(this + 0xcf8) != 0) {
          bVar6 = SUB41((uint)(*(int *)(this + 0xcf0) << 0x17) >> 0x1f,0);
        }
        iVar5 = initRaw(this,bVar6);
        if (iVar5 == 0) {
          iVar5 = DAT_0002ba98 + 0x2b956;
          iVar8 = DAT_0002ba9c + 0x2b958;
          goto LAB_0002b956;
        }
      }
    }
    else if ((*(int *)(this + 0x3e4) == 2) && (iVar5 = initRawSnapshot(), iVar5 == 0)) {
      iVar5 = DAT_0002baa0 + 0x2b97c;
      iVar8 = DAT_0002baa4 + 0x2b97e;
LAB_0002b956:
      __android_log_print(6,iVar5,iVar8);
      pthread_mutex_unlock(ppVar10);
      goto LAB_0002b962;
    }
    pthread_mutex_lock((pthread_mutex_t *)(this + 0x3a8));
    this[0x3a4] = (QualcommCameraHardware)0x1;
    pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3a8));
    pthread_mutex_lock((pthread_mutex_t *)(this + 0xd78));
    this[0xd74] = (QualcommCameraHardware)0x0;
    pthread_mutex_unlock((pthread_mutex_t *)(this + 0xd78));
    *(undefined4 *)(this + 0x10) = 0;
    pthread_attr_init((pthread_attr_t *)&stack0xffffffc4);
    pthread_attr_setdetachstate((pthread_attr_t *)&stack0xffffffc4,1);
    iVar5 = pthread_create((pthread_t *)(this + 0x5e4),(pthread_attr_t *)&stack0xffffffc4,
                           *(__start_routine **)(DAT_0002baa8 + 0x2b9ce),(void *)0x0);
    this[0x3ac] = (QualcommCameraHardware)(iVar5 == 0);
    pthread_mutex_unlock(ppVar10);
    pthread_mutex_lock((pthread_mutex_t *)(this + 0x3cc));
    this[0x3c8] = (QualcommCameraHardware)0x1;
    pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3cc));
    __android_log_print(6,DAT_0002baac + 0x2ba0a,DAT_0002bab0 + 0x2ba0c);
    if (this[0x3ac] == (QualcommCameraHardware)0x0) {
      uVar3 = 0x80000000;
    }
    else {
      uVar3 = 0;
    }
  }
LAB_0002ba1c:
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return uVar3;
}



// Function: initZslBuffers @ 0002bab4

/* android::QualcommCameraHardware::initZslBuffers(bool) */

bool __thiscall
android::QualcommCameraHardware::initZslBuffers(QualcommCameraHardware *this,bool param_1)

{
  int iVar1;
  int iVar2;
  int iVar3;
  uint uVar4;
  undefined1 auStack_2c [4];
  undefined4 local_28;
  undefined4 local_24;
  undefined4 local_20;
  undefined4 local_1c;
  
  __android_log_print(6,DAT_0002bc88 + 0x2bac4,DAT_0002bc84 + 0x2bac2);
  *(uint *)(this + 0xd54) = (uint)*(ushort *)(this + 0x440);
  *(uint *)(this + 0xd58) = (uint)*(ushort *)(this + 0x442);
  iVar2 = *(int *)(this + 0xd4c);
  iVar3 = *(int *)(this + 0xd50);
  *(uint *)(this + 0x42c) = iVar3 * iVar2 + 3U & 0xfffffffc;
  iVar1 = DAT_0002bc8c;
  *(int *)(this + 0x428) = (iVar3 * iVar2 * 3) / 2;
  iVar1 = *(int *)(iVar1 + 0x2bb06);
  if (iVar1 == 2) {
    uVar4 = (iVar2 + 0x1fU & 0xffffffe0) * (iVar3 + 0x1fU & 0xffffffe0) + 0xfff & 0xfffff000;
    *(uint *)(this + 0x42c) = uVar4;
    *(uint *)(this + 0x428) =
         uVar4 + (iVar3 / 2 + 0x1fU & 0xffffffe0) * (iVar2 / 2 + 0x1fU & 0xffffffe0) * 2;
  }
  iVar2 = *(int *)(DAT_0002bc90 + 0x2bb58);
  if ((iVar2 == 2) || (iVar2 == 3 || iVar2 == 1)) {
    iVar2 = (int)((*(int *)(this + 0xd50) + 0xfU & 0xfff0) *
                  (*(int *)(this + 0xd4c) + 0xfU & 0xfff0) * 3) >> 1;
  }
  else {
    iVar3 = *(int *)(this + 0xd4c);
    iVar2 = *(int *)(this + 0xd50);
    *(int *)(this + 0x430) = (iVar2 * iVar3 * 3) / 2;
    if (iVar1 != 2) goto LAB_0002bbe2;
    iVar2 = ((iVar3 + 0x1fU & 0xffffffe0) * (iVar2 + 0x1fU & 0xffffffe0) + 0xfff & 0xfffff000) +
            (iVar2 / 2 + 0x1fU & 0xffffffe0) * (iVar3 / 2 + 0x1fU & 0xffffffe0) * 2;
  }
  *(int *)(this + 0x430) = iVar2;
LAB_0002bbe2:
  local_20 = *(undefined4 *)(this + 0xd4c);
  local_1c = *(undefined4 *)(this + 0xd50);
  if (iVar1 != 2) {
    (**(code **)(DAT_0002bc94 + 0x2bc06))(0x26,auStack_2c);
    *(undefined4 *)(this + 0x428) = local_24;
    *(undefined4 *)(this + 0x430) = local_24;
    *(undefined4 *)(this + 0x42c) = local_28;
  }
  iVar2 = DAT_0002bc98 + 0x2bc1e;
  __android_log_print(2,iVar2,DAT_0002bc9c + 0x2bc20);
  iVar1 = createSnapshotMemory(this,5,5,param_1,1);
  if (iVar1 != 0) {
    initImageEncodeParameters(this,5);
    __android_log_print(2,iVar2,DAT_0002bca4 + 0x2bc58);
  }
  else {
    __android_log_print(6,iVar2,DAT_0002bca0 + 0x2bc40);
  }
  return iVar1 != 0;
}



// Function: startPreviewInternal @ 0002bca8

/* android::QualcommCameraHardware::startPreviewInternal() */

undefined4 android::QualcommCameraHardware::startPreviewInternal(void)

{
  int iVar1;
  QualcommCameraHardware QVar2;
  QualcommCameraHardware *in_r0;
  undefined4 uVar3;
  undefined1 uVar4;
  int iVar5;
  uint in_r3;
  int iVar6;
  pthread_mutex_t *__mutex;
  undefined4 *puVar7;
  undefined4 uStack_1c;
  
  iVar6 = DAT_0002bfbc + 0x2bcb6;
  uStack_1c = in_r3;
  __android_log_print(2,iVar6,DAT_0002bfc0 + 0x2bcba);
  if (in_r0[0x3e0] == (QualcommCameraHardware)0x0) {
    uVar3 = 6;
    iVar5 = DAT_0002bfc4 + 0x2bccc;
LAB_0002bcde:
    __android_log_print(uVar3,iVar6,iVar5);
    return 0;
  }
  in_r0[0xd85] = (QualcommCameraHardware)0x0;
  iVar5 = DAT_0002bfd0;
  if (in_r0[0x34] != (QualcommCameraHardware)0x0) {
    uVar3 = 2;
    iVar5 = DAT_0002bfc8 + 0x2bce0;
    goto LAB_0002bcde;
  }
  if (*(int *)(in_r0 + 0xd64) != 0) {
    __android_log_print(4,iVar6,DAT_0002bfcc + 0x2bcf8);
    uStack_1c = CONCAT13(1,(undefined3)uStack_1c);
    iVar5 = (**(code **)(iVar5 + 0x2bd02))(0x2a,(int)&uStack_1c + 3);
    if (iVar5 != 0) {
      iVar5 = DAT_0002bfd4 + 0x2bd12;
      goto LAB_0002bf76;
    }
  }
  if ((in_r0[0x698] != (QualcommCameraHardware)0x0) && (*(int *)(DAT_0002bfd8 + 0x2bd1e) == 0)) {
    iVar6 = DAT_0002bfdc + 0x2bd2a;
    __android_log_print(2,iVar6,DAT_0002bfe0 + 0x2bd2c);
    native_set_parms();
    FUN_0001f008(0x13,*(undefined4 *)(in_r0 + 0x690),*(undefined4 *)(in_r0 + 0x694));
    in_r0[0x698] = (QualcommCameraHardware)0x0;
    iVar5 = DAT_0002bfe4 + 0x2bd5e;
    __android_log_print(2,iVar6,iVar5,*(undefined4 *)(in_r0 + 0x690),*(undefined4 *)(in_r0 + 0x694))
    ;
    __android_log_print(2,iVar6,iVar5,*(undefined2 *)(in_r0 + 0x440),*(undefined2 *)(in_r0 + 0x442))
    ;
  }
  if (in_r0[0x3c] == (QualcommCameraHardware)0x0) {
    *(undefined4 *)(DAT_0002bfe8 + 0x2bd8c) = 0;
    iVar6 = initPreview(in_r0);
    in_r0[0x3c] = SUB41(iVar6,0);
    if (iVar6 == 0) {
      __android_log_print(6,DAT_0002bfec + 0x2bda2,DAT_0002bff0 + 0x2bda4);
      FrameQueue::deinit((FrameQueue *)(in_r0 + 0x330));
      return 0x80000000;
    }
  }
  if (*(int *)(DAT_0002bff4 + 0x2bdb6) == 1) {
    if (in_r0[0x6b9] != in_r0[0x6b8]) {
      if (in_r0[0x6b8] == (QualcommCameraHardware)0x0) {
        uVar4 = 0;
      }
      else {
        uVar4 = 0xb4;
      }
      FUN_0001f008(0x23,uVar4,0);
      in_r0[0x6b9] = in_r0[0x6b8];
    }
    if (*(int *)(in_r0 + 0x678) == 0) {
      iVar6 = android::CameraParameters::getInt((char *)(in_r0 + 0x18));
      FUN_0001f008(8,iVar6 + 4,0);
      setDropFrame(in_r0,3);
    }
    if (*(int *)(in_r0 + 0x69c) == 1) {
      iVar6 = 8;
      goto LAB_0002be18;
    }
  }
  else {
    iVar6 = 0;
LAB_0002be18:
    setDropFrame(in_r0,iVar6);
  }
  if ((in_r0[0xcec] == (QualcommCameraHardware)0x0) ||
     (startRecordingInternal(in_r0), in_r0[0x365] != (QualcommCameraHardware)0x0)) {
    iVar6 = DAT_0002c004;
    __mutex = (pthread_mutex_t *)(in_r0 + 0x38);
    pthread_mutex_lock(__mutex);
    iVar5 = DAT_0002c014;
    if (*(int *)(iVar6 + 0x2be48) - 4U < 3) {
      if (*(int *)(in_r0 + 0xd64) == 0) {
        iVar5 = DAT_0002c008 + 0x2be6c;
        __android_log_print(6,iVar5,DAT_0002c00c + 0x2be6e);
        uVar3 = FUN_0001ef80(2,0);
        iVar6 = DAT_0002c010 + 0x2be82;
        in_r0[0x34] = SUB41(uVar3,0);
        __android_log_print(6,iVar5,iVar6,uVar3);
      }
      else {
        initZslParameter();
        puVar7 = *(undefined4 **)(iVar5 + 0x2be9c);
        iVar6 = DAT_0002c018 + 0x2bea2;
        in_r0[0x34] = (QualcommCameraHardware)0x0;
        iVar6 = (*(code *)*puVar7)(1,iVar6);
        if (iVar6 == 0) {
          iVar6 = initZslBuffers(in_r0,true);
          if (iVar6 == 0) {
            __android_log_print(6,DAT_0002c01c + 0x2bec4,DAT_0002c020 + 0x2bec6);
            pthread_mutex_unlock(__mutex);
            return 0;
          }
          iVar6 = (*(code *)puVar7[1])(1,0,0);
          if (iVar6 == 0) {
            in_r0[0x34] = (QualcommCameraHardware)0x1;
          }
        }
        if (in_r0[0x34] == (QualcommCameraHardware)0x0) {
          __android_log_print(6,DAT_0002c024 + 0x2bef2,DAT_0002c028 + 0x2bef4);
        }
      }
    }
    else {
      QVar2 = (QualcommCameraHardware)FUN_0001ef80(0,0);
      in_r0[0x34] = QVar2;
    }
    pthread_mutex_unlock(__mutex);
    iVar5 = DAT_0002c054;
    iVar6 = DAT_0002c050;
    if (in_r0[0x34] != (QualcommCameraHardware)0x0) {
      puVar7 = (undefined4 *)(DAT_0002c04c + 0x2bf94);
      *(undefined4 *)(DAT_0002c048 + 0x2bf8e) = 0;
      *puVar7 = *(undefined4 *)(in_r0 + 0xd10);
      iVar1 = DAT_0002c058;
      *(undefined4 *)(iVar6 + 0x2bf9a) = *(undefined4 *)(in_r0 + 0xd14);
      __android_log_print(2,iVar5 + 0x2bfa0,iVar1 + 0x2bfb0);
      return 0;
    }
    deinitPreview(in_r0);
    if (*(int *)(in_r0 + 0xd64) != 0) {
      iVar6 = DAT_0002c02c + 0x2bf1a;
      __android_log_print(4,iVar6,DAT_0002c030 + 0x2bf1c);
      uStack_1c = uStack_1c & 0xffffff;
      iVar5 = (**(code **)(DAT_0002c034 + 0x2bf30))(0x2a,(int)&uStack_1c + 3);
      if (iVar5 != 0) {
        iVar5 = DAT_0002c038 + 0x2bf3c;
        goto LAB_0002bf76;
      }
    }
    iVar6 = DAT_0002c03c;
    cam_frame_flush_video();
    puVar7 = *(undefined4 **)(iVar6 + 0x2bf4c);
    (*(code *)*puVar7)(0);
    (*(code *)*puVar7)(2);
    in_r0[0x3c] = (QualcommCameraHardware)0x0;
    pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x408));
    pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x408));
    iVar6 = DAT_0002c040 + 0x2bf74;
    iVar5 = DAT_0002c044 + 0x2bf76;
  }
  else {
    iVar6 = DAT_0002bffc + 0x2be36;
    iVar5 = DAT_0002c000 + 0x2be38;
  }
LAB_0002bf76:
  __android_log_print(6,iVar6,iVar5);
  return 0x80000000;
}



// Function: startRecording @ 0002c05c

/* android::QualcommCameraHardware::startRecording() */

int __thiscall android::QualcommCameraHardware::startRecording(QualcommCameraHardware *this)

{
  int iVar1;
  undefined4 *puVar2;
  int iVar3;
  undefined4 uVar4;
  int iVar5;
  QualcommCameraHardware *pQVar6;
  int iVar7;
  int *piVar8;
  int iVar9;
  QualcommCameraHardware *pQVar10;
  int iVar11;
  int iVar12;
  int local_48;
  
  iVar11 = DAT_0002c324;
  iVar5 = DAT_0002c31c + 0x2c06a;
  __android_log_print(2,iVar5,DAT_0002c320 + 0x2c06e);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  this[0x3fd] = (QualcommCameraHardware)0x0;
  local_48 = startPreviewInternal();
  iVar11 = iVar11 + 0x2c094;
  if (local_48 == 0) {
    if (*(char *)(DAT_0002c328 + 0x2c0a2) != '\0') {
      __android_log_print(4,iVar5,DAT_0002c32c + 0x2c0b0);
      iVar1 = setVpeParameters();
      if (iVar1 != 0) {
        __android_log_print(6,iVar5,DAT_0002c330 + 0x2c0c4);
        local_48 = 1;
        goto LAB_0002c30e;
      }
    }
    if (*(int *)(DAT_0002c334 + 0x2c0d2) - 4U < 3) {
      piVar8 = (int *)(DAT_0002c338 + 0x2c0ee);
      iVar1 = DAT_0002c33c + 0x2c0f0;
      iVar12 = DAT_0002c340 + 0x2c0f4;
      pQVar6 = this;
      for (iVar5 = 0; iVar5 < *piVar8; iVar5 = iVar5 + 1) {
        if (*(int *)(this + 0x3ec) != 0) {
          __android_log_print(6,iVar1,iVar12);
          puVar2 = (undefined4 *)
                   (**(code **)(this + 0xd00))(0xffffffff,8,1,*(undefined4 *)(this + 0xd04));
          *(undefined4 **)(pQVar6 + 0x7b4) = puVar2;
          puVar2 = (undefined4 *)*puVar2;
          iVar3 = native_handle_create(1,2);
          puVar2[1] = iVar3;
          *puVar2 = 0;
          uVar4 = *(undefined4 *)(pQVar6 + 0x714);
          *(undefined4 *)(iVar3 + 0x10) = 0;
          *(undefined4 *)(iVar3 + 0xc) = uVar4;
          *(undefined4 *)(iVar3 + 0x14) = *(undefined4 *)(this + 0x424);
        }
        pQVar6 = pQVar6 + 4;
      }
      __android_log_print(2,DAT_0002c344 + 0x2c15a,DAT_0002c348 + 0x2c15c);
      FUN_0001ef80(10,0);
      if (*(int *)(this + 0x638) == 2) {
        *(undefined4 *)(this + 0x638) = 3;
        FUN_0001f008(0,3,0);
      }
      iVar1 = DAT_0002c354;
      iVar5 = DAT_0002c350;
      piVar8 = (int *)(DAT_0002c34c + 0x2c18a);
      *(undefined4 *)(this + 0xd94) = 1;
      __android_log_print(2,iVar5 + 0x2c190,iVar1 + 0x2c192,*piVar8);
      iVar5 = DAT_0002c358;
      while (iVar3 = DAT_0002c36c, iVar12 = DAT_0002c368, iVar1 = DAT_0002c364, 0 < *piVar8) {
        uVar4 = FUN_00029694();
        (*(code *)**(undefined4 **)(iVar11 + iVar5))(0,uVar4);
      }
      iVar7 = 0;
      __android_log_print(2,DAT_0002c35c + 0x2c1be,DAT_0002c360 + 0x2c1c4);
      iVar5 = DAT_0002c358;
      for (iVar9 = 0; iVar9 < *(int *)(iVar1 + 0x2c1d0); iVar9 = iVar9 + 1) {
        if (*(char *)(*(int *)(this + 0xcc8) + iVar9) != '\0') {
          __android_log_print(4,iVar12 + 0x2c1d2,iVar3 + 0x2c1d4,iVar9,
                              *(undefined4 *)(*(int *)(this + 0xcc0) + iVar7 + 0x10));
          (*(code *)**(undefined4 **)(iVar11 + iVar5))(0,*(int *)(this + 0xcc0) + iVar7);
          *(undefined1 *)(*(int *)(this + 0xcc8) + iVar9) = 0;
        }
        iVar7 = iVar7 + 0x68;
      }
      pthread_mutex_lock((pthread_mutex_t *)(this + 0x368));
      this[0x364] = (QualcommCameraHardware)0x0;
      pthread_attr_init((pthread_attr_t *)&stack0xffffffc0);
      pthread_attr_setdetachstate((pthread_attr_t *)&stack0xffffffc0,1);
      iVar11 = pthread_create((pthread_t *)(this + 0x5dc),(pthread_attr_t *)&stack0xffffffc0,
                              *(__start_routine **)(iVar11 + DAT_0002c370),(void *)0x0);
      this[0x365] = (QualcommCameraHardware)(iVar11 != 0);
      pthread_mutex_unlock((pthread_mutex_t *)(this + 0x368));
    }
    else if (*(int *)(DAT_0002c334 + 0x2c0d2) == 3) {
      pQVar10 = this + 0xa1c;
      iVar5 = DAT_0002c374 + 0x2c28c;
      iVar1 = DAT_0002c378 + 0x2c28e;
      pQVar6 = this;
      for (iVar11 = 0; iVar11 < *(int *)(this + 0xd5c); iVar11 = iVar11 + 1) {
        if (*(int *)(this + 0x3ec) != 0) {
          __android_log_print(6,iVar5,iVar1);
          puVar2 = (undefined4 *)
                   (**(code **)(this + 0xd00))(0xffffffff,8,1,*(undefined4 *)(this + 0xd04));
          *(undefined4 **)(pQVar6 + 0x7b4) = puVar2;
          puVar2 = (undefined4 *)*puVar2;
          iVar12 = native_handle_create(1,3);
          *puVar2 = 0;
          puVar2[1] = iVar12;
          uVar4 = *(undefined4 *)pQVar10;
          *(undefined4 *)(iVar12 + 0x10) = 0;
          *(undefined4 *)(iVar12 + 0xc) = uVar4;
          *(int *)(iVar12 + 0x14) = (*(int *)(this + 0xd14) * *(int *)(this + 0xd10) * 3) / 2;
          *(undefined4 *)(iVar12 + 0x18) = **(undefined4 **)(pQVar6 + 0x738);
        }
        pQVar6 = pQVar6 + 4;
        pQVar10 = pQVar10 + 0x68;
      }
    }
    *(undefined4 *)(DAT_0002c37c + 0x2c30c) = 1;
  }
LAB_0002c30e:
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return local_48;
}



// Function: getBuffersAndStartPreview @ 0002c380

/* android::QualcommCameraHardware::getBuffersAndStartPreview() */

int __thiscall
android::QualcommCameraHardware::getBuffersAndStartPreview(QualcommCameraHardware *this)

{
  QualcommCameraHardware QVar1;
  int iVar2;
  undefined4 uVar3;
  int iVar4;
  char *pcVar5;
  int iVar6;
  int iVar7;
  uint uVar8;
  int iVar9;
  int iVar10;
  int iVar11;
  uint uVar12;
  int iVar13;
  undefined4 uVar14;
  uint uVar15;
  int iVar16;
  undefined4 *puVar17;
  uint uVar18;
  int iVar19;
  int iVar20;
  int iVar21;
  pthread_mutex_t *__mutex;
  undefined4 uVar22;
  uint *puVar23;
  int iVar24;
  QualcommCameraHardware *pQVar25;
  pthread_mutex_t *__mutex_00;
  QualcommCameraHardware *pQVar26;
  QualcommCameraHardware *local_64;
  QualcommCameraHardware *local_60;
  int local_58;
  undefined4 local_40;
  int *local_34;
  uint local_30;
  undefined1 auStack_2c [8];
  
  iVar4 = DAT_0002c970;
  iVar11 = DAT_0002c96c;
  __mutex_00 = (pthread_mutex_t *)(this + 0x35c);
  __android_log_print(4,DAT_0002c960 + 0x2c390,DAT_0002c964 + 0x2c39a,DAT_0002c968 + 0x2c39c);
  iVar19 = DAT_0002c978;
  iVar21 = DAT_0002c974;
  pthread_mutex_lock(__mutex_00);
  while (iVar2 = DAT_0002c97c, this[0x358] != (QualcommCameraHardware)0x0) {
    __android_log_print(2,iVar11 + 0x2c3c2,iVar4 + 0x2c3c4,iVar21 + 0x2c3c6);
    pthread_cond_wait((pthread_cond_t *)(this + 0x360),__mutex_00);
    __android_log_print(2,iVar11 + 0x2c3c2,iVar19 + 0x2c3c8,iVar21 + 0x2c3c6);
  }
  __mutex = (pthread_mutex_t *)(this + 0x3b0);
  pthread_mutex_unlock(__mutex_00);
  iVar4 = DAT_0002c984;
  iVar11 = DAT_0002c980;
  pthread_mutex_lock(__mutex);
  while (QVar1 = this[0x3ac], (byte)QVar1 != 0) {
    __android_log_print(2,iVar2 + 0x2c414,iVar11 + 0x2c416);
    pthread_cond_wait((pthread_cond_t *)(this + 0x3b4),__mutex);
    __android_log_print(2,iVar2 + 0x2c414,iVar4 + 0x2c418);
  }
  pthread_mutex_unlock(__mutex);
  if (*(int *)(this + 0xcd0) == 0) {
    iVar11 = DAT_0002ca4c + 0x2c8f6;
    iVar4 = DAT_0002ca50 + 0x2c8f8;
    iVar21 = DAT_0002ca54 + 0x2c8fa;
LAB_0002c8f8:
    __android_log_print(6,iVar11,iVar4,iVar21);
    return -0x80000000;
  }
  iVar21 = DAT_0002c988 + 0x2c462;
  __android_log_print(2,iVar21,DAT_0002c98c + 0x2c464,DAT_0002c990 + 0x2c468);
  uVar3 = android::CameraParameters::getPreviewFormat();
  local_30 = (uint)(byte)QVar1;
  iVar4 = (**(code **)(*(int *)(this + 0xcd0) + 0x20))(*(int *)(this + 0xcd0),&local_30);
  iVar11 = DAT_0002c998;
  if (iVar4 == 0) {
    *(uint *)(this + 0xd5c) = local_30 + 4;
    iVar11 = FUN_0001f71c(iVar11 + 0x2c4ac,2,uVar3);
    if (iVar11 == -1) {
      iVar11 = 0x11;
    }
    if (*(int *)(this + 0xd64) == 0) {
      iVar4 = *(int *)(this + 0xc);
    }
    else {
      iVar4 = 3;
    }
    iVar4 = (**(code **)(*(int *)(this + 0xcd0) + 0xc))
                      (*(int *)(this + 0xcd0),iVar4 + *(int *)(this + 0xd5c));
    if (iVar4 == 0) {
      android::CameraParameters::getPreviewSize((int *)(this + 0x18),(int *)(this + 0xd10));
      iVar11 = (**(code **)(*(int *)(this + 0xcd0) + 0x10))
                         (*(int *)(this + 0xcd0),*(undefined4 *)(this + 0xd10),
                          *(undefined4 *)(this + 0xd14),iVar11);
      if (iVar11 != 0) {
        __android_log_print(6,DAT_0002c9a8 + 0x2c52a,DAT_0002c9ac + 0x2c52c,DAT_0002c9b0 + 0x2c52e);
        return iVar11;
      }
      (**(code **)(*(int *)(this + 0xcd0) + 0x18))(*(int *)(this + 0xcd0),0x8100000);
      uVar12 = *(int *)(this + 0xd10) * *(int *)(this + 0xd14) + 3U & 0xfffffffc;
      local_64 = this + 0xa14;
      iVar13 = (*(int *)(this + 0xd10) * *(int *)(this + 0xd14) * 3) / 2;
      uVar3 = *(undefined4 *)(this + 0xd5c);
      __android_log_print(6,DAT_0002c9b8 + 0x2c584,DAT_0002c9bc + 0x2c588,DAT_0002c9b4 + 0x2c55c,
                          uVar3);
      iVar2 = DAT_0002c9c0;
      local_60 = this + 0xa10;
      iVar6 = DAT_0002c9c4 + 0x2c5ae;
      iVar20 = DAT_0002c9c8 + 0x2c5b4;
      iVar10 = DAT_0002c9cc + 0x2c5b6;
      iVar16 = DAT_0002c9d0 + 0x2c5ba;
      iVar21 = 0;
      local_58 = 0;
      local_40 = 0;
      pQVar25 = this + 0xa00;
      pQVar26 = this;
      for (iVar19 = 0; iVar4 = DAT_0002ca2c, iVar11 = DAT_0002ca28, iVar19 < *(int *)(this + 0xd5c);
          iVar19 = iVar19 + 1) {
        local_34 = (int *)0x0;
        iVar4 = (*(code *)**(undefined4 **)(this + 0xcd0))
                          (*(undefined4 **)(this + 0xcd0),&local_34,auStack_2c);
        if (iVar4 != 0) {
          iVar21 = DAT_0002c9e0 + 0x2c614;
          iVar11 = DAT_0002c9e4 + 0x2c616;
          pcVar5 = (char *)(DAT_0002c9e8 + 0x2c61a);
LAB_0002c7ea:
          uVar3 = 6;
          goto LAB_0002c954;
        }
        iVar4 = (**(code **)(*(int *)(this + 0xcd0) + 0x24))(*(int *)(this + 0xcd0),local_34);
        iVar11 = genlock_lock_buffer(*local_34,2,1000);
        if (iVar11 != 0) {
          __android_log_print(6,DAT_0002c9d4 + 0x2c5fe,DAT_0002c9d8 + 0x2c600,DAT_0002c9dc + 0x2c604
                              ,iVar19);
          return -0x16;
        }
        if (iVar4 != 0) {
          iVar21 = DAT_0002ca10 + 0x2c7e6;
          iVar11 = DAT_0002ca14 + 0x2c7e8;
          pcVar5 = (char *)(DAT_0002ca18 + 0x2c7ec);
          iVar19 = iVar4;
          goto LAB_0002c7ea;
        }
        iVar4 = *local_34;
        uVar14 = *(undefined4 *)(iVar4 + 0xc);
        uVar3 = *(undefined4 *)(iVar4 + 0x28);
        uVar22 = *(undefined4 *)(iVar4 + 0x1c);
        __android_log_print(6,iVar20,iVar6,iVar4,uVar14,uVar3,uVar22);
        iVar11 = (**(code **)(this + 0xd00))
                           (*(undefined4 *)(iVar4 + 0xc),*(undefined4 *)(iVar4 + 0x1c),1,
                            *(undefined4 *)(this + 0xd04));
        *(int *)(pQVar26 + 0x738) = iVar11;
        if (iVar11 == 0) {
          __android_log_print(6,iVar20,iVar10,iVar19,uVar14,uVar3,uVar22);
        }
        puVar17 = *(undefined4 **)(pQVar26 + 0x738);
        iVar11 = iVar2 + 0x2c66c;
        uVar14 = puVar17[3];
        uVar3 = puVar17[1];
        __android_log_print(6,iVar11,iVar16,*puVar17,puVar17[2],uVar14,uVar3);
        *(undefined4 *)(this + iVar21 + 0xa1c) = *(undefined4 *)(iVar4 + 0xc);
        iVar7 = **(int **)(pQVar26 + 0x738);
        *(int *)pQVar25 = iVar7;
        iVar24 = DAT_0002c9fc;
        if (0xfffffffd < iVar7 - 1U) {
          iVar4 = DAT_0002c9ec + 0x2c6a8;
          iVar21 = DAT_0002c9f0 + 0x2c6aa;
          goto LAB_0002c8f8;
        }
        if ((*(int *)(DAT_0002c9f4 + 0x2c6b0) == 4) && (*(int *)(DAT_0002c9f8 + 0x2c6ba) != 3)) {
          uVar18 = *(int *)(this + 0xd10) * *(int *)(this + 0xd14) + 3U & 0xfffffffc;
          puVar23 = (uint *)(DAT_0002c9fc + 0x2c6d6);
          *puVar23 = uVar18;
          iVar7 = DAT_0002ca00;
          uVar15 = *(int *)(this + 0xd10) * *(int *)(this + 0xd14);
          uVar8 = uVar15 + 3 & (int)uVar15 >> 0x20;
          if (uVar15 < 0xfffffffd) {
            uVar8 = uVar15;
          }
          iVar9 = (((int)uVar8 >> 2) + 3U & 0xfffffffc) + uVar18;
          *(int *)(iVar24 + 0x2c6da) = iVar9;
          __android_log_print(6,iVar11,iVar7 + 0x2c702,uVar18,iVar9,uVar14,uVar3);
          *(undefined4 *)local_60 = 0;
          *(uint *)local_64 = *puVar23;
          *(undefined4 *)(this + iVar21 + 0xa18) = *(undefined4 *)(iVar24 + 0x2c6da);
          *(undefined4 *)(this + iVar21 + 0x9f8) = 1;
          local_40 = 1;
        }
        else {
          *(undefined4 *)local_60 = 0;
          *(uint *)local_64 = uVar12;
          *(undefined4 *)(this + iVar21 + 0xa18) = 0;
          *(undefined4 *)(this + iVar21 + 0x9f8) = 1;
        }
        iVar21 = iVar21 + 0x68;
        pQVar26 = pQVar26 + 4;
        *(QualcommCameraHardware **)(this + local_58 + 0xc60) = this + iVar19 * 0x68 + 0x9f0;
        *(int **)(this + local_58 + 0xc64) = local_34;
        uVar22 = *(undefined4 *)(iVar4 + 0x1c);
        *(undefined4 *)(this + local_58 + 0xc6c) = 1;
        iVar11 = DAT_0002ca04;
        *(undefined4 *)(this + local_58 + 0xc68) = uVar22;
        iVar24 = DAT_0002ca08 + 0x2c784;
        __android_log_print(6,iVar24,iVar11 + 0x2c780,iVar19,*(undefined4 *)(iVar4 + 0xc),uVar14,
                            uVar3);
        uVar3 = *(undefined4 *)(iVar4 + 0xc);
        FUN_0001f76c(iVar13,iVar13,uVar12,0,uVar3,0,*(undefined4 *)pQVar25,0xf,iVar19 < 3,1,local_40
                    );
        __android_log_print(6,iVar24,DAT_0002ca0c + 0x2c7c6);
        local_58 = local_58 + 0x10;
        local_60 = local_60 + 0x68;
        local_64 = local_64 + 0x68;
        pQVar25 = pQVar25 + 0x68;
      }
      pQVar25 = this + 0xcd4;
      __android_log_print(6,DAT_0002ca1c + 0x2c808,DAT_0002ca20 + 0x2c80c,DAT_0002ca24 + 0x2c810,
                          uVar3);
      iVar21 = 0;
      while( true ) {
        if (*(int *)(this + 0xd64) == 0) {
          iVar19 = *(int *)(this + 0xc);
        }
        else {
          iVar19 = 3;
        }
        if (iVar19 <= iVar21) break;
        iVar19 = (*(code *)**(undefined4 **)(this + 0xcd0))
                           (*(undefined4 **)(this + 0xcd0),this + (iVar21 + 0x336) * 4,auStack_2c);
        pQVar25 = pQVar25 + 4;
        __android_log_print(6,iVar11 + 0x2c820,iVar4 + 0x2c822,
                            *(undefined4 *)(**(int **)pQVar25 + 0xc));
        if (iVar19 != 0) {
          __android_log_print(6,iVar11 + 0x2c820,DAT_0002ca30 + 0x2c85c,DAT_0002ca34 + 0x2c862,
                              iVar19);
          return iVar19;
        }
        iVar21 = iVar21 + 1;
      }
      iVar4 = DAT_0002ca38 + 0x2c890;
      iVar21 = DAT_0002ca3c + 0x2c892;
      pQVar25 = this + 0xcac;
      for (iVar11 = 4; iVar11 < *(int *)(this + 0xd5c); iVar11 = iVar11 + 1) {
        iVar19 = genlock_unlock_buffer(**(undefined4 **)(pQVar25 + -8));
        if (iVar19 == 2) {
          __android_log_print(6,DAT_0002ca40 + 0x2c8ae,DAT_0002ca44 + 0x2c8b0,DAT_0002ca48 + 0x2c8b2
                              ,uVar3);
          return -0x16;
        }
        *(undefined4 *)pQVar25 = 0;
        (**(code **)(*(int *)(this + 0xcd0) + 8))
                  (*(int *)(this + 0xcd0),*(undefined4 *)(pQVar25 + -8));
        uVar3 = *(undefined4 *)(*(int *)(pQVar25 + -0xc) + 0x2c);
        __android_log_print(6,iVar4,iVar21,iVar11,uVar3);
        pQVar25 = pQVar25 + 0x10;
      }
      FrameQueue::init((FrameQueue *)(this + 0x330));
      (*(code *)**(undefined4 **)(DAT_0002ca58 + 0x2c914))(2);
      (*(code *)**(undefined4 **)(DAT_0002ca5c + 0x2c924))(2,this + 0xb28);
      this[0x3e0] = (QualcommCameraHardware)0x1;
      iVar21 = DAT_0002ca60 + 0x2c938;
      __android_log_print(6,iVar21,DAT_0002ca64 + 0x2c93a);
      iVar4 = startPreviewInternal();
      iVar11 = DAT_0002ca68 + 0x2c94c;
      pcVar5 = (char *)(DAT_0002ca6c + 0x2c94e);
      uVar3 = 4;
      iVar19 = iVar4;
    }
    else {
      iVar21 = DAT_0002c99c + 0x2c4ea;
      iVar11 = DAT_0002c9a0 + 0x2c4ee;
      pcVar5 = (char *)(DAT_0002c9a4 + 0x2c4f0);
      uVar3 = 6;
      iVar19 = 5;
    }
  }
  else {
    pcVar5 = strerror(-iVar4);
    iVar11 = DAT_0002c994 + 0x2c496;
    uVar3 = 5;
    iVar19 = -iVar4;
  }
LAB_0002c954:
  __android_log_print(uVar3,iVar21,iVar11,pcVar5,iVar19);
  return iVar4;
}



// Function: startPreview @ 0002ca70

/* android::QualcommCameraHardware::startPreview() */

undefined4 __thiscall android::QualcommCameraHardware::startPreview(QualcommCameraHardware *this)

{
  undefined4 uVar1;
  int iVar2;
  
  iVar2 = DAT_0002cadc + 0x2ca7c;
  __android_log_print(2,iVar2,DAT_0002cae0 + 0x2ca80);
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f0));
  if (*(int *)(this + 0xcd0) == 0) {
    __android_log_print(2,iVar2,DAT_0002cae4 + 0x2caa0,DAT_0002cae8 + 0x2caa2);
    uVar1 = startInitialPreview(this);
  }
  else {
    __android_log_print(2,iVar2,DAT_0002caec + 0x2cab8,DAT_0002caf0 + 0x2caba);
    uVar1 = getBuffersAndStartPreview(this);
  }
  __android_log_print(2,DAT_0002caf4 + 0x2cace,DAT_0002caf8 + 0x2cad0);
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f0));
  return uVar1;
}



// Function: setPreviewWindow @ 0002cafc

/* android::QualcommCameraHardware::setPreviewWindow(preview_stream_ops*) */

undefined4 __thiscall
android::QualcommCameraHardware::setPreviewWindow
          (QualcommCameraHardware *this,preview_stream_ops *param_1)

{
  undefined4 uVar1;
  int iVar2;
  
  iVar2 = DAT_0002cb6c + 0x2cb0a;
  __android_log_print(2,iVar2,DAT_0002cb70 + 0x2cb0e,DAT_0002cb74 + 0x2cb10);
  if (param_1 == (preview_stream_ops *)0x0) {
    __android_log_print(5,iVar2,DAT_0002cb78 + 0x2cb22);
  }
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f4));
  *(preview_stream_ops **)(this + 0xcd0) = param_1;
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f4));
  if ((*(int *)(this + 0xcd0) == 0) || (this[0x34] == (QualcommCameraHardware)0x0)) {
    uVar1 = 0;
  }
  else {
    stopInitialPreview(this);
    uVar1 = getBuffersAndStartPreview(this);
  }
  __android_log_print(2,DAT_0002cb7c + 0x2cb62,DAT_0002cb80 + 0x2cb64,DAT_0002cb84 + 0x2cb66);
  return uVar1;
}



// Function: runHFRThread @ 0002cb88

/* android::QualcommCameraHardware::runHFRThread(void*) */

void android::QualcommCameraHardware::runHFRThread(void *param_1)

{
  int iVar1;
  undefined4 uVar2;
  int iVar3;
  int iVar4;
  int iVar5;
  int *piVar6;
  int iVar7;
  int iVar8;
  int iVar9;
  int iVar10;
  int iVar11;
  int iVar12;
  
  iVar5 = DAT_0002cd84 + 0x2cb96;
  __android_log_print(3,iVar5,DAT_0002cd88 + 0x2cb9a);
  iVar7 = DAT_0002cd90;
  iVar3 = DAT_0002cd8c + 0x2cbaa;
  *(undefined1 *)((int)param_1 + 0xd86) = 1;
  __android_log_print(4,iVar5,iVar3,iVar7 + 0x2cbb0);
  stopPreviewInternal();
  iVar7 = DAT_0002cd94;
  if (*(int *)((int)param_1 + 0xcd0) != 0) {
    piVar6 = (int *)((int)param_1 + 0x6d0);
    iVar11 = 0;
    iVar8 = DAT_0002cd98 + 0x2cbe0;
    iVar5 = DAT_0002cd9c + 0x2cbe4;
    iVar10 = DAT_0002cda0 + 0x2cbe6;
    iVar3 = DAT_0002cda4 + 0x2cbea;
    while( true ) {
      if (*(int *)((int)param_1 + 0xd64) == 0) {
        iVar4 = *(int *)((int)param_1 + 0xc);
      }
      else {
        iVar4 = 3;
      }
      if (iVar4 <= iVar11) break;
      if ((*(int *)((int)param_1 + 0xcd0) != 0) && ((int *)piVar6[0x182] != (int *)0x0)) {
        iVar4 = *(int *)piVar6[0x182];
        uVar2 = *(undefined4 *)(iVar4 + 0xc);
        __android_log_print(2,iVar8,iVar5,iVar10,uVar2);
        __android_log_print(2,iVar8,iVar3);
        pthread_mutex_lock((pthread_mutex_t *)((int)param_1 + 0x3f4));
        if (*piVar6 == 1) {
          iVar1 = genlock_unlock_buffer(iVar4);
          if (iVar1 != 2) {
            *piVar6 = 0;
            goto LAB_0002cc48;
          }
          __android_log_print(6,iVar8,DAT_0002cda8 + 0x2cc40,iVar10,uVar2);
        }
        else {
LAB_0002cc48:
          iVar1 = (**(code **)(*(int *)((int)param_1 + 0xcd0) + 8))
                            (*(int *)((int)param_1 + 0xcd0),piVar6[0x182]);
          if (iVar1 != 0) {
            __android_log_print(6,DAT_0002cdac + 0x2cc62,DAT_0002cdb0 + 0x2cc66,
                                DAT_0002cdb4 + 0x2cc68,*(undefined4 *)(iVar4 + 0xc));
          }
          if ((piVar6[-5] != 0) && (*(int *)((int)param_1 + 0x3e4) == 1)) {
            iVar12 = *(int *)((int)param_1 + 0xd10) * *(int *)((int)param_1 + 0xd14);
            iVar9 = DAT_0002cdb8 + 0x2cc98;
            __android_log_print(6,iVar9,DAT_0002cdbc + 0x2cc9a,DAT_0002cdc0 + 0x2cc9e,
                                *(undefined4 *)(iVar4 + 0xc));
            iVar1 = (iVar12 * 3) / 2;
            FUN_0001f76c(iVar1,iVar1,iVar12 + 3U & 0xfffffffc,0,*(undefined4 *)(iVar4 + 0xc),0,
                         piVar6[-5],3,0,0,0);
            iVar4 = munmap((void *)piVar6[-5],*(size_t *)(iVar4 + 0x1c));
            if (iVar4 == -1) {
              __android_log_print(6,iVar9,DAT_0002cdc4 + 0x2ccf4,
                                  *(undefined4 *)(DAT_0002cdc8 + 0x2ccf6));
            }
            piVar6[0x182] = 0;
            piVar6[-5] = 0;
          }
          __android_log_print(2,iVar7 + 0x2cd0e,DAT_0002cdcc + 0x2cd10);
        }
        pthread_mutex_unlock((pthread_mutex_t *)((int)param_1 + 0x3f4));
      }
      iVar11 = iVar11 + 1;
      piVar6 = piVar6 + 1;
    }
  }
  iVar3 = DAT_0002cdd0 + 0x2cd3c;
  iVar7 = DAT_0002cdd4 + 0x2cd3e;
  __android_log_print(2,iVar3,DAT_0002cdd8 + 0x2cd42,iVar7);
  (**(code **)(*(int *)param_1 + 0x5c))(param_1,(int)param_1 + 0x18);
  __android_log_print(2,iVar3,DAT_0002cddc + 0x2cd5c,iVar7);
  if (*(int *)((int)param_1 + 0xcd0) == 0) {
    startPreviewInternal();
  }
  else {
    getBuffersAndStartPreview(param_1);
  }
  *(undefined1 *)((int)param_1 + 0xd75) = 0;
  *(undefined1 *)((int)param_1 + 0xd86) = 0;
  return;
}



// Function: hfr_thread @ 0002cde0

/* android::hfr_thread(void*) */

undefined4 android::hfr_thread(void *param_1)

{
  void *pvVar1;
  int iVar2;
  
  iVar2 = DAT_0002ce20 + 0x2cdec;
  __android_log_print(4,iVar2,DAT_0002ce24 + 0x2cdf0);
  pvVar1 = (void *)QualcommCameraHardware::getInstance();
  if (pvVar1 == (void *)0x0) {
    __android_log_print(6,iVar2,DAT_0002ce28 + 0x2ce0c);
  }
  else {
    QualcommCameraHardware::runHFRThread(pvVar1);
  }
  __android_log_print(4,DAT_0002ce2c + 0x2ce18,DAT_0002ce30 + 0x2ce1a);
  return 0;
}



// Function: FUN_0002ce34 @ 0002ce34

undefined4 * FUN_0002ce34(undefined4 *param_1)

{
  if ((void *)*param_1 != (void *)0x0) {
    android::RefBase::decStrong((void *)*param_1);
  }
  *param_1 = 0;
  return param_1;
}



// Function: initDefaultParameters @ 0002ce4c

/* android::QualcommCameraHardware::initDefaultParameters() */

void __thiscall android::QualcommCameraHardware::initDefaultParameters(QualcommCameraHardware *this)

{
  int iVar1;
  int iVar2;
  char cVar3;
  String8 *pSVar4;
  char *pcVar5;
  undefined2 uVar6;
  undefined4 uVar7;
  int iVar8;
  undefined4 *puVar9;
  int iVar10;
  QualcommCameraHardware *pQVar11;
  int iVar12;
  String8 *pSVar13;
  float fVar14;
  char acStack_dc [8];
  String8 aSStack_d4 [4];
  String8 aSStack_d0 [4];
  String8 aSStack_cc [4];
  String8 local_c8 [4];
  String8 local_c4 [4];
  String8 aSStack_c0 [4];
  String8 aSStack_bc [4];
  String8 aSStack_b8 [4];
  String8 aSStack_b4 [4];
  String8 aSStack_b0 [4];
  String8 aSStack_ac [4];
  String8 aSStack_a8 [4];
  String8 aSStack_a4 [4];
  String8 aSStack_a0 [4];
  String8 aSStack_9c [4];
  String8 aSStack_98 [4];
  String8 aSStack_94 [4];
  String8 aSStack_90 [4];
  String8 aSStack_8c [4];
  String8 aSStack_88 [4];
  String8 aSStack_84 [4];
  String8 aSStack_80 [4];
  String8 aSStack_7c [4];
  String8 aSStack_78 [4];
  String8 aSStack_74 [4];
  String8 aSStack_70 [4];
  String8 aSStack_6c [4];
  String8 aSStack_68 [4];
  String8 aSStack_64 [4];
  String8 aSStack_60 [4];
  String8 aSStack_5c [4];
  String8 aSStack_58 [4];
  String8 aSStack_54 [4];
  String8 aSStack_50 [4];
  String8 aSStack_4c [4];
  String8 aSStack_48 [4];
  char acStack_44 [32];
  int local_24;
  
  iVar10 = DAT_0002d984 + 0x2ce5e;
  local_24 = **(int **)(iVar10 + DAT_0002d988);
  __android_log_print(4,DAT_0002d990 + 0x2ce70,DAT_0002d98c + 0x2ce68);
  if (*(int *)(DAT_0002d994 + 0x2ce7e) == 0) {
    *(undefined2 *)(this + 0x43c) = 0xcc0;
    *(undefined2 *)(this + 0x43e) = 0x990;
    *(undefined4 *)(this + 0x3e4) = 2;
  }
  else {
    *(undefined4 *)(this + 0x3e4) = 1;
    *(undefined2 *)(this + 0x43c) = 0x640;
    *(undefined2 *)(this + 0x43e) = 0x4b0;
  }
  *(undefined4 *)(this + 0x67c) = 0;
  *(undefined2 *)(this + 0x44c) = 0x180;
  *(undefined2 *)(this + 0x44e) = 0x200;
  hasAutoFocusSupport(this);
  cVar3 = (**(code **)(DAT_0002d998 + 0x2cee0))(0x1b);
  if (cVar3 == '\0') {
    __android_log_print(2,DAT_0002d99c + 0x2ceea,DAT_0002d9a0 + 0x2ceec);
    this[0xd34] = (QualcommCameraHardware)0x0;
  }
  else {
    __android_log_print(2,DAT_0002d9a4 + 0x2cf02,DAT_0002d9a8 + 0x2cf04);
  }
  if (*(char *)(DAT_0002d9ac + 0x2cf0e) == '\0') {
    if (this[0xcec] == (QualcommCameraHardware)0x0) {
      pSVar13 = aSStack_4c;
      FUN_0001f400(pSVar13,DAT_0002d9b8 + 0x2cf40,4);
      pSVar4 = (String8 *)(DAT_0002d9bc + 0x2cf4a);
    }
    else {
      pSVar13 = aSStack_48;
      FUN_0001f400(pSVar13,DAT_0002d9b0 + 0x2cf28,3);
      pSVar4 = (String8 *)(DAT_0002d9b4 + 0x2cf32);
    }
    android::String8::setTo(pSVar4);
    android::String8::~String8(pSVar13);
    FUN_0001f400(aSStack_50,DAT_0002d9c0 + 0x2cf62,5);
    android::String8::setTo((String8 *)(DAT_0002d9c4 + 0x2cf74));
    android::String8::~String8(aSStack_50);
    FUN_0001f400(aSStack_54,DAT_0002d9c8 + 0x2cf8c,3);
    android::String8::setTo((String8 *)(DAT_0002d9cc + 0x2cf9a));
    android::String8::~String8(aSStack_54);
    FUN_0001f400(aSStack_58,DAT_0002d9d0 + 0x2cfae,5);
    android::String8::setTo((String8 *)(DAT_0002d9d4 + 0x2cfbc));
    android::String8::~String8(aSStack_58);
    filterPictureSizes(this);
    iVar8 = DAT_0002d9e0;
    FUN_00026b38(aSStack_5c,*(undefined4 *)(DAT_0002d9d8 + 0x2cfd8),
                 *(undefined4 *)(DAT_0002d9dc + 0x2cfe0));
    android::String8::setTo((String8 *)(DAT_0002d9e4 + 0x2cff0));
    android::String8::~String8(aSStack_5c);
    filterPreviewSizes();
    iVar12 = DAT_0002d9f0;
    FUN_00026b38(aSStack_60,*(undefined4 *)(DAT_0002d9e8 + 0x2d00c),
                 *(undefined4 *)(DAT_0002d9ec + 0x2d014));
    android::String8::setTo((String8 *)(iVar8 + 0x2cfee));
    android::String8::~String8(aSStack_60);
    filterVideoSizes();
    FUN_00026b38(aSStack_64,*(undefined4 *)(DAT_0002d9f4 + 0x2d03a),
                 *(undefined4 *)(DAT_0002d9f8 + 0x2d03e));
    android::String8::setTo((String8 *)(DAT_0002d9fc + 0x2d04c));
    android::String8::~String8(aSStack_64);
    iVar8 = DAT_0002da04;
    pQVar11 = this + 0x18;
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002da00));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar12));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002da14));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002da18));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002da1c));
    if (this[0xcec] == (QualcommCameraHardware)0x0) {
      FUN_00026b38(aSStack_68,*(undefined4 *)(DAT_0002da20 + 0x2d0d4));
      android::String8::setTo((String8 *)(DAT_0002da24 + 0x2d0e2));
      android::String8::~String8(aSStack_68);
    }
    iVar8 = DAT_0002da28;
    android::String8::String8(aSStack_6c);
    sprintf(acStack_44,(char *)(DAT_0002da30 + 0x2d10c),*(undefined4 *)(DAT_0002da2c + 0x2d108),
            *(undefined4 *)(DAT_0002da2c + 0x2d10c));
    android::String8::append((char *)aSStack_6c);
    android::String8::setTo((String8 *)(iVar8 + 0x2d10a));
    android::String8::~String8(aSStack_6c);
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002da34));
    android::CameraParameters::setPreviewFpsRange((int)pQVar11,5000);
    FUN_0001f400(aSStack_70,DAT_0002da38 + 0x2d158,4);
    android::String8::setTo((String8 *)(DAT_0002da3c + 0x2d164));
    android::String8::~String8(aSStack_70);
    if (this[0xd28] != (QualcommCameraHardware)0x0) {
      FUN_0001f400(aSStack_74,DAT_0002da40 + 0x2d180,6);
      android::String8::setTo((String8 *)(DAT_0002da44 + 0x2d18c));
      android::String8::~String8(aSStack_74);
    }
    if (this[0xcec] == (QualcommCameraHardware)0x0) {
      pSVar13 = aSStack_7c;
      FUN_0001f400(pSVar13,DAT_0002da50 + 0x2d1c0,7);
      pSVar4 = (String8 *)(DAT_0002da54 + 0x2d1ca);
    }
    else {
      pSVar13 = aSStack_78;
      FUN_0001f400(pSVar13,DAT_0002da48 + 0x2d1a8,6);
      pSVar4 = (String8 *)(DAT_0002da4c + 0x2d1b2);
    }
    android::String8::setTo(pSVar4);
    android::String8::~String8(pSVar13);
    FUN_0001f400(aSStack_80,DAT_0002da58 + 0x2d1e2,2);
    android::String8::setTo((String8 *)(DAT_0002da5c + 0x2d1f2));
    android::String8::~String8(aSStack_80);
    FUN_0001f400(aSStack_84,DAT_0002da60 + 0x2d206,2);
    android::String8::setTo((String8 *)(DAT_0002da64 + 0x2d212));
    android::String8::~String8(aSStack_84);
    if (this[0xcec] == (QualcommCameraHardware)0x0) {
      FUN_0001f400(aSStack_88,DAT_0002da68 + 0x2d230,4);
      android::String8::setTo((String8 *)(DAT_0002da6c + 0x2d23c));
      android::String8::~String8(aSStack_88);
    }
    if (*(int *)(DAT_0002da70 + 0x2d24c) == 6) {
      FUN_0001f400(aSStack_8c,DAT_0002da74 + 0x2d25e,2);
      android::String8::setTo((String8 *)(DAT_0002da78 + 0x2d26a));
      android::String8::~String8(aSStack_8c);
    }
    if (*(int *)(DAT_0002da7c + 0x2d27a) == 6) {
      FUN_0001f400(aSStack_90,DAT_0002da80 + 0x2d28e,2);
      android::String8::setTo((String8 *)(DAT_0002da84 + 0x2d29a));
      android::String8::~String8(aSStack_90);
    }
    if (*(int *)(DAT_0002da88 + 0x2d2aa) - 5U < 2) {
      FUN_0001f400(aSStack_94,DAT_0002da8c + 0x2d2be,2);
      android::String8::setTo((String8 *)(DAT_0002da90 + 0x2d2ca));
      android::String8::~String8(aSStack_94);
    }
    if (this[0xd28] != (QualcommCameraHardware)0x0) {
      FUN_0001f400(aSStack_98,DAT_0002da94 + 0x2d2e8,2);
      android::String8::setTo((String8 *)(DAT_0002da98 + 0x2d2f4));
      android::String8::~String8(aSStack_98);
    }
    FUN_0001f400(aSStack_9c,DAT_0002da9c + 0x2d30c,2);
    android::String8::setTo((String8 *)(DAT_0002daa0 + 0x2d318));
    android::String8::~String8(aSStack_9c);
    if (*(int *)(this + 0xd64) == 0) {
      pSVar13 = aSStack_a4;
      FUN_0001f400(pSVar13,DAT_0002daac + 0x2d34c,2);
      pSVar4 = (String8 *)(DAT_0002dab0 + 0x2d356);
    }
    else {
      pSVar13 = aSStack_a0;
      FUN_0001f400(pSVar13,DAT_0002daa4 + 0x2d334,1);
      pSVar4 = (String8 *)(DAT_0002daa8 + 0x2d33e);
    }
    android::String8::setTo(pSVar4);
    android::String8::~String8(pSVar13);
    iVar8 = *(int *)(DAT_0002dab4 + 0x2d368);
    if ((iVar8 == 6) || (iVar8 == 3 || iVar8 == 1)) {
      FUN_0001f400(aSStack_a8,DAT_0002dab8 + 0x2d392,2);
      android::String8::setTo((String8 *)(DAT_0002dabc + 0x2d39e));
      android::String8::~String8(aSStack_a8);
    }
    if (*(int *)(DAT_0002dac0 + 0x2d3ae) == 1) {
      uVar7 = 0;
      puVar9 = (undefined4 *)(DAT_0002dac8 + 0x2d3c2);
      *(undefined1 *)(DAT_0002dac4 + 0x2d3c0) = 0;
    }
    else {
      puVar9 = (undefined4 *)(DAT_0002dad0 + 0x2d3d6);
      *(undefined1 *)(DAT_0002dacc + 0x2d3d4) = 1;
      uVar7 = 0x1e;
    }
    *puVar9 = uVar7;
    if (*(int *)(DAT_0002dad4 + 0x2d3e2) == 0) {
      iVar8 = DAT_0002dadc + 0x2d3f4;
    }
    else {
      iVar8 = DAT_0002dad8 + 0x2d3ec;
    }
    __android_log_print(2,DAT_0002dae8 + 0x2d414,DAT_0002daec + 0x2d41a,iVar8,
                        *(undefined4 *)(DAT_0002dae4 + 0x2d40a),
                        *(undefined1 *)(DAT_0002dae0 + 0x2d404));
    iVar12 = 6;
    android::String8::String8(aSStack_ac);
    snprintf(acStack_44,0x20,(char *)(DAT_0002daf0 + 0x2d436),5);
    iVar8 = DAT_0002daf4;
    android::String8::append((char *)aSStack_ac);
    do {
      snprintf(acStack_44,0x20,(char *)(iVar8 + 0x2d44a),iVar12);
      iVar12 = iVar12 + 1;
      android::String8::append((char *)aSStack_ac);
    } while (iVar12 != 0x20);
    android::String8::setTo((String8 *)(DAT_0002daf8 + 0x2d46c));
    android::String8::~String8(aSStack_ac);
    FUN_0001f400(aSStack_b0,DAT_0002dafc + 0x2d482,0xf);
    android::String8::setTo((String8 *)(DAT_0002db00 + 0x2d490));
    android::String8::~String8(aSStack_b0);
    iVar8 = supportsSceneDetection();
    if (iVar8 != 0) {
      FUN_0001f400(aSStack_b4,DAT_0002db04 + 0x2d4ae,2);
      android::String8::setTo((String8 *)(DAT_0002db08 + 0x2d4ba));
      android::String8::~String8(aSStack_b4);
    }
    if ((this[0xd28] != (QualcommCameraHardware)0x0) &&
       (iVar8 = supportsSelectableZoneAf(), iVar8 != 0)) {
      FUN_0001f400(aSStack_b8,DAT_0002db0c + 0x2d4de,4);
      android::String8::setTo((String8 *)(DAT_0002db10 + 0x2d4ea));
      android::String8::~String8(aSStack_b8);
    }
    if ((this[0xd28] != (QualcommCameraHardware)0x0) &&
       (iVar8 = supportsFaceDetection(), iVar8 != 0)) {
      FUN_0001f400(aSStack_bc,DAT_0002db14 + 0x2d510,2);
      android::String8::setTo((String8 *)(DAT_0002db18 + 0x2d51c));
      android::String8::~String8(aSStack_bc);
    }
    FUN_0001f400(aSStack_c0,DAT_0002db1c + 0x2d534,2);
    android::String8::setTo((String8 *)(DAT_0002db20 + 0x2d540));
    android::String8::~String8(aSStack_c0);
    *(undefined1 *)(DAT_0002db24 + 0x2d552) = 1;
  }
  if (this[0xcec] == (QualcommCameraHardware)0x0) {
    android::CameraParameters::setPreviewSize((int)(this + 0x18),0x280);
    uVar6 = 0x1e0;
    *(undefined2 *)(this + 0x440) = 0x280;
  }
  else {
    __android_log_print(6,DAT_0002db28 + 0x2d570,DAT_0002db2c + 0x2d572);
    android::CameraParameters::setPreviewSize((int)(this + 0x18),0x500);
    *(undefined2 *)(this + 0x440) = 0x500;
    uVar6 = 0x2d0;
  }
  pQVar11 = this + 0x18;
  *(undefined2 *)(this + 0x442) = uVar6;
  android::CameraParameters::setPreviewFrameRateMode((char *)pQVar11);
  android::CameraParameters::setPreviewFormat((char *)pQVar11);
  iVar12 = DAT_0002db40;
  iVar8 = DAT_0002db3c;
  android::CameraParameters::setPictureFormat((char *)pQVar11);
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002db44));
  iVar1 = DAT_0002db4c;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
  android::CameraParameters::set((char *)pQVar11,(char *)(DAT_0002db50 + 0x2d60e));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002db58));
  iVar2 = DAT_0002db64;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar12));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002db68));
  iVar12 = DAT_0002db70;
  *(undefined2 *)(this + 0x44c) = 0x180;
  *(undefined2 *)(this + 0x44e) = 0x200;
  iVar8 = DAT_0002da34;
  if (*(int *)(iVar12 + 0x2d660) == 0) {
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002db74));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
    pSVar13 = local_c4;
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar1));
    android::CameraParameters::setPreviewFrameRate((int)pQVar11);
    android::CameraParameters::setPictureSize((int)pQVar11,0xcc0);
    FUN_00026b38(pSVar13,DAT_0002db84 + 0x2d6ba,4);
    pcVar5 = *(char **)(iVar10 + iVar2);
  }
  else {
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002db74));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
    pSVar13 = local_c8;
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar1));
    android::CameraParameters::setPreviewFrameRate((int)pQVar11);
    android::CameraParameters::setPictureSize((int)pQVar11,0x640);
    FUN_00026b38(pSVar13,DAT_0002db94 + 0x2d716,3);
    pcVar5 = *(char **)(iVar10 + iVar2);
  }
  android::CameraParameters::set((char *)pQVar11,pcVar5);
  android::String8::~String8(pSVar13);
  iVar12 = DAT_0002dba4;
  iVar8 = DAT_0002db9c;
  if (*(char *)(DAT_0002db98 + 0x2d736) == '\0') {
    pcVar5 = *(char **)(iVar10 + DAT_0002db9c);
  }
  else {
    android::CameraParameters::set((char *)pQVar11,*(int *)(iVar10 + DAT_0002dba0));
    android::CameraParameters::set((char *)pQVar11,*(int *)(iVar10 + iVar12));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002dba8));
    pcVar5 = *(char **)(iVar10 + iVar8);
  }
  iVar8 = DAT_0002dbb8;
  android::CameraParameters::set((char *)pQVar11,pcVar5);
  if ((*(char *)(iVar8 + 0x2d78c) == '\0') || (*(char *)(DAT_0002dbbc + 0x2d796) == '\0')) {
    pcVar5 = (char *)(DAT_0002dbc8 + 0x2d7b4);
  }
  else {
    pcVar5 = (char *)(DAT_0002dbc0 + 0x2d7a6);
  }
  android::CameraParameters::set((char *)pQVar11,pcVar5);
  android::CameraParameters::set((char *)pQVar11,*(int *)(iVar10 + DAT_0002dbd4));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002dbe0));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002dbe4));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002dbe8));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002dbec));
  iVar1 = DAT_0002dc0c;
  iVar12 = DAT_0002dc04;
  iVar8 = DAT_0002dbf8;
  if (*(int *)(DAT_0002dbf4 + 0x2d818) - 3U < 4) {
    if (*(int *)(DAT_0002dbf4 + 0x2d818) - 2U < 2) {
      pSVar13 = aSStack_cc;
      FUN_0001f400(pSVar13,DAT_0002dc00 + 0x2d846,2);
      pSVar4 = (String8 *)(iVar12 + 0x2d84e);
    }
    else {
      pSVar13 = aSStack_d0;
      FUN_0001f400(pSVar13,DAT_0002dc08 + 0x2d85a,3);
      pSVar4 = (String8 *)(iVar1 + 0x2d864);
    }
    android::String8::setTo(pSVar4);
    android::String8::~String8(pSVar13);
    pcVar5 = *(char **)(iVar10 + iVar8);
  }
  else {
    pcVar5 = *(char **)(iVar10 + DAT_0002dbf8);
  }
  android::CameraParameters::set((char *)pQVar11,pcVar5);
  pSVar13 = (String8 *)(DAT_0002dc10 + 0x2d892);
  FUN_0001f400(aSStack_d4,DAT_0002dc14 + 0x2d890,2);
  android::String8::setTo(pSVar13);
  android::String8::~String8(aSStack_d4);
  cVar3 = (**(code **)(DAT_0002dc18 + 0x2d8b4))(5);
  if (cVar3 != '\0') {
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002dc1c));
  }
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002da00));
  iVar8 = DAT_0002dc34;
  if (*(int *)(DAT_0002dc24 + 0x2d8dc) == 0) {
    pcVar5 = *(char **)(iVar10 + DAT_0002dc28);
  }
  else {
    pcVar5 = *(char **)(iVar10 + DAT_0002dc28);
  }
  android::CameraParameters::set((char *)pQVar11,pcVar5);
  iVar1 = DAT_0002dc40;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002dc38));
  iVar12 = DAT_0002da18;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002dc48));
  iVar8 = DAT_0002dc54;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar1));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002dc58));
  if (this[0xd28] == (QualcommCameraHardware)0x0) {
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002da14));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar12));
    pcVar5 = *(char **)(iVar10 + iVar8);
  }
  else {
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002da14));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar12));
    pcVar5 = *(char **)(iVar10 + iVar8);
  }
  android::CameraParameters::set((char *)pQVar11,pcVar5);
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e0f8));
  iVar8 = DAT_0002e10c;
  if (*(int *)(DAT_0002e100 + 0x2dcb6) == 0) {
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e104));
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
  }
  iVar8 = DAT_0002e118;
  android::CameraParameters::set((char *)pQVar11,*(int *)(iVar10 + DAT_0002e114));
  android::CameraParameters::set((char *)pQVar11,*(int *)(iVar10 + DAT_0002e120));
  android::CameraParameters::set((char *)pQVar11,*(int *)(iVar10 + DAT_0002e124));
  android::CameraParameters::set((char *)pQVar11,*(int *)(iVar10 + DAT_0002e128));
  android::CameraParameters::set((char *)pQVar11,*(int *)(iVar10 + DAT_0002e12c));
  fVar14 = (float)android::CameraParameters::set((char *)pQVar11,*(int *)(iVar10 + iVar8));
  android::CameraParameters::setFloat((char *)pQVar11,fVar14);
  iVar8 = DAT_0002e13c;
  android::CameraParameters::set((char *)pQVar11,(char *)(DAT_0002e130 + 0x2dd54));
  android::CameraParameters::set((char *)pQVar11,(char *)(DAT_0002e140 + 0x2dd6a));
  android::CameraParameters::set((char *)pQVar11,DAT_0002e144 + 0x2dd76);
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e14c));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e150));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e160));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e168));
  iVar8 = DAT_0002e174;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e170));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e178));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e184));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e188));
  iVar12 = DAT_0002e194;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e198));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e1a0));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar12));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e1b4));
  android::CameraParameters::set((char *)pQVar11,(char *)(DAT_0002e1b8 + 0x2de76));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e1c0));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e1c8));
  iVar8 = DAT_0002e1d4;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e1cc));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e1d8));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
  android::CameraParameters::set((char *)pQVar11,(char *)(DAT_0002e1e8 + 0x2dedc));
  android::CameraParameters::set((char *)pQVar11,(char *)(DAT_0002e1ec + 0x2deea));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e1f0));
  iVar8 = DAT_0002e1f8;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e1f4));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e1fc));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e210));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e214));
  iVar8 = DAT_0002e220;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e21c));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e224));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e230));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e234));
  iVar12 = DAT_0002e240;
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + iVar8));
  android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e244));
  if (*(int *)(DAT_0002e24c + 0x2dfb4) == 0) {
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e250));
    pcVar5 = *(char **)(iVar10 + iVar12);
  }
  else {
    android::CameraParameters::set((char *)pQVar11,*(char **)(iVar10 + DAT_0002e250));
    pcVar5 = *(char **)(iVar10 + iVar12);
  }
  android::CameraParameters::set((char *)pQVar11,pcVar5);
  *(undefined4 *)(this + 0xc) = 1;
  if (*(int *)(this + 0xd64) != 0) {
    property_get(DAT_0002e264 + 0x2dffa,acStack_dc,DAT_0002e268 + 0x2dffc);
    iVar8 = atoi(acStack_dc);
    *(int *)(this + 0xc) = iVar8;
    if (iVar8 < 4) {
      if (iVar8 < 1) {
        *(undefined4 *)(this + 0xc) = 1;
      }
    }
    else {
      *(undefined4 *)(this + 0xc) = 3;
    }
    android::CameraParameters::set((char *)pQVar11,DAT_0002e26c + 0x2e020);
    android::CameraParameters::set((char *)pQVar11,(char *)(DAT_0002e270 + 0x2e02c));
  }
  android::CameraParameters::set((char *)pQVar11,DAT_0002e278 + 0x2e03a);
  __android_log_print(4,DAT_0002e27c + 0x2e04a,DAT_0002e280 + 0x2e04c,DAT_0002e284 + 0x2e050,
                      *(undefined4 *)(this + 0xc));
  if (this[0xcec] != (QualcommCameraHardware)0x0) {
    android::CameraParameters::set((char *)pQVar11,(char *)(DAT_0002e288 + 0x2e062));
  }
  iVar8 = (**(code **)(*(int *)this + 0x5c))(this,pQVar11);
  if (iVar8 != 0) {
    __android_log_print(6,DAT_0002e290 + 0x2e07c,DAT_0002e294 + 0x2e07e);
  }
  pthread_mutex_lock((pthread_mutex_t *)(this + 0x3f8));
  this[0x3fc] = (QualcommCameraHardware)0x0;
  FUN_0002ce34(this + 0x310);
  FUN_0002ce34(this + 0x2f4);
  FUN_0002ce34(this + 0x308);
  FUN_0002ce34(this + 0x2ec);
  this[0x618] = (QualcommCameraHardware)0x1;
  this[0xd48] = (QualcommCameraHardware)0x0;
  setExifFixedAttribute(this);
  __android_log_print(4,DAT_0002e298 + 0x2e0ca,DAT_0002e29c + 0x2e0cc);
  pthread_mutex_unlock((pthread_mutex_t *)(this + 0x3f8));
  if (local_24 == **(int **)(iVar10 + DAT_0002e2a0)) {
    return;
  }
                    /* WARNING: Subroutine does not return */
  __stack_chk_fail();
}



// Function: _FINI_1 @ 0002d488

void _FINI_1(void)

{
  int iVar1;
  int iVar2;
  int iVar3;
  char cVar4;
  int iVar5;
  char *pcVar6;
  undefined2 uVar7;
  int unaff_r4;
  QualcommCameraHardware *unaff_r5;
  QualcommCameraHardware *pQVar8;
  int unaff_r7;
  String8 *unaff_r8;
  String8 *pSVar9;
  String8 *pSVar10;
  float fVar11;
  int in_stack_000000c4;
  
  android::String8::setTo((String8 *)(unaff_r7 * 0x2000000 + 0x2d490));
  android::String8::~String8(unaff_r8);
  iVar5 = android::QualcommCameraHardware::supportsSceneDetection();
  if (iVar5 != 0) {
    FUN_0001f400((String8 *)&stack0x00000034,DAT_0002db04 + 0x2d4ae,2);
    android::String8::setTo((String8 *)(DAT_0002db08 + 0x2d4ba));
    android::String8::~String8((String8 *)&stack0x00000034);
  }
  if ((unaff_r5[0xd28] != (QualcommCameraHardware)0x0) &&
     (iVar5 = android::QualcommCameraHardware::supportsSelectableZoneAf(), iVar5 != 0)) {
    FUN_0001f400((String8 *)&stack0x00000030,DAT_0002db0c + 0x2d4de,4);
    android::String8::setTo((String8 *)(DAT_0002db10 + 0x2d4ea));
    android::String8::~String8((String8 *)&stack0x00000030);
  }
  if ((unaff_r5[0xd28] != (QualcommCameraHardware)0x0) &&
     (iVar5 = android::QualcommCameraHardware::supportsFaceDetection(), iVar5 != 0)) {
    FUN_0001f400((String8 *)&stack0x0000002c,DAT_0002db14 + 0x2d510,2);
    android::String8::setTo((String8 *)(DAT_0002db18 + 0x2d51c));
    android::String8::~String8((String8 *)&stack0x0000002c);
  }
  FUN_0001f400((String8 *)&stack0x00000028,DAT_0002db1c + 0x2d534,2);
  android::String8::setTo((String8 *)(DAT_0002db20 + 0x2d540));
  android::String8::~String8((String8 *)&stack0x00000028);
  *(undefined1 *)(DAT_0002db24 + 0x2d552) = 1;
  if (unaff_r5[0xcec] == (QualcommCameraHardware)0x0) {
    android::CameraParameters::setPreviewSize((int)(unaff_r5 + 0x18),0x280);
    uVar7 = 0x1e0;
    *(undefined2 *)(unaff_r5 + 0x440) = 0x280;
  }
  else {
    __android_log_print(6,DAT_0002db28 + 0x2d570,DAT_0002db2c + 0x2d572);
    android::CameraParameters::setPreviewSize((int)(unaff_r5 + 0x18),0x500);
    *(undefined2 *)(unaff_r5 + 0x440) = 0x500;
    uVar7 = 0x2d0;
  }
  pQVar8 = unaff_r5 + 0x18;
  *(undefined2 *)(unaff_r5 + 0x442) = uVar7;
  android::CameraParameters::setPreviewFrameRateMode((char *)pQVar8);
  android::CameraParameters::setPreviewFormat((char *)pQVar8);
  iVar1 = DAT_0002db40;
  iVar5 = DAT_0002db3c;
  android::CameraParameters::setPictureFormat((char *)pQVar8);
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002db44));
  iVar2 = DAT_0002db4c;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
  android::CameraParameters::set((char *)pQVar8,(char *)(DAT_0002db50 + 0x2d60e));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002db58));
  iVar3 = DAT_0002db64;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar1));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002db68));
  iVar1 = DAT_0002db70;
  *(undefined2 *)(unaff_r5 + 0x44c) = 0x180;
  *(undefined2 *)(unaff_r5 + 0x44e) = 0x200;
  iVar5 = DAT_0002da34;
  if (*(int *)(iVar1 + 0x2d660) == 0) {
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002db74));
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
    pSVar9 = (String8 *)&stack0x00000024;
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar2));
    android::CameraParameters::setPreviewFrameRate((int)pQVar8);
    android::CameraParameters::setPictureSize((int)pQVar8,0xcc0);
    FUN_00026b38(pSVar9,DAT_0002db84 + 0x2d6ba,4);
    pcVar6 = *(char **)(unaff_r4 + iVar3);
  }
  else {
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002db74));
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
    pSVar9 = (String8 *)&stack0x00000020;
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar2));
    android::CameraParameters::setPreviewFrameRate((int)pQVar8);
    android::CameraParameters::setPictureSize((int)pQVar8,0x640);
    FUN_00026b38(pSVar9,DAT_0002db94 + 0x2d716,3);
    pcVar6 = *(char **)(unaff_r4 + iVar3);
  }
  android::CameraParameters::set((char *)pQVar8,pcVar6);
  android::String8::~String8(pSVar9);
  iVar1 = DAT_0002dba4;
  iVar5 = DAT_0002db9c;
  if (*(char *)(DAT_0002db98 + 0x2d736) == '\0') {
    pcVar6 = *(char **)(unaff_r4 + DAT_0002db9c);
  }
  else {
    android::CameraParameters::set((char *)pQVar8,*(int *)(unaff_r4 + DAT_0002dba0));
    android::CameraParameters::set((char *)pQVar8,*(int *)(unaff_r4 + iVar1));
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002dba8));
    pcVar6 = *(char **)(unaff_r4 + iVar5);
  }
  iVar5 = DAT_0002dbb8;
  android::CameraParameters::set((char *)pQVar8,pcVar6);
  if ((*(char *)(iVar5 + 0x2d78c) == '\0') || (*(char *)(DAT_0002dbbc + 0x2d796) == '\0')) {
    pcVar6 = (char *)(DAT_0002dbc8 + 0x2d7b4);
  }
  else {
    pcVar6 = (char *)(DAT_0002dbc0 + 0x2d7a6);
  }
  android::CameraParameters::set((char *)pQVar8,pcVar6);
  android::CameraParameters::set((char *)pQVar8,*(int *)(unaff_r4 + DAT_0002dbd4));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002dbe0));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002dbe4));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002dbe8));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002dbec));
  iVar2 = DAT_0002dc0c;
  iVar1 = DAT_0002dc04;
  iVar5 = DAT_0002dbf8;
  if (*(int *)(DAT_0002dbf4 + 0x2d818) - 3U < 4) {
    if (*(int *)(DAT_0002dbf4 + 0x2d818) - 2U < 2) {
      pSVar9 = (String8 *)&stack0x0000001c;
      FUN_0001f400(pSVar9,DAT_0002dc00 + 0x2d846,2);
      pSVar10 = (String8 *)(iVar1 + 0x2d84e);
    }
    else {
      pSVar9 = (String8 *)&stack0x00000018;
      FUN_0001f400(pSVar9,DAT_0002dc08 + 0x2d85a,3);
      pSVar10 = (String8 *)(iVar2 + 0x2d864);
    }
    android::String8::setTo(pSVar10);
    android::String8::~String8(pSVar9);
    pcVar6 = *(char **)(unaff_r4 + iVar5);
  }
  else {
    pcVar6 = *(char **)(unaff_r4 + DAT_0002dbf8);
  }
  android::CameraParameters::set((char *)pQVar8,pcVar6);
  pSVar9 = (String8 *)(DAT_0002dc10 + 0x2d892);
  FUN_0001f400((String8 *)&stack0x00000014,DAT_0002dc14 + 0x2d890,2);
  android::String8::setTo(pSVar9);
  android::String8::~String8((String8 *)&stack0x00000014);
  cVar4 = (**(code **)(DAT_0002dc18 + 0x2d8b4))(5);
  if (cVar4 != '\0') {
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002dc1c));
  }
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002da00));
  iVar5 = DAT_0002dc34;
  if (*(int *)(DAT_0002dc24 + 0x2d8dc) == 0) {
    pcVar6 = *(char **)(unaff_r4 + DAT_0002dc28);
  }
  else {
    pcVar6 = *(char **)(unaff_r4 + DAT_0002dc28);
  }
  android::CameraParameters::set((char *)pQVar8,pcVar6);
  iVar2 = DAT_0002dc40;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002dc38));
  iVar1 = DAT_0002da18;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002dc48));
  iVar5 = DAT_0002dc54;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar2));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002dc58));
  if (unaff_r5[0xd28] == (QualcommCameraHardware)0x0) {
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002da14));
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar1));
    pcVar6 = *(char **)(unaff_r4 + iVar5);
  }
  else {
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002da14));
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar1));
    pcVar6 = *(char **)(unaff_r4 + iVar5);
  }
  android::CameraParameters::set((char *)pQVar8,pcVar6);
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e0f8));
  iVar5 = DAT_0002e10c;
  if (*(int *)(DAT_0002e100 + 0x2dcb6) == 0) {
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e104));
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
  }
  iVar5 = DAT_0002e118;
  android::CameraParameters::set((char *)pQVar8,*(int *)(unaff_r4 + DAT_0002e114));
  android::CameraParameters::set((char *)pQVar8,*(int *)(unaff_r4 + DAT_0002e120));
  android::CameraParameters::set((char *)pQVar8,*(int *)(unaff_r4 + DAT_0002e124));
  android::CameraParameters::set((char *)pQVar8,*(int *)(unaff_r4 + DAT_0002e128));
  android::CameraParameters::set((char *)pQVar8,*(int *)(unaff_r4 + DAT_0002e12c));
  fVar11 = (float)android::CameraParameters::set((char *)pQVar8,*(int *)(unaff_r4 + iVar5));
  android::CameraParameters::setFloat((char *)pQVar8,fVar11);
  iVar5 = DAT_0002e13c;
  android::CameraParameters::set((char *)pQVar8,(char *)(DAT_0002e130 + 0x2dd54));
  android::CameraParameters::set((char *)pQVar8,(char *)(DAT_0002e140 + 0x2dd6a));
  android::CameraParameters::set((char *)pQVar8,DAT_0002e144 + 0x2dd76);
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e14c));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e150));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e160));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e168));
  iVar5 = DAT_0002e174;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e170));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e178));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e184));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e188));
  iVar1 = DAT_0002e194;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e198));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e1a0));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar1));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e1b4));
  android::CameraParameters::set((char *)pQVar8,(char *)(DAT_0002e1b8 + 0x2de76));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e1c0));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e1c8));
  iVar5 = DAT_0002e1d4;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e1cc));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e1d8));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
  android::CameraParameters::set((char *)pQVar8,(char *)(DAT_0002e1e8 + 0x2dedc));
  android::CameraParameters::set((char *)pQVar8,(char *)(DAT_0002e1ec + 0x2deea));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e1f0));
  iVar5 = DAT_0002e1f8;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e1f4));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e1fc));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e210));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e214));
  iVar5 = DAT_0002e220;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e21c));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e224));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e230));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e234));
  iVar1 = DAT_0002e240;
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + iVar5));
  android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e244));
  if (*(int *)(DAT_0002e24c + 0x2dfb4) == 0) {
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e250));
    pcVar6 = *(char **)(unaff_r4 + iVar1);
  }
  else {
    android::CameraParameters::set((char *)pQVar8,*(char **)(unaff_r4 + DAT_0002e250));
    pcVar6 = *(char **)(unaff_r4 + iVar1);
  }
  android::CameraParameters::set((char *)pQVar8,pcVar6);
  *(undefined4 *)(unaff_r5 + 0xc) = 1;
  if (*(int *)(unaff_r5 + 0xd64) != 0) {
    property_get(DAT_0002e264 + 0x2dffa,&stack0x0000000c,DAT_0002e268 + 0x2dffc);
    iVar5 = atoi(&stack0x0000000c);
    *(int *)(unaff_r5 + 0xc) = iVar5;
    if (iVar5 < 4) {
      if (iVar5 < 1) {
        *(undefined4 *)(unaff_r5 + 0xc) = 1;
      }
    }
    else {
      *(undefined4 *)(unaff_r5 + 0xc) = 3;
    }
    android::CameraParameters::set((char *)pQVar8,DAT_0002e26c + 0x2e020);
    android::CameraParameters::set((char *)pQVar8,(char *)(DAT_0002e270 + 0x2e02c));
  }
  android::CameraParameters::set((char *)pQVar8,DAT_0002e278 + 0x2e03a);
  __android_log_print(4,DAT_0002e27c + 0x2e04a,DAT_0002e280 + 0x2e04c,DAT_0002e284 + 0x2e050);
  if (unaff_r5[0xcec] != (QualcommCameraHardware)0x0) {
    android::CameraParameters::set((char *)pQVar8,(char *)(DAT_0002e288 + 0x2e062));
  }
  iVar5 = (**(code **)(*(int *)unaff_r5 + 0x5c))();
  if (iVar5 != 0) {
    __android_log_print(6,DAT_0002e290 + 0x2e07c,DAT_0002e294 + 0x2e07e);
  }
  pthread_mutex_lock((pthread_mutex_t *)(unaff_r5 + 0x3f8));
  unaff_r5[0x3fc] = (QualcommCameraHardware)0x0;
  FUN_0002ce34(unaff_r5 + 0x310);
  FUN_0002ce34(unaff_r5 + 0x2f4);
  FUN_0002ce34(unaff_r5 + 0x308);
  FUN_0002ce34(unaff_r5 + 0x2ec);
  unaff_r5[0x618] = (QualcommCameraHardware)0x1;
  unaff_r5[0xd48] = (QualcommCameraHardware)0x0;
  android::QualcommCameraHardware::setExifFixedAttribute(unaff_r5);
  __android_log_print(4,DAT_0002e298 + 0x2e0ca,DAT_0002e29c + 0x2e0cc);
  pthread_mutex_unlock((pthread_mutex_t *)(unaff_r5 + 0x3f8));
  if (in_stack_000000c4 != **(int **)(unaff_r4 + DAT_0002e2a0)) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail();
  }
  return;
}



// Function: createInstance @ 0002e2a4

/* android::QualcommCameraHardware::createInstance() */

QualcommCameraHardware * android::QualcommCameraHardware::createInstance(void)

{
  int iVar1;
  int iVar2;
  int *piVar3;
  char *pcVar4;
  undefined4 *puVar5;
  QualcommCameraHardware *this;
  int iVar6;
  stat sStack_80;
  
  iVar6 = DAT_0002e36c + 0x2e2b0;
  __android_log_print(4,iVar6,DAT_0002e370 + 0x2e2b4);
  iVar2 = stat((char *)(DAT_0002e374 + 0x2e2c0),&sStack_80);
  iVar1 = DAT_0002e380;
  if (iVar2 < 0) {
    piVar3 = (int *)__errno();
    this = (QualcommCameraHardware *)0x0;
    pcVar4 = strerror(*piVar3);
    __android_log_print(3,iVar6,DAT_0002e378 + 0x2e2da,pcVar4);
    pthread_mutex_unlock((pthread_mutex_t *)(DAT_0002e37c + 0x2e2e6));
  }
  else {
    this = operator_new(0xd98);
    QualcommCameraHardware(this);
    iVar2 = DAT_0002e388;
    puVar5 = *(undefined4 **)(DAT_0002e384 + 0x2e308);
    *(char *)(iVar1 + 0x2e2f8) = '\x01';
    *puVar5 = this;
    __android_log_print(4,iVar6,iVar2 + 0x2e30e,this);
    iVar2 = startCamera();
    if (iVar2 == 0) {
      __android_log_print(6,iVar6,DAT_0002e38c + 0x2e32c,DAT_0002e390 + 0x2e330);
      this = (QualcommCameraHardware *)0x0;
    }
    else if (*(char *)(iVar1 + 0x2e2f8) == '\0') {
      __android_log_print(6,iVar6,DAT_0002e394 + 0x2e344,DAT_0002e398 + 0x2e348);
      pthread_mutex_unlock((pthread_mutex_t *)(DAT_0002e39c + 0x2e350));
      this = (QualcommCameraHardware *)0x0;
    }
    else {
      initDefaultParameters(this);
      __android_log_print(4,iVar6,DAT_0002e3a0 + 0x2e364);
    }
  }
  return this;
}



// Function: HAL_openCameraHardware @ 0002e3a4

undefined8
HAL_openCameraHardware(int param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 *puVar1;
  undefined4 uVar2;
  int *piVar3;
  int iVar4;
  int iVar5;
  int iVar6;
  
  iVar4 = 0;
  iVar6 = param_1;
  __android_log_print(4,DAT_0002e42c + 0x2e3b0,DAT_0002e430 + 0x2e3b4,param_4,param_1,param_2,
                      param_3);
  do {
    if (*(int *)(DAT_0002e434 + 0x2e3be) <= iVar4) {
      __android_log_print(6,DAT_0002e458 + 0x2e422,DAT_0002e45c + 0x2e424,param_1,iVar6,param_2,
                          param_3);
      uVar2 = 0;
LAB_0002e428:
      return CONCAT44(iVar6,uVar2);
    }
    if (iVar4 == param_1) {
      iVar5 = DAT_0002e438 + 0x2e3d0;
      __android_log_print(4,iVar5,DAT_0002e43c + 0x2e3d4,iVar4,iVar6,param_2,param_3);
      iVar6 = DAT_0002e44c;
      piVar3 = (int *)(DAT_0002e444 + 0x2e3e8);
      puVar1 = (undefined4 *)(DAT_0002e448 + 0x2e3ea);
      *(undefined1 *)(DAT_0002e440 + 0x2e3e4) = 0;
      *piVar3 = iVar4;
      iVar4 = DAT_0002e450;
      *puVar1 = 1;
      *(undefined4 *)(iVar6 + 0x2e3f0) = 4;
      iVar6 = 4;
      __android_log_print(4,iVar5,iVar4 + 0x2e3fe,DAT_0002e454 + 0x2e406,4,1);
      uVar2 = android::QualcommCameraHardware::createInstance();
      goto LAB_0002e428;
    }
    iVar4 = iVar4 + 1;
  } while( true );
}



// Function: release @ 0002e460

/* android::QualcommCameraHardware::release() */

void android::QualcommCameraHardware::release(void)

{
  int iVar1;
  QualcommCameraHardware *in_r0;
  undefined4 uVar2;
  int iVar3;
  int iVar4;
  int iVar5;
  pthread_mutex_t *__mutex;
  pthread_mutex_t *ppVar6;
  undefined8 uVar7;
  
  iVar3 = DAT_0002e720 + 0x2e46e;
  __android_log_print(4,iVar3,DAT_0002e724 + 0x2e472);
  pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x3f0));
  __android_log_print(4,iVar3,DAT_0002e728 + 0x2e48a,*(undefined4 *)(DAT_0002e72c + 0x2e48e));
  iVar4 = DAT_0002e73c;
  iVar1 = DAT_0002e738;
  iVar3 = DAT_0002e734;
  if (*(int *)(DAT_0002e730 + 0x2e498) == 0) {
    ppVar6 = (pthread_mutex_t *)(in_r0 + 0x3c0);
    pthread_mutex_lock(ppVar6);
    iVar5 = DAT_0002e740 + 0x2e4bc;
    while (in_r0[0x3bc] != (QualcommCameraHardware)0x0) {
      __android_log_print(2,iVar3 + 0x2e4b6,iVar1 + 0x2e4b8,iVar4 + 0x2e4ba);
      pthread_cond_wait((pthread_cond_t *)(in_r0 + 0x3c4),ppVar6);
      __android_log_print(2,iVar3 + 0x2e4b6,iVar5,iVar4 + 0x2e4ba);
    }
    pthread_mutex_unlock(ppVar6);
  }
  __android_log_print(4,DAT_0002e744 + 0x2e4f4,DAT_0002e748 + 0x2e4fa,in_r0[0x34]);
  if (in_r0[0x34] != (QualcommCameraHardware)0x0) {
    if ((*(int *)(in_r0 + 0xcfc) != 0) && (*(int *)(in_r0 + 0xcf0) << 0x1a < 0)) {
      pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x410));
      in_r0[0x3fd] = (QualcommCameraHardware)0x1;
      pthread_cond_signal((pthread_cond_t *)(in_r0 + 0x414));
      pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x410));
    }
    stopPreviewInternal();
    __android_log_print(4,DAT_0002e74c + 0x2e53c,DAT_0002e750 + 0x2e53e);
  }
  (*(code *)**(undefined4 **)(DAT_0002e754 + 0x2e550))();
  if (*(int *)(in_r0 + 0x3e4) == 1) {
    uVar2 = 0xd;
  }
  else {
    uVar2 = 0xe;
  }
  iVar4 = DAT_0002e75c + 0x2e572;
  (**(code **)(*(int *)(DAT_0002e758 + 0x2e55e) + 0xc))(uVar2,0,0);
  pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x3c0));
  in_r0[0x3bc] = (QualcommCameraHardware)0x0;
  ppVar6 = (pthread_mutex_t *)(in_r0 + 0x3b0);
  pthread_cond_signal((pthread_cond_t *)(in_r0 + 0x3c4));
  iVar1 = DAT_0002e764;
  iVar3 = DAT_0002e760;
  pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x3c0));
  pthread_mutex_lock(ppVar6);
  while (in_r0[0x3ac] != (QualcommCameraHardware)0x0) {
    __android_log_print(2,iVar3 + 0x2e59e,iVar1 + 0x2e5a0);
    pthread_cond_wait((pthread_cond_t *)(in_r0 + 0x3b4),ppVar6);
    __android_log_print(2,iVar3 + 0x2e59e,iVar4);
  }
  pthread_mutex_unlock(ppVar6);
  pthread_mutex_lock((pthread_mutex_t *)(in_r0 + 0x3b8));
  deinitRaw(in_r0);
  pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x3b8));
  iVar1 = DAT_0002e76c;
  iVar3 = DAT_0002e768;
  deinitRawSnapshot(in_r0);
  __android_log_print(4,iVar3 + 0x2e5f0,DAT_0002e770 + 0x2e5f4);
  if (*(int *)(iVar1 + 0x2e5fc) == 6) {
    __android_log_print(2,iVar3 + 0x2e5f0,DAT_0002e774 + 0x2e60e);
    sp<android::QualcommCameraHardware::IonPool>::clear
              ((sp<android::QualcommCameraHardware::IonPool> *)(in_r0 + 0x308));
    FUN_0002ce34((sp<android::QualcommCameraHardware::IonPool> *)(in_r0 + 0x308));
    sp<android::QualcommCameraHardware::IonPool>::clear
              ((sp<android::QualcommCameraHardware::IonPool> *)(in_r0 + 0x2ec));
    FUN_0002ce34((sp<android::QualcommCameraHardware::IonPool> *)(in_r0 + 0x2ec));
    sp<android::QualcommCameraHardware::IonPool>::clear
              ((sp<android::QualcommCameraHardware::IonPool> *)(in_r0 + 0x310));
    FUN_0002ce34((sp<android::QualcommCameraHardware::IonPool> *)(in_r0 + 0x310));
    sp<android::QualcommCameraHardware::IonPool>::clear
              ((sp<android::QualcommCameraHardware::IonPool> *)(in_r0 + 0x2f4));
    FUN_0002ce34((sp<android::QualcommCameraHardware::IonPool> *)(in_r0 + 0x2f4));
  }
  __mutex = (pthread_mutex_t *)(DAT_0002e77c + 0x2e65e);
  iVar4 = DAT_0002e780 + 0x2e662;
  (*(code *)**(undefined4 **)(DAT_0002e778 + 0x2e656))();
  pthread_mutex_lock(__mutex);
  ppVar6 = (pthread_mutex_t *)(in_r0 + 0x35c);
  *(undefined1 *)(DAT_0002e784 + 0x2e674) = 1;
  uVar7 = systemTime();
  *(undefined8 *)(DAT_0002e788 + 0x2e680) = uVar7;
  pthread_mutex_unlock(__mutex);
  __android_log_print(4,iVar4,DAT_0002e78c + 0x2e69a,in_r0[0x34],in_r0[0x358]);
  __android_log_print(4,iVar4,DAT_0002e790 + 0x2e6b2,in_r0[0x365],in_r0[0x3ac],in_r0[0x3bc]);
  iVar1 = DAT_0002e79c;
  iVar3 = DAT_0002e798;
  __android_log_print(4,iVar4,DAT_0002e794 + 0x2e6ca,in_r0[0x3fc],in_r0[0x5cc]);
  iVar4 = DAT_0002e7a0;
  pthread_mutex_lock(ppVar6);
  while (in_r0[0x358] != (QualcommCameraHardware)0x0) {
    __android_log_print(2,iVar3 + 0x2e6e6,iVar1 + 0x2e6e8);
    pthread_cond_wait((pthread_cond_t *)(in_r0 + 0x360),ppVar6);
    __android_log_print(2,iVar3 + 0x2e6e6,iVar4 + 0x2e6ea);
  }
  pthread_mutex_unlock(ppVar6);
  pthread_mutex_unlock((pthread_mutex_t *)(in_r0 + 0x3f0));
  return;
}



// Function: FUN_0002e7a4 @ 0002e7a4

undefined4 * FUN_0002e7a4(undefined4 *param_1)

{
  int *piVar1;
  
  piVar1 = (int *)*param_1;
  if (piVar1 != (int *)0x0) {
    android::RefBase::decStrong((void *)((int)piVar1 + *(int *)(*piVar1 + -0xc)));
  }
  *param_1 = 0;
  return param_1;
}



// Function: getBufferInfo @ 0002e7c4

/* android::QualcommCameraHardware::getBufferInfo(android::sp<android::IMemory>&, unsigned int*) */

undefined4 __thiscall
android::QualcommCameraHardware::getBufferInfo
          (QualcommCameraHardware *this,sp *param_1,uint *param_2)

{
  int iVar1;
  int *piVar2;
  uint uVar3;
  int iVar4;
  int *piVar5;
  undefined4 uVar6;
  
  iVar4 = DAT_0002e8b0 + 0x2e7d2;
  __android_log_print(2,iVar4,DAT_0002e8b4 + 0x2e7da);
  if (*(int *)(DAT_0002e8b8 + 0x2e7e4) - 4U < 3) {
    if (*(int *)(this + 0x2e8) == 0) {
      iVar1 = DAT_0002e8d0 + 0x2e850;
      goto LAB_0002e888;
    }
    __android_log_print(2,iVar4,DAT_0002e8bc + 0x2e7fa);
    piVar5 = (int *)**(undefined4 **)(*(int *)(this + 0x2e8) + 0x1c);
    if (piVar5 != (int *)0x0) {
      android::RefBase::incStrong((void *)((int)piVar5 + *(int *)(*piVar5 + -0xc)));
    }
    piVar2 = *(int **)param_1;
    if (piVar2 != (int *)0x0) {
      android::RefBase::decStrong((void *)((int)piVar2 + *(int *)(*piVar2 + -0xc)));
    }
    *(int **)param_1 = piVar5;
    if (param_2 != (uint *)0x0) {
      uVar3 = *(uint *)(*(int *)(this + 0x2e8) + 0xc);
      iVar4 = DAT_0002e8c0 + 0x2e83a;
      iVar1 = DAT_0002e8c4 + 0x2e83c;
      *param_2 = uVar3;
      goto LAB_0002e86e;
    }
    iVar4 = DAT_0002e8c8 + 0x2e848;
    iVar1 = DAT_0002e8cc + 0x2e84a;
LAB_0002e87e:
    __android_log_print(6,iVar4,iVar1);
  }
  else {
    if (this != (QualcommCameraHardware *)0xfffffd34) {
      __android_log_print(2,iVar4,DAT_0002e8d4 + 0x2e860);
      if (param_2 == (uint *)0x0) {
        iVar1 = DAT_0002e8dc + 0x2e880;
        goto LAB_0002e87e;
      }
      uVar3 = *param_2;
      iVar1 = DAT_0002e8d8 + 0x2e870;
LAB_0002e86e:
      __android_log_print(2,iVar4,iVar1,uVar3);
      uVar6 = 0;
      goto LAB_0002e89a;
    }
    iVar1 = DAT_0002e8e0 + 0x2e88a;
LAB_0002e888:
    __android_log_print(6,iVar4,iVar1);
    FUN_0002e7a4(param_1);
  }
  uVar6 = 0x80000000;
LAB_0002e89a:
  __android_log_print(2,DAT_0002e8e4 + 0x2e8a4,DAT_0002e8e8 + 0x2e8a6);
  return uVar6;
}



// Function: __aeabi_unwind_cpp_pr0 @ 00039000

/* WARNING: Control flow encountered bad instruction data */

void __aeabi_unwind_cpp_pr0(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: memcpy @ 00039004

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * memcpy(void *__dest,void *__src,size_t __n)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: CameraParameters @ 00039008

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::CameraParameters::CameraParameters(CameraParameters *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __aeabi_atexit @ 0003900c

/* WARNING: Control flow encountered bad instruction data */

void __aeabi_atexit(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: String8 @ 00039010

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::String8::String8(String8 *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: ~CameraParameters @ 00039014

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::CameraParameters::~CameraParameters(CameraParameters *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: ~String8 @ 00039018

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::String8::~String8(String8 *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __android_log_print @ 0003901c

/* WARNING: Control flow encountered bad instruction data */

void __android_log_print(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: operator= @ 00039020

/* WARNING: Control flow encountered bad instruction data */

void __thiscall
android::SortedVectorImpl::operator=(SortedVectorImpl *this,SortedVectorImpl *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setTo @ 00039024

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::String8::setTo(String8 *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: flatten @ 00039028

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::flatten(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: String8 @ 0003902c

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::String8::String8(String8 *this,char *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: unflatten @ 00039030

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::unflatten(String8 *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: free @ 00039034

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void free(void *__ptr)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: atoi @ 00039038

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int atoi(char *__nptr)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: strcmp @ 0003903c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int strcmp(char *__s1,char *__s2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: malloc @ 00039040

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * malloc(size_t __size)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: memset @ 00039044

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * memset(void *__s,int __c,size_t __n)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __aeabi_idiv @ 00039048

/* WARNING: Control flow encountered bad instruction data */

void __aeabi_idiv(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: decWeak @ 0003904c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::weakref_type::decWeak(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_cond_destroy @ 00039050

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_cond_destroy(pthread_cond_t *__cond)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_mutex_destroy @ 00039054

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_destroy(pthread_mutex_t *__mutex)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __aeabi_unwind_cpp_pr1 @ 00039058

/* WARNING: Control flow encountered bad instruction data */

void __aeabi_unwind_cpp_pr1(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __aeabi_uidivmod @ 0003905c

/* WARNING: Control flow encountered bad instruction data */

void __aeabi_uidivmod(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: property_get @ 00039060

/* WARNING: Control flow encountered bad instruction data */

void property_get(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __stack_chk_fail @ 00039064

/* WARNING: Control flow encountered bad instruction data */

void __stack_chk_fail(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: dlopen @ 0003906c

/* WARNING: Control flow encountered bad instruction data */

void dlopen(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: dlerror @ 00039070

/* WARNING: Control flow encountered bad instruction data */

void dlerror(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: dlsym @ 00039074

/* WARNING: Control flow encountered bad instruction data */

void dlsym(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: dlclose @ 00039078

/* WARNING: Control flow encountered bad instruction data */

void dlclose(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_mutex_lock @ 0003907c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_lock(pthread_mutex_t *__mutex)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_mutex_unlock @ 00039080

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_unlock(pthread_mutex_t *__mutex)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_cond_signal @ 00039084

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_cond_signal(pthread_cond_t *__cond)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: incStrong @ 00039088

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::incStrong(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: decStrong @ 0003908c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::decStrong(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __errno @ 00039090

/* WARNING: Control flow encountered bad instruction data */

void __errno(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: strerror @ 00039094

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * strerror(int __errnum)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __aeabi_idivmod @ 00039098

/* WARNING: Control flow encountered bad instruction data */

void __aeabi_idivmod(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: append @ 0003909c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::String8::append(char *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: snprintf @ 000390a0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int snprintf(char *__s,size_t __maxlen,char *__format,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getDevice @ 000390a4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::MemoryHeapBase::getDevice(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: write @ 000390a8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

ssize_t write(int __fd,void *__buf,size_t __n)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: operator.delete[] @ 000390ac

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void operator_delete__(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: ~RefBase @ 000390b0

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::RefBase::~RefBase(RefBase *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: operator.delete @ 000390b4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void operator_delete(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: operator.new[] @ 000390b8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * operator_new__(uint param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: operator.new @ 000390bc

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * operator_new(uint param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: MemoryBase @ 000390c0

/* WARNING: Control flow encountered bad instruction data */

void __thiscall
android::MemoryBase::MemoryBase(MemoryBase *this,sp *param_1,long param_2,uint param_3)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: RefBase @ 000390c4

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::RefBase::RefBase(RefBase *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: MemoryHeapBase @ 000390cc

/* WARNING: Control flow encountered bad instruction data */

void __thiscall
android::MemoryHeapBase::MemoryHeapBase
          (MemoryHeapBase *this,uint param_1,uint param_2,char *param_3)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: attemptIncStrong @ 000390d0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::weakref_type::attemptIncStrong(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: createWeak @ 000390d4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::createWeak(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: MemoryHeapBase @ 000390d8

/* WARNING: Control flow encountered bad instruction data */

void __thiscall
android::MemoryHeapBase::MemoryHeapBase
          (MemoryHeapBase *this,char *param_1,uint param_2,uint param_3)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: MemoryHeapPmem @ 000390dc

/* WARNING: Control flow encountered bad instruction data */

void __thiscall
android::MemoryHeapPmem::MemoryHeapPmem(MemoryHeapPmem *this,sp *param_1,uint param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: ioctl @ 000390e0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int ioctl(int __fd,ulong __request,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: get @ 000390e4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::get(char *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: set @ 000390f0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::set(char *param_1,char *param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getInt @ 00039148

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getInt(char *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: set @ 00039150

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::set(char *param_1,int param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_cond_wait @ 00039188

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_cond_wait(pthread_cond_t *__cond,pthread_mutex_t *__mutex)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: strncmp @ 00039190

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int strncmp(char *__s1,char *__s2,size_t __n)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: remove @ 00039194

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::remove(char *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getPreviewSize @ 000391ac

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewSize(int *param_1,int *param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getMeteringAreaCenter @ 000391b0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getMeteringAreaCenter(int *param_1,int *param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setTouchIndexAec @ 000391b4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setTouchIndexAec(int param_1,int param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setTouchIndexAf @ 000391b8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setTouchIndexAf(int param_1,int param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_attr_init @ 000391c0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_attr_init(pthread_attr_t *__attr)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_attr_setdetachstate @ 000391c4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_attr_setdetachstate(pthread_attr_t *__attr,int __detachstate)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_create @ 000391c8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_create(pthread_t *__newthread,pthread_attr_t *__attr,__start_routine *__start_routine,
                  void *__arg)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getPreviewFormat @ 000391cc

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewFormat(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getPictureSize @ 000391d4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPictureSize(int *param_1,int *param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setPictureSize @ 000391d8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPictureSize(int param_1,int param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getPreviewFrameRateMode @ 000391dc

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewFrameRateMode(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getPreviewFrameRate @ 000391e0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewFrameRate(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setPreviewFrameRateMode @ 000391e4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewFrameRateMode(char *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setPreviewFrameRate @ 000391e8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewFrameRate(int param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getPreviewFpsRange @ 000391ec

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPreviewFpsRange(int *param_1,int *param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setPreviewFpsRange @ 000391f4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewFpsRange(int param_1,int param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setPreviewSize @ 000391f8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewSize(int param_1,int param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: strtol @ 000391fc

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

long strtol(char *__nptr,char **__endptr,int __base)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: strcpy @ 00039204

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * strcpy(char *__dest,char *__src)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: log @ 00039208

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

double log(double __x)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setFloat @ 0003920c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setFloat(char *param_1,float param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: strlen @ 00039214

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

size_t strlen(char *__s)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: time @ 00039220

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

time_t time(time_t *__timer)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: localtime @ 00039224

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

tm * localtime(time_t *__timer)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: strftime @ 00039228

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

size_t strftime(char *__s,size_t __maxsize,char *__format,tm *__tp)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pow @ 0003922c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

double pow(double __x,double __y)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getFloat @ 00039230

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getFloat(char *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: strtod @ 00039234

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

double strtod(char *__nptr,char **__endptr)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: lround @ 0003923c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

long lround(double __x)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: atol @ 00039240

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

long atol(char *__nptr)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: gmtime_r @ 00039244

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

tm * gmtime_r(time_t *__timer,tm *__tp)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: strncpy @ 00039248

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * strncpy(char *__dest,char *__src,size_t __n)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: JpegEncoder @ 0003924c

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::JpegEncoder::JpegEncoder(JpegEncoder *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: makeExif @ 00039250

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::JpegEncoder::makeExif
               (uchar *param_1,uchar *param_2,exif_attribute_t *param_3,uint *param_4,long param_5,
               bool param_6)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: ~JpegEncoder @ 00039254

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::JpegEncoder::~JpegEncoder(JpegEncoder *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: genlock_unlock_buffer @ 00039258

/* WARNING: Control flow encountered bad instruction data */

void genlock_unlock_buffer(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: native_handle_delete @ 0003925c

/* WARNING: Control flow encountered bad instruction data */

void native_handle_delete(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: open @ 00039260

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int open(char *__file,int __oflag,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: mmap @ 00039264

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * mmap(void *__addr,size_t __len,int __prot,int __flags,int __fd,__off_t __offset)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: close @ 00039268

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int close(int __fd)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __aeabi_l2f @ 0003926c

/* WARNING: Control flow encountered bad instruction data */

void __aeabi_l2f(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: systemTime @ 00039270

/* WARNING: Control flow encountered bad instruction data */

void systemTime(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: String8 @ 00039274

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::String8::String8(String8 *this,String8 *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: finish_vector @ 00039278

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::finish_vector(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: ~SortedVectorImpl @ 0003927c

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::SortedVectorImpl::~SortedVectorImpl(SortedVectorImpl *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: VectorImpl @ 00039280

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::VectorImpl::VectorImpl(VectorImpl *this,VectorImpl *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_mutex_trylock @ 00039284

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_trylock(pthread_mutex_t *__mutex)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: usleep @ 00039288

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int usleep(__useconds_t __useconds)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: munmap @ 00039290

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int munmap(void *__addr,size_t __len)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: index @ 00039294

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

char * index(char *__s,int __c)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: fopen @ 00039298

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

FILE * fopen(char *__filename,char *__modes)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: fscanf @ 0003929c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int fscanf(FILE *__stream,char *__format,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: fclose @ 000392a0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int fclose(FILE *__stream)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: genlock_lock_buffer @ 000392a4

/* WARNING: Control flow encountered bad instruction data */

void genlock_lock_buffer(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_join @ 000392a8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_join(pthread_t __th,void **__thread_return)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: sprintf @ 000392ac

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int sprintf(char *__s,char *__format,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: clear @ 000392b0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::clear(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: removeItemsAt @ 000392b4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::removeItemsAt(uint param_1,uint param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: add @ 000392b8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::add(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: ~VectorImpl @ 000392bc

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::VectorImpl::~VectorImpl(VectorImpl *this)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: memmove @ 000392c0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void * memmove(void *__dest,void *__src,size_t __n)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_mutex_init @ 000392c4

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_mutex_init(pthread_mutex_t *__mutex,pthread_mutexattr_t *__mutexattr)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: pthread_cond_init @ 000392c8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int pthread_cond_init(pthread_cond_t *__cond,pthread_condattr_t *__cond_attr)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: VectorImpl @ 000392cc

/* WARNING: Control flow encountered bad instruction data */

void __thiscall android::VectorImpl::VectorImpl(VectorImpl *this,uint param_1,uint param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: gmtime @ 000392d0

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

tm * gmtime(time_t *__timer)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: sscanf @ 000392d8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int sscanf(char *__s,char *__format,...)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: getPictureFormat @ 000392dc

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::getPictureFormat(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: native_handle_create @ 000392e4

/* WARNING: Control flow encountered bad instruction data */

void native_handle_create(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setPreviewFormat @ 000392e8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPreviewFormat(char *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: setPictureFormat @ 000392ec

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::CameraParameters::setPictureFormat(char *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: stat @ 000393f8

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

int stat(char *__file,stat *__buf)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: onFirstRef @ 000393fc

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::onFirstRef(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: onLastStrongRef @ 00039400

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::onLastStrongRef(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: onIncStrongAttempted @ 00039404

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::onIncStrongAttempted(uint param_1,void *param_2)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: onLastWeakRef @ 00039408

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::RefBase::onLastWeakRef(void *param_1)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedVectorImpl1 @ 0003940c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl1(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedVectorImpl2 @ 00039410

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl2(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedVectorImpl3 @ 00039414

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl3(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedVectorImpl4 @ 00039418

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl4(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedVectorImpl5 @ 0003941c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl5(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedVectorImpl6 @ 00039420

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl6(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedVectorImpl7 @ 00039424

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl7(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedVectorImpl8 @ 00039428

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::VectorImpl::reservedVectorImpl8(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedSortedVectorImpl1 @ 0003942c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl1(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedSortedVectorImpl2 @ 00039430

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl2(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedSortedVectorImpl3 @ 00039434

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl3(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedSortedVectorImpl4 @ 00039438

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl4(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedSortedVectorImpl5 @ 0003943c

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl5(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedSortedVectorImpl6 @ 00039440

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl6(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedSortedVectorImpl7 @ 00039444

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl7(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: reservedSortedVectorImpl8 @ 00039448

/* WARNING: Control flow encountered bad instruction data */
/* WARNING: Unknown calling convention -- yet parameter storage is locked */

void android::SortedVectorImpl::reservedSortedVectorImpl8(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



// Function: __cxa_finalize @ 00039518

/* WARNING: Control flow encountered bad instruction data */

void __cxa_finalize(void)

{
                    /* WARNING: Bad instruction - Truncating control flow here */
  halt_baddata();
}



