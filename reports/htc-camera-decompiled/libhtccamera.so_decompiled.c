/* Decompiled from libhtccamera.so using Ghidra */
/* HTC EV Shooter Android ROM - Camera Binaries */

/* ============================================= */
/* Function: entry */
/* Address: 0x000121a0 */
/* ============================================= */

void processEntry entry(void)

{
  __cxa_finalize(&__dso_handle);
  return;
}



/* ============================================= */
/* Function: Java_com_android_camera_CameraNativeLibrary_initialize */
/* Address: 0x000121b0 */
/* ============================================= */

undefined4
Java_com_android_camera_CameraNativeLibrary_initialize
          (undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  config::screen_width = param_3;
  config::screen_height = param_4;
  return 1;
}



/* ============================================= */
/* Function: Java_com_android_camera_imaging_ImageUtility_yvu420spToRgb565___3BIILjava_nio_ByteBuffer_2IZ */
/* Address: 0x000121cc */
/* ============================================= */

undefined4
Java_com_android_camera_imaging_ImageUtility_yvu420spToRgb565___3BIILjava_nio_ByteBuffer_2IZ
          (int *param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4,undefined4 param_5,
          undefined4 param_6,int param_7,char param_8)

{
  int iVar1;
  int iVar2;
  YuvBitmap *this;
  RgbBitmap *this_00;
  char *pcVar3;
  
  iVar1 = (**(code **)(*param_1 + 0x2e0))(param_1,param_3,0);
  if (iVar1 == 0) {
    __android_log_print(6,"libhtccamera","ImageUtility::yvu420spToRgb565() - Cannot get YUV buffer")
    ;
    return 0;
  }
  iVar2 = (**(code **)(*param_1 + 0x398))(param_1,param_6);
  if (iVar2 == 0) {
    pcVar3 = "ImageUtility::yvu420spToRgb565() - Cannot get RGB buffer";
  }
  else {
    this = (YuvBitmap *)YuvBitmap::Create(iVar1,param_4,param_5,2);
    if (this != (YuvBitmap *)0x0) {
      this_00 = (RgbBitmap *)RgbBitmap::Create(iVar2,param_4,param_5,0);
      if (this_00 != (RgbBitmap *)0x0) {
        YuvBitmap::ToRgbBitmap(this,this_00);
        RgbBitmap::Rotate(this_00,param_7);
        if (param_8 == '\x01') {
          RgbBitmap::FlipY(this_00);
        }
        YuvBitmap::~YuvBitmap(this);
        operator_delete(this);
        RgbBitmap::~RgbBitmap(this_00);
        operator_delete(this_00);
        (**(code **)(*param_1 + 0x300))(param_1,param_3,iVar1,2);
        return 1;
      }
      __android_log_print(6,"libhtccamera",
                          "ImageUtility::yvu420spToRgb565() - Cannot create RGB bitmap");
      YuvBitmap::~YuvBitmap(this);
      operator_delete(this);
      goto LAB_00012296;
    }
    pcVar3 = "ImageUtility::yvu420spToRgb565() - Cannot create YUV bitmap";
  }
  __android_log_print(6,"libhtccamera",pcVar3);
LAB_00012296:
  (**(code **)(*param_1 + 0x300))(param_1,param_3,iVar1,2);
  return 0;
}



/* ============================================= */
/* Function: Java_com_android_camera_imaging_ImageUtility_rotateRgb565Image90Clockwise */
/* Address: 0x00012300 */
/* ============================================= */

undefined4
Java_com_android_camera_imaging_ImageUtility_rotateRgb565Image90Clockwise
          (int *param_1,undefined4 param_2,undefined4 param_3,int param_4,int param_5,
          undefined4 param_6)

{
  uchar *puVar1;
  uchar *puVar2;
  undefined4 uVar3;
  
  puVar1 = (uchar *)(**(code **)(*param_1 + 0x398))(param_1,param_3);
  puVar2 = (uchar *)(**(code **)(*param_1 + 0x398))(param_1,param_6);
  if ((puVar2 == (uchar *)0x0) || (puVar1 == (uchar *)0x0)) {
    uVar3 = 0;
  }
  else {
    uVar3 = rotate_rgb_90_clockwise(puVar1,param_4,param_5,puVar2,1);
  }
  return uVar3;
}



/* ============================================= */
/* Function: Java_com_android_camera_imaging_ImageUtility_yvu420spToRgb565__Ljava_nio_ByteBuffer_2IILjava_nio_ByteBuffer_2 */
/* Address: 0x00012340 */
/* ============================================= */

undefined4
Java_com_android_camera_imaging_ImageUtility_yvu420spToRgb565__Ljava_nio_ByteBuffer_2IILjava_nio_ByteBuffer_2
          (int *param_1,undefined4 param_2,undefined4 param_3,int param_4,int param_5,
          undefined4 param_6)

{
  uchar *puVar1;
  uchar *puVar2;
  undefined4 uVar3;
  
  puVar1 = (uchar *)(**(code **)(*param_1 + 0x398))(param_1,param_3);
  puVar2 = (uchar *)(**(code **)(*param_1 + 0x398))(param_1,param_6);
  if ((puVar2 == (uchar *)0x0) || (puVar1 == (uchar *)0x0)) {
    uVar3 = 0;
  }
  else {
    uVar3 = yvu420sp_to_rgb(puVar1,param_4,param_5,puVar2,1);
  }
  return uVar3;
}



/* ============================================= */
/* Function: Java_com_android_camera_io_NativeBuffer_getSize */
/* Address: 0x00012380 */
/* ============================================= */

undefined4 Java_com_android_camera_io_NativeBuffer_getSize(void)

{
  return DAT_000d79a0;
}



/* ============================================= */
/* Function: FUN_00012390 */
/* Address: 0x00012390 */
/* ============================================= */

void FUN_00012390(void)

{
  DAT_000d79a4 = 0;
  DAT_000d79a8 = 0;
  DAT_000d79ac = 0;
  __aeabi_atexit(&DAT_000d79a4,0x123fd,&__dso_handle);
  return;
}



/* ============================================= */
/* Function: Clear */
/* Address: 0x000123c0 */
/* ============================================= */

/* LinkedList<Buffer*>::Clear() */

void __thiscall LinkedList<Buffer*>::Clear(LinkedList<Buffer*> *this)

{
  undefined4 *puVar1;
  undefined4 *puVar2;
  undefined4 *puVar3;
  
  puVar1 = *(undefined4 **)this;
  while (puVar1 != (undefined4 *)0x0) {
    puVar3 = (undefined4 *)puVar1[2];
    puVar2 = puVar3;
    if (puVar1[3] != 0) {
      *(undefined4 **)(puVar1[3] + 8) = puVar3;
      puVar2 = (undefined4 *)puVar1[2];
    }
    if (puVar2 != (undefined4 *)0x0) {
      puVar2[3] = puVar1[3];
    }
    puVar1[3] = 0;
    puVar1[2] = 0;
    *puVar1 = 0;
    operator_delete(puVar1);
    puVar1 = puVar3;
  }
  *(undefined4 *)this = 0;
  *(undefined4 *)(this + 4) = 0;
  *(undefined4 *)(this + 8) = 0;
  return;
}



/* ============================================= */
/* Function: ~LinkedList */
/* Address: 0x000123fc */
/* ============================================= */

/* LinkedList<Buffer*>::~LinkedList() */

LinkedList<Buffer*> * __thiscall LinkedList<Buffer*>::~LinkedList(LinkedList<Buffer*> *this)

{
  Clear(this);
  return this;
}



/* ============================================= */
/* Function: Java_com_android_camera_io_NativeBuffer_clear */
/* Address: 0x00012408 */
/* ============================================= */

void Java_com_android_camera_io_NativeBuffer_clear(void)

{
  int iVar1;
  Buffer *this;
  
  for (iVar1 = DAT_000d79a4; iVar1 != 0; iVar1 = *(int *)(iVar1 + 8)) {
    this = *(Buffer **)(iVar1 + 4);
    if (this != (Buffer *)0x0) {
      Buffer::~Buffer(this);
      operator_delete(this);
    }
  }
  LinkedList<Buffer*>::Clear((LinkedList<Buffer*> *)&DAT_000d79a4);
  DAT_000d79a0 = 0;
  return;
}



/* ============================================= */
/* Function: FUN_00012444 */
/* Address: 0x00012444 */
/* ============================================= */

undefined4 FUN_00012444(undefined4 *param_1)

{
  undefined4 uVar1;
  
  if ((param_1 == (undefined4 *)0x0) || ((undefined4 **)*param_1 != &DAT_000d79a4)) {
    uVar1 = 0;
  }
  else {
    if (param_1 == DAT_000d79a4) {
      DAT_000d79a4 = (undefined4 *)param_1[2];
    }
    if (param_1 == DAT_000d79a8) {
      DAT_000d79a8 = (undefined4 *)param_1[3];
    }
    if (param_1[3] != 0) {
      *(undefined4 *)(param_1[3] + 8) = param_1[2];
    }
    if (param_1[2] != 0) {
      *(undefined4 *)(param_1[2] + 0xc) = param_1[3];
    }
    param_1[3] = 0;
    param_1[2] = 0;
    param_1[3] = 0;
    param_1[2] = 0;
    *param_1 = 0;
    operator_delete(param_1);
    uVar1 = 1;
    DAT_000d79ac = DAT_000d79ac + -1;
  }
  return uVar1;
}



/* ============================================= */
/* Function: Java_com_android_camera_io_NativeBuffer_get */
/* Address: 0x000124b8 */
/* ============================================= */

undefined4
Java_com_android_camera_io_NativeBuffer_get
          (undefined4 param_1,undefined4 param_2,_JNIEnv *param_3,undefined4 param_4,char param_5)

{
  int iVar1;
  undefined4 uVar2;
  int iVar3;
  int iVar4;
  
  __android_log_print(3,"libhtccamera","NativeBuffer::get(%d)",param_3);
  iVar4 = DAT_000d79a4;
  while( true ) {
    if (iVar4 == 0) {
      return 0;
    }
    if (*(_JNIEnv **)(iVar4 + 4) == param_3) break;
    iVar4 = *(int *)(iVar4 + 8);
  }
  uVar2 = Buffer::ToJavaArray(param_3);
  iVar1 = DAT_000d79a0;
  if (param_5 != '\x01') {
    return uVar2;
  }
  iVar3 = Buffer::GetSize((Buffer *)param_3);
  DAT_000d79a0 = iVar1 - iVar3;
  if (param_3 != (_JNIEnv *)0x0) {
    Buffer::~Buffer((Buffer *)param_3);
    operator_delete(param_3);
  }
  FUN_00012444(iVar4);
  return uVar2;
}



/* ============================================= */
/* Function: Java_com_android_camera_io_NativeBuffer_remove */
/* Address: 0x00012544 */
/* ============================================= */

void Java_com_android_camera_io_NativeBuffer_remove
               (undefined4 param_1,undefined4 param_2,Buffer *param_3)

{
  int iVar1;
  int iVar2;
  int iVar3;
  
  iVar1 = DAT_000d79a0;
  iVar3 = DAT_000d79a4;
  while( true ) {
    if (iVar3 == 0) {
      return;
    }
    if (*(Buffer **)(iVar3 + 4) == param_3) break;
    iVar3 = *(int *)(iVar3 + 8);
  }
  iVar2 = Buffer::GetSize(param_3);
  DAT_000d79a0 = iVar1 - iVar2;
  if (param_3 != (Buffer *)0x0) {
    Buffer::~Buffer(param_3);
    operator_delete(param_3);
  }
  FUN_00012444(iVar3);
  return;
}



/* ============================================= */
/* Function: Java_com_android_camera_io_NativeBuffer_add */
/* Address: 0x00012590 */
/* ============================================= */

undefined8
Java_com_android_camera_io_NativeBuffer_add
          (_JNIEnv *param_1,undefined4 param_2,_jbyteArray *param_3)

{
  Buffer *this;
  undefined4 *puVar1;
  int iVar2;
  int iVar3;
  
  if ((param_3 == (_jbyteArray *)0x0) ||
     (this = (Buffer *)Buffer::Create(param_1,param_3), this == (Buffer *)0x0)) {
    this = (Buffer *)0x0;
    iVar3 = 0;
  }
  else {
    __android_log_print(3,"libhtccamera","NativeBuffer::add() - buffer : %d",this);
    puVar1 = operator_new(0x10);
    puVar1[1] = this;
    *puVar1 = &DAT_000d79a4;
    puVar1[2] = 0;
    puVar1[3] = 0;
    if (DAT_000d79a4 == (undefined4 *)0x0) {
      DAT_000d79a4 = puVar1;
    }
    if (DAT_000d79a8 != (undefined4 *)0x0) {
      if (puVar1[2] == 0) {
        iVar3 = DAT_000d79a8[2];
        puVar1[2] = iVar3;
        if (iVar3 != 0) {
          *(undefined4 **)(iVar3 + 0xc) = puVar1;
        }
        puVar1[3] = DAT_000d79a8;
        DAT_000d79a8[2] = puVar1;
      }
      else {
        __android_log_print(6,"libhtccamera",
                            "LinkedListNode<T>::Link() - Linked-list node is already linked to another node"
                           );
      }
    }
    iVar3 = (int)this >> 0x1f;
    DAT_000d79ac = DAT_000d79ac + 1;
    DAT_000d79a8 = puVar1;
    iVar2 = Buffer::GetSize(this);
    DAT_000d79a0 = iVar2 + DAT_000d79a0;
  }
  return CONCAT44(iVar3,this);
}



/* ============================================= */
/* Function: HistogramEqualizationIntensity */
/* Address: 0x00012648 */
/* ============================================= */

/* HistogramEqualizationIntensity(unsigned char*, int, int, unsigned char*) */

undefined4 HistogramEqualizationIntensity(uchar *param_1,int param_2,int param_3,uchar *param_4)

{
  int iVar1;
  int iVar2;
  int iVar3;
  uint uVar4;
  ushort *puVar5;
  int *piVar6;
  int iVar7;
  uint uVar8;
  int iVar9;
  uint uVar10;
  int iVar11;
  undefined8 uVar12;
  undefined8 uVar13;
  int local_428 [256];
  int iStack_28;
  
  iVar3 = param_3 * param_2;
  memset(local_428,0,0x400);
  if (0 < iVar3) {
    puVar5 = (ushort *)param_1;
    do {
      uVar4 = (uint)*puVar5;
      puVar5 = puVar5 + 1;
      iVar1 = __divsi3(((int)(uVar4 & 0x7e0) >> 3) + ((int)uVar4 >> 0xb) * 8 + (uVar4 & 0x1f) * 8,3)
      ;
      local_428[iVar1] = local_428[iVar1] + 1;
    } while (puVar5 != (ushort *)(param_1 + iVar3 * 2));
  }
  iVar1 = 0;
  piVar6 = local_428;
  do {
    iVar1 = iVar1 + *piVar6;
    uVar12 = __aeabi_i2d(iVar1);
    uVar13 = __aeabi_i2d(iVar3);
    uVar12 = __divdf3((int)uVar12,(int)((ulonglong)uVar12 >> 0x20),(int)uVar13,
                      (int)((ulonglong)uVar13 >> 0x20));
    __muldf3((int)uVar12,(int)((ulonglong)uVar12 >> 0x20),0,0x406fe000);
    iVar2 = __aeabi_d2iz();
    *piVar6 = iVar2;
    piVar6 = piVar6 + 1;
  } while (piVar6 != &iStack_28);
  if (DAT_000d79b0 == '\0') {
    DAT_000d79b0 = '\x01';
  }
  if (0 < iVar3) {
    iVar1 = 0;
    do {
      uVar4 = (uint)*(ushort *)(param_1 + iVar1);
      iVar9 = ((int)uVar4 >> 0xb) * 8;
      iVar7 = (int)(uVar4 & 0x7e0) >> 3;
      iVar11 = (uVar4 & 0x1f) * 8;
      iVar2 = __divsi3(iVar7 + iVar9 + iVar11,3);
      iVar2 = local_428[iVar2] - iVar2;
      uVar10 = iVar2 + iVar9;
      uVar8 = iVar2 + iVar7;
      uVar4 = iVar2 + iVar11;
      uVar10 = uVar10 & (int)~uVar10 >> 0x1f;
      uVar8 = uVar8 & (int)~uVar8 >> 0x1f;
      uVar4 = uVar4 & (int)~uVar4 >> 0x1f;
      if (0xff < (int)uVar10) {
        uVar10 = 0xff;
      }
      if (0xff < (int)uVar8) {
        uVar8 = 0xff;
      }
      if (0xff < (int)uVar4) {
        uVar4 = 0xff;
      }
      *(ushort *)(param_4 + iVar1) =
           (ushort)(((int)uVar8 >> 2) << 5) | (ushort)(((int)uVar10 >> 3) << 0xb) |
           (ushort)((int)uVar4 >> 3);
      iVar1 = iVar1 + 2;
    } while (iVar1 != iVar3 * 2);
  }
  return 1;
}



/* ============================================= */
/* Function: Java_com_android_camera_effect_RealtimeEffect_channelFilter */
/* Address: 0x000127a8 */
/* ============================================= */

undefined4
Java_com_android_camera_effect_RealtimeEffect_channelFilter
          (int *param_1,undefined4 param_2,undefined4 param_3,int param_4,int param_5,
          undefined4 param_6,undefined4 param_7,int param_8)

{
  uchar *puVar1;
  uchar *puVar2;
  undefined4 uVar3;
  ushort uVar4;
  int iVar5;
  int iVar6;
  ushort uVar7;
  uint uVar8;
  int iVar9;
  int iVar10;
  
  puVar1 = (uchar *)(**(code **)(*param_1 + 0x398))(param_1,param_3);
  puVar2 = (uchar *)(**(code **)(*param_1 + 0x398))(param_1,param_7);
  if ((puVar2 == (uchar *)0x0) || (puVar1 == (uchar *)0x0)) {
    uVar3 = 0;
  }
  else if (param_8 == 4) {
    HistogramEqualizationIntensity(puVar1,param_4,param_5,puVar2);
    uVar3 = 1;
  }
  else {
    if (0 < param_5 * param_4) {
      iVar10 = 0;
      do {
        uVar7 = *(ushort *)(puVar1 + iVar10);
        uVar8 = (uint)uVar7;
        iVar5 = (int)uVar8 >> 0xb;
        iVar9 = (uVar8 & 0x1f) * 8;
        iVar6 = (int)(uVar8 & 0x7e0) >> 3;
        switch(param_8) {
        default:
          uVar4 = (ushort)(iVar6 << 3) | (ushort)(iVar5 << 0xb);
          uVar7 = uVar7 & 0x1f;
          break;
        case 1:
          uVar4 = (ushort)(iVar5 << 0xb);
          uVar7 = 0;
          break;
        case 2:
          uVar4 = (ushort)(iVar6 << 3);
          uVar7 = 0;
          break;
        case 3:
          uVar7 = uVar7 & 0x1f;
          uVar4 = 0;
          break;
        case 4:
          iVar5 = __divsi3(iVar6 + iVar5 * 8 + iVar9,3);
          uVar4 = (ushort)((iVar5 >> 2) << 5) | (ushort)((iVar5 >> 3) << 0xb);
          uVar7 = (ushort)(iVar5 >> 3);
          break;
        case 5:
          iVar5 = __divsi3(iVar6 + iVar5 * 8 + iVar9,3);
          uVar8 = iVar5 - 0x32;
          iVar5 = iVar5 + 0x32;
          if (iVar5 < 0) {
            iVar5 = 0;
          }
          else if (0xff < iVar5) {
            iVar5 = 0xff;
          }
          if (0xff < (int)uVar8) {
            uVar4 = (ushort)((iVar5 >> 2) << 5) | (ushort)((iVar5 >> 3) << 0xb);
            uVar7 = 0x1f;
            break;
          }
          *(ushort *)(puVar2 + iVar10) =
               (ushort)(((uVar8 & (int)~uVar8 >> 0x1f) << 0xd) >> 0x10) |
               (ushort)((iVar5 >> 2) << 5) | (ushort)((iVar5 >> 3) << 0xb);
          goto joined_r0x00012882;
        }
        *(ushort *)(puVar2 + iVar10) = uVar7 | uVar4;
joined_r0x00012882:
        iVar10 = iVar10 + 2;
      } while (iVar10 != param_5 * param_4 * 2);
    }
    uVar3 = 1;
  }
  return uVar3;
}



/* ============================================= */
/* Function: Java_com_android_camera_component_ViewFinder_preparePreviewFrame */
/* Address: 0x00012900 */
/* ============================================= */

undefined4
Java_com_android_camera_component_ViewFinder_preparePreviewFrame
          (int *param_1,undefined4 param_2,undefined4 param_3,int param_4,int param_5,
          undefined4 param_6,int param_7,int param_8,char param_9)

{
  int iVar1;
  uchar *puVar2;
  YuvBitmap *this;
  RgbBitmap *this_00;
  Bitmap *this_01;
  undefined4 uVar3;
  YuvBitmap *this_02;
  
  iVar1 = (**(code **)(*param_1 + 0x398))(param_1,param_3);
  puVar2 = (uchar *)(**(code **)(*param_1 + 0x398))(param_1,param_6);
  if (((puVar2 == (uchar *)0x0) || (iVar1 == 0)) ||
     (this = (YuvBitmap *)YuvBitmap::Create(iVar1,param_4,param_5,2), this == (YuvBitmap *)0x0)) {
    uVar3 = 0;
  }
  else {
    this_00 = (RgbBitmap *)RgbBitmap::Create(puVar2,param_7,param_8,0);
    if (this_00 == (RgbBitmap *)0x0) {
      YuvBitmap::~YuvBitmap(this);
      operator_delete(this);
      uVar3 = 0;
    }
    else {
      if ((param_4 != param_8) || (this_01 = (Bitmap *)this, param_5 != param_7)) {
        this_01 = (Bitmap *)YuvBitmap::Scale(this,param_8,param_7);
        if (this_01 == (Bitmap *)0x0) {
          this_02 = this + *(int *)(this + -4) * 0x18;
          while (this != this_02) {
            this_02 = this_02 + -0x18;
            YuvBitmap::~YuvBitmap(this_02);
          }
          operator_delete__(this + -8);
          return 0;
        }
        Bitmap::GetRawData(this_01);
      }
      iVar1 = YuvBitmap::ToRgbBitmap((YuvBitmap *)this_01,this_00);
      if (((iVar1 == 0) ||
          (iVar1 = rotate_rgb_90_clockwise(puVar2,param_8,param_7,puVar2,1), iVar1 == 0)) ||
         ((param_9 == '\x01' &&
          (iVar1 = flip_rgb_vertically(puVar2,param_7,param_8,puVar2,1), iVar1 == 0)))) {
        uVar3 = 0;
      }
      else {
        uVar3 = 1;
      }
      RgbBitmap::~RgbBitmap(this_00);
      operator_delete(this_00);
      if (this_01 != (Bitmap *)this) {
        YuvBitmap::~YuvBitmap((YuvBitmap *)this_01);
        operator_delete(this_01);
      }
      YuvBitmap::~YuvBitmap(this);
      operator_delete(this);
    }
  }
  return uVar3;
}



/* ============================================= */
/* Function: Bitmap */
/* Address: 0x00012a28 */
/* ============================================= */

/* Bitmap::Bitmap(PixelFormat, int, int, bool) */

void __thiscall
Bitmap::Bitmap(Bitmap *this,undefined4 param_2,undefined4 param_3,undefined4 param_4,Bitmap param_5)

{
  *(undefined4 *)this = param_4;
  this[4] = param_5;
  *(undefined4 *)(this + 8) = param_2;
  *(undefined4 *)(this + 0xc) = 0;
  *(undefined4 *)(this + 0x10) = 0;
  *(undefined4 *)(this + 0x14) = param_3;
  return;
}



/* ============================================= */
/* Function: Bitmap */
/* Address: 0x00012a40 */
/* ============================================= */

/* Bitmap::Bitmap(PixelFormat, int, int, bool) */

void __thiscall
Bitmap::Bitmap(Bitmap *this,undefined4 param_2,undefined4 param_3,undefined4 param_4,Bitmap param_5)

{
  *(undefined4 *)this = param_4;
  this[4] = param_5;
  *(undefined4 *)(this + 8) = param_2;
  *(undefined4 *)(this + 0xc) = 0;
  *(undefined4 *)(this + 0x10) = 0;
  *(undefined4 *)(this + 0x14) = param_3;
  return;
}



/* ============================================= */
/* Function: ~Bitmap */
/* Address: 0x00012a58 */
/* ============================================= */

/* Bitmap::~Bitmap() */

void __thiscall Bitmap::~Bitmap(Bitmap *this)

{
  *(undefined4 *)(this + 0xc) = 0;
  *(undefined4 *)(this + 0x10) = 0;
  return;
}



/* ============================================= */
/* Function: ~Bitmap */
/* Address: 0x00012a60 */
/* ============================================= */

/* Bitmap::~Bitmap() */

void __thiscall Bitmap::~Bitmap(Bitmap *this)

{
  *(undefined4 *)(this + 0xc) = 0;
  *(undefined4 *)(this + 0x10) = 0;
  return;
}



/* ============================================= */
/* Function: GetBitsPerPixel */
/* Address: 0x00012a68 */
/* ============================================= */

/* Bitmap::GetBitsPerPixel(PixelFormat) */

undefined4 Bitmap::GetBitsPerPixel(uint param_1)

{
  undefined4 uVar1;
  
  uVar1 = 0;
  if (param_1 < 3) {
    uVar1 = *(undefined4 *)(&DAT_00015b38 + param_1 * 4);
  }
  return uVar1;
}



/* ============================================= */
/* Function: GetHeight */
/* Address: 0x00012a80 */
/* ============================================= */

/* Bitmap::GetHeight() */

undefined4 __thiscall Bitmap::GetHeight(Bitmap *this)

{
  return *(undefined4 *)this;
}



/* ============================================= */
/* Function: GetPixelFormat */
/* Address: 0x00012a84 */
/* ============================================= */

/* Bitmap::GetPixelFormat() */

undefined4 __thiscall Bitmap::GetPixelFormat(Bitmap *this)

{
  return *(undefined4 *)(this + 8);
}



/* ============================================= */
/* Function: GetRawData */
/* Address: 0x00012a88 */
/* ============================================= */

/* Bitmap::GetRawData() */

undefined4 __thiscall Bitmap::GetRawData(Bitmap *this)

{
  return *(undefined4 *)(this + 0xc);
}



/* ============================================= */
/* Function: GetRawDataSize */
/* Address: 0x00012a8c */
/* ============================================= */

/* Bitmap::GetRawDataSize() */

undefined4 __thiscall Bitmap::GetRawDataSize(Bitmap *this)

{
  return *(undefined4 *)(this + 0x10);
}



/* ============================================= */
/* Function: GetWidth */
/* Address: 0x00012a90 */
/* ============================================= */

/* Bitmap::GetWidth() */

undefined4 __thiscall Bitmap::GetWidth(Bitmap *this)

{
  return *(undefined4 *)(this + 0x14);
}



/* ============================================= */
/* Function: Buffer */
/* Address: 0x00012a94 */
/* ============================================= */

/* Buffer::Buffer(unsigned char*, unsigned int) */

void __thiscall Buffer::Buffer(Buffer *this,uchar *param_1,uint param_2)

{
  *(uchar **)this = param_1;
  *(uint *)(this + 4) = param_2;
  return;
}



/* ============================================= */
/* Function: Buffer */
/* Address: 0x00012a9c */
/* ============================================= */

/* Buffer::Buffer(unsigned char*, unsigned int) */

void __thiscall Buffer::Buffer(Buffer *this,uchar *param_1,uint param_2)

{
  *(uchar **)this = param_1;
  *(uint *)(this + 4) = param_2;
  return;
}



/* ============================================= */
/* Function: GetData */
/* Address: 0x00012aa4 */
/* ============================================= */

/* Buffer::GetData() */

undefined4 __thiscall Buffer::GetData(Buffer *this)

{
  return *(undefined4 *)this;
}



/* ============================================= */
/* Function: GetSize */
/* Address: 0x00012aa8 */
/* ============================================= */

/* Buffer::GetSize() */

undefined4 __thiscall Buffer::GetSize(Buffer *this)

{
  return *(undefined4 *)(this + 4);
}



/* ============================================= */
/* Function: ToJavaArray */
/* Address: 0x00012aac */
/* ============================================= */

/* Buffer::ToJavaArray(_JNIEnv*) */

int Buffer::ToJavaArray(_JNIEnv *param_1)

{
  int iVar1;
  void *__dest;
  int *in_r1;
  
  if ((in_r1 != (int *)0x0) && (*(int *)param_1 != 0)) {
    iVar1 = (**(code **)(*in_r1 + 0x2c0))(in_r1,*(undefined4 *)(param_1 + 4));
    if (iVar1 == 0) {
      return 0;
    }
    __dest = (void *)(**(code **)(*in_r1 + 0x2e0))(in_r1,iVar1,0);
    if (__dest != (void *)0x0) {
      memcpy(__dest,*(void **)param_1,*(size_t *)(param_1 + 4));
      (**(code **)(*in_r1 + 0x300))(in_r1,iVar1,__dest,0);
      return iVar1;
    }
  }
  return 0;
}



/* ============================================= */
/* Function: ~Buffer */
/* Address: 0x00012b04 */
/* ============================================= */

/* Buffer::~Buffer() */

Buffer * __thiscall Buffer::~Buffer(Buffer *this)

{
  if (*(void **)this != (void *)0x0) {
    operator_delete__(*(void **)this);
  }
  *(undefined4 *)this = 0;
  *(undefined4 *)(this + 4) = 0;
  return this;
}



/* ============================================= */
/* Function: ~Buffer */
/* Address: 0x00012b1c */
/* ============================================= */

/* Buffer::~Buffer() */

Buffer * __thiscall Buffer::~Buffer(Buffer *this)

{
  if (*(void **)this != (void *)0x0) {
    operator_delete__(*(void **)this);
  }
  *(undefined4 *)this = 0;
  *(undefined4 *)(this + 4) = 0;
  return this;
}



/* ============================================= */
/* Function: Create */
/* Address: 0x00012b34 */
/* ============================================= */

/* Buffer::Create(_JNIEnv*, _jbyteArray*) */

Buffer * Buffer::Create(_JNIEnv *param_1,_jbyteArray *param_2)

{
  uint __n;
  uchar *__dest;
  void *__src;
  Buffer *this;
  
  if ((((param_2 == (_jbyteArray *)0x0) || (param_1 == (_JNIEnv *)0x0)) ||
      (__n = (**(code **)(*(int *)param_1 + 0x2ac))(param_1,param_2), __n == 0)) ||
     (__dest = operator_new__(__n), __dest == (uchar *)0x0)) {
    this = (Buffer *)0x0;
  }
  else {
    __src = (void *)(**(code **)(*(int *)param_1 + 0x2e0))(param_1,param_2,0);
    if (__src == (void *)0x0) {
      operator_delete__(__dest);
      this = (Buffer *)0x0;
    }
    else {
      memcpy(__dest,__src,__n);
      (**(code **)(*(int *)param_1 + 0x300))(param_1,param_2,__src,2);
      this = operator_new(8);
      Buffer(this,__dest,__n);
      if (this == (Buffer *)0x0) {
        operator_delete__(__dest);
      }
    }
  }
  return this;
}



/* ============================================= */
/* Function: Create */
/* Address: 0x00012bc4 */
/* ============================================= */

/* Buffer::Create(unsigned int) */

Buffer * Buffer::Create(uint param_1)

{
  uchar *puVar1;
  Buffer *this;
  
  if ((param_1 == 0) || (puVar1 = operator_new__(param_1), puVar1 == (uchar *)0x0)) {
    this = (Buffer *)0x0;
  }
  else {
    this = operator_new(8);
    Buffer(this,puVar1,param_1);
    if (this == (Buffer *)0x0) {
      operator_delete__(puVar1);
    }
  }
  return this;
}



/* ============================================= */
/* Function: get_image_buffer_size */
/* Address: 0x00012bf8 */
/* ============================================= */

/* get_image_buffer_size(int, int, int) */

int get_image_buffer_size(int param_1,int param_2,int param_3)

{
  int iVar1;
  
  iVar1 = param_2 * param_1 * 2;
  if ((param_3 != 1) && (iVar1 = param_2 * param_1 * 4, param_3 != 2)) {
    iVar1 = -1;
  }
  return iVar1;
}



/* ============================================= */
/* Function: get_ratio_stretch_size */
/* Address: 0x00012c10 */
/* ============================================= */

/* get_ratio_stretch_size(int, int, int, int, int*, int*) */

void get_ratio_stretch_size
               (int param_1,int param_2,int param_3,int param_4,int *param_5,int *param_6)

{
  int iVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  undefined4 uVar4;
  undefined8 uVar5;
  undefined8 uVar6;
  undefined8 uVar7;
  undefined8 uVar8;
  
  if ((((param_2 < 1) || (param_1 < 1)) || (param_4 < 1)) || (param_3 < 1)) {
    *param_5 = 0;
    *param_6 = 0;
  }
  else {
    uVar5 = __aeabi_i2d(param_3);
    uVar2 = (undefined4)((ulonglong)uVar5 >> 0x20);
    uVar6 = __aeabi_i2d(param_1);
    uVar6 = __divdf3((int)uVar6,(int)((ulonglong)uVar6 >> 0x20),(int)uVar5,uVar2);
    uVar7 = __aeabi_i2d(param_4);
    uVar3 = (undefined4)((ulonglong)uVar7 >> 0x20);
    uVar8 = __aeabi_i2d(param_2);
    uVar8 = __divdf3((int)uVar8,(int)((ulonglong)uVar8 >> 0x20),(int)uVar7,uVar3);
    iVar1 = __aeabi_dcmple((int)uVar6,(int)((ulonglong)uVar6 >> 0x20),(int)uVar8,
                           (int)((ulonglong)uVar8 >> 0x20));
    if (iVar1 == 0) {
      uVar6 = uVar8;
    }
    uVar4 = (undefined4)((ulonglong)uVar6 >> 0x20);
    __muldf3((int)uVar6,uVar4,(int)uVar5,uVar2);
    iVar1 = __aeabi_d2iz();
    *param_5 = iVar1;
    __muldf3((int)uVar6,uVar4,(int)uVar7,uVar3);
    iVar1 = __aeabi_d2iz();
    *param_6 = iVar1;
  }
  return;
}



/* ============================================= */
/* Function: scale_yvu420sp */
/* Address: 0x00012cc0 */
/* ============================================= */

/* scale_yvu420sp(unsigned char*, int, int, unsigned char*, int, int) */

undefined4
scale_yvu420sp(uchar *param_1,int param_2,int param_3,uchar *param_4,int param_5,int param_6)

{
  int iVar1;
  uint uVar2;
  int iVar3;
  uint uVar4;
  uchar *puVar5;
  uchar *puVar6;
  undefined8 uVar7;
  undefined8 uVar8;
  undefined8 uVar9;
  int local_4c;
  
  uVar7 = __aeabi_i2d(param_2);
  uVar8 = __aeabi_i2d(param_5);
  uVar7 = __divdf3((int)uVar7,(int)((ulonglong)uVar7 >> 0x20),(int)uVar8,
                   (int)((ulonglong)uVar8 >> 0x20));
  uVar8 = __aeabi_i2d(param_3);
  uVar9 = __aeabi_i2d(param_6);
  uVar8 = __divdf3((int)uVar8,(int)((ulonglong)uVar8 >> 0x20),(int)uVar9,
                   (int)((ulonglong)uVar9 >> 0x20));
  if (0 < param_6) {
    local_4c = 0;
    puVar6 = param_4;
    do {
      uVar9 = __aeabi_i2d(local_4c);
      __muldf3((int)uVar9,(int)((ulonglong)uVar9 >> 0x20),(int)uVar8,(int)((ulonglong)uVar8 >> 0x20)
              );
      iVar1 = __aeabi_d2iz();
      if (param_3 <= iVar1) {
        iVar1 = param_3 + -1;
      }
      iVar3 = (iVar1 >> 1) * param_2 + param_3 * param_2;
      if (0 < param_5) {
        uVar4 = 0;
        puVar5 = param_4 + (local_4c >> 1) * param_5 + param_6 * param_5;
        do {
          uVar9 = __aeabi_i2d(uVar4);
          __muldf3((int)uVar9,(int)((ulonglong)uVar9 >> 0x20),(int)uVar7,
                   (int)((ulonglong)uVar7 >> 0x20));
          uVar2 = __aeabi_d2iz();
          if (param_2 <= (int)uVar2) {
            uVar2 = param_2 - 1;
          }
          puVar6[uVar4] = param_1[uVar2 + iVar1 * param_2];
          if ((uVar4 & 1) == 0) {
            *puVar5 = param_1[(uVar2 & 0xfffffffe) + iVar3];
            puVar5[1] = param_1[(uVar2 & 0xfffffffe) + iVar3 + 1];
            puVar5 = puVar5 + 2;
          }
          uVar4 = uVar4 + 1;
        } while (uVar4 != param_5);
        puVar6 = puVar6 + param_5;
      }
      local_4c = local_4c + 1;
    } while (local_4c != param_6);
  }
  return 1;
}



/* ============================================= */
/* Function: yvu420sp_to_rgb */
/* Address: 0x00012de4 */
/* ============================================= */

/* yvu420sp_to_rgb(unsigned char*, int, int, unsigned char*, int) */

undefined4 yvu420sp_to_rgb(uchar *param_1,int param_2,int param_3,uchar *param_4,int param_5)

{
  int iVar1;
  int iVar2;
  int iVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  int iVar7;
  byte *local_54;
  uchar *local_40;
  int local_34;
  
  if (0 < param_3) {
    local_34 = 0;
    local_40 = param_1;
    do {
      if (0 < param_2) {
        local_54 = param_1 + (local_34 >> 1) * param_2 + param_3 * param_2;
        uVar4 = 0;
        uVar6 = 0;
        uVar5 = 0;
        iVar7 = 0;
        do {
          if ((uVar4 & 1) == 0) {
            uVar6 = (uint)local_54[1];
            uVar5 = (uint)*local_54;
            local_54 = local_54 + 2;
            iVar7 = *(int *)(&DAT_00015b44 + (uVar6 * 0x100 + uVar5) * 4);
          }
          iVar2 = (uint)local_40[uVar4] * 0x100;
          iVar1 = *(int *)(&DAT_00065b44 + (uint)local_40[uVar4] * 4) - iVar7 >> 10;
          iVar3 = 0;
          if ((-1 < iVar1) && (iVar3 = iVar1, 0xff < iVar1)) {
            iVar3 = 0xff;
          }
          if (param_5 == 1) {
            *(ushort *)param_4 =
                 (short)((int)(uint)(byte)(&DAT_00055b44)[iVar2 + uVar5] >> 3) * 0x800 |
                 (ushort)((int)(uint)(byte)(&DAT_00065f44)[iVar2 + uVar6] >> 3) |
                 (ushort)((iVar3 >> 2) << 5);
            param_4 = (uchar *)((int)param_4 + 2);
          }
          else {
            if (param_5 != 2) {
              return 0;
            }
            *(uint *)param_4 =
                 (byte)(&DAT_00055b44)[iVar2 + uVar5] | 0xff000000 |
                 (uint)(byte)(&DAT_00065f44)[iVar2 + uVar6] << 0x10 | iVar3 << 8;
            param_4 = (uchar *)((int)param_4 + 4);
          }
          uVar4 = uVar4 + 1;
        } while (uVar4 != param_2);
        local_40 = local_40 + param_2;
      }
      local_34 = local_34 + 1;
    } while (local_34 != param_3);
  }
  return 1;
}



/* ============================================= */
/* Function: rotate_rgb_90_clockwise */
/* Address: 0x00012f18 */
/* ============================================= */

/* rotate_rgb_90_clockwise(unsigned char*, int, int, unsigned char*, int) */

undefined4
rotate_rgb_90_clockwise(uchar *param_1,int param_2,int param_3,uchar *param_4,int param_5)

{
  uint __n;
  undefined4 uVar1;
  uchar *__src;
  uchar *puVar2;
  uchar *puVar3;
  int iVar4;
  int iVar5;
  uchar *puVar6;
  uchar *puVar7;
  
  __n = get_image_buffer_size(param_2,param_3,param_5);
  if ((__n == 0) ||
     ((__src = param_4, param_1 == param_4 && (__src = operator_new__(__n), __src == (uchar *)0x0)))
     ) {
    uVar1 = 0;
  }
  else {
    if (param_5 == 1) {
      if (0 < param_3) {
        puVar3 = __src + (param_3 + -1) * 2;
        puVar6 = param_1;
        do {
          if (0 < param_2) {
            iVar5 = 0;
            puVar7 = puVar3;
            do {
              puVar2 = puVar6 + iVar5;
              iVar5 = iVar5 + 2;
              *(undefined2 *)puVar7 = *(undefined2 *)puVar2;
              puVar7 = puVar7 + param_3 * 2;
            } while (iVar5 != param_2 * 2);
            puVar6 = puVar6 + param_2 * 2;
          }
          puVar3 = puVar3 + -2;
        } while (puVar3 != __src + -2);
      }
    }
    else if ((param_5 == 2) && (0 < param_3)) {
      puVar3 = __src + (param_3 + -1) * 4;
      iVar5 = 0;
      puVar6 = param_1;
      do {
        if (0 < param_2) {
          iVar4 = 0;
          puVar7 = puVar6;
          puVar2 = puVar3;
          do {
            uVar1 = *(undefined4 *)puVar7;
            puVar7 = puVar7 + 4;
            iVar4 = iVar4 + 1;
            *(undefined4 *)puVar2 = uVar1;
            puVar2 = puVar2 + param_3 * 4;
          } while (iVar4 != param_2);
          puVar6 = puVar6 + param_2 * 4;
        }
        iVar5 = iVar5 + 1;
        puVar3 = puVar3 + -4;
      } while (iVar5 != param_3);
    }
    if ((__src == param_4) || (memcpy(param_1,__src,__n), __src == (uchar *)0x0)) {
      uVar1 = 1;
    }
    else {
      operator_delete__(__src);
      uVar1 = 1;
    }
  }
  return uVar1;
}



/* ============================================= */
/* Function: flip_rgb_vertically */
/* Address: 0x0001302c */
/* ============================================= */

/* flip_rgb_vertically(unsigned char*, int, int, unsigned char*, int) */

undefined4 flip_rgb_vertically(uchar *param_1,int param_2,int param_3,uchar *param_4,int param_5)

{
  uint __n;
  undefined4 uVar1;
  uchar *__src;
  uchar *puVar2;
  uchar *puVar3;
  
  __n = get_image_buffer_size(param_2,param_3,param_5);
  if ((__n == 0) ||
     ((__src = param_4, param_1 == param_4 && (__src = operator_new__(__n), __src == (uchar *)0x0)))
     ) {
    uVar1 = 0;
  }
  else {
    if (param_5 == 1) {
      if (0 < param_3) {
        puVar2 = __src + (param_3 + -1) * param_2 * 2;
        puVar3 = param_1;
        do {
          param_3 = param_3 + -1;
          memcpy(puVar2,puVar3,param_2 * 2);
          puVar3 = puVar3 + param_2 * 2;
          puVar2 = puVar2 + param_2 * -2;
        } while (param_3 != 0);
      }
    }
    else if ((param_5 == 2) && (0 < param_3)) {
      puVar2 = __src + (param_3 + -1) * param_2 * 4;
      puVar3 = param_1;
      do {
        param_3 = param_3 + -1;
        memcpy(puVar2,puVar3,param_2 * 4);
        puVar3 = puVar3 + param_2 * 4;
        puVar2 = puVar2 + param_2 * -4;
      } while (param_3 != 0);
    }
    if ((__src == param_4) || (memcpy(param_1,__src,__n), __src == (uchar *)0x0)) {
      uVar1 = 1;
    }
    else {
      operator_delete__(__src);
      uVar1 = 1;
    }
  }
  return uVar1;
}



/* ============================================= */
/* Function: GetPixels */
/* Address: 0x00013100 */
/* ============================================= */

/* RgbBitmap::GetPixels() */

undefined4 __thiscall RgbBitmap::GetPixels(RgbBitmap *this)

{
  return *(undefined4 *)(this + 0x18);
}



/* ============================================= */
/* Function: GetStride */
/* Address: 0x00013104 */
/* ============================================= */

/* RgbBitmap::GetStride() */

undefined4 __thiscall RgbBitmap::GetStride(RgbBitmap *this)

{
  return *(undefined4 *)(this + 0x1c);
}



/* ============================================= */
/* Function: Rotate90Clockwise */
/* Address: 0x00013108 */
/* ============================================= */

/* RgbBitmap::Rotate90Clockwise(unsigned char*) */

void __thiscall RgbBitmap::Rotate90Clockwise(RgbBitmap *this,uchar *param_1)

{
  undefined2 *puVar1;
  undefined4 *puVar2;
  int iVar3;
  uchar *puVar4;
  uchar *puVar5;
  int iVar6;
  undefined4 *puVar7;
  int iVar8;
  undefined4 uVar9;
  int iVar10;
  
  iVar3 = *(int *)this;
  iVar10 = *(int *)(this + 0x14);
  if (*(int *)(this + 8) == 0) {
    iVar8 = *(int *)(this + 0xc);
    if (0 < iVar3) {
      puVar4 = param_1 + (iVar3 + -1) * 2;
      do {
        if (0 < iVar10) {
          iVar6 = 0;
          puVar5 = puVar4;
          do {
            puVar1 = (undefined2 *)(iVar8 + iVar6);
            iVar6 = iVar6 + 2;
            *(undefined2 *)puVar5 = *puVar1;
            puVar5 = puVar5 + iVar3 * 2;
          } while (iVar6 != iVar10 * 2);
          iVar8 = iVar8 + iVar10 * 2;
        }
        puVar4 = puVar4 + -2;
      } while (puVar4 != param_1 + -2);
    }
  }
  else if ((*(int *)(this + 8) == 1) && (puVar7 = *(undefined4 **)(this + 0xc), 0 < iVar3)) {
    puVar4 = param_1 + (iVar3 + -1) * 4;
    iVar8 = 0;
    do {
      if (0 < iVar10) {
        iVar6 = 0;
        puVar2 = puVar7;
        puVar5 = puVar4;
        do {
          uVar9 = *puVar2;
          puVar2 = puVar2 + 1;
          iVar6 = iVar6 + 1;
          *(undefined4 *)puVar5 = uVar9;
          puVar5 = puVar5 + iVar3 * 4;
        } while (iVar6 != iVar10);
        puVar7 = puVar7 + iVar10;
      }
      iVar8 = iVar8 + 1;
      puVar4 = puVar4 + -4;
    } while (iVar8 != iVar3);
  }
  return;
}



/* ============================================= */
/* Function: Rotate */
/* Address: 0x00013194 */
/* ============================================= */

/* RgbBitmap::Rotate(int) */

void __thiscall RgbBitmap::Rotate(RgbBitmap *this,int param_1)

{
  uchar *__src;
  int iVar1;
  void *pvVar2;
  void *__dest;
  int iVar3;
  
  if (param_1 == 0) {
    return;
  }
  __src = operator_new__(*(uint *)(this + 0x10));
  if (__src == (uchar *)0x0) {
    __android_log_print(6,"libhtccamera","RgbBitmap::Rotate() - Cannot allocate temp buffer");
    return;
  }
  if ((param_1 == 0xb4) || (param_1 == 0x10e)) {
    if (param_1 == 0x10e) goto LAB_000131c0;
  }
  else {
    if (param_1 != 0x5a) {
      return;
    }
    Rotate90Clockwise(this,__src);
LAB_000131c0:
    iVar3 = *(int *)this;
    *(undefined4 *)this = *(undefined4 *)(this + 0x14);
    *(int *)(this + 0x14) = iVar3;
    iVar1 = Bitmap::GetBitsPerPixel(*(undefined4 *)(this + 8));
    *(uint *)(this + 0x1c) = (uint)(iVar1 * iVar3) >> 3;
    if (*(void **)(this + 0x18) != (void *)0x0) {
      operator_delete__(*(void **)(this + 0x18));
    }
    pvVar2 = operator_new__(*(int *)this << 2);
    *(void **)(this + 0x18) = pvVar2;
    if (pvVar2 == (void *)0x0) {
      return;
    }
    __dest = *(void **)(this + 0xc);
    if (*(int *)this < 1) goto LAB_00013210;
    iVar1 = 0;
    while( true ) {
      *(void **)(iVar1 * 4 + (int)pvVar2) = __dest;
      iVar1 = iVar1 + 1;
      __dest = (void *)((int)__dest + *(int *)(this + 0x1c));
      if (*(int *)this <= iVar1) break;
      pvVar2 = *(void **)(this + 0x18);
    }
  }
  __dest = *(void **)(this + 0xc);
LAB_00013210:
  memcpy(__dest,__src,*(size_t *)(this + 0x10));
  operator_delete__(__src);
  return;
}



/* ============================================= */
/* Function: FlipY */
/* Address: 0x00013244 */
/* ============================================= */

/* RgbBitmap::FlipY() */

void __thiscall RgbBitmap::FlipY(RgbBitmap *this)

{
  void *__src;
  void *__src_00;
  int iVar1;
  int iVar2;
  void *pvVar3;
  
  __src = operator_new__(*(uint *)(this + 0x10));
  if (__src == (void *)0x0) {
    __android_log_print(6,"libhtccamera","RgbBitmap::FlipY() - Cannot allocate temp buffer");
    return;
  }
  iVar1 = *(int *)(this + 0x14);
  iVar2 = *(int *)this;
  if (*(int *)(this + 8) == 0) {
    __src_00 = *(void **)(this + 0xc);
    if (iVar2 < 1) goto LAB_0001329e;
    pvVar3 = (void *)((iVar2 + -1) * iVar1 * 2 + (int)__src);
    do {
      memcpy(pvVar3,__src_00,iVar1 * 2);
      iVar2 = iVar2 + -1;
      __src_00 = (void *)((int)__src_00 + iVar1 * 2);
      pvVar3 = (void *)((int)pvVar3 + iVar1 * -2);
    } while (iVar2 != 0);
  }
  else if (*(int *)(this + 8) == 1) {
    __src_00 = *(void **)(this + 0xc);
    if (iVar2 < 1) goto LAB_0001329e;
    pvVar3 = (void *)((iVar2 + -1) * iVar1 * 4 + (int)__src);
    do {
      memcpy(pvVar3,__src_00,iVar1 * 4);
      iVar2 = iVar2 + -1;
      __src_00 = (void *)((int)__src_00 + iVar1 * 4);
      pvVar3 = (void *)((int)pvVar3 + iVar1 * -4);
    } while (iVar2 != 0);
  }
  __src_00 = *(void **)(this + 0xc);
LAB_0001329e:
  memcpy(__src_00,__src,*(size_t *)(this + 0x10));
  operator_delete__(__src);
  return;
}



/* ============================================= */
/* Function: ~RgbBitmap */
/* Address: 0x0001330c */
/* ============================================= */

/* RgbBitmap::~RgbBitmap() */

RgbBitmap * __thiscall RgbBitmap::~RgbBitmap(RgbBitmap *this)

{
  if (*(void **)(this + 0x18) != (void *)0x0) {
    operator_delete__(*(void **)(this + 0x18));
    *(undefined4 *)(this + 0x18) = 0;
  }
  *(undefined4 *)(this + 0x1c) = 0;
  if ((*(void **)(this + 0xc) != (void *)0x0) && (this[4] != (RgbBitmap)0x0)) {
    operator_delete__(*(void **)(this + 0xc));
  }
  Bitmap::~Bitmap((Bitmap *)this);
  Bitmap::~Bitmap((Bitmap *)this);
  return this;
}



/* ============================================= */
/* Function: ~RgbBitmap */
/* Address: 0x00013344 */
/* ============================================= */

/* RgbBitmap::~RgbBitmap() */

RgbBitmap * __thiscall RgbBitmap::~RgbBitmap(RgbBitmap *this)

{
  if (*(void **)(this + 0x18) != (void *)0x0) {
    operator_delete__(*(void **)(this + 0x18));
    *(undefined4 *)(this + 0x18) = 0;
  }
  *(undefined4 *)(this + 0x1c) = 0;
  if ((*(void **)(this + 0xc) != (void *)0x0) && (this[4] != (RgbBitmap)0x0)) {
    operator_delete__(*(void **)(this + 0xc));
  }
  Bitmap::~Bitmap((Bitmap *)this);
  Bitmap::~Bitmap((Bitmap *)this);
  return this;
}



/* ============================================= */
/* Function: RgbBitmap */
/* Address: 0x0001337c */
/* ============================================= */

/* RgbBitmap::RgbBitmap(PixelFormat, int, int, unsigned char*, unsigned int, bool) */

RgbBitmap * __thiscall RgbBitmap::RgbBitmap(RgbBitmap *this)

{
  undefined4 in_stack_00000000;
  undefined4 in_stack_00000004;
  
  Bitmap::Bitmap((Bitmap *)this);
  *(undefined4 *)(this + 0xc) = in_stack_00000000;
  *(undefined4 *)(this + 0x10) = in_stack_00000004;
  return this;
}



/* ============================================= */
/* Function: Create */
/* Address: 0x0001339c */
/* ============================================= */

/* RgbBitmap::Create(unsigned char*, int, int, PixelFormat, bool) */

RgbBitmap * RgbBitmap::Create(void *param_1,int param_2,int param_3,int param_4,undefined1 param_5)

{
  undefined4 *puVar1;
  RgbBitmap *pRVar2;
  undefined4 *puVar3;
  void *pvVar4;
  int iVar5;
  int iVar6;
  
  iVar5 = 2;
  if (param_4 != 0) {
    if (param_4 != 1) {
      return (RgbBitmap *)0x0;
    }
    iVar5 = 4;
  }
  iVar5 = iVar5 * param_2;
  if (iVar5 * param_3 != 0) {
    puVar1 = operator_new__(param_3 << 2);
    if (puVar1 != (undefined4 *)0x0) {
      if (0 < param_3) {
        iVar6 = 0;
        puVar3 = puVar1;
        pvVar4 = param_1;
        do {
          iVar6 = iVar6 + 1;
          *puVar3 = pvVar4;
          puVar3 = puVar3 + 1;
          pvVar4 = (void *)((int)pvVar4 + iVar5);
        } while (iVar6 != param_3);
      }
      pRVar2 = operator_new(0x20);
      RgbBitmap(pRVar2,param_4,param_2,param_3,param_1,iVar5 * param_3,param_5);
      if (pRVar2 == (RgbBitmap *)0x0) {
        operator_delete__(puVar1);
        return (RgbBitmap *)0x0;
      }
      *(int *)(pRVar2 + 0x1c) = iVar5;
      *(undefined4 **)(pRVar2 + 0x18) = puVar1;
      return pRVar2;
    }
    if (param_1 != (void *)0x0) {
      operator_delete__(param_1);
      return (RgbBitmap *)0x0;
    }
  }
  return (RgbBitmap *)0x0;
}



/* ============================================= */
/* Function: Create */
/* Address: 0x00013448 */
/* ============================================= */

/* RgbBitmap::Create(int, int, PixelFormat) */

int RgbBitmap::Create(int param_1,int param_2,int param_3)

{
  uint uVar1;
  void *pvVar2;
  int iVar3;
  
  if ((0 < param_2) && (0 < param_1)) {
    iVar3 = 2;
    if (param_3 != 0) {
      if (param_3 != 1) {
        return 0;
      }
      iVar3 = 4;
    }
    uVar1 = param_2 * param_1 * iVar3;
    if ((uVar1 != 0) && (pvVar2 = operator_new__(uVar1), pvVar2 != (void *)0x0)) {
      iVar3 = Create(pvVar2,param_1,param_2,param_3,1);
      if (iVar3 == 0) {
        operator_delete__(pvVar2);
        return 0;
      }
      return iVar3;
    }
  }
  return 0;
}



/* ============================================= */
/* Function: Create */
/* Address: 0x000134a0 */
/* ============================================= */

/* RgbBitmap::Create(RgbBitmap*) */

Bitmap * RgbBitmap::Create(RgbBitmap *param_1)

{
  undefined4 uVar1;
  undefined4 uVar2;
  undefined4 uVar3;
  void *__dest;
  void *__src;
  size_t __n;
  Bitmap *this;
  
  if ((param_1 == (RgbBitmap *)0x0) || (*(int *)(param_1 + 0xc) == 0)) {
    this = (Bitmap *)0x0;
  }
  else {
    uVar1 = Bitmap::GetWidth((Bitmap *)param_1);
    uVar2 = Bitmap::GetHeight((Bitmap *)param_1);
    uVar3 = Bitmap::GetPixelFormat((Bitmap *)param_1);
    this = (Bitmap *)Create(uVar1,uVar2,uVar3);
    if (this != (Bitmap *)0x0) {
      __dest = (void *)Bitmap::GetRawData(this);
      __src = (void *)Bitmap::GetRawData((Bitmap *)param_1);
      __n = Bitmap::GetRawDataSize((Bitmap *)param_1);
      memcpy(__dest,__src,__n);
    }
  }
  return this;
}



/* ============================================= */
/* Function: Create */
/* Address: 0x000134f8 */
/* ============================================= */

/* RgbBitmap::Create(unsigned char*, int, int, PixelFormat) */

undefined4 RgbBitmap::Create(int param_1,int param_2,int param_3)

{
  undefined4 uVar1;
  
  if (((param_2 < 1) || (param_1 == 0)) || (param_3 < 1)) {
    uVar1 = 0;
  }
  else {
    uVar1 = Create();
  }
  return uVar1;
}



/* ============================================= */
/* Function: RgbBitmap */
/* Address: 0x00013518 */
/* ============================================= */

/* RgbBitmap::RgbBitmap(PixelFormat, int, int, unsigned char*, unsigned int, bool) */

RgbBitmap * __thiscall RgbBitmap::RgbBitmap(RgbBitmap *this)

{
  undefined4 in_stack_00000000;
  undefined4 in_stack_00000004;
  
  Bitmap::Bitmap((Bitmap *)this);
  *(undefined4 *)(this + 0xc) = in_stack_00000000;
  *(undefined4 *)(this + 0x10) = in_stack_00000004;
  return this;
}



/* ============================================= */
/* Function: GetUVData */
/* Address: 0x00013538 */
/* ============================================= */

/* YuvBitmap::GetUVData() */

int __thiscall YuvBitmap::GetUVData(YuvBitmap *this)

{
  return *(int *)(this + 0xc) + *(int *)this * *(int *)(this + 0x14);
}



/* ============================================= */
/* Function: GetYData */
/* Address: 0x00013548 */
/* ============================================= */

/* YuvBitmap::GetYData() */

undefined4 __thiscall YuvBitmap::GetYData(YuvBitmap *this)

{
  return *(undefined4 *)(this + 0xc);
}



/* ============================================= */
/* Function: ToRgbBitmap */
/* Address: 0x0001354c */
/* ============================================= */

/* YuvBitmap::ToRgbBitmap(RgbBitmap*) */

undefined4 __thiscall YuvBitmap::ToRgbBitmap(YuvBitmap *this,RgbBitmap *param_1)

{
  int iVar1;
  uint uVar2;
  undefined4 uVar3;
  int iVar4;
  int iVar5;
  int iVar6;
  int iVar7;
  int iVar8;
  int iVar9;
  uint uVar10;
  uint uVar11;
  byte *local_4c;
  uint *local_3c;
  int local_38;
  int local_34;
  
  if (*(int *)(this + 0xc) == 0) {
    __android_log_print(6,"libhtccamera","YuvBitmap::ToRgbBitmap() - _RawData = null");
    uVar3 = 0;
  }
  else if (param_1 == (RgbBitmap *)0x0) {
    __android_log_print(6,"libhtccamera","YuvBitmap::ToRgbBitmap() - bitmap = null");
    uVar3 = 0;
  }
  else {
    iVar1 = Bitmap::GetPixelFormat((Bitmap *)param_1);
    iVar8 = *(int *)this;
    iVar5 = *(int *)(this + 0x14);
    local_38 = *(int *)(this + 0xc);
    local_3c = (uint *)Bitmap::GetRawData((Bitmap *)param_1);
    iVar9 = *(int *)this;
    if (0 < iVar9) {
      iVar6 = *(int *)(this + 0x14);
      local_34 = 0;
      do {
        if (0 < iVar6) {
          local_4c = (byte *)(*(int *)(this + 0xc) + (local_34 >> 1) * iVar6 + iVar8 * iVar5);
          uVar10 = 0;
          uVar11 = 0;
          uVar2 = 0;
          iVar9 = 0;
          do {
            if ((uVar10 & 1) == 0) {
              uVar11 = (uint)local_4c[1];
              uVar2 = (uint)*local_4c;
              local_4c = local_4c + 2;
              iVar9 = *(int *)(&DAT_00075fac + (uVar11 * 0x100 + uVar2) * 4);
            }
            iVar7 = (uint)*(byte *)(local_38 + uVar10) * 0x100;
            iVar4 = *(int *)(&DAT_000c5fac + (uint)*(byte *)(local_38 + uVar10) * 4) - iVar9 >> 10;
            iVar6 = 0;
            if ((-1 < iVar4) && (iVar6 = iVar4, 0xff < iVar4)) {
              iVar6 = 0xff;
            }
            if (iVar1 == 0) {
              *(ushort *)local_3c =
                   (short)((int)(uint)(byte)(&DAT_000b5fac)[iVar7 + uVar2] >> 3) * 0x800 |
                   (ushort)((int)(uint)(byte)(&DAT_000c63ac)[iVar7 + uVar11] >> 3) |
                   (ushort)((iVar6 >> 2) << 5);
              local_3c = (uint *)((int)local_3c + 2);
            }
            else {
              if (iVar1 != 1) {
                return 0;
              }
              *local_3c = (byte)(&DAT_000b5fac)[iVar7 + uVar2] | 0xff000000 |
                          (uint)(byte)(&DAT_000c63ac)[iVar7 + uVar11] << 0x10 | iVar6 << 8;
              local_3c = local_3c + 1;
            }
            iVar6 = *(int *)(this + 0x14);
            uVar10 = uVar10 + 1;
          } while ((int)uVar10 < iVar6);
          local_38 = local_38 + uVar10;
          iVar9 = *(int *)this;
        }
        local_34 = local_34 + 1;
      } while (local_34 < iVar9);
    }
    uVar3 = 1;
  }
  return uVar3;
}



/* ============================================= */
/* Function: ~YuvBitmap */
/* Address: 0x000136dc */
/* ============================================= */

/* YuvBitmap::~YuvBitmap() */

YuvBitmap * __thiscall YuvBitmap::~YuvBitmap(YuvBitmap *this)

{
  if ((*(void **)(this + 0xc) != (void *)0x0) && (this[4] != (YuvBitmap)0x0)) {
    operator_delete__(*(void **)(this + 0xc));
  }
  Bitmap::~Bitmap((Bitmap *)this);
  Bitmap::~Bitmap((Bitmap *)this);
  return this;
}



/* ============================================= */
/* Function: ~YuvBitmap */
/* Address: 0x00013700 */
/* ============================================= */

/* YuvBitmap::~YuvBitmap() */

YuvBitmap * __thiscall YuvBitmap::~YuvBitmap(YuvBitmap *this)

{
  if ((*(void **)(this + 0xc) != (void *)0x0) && (this[4] != (YuvBitmap)0x0)) {
    operator_delete__(*(void **)(this + 0xc));
  }
  Bitmap::~Bitmap((Bitmap *)this);
  Bitmap::~Bitmap((Bitmap *)this);
  return this;
}



/* ============================================= */
/* Function: YuvBitmap */
/* Address: 0x00013724 */
/* ============================================= */

/* YuvBitmap::YuvBitmap(unsigned char*, int, int, PixelFormat, bool) */

YuvBitmap * __thiscall
YuvBitmap::YuvBitmap
          (YuvBitmap *this,undefined4 param_1,int param_2,int param_3,undefined4 param_5,
          undefined1 param_6)

{
  Bitmap::Bitmap((Bitmap *)this,param_5,param_2,param_3,param_6);
  *(undefined4 *)(this + 0xc) = param_1;
  *(int *)(this + 0x10) = param_2 * 3 * param_3 >> 1;
  return this;
}



/* ============================================= */
/* Function: Create */
/* Address: 0x00013754 */
/* ============================================= */

/* YuvBitmap::Create(unsigned char*, int, int, PixelFormat) */

YuvBitmap * YuvBitmap::Create(int param_1,uint param_2,uint param_3,int param_4)

{
  YuvBitmap *pYVar1;
  
  if (((((int)param_2 < 1) || (param_1 == 0)) || ((int)param_3 < 1)) ||
     ((((param_3 | param_2) & 3) != 0 || (param_4 != 2)))) {
    pYVar1 = (YuvBitmap *)0x0;
  }
  else {
    pYVar1 = operator_new(0x18);
    YuvBitmap(pYVar1,param_1,param_2,param_3,2,0);
  }
  return pYVar1;
}



/* ============================================= */
/* Function: Create */
/* Address: 0x000137ac */
/* ============================================= */

/* YuvBitmap::Create(int, int, PixelFormat) */

YuvBitmap * YuvBitmap::Create(uint param_1,uint param_2,int param_3)

{
  void *__s;
  YuvBitmap *pYVar1;
  uint __n;
  
  if ((((0 < (int)param_2) && (0 < (int)param_1)) && (((param_2 | param_1) & 3) == 0)) &&
     (param_3 == 2)) {
    __n = (int)(param_1 * 3 * param_2) >> 1;
    __s = operator_new__(__n);
    if (__s != (void *)0x0) {
      memset(__s,0,__n);
      pYVar1 = operator_new(0x18);
      YuvBitmap(pYVar1,__s,param_1,param_2,2,1);
      if (pYVar1 != (YuvBitmap *)0x0) {
        return pYVar1;
      }
      operator_delete__(__s);
      return (YuvBitmap *)0x0;
    }
  }
  return (YuvBitmap *)0x0;
}



/* ============================================= */
/* Function: Scale */
/* Address: 0x00013820 */
/* ============================================= */

/* YuvBitmap::Scale(int, int) */

Bitmap * __thiscall YuvBitmap::Scale(YuvBitmap *this,int param_1,int param_2)

{
  int iVar1;
  int iVar2;
  int iVar3;
  int iVar4;
  uint uVar5;
  int iVar6;
  uint uVar7;
  int iVar8;
  int iVar9;
  undefined1 *puVar10;
  int iVar11;
  undefined8 uVar12;
  undefined8 uVar13;
  undefined8 uVar14;
  int local_50;
  Bitmap *local_44;
  
  if (*(int *)(this + 0xc) == 0) {
    __android_log_print(6,"libhtccamera","YuvBitmap::Scale() - _RawData = null");
    local_44 = (Bitmap *)0x0;
  }
  else if (((param_2 | param_1) & 3U) == 0) {
    local_44 = (Bitmap *)Create(param_1,param_2,*(undefined4 *)(this + 8));
    if (local_44 != (Bitmap *)0x0) {
      iVar1 = *(int *)(this + 0x14);
      iVar9 = *(int *)this;
      uVar12 = __aeabi_i2d();
      uVar13 = __aeabi_i2d(param_1);
      uVar12 = __divdf3((int)uVar12,(int)((ulonglong)uVar12 >> 0x20),(int)uVar13,
                        (int)((ulonglong)uVar13 >> 0x20));
      uVar13 = __aeabi_i2d(iVar9);
      uVar14 = __aeabi_i2d(param_2);
      uVar13 = __divdf3((int)uVar13,(int)((ulonglong)uVar13 >> 0x20),(int)uVar14,
                        (int)((ulonglong)uVar14 >> 0x20));
      iVar2 = Bitmap::GetRawData((Bitmap *)this);
      iVar3 = Bitmap::GetRawData(local_44);
      if (0 < param_2) {
        local_50 = 0;
        iVar11 = iVar3;
        do {
          uVar14 = __aeabi_i2d(local_50);
          __muldf3((int)uVar14,(int)((ulonglong)uVar14 >> 0x20),(int)uVar13,
                   (int)((ulonglong)uVar13 >> 0x20));
          iVar4 = __aeabi_d2iz();
          if (*(int *)this <= iVar4) {
            iVar4 = *(int *)this + -1;
          }
          iVar8 = *(int *)(this + 0x14);
          iVar6 = iVar4 * iVar8;
          iVar4 = iVar2 + (iVar4 >> 1) * iVar8 + iVar9 * iVar1;
          if (0 < param_1) {
            uVar7 = 0;
            puVar10 = (undefined1 *)(iVar3 + (local_50 >> 1) * param_1 + param_2 * param_1);
            while( true ) {
              uVar14 = __aeabi_i2d(uVar7);
              __muldf3((int)uVar14,(int)((ulonglong)uVar14 >> 0x20),(int)uVar12,
                       (int)((ulonglong)uVar12 >> 0x20));
              uVar5 = __aeabi_d2iz();
              if (iVar8 <= (int)uVar5) {
                uVar5 = iVar8 - 1;
              }
              *(undefined1 *)(iVar11 + uVar7) = *(undefined1 *)(iVar2 + iVar6 + uVar5);
              if ((uVar7 & 1) == 0) {
                *puVar10 = *(undefined1 *)(iVar4 + (uVar5 & 0xfffffffe));
                puVar10[1] = *(undefined1 *)(iVar4 + (uVar5 & 0xfffffffe) + 1);
                puVar10 = puVar10 + 2;
              }
              uVar7 = uVar7 + 1;
              if (uVar7 == param_1) break;
              iVar8 = *(int *)(this + 0x14);
            }
            iVar11 = iVar11 + param_1;
          }
          local_50 = local_50 + 1;
        } while (local_50 != param_2);
      }
    }
  }
  else {
    __android_log_print(6,"libhtccamera","YuvBitmap::Scale() - newWidth = %d, newHeight = %d",
                        param_1,param_2);
    local_44 = (Bitmap *)0x0;
  }
  return local_44;
}



/* ============================================= */
/* Function: YuvBitmap */
/* Address: 0x000139b0 */
/* ============================================= */

/* YuvBitmap::YuvBitmap(unsigned char*, int, int, PixelFormat, bool) */

YuvBitmap * __thiscall
YuvBitmap::YuvBitmap
          (YuvBitmap *this,undefined4 param_1,int param_2,int param_3,undefined4 param_5,
          undefined1 param_6)

{
  Bitmap::Bitmap((Bitmap *)this,param_5,param_2,param_3,param_6);
  *(undefined4 *)(this + 0xc) = param_1;
  *(int *)(this + 0x10) = param_2 * 3 * param_3 >> 1;
  return this;
}



/* ============================================= */
/* Function: __divsi3 */
/* Address: 0x000139e0 */
/* ============================================= */

uint __divsi3(uint param_1,uint param_2)

{
  uint uVar1;
  uint uVar2;
  uint uVar3;
  uint uVar4;
  uint uVar5;
  bool bVar6;
  
  uVar5 = param_1 ^ param_2;
  if (param_2 == 0) {
    __div0();
    return 0;
  }
  uVar2 = param_2;
  if ((int)param_2 < 0) {
    uVar2 = -param_2;
  }
  if (uVar2 - 1 == 0) {
    if ((int)param_2 < 0) {
      param_1 = -param_1;
    }
    return param_1;
  }
  uVar4 = param_1;
  if ((int)param_1 < 0) {
    uVar4 = -param_1;
  }
  if (uVar4 <= uVar2) {
    if (uVar4 < uVar2) {
      param_1 = 0;
    }
    if (uVar4 == uVar2) {
      param_1 = (int)uVar5 >> 0x1f | 1;
    }
    return param_1;
  }
  if ((uVar2 & uVar2 - 1) == 0) {
    uVar4 = uVar4 >> (0x1fU - LZCOUNT(uVar2) & 0xff);
    if ((int)uVar5 < 0) {
      uVar4 = -uVar4;
    }
    return uVar4;
  }
  uVar3 = uVar2 << (LZCOUNT(uVar2) - LZCOUNT(uVar4) & 0xffU);
  uVar2 = 1 << (LZCOUNT(uVar2) - LZCOUNT(uVar4) & 0xffU);
  uVar1 = 0;
  while( true ) {
    if (uVar3 <= uVar4) {
      uVar4 = uVar4 - uVar3;
      uVar1 = uVar1 | uVar2;
    }
    if (uVar3 >> 1 <= uVar4) {
      uVar4 = uVar4 - (uVar3 >> 1);
      uVar1 = uVar1 | uVar2 >> 1;
    }
    if (uVar3 >> 2 <= uVar4) {
      uVar4 = uVar4 - (uVar3 >> 2);
      uVar1 = uVar1 | uVar2 >> 2;
    }
    if (uVar3 >> 3 <= uVar4) {
      uVar4 = uVar4 - (uVar3 >> 3);
      uVar1 = uVar1 | uVar2 >> 3;
    }
    bVar6 = uVar4 == 0;
    if (!bVar6) {
      uVar2 = uVar2 >> 4;
      bVar6 = uVar2 == 0;
    }
    if (bVar6) break;
    uVar3 = uVar3 >> 4;
  }
  if ((int)uVar5 < 0) {
    uVar1 = -uVar1;
  }
  return uVar1;
}



/* ============================================= */
/* Function: __aeabi_idivmod */
/* Address: 0x00013abc */
/* ============================================= */

void __aeabi_idivmod(void)

{
  __divsi3();
  return;
}



/* ============================================= */
/* Function: __aeabi_drsub */
/* Address: 0x00013ad4 */
/* ============================================= */

void __aeabi_drsub(undefined4 param_1,uint param_2)

{
  __aeabi_dadd(param_1,param_2 ^ 0x80000000);
  return;
}



/* ============================================= */
/* Function: __subdf3 */
/* Address: 0x00013adc */
/* ============================================= */

ulonglong __subdf3(uint param_1,uint param_2,uint param_3,uint param_4)

{
  int iVar1;
  byte bVar2;
  byte bVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  uint uVar7;
  uint uVar8;
  uint uVar9;
  int iVar10;
  uint uVar11;
  uint uVar12;
  uint uVar13;
  bool bVar14;
  bool bVar15;
  
  uVar8 = param_4 ^ 0x80000000;
  uVar11 = param_2 << 1;
  param_4 = param_4 << 1;
  iVar10 = (int)uVar11 >> 0x15;
  iVar1 = (int)param_4 >> 0x15;
  if ((((uVar11 == param_4 && param_1 == param_3 || uVar11 == 0 && param_1 == 0) ||
       param_4 == 0 && param_3 == 0) || iVar10 == -1) || iVar1 == -1) {
    if (iVar10 == -1 || iVar1 == -1) {
      uVar11 = param_3;
      uVar12 = uVar8;
      if (iVar10 == -1) {
        uVar11 = param_1;
        uVar12 = param_2;
      }
      if (iVar10 != -1 || iVar1 != -1) {
        param_3 = uVar11;
        uVar8 = uVar12;
      }
      bVar14 = (uVar12 & 0xfffff) == 0;
      bVar15 = uVar11 == 0 && bVar14;
      if (uVar11 == 0 && bVar14) {
        bVar15 = param_3 == 0 && (uVar8 & 0xfffff) == 0;
      }
      if (!bVar15 || uVar12 != uVar8) {
        uVar12 = uVar12 | 0x80000;
      }
      return CONCAT44(uVar12,uVar11);
    }
    if (uVar11 != param_4 || param_1 != param_3) {
      if (uVar11 == 0 && param_1 == 0) {
        param_1 = param_3;
        param_2 = uVar8;
      }
      return CONCAT44(param_2,param_1);
    }
    if (param_2 != uVar8) {
      return 0;
    }
    if (uVar11 >> 0x15 == 0) {
      bVar2 = (byte)(param_1 >> 0x1f);
      uVar11 = param_2 * 2 + (uint)bVar2;
      if (CARRY4(param_2,param_2) || CARRY4(param_2 * 2,(uint)bVar2)) {
        uVar11 = uVar11 | 0x80000000;
      }
      return CONCAT44(uVar11,param_1 << 1);
    }
    if (uVar11 < 0xffc00000) {
      return CONCAT44(param_2 + 0x100000,param_1);
    }
    param_2 = param_2 & 0x80000000;
LAB_00013d50:
    return (ulonglong)(param_2 | 0x7ff00000) << 0x20;
  }
  uVar11 = uVar11 >> 0x15;
  param_4 = param_4 >> 0x15;
  uVar12 = param_4 - uVar11;
  bVar14 = uVar12 != 0;
  if (param_4 < uVar11) {
    uVar12 = -uVar12;
  }
  uVar6 = param_1;
  uVar9 = param_2;
  if (bVar14 && uVar11 <= param_4) {
    uVar11 = uVar11 + uVar12;
    uVar6 = param_3;
    uVar9 = uVar8;
    param_3 = param_1;
    uVar8 = param_2;
  }
  if (0x36 < uVar12) {
    return CONCAT44(uVar9,uVar6);
  }
  uVar5 = uVar9 & 0xfffff | 0x100000;
  if ((uVar9 & 0x80000000) != 0) {
    bVar14 = uVar6 != 0;
    uVar6 = -uVar6;
    uVar5 = -(uVar5 + bVar14);
  }
  uVar9 = uVar8 & 0xfffff | 0x100000;
  if ((uVar8 & 0x80000000) != 0) {
    bVar14 = param_3 != 0;
    param_3 = -param_3;
    uVar9 = -(uVar9 + bVar14);
  }
  if (uVar11 == uVar12) {
    uVar9 = uVar9 ^ 0x100000;
    if (uVar11 == 0) {
      uVar5 = uVar5 ^ 0x100000;
      uVar11 = 1;
    }
    else {
      uVar12 = uVar12 - 1;
    }
  }
  uVar8 = -uVar12 + 0x20;
  if ((int)uVar12 < 0x21) {
    uVar13 = param_3 << (uVar8 & 0xff);
    param_3 = param_3 >> (uVar12 & 0xff);
    uVar4 = uVar6 + param_3;
    uVar7 = uVar9 << (uVar8 & 0xff);
    uVar8 = uVar4 + uVar7;
    uVar12 = uVar5 + CARRY4(uVar6,param_3) + ((int)uVar9 >> (uVar12 & 0xff)) +
             (uint)CARRY4(uVar4,uVar7);
  }
  else {
    uVar13 = uVar9 << (-uVar12 + 0x40 & 0xff);
    if (param_3 != 0) {
      uVar13 = uVar13 | 2;
    }
    uVar12 = (int)uVar9 >> (uVar12 - 0x20 & 0xff);
    uVar8 = uVar6 + uVar12;
    uVar12 = uVar5 + ((int)uVar9 >> 0x1f) + (uint)CARRY4(uVar6,uVar12);
  }
  param_2 = uVar12 & 0x80000000;
  uVar6 = uVar12;
  if ((int)uVar12 < 0) {
    bVar14 = uVar13 == 0;
    uVar13 = -uVar13;
    uVar6 = -uVar8;
    uVar8 = -(uVar8 + !bVar14);
    uVar6 = -(uVar12 + (bVar14 <= uVar6));
  }
  if (0xfffff < uVar6) {
    uVar9 = uVar11 - 1;
    if (0x1fffff < uVar6) {
      bVar2 = (byte)uVar6;
      uVar6 = uVar6 >> 1;
      bVar3 = (byte)uVar8;
      uVar8 = (uint)(bVar2 & 1) << 0x1f | uVar8 >> 1;
      uVar13 = (uint)(bVar3 & 1) << 0x1f | uVar13 >> 1;
      uVar9 = uVar11;
      if (0xffbfffff < uVar11 * 0x200000) goto LAB_00013d50;
    }
LAB_00013bf8:
    bVar14 = 0x7fffffff < uVar13;
    if (uVar13 == 0x80000000) {
      bVar14 = (bool)((byte)uVar8 & 1);
    }
    return CONCAT44(uVar6 + uVar9 * 0x100000 + (uint)CARRY4(uVar8,(uint)bVar14) | param_2,
                    uVar8 + bVar14);
  }
  bVar2 = (byte)(uVar13 >> 0x1f);
  uVar13 = uVar13 << 1;
  uVar9 = uVar8 * 2;
  bVar14 = CARRY4(uVar8,uVar8);
  uVar8 = uVar8 * 2 + (uint)bVar2;
  uVar6 = uVar6 * 2 + (uint)(bVar14 || CARRY4(uVar9,(uint)bVar2));
  uVar9 = uVar11 - 2;
  if ((uVar6 & 0x100000) != 0) goto LAB_00013bf8;
  uVar11 = uVar8;
  uVar5 = uVar6;
  if (uVar6 == 0) {
    uVar11 = 0;
    uVar5 = uVar8;
  }
  iVar10 = LZCOUNT(uVar5);
  if (uVar6 == 0) {
    iVar10 = iVar10 + 0x20;
  }
  uVar4 = iVar10 - 0xb;
  bVar15 = SBORROW4(uVar4,0x20);
  uVar7 = iVar10 - 0x2b;
  bVar14 = uVar7 == 0;
  uVar8 = uVar7;
  uVar6 = uVar7;
  if ((int)uVar4 < 0x20) {
    bVar15 = SCARRY4(uVar7,0xc);
    uVar6 = iVar10 - 0x1f;
    bVar14 = uVar6 == 0;
    uVar8 = uVar4;
    if (!bVar14 && -0xd < (int)uVar7) {
      uVar11 = uVar5 << (uVar4 & 0xff);
      uVar5 = uVar5 >> (0xc - uVar6 & 0xff);
      goto LAB_00013c78;
    }
  }
  if (bVar14 || (int)uVar6 < 0 != bVar15) {
    uVar13 = 0x20 - uVar8;
  }
  uVar5 = uVar5 << (uVar8 & 0xff);
  if (bVar14 || (int)uVar6 < 0 != bVar15) {
    uVar5 = uVar5 | uVar11 >> (uVar13 & 0xff);
    uVar11 = uVar11 << (uVar8 & 0xff);
  }
LAB_00013c78:
  if ((int)uVar4 <= (int)uVar9) {
    return CONCAT44(uVar5 + (uVar9 - uVar4) * 0x100000 | param_2,uVar11);
  }
  uVar8 = ~(uVar9 - uVar4);
  if ((int)uVar8 < 0x1f) {
    if (uVar8 - 0x13 != 0 && -0xd < (int)(uVar8 - 0x1f)) {
      uVar8 = 0xc - (uVar8 - 0x13);
      return CONCAT44(uVar12,uVar11 >> (0x20 - uVar8 & 0xff) | uVar5 << (uVar8 & 0xff)) &
             0x80000000ffffffff;
    }
    uVar8 = uVar8 + 1;
    return CONCAT44(param_2 | uVar5 >> (uVar8 & 0xff),
                    uVar11 >> (uVar8 & 0xff) | uVar5 << (0x20 - uVar8 & 0xff));
  }
  return CONCAT44(uVar12,uVar5 >> (uVar8 - 0x1f & 0xff)) & 0x80000000ffffffff;
}



/* ============================================= */
/* Function: __aeabi_dadd */
/* Address: 0x00013ae0 */
/* ============================================= */

ulonglong __aeabi_dadd(uint param_1,uint param_2,uint param_3,uint param_4)

{
  int iVar1;
  byte bVar2;
  byte bVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  uint uVar7;
  int iVar8;
  uint uVar9;
  uint uVar10;
  uint uVar11;
  uint uVar12;
  uint uVar13;
  bool bVar14;
  bool bVar15;
  
  uVar9 = param_2 << 1;
  uVar5 = param_4 << 1;
  iVar8 = (int)uVar9 >> 0x15;
  iVar1 = (int)uVar5 >> 0x15;
  if ((((uVar9 == uVar5 && param_1 == param_3 || uVar9 == 0 && param_1 == 0) ||
       uVar5 == 0 && param_3 == 0) || iVar8 == -1) || iVar1 == -1) {
    if (iVar8 == -1 || iVar1 == -1) {
      uVar9 = param_3;
      uVar5 = param_4;
      if (iVar8 == -1) {
        uVar9 = param_1;
        uVar5 = param_2;
      }
      if (iVar8 != -1 || iVar1 != -1) {
        param_3 = uVar9;
        param_4 = uVar5;
      }
      bVar14 = (uVar5 & 0xfffff) == 0;
      bVar15 = uVar9 == 0 && bVar14;
      if (uVar9 == 0 && bVar14) {
        bVar15 = param_3 == 0 && (param_4 & 0xfffff) == 0;
      }
      if (!bVar15 || uVar5 != param_4) {
        uVar5 = uVar5 | 0x80000;
      }
      return CONCAT44(uVar5,uVar9);
    }
    if (uVar9 != uVar5 || param_1 != param_3) {
      if (uVar9 == 0 && param_1 == 0) {
        param_1 = param_3;
        param_2 = param_4;
      }
      return CONCAT44(param_2,param_1);
    }
    if (param_2 != param_4) {
      return 0;
    }
    if (uVar9 >> 0x15 == 0) {
      bVar2 = (byte)(param_1 >> 0x1f);
      uVar9 = param_2 * 2 + (uint)bVar2;
      if (CARRY4(param_2,param_2) || CARRY4(param_2 * 2,(uint)bVar2)) {
        uVar9 = uVar9 | 0x80000000;
      }
      return CONCAT44(uVar9,param_1 << 1);
    }
    if (uVar9 < 0xffc00000) {
      return CONCAT44(param_2 + 0x100000,param_1);
    }
    param_2 = param_2 & 0x80000000;
LAB_00013d50:
    return (ulonglong)(param_2 | 0x7ff00000) << 0x20;
  }
  uVar9 = uVar9 >> 0x15;
  uVar5 = uVar5 >> 0x15;
  uVar11 = uVar5 - uVar9;
  bVar14 = uVar11 != 0;
  if (uVar5 < uVar9) {
    uVar11 = -uVar11;
  }
  uVar10 = param_1;
  uVar7 = param_2;
  if (bVar14 && uVar9 <= uVar5) {
    uVar9 = uVar9 + uVar11;
    uVar10 = param_3;
    uVar7 = param_4;
    param_3 = param_1;
    param_4 = param_2;
  }
  if (0x36 < uVar11) {
    return CONCAT44(uVar7,uVar10);
  }
  uVar5 = uVar7 & 0xfffff | 0x100000;
  if ((uVar7 & 0x80000000) != 0) {
    bVar14 = uVar10 != 0;
    uVar10 = -uVar10;
    uVar5 = -(uVar5 + bVar14);
  }
  uVar7 = param_4 & 0xfffff | 0x100000;
  if ((param_4 & 0x80000000) != 0) {
    bVar14 = param_3 != 0;
    param_3 = -param_3;
    uVar7 = -(uVar7 + bVar14);
  }
  if (uVar9 == uVar11) {
    uVar7 = uVar7 ^ 0x100000;
    if (uVar9 == 0) {
      uVar5 = uVar5 ^ 0x100000;
      uVar9 = 1;
    }
    else {
      uVar11 = uVar11 - 1;
    }
  }
  uVar13 = -uVar11 + 0x20;
  if ((int)uVar11 < 0x21) {
    uVar12 = param_3 << (uVar13 & 0xff);
    param_3 = param_3 >> (uVar11 & 0xff);
    uVar4 = uVar10 + param_3;
    uVar6 = uVar7 << (uVar13 & 0xff);
    uVar13 = uVar4 + uVar6;
    uVar5 = uVar5 + CARRY4(uVar10,param_3) + ((int)uVar7 >> (uVar11 & 0xff)) +
            (uint)CARRY4(uVar4,uVar6);
  }
  else {
    uVar12 = uVar7 << (-uVar11 + 0x40 & 0xff);
    if (param_3 != 0) {
      uVar12 = uVar12 | 2;
    }
    uVar11 = (int)uVar7 >> (uVar11 - 0x20 & 0xff);
    uVar13 = uVar10 + uVar11;
    uVar5 = uVar5 + ((int)uVar7 >> 0x1f) + (uint)CARRY4(uVar10,uVar11);
  }
  param_2 = uVar5 & 0x80000000;
  uVar11 = uVar5;
  if ((int)uVar5 < 0) {
    bVar14 = uVar12 == 0;
    uVar12 = -uVar12;
    uVar11 = -uVar13;
    uVar13 = -(uVar13 + !bVar14);
    uVar11 = -(uVar5 + (bVar14 <= uVar11));
  }
  if (0xfffff < uVar11) {
    uVar10 = uVar9 - 1;
    if (0x1fffff < uVar11) {
      bVar2 = (byte)uVar11;
      uVar11 = uVar11 >> 1;
      bVar3 = (byte)uVar13;
      uVar13 = (uint)(bVar2 & 1) << 0x1f | uVar13 >> 1;
      uVar12 = (uint)(bVar3 & 1) << 0x1f | uVar12 >> 1;
      uVar10 = uVar9;
      if (0xffbfffff < uVar9 * 0x200000) goto LAB_00013d50;
    }
LAB_00013bf8:
    bVar14 = 0x7fffffff < uVar12;
    if (uVar12 == 0x80000000) {
      bVar14 = (bool)((byte)uVar13 & 1);
    }
    return CONCAT44(uVar11 + uVar10 * 0x100000 + (uint)CARRY4(uVar13,(uint)bVar14) | param_2,
                    uVar13 + bVar14);
  }
  bVar2 = (byte)(uVar12 >> 0x1f);
  uVar12 = uVar12 << 1;
  uVar10 = uVar13 * 2;
  bVar14 = CARRY4(uVar13,uVar13);
  uVar13 = uVar13 * 2 + (uint)bVar2;
  uVar11 = uVar11 * 2 + (uint)(bVar14 || CARRY4(uVar10,(uint)bVar2));
  uVar10 = uVar9 - 2;
  if ((uVar11 & 0x100000) != 0) goto LAB_00013bf8;
  uVar9 = uVar13;
  uVar7 = uVar11;
  if (uVar11 == 0) {
    uVar9 = 0;
    uVar7 = uVar13;
  }
  iVar8 = LZCOUNT(uVar7);
  if (uVar11 == 0) {
    iVar8 = iVar8 + 0x20;
  }
  uVar4 = iVar8 - 0xb;
  bVar15 = SBORROW4(uVar4,0x20);
  uVar6 = iVar8 - 0x2b;
  bVar14 = uVar6 == 0;
  uVar11 = uVar6;
  uVar13 = uVar6;
  if ((int)uVar4 < 0x20) {
    bVar15 = SCARRY4(uVar6,0xc);
    uVar13 = iVar8 - 0x1f;
    bVar14 = uVar13 == 0;
    uVar11 = uVar4;
    if (!bVar14 && -0xd < (int)uVar6) {
      uVar9 = uVar7 << (uVar4 & 0xff);
      uVar7 = uVar7 >> (0xc - uVar13 & 0xff);
      goto LAB_00013c78;
    }
  }
  if (bVar14 || (int)uVar13 < 0 != bVar15) {
    uVar12 = 0x20 - uVar11;
  }
  uVar7 = uVar7 << (uVar11 & 0xff);
  if (bVar14 || (int)uVar13 < 0 != bVar15) {
    uVar7 = uVar7 | uVar9 >> (uVar12 & 0xff);
    uVar9 = uVar9 << (uVar11 & 0xff);
  }
LAB_00013c78:
  if ((int)uVar4 <= (int)uVar10) {
    return CONCAT44(uVar7 + (uVar10 - uVar4) * 0x100000 | param_2,uVar9);
  }
  uVar11 = ~(uVar10 - uVar4);
  if ((int)uVar11 < 0x1f) {
    if (uVar11 - 0x13 != 0 && -0xd < (int)(uVar11 - 0x1f)) {
      uVar11 = 0xc - (uVar11 - 0x13);
      return CONCAT44(uVar5,uVar9 >> (0x20 - uVar11 & 0xff) | uVar7 << (uVar11 & 0xff)) &
             0x80000000ffffffff;
    }
    uVar11 = uVar11 + 1;
    return CONCAT44(param_2 | uVar7 >> (uVar11 & 0xff),
                    uVar9 >> (uVar11 & 0xff) | uVar7 << (0x20 - uVar11 & 0xff));
  }
  return CONCAT44(uVar5,uVar7 >> (uVar11 - 0x1f & 0xff)) & 0x80000000ffffffff;
}



/* ============================================= */
/* Function: __floatunsidf */
/* Address: 0x00013d8c */
/* ============================================= */

ulonglong __floatunsidf(uint param_1)

{
  uint uVar1;
  uint uVar2;
  uint uVar3;
  int iVar4;
  uint uVar5;
  uint uVar6;
  uint in_r12;
  bool bVar7;
  bool bVar8;
  
  if (param_1 == 0) {
    return 0;
  }
  uVar1 = 0;
  iVar4 = LZCOUNT(param_1);
  uVar5 = iVar4 + 0x15;
  bVar8 = SBORROW4(uVar5,0x20);
  uVar2 = iVar4 - 0xb;
  bVar7 = uVar2 == 0;
  uVar6 = uVar2;
  uVar3 = uVar2;
  if (uVar5 < 0x20) {
    bVar8 = SCARRY4(uVar2,0xc);
    uVar3 = iVar4 + 1;
    bVar7 = uVar3 == 0;
    uVar6 = uVar5;
    if (!bVar7 && -0xd < (int)uVar2) {
      uVar1 = param_1 << uVar5;
      param_1 = param_1 >> (0xc - uVar3 & 0xff);
      goto LAB_00013c78;
    }
  }
  if (bVar7 || (int)uVar3 < 0 != bVar8) {
    in_r12 = 0x20 - uVar6;
  }
  param_1 = param_1 << (uVar6 & 0xff);
  if (bVar7 || (int)uVar3 < 0 != bVar8) {
    param_1 = param_1 | 0U >> (in_r12 & 0xff);
    uVar1 = 0 << (uVar6 & 0xff);
  }
LAB_00013c78:
  if (uVar5 < 0x433) {
    return CONCAT44(param_1 + (0x432 - uVar5) * 0x100000,uVar1);
  }
  uVar6 = ~(0x432 - uVar5);
  if (0x1e < (int)uVar6) {
    return (ulonglong)(param_1 >> (uVar6 - 0x1f & 0xff));
  }
  if (uVar6 - 0x13 == 0 || (int)(uVar6 - 0x1f) < -0xc) {
    uVar6 = uVar6 + 1;
    return CONCAT44(param_1 >> (uVar6 & 0xff),
                    uVar1 >> (uVar6 & 0xff) | param_1 << (0x20 - uVar6 & 0xff));
  }
  uVar6 = 0xc - (uVar6 - 0x13);
  return (ulonglong)(uVar1 >> (0x20 - uVar6 & 0xff) | param_1 << (uVar6 & 0xff));
}



/* ============================================= */
/* Function: __aeabi_i2d */
/* Address: 0x00013db0 */
/* ============================================= */

ulonglong __aeabi_i2d(uint param_1)

{
  uint uVar1;
  uint uVar2;
  uint uVar3;
  uint uVar4;
  int iVar5;
  uint uVar6;
  uint uVar7;
  uint uVar8;
  uint in_r12;
  bool bVar9;
  bool bVar10;
  
  if (param_1 == 0) {
    return 0;
  }
  uVar8 = param_1 & 0x80000000;
  uVar2 = param_1;
  if ((int)uVar8 < 0) {
    uVar2 = -param_1;
  }
  uVar1 = 0;
  iVar5 = LZCOUNT(uVar2);
  uVar6 = iVar5 + 0x15;
  bVar10 = SBORROW4(uVar6,0x20);
  uVar3 = iVar5 - 0xb;
  bVar9 = uVar3 == 0;
  uVar7 = uVar3;
  uVar4 = uVar3;
  if (uVar6 < 0x20) {
    bVar10 = SCARRY4(uVar3,0xc);
    uVar4 = iVar5 + 1;
    bVar9 = uVar4 == 0;
    uVar7 = uVar6;
    if (!bVar9 && -0xd < (int)uVar3) {
      uVar1 = uVar2 << uVar6;
      uVar2 = uVar2 >> (0xc - uVar4 & 0xff);
      goto LAB_00013c78;
    }
  }
  if (bVar9 || (int)uVar4 < 0 != bVar10) {
    in_r12 = 0x20 - uVar7;
  }
  uVar2 = uVar2 << (uVar7 & 0xff);
  if (bVar9 || (int)uVar4 < 0 != bVar10) {
    uVar2 = uVar2 | 0U >> (in_r12 & 0xff);
    uVar1 = 0 << (uVar7 & 0xff);
  }
LAB_00013c78:
  if (uVar6 < 0x433) {
    return CONCAT44(uVar2 + (0x432 - uVar6) * 0x100000 | uVar8,uVar1);
  }
  uVar7 = ~(0x432 - uVar6);
  if (0x1e < (int)uVar7) {
    return CONCAT44(param_1,uVar2 >> (uVar7 - 0x1f & 0xff)) & 0x80000000ffffffff;
  }
  if (uVar7 - 0x13 == 0 || (int)(uVar7 - 0x1f) < -0xc) {
    uVar7 = uVar7 + 1;
    return CONCAT44(uVar8 | uVar2 >> (uVar7 & 0xff),
                    uVar1 >> (uVar7 & 0xff) | uVar2 << (0x20 - uVar7 & 0xff));
  }
  uVar8 = 0xc - (uVar7 - 0x13);
  return CONCAT44(param_1,uVar1 >> (0x20 - uVar8 & 0xff) | uVar2 << (uVar8 & 0xff)) &
         0x80000000ffffffff;
}



/* ============================================= */
/* Function: __aeabi_f2d */
/* Address: 0x00013dd8 */
/* ============================================= */

ulonglong __aeabi_f2d(uint param_1,undefined4 param_2,undefined4 param_3,uint param_4)

{
  uint uVar1;
  uint uVar2;
  uint uVar3;
  uint uVar4;
  uint uVar5;
  int iVar6;
  uint uVar7;
  uint uVar8;
  uint uVar9;
  uint in_r12;
  bool bVar10;
  bool bVar11;
  
  uVar2 = param_1 << 1;
  bVar10 = uVar2 == 0;
  uVar9 = param_1 & 0x80000000;
  uVar8 = (uint)((int)uVar2 >> 3) >> 1;
  uVar1 = uVar9 | uVar8;
  param_1 = param_1 << 0x1d;
  if (!bVar10) {
    param_4 = uVar2 & 0xff000000;
    bVar10 = param_4 == 0;
  }
  if (!bVar10) {
    bVar10 = param_4 == 0xff000000;
  }
  if (!bVar10) {
    return CONCAT44(uVar1,param_1) ^ 0x3800000000000000;
  }
  if (uVar2 == 0 || param_4 == 0xff000000) {
    return CONCAT44(uVar1,param_1);
  }
  uVar2 = param_1;
  uVar3 = uVar8;
  if (uVar8 == 0) {
    uVar2 = 0;
    uVar3 = param_1;
  }
  iVar6 = LZCOUNT(uVar3);
  if (uVar8 == 0) {
    iVar6 = iVar6 + 0x20;
  }
  uVar7 = iVar6 - 0xb;
  bVar11 = SBORROW4(uVar7,0x20);
  uVar4 = iVar6 - 0x2b;
  bVar10 = uVar4 == 0;
  uVar8 = uVar4;
  uVar5 = uVar4;
  if ((int)uVar7 < 0x20) {
    bVar11 = SCARRY4(uVar4,0xc);
    uVar5 = iVar6 - 0x1f;
    bVar10 = uVar5 == 0;
    uVar8 = uVar7;
    if (!bVar10 && -0xd < (int)uVar4) {
      uVar2 = uVar3 << (uVar7 & 0xff);
      uVar3 = uVar3 >> (0xc - uVar5 & 0xff);
      goto LAB_00013c78;
    }
  }
  if (bVar10 || (int)uVar5 < 0 != bVar11) {
    in_r12 = 0x20 - uVar8;
  }
  uVar3 = uVar3 << (uVar8 & 0xff);
  if (bVar10 || (int)uVar5 < 0 != bVar11) {
    uVar3 = uVar3 | uVar2 >> (in_r12 & 0xff);
    uVar2 = uVar2 << (uVar8 & 0xff);
  }
LAB_00013c78:
  if ((int)uVar7 < 0x381) {
    return CONCAT44(uVar3 + (0x380 - uVar7) * 0x100000 | uVar9,uVar2);
  }
  uVar8 = ~(0x380 - uVar7);
  if ((int)uVar8 < 0x1f) {
    if (uVar8 - 0x13 != 0 && -0xd < (int)(uVar8 - 0x1f)) {
      uVar9 = 0xc - (uVar8 - 0x13);
      return CONCAT44(uVar1,uVar2 >> (0x20 - uVar9 & 0xff) | uVar3 << (uVar9 & 0xff)) &
             0x80000000ffffffff;
    }
    uVar8 = uVar8 + 1;
    return CONCAT44(uVar9 | uVar3 >> (uVar8 & 0xff),
                    uVar2 >> (uVar8 & 0xff) | uVar3 << (0x20 - uVar8 & 0xff));
  }
  return CONCAT44(uVar1,uVar3 >> (uVar8 - 0x1f & 0xff)) & 0x80000000ffffffff;
}



/* ============================================= */
/* Function: __floatundidf */
/* Address: 0x00013e18 */
/* ============================================= */

ulonglong __floatundidf(uint param_1,uint param_2)

{
  byte bVar1;
  byte bVar2;
  uint uVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  uint uVar7;
  int iVar8;
  uint uVar9;
  uint uVar10;
  int iVar11;
  bool bVar12;
  bool bVar13;
  
  if (param_1 == 0 && param_2 == 0) {
    return CONCAT44(param_2,param_1);
  }
  iVar11 = 0x432;
  uVar10 = 0;
  if (param_2 >> 0x16 != 0) {
    iVar11 = 3;
    if (param_2 >> 0x19 != 0) {
      iVar11 = 6;
    }
    if (param_2 >> 0x1c != 0) {
      iVar11 = iVar11 + 3;
    }
    iVar11 = iVar11 - ((int)param_2 >> 0x1f);
    uVar10 = param_1 << (0x20U - iVar11 & 0xff);
    param_1 = param_1 >> iVar11 | param_2 << (0x20U - iVar11 & 0xff);
    param_2 = param_2 >> iVar11;
    iVar11 = iVar11 + 0x432;
  }
  if (0xfffff < param_2) {
    if (0x1fffff < param_2) {
      bVar1 = (byte)param_2;
      param_2 = param_2 >> 1;
      bVar2 = (byte)param_1;
      param_1 = (uint)(bVar1 & 1) << 0x1f | param_1 >> 1;
      uVar10 = (uint)(bVar2 & 1) << 0x1f | uVar10 >> 1;
      iVar11 = iVar11 + 1;
      if (0xffbfffff < (uint)(iVar11 * 0x200000)) {
        return 0x7ff0000000000000;
      }
    }
LAB_00013bf8:
    bVar12 = 0x7fffffff < uVar10;
    if (uVar10 == 0x80000000) {
      bVar12 = (bool)((byte)param_1 & 1);
    }
    return CONCAT44(param_2 + iVar11 * 0x100000 + (uint)CARRY4(param_1,(uint)bVar12),
                    param_1 + bVar12);
  }
  bVar1 = (byte)(uVar10 >> 0x1f);
  uVar10 = uVar10 << 1;
  uVar3 = param_1 * 2;
  bVar12 = CARRY4(param_1,param_1);
  param_1 = param_1 * 2 + (uint)bVar1;
  param_2 = param_2 * 2 + (uint)(bVar12 || CARRY4(uVar3,(uint)bVar1));
  iVar11 = iVar11 + -1;
  if ((param_2 & 0x100000) != 0) goto LAB_00013bf8;
  uVar3 = param_1;
  uVar4 = param_2;
  if (param_2 == 0) {
    uVar3 = 0;
    uVar4 = param_1;
  }
  iVar8 = LZCOUNT(uVar4);
  if (param_2 == 0) {
    iVar8 = iVar8 + 0x20;
  }
  uVar9 = iVar8 - 0xb;
  bVar13 = SBORROW4(uVar9,0x20);
  uVar5 = iVar8 - 0x2b;
  bVar12 = uVar5 == 0;
  uVar7 = uVar5;
  uVar6 = uVar5;
  if ((int)uVar9 < 0x20) {
    bVar13 = SCARRY4(uVar5,0xc);
    uVar6 = iVar8 - 0x1f;
    bVar12 = uVar6 == 0;
    uVar7 = uVar9;
    if (!bVar12 && -0xd < (int)uVar5) {
      uVar3 = uVar4 << (uVar9 & 0xff);
      uVar4 = uVar4 >> (0xc - uVar6 & 0xff);
      goto LAB_00013c78;
    }
  }
  if (bVar12 || (int)uVar6 < 0 != bVar13) {
    uVar10 = 0x20 - uVar7;
  }
  uVar4 = uVar4 << (uVar7 & 0xff);
  if (bVar12 || (int)uVar6 < 0 != bVar13) {
    uVar4 = uVar4 | uVar3 >> (uVar10 & 0xff);
    uVar3 = uVar3 << (uVar7 & 0xff);
  }
LAB_00013c78:
  if ((int)uVar9 <= iVar11) {
    return CONCAT44(uVar4 + (iVar11 - uVar9) * 0x100000,uVar3);
  }
  uVar10 = ~(iVar11 - uVar9);
  if (0x1e < (int)uVar10) {
    return (ulonglong)(uVar4 >> (uVar10 - 0x1f & 0xff));
  }
  if (uVar10 - 0x13 == 0 || (int)(uVar10 - 0x1f) < -0xc) {
    uVar10 = uVar10 + 1;
    return CONCAT44(uVar4 >> (uVar10 & 0xff),
                    uVar3 >> (uVar10 & 0xff) | uVar4 << (0x20 - uVar10 & 0xff));
  }
  uVar10 = 0xc - (uVar10 - 0x13);
  return (ulonglong)(uVar3 >> (0x20 - uVar10 & 0xff) | uVar4 << (uVar10 & 0xff));
}



/* ============================================= */
/* Function: __aeabi_l2d */
/* Address: 0x00013e2c */
/* ============================================= */

ulonglong __aeabi_l2d(uint param_1,uint param_2)

{
  byte bVar1;
  byte bVar2;
  uint uVar3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  uint uVar7;
  int iVar8;
  uint uVar9;
  int iVar10;
  uint uVar11;
  uint uVar12;
  bool bVar13;
  bool bVar14;
  
  if (param_1 == 0 && param_2 == 0) {
    return CONCAT44(param_2,param_1);
  }
  uVar11 = param_2 & 0x80000000;
  uVar4 = param_2;
  if ((int)uVar11 < 0) {
    bVar13 = param_1 != 0;
    param_1 = -param_1;
    uVar4 = -(param_2 + bVar13);
  }
  iVar10 = 0x432;
  uVar12 = 0;
  if (uVar4 >> 0x16 != 0) {
    iVar10 = 3;
    if (uVar4 >> 0x19 != 0) {
      iVar10 = 6;
    }
    if (uVar4 >> 0x1c != 0) {
      iVar10 = iVar10 + 3;
    }
    iVar10 = iVar10 - ((int)uVar4 >> 0x1f);
    uVar12 = param_1 << (0x20U - iVar10 & 0xff);
    param_1 = param_1 >> iVar10 | uVar4 << (0x20U - iVar10 & 0xff);
    uVar4 = uVar4 >> iVar10;
    iVar10 = iVar10 + 0x432;
  }
  if (0xfffff < uVar4) {
    if (0x1fffff < uVar4) {
      bVar1 = (byte)uVar4;
      uVar4 = uVar4 >> 1;
      bVar2 = (byte)param_1;
      param_1 = (uint)(bVar1 & 1) << 0x1f | param_1 >> 1;
      uVar12 = (uint)(bVar2 & 1) << 0x1f | uVar12 >> 1;
      iVar10 = iVar10 + 1;
      if (0xffbfffff < (uint)(iVar10 * 0x200000)) {
        return (ulonglong)(uVar11 | 0x7ff00000) << 0x20;
      }
    }
LAB_00013bf8:
    bVar13 = 0x7fffffff < uVar12;
    if (uVar12 == 0x80000000) {
      bVar13 = (bool)((byte)param_1 & 1);
    }
    return CONCAT44(uVar4 + iVar10 * 0x100000 + (uint)CARRY4(param_1,(uint)bVar13) | uVar11,
                    param_1 + bVar13);
  }
  bVar1 = (byte)(uVar12 >> 0x1f);
  uVar12 = uVar12 << 1;
  uVar3 = param_1 * 2;
  bVar13 = CARRY4(param_1,param_1);
  param_1 = param_1 * 2 + (uint)bVar1;
  uVar4 = uVar4 * 2 + (uint)(bVar13 || CARRY4(uVar3,(uint)bVar1));
  iVar10 = iVar10 + -1;
  if ((uVar4 & 0x100000) != 0) goto LAB_00013bf8;
  uVar3 = param_1;
  uVar5 = uVar4;
  if (uVar4 == 0) {
    uVar3 = 0;
    uVar5 = param_1;
  }
  iVar8 = LZCOUNT(uVar5);
  if (uVar4 == 0) {
    iVar8 = iVar8 + 0x20;
  }
  uVar9 = iVar8 - 0xb;
  bVar14 = SBORROW4(uVar9,0x20);
  uVar6 = iVar8 - 0x2b;
  bVar13 = uVar6 == 0;
  uVar4 = uVar6;
  uVar7 = uVar6;
  if ((int)uVar9 < 0x20) {
    bVar14 = SCARRY4(uVar6,0xc);
    uVar7 = iVar8 - 0x1f;
    bVar13 = uVar7 == 0;
    uVar4 = uVar9;
    if (!bVar13 && -0xd < (int)uVar6) {
      uVar3 = uVar5 << (uVar9 & 0xff);
      uVar5 = uVar5 >> (0xc - uVar7 & 0xff);
      goto LAB_00013c78;
    }
  }
  if (bVar13 || (int)uVar7 < 0 != bVar14) {
    uVar12 = 0x20 - uVar4;
  }
  uVar5 = uVar5 << (uVar4 & 0xff);
  if (bVar13 || (int)uVar7 < 0 != bVar14) {
    uVar5 = uVar5 | uVar3 >> (uVar12 & 0xff);
    uVar3 = uVar3 << (uVar4 & 0xff);
  }
LAB_00013c78:
  if ((int)uVar9 <= iVar10) {
    return CONCAT44(uVar5 + (iVar10 - uVar9) * 0x100000 | uVar11,uVar3);
  }
  uVar4 = ~(iVar10 - uVar9);
  if (0x1e < (int)uVar4) {
    return CONCAT44(param_2,uVar5 >> (uVar4 - 0x1f & 0xff)) & 0x80000000ffffffff;
  }
  if (uVar4 - 0x13 == 0 || (int)(uVar4 - 0x1f) < -0xc) {
    uVar4 = uVar4 + 1;
    return CONCAT44(uVar11 | uVar5 >> (uVar4 & 0xff),
                    uVar3 >> (uVar4 & 0xff) | uVar5 << (0x20 - uVar4 & 0xff));
  }
  uVar4 = 0xc - (uVar4 - 0x13);
  return CONCAT44(param_2,uVar3 >> (0x20 - uVar4 & 0xff) | uVar5 << (uVar4 & 0xff)) &
         0x80000000ffffffff;
}



/* ============================================= */
/* Function: __muldf3 */
/* Address: 0x00013e8c */
/* ============================================= */

ulonglong __muldf3(undefined4 param_1,uint param_2,uint param_3,uint param_4)

{
  longlong lVar1;
  ulonglong uVar2;
  byte bVar3;
  uint uVar4;
  uint extraout_r2;
  uint extraout_r3;
  uint uVar5;
  uint uVar6;
  int iVar7;
  uint uVar8;
  uint unaff_r5;
  uint uVar9;
  uint uVar10;
  uint extraout_r12;
  bool bVar11;
  bool bVar12;
  bool bVar13;
  ulonglong uVar14;
  
  uVar14 = CONCAT44(param_2,param_1);
  uVar10 = 0x7ff;
  uVar6 = param_2 >> 0x14 & 0x7ff;
  bVar11 = uVar6 == 0;
  if (!bVar11) {
    unaff_r5 = param_4 >> 0x14 & 0x7ff;
    bVar11 = unaff_r5 == 0;
  }
  if (!bVar11) {
    bVar11 = uVar6 == 0x7ff;
  }
  if (!bVar11) {
    bVar11 = unaff_r5 == 0x7ff;
  }
  if (bVar11) {
    uVar14 = FUN_0001406c();
    param_3 = extraout_r2;
    param_4 = extraout_r3;
    uVar10 = extraout_r12;
  }
  uVar4 = (uint)(uVar14 >> 0x20);
  iVar7 = uVar6 + unaff_r5;
  uVar9 = uVar4 ^ param_4;
  uVar4 = uVar4 & ~(uVar10 << 0x15);
  param_4 = param_4 & ~(uVar10 << 0x15);
  uVar6 = uVar4 | 0x100000;
  uVar5 = param_4 | 0x100000;
  if ((uint)uVar14 == 0 && (uVar4 & 0xfffff) == 0 || param_3 == 0 && (param_4 & 0xfffff) == 0) {
    param_3 = (uint)uVar14 | param_3;
    uVar5 = (uVar9 & 0x80000000 | uVar6) ^ uVar5;
    uVar6 = uVar10 >> 1;
    bVar13 = SBORROW4(iVar7,uVar6);
    uVar8 = iVar7 - uVar6;
    bVar11 = uVar8 == 0;
    uVar4 = uVar8;
    if (!bVar11 && (int)uVar6 <= iVar7) {
      bVar13 = SBORROW4(uVar10,uVar8);
      uVar4 = uVar10 - uVar8;
      bVar11 = uVar10 == uVar8;
    }
    if (!bVar11 && (int)uVar4 < 0 == bVar13) {
      return CONCAT44(uVar5 | uVar8 * 0x100000,param_3);
    }
    uVar5 = uVar5 | 0x100000;
    uVar10 = 0;
    bVar13 = SBORROW4(uVar8,1);
    uVar8 = uVar8 - 1;
    bVar11 = uVar8 == 0;
    uVar6 = uVar8;
  }
  else {
    uVar2 = (ulonglong)param_3 * (uVar14 & 0xffffffff);
    uVar14 = (ulonglong)uVar5 * (uVar14 & 0xffffffff) +
             (ulonglong)param_3 * (ulonglong)uVar6 + (uVar2 >> 0x20);
    uVar4 = (uint)uVar14;
    lVar1 = (ulonglong)uVar5 * (ulonglong)uVar6 + (uVar14 >> 0x20);
    uVar10 = (uint)lVar1;
    uVar6 = (uint)((ulonglong)lVar1 >> 0x20);
    if ((int)uVar2 != 0) {
      uVar4 = uVar4 | 1;
    }
    uVar8 = (iVar7 + -0xff) - ((uVar6 < 0x200) + 0x300);
    if (uVar6 < 0x200) {
      bVar3 = (byte)(uVar4 >> 0x1f);
      uVar4 = uVar4 << 1;
      lVar1 = CONCAT44(uVar6 * 2 + (uint)(CARRY4(uVar10,uVar10) || CARRY4(uVar10 * 2,(uint)bVar3)),
                       uVar10 * 2 + (uint)bVar3);
    }
    uVar5 = uVar9 & 0x80000000 | (int)((ulonglong)lVar1 >> 0x20) << 0xb | (uint)lVar1 >> 0x15;
    param_3 = (uint)lVar1 << 0xb | uVar4 >> 0x15;
    uVar10 = uVar4 * 0x800;
    bVar12 = 0xfc < uVar8;
    bVar13 = SBORROW4(uVar8,0xfd);
    uVar9 = uVar8 - 0xfd;
    bVar11 = uVar9 == 0;
    uVar6 = uVar9;
    if (bVar12 && !bVar11) {
      bVar12 = 0x6ff < uVar9;
      bVar13 = SBORROW4(uVar9,0x700);
      uVar6 = uVar8 - 0x7fd;
      bVar11 = uVar9 == 0x700;
    }
    if (!bVar12 || bVar11) {
      bVar11 = 0x7fffffff < uVar10;
      if (uVar10 == 0x80000000) {
        bVar11 = (bool)((byte)(uVar4 >> 0x15) & 1);
      }
      return CONCAT44(uVar5 + uVar8 * 0x100000 + (uint)CARRY4(param_3,(uint)bVar11),param_3 + bVar11
                     );
    }
  }
  if (!bVar11 && (int)uVar6 < 0 == bVar13) {
    return (ulonglong)(uVar5 & 0x80000000 | 0x7ff00000) << 0x20;
  }
  if (-0x36 < (int)uVar8) {
    uVar6 = -uVar8;
    uVar4 = uVar6 - 0x20;
    if (0x1f < (int)uVar6) {
      uVar9 = param_3 >> (uVar4 & 0xff) | uVar5 << (0x20 - uVar4 & 0xff);
      uVar6 = (uVar5 >> (uVar4 & 0xff) & ~((uVar5 & 0x80000000) >> (uVar4 & 0xff))) -
              ((int)uVar9 >> 0x1f);
      if ((uVar10 == 0 && param_3 << (0x20 - uVar4 & 0xff) == 0) && (uVar9 & 0x7fffffff) == 0) {
        uVar6 = uVar6 & ~(uVar9 >> 0x1f);
      }
      return CONCAT44(uVar5,uVar6) & 0x80000000ffffffff;
    }
    if (uVar6 - 0x14 != 0 && -0xd < (int)uVar4) {
      uVar4 = 0xc - (uVar6 - 0x14);
      uVar6 = param_3 << (uVar4 & 0xff);
      uVar4 = param_3 >> (0x20 - uVar4 & 0xff) | uVar5 << (uVar4 & 0xff);
      uVar9 = uVar4 + -((int)uVar6 >> 0x1f);
      if (uVar10 == 0 && (uVar6 & 0x7fffffff) == 0) {
        uVar9 = uVar9 & ~(uVar6 >> 0x1f);
      }
      return CONCAT44((uVar5 & 0x80000000) + (uint)CARRY4(uVar4,-((int)uVar6 >> 0x1f)),uVar9);
    }
    uVar4 = param_3 << (uVar8 + 0x20 & 0xff);
    uVar9 = param_3 >> (uVar6 & 0xff) | uVar5 << (uVar8 + 0x20 & 0xff);
    uVar8 = uVar9 + -((int)uVar4 >> 0x1f);
    if (uVar10 == 0 && (uVar4 & 0x7fffffff) == 0) {
      uVar8 = uVar8 & ~(uVar4 >> 0x1f);
    }
    return CONCAT44((uVar5 & 0x80000000) + ((uVar5 & 0x7fffffff) >> (uVar6 & 0xff)) +
                    (uint)CARRY4(uVar9,-((int)uVar4 >> 0x1f)),uVar8);
  }
  return (ulonglong)(uVar5 & 0x80000000) << 0x20;
}



/* ============================================= */
/* Function: FUN_0001406c */
/* Address: 0x0001406c */
/* ============================================= */

ulonglong FUN_0001406c(int param_1,uint param_2,int param_3,uint param_4)

{
  bool bVar1;
  int iVar2;
  uint uVar3;
  uint unaff_r4;
  uint uVar4;
  uint uVar5;
  uint in_r12;
  
  uVar4 = in_r12 & param_4 >> 0x14;
  if (unaff_r4 != in_r12 && uVar4 != in_r12) {
    if (param_1 == 0 && (param_2 & 0x7fffffff) == 0 || param_3 == 0 && (param_4 & 0x7fffffff) == 0)
    {
      return (ulonglong)((param_2 ^ param_4) & 0x80000000) << 0x20;
    }
    if (unaff_r4 == 0) {
      uVar5 = param_2 & 0x80000000;
      do {
        iVar2 = param_1 >> 0x1f;
        param_1 = param_1 << 1;
        param_2 = param_2 * 2 - iVar2;
      } while ((param_2 & 0x100000) == 0);
      param_2 = param_2 | uVar5;
      if (uVar4 != 0) {
        return CONCAT44(param_2,param_1);
      }
    }
    do {
      iVar2 = param_3 >> 0x1f;
      param_3 = param_3 << 1;
      param_4 = param_4 * 2 - iVar2;
    } while ((param_4 & 0x100000) == 0);
    return CONCAT44(param_2,param_1);
  }
  bVar1 = (param_2 & 0x7fffffff) != 0;
  uVar5 = param_4;
  iVar2 = param_3;
  if (param_1 != 0 || bVar1) {
    uVar5 = param_2;
    iVar2 = param_1;
  }
  uVar3 = uVar5;
  if ((((param_1 != 0 || bVar1) && (param_3 != 0 || (param_4 & 0x7fffffff) != 0)) &&
      ((unaff_r4 != in_r12 || (iVar2 == 0 && (uVar5 & 0xfffff) == 0)))) &&
     ((uVar4 != in_r12 ||
      (iVar2 = param_3, uVar3 = param_4, param_3 == 0 && (param_4 & 0xfffff) == 0)))) {
    return (ulonglong)((uVar5 ^ param_4) & 0x80000000 | 0x7ff00000) << 0x20;
  }
  return CONCAT44(uVar3,iVar2) | 0x7ff8000000000000;
}



/* ============================================= */
/* Function: __divdf3 */
/* Address: 0x000140f8 */
/* ============================================= */

ulonglong __divdf3(undefined4 param_1,uint param_2,uint param_3,uint param_4)

{
  uint uVar1;
  uint uVar2;
  uint extraout_r2;
  uint uVar3;
  uint extraout_r3;
  uint uVar4;
  uint uVar5;
  uint uVar6;
  int iVar7;
  uint uVar8;
  uint unaff_r5;
  uint uVar9;
  uint uVar10;
  uint uVar11;
  uint extraout_r12;
  uint uVar12;
  bool bVar13;
  bool bVar14;
  bool bVar15;
  undefined8 uVar16;
  
  uVar16 = CONCAT44(param_2,param_1);
  uVar11 = 0x7ff;
  uVar6 = param_2 >> 0x14 & 0x7ff;
  bVar13 = uVar6 == 0;
  if (!bVar13) {
    unaff_r5 = param_4 >> 0x14 & 0x7ff;
    bVar13 = unaff_r5 == 0;
  }
  if (!bVar13) {
    bVar13 = uVar6 == 0x7ff;
  }
  if (!bVar13) {
    bVar13 = unaff_r5 == 0x7ff;
  }
  if (bVar13) {
    uVar16 = FUN_0001428c();
    param_3 = extraout_r2;
    param_4 = extraout_r3;
    uVar11 = extraout_r12;
  }
  uVar8 = (uint)((ulonglong)uVar16 >> 0x20);
  uVar2 = (uint)uVar16;
  iVar7 = uVar6 - unaff_r5;
  if (param_3 == 0 && (param_4 & 0xfffff) == 0) {
    uVar6 = (uVar8 ^ param_4) & 0x80000000 | uVar8 & 0xfffff;
    bVar14 = SCARRY4(iVar7,uVar11 >> 1);
    uVar8 = iVar7 + (uVar11 >> 1);
    bVar13 = (int)uVar8 < 0;
    bVar15 = uVar8 == 0;
    if (!bVar15 && bVar13 == bVar14) {
      bVar14 = SBORROW4(uVar11,uVar8);
      bVar13 = (int)(uVar11 - uVar8) < 0;
      bVar15 = uVar11 == uVar8;
    }
    if (!bVar15 && bVar13 == bVar14) {
      return CONCAT44(uVar6 | uVar8 * 0x100000,uVar2);
    }
    uVar6 = uVar6 | 0x100000;
    uVar4 = 0;
    bVar15 = SBORROW4(uVar8,1);
    uVar8 = uVar8 - 1;
    bVar13 = uVar8 == 0;
    uVar11 = uVar8;
  }
  else {
    uVar4 = (param_4 << 0xc) >> 4 | 0x10000000 | param_3 >> 0x18;
    uVar11 = param_3 << 8;
    uVar9 = (uVar8 << 0xc) >> 4 | 0x10000000 | uVar2 >> 0x18;
    uVar2 = uVar2 * 0x100;
    uVar6 = (uVar8 ^ param_4) & 0x80000000;
    bVar13 = uVar4 <= uVar9;
    if (uVar9 == uVar4) {
      bVar13 = uVar11 <= uVar2;
    }
    iVar7 = iVar7 + (uint)bVar13;
    uVar8 = iVar7 + 0x3fd;
    if (!bVar13) {
      uVar4 = uVar4 >> 1;
      uVar11 = (uint)((byte)(param_3 >> 0x18) & 1) << 0x1f | uVar11 >> 1;
    }
    uVar10 = uVar2 - uVar11;
    uVar9 = uVar9 - (uVar4 + (uVar2 < uVar11));
    uVar5 = uVar4 >> 1;
    uVar3 = (uint)((byte)uVar4 & 1) << 0x1f | uVar11 >> 1;
    uVar2 = 0x100000;
    uVar11 = 0x80000;
    while( true ) {
      bVar13 = uVar3 <= uVar10;
      if (uVar5 < uVar9 || uVar9 - uVar5 < (uint)bVar13) {
        uVar10 = uVar10 - uVar3;
        uVar2 = uVar2 | uVar11;
        uVar9 = uVar9 - (uVar5 + !bVar13);
      }
      uVar4 = uVar5 >> 1;
      uVar3 = (uint)((byte)uVar5 & 1) << 0x1f | uVar3 >> 1;
      bVar13 = uVar3 <= uVar10;
      if (uVar4 < uVar9 || uVar9 - uVar4 < (uint)bVar13) {
        uVar10 = uVar10 - uVar3;
        uVar2 = uVar2 | uVar11 >> 1;
        uVar9 = uVar9 - (uVar4 + !bVar13);
      }
      uVar12 = uVar5 >> 2;
      uVar1 = (uint)((byte)uVar4 & 1) << 0x1f | uVar3 >> 1;
      bVar13 = uVar1 <= uVar10;
      if (uVar12 < uVar9 || uVar9 - uVar12 < (uint)bVar13) {
        uVar10 = uVar10 - uVar1;
        uVar2 = uVar2 | uVar11 >> 2;
        uVar9 = uVar9 - (uVar12 + !bVar13);
      }
      uVar5 = uVar5 >> 3;
      uVar3 = (uint)((byte)uVar12 & 1) << 0x1f | uVar1 >> 1;
      bVar13 = uVar3 <= uVar10;
      if (uVar5 < uVar9 || uVar9 - uVar5 < (uint)bVar13) {
        uVar10 = uVar10 - uVar3;
        uVar2 = uVar2 | uVar11 >> 3;
        uVar9 = uVar9 - (uVar5 + !bVar13);
      }
      uVar4 = uVar9 | uVar10;
      if (uVar4 == 0) break;
      uVar9 = uVar9 << 4 | uVar10 >> 0x1c;
      uVar10 = uVar10 << 4;
      uVar5 = uVar5 << 3 | uVar3 >> 0x1d;
      uVar3 = (uVar1 >> 1) << 3;
      uVar11 = uVar11 >> 4;
      if (uVar11 == 0) {
        if ((uVar6 & 0x100000) != 0) goto LAB_00014238;
        uVar6 = uVar6 | uVar2;
        uVar2 = 0;
        uVar11 = 0x80000000;
      }
    }
    if ((uVar6 & 0x100000) == 0) {
      uVar6 = uVar6 | uVar2;
      uVar2 = 0;
    }
LAB_00014238:
    bVar14 = 0xfc < uVar8;
    bVar15 = SBORROW4(uVar8,0xfd);
    uVar12 = iVar7 + 0x300;
    bVar13 = uVar12 == 0;
    uVar11 = uVar12;
    if (bVar14 && !bVar13) {
      bVar14 = 0x6ff < uVar12;
      bVar15 = SBORROW4(uVar12,0x700);
      uVar11 = iVar7 - 0x400;
      bVar13 = uVar12 == 0x700;
    }
    if (!bVar14 || bVar13) {
      bVar13 = uVar5 <= uVar9;
      if (uVar9 == uVar5) {
        bVar13 = uVar3 <= uVar10;
      }
      if (uVar9 == uVar5 && uVar10 == uVar3) {
        bVar13 = (bool)((byte)uVar2 & 1);
      }
      return CONCAT44(uVar6 + uVar8 * 0x100000 + (uint)CARRY4(uVar2,(uint)bVar13),uVar2 + bVar13);
    }
  }
  if (!bVar13 && (int)uVar11 < 0 == bVar15) {
    return (ulonglong)(uVar6 & 0x80000000 | 0x7ff00000) << 0x20;
  }
  if ((int)uVar8 < -0x35) {
    return (ulonglong)(uVar6 & 0x80000000) << 0x20;
  }
  uVar11 = -uVar8;
  uVar9 = uVar11 - 0x20;
  if (0x1f < (int)uVar11) {
    uVar8 = uVar2 >> (uVar9 & 0xff) | uVar6 << (0x20 - uVar9 & 0xff);
    uVar11 = (uVar6 >> (uVar9 & 0xff) & ~((uVar6 & 0x80000000) >> (uVar9 & 0xff))) -
             ((int)uVar8 >> 0x1f);
    if ((uVar4 == 0 && uVar2 << (0x20 - uVar9 & 0xff) == 0) && (uVar8 & 0x7fffffff) == 0) {
      uVar11 = uVar11 & ~(uVar8 >> 0x1f);
    }
    return CONCAT44(uVar6,uVar11) & 0x80000000ffffffff;
  }
  if (uVar11 - 0x14 != 0 && -0xd < (int)uVar9) {
    uVar8 = 0xc - (uVar11 - 0x14);
    uVar11 = uVar2 << (uVar8 & 0xff);
    uVar2 = uVar2 >> (0x20 - uVar8 & 0xff) | uVar6 << (uVar8 & 0xff);
    uVar8 = uVar2 + -((int)uVar11 >> 0x1f);
    if (uVar4 == 0 && (uVar11 & 0x7fffffff) == 0) {
      uVar8 = uVar8 & ~(uVar11 >> 0x1f);
    }
    return CONCAT44((uVar6 & 0x80000000) + (uint)CARRY4(uVar2,-((int)uVar11 >> 0x1f)),uVar8);
  }
  uVar9 = uVar2 << (uVar8 + 0x20 & 0xff);
  uVar2 = uVar2 >> (uVar11 & 0xff) | uVar6 << (uVar8 + 0x20 & 0xff);
  uVar8 = uVar2 + -((int)uVar9 >> 0x1f);
  if (uVar4 == 0 && (uVar9 & 0x7fffffff) == 0) {
    uVar8 = uVar8 & ~(uVar9 >> 0x1f);
  }
  return CONCAT44((uVar6 & 0x80000000) + ((uVar6 & 0x7fffffff) >> (uVar11 & 0xff)) +
                  (uint)CARRY4(uVar2,-((int)uVar9 >> 0x1f)),uVar8);
}



/* ============================================= */
/* Function: FUN_0001428c */
/* Address: 0x0001428c */
/* ============================================= */

ulonglong FUN_0001428c(int param_1,uint param_2,int param_3,uint param_4)

{
  int iVar1;
  uint unaff_r4;
  uint uVar2;
  uint uVar3;
  uint in_r12;
  
  uVar2 = in_r12 & param_4 >> 0x14;
  uVar3 = param_2;
  if (unaff_r4 != in_r12 || uVar2 != in_r12) {
    if (unaff_r4 == in_r12) {
      if ((param_1 == 0 && (param_2 & 0xfffff) == 0) &&
         (param_1 = param_3, uVar3 = param_4, uVar2 != in_r12)) {
LAB_000140d4:
        return (ulonglong)((param_2 ^ param_4) & 0x80000000 | 0x7ff00000) << 0x20;
      }
    }
    else if (uVar2 == in_r12) {
      param_1 = param_3;
      uVar3 = param_4;
      if (param_3 == 0 && (param_4 & 0xfffff) == 0) {
LAB_00014088:
        return (ulonglong)((param_2 ^ param_4) & 0x80000000) << 0x20;
      }
    }
    else {
      if ((param_1 != 0 || (param_2 & 0x7fffffff) != 0) &&
          (param_3 != 0 || (param_4 & 0x7fffffff) != 0)) {
        if (unaff_r4 == 0) {
          uVar3 = param_2 & 0x80000000;
          do {
            iVar1 = param_1 >> 0x1f;
            param_1 = param_1 << 1;
            param_2 = param_2 * 2 - iVar1;
          } while ((param_2 & 0x100000) == 0);
          param_2 = param_2 | uVar3;
          if (uVar2 != 0) {
            return CONCAT44(param_2,param_1);
          }
        }
        do {
          iVar1 = param_3 >> 0x1f;
          param_3 = param_3 << 1;
          param_4 = param_4 * 2 - iVar1;
        } while ((param_4 & 0x100000) == 0);
        return CONCAT44(param_2,param_1);
      }
      if (param_1 != 0 || (param_2 & 0x7fffffff) != 0) goto LAB_000140d4;
      if (param_3 != 0 || (param_4 & 0x7fffffff) != 0) goto LAB_00014088;
    }
  }
  return CONCAT44(uVar3,param_1) | 0x7ff8000000000000;
}



/* ============================================= */
/* Function: __gedf2 */
/* Address: 0x000142fc */
/* ============================================= */

uint __gedf2(uint param_1,uint param_2,uint param_3,uint param_4)

{
  uint uVar1;
  bool bVar2;
  bool bVar3;
  
  if (((int)(param_2 << 1) >> 0x15 == -1 || (int)(param_4 << 1) >> 0x15 == -1) &&
     ((((int)(param_2 << 1) >> 0x15 == -1 && (param_1 != 0 || (param_2 & 0xfffff) != 0)) ||
      (((int)(param_4 << 1) >> 0x15 == -1 && (param_3 != 0 || (param_4 & 0xfffff) != 0)))))) {
    return 0xffffffff;
  }
  bVar3 = (param_2 & 0x7fffffff) == 0;
  bVar2 = param_1 == 0 && bVar3;
  if (param_1 == 0 && bVar3) {
    bVar2 = param_3 == 0 && (param_4 & 0x7fffffff) == 0;
  }
  if (!bVar2) {
    bVar2 = param_2 == param_4;
  }
  if (!bVar2 || param_1 != param_3) {
    uVar1 = param_2 ^ param_4;
    bVar3 = uVar1 == 0;
    if (-1 < (int)uVar1) {
      bVar3 = param_2 == param_4;
    }
    bVar2 = -1 < (int)uVar1 && param_4 <= param_2;
    if (bVar3) {
      bVar2 = param_3 <= param_1;
    }
    param_4 = (int)param_4 >> 0x1f;
    if (!bVar2) {
      param_4 = ~param_4;
    }
    return param_4 | 1;
  }
  return 0;
}



/* ============================================= */
/* Function: __ledf2 */
/* Address: 0x00014304 */
/* ============================================= */

uint __ledf2(uint param_1,uint param_2,uint param_3,uint param_4)

{
  uint uVar1;
  bool bVar2;
  bool bVar3;
  
  if (((int)(param_2 << 1) >> 0x15 == -1 || (int)(param_4 << 1) >> 0x15 == -1) &&
     ((((int)(param_2 << 1) >> 0x15 == -1 && (param_1 != 0 || (param_2 & 0xfffff) != 0)) ||
      (((int)(param_4 << 1) >> 0x15 == -1 && (param_3 != 0 || (param_4 & 0xfffff) != 0)))))) {
    return 1;
  }
  bVar3 = (param_2 & 0x7fffffff) == 0;
  bVar2 = param_1 == 0 && bVar3;
  if (param_1 == 0 && bVar3) {
    bVar2 = param_3 == 0 && (param_4 & 0x7fffffff) == 0;
  }
  if (!bVar2) {
    bVar2 = param_2 == param_4;
  }
  if (!bVar2 || param_1 != param_3) {
    uVar1 = param_2 ^ param_4;
    bVar3 = uVar1 == 0;
    if (-1 < (int)uVar1) {
      bVar3 = param_2 == param_4;
    }
    bVar2 = -1 < (int)uVar1 && param_4 <= param_2;
    if (bVar3) {
      bVar2 = param_3 <= param_1;
    }
    param_4 = (int)param_4 >> 0x1f;
    if (!bVar2) {
      param_4 = ~param_4;
    }
    return param_4 | 1;
  }
  return 0;
}



/* ============================================= */
/* Function: __nedf2 */
/* Address: 0x0001430c */
/* ============================================= */

uint __nedf2(uint param_1,uint param_2,uint param_3,uint param_4)

{
  uint uVar1;
  bool bVar2;
  bool bVar3;
  
  if (((int)(param_2 << 1) >> 0x15 == -1 || (int)(param_4 << 1) >> 0x15 == -1) &&
     ((((int)(param_2 << 1) >> 0x15 == -1 && (param_1 != 0 || (param_2 & 0xfffff) != 0)) ||
      (((int)(param_4 << 1) >> 0x15 == -1 && (param_3 != 0 || (param_4 & 0xfffff) != 0)))))) {
    return 1;
  }
  bVar3 = (param_2 & 0x7fffffff) == 0;
  bVar2 = param_1 == 0 && bVar3;
  if (param_1 == 0 && bVar3) {
    bVar2 = param_3 == 0 && (param_4 & 0x7fffffff) == 0;
  }
  if (!bVar2) {
    bVar2 = param_2 == param_4;
  }
  if (!bVar2 || param_1 != param_3) {
    uVar1 = param_2 ^ param_4;
    bVar3 = uVar1 == 0;
    if (-1 < (int)uVar1) {
      bVar3 = param_2 == param_4;
    }
    bVar2 = -1 < (int)uVar1 && param_4 <= param_2;
    if (bVar3) {
      bVar2 = param_3 <= param_1;
    }
    param_4 = (int)param_4 >> 0x1f;
    if (!bVar2) {
      param_4 = ~param_4;
    }
    return param_4 | 1;
  }
  return 0;
}



/* ============================================= */
/* Function: __aeabi_cdrcmple */
/* Address: 0x00014394 */
/* ============================================= */

void __aeabi_cdrcmple(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  __aeabi_cdcmpeq(param_3,param_4,param_1,param_2);
  return;
}



/* ============================================= */
/* Function: __aeabi_cdcmpeq */
/* Address: 0x000143b0 */
/* ============================================= */

undefined4 __aeabi_cdcmpeq(undefined4 param_1)

{
  __nedf2();
  return param_1;
}



/* ============================================= */
/* Function: __aeabi_dcmpeq */
/* Address: 0x000143c4 */
/* ============================================= */

undefined1 __aeabi_dcmpeq(void)

{
  undefined1 in_ZR;
  
  __aeabi_cdcmpeq();
  return in_ZR;
}



/* ============================================= */
/* Function: __aeabi_dcmplt */
/* Address: 0x000143d8 */
/* ============================================= */

bool __aeabi_dcmplt(void)

{
  undefined1 in_CY;
  
  __aeabi_cdcmpeq();
  return !(bool)in_CY;
}



/* ============================================= */
/* Function: __aeabi_dcmple */
/* Address: 0x000143ec */
/* ============================================= */

bool __aeabi_dcmple(void)

{
  undefined1 in_ZR;
  undefined1 in_CY;
  
  __aeabi_cdcmpeq();
  return !(bool)in_CY || (bool)in_ZR;
}



/* ============================================= */
/* Function: __aeabi_dcmpge */
/* Address: 0x00014400 */
/* ============================================= */

bool __aeabi_dcmpge(void)

{
  undefined1 in_ZR;
  undefined1 in_CY;
  
  __aeabi_cdrcmple();
  return !(bool)in_CY || (bool)in_ZR;
}



/* ============================================= */
/* Function: __aeabi_dcmpgt */
/* Address: 0x00014414 */
/* ============================================= */

bool __aeabi_dcmpgt(void)

{
  undefined1 in_CY;
  
  __aeabi_cdrcmple();
  return !(bool)in_CY;
}



/* ============================================= */
/* Function: __aeabi_d2iz */
/* Address: 0x00014428 */
/* ============================================= */

uint __aeabi_d2iz(uint param_1,uint param_2)

{
  uint uVar1;
  int iVar2;
  uint uVar3;
  
  iVar2 = param_2 * 2 + 0x200000;
  if (param_2 * 2 < 0xffe00000) {
    if (-1 < iVar2) {
      return 0;
    }
    uVar1 = iVar2 >> 0x15;
    uVar3 = -uVar1 - 0x3e1;
    if (uVar1 < 0xfffffc20 && uVar3 != 0) {
      uVar1 = (param_2 << 0xb | 0x80000000 | param_1 >> 0x15) >> (uVar3 & 0xff);
      if ((param_2 & 0x80000000) != 0) {
        uVar1 = -uVar1;
      }
      return uVar1;
    }
  }
  else if (param_1 != 0 || (param_2 & 0xfffff) != 0) {
    return 0;
  }
  param_2 = param_2 & 0x80000000;
  if (param_2 == 0) {
    param_2 = 0x7fffffff;
  }
  return param_2;
}



/* ============================================= */
/* Function: __div0 */
/* Address: 0x00014484 */
/* ============================================= */

void __div0(void)

{
  raise(8);
  return;
}



/* ============================================= */
/* Function: _Unwind_VRS_Get */
/* Address: 0x00014494 */
/* ============================================= */

undefined4 _Unwind_VRS_Get(int param_1,uint param_2,uint param_3,int param_4,undefined4 *param_5)

{
  bool bVar1;
  
  if (param_2 == 0) {
    bVar1 = param_3 == 0xf;
    if (param_3 < 0x10) {
      bVar1 = param_4 == 0;
    }
    if (bVar1) {
      *param_5 = *(undefined4 *)(param_1 + param_3 * 4 + 4);
      return 0;
    }
  }
  else if (param_2 < 5) {
    return 1;
  }
  return 2;
}



/* ============================================= */
/* Function: _Unwind_VRS_Set */
/* Address: 0x000144e0 */
/* ============================================= */

undefined4 _Unwind_VRS_Set(int param_1,uint param_2,uint param_3,int param_4,undefined4 *param_5)

{
  bool bVar1;
  
  if (param_2 == 0) {
    bVar1 = param_3 == 0xf;
    if (param_3 < 0x10) {
      bVar1 = param_4 == 0;
    }
    if (bVar1) {
      *(undefined4 *)(param_1 + param_3 * 4 + 4) = *param_5;
      return 0;
    }
  }
  else if (param_2 < 5) {
    return 1;
  }
  return 2;
}



/* ============================================= */
/* Function: FUN_0001452c */
/* Address: 0x0001452c */
/* ============================================= */

int FUN_0001452c(uint *param_1)

{
  uint uVar1;
  
  uVar1 = *param_1;
  if ((uVar1 & 0x40000000) == 0) {
    uVar1 = uVar1 & 0x7fffffff;
  }
  else {
    uVar1 = uVar1 | 0x80000000;
  }
  return (int)param_1 + uVar1;
}



/* ============================================= */
/* Function: _Unwind_GetCFA */
/* Address: 0x00014544 */
/* ============================================= */

undefined4 _Unwind_GetCFA(int param_1)

{
  return *(undefined4 *)(param_1 + 0x44);
}



/* ============================================= */
/* Function: _Unwind_Complete */
/* Address: 0x0001454c */
/* ============================================= */

void _Unwind_Complete(void)

{
  return;
}



/* ============================================= */
/* Function: _Unwind_DeleteException */
/* Address: 0x00014550 */
/* ============================================= */

void _Unwind_DeleteException(int param_1)

{
  if (*(code **)(param_1 + 8) == (code *)0x0) {
    return;
  }
  (**(code **)(param_1 + 8))(1,param_1);
  return;
}



/* ============================================= */
/* Function: __aeabi_unwind_cpp_pr2 */
/* Address: 0x00014a04 */
/* ============================================= */

/* WARNING: Removing unreachable block (ram,0x000145a4) */
/* WARNING: Removing unreachable block (ram,0x00014620) */
/* WARNING: Removing unreachable block (ram,0x00014640) */

undefined4 __aeabi_unwind_cpp_pr2(uint param_1,code *param_2,undefined4 param_3)

{
  bool bVar1;
  int iVar2;
  code *pcVar3;
  undefined4 uVar4;
  uint uVar5;
  uint *puVar6;
  uint *puVar7;
  uint uVar8;
  uint *puVar9;
  uint uVar10;
  bool bVar11;
  code *pcVar12;
  uint uStack_58;
  int iStack_3c;
  uint *puStack_38;
  undefined1 uStack_34;
  undefined1 uStack_33;
  code *pcStack_30;
  code *apcStack_2c [2];
  
  uVar8 = param_1 & 3;
  puStack_38 = *(uint **)(param_2 + 0x4c) + 1;
  uVar5 = **(uint **)(param_2 + 0x4c);
  iStack_3c = uVar5 << 0x10;
  puVar9 = puStack_38 + (uVar5 >> 0x10 & 0xff);
  uStack_34 = 2;
  uStack_33 = (undefined1)(uVar5 >> 0x10);
  if (uVar8 == 2) {
    puVar9 = *(uint **)(param_2 + 0x38);
  }
  if ((*(uint *)(param_2 + 0x50) & 1) == 0) {
    pcVar3 = param_2 + 0x58;
    bVar11 = false;
LAB_00014968:
    do {
      while( true ) {
        uVar5 = *puVar9;
        if (uVar5 == 0) goto LAB_00014974;
        uVar10 = puVar9[1];
        puVar7 = puVar9 + 2;
        pcVar12 = (code *)((uVar10 & 0xfffffffe) + *(int *)(param_2 + 0x48));
        _Unwind_VRS_Get(param_3,0,0xf,0,&pcStack_30);
        if (pcStack_30 < pcVar12) {
          bVar1 = false;
        }
        else if (pcStack_30 < pcVar12 + (uVar5 & 0xfffffffe)) {
          bVar1 = true;
        }
        else {
          bVar1 = false;
        }
        uVar5 = uVar5 & 1 | (uVar10 & 1) << 1;
        if (uVar5 != 1) break;
        if (uVar8 == 0) {
          if (bVar1) {
            uVar5 = puVar9[3];
            if (uVar5 == 0xfffffffe) {
              return 9;
            }
            apcStack_2c[0] = pcVar3;
            if (uVar5 != 0xffffffff) {
              if (uVar5 == 0) {
                uVar4 = 0;
              }
              else {
                uVar4 = *(undefined4 *)((int)puVar9 + uVar5 + 0xc);
              }
              iVar2 = __cxa_type_match(param_2,uVar4,*puVar7 >> 0x1f,apcStack_2c);
              if (iVar2 == 0) {
                apcStack_2c[0] = (code *)0x0;
              }
            }
            if (apcStack_2c[0] != (code *)0x0) {
LAB_00014868:
              _Unwind_VRS_Get(param_3,0,0xd,0,&pcStack_30);
              *(uint **)(param_2 + 0x28) = puVar7;
              *(code **)(param_2 + 0x20) = pcStack_30;
              *(code **)(param_2 + 0x24) = apcStack_2c[0];
              return 6;
            }
          }
        }
        else {
          pcVar12 = *(code **)(param_2 + 0x20);
          _Unwind_VRS_Get(param_3,0,0xd,0,&pcStack_30);
          if ((pcVar12 == pcStack_30) && (puVar7 == *(uint **)(param_2 + 0x28))) {
            pcStack_30 = (code *)FUN_0001452c(puVar7);
            _Unwind_VRS_Set(param_3,0,0xf,0,&pcStack_30);
LAB_0001492c:
            uVar4 = 0;
            pcStack_30 = param_2;
            goto LAB_000149e8;
          }
        }
        puVar9 = puVar9 + 4;
      }
      if (uVar5 != 0) {
        if (uVar5 != 2) {
          return 9;
        }
        uVar5 = *puVar7 & 0x7fffffff;
        if (uVar8 == 0) {
          if ((bVar1) && ((param_1 & 8) == 0 || uVar5 == 0)) {
            uStack_58 = 0;
            puVar6 = puVar7;
            do {
              if (uStack_58 == uVar5) goto LAB_00014868;
              puVar6 = puVar6 + 1;
              uStack_58 = uStack_58 + 1;
              uVar4 = 0;
              if (*puVar6 != 0) {
                uVar4 = *(undefined4 *)((int)puVar6 + *puVar6);
              }
              apcStack_2c[0] = pcVar3;
              iVar2 = __cxa_type_match(param_2,uVar4,0,apcStack_2c);
            } while (iVar2 == 0);
          }
        }
        else {
          pcVar12 = *(code **)(param_2 + 0x20);
          _Unwind_VRS_Get(param_3,0,0xd,0,&pcStack_30);
          if ((pcVar12 == pcStack_30) && (puVar7 == *(uint **)(param_2 + 0x28))) {
            *(uint **)(param_2 + 0x34) = puVar9 + 3;
            *(uint *)(param_2 + 0x28) = uVar5;
            *(undefined4 *)(param_2 + 0x2c) = 0;
            *(undefined4 *)(param_2 + 0x30) = 4;
            if (-1 < (int)*puVar7) {
              pcStack_30 = (code *)FUN_0001452c(puVar7 + uVar5 + 1);
              _Unwind_VRS_Set(param_3,0,0xf,0,&pcStack_30);
              goto LAB_0001492c;
            }
            bVar11 = true;
          }
        }
        if ((int)*puVar7 < 0) {
          puVar7 = puVar9 + 3;
        }
        puVar9 = puVar7 + uVar5 + 1;
        goto LAB_00014968;
      }
      if (uVar8 == 0) {
        bVar1 = false;
      }
      puVar9 = puVar9 + 3;
    } while (!bVar1);
    pcVar3 = (code *)FUN_0001452c(puVar7);
    *(uint **)(param_2 + 0x38) = puVar9;
    iVar2 = __cxa_begin_cleanup(param_2);
    if (iVar2 != 0) {
      uVar4 = 0xf;
      pcStack_30 = pcVar3;
      goto LAB_000149e8;
    }
  }
  else {
    bVar11 = false;
LAB_00014974:
    iVar2 = __gnu_unwind_execute(param_3,&iStack_3c);
    if (iVar2 == 0) {
      if (!bVar11) {
        return 8;
      }
      _Unwind_VRS_Get(param_3,0,0xf,0,&pcStack_30);
      _Unwind_VRS_Set(param_3,0,0xe,0,&pcStack_30);
      uVar4 = 0xf;
      pcStack_30 = __cxa_call_unexpected;
LAB_000149e8:
      _Unwind_VRS_Set(param_3,0,uVar4,0,&pcStack_30);
      return 7;
    }
  }
  return 9;
}



/* ============================================= */
/* Function: __aeabi_unwind_cpp_pr1 */
/* Address: 0x00014a0c */
/* ============================================= */

/* WARNING: Removing unreachable block (ram,0x000145a4) */
/* WARNING: Removing unreachable block (ram,0x00014624) */
/* WARNING: Removing unreachable block (ram,0x0001463c) */

undefined4 __aeabi_unwind_cpp_pr1(uint param_1,code *param_2,undefined4 param_3)

{
  ushort uVar1;
  bool bVar2;
  int iVar3;
  code *pcVar4;
  undefined4 uVar5;
  uint uVar6;
  uint *puVar7;
  uint *puVar8;
  uint uVar9;
  uint *puVar10;
  bool bVar11;
  code *pcVar12;
  uint uStack_58;
  int iStack_3c;
  uint *puStack_38;
  undefined1 uStack_34;
  undefined1 uStack_33;
  code *pcStack_30;
  code *apcStack_2c [2];
  
  uVar9 = param_1 & 3;
  puStack_38 = *(uint **)(param_2 + 0x4c) + 1;
  uVar6 = **(uint **)(param_2 + 0x4c);
  iStack_3c = uVar6 << 0x10;
  puVar10 = puStack_38 + (uVar6 >> 0x10 & 0xff);
  uStack_34 = 2;
  uStack_33 = (undefined1)(uVar6 >> 0x10);
  if (uVar9 == 2) {
    puVar10 = *(uint **)(param_2 + 0x38);
  }
  if ((*(uint *)(param_2 + 0x50) & 1) == 0) {
    pcVar4 = param_2 + 0x58;
    bVar11 = false;
LAB_00014968:
    do {
      while( true ) {
        if (*puVar10 == 0) goto LAB_00014974;
        uVar1 = *(ushort *)((int)puVar10 + 2);
        uVar6 = *puVar10;
        puVar8 = puVar10 + 1;
        pcVar12 = (code *)((uVar1 & 0xfffffffe) + *(int *)(param_2 + 0x48));
        _Unwind_VRS_Get(param_3,0,0xf,0,&pcStack_30);
        if (pcStack_30 < pcVar12) {
          bVar2 = false;
        }
        else if (pcStack_30 < pcVar12 + ((ushort)uVar6 & 0xfffffffe)) {
          bVar2 = true;
        }
        else {
          bVar2 = false;
        }
        uVar6 = (ushort)uVar6 & 1 | (uVar1 & 1) << 1;
        if (uVar6 != 1) break;
        if (uVar9 == 0) {
          if (bVar2) {
            uVar6 = puVar10[2];
            if (uVar6 == 0xfffffffe) {
              return 9;
            }
            apcStack_2c[0] = pcVar4;
            if (uVar6 != 0xffffffff) {
              if (uVar6 == 0) {
                uVar5 = 0;
              }
              else {
                uVar5 = *(undefined4 *)((int)puVar10 + uVar6 + 8);
              }
              iVar3 = __cxa_type_match(param_2,uVar5,*puVar8 >> 0x1f,apcStack_2c);
              if (iVar3 == 0) {
                apcStack_2c[0] = (code *)0x0;
              }
            }
            if (apcStack_2c[0] != (code *)0x0) {
LAB_00014868:
              _Unwind_VRS_Get(param_3,0,0xd,0,&pcStack_30);
              *(uint **)(param_2 + 0x28) = puVar8;
              *(code **)(param_2 + 0x20) = pcStack_30;
              *(code **)(param_2 + 0x24) = apcStack_2c[0];
              return 6;
            }
          }
        }
        else {
          pcVar12 = *(code **)(param_2 + 0x20);
          _Unwind_VRS_Get(param_3,0,0xd,0,&pcStack_30);
          if ((pcVar12 == pcStack_30) && (puVar8 == *(uint **)(param_2 + 0x28))) {
            pcStack_30 = (code *)FUN_0001452c(puVar8);
            _Unwind_VRS_Set(param_3,0,0xf,0,&pcStack_30);
LAB_0001492c:
            uVar5 = 0;
            pcStack_30 = param_2;
            goto LAB_000149e8;
          }
        }
        puVar10 = puVar10 + 3;
      }
      if (uVar6 != 0) {
        if (uVar6 != 2) {
          return 9;
        }
        uVar6 = *puVar8 & 0x7fffffff;
        if (uVar9 == 0) {
          if ((bVar2) && ((param_1 & 8) == 0 || uVar6 == 0)) {
            uStack_58 = 0;
            puVar7 = puVar8;
            do {
              if (uStack_58 == uVar6) goto LAB_00014868;
              puVar7 = puVar7 + 1;
              uStack_58 = uStack_58 + 1;
              uVar5 = 0;
              if (*puVar7 != 0) {
                uVar5 = *(undefined4 *)((int)puVar7 + *puVar7);
              }
              apcStack_2c[0] = pcVar4;
              iVar3 = __cxa_type_match(param_2,uVar5,0,apcStack_2c);
            } while (iVar3 == 0);
          }
        }
        else {
          pcVar12 = *(code **)(param_2 + 0x20);
          _Unwind_VRS_Get(param_3,0,0xd,0,&pcStack_30);
          if ((pcVar12 == pcStack_30) && (puVar8 == *(uint **)(param_2 + 0x28))) {
            *(uint **)(param_2 + 0x34) = puVar10 + 2;
            *(uint *)(param_2 + 0x28) = uVar6;
            *(undefined4 *)(param_2 + 0x2c) = 0;
            *(undefined4 *)(param_2 + 0x30) = 4;
            if (-1 < (int)*puVar8) {
              pcStack_30 = (code *)FUN_0001452c(puVar8 + uVar6 + 1);
              _Unwind_VRS_Set(param_3,0,0xf,0,&pcStack_30);
              goto LAB_0001492c;
            }
            bVar11 = true;
          }
        }
        if ((int)*puVar8 < 0) {
          puVar8 = puVar10 + 2;
        }
        puVar10 = puVar8 + uVar6 + 1;
        goto LAB_00014968;
      }
      if (uVar9 == 0) {
        bVar2 = false;
      }
      puVar10 = puVar10 + 2;
    } while (!bVar2);
    pcVar4 = (code *)FUN_0001452c(puVar8);
    *(uint **)(param_2 + 0x38) = puVar10;
    iVar3 = __cxa_begin_cleanup(param_2);
    if (iVar3 != 0) {
      uVar5 = 0xf;
      pcStack_30 = pcVar4;
      goto LAB_000149e8;
    }
  }
  else {
    bVar11 = false;
LAB_00014974:
    iVar3 = __gnu_unwind_execute(param_3,&iStack_3c);
    if (iVar3 == 0) {
      if (!bVar11) {
        return 8;
      }
      _Unwind_VRS_Get(param_3,0,0xf,0,&pcStack_30);
      _Unwind_VRS_Set(param_3,0,0xe,0,&pcStack_30);
      uVar5 = 0xf;
      pcStack_30 = __cxa_call_unexpected;
LAB_000149e8:
      _Unwind_VRS_Set(param_3,0,uVar5,0,&pcStack_30);
      return 7;
    }
  }
  return 9;
}



/* ============================================= */
/* Function: __aeabi_unwind_cpp_pr0 */
/* Address: 0x00014a14 */
/* ============================================= */

/* WARNING: Removing unreachable block (ram,0x000145c0) */
/* WARNING: Removing unreachable block (ram,0x00014624) */
/* WARNING: Removing unreachable block (ram,0x0001463c) */

undefined4 __aeabi_unwind_cpp_pr0(uint param_1,code *param_2,undefined4 param_3)

{
  ushort uVar1;
  bool bVar2;
  int iVar3;
  code *pcVar4;
  undefined4 uVar5;
  uint *puVar6;
  uint *puVar7;
  uint uVar8;
  uint *puVar9;
  uint uVar10;
  bool bVar11;
  code *pcVar12;
  uint local_58;
  int local_3c;
  uint *local_38;
  undefined1 local_34;
  undefined1 local_33;
  code *local_30;
  code *local_2c [2];
  
  uVar8 = param_1 & 3;
  local_38 = (uint *)(*(int **)(param_2 + 0x4c) + 1);
  local_3c = **(int **)(param_2 + 0x4c) << 8;
  local_34 = 3;
  local_33 = 0;
  puVar9 = local_38;
  if (uVar8 == 2) {
    puVar9 = *(uint **)(param_2 + 0x38);
  }
  if ((*(uint *)(param_2 + 0x50) & 1) == 0) {
    pcVar4 = param_2 + 0x58;
    bVar11 = false;
LAB_00014968:
    do {
      while( true ) {
        if (*puVar9 == 0) goto LAB_00014974;
        uVar1 = *(ushort *)((int)puVar9 + 2);
        uVar10 = *puVar9;
        puVar7 = puVar9 + 1;
        pcVar12 = (code *)((uVar1 & 0xfffffffe) + *(int *)(param_2 + 0x48));
        _Unwind_VRS_Get(param_3,0,0xf,0,&local_30);
        if (local_30 < pcVar12) {
          bVar2 = false;
        }
        else if (local_30 < pcVar12 + ((ushort)uVar10 & 0xfffffffe)) {
          bVar2 = true;
        }
        else {
          bVar2 = false;
        }
        uVar10 = (ushort)uVar10 & 1 | (uVar1 & 1) << 1;
        if (uVar10 != 1) break;
        if (uVar8 == 0) {
          if (bVar2) {
            uVar10 = puVar9[2];
            if (uVar10 == 0xfffffffe) {
              return 9;
            }
            local_2c[0] = pcVar4;
            if (uVar10 != 0xffffffff) {
              if (uVar10 == 0) {
                uVar5 = 0;
              }
              else {
                uVar5 = *(undefined4 *)((int)puVar9 + uVar10 + 8);
              }
              iVar3 = __cxa_type_match(param_2,uVar5,*puVar7 >> 0x1f,local_2c);
              if (iVar3 == 0) {
                local_2c[0] = (code *)0x0;
              }
            }
            if (local_2c[0] != (code *)0x0) {
LAB_00014868:
              _Unwind_VRS_Get(param_3,0,0xd,0,&local_30);
              *(uint **)(param_2 + 0x28) = puVar7;
              *(code **)(param_2 + 0x20) = local_30;
              *(code **)(param_2 + 0x24) = local_2c[0];
              return 6;
            }
          }
        }
        else {
          pcVar12 = *(code **)(param_2 + 0x20);
          _Unwind_VRS_Get(param_3,0,0xd,0,&local_30);
          if ((pcVar12 == local_30) && (puVar7 == *(uint **)(param_2 + 0x28))) {
            local_30 = (code *)FUN_0001452c(puVar7);
            _Unwind_VRS_Set(param_3,0,0xf,0,&local_30);
LAB_0001492c:
            uVar5 = 0;
            local_30 = param_2;
            goto LAB_000149e8;
          }
        }
        puVar9 = puVar9 + 3;
      }
      if (uVar10 != 0) {
        if (uVar10 != 2) {
          return 9;
        }
        uVar10 = *puVar7 & 0x7fffffff;
        if (uVar8 == 0) {
          if ((bVar2) && ((param_1 & 8) == 0 || uVar10 == 0)) {
            local_58 = 0;
            puVar6 = puVar7;
            do {
              if (local_58 == uVar10) goto LAB_00014868;
              puVar6 = puVar6 + 1;
              local_58 = local_58 + 1;
              uVar5 = 0;
              if (*puVar6 != 0) {
                uVar5 = *(undefined4 *)((int)puVar6 + *puVar6);
              }
              local_2c[0] = pcVar4;
              iVar3 = __cxa_type_match(param_2,uVar5,0,local_2c);
            } while (iVar3 == 0);
          }
        }
        else {
          pcVar12 = *(code **)(param_2 + 0x20);
          _Unwind_VRS_Get(param_3,0,0xd,0,&local_30);
          if ((pcVar12 == local_30) && (puVar7 == *(uint **)(param_2 + 0x28))) {
            *(uint **)(param_2 + 0x34) = puVar9 + 2;
            *(uint *)(param_2 + 0x28) = uVar10;
            *(undefined4 *)(param_2 + 0x2c) = 0;
            *(undefined4 *)(param_2 + 0x30) = 4;
            if (-1 < (int)*puVar7) {
              local_30 = (code *)FUN_0001452c(puVar7 + uVar10 + 1);
              _Unwind_VRS_Set(param_3,0,0xf,0,&local_30);
              goto LAB_0001492c;
            }
            bVar11 = true;
          }
        }
        if ((int)*puVar7 < 0) {
          puVar7 = puVar9 + 2;
        }
        puVar9 = puVar7 + uVar10 + 1;
        goto LAB_00014968;
      }
      if (uVar8 == 0) {
        bVar2 = false;
      }
      puVar9 = puVar9 + 2;
    } while (!bVar2);
    pcVar4 = (code *)FUN_0001452c(puVar7);
    *(uint **)(param_2 + 0x38) = puVar9;
    iVar3 = __cxa_begin_cleanup(param_2);
    if (iVar3 != 0) {
      uVar5 = 0xf;
      local_30 = pcVar4;
      goto LAB_000149e8;
    }
  }
  else {
    bVar11 = false;
LAB_00014974:
    iVar3 = __gnu_unwind_execute(param_3,&local_3c);
    if (iVar3 == 0) {
      if (!bVar11) {
        return 8;
      }
      _Unwind_VRS_Get(param_3,0,0xf,0,&local_30);
      _Unwind_VRS_Set(param_3,0,0xe,0,&local_30);
      uVar5 = 0xf;
      local_30 = __cxa_call_unexpected;
LAB_000149e8:
      _Unwind_VRS_Set(param_3,0,uVar5,0,&local_30);
      return 7;
    }
  }
  return 9;
}



/* ============================================= */
/* Function: FUN_00014a1c */
/* Address: 0x00014a1c */
/* ============================================= */

/* WARNING: Removing unreachable block (ram,0x00014a68) */

undefined4 FUN_00014a1c(int param_1,int param_2)

{
  int iVar1;
  int iVar2;
  uint uVar3;
  int iVar4;
  undefined4 uVar5;
  uint uVar6;
  uint uVar7;
  int iVar8;
  int iVar9;
  int iVar10;
  int local_2c [2];
  
  uVar7 = param_2 - 2;
  iVar2 = __gnu_Unwind_Find_exidx(uVar7,local_2c);
  if (iVar2 == 0) {
    *(undefined4 *)(param_1 + 0x10) = 0;
    return 9;
  }
  if (local_2c[0] != 0) {
    iVar9 = 0;
    iVar10 = local_2c[0] + -1;
    do {
      while( true ) {
        iVar1 = (iVar10 + iVar9) / 2;
        iVar8 = iVar2 + iVar1 * 8;
        uVar3 = FUN_0001452c(iVar8);
        uVar6 = 0xffffffff;
        if (iVar1 != local_2c[0] + -1) {
          iVar4 = FUN_0001452c(iVar2 + (iVar1 + 1) * 8);
          uVar6 = iVar4 - 1;
        }
        if (uVar3 <= uVar7) break;
        iVar10 = iVar1 + -1;
        if (iVar1 == iVar9) goto LAB_00014ba8;
      }
      iVar9 = iVar1 + 1;
    } while (uVar6 < uVar7);
    if (iVar8 != 0) {
      uVar5 = FUN_0001452c(iVar8);
      *(undefined4 *)(param_1 + 0x48) = uVar5;
      if (*(int *)(iVar8 + 4) == 1) {
        *(undefined4 *)(param_1 + 0x10) = 0;
        return 5;
      }
      if (*(int *)(iVar8 + 4) < 0) {
        *(int *)(param_1 + 0x4c) = iVar8 + 4;
        *(undefined4 *)(param_1 + 0x50) = 1;
      }
      else {
        uVar5 = FUN_0001452c();
        *(undefined4 *)(param_1 + 0x50) = 0;
        *(undefined4 *)(param_1 + 0x4c) = uVar5;
      }
      if (-1 < (int)**(uint **)(param_1 + 0x4c)) {
        uVar5 = FUN_0001452c();
        *(undefined4 *)(param_1 + 0x10) = uVar5;
        return 0;
      }
      uVar7 = **(uint **)(param_1 + 0x4c) >> 0x18 & 0xf;
      if (uVar7 == 0) {
        iVar2 = 0x4c;
      }
      else if (uVar7 == 1) {
        iVar2 = 0x6c;
      }
      else {
        if (uVar7 != 2) goto LAB_00014ba8;
        iVar2 = 0x58;
      }
      *(undefined4 *)(param_1 + 0x10) = *(undefined4 *)((int)&__DT_PLTGOT + iVar2);
      return 0;
    }
  }
LAB_00014ba8:
  *(undefined4 *)(param_1 + 0x10) = 0;
  return 9;
}



/* ============================================= */
/* Function: FUN_00014bec */
/* Address: 0x00014bec */
/* ============================================= */

void FUN_00014bec(uint *param_1)

{
  if ((*param_1 & 1) == 0) {
    if ((*param_1 & 2) == 0) {
      __gnu_Unwind_Restore_VFP(param_1 + 0x12);
    }
    else {
      __gnu_Unwind_Restore_VFP_D();
    }
  }
  if ((*param_1 & 4) == 0) {
    __gnu_Unwind_Restore_VFP_D_16_to_31(param_1 + 0x34);
  }
  if ((*param_1 & 8) == 0) {
    __gnu_Unwind_Restore_WMMXD(param_1 + 0x6c);
  }
  if ((*param_1 & 0x10) != 0) {
    return;
  }
  __gnu_Unwind_Restore_WMMXC(param_1 + 0x8c);
  return;
}



/* ============================================= */
/* Function: __gnu_Unwind_Backtrace */
/* Address: 0x00014c58 */
/* ============================================= */

int __gnu_Unwind_Backtrace(code *param_1,undefined4 param_2,int param_3)

{
  int iVar1;
  undefined4 local_2c0;
  undefined4 local_2bc;
  undefined4 uStack_2b8;
  undefined4 uStack_2b4;
  undefined4 uStack_2b0;
  undefined4 local_2ac;
  undefined4 uStack_2a8;
  undefined4 uStack_2a4;
  undefined4 uStack_2a0;
  undefined4 local_29c;
  undefined4 uStack_298;
  undefined4 uStack_294;
  undefined4 uStack_290;
  undefined4 local_28c;
  undefined4 uStack_288;
  undefined4 uStack_284;
  undefined4 local_280;
  undefined1 auStack_80 [16];
  code *local_70;
  undefined1 *local_24 [2];
  
  *(undefined4 *)(param_3 + 0x40) = *(undefined4 *)(param_3 + 0x3c);
  local_2bc = *(undefined4 *)(param_3 + 4);
  uStack_2b8 = *(undefined4 *)(param_3 + 8);
  uStack_2b4 = *(undefined4 *)(param_3 + 0xc);
  uStack_2b0 = *(undefined4 *)(param_3 + 0x10);
  local_2ac = *(undefined4 *)(param_3 + 0x14);
  uStack_2a8 = *(undefined4 *)(param_3 + 0x18);
  uStack_2a4 = *(undefined4 *)(param_3 + 0x1c);
  uStack_2a0 = *(undefined4 *)(param_3 + 0x20);
  local_29c = *(undefined4 *)(param_3 + 0x24);
  uStack_298 = *(undefined4 *)(param_3 + 0x28);
  uStack_294 = *(undefined4 *)(param_3 + 0x2c);
  uStack_290 = *(undefined4 *)(param_3 + 0x30);
  local_28c = *(undefined4 *)(param_3 + 0x34);
  uStack_288 = *(undefined4 *)(param_3 + 0x38);
  uStack_284 = *(undefined4 *)(param_3 + 0x3c);
  local_280 = *(undefined4 *)(param_3 + 0x40);
  local_2c0 = 0xffffffff;
  do {
    iVar1 = FUN_00014a1c(auStack_80,local_280);
    if (iVar1 != 0) {
LAB_00014d14:
      iVar1 = 9;
      break;
    }
    local_24[0] = auStack_80;
    _Unwind_VRS_Set(&local_2c0,0,0xc,0,local_24);
    iVar1 = (*param_1)(&local_2c0,param_2);
    if (iVar1 != 0) goto LAB_00014d14;
    iVar1 = (*local_70)(8,auStack_80,&local_2c0);
  } while (iVar1 != 9 && iVar1 != 5);
  FUN_00014bec(&local_2c0);
  return iVar1;
}



/* ============================================= */
/* Function: FUN_00014d2c */
/* Address: 0x00014d2c */
/* ============================================= */

int FUN_00014d2c(int param_1,int param_2,int param_3)

{
  int iVar1;
  int iVar2;
  int iVar3;
  code *pcVar4;
  uint uVar5;
  undefined4 uVar6;
  undefined1 auStack_4a8 [56];
  undefined4 local_470;
  undefined4 local_268;
  undefined4 local_264;
  undefined4 uStack_260;
  undefined4 uStack_25c;
  undefined4 uStack_258;
  undefined4 local_254;
  undefined4 uStack_250;
  undefined4 uStack_24c;
  undefined4 uStack_248;
  undefined4 local_244;
  undefined4 uStack_240;
  undefined4 uStack_23c;
  undefined4 uStack_238;
  undefined4 local_234;
  undefined4 local_230;
  undefined4 uStack_22c;
  undefined4 local_228;
  undefined4 local_224;
  
  local_264 = *(undefined4 *)(param_2 + 4);
  uStack_260 = *(undefined4 *)(param_2 + 8);
  uStack_25c = *(undefined4 *)(param_2 + 0xc);
  uStack_258 = *(undefined4 *)(param_2 + 0x10);
  local_254 = *(undefined4 *)(param_2 + 0x14);
  uStack_250 = *(undefined4 *)(param_2 + 0x18);
  uStack_24c = *(undefined4 *)(param_2 + 0x1c);
  uStack_248 = *(undefined4 *)(param_2 + 0x20);
  local_244 = *(undefined4 *)(param_2 + 0x24);
  uStack_240 = *(undefined4 *)(param_2 + 0x28);
  uStack_23c = *(undefined4 *)(param_2 + 0x2c);
  uStack_238 = *(undefined4 *)(param_2 + 0x30);
  local_234 = *(undefined4 *)(param_2 + 0x34);
  local_230 = *(undefined4 *)(param_2 + 0x38);
  uStack_22c = *(undefined4 *)(param_2 + 0x3c);
  local_228 = *(undefined4 *)(param_2 + 0x40);
  iVar3 = 0;
  pcVar4 = *(code **)(param_1 + 0xc);
  uVar6 = *(undefined4 *)(param_1 + 0x18);
  local_268 = 0;
  do {
    iVar1 = FUN_00014a1c(param_1,local_228);
    if (param_3 == 0) {
      uVar5 = 9;
    }
    else {
      uVar5 = 10;
    }
    if (iVar1 == 0) {
      *(undefined4 *)(param_1 + 0x14) = local_228;
      memcpy(auStack_4a8,&local_268,0x240);
      iVar3 = (**(code **)(param_1 + 0x10))(uVar5,param_1,auStack_4a8);
      local_224 = local_470;
    }
    else {
      uVar5 = uVar5 | 0x10;
      local_224 = local_230;
    }
    iVar2 = (*pcVar4)(1,uVar5,param_1,param_1,&local_268,uVar6);
    if (iVar2 != 0) {
      return 9;
    }
    if (iVar1 != 0) {
      return iVar1;
    }
    memcpy(&local_268,auStack_4a8,0x240);
    param_3 = 0;
  } while (iVar3 == 8);
  if (iVar3 == 7) {
    restore_core_regs(&local_264);
  }
  return 9;
}



/* ============================================= */
/* Function: __gnu_Unwind_ForcedUnwind */
/* Address: 0x00014e48 */
/* ============================================= */

void __gnu_Unwind_ForcedUnwind(int param_1,undefined4 param_2,undefined4 param_3,int param_4)

{
  *(undefined4 *)(param_1 + 0xc) = param_2;
  *(undefined4 *)(param_1 + 0x18) = param_3;
  *(undefined4 *)(param_4 + 0x40) = *(undefined4 *)(param_4 + 0x3c);
  FUN_00014d2c(param_1,param_4,0);
  return;
}



/* ============================================= */
/* Function: FUN_00014e64 */
/* Address: 0x00014e64 */
/* ============================================= */

undefined4 FUN_00014e64(int param_1,int param_2)

{
  int iVar1;
  int iVar2;
  int iVar3;
  undefined8 uVar4;
  undefined4 uStack_268;
  undefined4 uStack_264;
  undefined4 uStack_260;
  undefined4 uStack_25c;
  undefined4 uStack_258;
  undefined4 uStack_254;
  undefined4 uStack_250;
  undefined4 uStack_24c;
  undefined4 uStack_248;
  undefined4 uStack_244;
  undefined4 uStack_240;
  undefined4 uStack_23c;
  undefined4 uStack_238;
  undefined4 uStack_234;
  undefined4 uStack_230;
  undefined4 uStack_22c;
  undefined4 uStack_228;
  int iStack_24;
  int iStack_20;
  
  do {
    iVar1 = FUN_00014a1c(param_1,*(undefined4 *)(param_2 + 0x40));
    if (iVar1 != 0) goto LAB_00014eb0;
    *(undefined4 *)(param_1 + 0x14) = *(undefined4 *)(param_2 + 0x40);
    iVar1 = (**(code **)(param_1 + 0x10))(1,param_1,param_2);
  } while (iVar1 == 8);
  if (iVar1 == 7) {
    uVar4 = restore_core_regs(param_2 + 4);
    iVar3 = (int)((ulonglong)uVar4 >> 0x20);
    iVar1 = (int)uVar4;
    *(undefined4 *)(iVar3 + 0x40) = *(undefined4 *)(iVar3 + 0x3c);
    uStack_264 = *(undefined4 *)(iVar3 + 4);
    uStack_260 = *(undefined4 *)(iVar3 + 8);
    uStack_25c = *(undefined4 *)(iVar3 + 0xc);
    uStack_258 = *(undefined4 *)(iVar3 + 0x10);
    uStack_254 = *(undefined4 *)(iVar3 + 0x14);
    uStack_250 = *(undefined4 *)(iVar3 + 0x18);
    uStack_24c = *(undefined4 *)(iVar3 + 0x1c);
    uStack_248 = *(undefined4 *)(iVar3 + 0x20);
    uStack_244 = *(undefined4 *)(iVar3 + 0x24);
    uStack_240 = *(undefined4 *)(iVar3 + 0x28);
    uStack_23c = *(undefined4 *)(iVar3 + 0x2c);
    uStack_238 = *(undefined4 *)(iVar3 + 0x30);
    uStack_234 = *(undefined4 *)(iVar3 + 0x34);
    uStack_230 = *(undefined4 *)(iVar3 + 0x38);
    uStack_22c = *(undefined4 *)(iVar3 + 0x3c);
    uStack_228 = *(undefined4 *)(iVar3 + 0x40);
    uStack_268 = 0xffffffff;
    iStack_24 = param_2;
    iStack_20 = param_1;
    do {
      iVar2 = FUN_00014a1c(iVar1,uStack_228);
      if (iVar2 != 0) {
        return 9;
      }
      iVar2 = (**(code **)(iVar1 + 0x10))(0,iVar1,&uStack_268);
    } while (iVar2 == 8);
    FUN_00014bec(&uStack_268);
    if (iVar2 == 6) {
      FUN_00014e64(iVar1,iVar3);
    }
    return 9;
  }
LAB_00014eb0:
                    /* WARNING: Subroutine does not return */
  abort();
}



/* ============================================= */
/* Function: __gnu_Unwind_RaiseException */
/* Address: 0x00014ebc */
/* ============================================= */

undefined4 __gnu_Unwind_RaiseException(int param_1,int param_2)

{
  int iVar1;
  undefined4 local_258;
  undefined4 local_254;
  undefined4 uStack_250;
  undefined4 uStack_24c;
  undefined4 uStack_248;
  undefined4 local_244;
  undefined4 uStack_240;
  undefined4 uStack_23c;
  undefined4 uStack_238;
  undefined4 local_234;
  undefined4 uStack_230;
  undefined4 uStack_22c;
  undefined4 uStack_228;
  undefined4 local_224;
  undefined4 uStack_220;
  undefined4 uStack_21c;
  undefined4 local_218;
  
  *(undefined4 *)(param_2 + 0x40) = *(undefined4 *)(param_2 + 0x3c);
  local_254 = *(undefined4 *)(param_2 + 4);
  uStack_250 = *(undefined4 *)(param_2 + 8);
  uStack_24c = *(undefined4 *)(param_2 + 0xc);
  uStack_248 = *(undefined4 *)(param_2 + 0x10);
  local_244 = *(undefined4 *)(param_2 + 0x14);
  uStack_240 = *(undefined4 *)(param_2 + 0x18);
  uStack_23c = *(undefined4 *)(param_2 + 0x1c);
  uStack_238 = *(undefined4 *)(param_2 + 0x20);
  local_234 = *(undefined4 *)(param_2 + 0x24);
  uStack_230 = *(undefined4 *)(param_2 + 0x28);
  uStack_22c = *(undefined4 *)(param_2 + 0x2c);
  uStack_228 = *(undefined4 *)(param_2 + 0x30);
  local_224 = *(undefined4 *)(param_2 + 0x34);
  uStack_220 = *(undefined4 *)(param_2 + 0x38);
  uStack_21c = *(undefined4 *)(param_2 + 0x3c);
  local_218 = *(undefined4 *)(param_2 + 0x40);
  local_258 = 0xffffffff;
  do {
    iVar1 = FUN_00014a1c(param_1,local_218);
    if (iVar1 != 0) {
      return 9;
    }
    iVar1 = (**(code **)(param_1 + 0x10))(0,param_1,&local_258);
  } while (iVar1 == 8);
  FUN_00014bec(&local_258);
  if (iVar1 == 6) {
    FUN_00014e64(param_1,param_2);
  }
  return 9;
}



/* ============================================= */
/* Function: __gnu_Unwind_Resume_or_Rethrow */
/* Address: 0x00014f60 */
/* ============================================= */

void __gnu_Unwind_Resume_or_Rethrow(int param_1,int param_2)

{
  if (*(int *)(param_1 + 0xc) == 0) {
    __gnu_Unwind_RaiseException();
    return;
  }
  *(undefined4 *)(param_2 + 0x40) = *(undefined4 *)(param_2 + 0x3c);
  FUN_00014d2c(param_1,param_2,0);
  return;
}



/* ============================================= */
/* Function: __gnu_Unwind_Resume */
/* Address: 0x00014f80 */
/* ============================================= */

void __gnu_Unwind_Resume(int param_1,int param_2)

{
  int iVar1;
  
  *(undefined4 *)(param_2 + 0x40) = *(undefined4 *)(param_1 + 0x14);
  if (*(int *)(param_1 + 0xc) == 0) {
    iVar1 = (**(code **)(param_1 + 0x10))(2,param_1,param_2);
    if (iVar1 == 7) {
      restore_core_regs(param_2 + 4);
    }
    else if (iVar1 != 8) goto LAB_00014fe8;
    FUN_00014e64(param_1,param_2);
  }
  else {
    FUN_00014d2c(param_1,param_2,1);
  }
LAB_00014fe8:
                    /* WARNING: Subroutine does not return */
  abort();
}



/* ============================================= */
/* Function: _Unwind_VRS_Pop */
/* Address: 0x00014fec */
/* ============================================= */

undefined4 _Unwind_VRS_Pop(uint *param_1,undefined4 param_2,uint param_3,int param_4)

{
  uint *puVar1;
  uint uVar2;
  uint uVar3;
  undefined4 *puVar4;
  int iVar5;
  uint uVar6;
  int iVar7;
  bool bVar8;
  bool bVar9;
  undefined4 auStack_1b8 [34];
  undefined4 auStack_130 [32];
  undefined4 local_b0 [32];
  undefined4 local_30 [5];
  
  switch(param_2) {
  case 0:
    if (param_4 != 0) {
      return 2;
    }
    puVar1 = (uint *)param_1[0xe];
    uVar6 = 0;
    do {
      if ((param_3 & 0xffff & 1 << (uVar6 & 0xff)) != 0) {
        param_1[uVar6 + 1] = *puVar1;
        puVar1 = puVar1 + 1;
      }
      uVar6 = uVar6 + 1;
    } while (uVar6 != 0x10);
    if ((param_3 & 0x2000) != 0) {
      return 0;
    }
    param_1[0xe] = (uint)puVar1;
    return 0;
  case 1:
    if (param_4 != 1 && param_4 != 5) {
      return 2;
    }
    uVar6 = param_3 & 0xffff;
    param_3 = param_3 >> 0x10;
    if (param_4 == 1) {
      uVar3 = 0x10;
    }
    else {
      uVar3 = 0x20;
    }
    uVar2 = uVar6 + param_3;
    if (uVar3 < uVar2) {
      return 2;
    }
    bVar8 = param_4 == 1;
    if (param_3 >= 0x10 && bVar8) {
      return 2;
    }
    uVar3 = uVar6;
    if ((param_3 < 0x10) && (uVar3 = 0, 0x10 < uVar2)) {
      uVar3 = uVar2 - 0x10;
    }
    if (uVar3 != 0 && param_4 != 5) {
      return 2;
    }
    if ((param_3 < 0x10) && (uVar2 = *param_1, (uVar2 & 1) != 0)) {
      *param_1 = uVar2 & 0xfffffffe;
      if (param_4 == 5) {
        *param_1 = uVar2 & 0xfffffffe | 2;
        __gnu_Unwind_Save_VFP_D();
      }
      else {
        *param_1 = uVar2 & 0xfffffffc;
        __gnu_Unwind_Save_VFP(param_1 + 0x12);
      }
    }
    if ((0 < (int)uVar3) && ((*param_1 & 4) != 0)) {
      *param_1 = *param_1 & 0xfffffffb;
      __gnu_Unwind_Save_VFP_D_16_to_31(param_1 + 0x34);
    }
    if (bVar8) {
      __gnu_Unwind_Save_VFP(auStack_1b8);
    }
    else {
      if (param_3 < 0x10) {
        __gnu_Unwind_Save_VFP_D(auStack_1b8);
      }
      if (uVar3 == 0) goto LAB_00015188;
      __gnu_Unwind_Save_VFP_D_16_to_31(local_b0);
    }
    if (0 < (int)uVar3) {
      uVar6 = 0x10 - param_3;
    }
LAB_00015188:
    uVar2 = param_1[0xe];
    if (0 < (int)uVar6) {
      iVar7 = uVar6 * 2;
      iVar5 = 0;
      while (bVar9 = iVar7 != 0, iVar7 = iVar7 + -1, bVar9) {
        *(undefined4 *)((int)auStack_1b8 + iVar5 + param_3 * 8) = *(undefined4 *)(iVar5 + uVar2);
        iVar5 = iVar5 + 4;
      }
      uVar2 = uVar2 + uVar6 * 8;
    }
    if (0 < (int)uVar3) {
      uVar6 = param_3;
      if (param_3 < 0x10) {
        uVar6 = 0x10;
      }
      iVar7 = uVar3 * 2;
      iVar5 = 0;
      while (bVar9 = iVar7 != 0, iVar7 = iVar7 + -1, bVar9) {
        *(undefined4 *)((int)local_b0 + iVar5 + (uVar6 - 0x10) * 8) = *(undefined4 *)(uVar2 + iVar5)
        ;
        iVar5 = iVar5 + 4;
      }
      uVar2 = uVar2 + uVar3 * 8;
    }
    if (bVar8) {
      uVar2 = uVar2 + 4;
    }
    param_1[0xe] = uVar2;
    if (bVar8) {
      __gnu_Unwind_Restore_VFP(auStack_1b8);
    }
    else {
      if (param_3 < 0x10) {
        __gnu_Unwind_Restore_VFP_D(auStack_1b8);
      }
      if (0 < (int)uVar3) {
        __gnu_Unwind_Restore_VFP_D_16_to_31(local_b0);
      }
    }
    return 0;
  case 2:
    return 1;
  case 3:
    if (param_4 == 3) {
      uVar6 = param_3 & 0xffff;
      if (uVar6 + (param_3 >> 0x10) < 0x11) {
        if ((*param_1 & 8) != 0) {
          *param_1 = *param_1 & 0xfffffff7;
          __gnu_Unwind_Save_WMMXD(param_1 + 0x6c);
        }
        __gnu_Unwind_Save_WMMXD(auStack_130);
        uVar3 = param_1[0xe];
        iVar5 = 0;
        for (iVar7 = uVar6 * 2; iVar7 != 0; iVar7 = iVar7 + -1) {
          *(undefined4 *)((int)auStack_130 + iVar5 + (param_3 >> 0x10) * 8) =
               *(undefined4 *)(iVar5 + uVar3);
          iVar5 = iVar5 + 4;
        }
        param_1[0xe] = uVar3 + uVar6 * 8;
        __gnu_Unwind_Restore_WMMXD(auStack_130);
        return 0;
      }
    }
    break;
  case 4:
    bVar8 = param_3 == 0x10;
    if (param_3 < 0x11) {
      bVar8 = param_4 == 0;
    }
    if (bVar8) {
      if ((*param_1 & 0x10) != 0) {
        *param_1 = *param_1 & 0xffffffef;
        __gnu_Unwind_Save_WMMXC(param_1 + 0x8c);
      }
      __gnu_Unwind_Save_WMMXC(local_30);
      puVar4 = (undefined4 *)param_1[0xe];
      uVar6 = 0;
      do {
        if ((param_3 & 1 << (uVar6 & 0xff)) != 0) {
          local_30[uVar6] = *puVar4;
          puVar4 = puVar4 + 1;
        }
        uVar6 = uVar6 + 1;
      } while (uVar6 != 4);
      param_1[0xe] = (uint)puVar4;
      __gnu_Unwind_Restore_WMMXC(local_30);
      return 0;
    }
  }
  return 2;
}



/* ============================================= */
/* Function: restore_core_regs */
/* Address: 0x00015358 */
/* ============================================= */

undefined8 restore_core_regs(undefined8 *param_1)

{
  return *param_1;
}



/* ============================================= */
/* Function: __gnu_Unwind_Restore_VFP */
/* Address: 0x0001536c */
/* ============================================= */

undefined4 __gnu_Unwind_Restore_VFP(undefined8 *param_1)

{
  return (int)*param_1;
}



/* ============================================= */
/* Function: __gnu_Unwind_Save_VFP */
/* Address: 0x00015374 */
/* ============================================= */

void __gnu_Unwind_Save_VFP(undefined8 *param_1)

{
  undefined8 in_d0;
  undefined8 in_d1;
  undefined8 in_d2;
  undefined8 in_d3;
  undefined8 in_d4;
  undefined8 in_d5;
  undefined8 in_d6;
  undefined8 in_d7;
  undefined8 unaff_d8;
  undefined8 unaff_d9;
  undefined8 unaff_d10;
  undefined8 unaff_d11;
  undefined8 unaff_d12;
  undefined8 unaff_d13;
  undefined8 unaff_d14;
  undefined8 unaff_d15;
  
  *param_1 = in_d0;
  param_1[1] = in_d1;
  param_1[2] = in_d2;
  param_1[3] = in_d3;
  param_1[4] = in_d4;
  param_1[5] = in_d5;
  param_1[6] = in_d6;
  param_1[7] = in_d7;
  param_1[8] = unaff_d8;
  param_1[9] = unaff_d9;
  param_1[10] = unaff_d10;
  param_1[0xb] = unaff_d11;
  param_1[0xc] = unaff_d12;
  param_1[0xd] = unaff_d13;
  param_1[0xe] = unaff_d14;
  param_1[0xf] = unaff_d15;
  return;
}



/* ============================================= */
/* Function: __gnu_Unwind_Restore_VFP_D */
/* Address: 0x0001537c */
/* ============================================= */

undefined4 __gnu_Unwind_Restore_VFP_D(undefined8 *param_1)

{
  return (int)*param_1;
}



/* ============================================= */
/* Function: __gnu_Unwind_Save_VFP_D */
/* Address: 0x00015384 */
/* ============================================= */

void __gnu_Unwind_Save_VFP_D(undefined8 *param_1)

{
  undefined8 in_d0;
  undefined8 in_d1;
  undefined8 in_d2;
  undefined8 in_d3;
  undefined8 in_d4;
  undefined8 in_d5;
  undefined8 in_d6;
  undefined8 in_d7;
  undefined8 unaff_d8;
  undefined8 unaff_d9;
  undefined8 unaff_d10;
  undefined8 unaff_d11;
  undefined8 unaff_d12;
  undefined8 unaff_d13;
  undefined8 unaff_d14;
  undefined8 unaff_d15;
  
  *param_1 = in_d0;
  param_1[1] = in_d1;
  param_1[2] = in_d2;
  param_1[3] = in_d3;
  param_1[4] = in_d4;
  param_1[5] = in_d5;
  param_1[6] = in_d6;
  param_1[7] = in_d7;
  param_1[8] = unaff_d8;
  param_1[9] = unaff_d9;
  param_1[10] = unaff_d10;
  param_1[0xb] = unaff_d11;
  param_1[0xc] = unaff_d12;
  param_1[0xd] = unaff_d13;
  param_1[0xe] = unaff_d14;
  param_1[0xf] = unaff_d15;
  return;
}



/* ============================================= */
/* Function: __gnu_Unwind_Restore_VFP_D_16_to_31 */
/* Address: 0x0001538c */
/* ============================================= */

void __gnu_Unwind_Restore_VFP_D_16_to_31(void)

{
  return;
}



/* ============================================= */
/* Function: __gnu_Unwind_Save_VFP_D_16_to_31 */
/* Address: 0x00015394 */
/* ============================================= */

void __gnu_Unwind_Save_VFP_D_16_to_31(undefined8 *param_1)

{
  undefined8 in_d16;
  undefined8 in_d17;
  undefined8 in_d18;
  undefined8 in_d19;
  undefined8 in_d20;
  undefined8 in_d21;
  undefined8 in_d22;
  undefined8 in_d23;
  undefined8 in_d24;
  undefined8 in_d25;
  undefined8 in_d26;
  undefined8 in_d27;
  undefined8 in_d28;
  undefined8 in_d29;
  undefined8 in_d30;
  undefined8 in_d31;
  
  *param_1 = in_d16;
  param_1[1] = in_d17;
  param_1[2] = in_d18;
  param_1[3] = in_d19;
  param_1[4] = in_d20;
  param_1[5] = in_d21;
  param_1[6] = in_d22;
  param_1[7] = in_d23;
  param_1[8] = in_d24;
  param_1[9] = in_d25;
  param_1[10] = in_d26;
  param_1[0xb] = in_d27;
  param_1[0xc] = in_d28;
  param_1[0xd] = in_d29;
  param_1[0xe] = in_d30;
  param_1[0xf] = in_d31;
  return;
}



/* ============================================= */
/* Function: __gnu_Unwind_Restore_WMMXD */
/* Address: 0x0001539c */
/* ============================================= */

int __gnu_Unwind_Restore_WMMXD(int param_1)

{
  undefined4 in_cr0;
  undefined4 in_cr1;
  undefined4 in_cr2;
  undefined4 in_cr3;
  undefined4 in_cr4;
  undefined4 in_cr5;
  undefined4 in_cr6;
  undefined4 in_cr7;
  undefined4 in_cr8;
  undefined4 in_cr9;
  undefined4 in_cr10;
  undefined4 in_cr11;
  undefined4 in_cr12;
  undefined4 in_cr13;
  undefined4 in_cr14;
  undefined4 in_cr15;
  
  coprocessor_loadlong(1,in_cr0,param_1);
  coprocessor_loadlong(1,in_cr1,param_1 + 8);
  coprocessor_loadlong(1,in_cr2,param_1 + 0x10);
  coprocessor_loadlong(1,in_cr3,param_1 + 0x18);
  coprocessor_loadlong(1,in_cr4,param_1 + 0x20);
  coprocessor_loadlong(1,in_cr5,param_1 + 0x28);
  coprocessor_loadlong(1,in_cr6,param_1 + 0x30);
  coprocessor_loadlong(1,in_cr7,param_1 + 0x38);
  coprocessor_loadlong(1,in_cr8,param_1 + 0x40);
  coprocessor_loadlong(1,in_cr9,param_1 + 0x48);
  coprocessor_loadlong(1,in_cr10,param_1 + 0x50);
  coprocessor_loadlong(1,in_cr11,param_1 + 0x58);
  coprocessor_loadlong(1,in_cr12,param_1 + 0x60);
  coprocessor_loadlong(1,in_cr13,param_1 + 0x68);
  coprocessor_loadlong(1,in_cr14,param_1 + 0x70);
  coprocessor_loadlong(1,in_cr15,param_1 + 0x78);
  return param_1 + 0x80;
}



/* ============================================= */
/* Function: __gnu_Unwind_Save_WMMXD */
/* Address: 0x000153e0 */
/* ============================================= */

int __gnu_Unwind_Save_WMMXD(int param_1)

{
  undefined4 in_cr0;
  undefined4 in_cr1;
  undefined4 in_cr2;
  undefined4 in_cr3;
  undefined4 in_cr4;
  undefined4 in_cr5;
  undefined4 in_cr6;
  undefined4 in_cr7;
  undefined4 in_cr8;
  undefined4 in_cr9;
  undefined4 in_cr10;
  undefined4 in_cr11;
  undefined4 in_cr12;
  undefined4 in_cr13;
  undefined4 in_cr14;
  undefined4 in_cr15;
  
  coprocessor_storelong(1,in_cr0,param_1);
  coprocessor_storelong(1,in_cr1,param_1 + 8);
  coprocessor_storelong(1,in_cr2,param_1 + 0x10);
  coprocessor_storelong(1,in_cr3,param_1 + 0x18);
  coprocessor_storelong(1,in_cr4,param_1 + 0x20);
  coprocessor_storelong(1,in_cr5,param_1 + 0x28);
  coprocessor_storelong(1,in_cr6,param_1 + 0x30);
  coprocessor_storelong(1,in_cr7,param_1 + 0x38);
  coprocessor_storelong(1,in_cr8,param_1 + 0x40);
  coprocessor_storelong(1,in_cr9,param_1 + 0x48);
  coprocessor_storelong(1,in_cr10,param_1 + 0x50);
  coprocessor_storelong(1,in_cr11,param_1 + 0x58);
  coprocessor_storelong(1,in_cr12,param_1 + 0x60);
  coprocessor_storelong(1,in_cr13,param_1 + 0x68);
  coprocessor_storelong(1,in_cr14,param_1 + 0x70);
  coprocessor_storelong(1,in_cr15,param_1 + 0x78);
  return param_1 + 0x80;
}



/* ============================================= */
/* Function: __gnu_Unwind_Restore_WMMXC */
/* Address: 0x00015424 */
/* ============================================= */

int __gnu_Unwind_Restore_WMMXC(int param_1)

{
  undefined4 in_cr8;
  undefined4 in_cr9;
  undefined4 in_cr10;
  undefined4 in_cr11;
  
  coprocessor_load2(1,in_cr8,param_1);
  coprocessor_load2(1,in_cr9,param_1 + 4);
  coprocessor_load2(1,in_cr10,param_1 + 8);
  coprocessor_load2(1,in_cr11,param_1 + 0xc);
  return param_1 + 0x10;
}



/* ============================================= */
/* Function: __gnu_Unwind_Save_WMMXC */
/* Address: 0x00015438 */
/* ============================================= */

int __gnu_Unwind_Save_WMMXC(int param_1)

{
  undefined4 in_cr8;
  undefined4 in_cr9;
  undefined4 in_cr10;
  undefined4 in_cr11;
  
  coprocessor_store2(1,in_cr8,param_1);
  coprocessor_store2(1,in_cr9,param_1 + 4);
  coprocessor_store2(1,in_cr10,param_1 + 8);
  coprocessor_store2(1,in_cr11,param_1 + 0xc);
  return param_1 + 0x10;
}



/* ============================================= */
/* Function: _Unwind_RaiseException */
/* Address: 0x0001544c */
/* ============================================= */

void _Unwind_RaiseException
               (undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_44;
  undefined4 uStack_40;
  undefined4 uStack_3c;
  undefined4 uStack_38;
  undefined4 uStack_34;
  
  uStack_44 = 0;
  uStack_40 = param_1;
  uStack_3c = param_2;
  uStack_38 = param_3;
  uStack_34 = param_4;
  __gnu_Unwind_RaiseException(param_1,&uStack_44,param_3,0,param_3);
  return;
}



/* ============================================= */
/* Function: _Unwind_Resume */
/* Address: 0x00015470 */
/* ============================================= */

void _Unwind_Resume(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_44;
  undefined4 uStack_40;
  undefined4 uStack_3c;
  undefined4 uStack_38;
  undefined4 uStack_34;
  
  uStack_44 = 0;
  uStack_40 = param_1;
  uStack_3c = param_2;
  uStack_38 = param_3;
  uStack_34 = param_4;
  __gnu_Unwind_Resume(param_1,&uStack_44,param_3,0,param_3);
  return;
}



/* ============================================= */
/* Function: ___Unwind_Resume_or_Rethrow */
/* Address: 0x00015494 */
/* ============================================= */

void ___Unwind_Resume_or_Rethrow
               (undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  undefined4 uStack_44;
  undefined4 uStack_40;
  undefined4 uStack_3c;
  undefined4 uStack_38;
  undefined4 uStack_34;
  
  uStack_44 = 0;
  uStack_40 = param_1;
  uStack_3c = param_2;
  uStack_38 = param_3;
  uStack_34 = param_4;
  __gnu_Unwind_Resume_or_Rethrow(param_1,&uStack_44,param_3,0,param_3);
  return;
}



/* ============================================= */
/* Function: _Unwind_ForcedUnwind */
/* Address: 0x000154b8 */
/* ============================================= */

void _Unwind_ForcedUnwind(void)

{
  __gnu_Unwind_ForcedUnwind();
  return;
}



/* ============================================= */
/* Function: ___Unwind_Backtrace */
/* Address: 0x000154dc */
/* ============================================= */

void ___Unwind_Backtrace(undefined4 param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4
                        )

{
  undefined4 uStack_44;
  undefined4 uStack_40;
  undefined4 uStack_3c;
  undefined4 uStack_38;
  undefined4 uStack_34;
  
  uStack_44 = 0;
  uStack_40 = param_1;
  uStack_3c = param_2;
  uStack_38 = param_3;
  uStack_34 = param_4;
  __gnu_Unwind_Backtrace(param_1,param_2,&uStack_44,0,param_3);
  return;
}



/* ============================================= */
/* Function: FUN_00015500 */
/* Address: 0x00015500 */
/* ============================================= */

uint FUN_00015500(uint *param_1)

{
  uint uVar1;
  char cVar2;
  
  if ((char)param_1[2] == '\0') {
    if (*(char *)((int)param_1 + 9) == '\0') {
      return 0xb0;
    }
    *(char *)((int)param_1 + 9) = *(char *)((int)param_1 + 9) + -1;
    *param_1 = *(uint *)param_1[1];
    param_1[1] = (uint)((uint *)param_1[1] + 1);
    cVar2 = '\x03';
  }
  else {
    cVar2 = (char)param_1[2] + -1;
  }
  uVar1 = *param_1;
  *(char *)(param_1 + 2) = cVar2;
  *param_1 = uVar1 << 8;
  return uVar1 >> 0x18;
}



/* ============================================= */
/* Function: _Unwind_GetTextRelBase */
/* Address: 0x00015558 */
/* ============================================= */

void _Unwind_GetTextRelBase(void)

{
                    /* WARNING: Subroutine does not return */
  abort();
}



/* ============================================= */
/* Function: _Unwind_GetDataRelBase */
/* Address: 0x00015560 */
/* ============================================= */

void _Unwind_GetDataRelBase(void)

{
                    /* WARNING: Subroutine does not return */
  abort();
}



/* ============================================= */
/* Function: _Unwind_GetLanguageSpecificData */
/* Address: 0x00015568 */
/* ============================================= */

int _Unwind_GetLanguageSpecificData(void)

{
  int in_r3;
  
  _Unwind_VRS_Get();
  return *(int *)(in_r3 + 0x4c) + (uint)*(byte *)(*(int *)(in_r3 + 0x4c) + 7) * 4 + 8;
}



/* ============================================= */
/* Function: _Unwind_GetRegionStart */
/* Address: 0x000155a0 */
/* ============================================= */

undefined4 _Unwind_GetRegionStart(void)

{
  int in_r3;
  
  _Unwind_VRS_Get();
  return *(undefined4 *)(in_r3 + 0x48);
}



/* ============================================= */
/* Function: __gnu_unwind_execute */
/* Address: 0x000155cc */
/* ============================================= */

undefined4 __gnu_unwind_execute(int *param_1,undefined4 param_2,undefined4 param_3,int param_4)

{
  uint uVar1;
  undefined4 uVar2;
  uint uVar3;
  bool bVar4;
  int iVar5;
  int *piVar6;
  undefined4 uVar7;
  int local_24;
  
  bVar4 = false;
  piVar6 = param_1;
  uVar7 = param_2;
  local_24 = param_4;
LAB_000155e4:
  do {
    while( true ) {
      uVar1 = FUN_00015500(param_2);
      if (uVar1 == 0xb0) {
        if (bVar4) {
          return 0;
        }
        _Unwind_VRS_Get(param_1,0,0xe,0,&local_24,uVar7,param_3);
        _Unwind_VRS_Set(param_1,0,0xf,0,&local_24);
        return 0;
      }
      if ((uVar1 & 0x80) != 0) break;
      _Unwind_VRS_Get(param_1,0,0xd,0,&local_24,uVar7,param_3);
      iVar5 = (uVar1 & 0x3f) * 4 + 4;
      if ((uVar1 & 0x40) != 0) {
        iVar5 = -iVar5;
      }
      local_24 = local_24 + iVar5;
LAB_0001571c:
      piVar6 = &local_24;
      _Unwind_VRS_Set(param_1,0,0xd,0);
    }
    uVar3 = uVar1 & 0xf0;
    if (uVar3 == 0x80) {
      uVar3 = FUN_00015500(param_2);
      uVar3 = uVar3 | uVar1 << 8;
      if (uVar3 == 0x8000) {
        return 9;
      }
      iVar5 = _Unwind_VRS_Pop(param_1,0,(uVar3 << 0x14) >> 0x10,0,piVar6);
      if (iVar5 != 0) {
        return 9;
      }
      if ((uVar3 & 0x800) != 0) {
        bVar4 = true;
      }
      goto LAB_000155e4;
    }
    if (uVar3 == 0x90) {
      uVar1 = uVar1 & 0xf;
      if (uVar1 == 0xd || uVar1 == 0xf) {
        return 9;
      }
      _Unwind_VRS_Get(param_1,0,uVar1,0,&local_24,uVar7,param_3);
      goto LAB_0001571c;
    }
    if (uVar3 == 0xa0) {
      uVar2 = 0;
    }
    else if (uVar3 == 0xb0) {
      if (uVar1 == 0xb1) {
        uVar1 = FUN_00015500(param_2);
        if (uVar1 == 0) {
          return 9;
        }
        uVar2 = 0;
        if ((uVar1 & 0xf0) != 0) {
          return 9;
        }
      }
      else {
        if (uVar1 == 0xb2) {
          uVar1 = 2;
          _Unwind_VRS_Get(param_1,0,0xd,0,&local_24,uVar7,param_3);
          while( true ) {
            uVar3 = FUN_00015500(param_2);
            if ((uVar3 & 0x80) == 0) break;
            local_24 = local_24 + ((uVar3 & 0x7f) << (uVar1 & 0xff));
            uVar1 = uVar1 + 7;
          }
          local_24 = local_24 + 0x204 + ((uVar3 & 0x7f) << (uVar1 & 0xff));
          goto LAB_0001571c;
        }
        if (uVar1 == 0xb3) {
          FUN_00015500(param_2);
          uVar2 = 1;
        }
        else if ((uVar1 & 0xfc) == 0xb4) {
          uVar2 = 2;
        }
        else {
          uVar2 = 1;
        }
      }
    }
    else if (uVar3 == 0xc0) {
      if (uVar1 == 0xc6) {
        FUN_00015500(param_2);
        uVar2 = 3;
      }
      else if (uVar1 == 199) {
        uVar1 = FUN_00015500(param_2);
        if (uVar1 == 0) {
          return 9;
        }
        if ((uVar1 & 0xf0) != 0) {
          return 9;
        }
        uVar2 = 4;
      }
      else {
        if ((uVar1 & 0xf8) != 0xc0) {
          if (uVar1 == 200) {
            FUN_00015500(param_2);
          }
          else {
            if (uVar1 != 0xc9) {
              return 9;
            }
            FUN_00015500(param_2);
          }
          goto LAB_00015920;
        }
        uVar2 = 3;
      }
    }
    else {
      if ((uVar1 & 0xf8) != 0xd0) {
        return 9;
      }
LAB_00015920:
      uVar2 = 1;
    }
    iVar5 = _Unwind_VRS_Pop(param_1,uVar2);
    if (iVar5 != 0) {
      return 9;
    }
  } while( true );
}



/* ============================================= */
/* Function: __gnu_unwind_frame */
/* Address: 0x00015944 */
/* ============================================= */

void __gnu_unwind_frame(int param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  int local_14;
  int local_10;
  undefined1 local_c;
  undefined1 local_b;
  undefined2 uStack_a;
  
  iVar1 = *(int *)(param_1 + 0x4c);
  local_14 = *(int *)(iVar1 + 4) << 8;
  local_10 = iVar1 + 8;
  uStack_a = (undefined2)((uint)param_4 >> 0x10);
  _local_c = CONCAT11(*(undefined1 *)(iVar1 + 7),3);
  __gnu_unwind_execute(param_2,&local_14,*(undefined1 *)(iVar1 + 7),local_10,param_1);
  return;
}



/* ============================================= */
/* Function: __aeabi_atexit */
/* Address: 0x00015984 */
/* ============================================= */

void __aeabi_atexit(undefined4 param_1,undefined4 param_2)

{
  __cxa_atexit(param_2,param_1);
  return;
}



