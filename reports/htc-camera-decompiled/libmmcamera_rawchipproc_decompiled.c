/* Decompiled from libmmcamera_rawchipproc.so using Ghidra */
/* HTC EV Shooter Android ROM - Camera Binaries */

/* ============================================= */
/* Function: FUN_00011900 */
/* Address: 0x00011900 */
/* ============================================= */

int FUN_00011900(undefined1 param_1)

{
  int iVar1;
  uint uVar2;
  undefined1 local_19;
  undefined4 local_18;
  undefined4 local_14;
  undefined4 local_10;
  undefined1 *local_c;
  
  local_18 = 8;
  local_c = &local_19;
  local_14 = 2000;
  local_10 = 1;
  local_19 = param_1;
  iVar1 = ioctl(*rawchipCtrl,0x40046706,&local_18);
  if (iVar1 < 0) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC",
                          "%s RAWCHIP_IOCTL_SET_DXOPRC_AF_STRATEGY: failed\n",
                          "rawchip_set_dxoaf_params");
    }
  }
  return iVar1;
}



/* ============================================= */
/* Function: rawchip_af_params_setting */
/* Address: 0x00011984 */
/* ============================================= */

void rawchip_af_params_setting
               (ushort *param_1,undefined2 *param_2,undefined2 param_3,undefined4 *param_4,
               ushort param_5)

{
  byte bVar1;
  ushort uVar2;
  short sVar3;
  ushort uVar4;
  undefined4 uVar5;
  undefined2 uVar6;
  undefined4 uVar7;
  ushort *puVar8;
  int iVar9;
  uint uVar10;
  int iVar11;
  uint uVar12;
  int iVar13;
  uint uVar14;
  uint uVar15;
  int iVar16;
  
  uVar6 = dxoframeSetting._2_2_;
  uVar12 = (uint)(ushort)dxoframeSetting._6_2_;
  uVar10 = (uint)(ushort)dxoframeSetting._2_2_;
  param_2[3] = dxoframeSetting._6_2_;
  param_2[1] = uVar6;
  uVar6 = dxoframeSetting._4_2_;
  param_2[4] = dxoframeSetting._8_2_;
  param_2[2] = uVar6;
  uVar15 = (uint)(ushort)dxoframeSetting._14_2_;
  uVar14 = (uint)(ushort)dxoframeSetting._16_2_;
  *(char *)(param_2 + 0x10e) =
       (char)((int)((uint)(ushort)dxoframeSetting._10_2_ + (uint)(ushort)dxoframeSetting._12_2_) >>
             1);
  iVar13 = (uVar12 + 1) - uVar10;
  *(char *)((int)param_2 + 0x21d) = (char)((int)(uVar15 + uVar14) >> 1);
  uVar7 = DAT_0001b19c;
  iVar11 = 0;
  uVar4 = param_1[0x14];
  puVar8 = param_1;
  while( true ) {
    if ((int)(uint)(byte)uVar4 <= iVar11) break;
    iVar9 = iVar11 + 0x36;
    iVar11 = iVar11 + 1;
    iVar9 = iVar9 * 4;
    uVar2 = (ushort)iVar13;
    *puVar8 = (ushort)*(byte *)(rawchipCtrl + iVar9 + 9) * (uVar2 >> 4) + param_2[1];
    iVar16 = rawchipCtrl + iVar9;
    puVar8[2] = (*(byte *)(iVar16 + 0xb) + 1) * (uVar2 >> 4) + param_2[1] + -1;
    puVar8[1] = (ushort)*(byte *)(iVar16 + 10) * (uVar2 >> 4) + param_2[2];
    uVar2 = param_2[2];
    bVar1 = *(byte *)(iVar9 + rawchipCtrl + 0xc);
    *puVar8 = *puVar8 + 1 & 0xfffe;
    puVar8[2] = (short)(((int)(puVar8[2] + 1) >> 1) << 1) - 1;
    puVar8[1] = puVar8[1] + 1 & 0xfffe;
    puVar8[3] = (short)(((int)(((bVar1 + 1) * ((uint)(iVar13 * 0x10000) >> 0x14) + (uVar2 - 1) &
                               0xffff) + 1) >> 1) << 1) - 1;
    *(undefined4 *)(puVar8 + 0x12) = uVar7;
    uVar5 = FocusValue;
    *(uint *)(puVar8 + 0xe) = (uint)param_5;
    *(undefined4 *)(puVar8 + 0x10) = uVar5;
    *(undefined4 *)(puVar8 + 0xc) = param_4[3];
    *(undefined4 *)(puVar8 + 10) = param_4[2];
    *(undefined4 *)(puVar8 + 6) = param_4[1];
    *(undefined4 *)(puVar8 + 8) = *param_4;
    *(undefined1 *)(puVar8 + 4) = 1;
    puVar8 = puVar8 + 0x14;
    param_4 = param_4 + 4;
  }
  *(undefined1 *)(param_1 + 0x14) = *(undefined1 *)(rawchipCtrl + 0xe0);
  *param_2 = param_3;
  iVar11 = rawchipCtrl;
  iVar13 = 0;
  uVar7 = __aeabi_idiv((uint)*(ushort *)(rawchipCtrl + 0xd4) * 0x26000,0x7d);
  *(undefined4 *)(param_2 + 0x10c) = 0;
  *(undefined4 *)(param_2 + 0x108) = uVar7;
  uVar6 = __aeabi_idiv(256000000,(uint)*(ushort *)(iVar11 + 0xd4) * 0x13);
  uVar7 = DAT_0001b19c;
  param_2[0x10a] = uVar6;
  sVar3 = *(short *)(iVar11 + 0xd0);
  *(undefined4 *)(param_2 + 0x110) = uVar7;
  *(undefined4 *)(param_2 + 0x106) = uVar7;
  param_2[0x10b] = sVar3 << 3;
  do {
    iVar11 = 0;
    do {
      iVar9 = iVar11 + 1;
      param_2[iVar11 + 6] = (ushort)(byte)lumTable[iVar11 + iVar13];
      iVar11 = iVar9;
    } while (iVar9 != 0x10);
    iVar13 = iVar13 + 0x10;
    param_2 = param_2 + 0x10;
  } while (iVar13 != 0x100);
  return;
}



/* ============================================= */
/* Function: get_vcm_clib */
/* Address: 0x00011b94 */
/* ============================================= */

void get_vcm_clib(int param_1,int param_2)

{
  if (*(ushort *)(param_1 + 0x10) < *(ushort *)(param_1 + 0x12)) {
    if (*(ushort *)(param_1 + 0x12) < *(ushort *)(param_1 + 0x14)) {
      *(ushort *)(param_2 + 4) = *(ushort *)(param_1 + 0x10);
      *(undefined2 *)(param_2 + 6) = *(undefined2 *)(param_1 + 0x12);
      *(undefined2 *)(param_2 + 8) = *(undefined2 *)(param_1 + 0x14);
    }
  }
  return;
}



/* ============================================= */
/* Function: rawchip_load_af_algo_params */
/* Address: 0x00011bb4 */
/* ============================================= */

undefined4
rawchip_load_af_algo_params(int param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  uint uVar2;
  undefined *puVar3;
  
  if (param_1 == 3) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019134,param_4,param_4);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","load rawchip af params for s5k3h2yx\n");
    }
    puVar3 = &DAT_0001903e;
  }
  else if (param_1 == 9) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019134,uVar2 << 0x18,
                                 param_4);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","load rawchip af params for imx175\n");
    }
    puVar3 = &DAT_0001907a;
  }
  else if (param_1 == 7) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019134,param_4,param_4);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","load rawchip af params for s5k4e5yx\n");
    }
    puVar3 = &DAT_000190f4;
  }
  else if (param_1 == 6) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019134,uVar2 << 0x18,
                                 param_4);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","load rawchip af params for ov5693\n");
    }
    puVar3 = &DAT_000190b8;
  }
  else {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019134,param_4,param_4);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","load rawchip af params for default\n");
    }
    puVar3 = &DAT_00019000;
  }
  iVar1 = rawchipCtrl;
  *(undefined **)(rawchipCtrl + 0x44) = puVar3;
  *(undefined4 *)(iVar1 + 0x48) = 6;
  return 0;
}



/* ============================================= */
/* Function: rawchip_thread_ready_signal */
/* Address: 0x00011d54 */
/* ============================================= */

void rawchip_thread_ready_signal(void)

{
  int iVar1;
  
  pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0xc));
  iVar1 = rawchipCtrl;
  *(undefined4 *)(rawchipCtrl + 0x10) = 1;
  pthread_cond_signal((pthread_cond_t *)(iVar1 + 8));
  pthread_mutex_unlock((pthread_mutex_t *)(rawchipCtrl + 0xc));
  return;
}



/* ============================================= */
/* Function: rawchip_init */
/* Address: 0x00011d80 */
/* ============================================= */

undefined4 rawchip_init(int param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  uint uVar2;
  int *piVar3;
  char *pcVar4;
  int iVar5;
  undefined4 uStack_14;
  
  uStack_14 = param_4;
  property_get("persist.camera.usedxoaf",&uStack_14,&DAT_000169ee,param_4,param_1,param_2,param_3);
  iVar1 = atoi((char *)&uStack_14);
  useDxOAF = (char)iVar1;
  get_vcm_clib(param_1,S_DxOAF_perUnit);
  if (useDxOAF != '\0') {
    dxoAf_setup = 0;
    DAT_0001b194 = 0;
  }
  uVar2 = (uint)DAT_00019134;
  if ((int)(uVar2 << 0x18) < 0) {
    uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
  }
  if ((int)(uVar2 << 0x1d) < 0) {
    __android_log_print(4,"CAMERA RAWCHIP_PROC",&DAT_000169f0,"rawchip_init");
  }
  piVar3 = malloc(0x104);
  rawchipCtrl = piVar3;
  if (piVar3 == (int *)0x0) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC","%s: out of memory\n","rawchip_init");
    }
  }
  else {
    memset(piVar3,0,0x104);
    iVar1 = open("/dev/rawchip0",2);
    *piVar3 = iVar1;
    piVar3 = rawchipCtrl;
    if (-1 < *rawchipCtrl) {
      rawchipCtrl[1] = -1;
      piVar3[5] = -1;
      piVar3[6] = -1;
      piVar3[0x37] = 0;
      memset(piVar3 + 0x38,0,0x15);
      piVar3 = rawchipCtrl;
      *(undefined1 *)((int)rawchipCtrl + 0xe3) = 9;
      *(undefined1 *)((int)piVar3 + 0x2d) = 0x1e;
      *(undefined1 *)((int)piVar3 + 0xe1) = 6;
      *(undefined1 *)((int)piVar3 + 0xe2) = 5;
      *(undefined1 *)(piVar3 + 0x39) = 7;
      piVar3[7] = 0;
      piVar3[8] = 0;
      piVar3[9] = 0;
      piVar3[10] = 0;
      *(undefined1 *)(piVar3 + 0xb) = 0;
      piVar3[0xd] = 0;
      piVar3[0xe] = 0;
      piVar3[0xf] = 0;
      piVar3[0x13] = 0;
      piVar3[0x15] = 1;
      *(undefined1 *)(piVar3 + 0x16) = 0;
      *(undefined1 *)((int)piVar3 + 0x5a) = 0;
      *(undefined1 *)(piVar3 + 0x2e) = 0;
      iVar1 = 0;
      do {
        iVar5 = iVar1 + 1;
        *(undefined1 *)((int)piVar3 + iVar1 + 0xb9) = 0;
        iVar1 = iVar5;
      } while (iVar5 != 0xc);
      rawchip_load_af_algo_params(*(undefined4 *)(param_1 + 0x28));
      memcpy(rawchipCtrl + 0x18,(void *)rawchipCtrl[0x11],10);
      return 0;
    }
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1b) < 0) {
      piVar3 = (int *)__errno();
      pcVar4 = strerror(*piVar3);
      __android_log_print(6,"CAMERA RAWCHIP_PROC","%s: open rawchip0 failed: %s!\n","rawchip_init",
                          pcVar4);
    }
  }
  return 0xffffffff;
}



/* ============================================= */
/* Function: rawchip_thread_release */
/* Address: 0x00011f80 */
/* ============================================= */

undefined8 rawchip_thread_release(undefined4 param_1,undefined4 param_2,undefined4 param_3)

{
  int iVar1;
  uint uVar2;
  ssize_t sVar3;
  int iVar4;
  undefined4 local_14;
  undefined4 uStack_10;
  
  local_14 = 1;
  uVar2 = (uint)DAT_00019134;
  uStack_10 = param_3;
  if ((int)(uVar2 << 0x18) < 0) {
    uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
  }
  if ((int)(uVar2 << 0x1d) < 0) {
    __android_log_print(4,"CAMERA RAWCHIP_PROC",&DAT_000169f0,"rawchip_thread_release");
  }
  if (*(int *)(rawchipCtrl + 4) == -1) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC");
    }
    iVar4 = 0;
    if ((int)(uVar2 << 0x1d) < 0) {
      iVar4 = 0;
      __android_log_print(4,"CAMERA RAWCHIP_PROC","Didn\'t launch rawchip thread\n");
    }
  }
  else {
    sVar3 = write(*(int *)(rawchipCtrl + 0x18),&local_14,4);
    if (sVar3 < 0) {
      uVar2 = (uint)DAT_00019134;
      if ((int)(uVar2 << 0x18) < 0) {
        uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
      }
      if ((int)(uVar2 << 0x1b) < 0) {
        __android_log_print(6,"CAMERA RAWCHIP_PROC","rawchipproc termination  : Failed\n");
      }
    }
    iVar4 = pthread_join(*(pthread_t *)(rawchipCtrl + 4),(void **)0x0);
    if (iVar4 != 0) {
      uVar2 = (uint)DAT_00019134;
      if ((int)(uVar2 << 0x18) < 0) {
        uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
      }
      if ((int)(uVar2 << 0x1b) < 0) {
        __android_log_print(6,"CAMERA RAWCHIP_PROC","rawchip_thread exit failure!\n");
      }
    }
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC",
                          "pthread_join succeeded on release_rawchip_thread\n");
    }
    if (-1 < *(int *)(rawchipCtrl + 0x14)) {
      close(*(int *)(rawchipCtrl + 0x14));
      *(undefined4 *)(rawchipCtrl + 0x14) = 0xffffffff;
    }
    if (-1 < *(int *)(rawchipCtrl + 0x18)) {
      close(*(int *)(rawchipCtrl + 0x18));
      *(undefined4 *)(rawchipCtrl + 0x18) = 0xffffffff;
    }
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 4) = 0xffffffff;
    *(undefined4 *)(iVar1 + 0x14) = 0xffffffff;
    *(undefined4 *)(iVar1 + 0x18) = 0xffffffff;
  }
  return CONCAT44(param_1,iVar4);
}



/* ============================================= */
/* Function: rawchip_destroy */
/* Address: 0x0001213c */
/* ============================================= */

int rawchip_destroy(void)

{
  uint uVar1;
  int iVar2;
  undefined4 in_r3;
  
  uVar1 = (uint)DAT_00019134;
  if ((int)(uVar1 << 0x18) < 0) {
    uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019134,uVar1 << 0x18,in_r3);
  }
  if ((int)(uVar1 << 0x1d) < 0) {
    __android_log_print(4,"CAMERA RAWCHIP_PROC",&DAT_000169f0,"rawchip_destroy");
  }
  iVar2 = rawchip_thread_release();
  if (iVar2 < 0) {
    uVar1 = (uint)DAT_00019134;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar1 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC","rawchip_thread_release Failed\n");
    }
  }
  if (-1 < *rawchipCtrl) {
    close(*rawchipCtrl);
    *rawchipCtrl = -1;
  }
  free(rawchipCtrl);
  return iVar2;
}



/* ============================================= */
/* Function: rawchip_get_af_active */
/* Address: 0x000121f4 */
/* ============================================= */

undefined4 rawchip_get_af_active(uint *param_1)

{
  int iVar1;
  uint uVar2;
  
  uVar2 = 0;
  if (*(int *)(rawchipCtrl + 0x1c) != 0) {
    if (((*(int *)(rawchipCtrl + 0x28) == 0) || (iVar1 = *(int *)(rawchipCtrl + 0x54), iVar1 == 3))
       || (iVar1 == 5)) {
      uVar2 = 1;
    }
    else {
      uVar2 = (uint)(iVar1 == 6);
    }
  }
  *param_1 = uVar2;
  return 0;
}



/* ============================================= */
/* Function: rawchip_get_af_roi_region_num */
/* Address: 0x00012224 */
/* ============================================= */

undefined4 rawchip_get_af_roi_region_num(undefined1 *param_1)

{
  *param_1 = 0x10;
  param_1[1] = 0xc;
  return 0;
}



/* ============================================= */
/* Function: rawchip_get_af_default_roi_region */
/* Address: 0x00012230 */
/* ============================================= */

undefined4 rawchip_get_af_default_roi_region(undefined1 *param_1)

{
  *param_1 = 6;
  param_1[1] = 9;
  param_1[2] = 5;
  param_1[3] = 7;
  return 0;
}



/* ============================================= */
/* Function: rawchip_set_sensor_info */
/* Address: 0x00012244 */
/* ============================================= */

undefined4 rawchip_set_sensor_info(uint param_1)

{
  if (*(int *)(rawchipCtrl + 0x48) - 1U < param_1) {
    param_1 = 0;
  }
  memcpy((void *)(rawchipCtrl + 0x60),(void *)(param_1 * 10 + *(int *)(rawchipCtrl + 0x44)),10);
  return 0;
}



/* ============================================= */
/* Function: rawchip_set_af_behavior */
/* Address: 0x00012270 */
/* ============================================= */

undefined4 rawchip_set_af_behavior(int param_1)

{
  if ((param_1 == 1) || (param_1 == 2)) {
    *(int *)(rawchipCtrl + 0x6c) = param_1;
  }
  else {
    *(undefined4 *)(rawchipCtrl + 0x6c) = 0;
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_set_bestshot_mode */
/* Address: 0x000122a8 */
/* ============================================= */

undefined4 rawchip_set_bestshot_mode(uint param_1)

{
  if (0x13 < param_1) {
    return 0xffffffff;
  }
  if (param_1 != 2) {
    if (param_1 != 0xb) {
      return 0;
    }
    param_1 = 1;
  }
  rawchip_set_af_behavior(param_1);
  return 0;
}



/* ============================================= */
/* Function: rawchip_set_af_roi_info */
/* Address: 0x000122cc */
/* ============================================= */

undefined4 rawchip_set_af_roi_info(uint param_1,uint param_2)

{
  int iVar1;
  undefined1 local_20;
  undefined1 local_1f;
  undefined1 local_1e;
  undefined1 local_1d;
  
  iVar1 = rawchipCtrl;
  local_20 = (undefined1)param_1;
  local_1f = (byte)(param_1 >> 8);
  local_1e = (byte)(param_1 >> 0x10);
  local_1d = (byte)(param_1 >> 0x18);
  if (useDxOAF != '\0') {
    DAT_00019181 = 1;
  }
  if (((((((uint)*(byte *)(rawchipCtrl + 0xe1) != (param_1 & 0xff)) ||
         (*(byte *)(rawchipCtrl + 0xe3) != local_1f)) || (*(byte *)(rawchipCtrl + 0xe2) != local_1e)
        ) || ((*(byte *)(rawchipCtrl + 0xe4) != local_1d || (*(int *)(rawchipCtrl + 0x50) - 1U < 2))
             )) && (((param_1 & 0xff) < 0x10 && ((local_1f < 0x10 && (local_1e < 0xc)))))) &&
     ((local_1d < 0xc &&
      ((*(int *)(rawchipCtrl + 0x1c) == 0 || (*(int *)(rawchipCtrl + 0x28) != 0)))))) {
    *(undefined1 *)(rawchipCtrl + 0xc5) = 1;
    *(uint *)(iVar1 + 0x50) = param_2 & 0xff;
    pthread_mutex_lock((pthread_mutex_t *)(iVar1 + 0xf8));
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 0x54) = 1;
    *(undefined1 *)(iVar1 + 0x58) = 0;
    *(undefined1 *)(iVar1 + 0x5a) = 0;
    *(undefined4 *)(iVar1 + 0xdc) = 1;
    *(undefined1 *)(iVar1 + 0xe1) = local_20;
    *(byte *)(iVar1 + 0xe3) = local_1f;
    *(byte *)(iVar1 + 0xe2) = local_1e;
    *(byte *)(iVar1 + 0xe4) = local_1d;
    pthread_mutex_unlock((pthread_mutex_t *)(iVar1 + 0xf8));
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_set_aec_exp_info */
/* Address: 0x00012398 */
/* ============================================= */

undefined4 rawchip_set_aec_exp_info(short param_1,float param_2,float param_3)

{
  int iVar1;
  uint uVar2;
  uint uVar3;
  float fVar4;
  uint uVar5;
  
  uVar2 = (uint)(0.0 < param_2) * (int)param_2 & 0xffff;
  fVar4 = (param_2 - (float)(longlong)(int)uVar2) * 256.0;
  uVar3 = uVar2 * 0x100 + ((uint)(0.0 < fVar4) * (int)fVar4 & 0xff);
  uVar5 = (uint)(0.0 < param_3 * 1000.0) * (int)(param_3 * 1000.0);
  uVar2 = uVar5 & 0xffff;
  if ((((*(short *)(rawchipCtrl + 0xd0) != param_1) ||
       ((uint)*(ushort *)(rawchipCtrl + 0xd2) != (uVar3 & 0xffff))) ||
      (*(ushort *)(rawchipCtrl + 0xd4) != uVar2)) && (uVar2 != 0 || param_1 != 0)) {
    pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0xd8));
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 0xcc) = 1;
    *(short *)(iVar1 + 0xd0) = param_1;
    *(short *)(iVar1 + 0xd2) = (short)uVar3;
    *(short *)(iVar1 + 0xd4) = (short)uVar5;
    pthread_mutex_unlock((pthread_mutex_t *)(iVar1 + 0xd8));
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_set_awb_gain_info */
/* Address: 0x00012450 */
/* ============================================= */

longlong rawchip_set_awb_gain_info(uint param_1)

{
  int iVar1;
  uint uVar2;
  
  uVar2 = (param_1 << 0x10) >> 0x18;
  if ((((uint)*(byte *)(rawchipCtrl + 0xd6) != (param_1 & 0xff)) ||
      (*(byte *)(rawchipCtrl + 0xd7) != uVar2)) && (((param_1 & 0xff) != 0 || (uVar2 != 0)))) {
    pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0xd8));
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 0xcc) = 1;
    *(char *)(iVar1 + 0xd6) = (char)param_1;
    *(char *)(iVar1 + 0xd7) = (char)(param_1 >> 8);
    pthread_mutex_unlock((pthread_mutex_t *)(iVar1 + 0xd8));
  }
  return (ulonglong)param_1 << 0x20;
}



/* ============================================= */
/* Function: rawchip_get_dxoprc_frameSetting */
/* Address: 0x000124a4 */
/* ============================================= */

int rawchip_get_dxoprc_frameSetting(undefined4 param_1)

{
  int iVar1;
  uint uVar2;
  undefined4 local_18;
  undefined4 uStack_14;
  undefined4 uStack_10;
  undefined4 local_c;
  
  local_18 = 8;
  uStack_14 = 2000;
  uStack_10 = 0x14;
  local_c = param_1;
  iVar1 = ioctl(*rawchipCtrl,0x40046708,&local_18);
  if (iVar1 < 0) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC","%s rawchip_get_dxoprc_frameSetting: failed\n",
                          "rawchip_get_dxoprc_frameSetting");
    }
  }
  return iVar1;
}



/* ============================================= */
/* Function: DxOAF_setup */
/* Address: 0x0001251c */
/* ============================================= */

longlong DxOAF_setup(uint param_1,uint param_2,undefined4 param_3)

{
  uint uVar1;
  uint uVar2;
  int iVar3;
  int extraout_r3;
  
  if (dxoAf_setup == 0) {
    uVar1 = (uint)DAT_00019134;
    uVar2 = param_1;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar1 << 0x1d) < 0) {
      uVar2 = (uint)*(ushort *)(param_1 + 6);
      param_2 = (uint)*(ushort *)(param_1 + 8);
      __android_log_print(4,"CAMERA RAWCHIP_PROC","DxOAF_boot %d %d %d\n",
                          *(undefined2 *)(param_1 + 4),uVar2,param_2,param_3);
    }
    param_1 = uVar2;
    uVar2 = (uint)DAT_00019134;
    iVar3 = uVar2 << 0x18;
    if (iVar3 < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
      iVar3 = extraout_r3;
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","dbgDxO : boot succ:\n",iVar3,param_1,param_2,
                          param_3);
    }
    rawchip_get_dxoprc_frameSetting(dxoframeSetting);
    dxoAf_setup = 1;
  }
  return (ulonglong)param_1 << 0x20;
}



/* ============================================= */
/* Function: rawchip_set_af_info_for_af */
/* Address: 0x000125c4 */
/* ============================================= */

undefined4 rawchip_set_af_info_for_af(int param_1)

{
  if (*(int *)(rawchipCtrl + 0x74) != param_1) {
    *(int *)(rawchipCtrl + 0x74) = param_1;
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_set_aec_info_for_af */
/* Address: 0x000125dc */
/* ============================================= */

undefined4 rawchip_set_aec_info_for_af(int param_1,int param_2,int param_3)

{
  int iVar1;
  
  iVar1 = rawchipCtrl;
  if (((*(int *)(rawchipCtrl + 0x78) != param_1) || (*(int *)(rawchipCtrl + 0x7c) != param_2)) ||
     (*(int *)(rawchipCtrl + 0x80) != param_3)) {
    *(int *)(rawchipCtrl + 0x78) = param_1;
    *(int *)(iVar1 + 0x7c) = param_2;
    *(int *)(iVar1 + 0x80) = param_3;
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_update_aec_awb_params */
/* Address: 0x00012614 */
/* ============================================= */

int rawchip_update_aec_awb_params
              (int param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  uint uVar2;
  undefined4 local_2c;
  undefined4 uStack_28;
  undefined4 uStack_24;
  undefined4 local_20;
  undefined4 local_1c;
  undefined4 local_18;
  undefined4 *local_14;
  
  local_14 = &local_2c;
  iVar1 = 0;
  if (param_1 != 0) {
    local_20 = 8;
    local_18 = 0xc;
    local_1c = 2000;
    local_2c = param_2;
    uStack_28 = param_3;
    uStack_24 = param_4;
    iVar1 = ioctl(*rawchipCtrl,0x40046703,&local_20);
    if (iVar1 < 0) {
      uVar2 = (uint)DAT_00019134;
      if ((int)(uVar2 << 0x18) < 0) {
        uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
      }
      if ((int)(uVar2 << 0x1b) < 0) {
        __android_log_print(6,"CAMERA RAWCHIP_PROC","%s RAWCHIP_IOCTL_UPDATE_AEC_AWB failed\n",
                            "rawchip_update_aec_awb_params");
      }
    }
  }
  return iVar1;
}



/* ============================================= */
/* Function: rawchip_update_af_params */
/* Address: 0x00012698 */
/* ============================================= */

int rawchip_update_af_params(int param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  uint uVar2;
  undefined4 local_30;
  undefined4 uStack_2c;
  undefined4 uStack_28;
  undefined4 *local_24;
  undefined4 local_c;
  undefined4 uStack_8;
  undefined4 uStack_4;
  
  local_24 = &local_c;
  iVar1 = 0;
  if (param_1 != 0) {
    local_30 = 8;
    uStack_2c = 2000;
    uStack_28 = 0x1c;
    local_c = param_2;
    uStack_8 = param_3;
    uStack_4 = param_4;
    iVar1 = ioctl(*rawchipCtrl,0x40046704,&local_30);
    if (iVar1 < 0) {
      uVar2 = (uint)DAT_00019134;
      if ((int)(uVar2 << 0x18) < 0) {
        uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
      }
      if ((int)(uVar2 << 0x1b) < 0) {
        __android_log_print(6,"CAMERA RAWCHIP_PROC","%s RAWCHIP_IOCTL_UPDATE_AF failed\n",
                            "rawchip_update_af_params");
      }
    }
  }
  return iVar1;
}



/* ============================================= */
/* Function: rawchip_update_3A_params */
/* Address: 0x00012724 */
/* ============================================= */

undefined4 rawchip_update_3A_params(int param_1)

{
  char cVar1;
  int *piVar2;
  pthread_mutex_t *__mutex;
  undefined4 uVar3;
  uint uVar4;
  uint uVar5;
  int iVar6;
  int iVar7;
  uint uVar8;
  int iVar9;
  uint uVar10;
  undefined4 local_64;
  int local_60;
  int iStack_5c;
  int iStack_58;
  undefined4 local_54;
  undefined4 local_50;
  undefined4 local_4c;
  undefined4 *local_48;
  int local_44;
  int local_40;
  int iStack_3c;
  int iStack_38;
  int local_34;
  int iStack_30;
  int iStack_2c;
  
  local_64 = 0;
  pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0x36));
  iVar9 = rawchipCtrl[0x33];
  local_60 = rawchipCtrl[0x33];
  iStack_5c = rawchipCtrl[0x34];
  iStack_58 = rawchipCtrl[0x35];
  pthread_mutex_unlock((pthread_mutex_t *)(rawchipCtrl + 0x36));
  pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0x3e));
  cVar1 = (char)rawchipCtrl[0x38];
  iVar7 = rawchipCtrl[0x37];
  local_44 = rawchipCtrl[0x37];
  local_40 = rawchipCtrl[0x38];
  iStack_3c = rawchipCtrl[0x39];
  iStack_38 = rawchipCtrl[0x3a];
  local_34 = rawchipCtrl[0x3b];
  iStack_30 = rawchipCtrl[0x3c];
  iStack_2c = rawchipCtrl[0x3d];
  pthread_mutex_unlock((pthread_mutex_t *)(rawchipCtrl + 0x3e));
  uVar3 = 0;
  if (iVar7 != 0 || iVar9 != 0) {
    pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0x40));
    __mutex = (pthread_mutex_t *)(rawchipCtrl + 0x40);
    if ((*(byte *)(rawchipCtrl + 0x3f) & 0x1c) == 0x1c) {
      *(undefined1 *)(rawchipCtrl + 0x3f) = 0;
      pthread_mutex_unlock(__mutex);
      uVar4 = rawchip_update_aec_awb_params(iVar9,local_60,iStack_5c,iStack_58);
      local_40 = CONCAT31(local_40._1_3_,cVar1);
      uVar5 = rawchip_update_af_params
                        (iVar7,local_44,local_40,iStack_3c,iStack_38,local_34,iStack_30,iStack_2c);
      uVar10 = 1 - uVar5;
      if (1 < uVar5) {
        uVar10 = 0;
      }
      if (iVar7 != 0) {
        local_64 = 1;
        if (cVar1 == '\0') {
          local_64 = 2;
        }
        *(undefined4 *)(param_1 + 8) = local_64;
      }
      local_54 = 8;
      local_4c = 4;
      local_48 = &local_64;
      local_50 = 2000;
      uVar5 = 1;
      iVar6 = ioctl(*rawchipCtrl,0x40046705,&local_54);
      if (iVar6 < 0) {
        uVar8 = (uint)DAT_00019134;
        if ((int)(uVar8 << 0x18) < 0) {
          uVar8 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
        }
        uVar5 = 0;
        if ((int)(uVar8 << 0x1b) < 0) {
          uVar5 = 0;
          __android_log_print(6,"CAMERA RAWCHIP_PROC","%s RAWCHIP_IOCTL_UPDATE_3A failed\n",
                              "rawchip_update_3A_params");
        }
      }
      uVar8 = 1 - uVar4;
      if (1 < uVar4) {
        uVar8 = 0;
      }
      if (iVar9 == 0) {
        uVar4 = 0;
      }
      else {
        uVar4 = uVar8 & 1;
      }
      if ((uVar4 != 0) && (uVar5 != 0)) {
        pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0x36));
        piVar2 = rawchipCtrl;
        rawchipCtrl[0x33] = 0;
        pthread_mutex_unlock((pthread_mutex_t *)(piVar2 + 0x36));
      }
      if (iVar7 != 0) {
        pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0x3e));
        piVar2 = rawchipCtrl;
        rawchipCtrl[0x37] = 0;
        pthread_mutex_unlock((pthread_mutex_t *)(piVar2 + 0x3e));
        piVar2 = rawchipCtrl;
        rawchipCtrl[8] = 0;
        if (cVar1 == '\0') {
          if (piVar2[0xf] != 0) {
            uVar4 = (uint)DAT_00019134;
            if ((int)(uVar4 << 0x18) < 0) {
              uVar4 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
            }
            if ((int)(uVar4 << 0x1d) < 0) {
              __android_log_print(4,"CAMERA RAWCHIP_PROC","Send AF Callback\n");
            }
            if (*(code **)(param_1 + 0xc) != (code *)0x0) {
              (**(code **)(param_1 + 0xc))(*(undefined4 *)(param_1 + 4),rawchipCtrl[0x10]);
            }
            rawchipCtrl[0xf] = 0;
          }
        }
        else {
          piVar2[9] = 0;
          if ((uVar5 & uVar10) == 0) {
            uVar4 = (uint)DAT_00019134;
            if ((int)(uVar4 << 0x18) < 0) {
              uVar4 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
            }
            if ((int)(uVar4 << 0x1b) < 0) {
              __android_log_print(6,"CAMERA RAWCHIP_PROC",
                                  "Start Rawchip AF: update active number Failed\n");
            }
            if (*(code **)(param_1 + 0xc) != (code *)0x0) {
              (**(code **)(param_1 + 0xc))(*(undefined4 *)(param_1 + 4),0);
            }
          }
          else {
            uVar4 = (uint)DAT_00019134;
            if ((int)(uVar4 << 0x18) < 0) {
              uVar4 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
            }
            if ((int)(uVar4 << 0x1d) < 0) {
              __android_log_print(4,"CAMERA RAWCHIP_PROC",
                                  "Start Rawchip AF: update active number Succeed\n");
            }
            piVar2 = rawchipCtrl;
            rawchipCtrl[8] = 1;
            piVar2[9] = 1;
            *(undefined1 *)(piVar2 + 0xb) = 0;
            piVar2[0xc] = 0;
          }
        }
      }
      if ((uVar8 & uVar10) == 0) {
        uVar3 = 0xffffffff;
      }
      else if (uVar5 == 0) {
        uVar3 = 0xffffffff;
      }
      else {
        uVar3 = 0;
      }
    }
    else {
      pthread_mutex_unlock(__mutex);
      uVar3 = 0;
    }
  }
  return uVar3;
}



/* ============================================= */
/* Function: rawchip_af_start */
/* Address: 0x00012a08 */
/* ============================================= */

undefined4 rawchip_af_start(undefined4 param_1)

{
  int iVar1;
  uint uVar2;
  
  if (*(int *)(rawchipCtrl + 0x1c) == 0) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","Start Rawchip AF\n");
    }
    pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0xf8));
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 0xdc) = 1;
    *(undefined1 *)(iVar1 + 0xe0) = 1;
    pthread_mutex_unlock((pthread_mutex_t *)(iVar1 + 0xf8));
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 0x1c) = 1;
    *(undefined4 *)(iVar1 + 0x28) = param_1;
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_thread_launch */
/* Address: 0x00012a80 */
/* ============================================= */

int rawchip_thread_launch(void *param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  uint uVar2;
  int iVar3;
  
  uVar2 = (uint)DAT_00019134;
  if ((int)(uVar2 << 0x18) < 0) {
    uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019134,param_4,param_4);
  }
  if ((int)(uVar2 << 0x1d) < 0) {
    __android_log_print(4,"CAMERA RAWCHIP_PROC",&DAT_000169f0,"rawchip_thread_launch");
  }
  if ((useDxOAF != '\0') && (iVar3 = DxOAF_setup(S_DxOAF_perUnit), iVar3 != 0)) {
    useDxOAF = '\0';
  }
  iVar3 = rawchipCtrl;
  *(undefined2 *)(rawchipCtrl + 0xd0) = 0x20;
  *(undefined2 *)(iVar3 + 0xd4) = 0x20;
  *(undefined1 *)(iVar3 + 0xd6) = 0x40;
  *(undefined1 *)(iVar3 + 0xd7) = 0x40;
  *(undefined1 *)(iVar3 + 0xfc) = 0x1c;
  *(undefined4 *)(iVar3 + 0xcc) = 0;
  *(undefined4 *)(iVar3 + 0x10) = 0;
  iVar3 = pthread_create((pthread_t *)(iVar3 + 4),(pthread_attr_t *)0x0,(__start_routine *)0x136a1,
                         param_1);
  if (iVar3 == 0) {
    pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0xc));
    while( true ) {
      if (*(int *)(rawchipCtrl + 0x10) != 0) break;
      iVar3 = pthread_cond_wait((pthread_cond_t *)(rawchipCtrl + 8),
                                (pthread_mutex_t *)(rawchipCtrl + 0xc));
    }
    pthread_mutex_unlock((pthread_mutex_t *)(rawchipCtrl + 0xc));
  }
  if ((*(int *)(rawchipCtrl + 0x38) != 0) && (*(int *)(rawchipCtrl + 0x4c) - 1U < 2)) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","Restart CAF when restart Preview\n");
    }
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 0x38) = 0;
    *(undefined4 *)(iVar1 + 0x54) = 1;
    *(undefined1 *)(iVar1 + 0x58) = 0;
    *(undefined1 *)(iVar1 + 0x5a) = 0;
    rawchip_af_start();
  }
  return iVar3;
}



/* ============================================= */
/* Function: rawchip_af_stop */
/* Address: 0x00012bbc */
/* ============================================= */

undefined4 rawchip_af_stop(undefined4 param_1)

{
  int iVar1;
  uint uVar2;
  
  if (*(int *)(rawchipCtrl + 0x1c) != 0) {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","Stop Rawchip AF\n");
    }
    pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0xf8));
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 0xdc) = 1;
    *(undefined1 *)(iVar1 + 0xe0) = 0;
    pthread_mutex_unlock((pthread_mutex_t *)(iVar1 + 0xf8));
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 0x1c) = 0;
    *(undefined4 *)(iVar1 + 0x28) = param_1;
    *(undefined4 *)(iVar1 + 0x24) = 0;
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_set_af_mode */
/* Address: 0x00012c38 */
/* ============================================= */

undefined4 rawchip_set_af_mode(int param_1)

{
  bool bVar1;
  int iVar2;
  uint uVar3;
  int iVar4;
  undefined1 uVar5;
  
  iVar2 = rawchipCtrl;
  iVar4 = *(int *)(rawchipCtrl + 0x4c);
  if (iVar4 != param_1) {
    if ((*(int *)(rawchipCtrl + 0x1c) == 0) || (*(int *)(rawchipCtrl + 0x28) != 0)) {
      *(int *)(rawchipCtrl + 0x4c) = param_1;
      bVar1 = 1 < iVar4 - 1U;
      if (param_1 == 2) {
        uVar5 = 0x2d;
        skip1stAF_inVideoRec = 1;
        g_mode_user = 2;
      }
      else {
        uVar5 = 0x1e;
        g_mode_user = 1;
      }
      *(undefined1 *)(iVar2 + 0x2d) = uVar5;
      iVar2 = rawchipCtrl;
      g_mode = g_mode_user;
      if (param_1 - 1U < 2) {
        *(undefined4 *)(rawchipCtrl + 0x54) = 1;
        *(undefined1 *)(iVar2 + 0x58) = 0;
        *(undefined1 *)(iVar2 + 0x5a) = 0;
        if (bVar1) {
          uVar3 = (uint)DAT_00019134;
          if ((int)(uVar3 << 0x18) < 0) {
            uVar3 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
          }
          if ((int)(uVar3 << 0x1d) < 0) {
            __android_log_print(4,"CAMERA RAWCHIP_PROC","Start CAF\n");
          }
          rawchip_af_start(1);
        }
      }
      else if (!bVar1) {
        uVar3 = (uint)DAT_00019134;
        if ((int)(uVar3 << 0x18) < 0) {
          uVar3 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
        }
        if ((int)(uVar3 << 0x1d) < 0) {
          __android_log_print(4,"CAMERA RAWCHIP_PROC","Stop CAF\n");
        }
        rawchip_af_stop(0);
      }
    }
    else {
      uVar3 = (uint)DAT_00019134;
      if ((int)(uVar3 << 0x18) < 0) {
        uVar3 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
      }
      if ((int)(uVar3 << 0x1d) < 0) {
        __android_log_print(4,"CAMERA RAWCHIP_PROC",
                            "Calling change AF mode when Once AF running, Do not change af_mode\n");
      }
    }
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_once_af_start */
/* Address: 0x00012d94 */
/* ============================================= */

undefined4 rawchip_once_af_start(void)

{
  int iVar1;
  uint uVar2;
  undefined4 in_r3;
  
  uVar2 = (uint)DAT_00019134;
  if ((int)(uVar2 << 0x18) < 0) {
    uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019134,uVar2 << 0x18,in_r3);
  }
  if ((int)(uVar2 << 0x1e) < 0) {
    __android_log_print(3,"CAMERA RAWCHIP_PROC","Start Once AF\n");
  }
  iVar1 = rawchipCtrl;
  if (useDxOAF != '\0') {
    AF_forSnapshot = 1;
  }
  if ((*(int *)(rawchipCtrl + 0x1c) == 0) || (*(int *)(rawchipCtrl + 0x28) == 0)) {
    *(undefined1 *)(rawchipCtrl + 0x58) = 0;
    *(undefined4 *)(iVar1 + 0x54) = 1;
    *(undefined1 *)(iVar1 + 0x5a) = 0;
    rawchip_af_start();
  }
  else {
    uVar2 = (uint)DAT_00019134;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019134,rawchipCtrl,in_r3);
    }
    if ((int)(uVar2 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC",
                          "Calling Once AF start when CAF running, Stop CAF and do Once AF\n");
    }
    iVar1 = rawchipCtrl;
    *(undefined4 *)(rawchipCtrl + 0x28) = 0;
    *(undefined1 *)(iVar1 + 0x2c) = 0;
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_once_af_stop */
/* Address: 0x00012e64 */
/* ============================================= */

undefined4 rawchip_once_af_stop(undefined4 param_1,undefined4 param_2)

{
  int iVar1;
  uint uVar2;
  
  uVar2 = (uint)DAT_00019134;
  if ((int)(uVar2 << 0x18) < 0) {
    uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
  }
  if ((int)(uVar2 << 0x1d) < 0) {
    __android_log_print(4,"CAMERA RAWCHIP_PROC","Stop Once AF, af_status=%d",param_2);
  }
  rawchip_af_stop(0);
  iVar1 = rawchipCtrl;
  if (*(int *)(rawchipCtrl + 0x34) == 0) {
    *(undefined4 *)(rawchipCtrl + 0x40) = param_2;
    *(undefined4 *)(iVar1 + 0x3c) = 1;
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_cancel_af */
/* Address: 0x00012ec4 */
/* ============================================= */

undefined4 rawchip_cancel_af(undefined4 param_1)

{
  uint uVar1;
  
  uVar1 = (uint)DAT_00019134;
  if ((int)(uVar1 << 0x18) < 0) {
    uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
  }
  if ((int)(uVar1 << 0x1d) < 0) {
    __android_log_print(4,"CAMERA RAWCHIP_PROC","Cancel AF\n");
  }
  if ((*(int *)(rawchipCtrl + 0x1c) != 0) && (*(int *)(rawchipCtrl + 0x28) == 0)) {
    uVar1 = (uint)DAT_00019134;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar1 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC","Once AF is running, stop Once AF\n");
    }
    rawchip_once_af_stop(param_1,0);
  }
  if (*(int *)(rawchipCtrl + 0x4c) - 1U < 2) {
    uVar1 = (uint)DAT_00019134;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar1 << 0x1d) < 0) {
      __android_log_print(4,"CAMERA RAWCHIP_PROC",
                          "Calling Cancel AF when focus mode is CAF, restart CAF\n");
    }
    g_mode = g_mode_user;
    *(undefined4 *)(rawchipCtrl + 0x54) = 0;
    rawchip_af_start(1);
  }
  return 0;
}



/* ============================================= */
/* Function: DxOAF_algorithm */
/* Address: 0x00012fd4 */
/* ============================================= */

void DxOAF_algorithm(void)

{
  undefined4 uVar1;
  undefined4 *puVar2;
  
  puVar2 = &DAT_0001b194;
  if (DAT_0001b194 == 0) {
    uVar1 = 1;
  }
  else {
    if (AF_forSnapshot == '\0') {
      if (DAT_00019181 != '\0') {
        DAT_00019181 = AF_forSnapshot;
      }
      return;
    }
    uVar1 = 0;
    AF_forSnapshot = '\0';
    AF_forSnapshot_running = 1;
    puVar2 = &g_mode;
  }
  *puVar2 = uVar1;
  return;
}



/* ============================================= */
/* Function: move_lens_delay_done */
/* Address: 0x0001301c */
/* ============================================= */

void move_lens_delay_done(int param_1)

{
  if (param_1 != 2) {
    return;
  }
  pthread_mutex_lock((pthread_mutex_t *)&DAT_00019184);
  pthread_cond_signal((pthread_cond_t *)&DAT_0001b18c);
  pthread_mutex_unlock((pthread_mutex_t *)&DAT_00019184);
  return;
}



/* ============================================= */
/* Function: rawchip_set_gsensor_info_for_af */
/* Address: 0x00013048 */
/* ============================================= */

undefined4 rawchip_set_gsensor_info_for_af(int param_1)

{
  if (*(int *)(rawchipCtrl + 0x84) != param_1) {
    move_lens_delay_done(param_1);
    *(int *)(rawchipCtrl + 0x84) = param_1;
  }
  return 0;
}



/* ============================================= */
/* Function: move_lens_delay_wait */
/* Address: 0x00013070 */
/* ============================================= */

int move_lens_delay_wait(int param_1)

{
  int iVar1;
  timeval local_20;
  timespec local_18;
  
  gettimeofday(&local_20,(__timezone_ptr_t)0x0);
  local_18.tv_nsec = local_20.tv_usec * 1000 + param_1 * 1000000;
  local_18.tv_sec = local_20.tv_sec;
  pthread_mutex_lock((pthread_mutex_t *)&DAT_00019184);
  iVar1 = pthread_cond_timedwait
                    ((pthread_cond_t *)&DAT_0001b18c,(pthread_mutex_t *)&DAT_00019184,&local_18);
  pthread_mutex_unlock((pthread_mutex_t *)&DAT_00019184);
  return iVar1;
}



/* ============================================= */
/* Function: rawchip_move_lens */
/* Address: 0x000130c4 */
/* ============================================= */

int rawchip_move_lens(int param_1)

{
  uint uVar1;
  int iVar2;
  
  if (*(code **)(param_1 + 0x1c) == (code *)0x0) {
    uVar1 = (uint)DAT_00019134;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar1 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC","rawchip_ext_af_move_lens is NULL\n");
    }
    iVar2 = -1;
  }
  else {
    iVar2 = (**(code **)(param_1 + 0x1c))(*(undefined4 *)(param_1 + 0x18));
    if (iVar2 < 0) {
      uVar1 = (uint)DAT_00019134;
      if ((int)(uVar1 << 0x18) < 0) {
        uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
      }
      if ((int)(uVar1 << 0x1b) < 0) {
        __android_log_print(6,"CAMERA RAWCHIP_PROC","rawchip_ext_af_move_lens failed\n");
      }
    }
  }
  return iVar2;
}



/* ============================================= */
/* Function: rawchip_default_focus */
/* Address: 0x00013154 */
/* ============================================= */

int rawchip_default_focus(int param_1)

{
  uint uVar1;
  int iVar2;
  
  if (*(code **)(param_1 + 0x20) == (code *)0x0) {
    uVar1 = (uint)DAT_00019134;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar1 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC","rawchip_ext_af_def_focus is NULL\n");
    }
    iVar2 = -1;
  }
  else {
    iVar2 = (**(code **)(param_1 + 0x20))(*(undefined4 *)(param_1 + 0x18));
    if (iVar2 < 0) {
      uVar1 = (uint)DAT_00019134;
      if ((int)(uVar1 << 0x18) < 0) {
        uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
      }
      if ((int)(uVar1 << 0x1b) < 0) {
        __android_log_print(6,"CAMERA RAWCHIP_PROC","rawchip_ext_af_def_focus failed\n");
      }
    }
  }
  return iVar2;
}



/* ============================================= */
/* Function: rawchip_get_curr_step_pos */
/* Address: 0x000131e4 */
/* ============================================= */

int rawchip_get_curr_step_pos(int param_1)

{
  uint uVar1;
  int iVar2;
  
  if (*(code **)(param_1 + 0x24) == (code *)0x0) {
    uVar1 = (uint)DAT_00019134;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar1 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC","rawchip_ext_af_get_curr_step_pos is NULL\n");
    }
    iVar2 = -1;
  }
  else {
    iVar2 = (**(code **)(param_1 + 0x24))(*(undefined4 *)(param_1 + 0x18));
    if (iVar2 < 0) {
      uVar1 = (uint)DAT_00019134;
      if ((int)(uVar1 << 0x18) < 0) {
        uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
      }
      if ((int)(uVar1 << 0x1b) < 0) {
        __android_log_print(6,"CAMERA RAWCHIP_PROC","rawchip_ext_af_get_curr_step_pos failed\n");
      }
    }
  }
  return iVar2;
}



/* ============================================= */
/* Function: rawchip_af_parse_stats */
/* Address: 0x00013274 */
/* ============================================= */

void rawchip_af_parse_stats(undefined4 param_1)

{
  bool bVar1;
  int *piVar2;
  short sVar3;
  char cVar4;
  int iVar5;
  uint uVar6;
  int iVar7;
  undefined4 uVar8;
  char *pcVar9;
  int *piVar10;
  char local_4db;
  ushort local_4da;
  undefined1 auStack_4d8 [4];
  undefined4 local_4d4;
  undefined4 local_4d0;
  undefined1 *local_4cc;
  undefined1 auStack_4c8 [44];
  undefined1 auStack_49c [80];
  undefined2 local_44c;
  undefined1 auStack_448 [548];
  undefined1 auStack_224 [512];
  int local_24;
  
  local_4d0 = 0x200;
  local_24 = __stack_chk_guard;
  local_4cc = auStack_224;
  local_4d4 = 0xffffffff;
  iVar5 = ioctl(*rawchipCtrl,0x80046702,auStack_4d8);
  if (iVar5 < 0) {
    uVar6 = (uint)DAT_00019134;
    if ((int)(uVar6 << 0x18) < 0) {
      uVar6 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar6 << 0x1b) < 0) {
      pcVar9 = "RAWCHIP_IOCTL_GET_AF_STATUS failed\n";
LAB_0001331c:
      __android_log_print(6,"CAMERA RAWCHIP_PROC",pcVar9);
    }
  }
  else {
    memcpy(auStack_49c,local_4cc,0x54);
    piVar2 = rawchipCtrl;
    rawchipCtrl[0x17] = (int)auStack_49c;
    iVar5 = rawchip_get_curr_step_pos(param_1,piVar2 + 0x1c);
    cVar4 = useDxOAF;
    if (-1 < iVar5) {
      if (rawchipCtrl[0xd] == 0) {
        if (useDxOAF == '\0') {
          if (DAT_000190b6 != '\0') {
            FUN_00011900(0x10);
            DAT_000190b6 = cVar4;
          }
          iVar5 = afsu_algorithm(rawchipCtrl + 0x13,rawchipCtrl + 0x32);
          if ('\0' < *(char *)((int)rawchipCtrl + 0xc9)) {
            if ((((rawchipCtrl[0x13] == 2) && ((char)rawchipCtrl[0x32] == '\x01')) &&
                ((short)rawchipCtrl[0x1c] < 0x33)) &&
               (((iVar7 = get_current_product(), iVar7 == 0xb ||
                 (iVar7 = get_current_product(), iVar7 == 0x13)) ||
                (iVar7 = get_current_product(), iVar7 == 0x18)))) {
              do {
                if (*(char *)((int)rawchipCtrl + 0xc9) < '\x01') goto LAB_000135da;
                rawchip_move_lens(param_1,1,1);
                piVar10 = rawchipCtrl;
                piVar2 = rawchipCtrl + 0x21;
                *(char *)((int)rawchipCtrl + 0xc9) = *(char *)((int)rawchipCtrl + 0xc9) + -1;
              } while (((*piVar2 != 2) &&
                       (iVar7 = move_lens_delay_wait(0xf), piVar10 = rawchipCtrl, iVar7 == 0x6e)) &&
                      (rawchipCtrl[0x21] != 2));
            }
            else {
              if (((rawchipCtrl[0x13] != 2) || ((char)rawchipCtrl[0x32] != '\0')) ||
                 (((short)rawchipCtrl[0x1c] < 0x50 ||
                  (((iVar7 = get_current_product(), iVar7 != 0xb &&
                    (iVar7 = get_current_product(), iVar7 != 0x13)) &&
                   (iVar7 = get_current_product(), iVar7 != 0x18)))))) {
                rawchip_move_lens(param_1,(int)(char)rawchipCtrl[0x32],
                                  (int)*(char *)((int)rawchipCtrl + 0xc9));
                goto LAB_000135da;
              }
              do {
                if (*(char *)((int)rawchipCtrl + 0xc9) < '\x01') goto LAB_000135da;
                rawchip_move_lens(param_1,0,1);
                piVar10 = rawchipCtrl;
                piVar2 = rawchipCtrl + 0x21;
                *(char *)((int)rawchipCtrl + 0xc9) = *(char *)((int)rawchipCtrl + 0xc9) + -1;
              } while (((*piVar2 != 2) &&
                       (iVar7 = move_lens_delay_wait(5), piVar10 = rawchipCtrl, iVar7 == 0x6e)) &&
                      (rawchipCtrl[0x21] != 2));
            }
            piVar10[0x15] = 1;
          }
        }
        else {
          if (DAT_0001903c != '\0') {
            FUN_00011900(0x11);
            DAT_0001903c = '\0';
          }
          rawchip_af_params_setting
                    (auStack_4c8,auStack_448,(int)(short)rawchipCtrl[0x1c],rawchipCtrl[0x17],
                     local_44c);
          DxOAF_algorithm(rawchipCtrl[0x1b],auStack_4c8,auStack_448,&DAT_00019188,&local_4da,
                          &local_4db);
          if (local_4db != -1) {
            FUN_00011900();
          }
          sVar3 = (short)rawchipCtrl[0x1c];
          if ((int)sVar3 < (int)(uint)local_4da) {
            cVar4 = (char)local_4da - (char)sVar3;
            uVar8 = 0;
LAB_00013526:
            rawchip_move_lens(param_1,uVar8,(int)cVar4);
          }
          else if ((int)(uint)local_4da < (int)sVar3) {
            cVar4 = (char)sVar3 - (char)local_4da;
            uVar8 = 1;
            goto LAB_00013526;
          }
          if (AF_forSnapshot_running == '\0') {
LAB_000135d4:
            iVar5 = 1;
          }
          else {
            if (DAT_0001b198 == local_4da) {
              DAT_0001b19a = DAT_0001b19a + '\x01';
              if (DAT_0001b19a == '\x05') {
                AF_forSnapshot_running = '\0';
                DAT_0001b19a = '\0';
                goto LAB_000135d4;
              }
            }
            else {
              DAT_0001b198 = local_4da;
              DAT_0001b19a = '\0';
            }
            iVar5 = 2;
          }
        }
LAB_000135da:
        if (rawchipCtrl[10] == 0) {
          if (iVar5 == 1) {
            if ((char)rawchipCtrl[0xb] != '\0') {
              usleep(100000);
            }
            uVar8 = 1;
LAB_00013654:
            rawchip_once_af_stop(param_1,uVar8);
          }
          else if ((iVar5 == 0) || (*(char *)((int)rawchipCtrl + 0x2d) < (char)rawchipCtrl[0xb])) {
            rawchip_get_curr_step_pos(param_1,rawchipCtrl + 0x1c);
            iVar5 = (int)(short)rawchipCtrl[0x1c];
            if (rawchipCtrl[0x1b] == 1) {
              bVar1 = iVar5 < 0x71;
              iVar7 = -0x70;
            }
            else {
              bVar1 = iVar5 < 0x19;
              iVar7 = -0x18;
            }
            iVar5 = iVar5 + iVar7;
            if (iVar5 < 0) {
              iVar5 = -iVar5;
            }
            rawchip_move_lens(param_1,!bVar1,(int)(char)iVar5);
            usleep(100000);
            uVar8 = 0;
            goto LAB_00013654;
          }
          *(char *)(rawchipCtrl + 0xb) = (char)rawchipCtrl[0xb] + '\x01';
        }
      }
      else {
        if (0x45 < (short)rawchipCtrl[0x1c]) {
          rawchipCtrl[0xd] = 0;
          rawchip_once_af_stop(param_1,1);
          uVar8 = 0;
          goto LAB_00013672;
        }
        rawchip_move_lens(param_1,0,1);
        usleep(100000);
      }
      uVar8 = 0;
      goto LAB_00013672;
    }
    uVar6 = (uint)DAT_00019134;
    if ((int)(uVar6 << 0x18) < 0) {
      uVar6 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar6 << 0x1b) < 0) {
      pcVar9 = "rawchip_get_curr_step_pos failed\n";
      goto LAB_0001331c;
    }
  }
  uVar8 = 0xffffffff;
LAB_00013672:
  if (local_24 == __stack_chk_guard) {
    return;
  }
                    /* WARNING: Subroutine does not return */
  __stack_chk_fail(uVar8);
}



/* ============================================= */
/* Function: rawchip_thread */
/* Address: 0x000136a0 */
/* ============================================= */

void rawchip_thread(undefined4 param_1)

{
  byte bVar1;
  int *piVar2;
  int iVar3;
  uint uVar4;
  __useconds_t _Var5;
  ssize_t sVar6;
  pthread_mutex_t *__mutex;
  ushort uVar7;
  int iVar8;
  uint local_250;
  pollfd local_24c;
  int local_244;
  undefined2 local_240;
  ushort local_23e;
  undefined1 auStack_23c [4];
  undefined4 local_238;
  undefined4 local_234;
  byte *local_230;
  byte local_22c [512];
  int local_2c;
  
  iVar8 = 0;
  local_250 = 0;
  local_2c = __stack_chk_guard;
  iVar3 = pipe(rawchipCtrl + 5);
  if (iVar3 < 0) {
    uVar4 = (uint)DAT_00019134;
    if ((int)(uVar4 << 0x18) < 0) {
      uVar4 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar4 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC",
                          "%s: thread termination pipe creation for rawchip_proc_terminate_fd failed\n"
                          ,"rawchip_thread");
    }
  }
  else {
    rawchip_thread_ready_signal();
LAB_00013712:
    do {
      local_24c.fd = *rawchipCtrl;
      local_244 = rawchipCtrl[5];
      local_24c.events = 0x41;
      local_240 = 1;
      iVar3 = poll(&local_24c,2,6000);
      if ((iVar3 == 0) || (iVar3 < 0)) {
        _Var5 = 20000;
        if (useDxOAF == '\0') {
LAB_000139b2:
          _Var5 = 100000;
        }
        usleep(_Var5);
      }
      else {
        uVar7 = local_23e & 1;
        if ((local_23e & 1) == 0) {
LAB_000137fc:
          if ((local_24c.revents & 0x41U) == 0x41) {
            local_230 = local_22c;
            local_234 = 0x200;
            local_238 = 0xffffffff;
            iVar3 = ioctl(*rawchipCtrl,0x80046701,auStack_23c);
            if (-1 < iVar3) {
              bVar1 = *local_230;
              if ((bVar1 & 1) != 0) {
                uVar4 = (uint)DAT_00019134;
                if ((int)(uVar4 << 0x18) < 0) {
                  uVar4 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
                }
                if ((int)(uVar4 << 0x1b) < 0) {
                  __android_log_print(6,"CAMERA RAWCHIP_PROC","Rawchip Get Error Interrupt\n");
                }
              }
              if ((bVar1 & 0x1c) != 0) {
                pthread_mutex_lock((pthread_mutex_t *)(rawchipCtrl + 0x40));
                __mutex = (pthread_mutex_t *)(rawchipCtrl + 0x40);
                *(byte *)(rawchipCtrl + 0x3f) = bVar1 & 0x1c | *(byte *)(rawchipCtrl + 0x3f);
                pthread_mutex_unlock(__mutex);
              }
              if ((bVar1 & 2) != 0) {
                if (useDxOAF != '\0') {
                  DAT_0001b19c = DAT_0001b19c + 1;
                }
                if (iVar8 == 2) {
                  uVar4 = (uint)DAT_00019134;
                  if ((int)(uVar4 << 0x18) < 0) {
                    uVar4 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
                  }
                  if ((int)(uVar4 << 0x1d) < 0) {
                    __android_log_print(4,"CAMERA RAWCHIP_PROC",
                                        "Update 3A to stop AF when release rawchip thread\n");
                  }
                  piVar2 = rawchipCtrl;
                  if ((rawchipCtrl[10] == 0) && (rawchipCtrl[0xd] == 0)) {
                    rawchipCtrl[0x10] = 0;
                    piVar2[0xf] = 1;
                  }
                  iVar8 = 0;
                  rawchip_update_3A_params(param_1);
                }
                else if (iVar8 == 1) {
                  uVar4 = (uint)DAT_00019134;
                  if ((int)(uVar4 << 0x18) < 0) {
                    uVar4 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
                  }
                  if ((int)(uVar4 << 0x1d) < 0) {
                    __android_log_print(4,"CAMERA RAWCHIP_PROC",
                                        "Stop AF when stop preview - skip 1st new frame ack\n");
                  }
                  iVar8 = 2;
                }
                else if (rawchipCtrl[9] != 0) {
                  if ((((useDxOAF != '\0') && (0 < rawchipCtrl[0xc])) || (2 < rawchipCtrl[0xc])) &&
                     (rawchip_af_parse_stats(param_1), rawchipCtrl[10] != 0)) {
                    _Var5 = 1000;
                    if (useDxOAF == '\0') {
                      if (rawchipCtrl[0x15] - 5U < 2) {
                        if (rawchipCtrl[0x1e] < 0x11d) {
                          _Var5 = 10000;
                        }
                        else {
LAB_0001399a:
                          _Var5 = 60000;
                        }
                      }
                      else {
                        if ((rawchipCtrl[0x15] != 3) || (0x11c < rawchipCtrl[0x1e]))
                        goto LAB_0001399a;
                        _Var5 = 30000;
                      }
                    }
                    usleep(_Var5);
                  }
                  rawchipCtrl[0xc] = rawchipCtrl[0xc] + 1;
                }
              }
            }
          }
          else if (uVar7 == 0) goto LAB_000139b2;
        }
        else {
          sVar6 = read(rawchipCtrl[5],&local_250,4);
          if (-1 < sVar6) {
            if (((local_250 == 0) || (rawchipCtrl[7] == 0)) || (rawchipCtrl[8] != 1)) {
              uVar7 = 1;
            }
            else {
              uVar4 = (uint)DAT_00019134;
              if ((int)(uVar4 << 0x18) < 0) {
                uVar4 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
              }
              if ((int)(uVar4 << 0x1d) < 0) {
                __android_log_print(4,"CAMERA RAWCHIP_PROC","Stop AF when stop preview\n");
              }
              rawchip_af_stop(0);
              uVar7 = 1;
              iVar8 = 1;
              if (rawchipCtrl[0x13] - 1U < 2) {
                rawchipCtrl[0xe] = 1;
              }
            }
            goto LAB_000137fc;
          }
          uVar4 = (uint)DAT_00019134;
          if ((int)(uVar4 << 0x18) < 0) {
            uVar4 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
          }
          if ((int)(uVar4 << 0x1b) < 0) {
            __android_log_print(6,"CAMERA RAWCHIP_PROC",
                                "%s: Cannot read from rawchip_proc_exit thread\n","rawchip_thread");
          }
          local_250 = 0;
        }
      }
      uVar4 = 1 - local_250;
      if (1 < local_250) {
        uVar4 = 0;
      }
      if (iVar8 != 0) {
        uVar4 = uVar4 | 1;
      }
    } while (uVar4 != 0);
    if ((*(byte *)(rawchipCtrl + 0x3f) & 0x1c) != 0x1c) {
      iVar8 = 0;
      goto LAB_00013712;
    }
  }
  if (local_2c != __stack_chk_guard) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail(0);
  }
  return;
}



/* ============================================= */
/* Function: rawchip_through_focus */
/* Address: 0x00013aa8 */
/* ============================================= */

undefined4 rawchip_through_focus(undefined4 param_1,undefined4 param_2)

{
  *(undefined4 *)(rawchipCtrl + 0x34) = 1;
  rawchip_set_af_roi_info(0x7050906,param_2);
  rawchip_once_af_start(param_1);
  return 0;
}



/* ============================================= */
/* Function: debug_through_focus_log */
/* Address: 0x00013ae8 */
/* ============================================= */

void debug_through_focus_log(int param_1)

{
  uint uVar1;
  undefined4 *puVar2;
  
  if (*(short *)(param_1 + 0x24) == 0) {
    uVar1 = (uint)DAT_0001913c;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
    }
    if ((int)(uVar1 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC_AF",
                          "%s, curr_step, focus_value, R, G, B, confidence",
                          "debug_through_focus_log");
    }
  }
  uVar1 = (uint)DAT_0001913c;
  if ((int)(uVar1 << 0x18) < 0) {
    uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
  }
  if ((int)(uVar1 << 0x1b) < 0) {
    puVar2 = *(undefined4 **)(param_1 + 0x10);
    __android_log_print(6,"CAMERA RAWCHIP_PROC_AF","%s, %d, %d, %d, %d, %d %d",
                        "debug_through_focus_log",(int)*(short *)(param_1 + 0x24),
                        *(undefined4 *)(param_1 + 0x28),puVar2[1],*puVar2,puVar2[2],puVar2[3]);
  }
  return;
}



/* ============================================= */
/* Function: afsu_check_range */
/* Address: 0x00013b98 */
/* ============================================= */

int afsu_check_range(uint param_1,uint param_2,uint param_3,uint param_4,int param_5,short param_6,
                    short param_7,byte param_8)

{
  uint uVar1;
  uint uVar2;
  
  if ((99 < param_4) &&
     (-1 < (int)((uint)((float)param_1 / (float)param_4 < (float)(longlong)(int)param_6) << 0x1f)))
  {
    uVar1 = param_2;
    if (param_1 <= param_2) {
      uVar1 = param_1;
    }
    if (param_3 <= uVar1) {
      uVar1 = param_3;
    }
    uVar2 = param_2;
    if (param_2 < param_1) {
      uVar2 = param_1;
    }
    if (uVar2 < param_3) {
      uVar2 = param_3;
    }
    if (((-1 < (int)((uint)((float)uVar1 < ((float)param_8 / 255.0) * (float)uVar2) << 0x1f)) &&
        (param_5 <= DAT_00019150)) &&
       (((double)param_2 * 1.02 <= (double)param_1 || ((double)param_3 * 1.02 <= (double)param_1))))
    {
      if ((int)((uint)((float)param_1 / (float)param_4 < (float)(longlong)(int)param_7) << 0x1f) < 0
         ) {
        return 1;
      }
      uVar1 = (uint)(param_2 < param_1);
      if (param_1 <= param_3) {
        uVar1 = 0;
      }
      return uVar1 << 1;
    }
  }
  return 0;
}



/* ============================================= */
/* Function: caf_triggered */
/* Address: 0x00013ca0 */
/* ============================================= */

void caf_triggered(int param_1,undefined4 param_2)

{
  uint uVar1;
  
  if ((*(int *)(param_1 + 0x38) == 2) || (*(char *)(param_1 + 0x79) != '\0')) {
    uVar1 = (uint)DAT_0001913c;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff,&DAT_0001913c,uVar1 << 0x18,
                                 param_1,param_2);
    }
    if ((int)(uVar1 << 0x1e) < 0) {
      __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",
                          "afsu_algorithm change back to [Trigger] state due to device move in [%d] state/ROI change = %d"
                          ,*(undefined4 *)(param_1 + 8),*(undefined1 *)(param_1 + 0x79));
    }
    *(undefined1 *)(param_1 + 0xc) = 0;
    *(undefined4 *)(param_1 + 8) = 1;
    *(undefined1 *)(param_1 + 0xe) = 0;
    *(undefined1 *)(param_1 + 0x79) = 0;
  }
  return;
}



/* ============================================= */
/* Function: afsu_algorithm */
/* Address: 0x00013d08 */
/* ============================================= */

void afsu_algorithm(uint *param_1,byte *param_2)

{
  char cVar1;
  ushort uVar2;
  uint *puVar3;
  int iVar4;
  size_t __n;
  uint uVar5;
  int extraout_r1;
  byte bVar6;
  short sVar7;
  uint uVar8;
  char *pcVar9;
  undefined1 uVar10;
  byte bVar11;
  char cVar12;
  uint uVar13;
  undefined1 *puVar14;
  int iVar15;
  short *psVar16;
  uint uVar17;
  bool bVar18;
  uint uVar19;
  uint uVar20;
  uint uVar21;
  float fVar22;
  char local_7c [12];
  char local_70;
  char acStack_6f [11];
  char local_64;
  char acStack_63 [11];
  char local_58;
  char local_57;
  char acStack_4c [32];
  int local_2c;
  
  cVar12 = *(char *)((int)param_1 + 0x15);
  uVar5 = param_1[5];
  puVar3 = (uint *)param_1[4];
  uVar13 = param_1[2];
  local_2c = __stack_chk_guard;
  uVar8 = *puVar3;
  uVar20 = puVar3[1];
  uVar21 = puVar3[2];
  uVar17 = puVar3[3];
  iVar4 = check_n_update_htccb_status(5,uVar13 != 0);
  if (iVar4 != 0) {
    do_htccb(5,uVar13 != 0,0);
  }
  __system_property_get("debug.through_focus.log",local_7c);
  __system_property_get("debug.through_focus.move",&local_70);
  uVar13 = atoi(acStack_6f);
  uVar19 = uVar13 & 0xffff;
  __system_property_get("debug.through_focus.delay",&local_64);
  iVar4 = atoi(acStack_63);
  __system_property_get("debug.through_focus.trigger",&local_58);
  if (local_7c[0] == 'A') {
    debug_through_focus_log(param_1);
  }
  if (local_70 == 'A') {
    if (*param_1 == DAT_00019154) {
      *param_2 = 0;
      if (uVar19 == 0) {
        bVar11 = 1;
      }
      else {
        bVar11 = (byte)uVar13;
      }
      param_2[1] = bVar11;
      bVar18 = true;
    }
    else {
      *param_2 = 1;
      param_2[1] = (byte)(short)param_1[9];
      bVar18 = false;
    }
    DAT_00019154 = *param_1;
  }
  else if (local_70 == 'B') {
    *param_2 = (int)uVar19 < (int)(short)param_1[9];
    iVar15 = (int)(short)param_1[9] - uVar19;
    bVar18 = true;
    if (iVar15 < 0) {
      iVar15 = -iVar15;
    }
    param_2[1] = (byte)iVar15;
  }
  else {
    bVar18 = false;
  }
  if (local_64 == 'A') {
    usleep(iVar4 * 1000);
  }
  if (bVar18) {
    uVar13 = 1;
    goto LAB_00015a92;
  }
  if (local_58 != local_57) {
    uVar13 = 1;
    param_1[2] = 1;
    local_58 = local_57;
    __system_property_set("debug.through_focus.trigger",&local_58);
    *param_2 = 1;
    param_2[1] = (byte)(short)param_1[9];
    goto LAB_00015a92;
  }
  if (param_1[1] == 2) {
    DAT_0001b31c = 1;
    DAT_0001b1a4 = 0;
  }
  else {
    DAT_0001b1a4 = DAT_0001b1a4 + 1;
    __aeabi_idivmod(DAT_0001b1a4,0x14);
    if (extraout_r1 == 0) {
      DAT_0001b31c = 0;
      DAT_0001b1a4 = 0;
    }
  }
  if (0x44 < (short)param_1[9]) {
    DAT_00019150 = 0x11d;
    DAT_00019144 = 3;
    DAT_00019140 = 5;
  }
  DAT_0001b2ea = DAT_0001b2ea + 1;
  uVar13 = (uint)DAT_0001b2ea;
  iVar4 = 0;
  param_2[1] = 0;
  uVar17 = afsu_check_range(uVar8,uVar20,uVar21,uVar17,param_1[0xb],(int)(short)param_1[6],
                            (int)*(short *)((int)param_1 + 0x1a),(char)param_1[7]);
  puVar3 = param_1;
  do {
    iVar4 = iVar4 + 1;
    *(undefined1 *)((int)puVar3 + 0x6d) = *(undefined1 *)((int)puVar3 + 0x6e);
    puVar3 = (uint *)((int)puVar3 + 1);
  } while (iVar4 != 0xb);
  uVar19 = param_1[2];
  *(char *)(param_1 + 0x1e) = (char)uVar17;
  if ((((1 < uVar19 - 3 && uVar19 != 6) && (uVar19 != 5)) &&
      ((param_1[0x1b] & 0xffffff00) == 0x2020200)) &&
     (((uVar17 & 0xff) == 0 && ((param_1[0x1d] & 0xffffff00) == 0)))) {
    *(undefined1 *)(param_1 + 0x1b) = 1;
  }
  if ((((DAT_0001b1a9 == '\x01') || (uVar19 == 1)) ||
      ((uVar19 == 3 || (((1 < *(byte *)((int)param_1 + 0xd) || (uVar19 == 4)) || (uVar19 == 6))))))
     || (uVar19 == 5)) {
    if (uVar17 == 1) {
      uVar17 = 0;
    }
    if (uVar19 != 3) goto LAB_00013f88;
    if (0 < (int)DAT_0001915c) goto LAB_00013f8c;
LAB_00013f92:
    if (uVar17 == 2) {
      uVar17 = 0;
    }
  }
  else {
LAB_00013f88:
    if (uVar19 == 1) goto LAB_00013f92;
LAB_00013f8c:
    if (1 < *(byte *)((int)param_1 + 0xd)) goto LAB_00013f92;
  }
  if ((uVar17 == 1 || uVar19 - 4 < 3) && (*param_1 == 2)) {
    uVar17 = 0;
  }
  uVar13 = DAT_0001b1ce - uVar13;
  if ((int)uVar13 < 0) {
    uVar13 = -uVar13;
  }
  if (uVar13 == DAT_0001b30a) {
    DAT_00019148 = param_1[10];
  }
  __system_property_get("ro.bootmode",acStack_4c);
  if (((char)param_1[0x1e] == '\x02') && (*(byte *)((int)param_1 + 0x77) < 2)) {
    if (DAT_0001b1c8 == 0) {
      fVar22 = 0.0;
    }
    else {
      fVar22 = ((float)param_1[10] - (float)DAT_0001b1c8) / (float)DAT_0001b1c8;
    }
    if ((fVar22 <= 0.07) && (DAT_0001b1c8 != 0)) {
      uVar17 = 0;
      *(undefined1 *)(param_1 + 0x1e) = 0;
    }
  }
  __n = strlen(acStack_4c);
  iVar4 = strncmp(acStack_4c,"factory2",__n);
  uVar13 = DAT_0001b310;
  if (((iVar4 != 0) && (param_1[1] != 1)) && (param_1[1] != 2)) {
    if (uVar17 == 1) {
      *param_2 = uVar21 <= uVar20;
      param_2[1] = cVar12 / '\x02';
      if (uVar13 == 0) {
        uVar5 = (uint)DAT_0001913c;
        if ((int)(uVar5 << 0x18) < 0) {
          uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
        }
        if ((int)(uVar5 << 0x1e) < 0) {
          __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",
                              "afsu Convergence Zone Region - curr_step:%d, direction:%d, focus_step:%d"
                              ,(int)(short)param_1[9],(int)(char)*param_2,(int)(char)param_2[1]);
        }
      }
      DAT_0001b1a8 = 0;
      DAT_0001b1c1 = DAT_0001b1c1 + 1;
      if (DAT_0001b1c1 < 4) {
        if (DAT_0001b1c1 == 1) {
          DAT_0001b300 = *param_2;
LAB_00015894:
          DAT_0001b1a1 = '\0';
        }
        else {
          if ((*param_2 != DAT_0001b300) || (param_1[10] <= DAT_0001b1c8)) goto LAB_00015894;
          DAT_0001b2e4 = (short)param_1[9];
          DAT_0001b1a1 = DAT_0001b1a1 + '\x01';
        }
        uVar13 = 2;
      }
      else {
        param_2[1] = 0;
        uVar13 = 1;
        DAT_0001b1a9 = '\x01';
      }
      bVar11 = 0;
      param_1[2] = 0;
      *(undefined1 *)((int)param_1 + 0xd) = 0;
      DAT_0001b2ec = 0;
      if (*param_2 == 1) {
        if ((int)(short)param_1[9] - (int)(char)param_2[1] < 0x13) {
LAB_000158bc:
          param_2[1] = bVar11;
        }
      }
      else if ((*param_2 == 0) && (0x75 < (int)(short)param_1[9] + (int)(char)param_2[1])) {
        param_2[1] = 0;
      }
    }
    else {
      if (uVar17 == 0) goto LAB_000140b0;
      uVar13 = 0;
      if (uVar17 == 2) {
        DAT_0001b1a8 = DAT_0001b1a8 + 1;
        DAT_0001b1c1 = 0;
        DAT_0001b1a9 = '\0';
        iVar4 = (int)((((float)uVar20 - (float)uVar21) / (float)uVar8) * 2048.0);
        *param_2 = (byte)~(byte)((uint)iVar4 >> 0x18) >> 7;
        fVar22 = (float)((double)(longlong)((char)uVar5 * iVar4) * 0.00048828125);
        if ((int)((uint)(fVar22 < 0.0) << 0x1f) < 0) {
          fVar22 = -fVar22;
        }
        if ((fVar22 < 0.5) || (-1 < (int)((uint)(fVar22 < 2.0) << 0x1f))) {
          bVar11 = (byte)(int)fVar22;
        }
        else {
          bVar11 = 2;
        }
        iVar4 = (int)cVar12;
        param_2[1] = bVar11;
        if (iVar4 / 2 < (int)(char)bVar11) {
          uVar5 = (uint)DAT_0001913c;
          if ((int)(uVar5 << 0x18) < 0) {
            uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
          }
          if ((int)(uVar5 << 0x1b) < 0) {
            __android_log_print(6,"CAMERA RAWCHIP_PROC_AF",
                                "calculated focus_step %d is over linearZoneRange/2, fix to linearZoneRange/2\n"
                                ,(int)(char)param_2[1]);
          }
          param_2[1] = (byte)((uint)((iVar4 - (iVar4 >> 0x1f)) * 0x800000) >> 0x18);
        }
        if (DAT_0001b310 == 0) {
          uVar5 = (uint)DAT_0001913c;
          if ((int)(uVar5 << 0x18) < 0) {
            uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
          }
          if ((int)(uVar5 << 0x1e) < 0) {
            __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",
                                "afsu Linear Zone Region - curr_step:%d, direction:%d, focus_step:%d"
                                ,(int)(short)param_1[9],(int)(char)*param_2,(int)(char)param_2[1]);
          }
        }
        if ((char)param_2[1] < '\x04') {
          param_2[1] = 0;
        }
        bVar11 = DAT_0001b1a8;
        if (param_2[1] == 0) {
          uVar13 = 1;
        }
        else {
          uVar13 = 2;
        }
        DAT_0001b2e4 = (short)param_1[9];
        param_1[2] = 0;
        *(undefined1 *)((int)param_1 + 0xd) = 0;
        DAT_0001b2ec = 0;
        if (2 < bVar11) {
          DAT_00019148 = param_1[10];
        }
      }
    }
    goto LAB_00015a68;
  }
  uVar5 = (uint)DAT_0001913c;
  if ((int)(uVar5 << 0x18) < 0) {
    uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
  }
  if ((int)(uVar5 << 0x1e) < 0) {
    __android_log_print(3,"CAMERA RAWCHIP_PROC_AF","%s ro.bootmode: %s ","afsu_algorithm",acStack_4c
                       );
  }
  if (param_1[1] - 1 < 2) {
    *(undefined1 *)(param_1 + 0x1e) = 0;
  }
LAB_000140b0:
  DAT_0001b1a8 = 0;
  if (param_1[2] == 0) {
    iVar4 = caf_triggered(param_1,param_2);
    if (iVar4 == 0) {
      cVar12 = *(char *)((int)param_1 + 0xd);
      uVar5 = param_1[0xb];
      if (cVar12 == '\0') {
        iVar4 = uVar5 - DAT_0001b304;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
        if (DAT_00019144 < iVar4) {
LAB_00014118:
          uVar10 = 1;
          DAT_0001b1ac = DAT_0001b304;
          DAT_0001b314 = DAT_0001b2f0;
          goto LAB_0001423e;
        }
        if (DAT_00019150 <= (int)uVar5) {
          iVar4 = DAT_0001b2f0 - param_1[0xc];
          if (iVar4 < 0) {
            iVar4 = -iVar4;
          }
          if (10 < iVar4) goto LAB_00014118;
        }
      }
      else {
        iVar4 = uVar5 - DAT_0001b1ac;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
        if (DAT_00019144 < iVar4) {
LAB_0001416c:
          *(char *)((int)param_1 + 0xd) = cVar12 + '\x01';
          if (2 < (byte)(cVar12 + 1U)) {
            *(undefined1 *)(param_1 + 3) = 0;
            param_1[2] = 1;
            *(undefined1 *)((int)param_1 + 0xe) = 0;
          }
          goto LAB_00014240;
        }
        if (DAT_00019150 <= (int)uVar5) {
          iVar4 = DAT_0001b314 - param_1[0xc];
          if (iVar4 < 0) {
            iVar4 = -iVar4;
          }
          if (10 < iVar4) goto LAB_0001416c;
        }
        uVar10 = 0;
LAB_0001423e:
        *(undefined1 *)((int)param_1 + 0xd) = uVar10;
      }
LAB_00014240:
      if (DAT_0001b2ec < 3) {
        iVar4 = param_1[10] - DAT_00019148;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
        if (0.19 <= (float)(longlong)iVar4 / (float)DAT_00019148) {
          iVar4 = (uint)DAT_0001b1ce - (uint)DAT_0001b2ea;
          if (iVar4 < 0) {
            iVar4 = -iVar4;
          }
          if ((int)(uint)DAT_0001b30a < iVar4) {
            DAT_0001b2ec = DAT_0001b2ec + 1;
            DAT_0001b2e8 = 0;
          }
        }
        if (DAT_0001b2ec == 1) {
          DAT_0001b308 = DAT_0001b2ea;
        }
        iVar4 = (uint)DAT_0001b2ea - (uint)DAT_0001b308;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
        if ((6 < iVar4) && (DAT_0001b2ec < 3)) {
          puVar14 = &DAT_0001b2ec;
LAB_00014350:
          *puVar14 = 0;
        }
      }
      else {
        iVar4 = param_1[10] - DAT_0001b1c8;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
        if (0.1 < (float)(longlong)iVar4 / (float)DAT_0001b1c8) {
          puVar14 = &DAT_0001b2e8;
          goto LAB_00014350;
        }
        DAT_0001b2e8 = DAT_0001b2e8 + 1;
        if (5 < DAT_0001b2e8) {
          *(undefined1 *)(param_1 + 3) = 0;
          param_1[2] = 1;
          *(undefined1 *)((int)param_1 + 0xe) = 0;
        }
      }
      if ((char)param_1[0x1b] == '\x01') {
        param_1[2] = 1;
        *(undefined1 *)(param_1 + 3) = 0;
        *(undefined1 *)((int)param_1 + 0xe) = 0;
        *(undefined1 *)(param_1 + 0x1b) = 0;
      }
      if (DAT_0001b1c0 != '\0') {
        uVar5 = (uint)DAT_0001913c;
        if ((int)(uVar5 << 0x18) < 0) {
          uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
        }
        if ((int)(uVar5 << 0x1e) < 0) {
          __android_log_print(3,"CAMERA RAWCHIP_PROC_AF","afsu boundary triggered");
        }
        *(undefined1 *)(param_1 + 3) = 0;
        param_1[2] = 1;
        *(undefined1 *)((int)param_1 + 0xe) = 0;
      }
      if (param_1[2] != 0) goto LAB_00014414;
      if (((*param_1 != 2) && (DAT_0001b1c1 != 0)) && (DAT_0001b1a1 < '\x03')) {
        cVar12 = (char)DAT_0001b2e4;
        if (DAT_0001b2e4 < (short)param_1[9]) {
          *param_2 = 1;
          bVar11 = (char)param_1[9] - cVar12;
        }
        else {
          *param_2 = 0;
          bVar11 = cVar12 - (char)param_1[9];
        }
        DAT_0001b300 = *param_2;
        param_2[1] = bVar11;
        DAT_0001b1a1 = '\0';
      }
LAB_00015a58:
      uVar17 = 0;
      uVar13 = 1;
      goto LAB_00015a68;
    }
LAB_00015a66:
    uVar17 = 0;
    uVar13 = 2;
  }
  else {
LAB_00014414:
    if (param_1[2] == 1) {
      uVar5 = (uint)DAT_0001913c;
      if ((int)(uVar5 << 0x18) < 0) {
        uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
      }
      if ((int)(uVar5 << 0x1e) < 0) {
        __android_log_print(3,"CAMERA RAWCHIP_PROC_AF","afsu [Trigger] state, curr_step:%d\n",
                            (int)(short)param_1[9]);
      }
      DAT_0001b1c1 = 0;
      DAT_0001b1a9 = '\0';
      iVar4 = 0;
      do {
        iVar15 = iVar4 + 1;
        *(undefined1 *)((int)param_1 + iVar4 + 0x6d) = 0;
        iVar4 = iVar15;
      } while (iVar15 != 0xc);
      uVar5 = param_1[0xc];
      if (DAT_00019150 < (int)param_1[0xb]) {
        iVar4 = param_1[0xd] - uVar5;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
        if (iVar4 <= DAT_00019140) goto LAB_000144b4;
        if (uVar5 < 0x1a) {
          iVar4 = DAT_0001b2f0 - uVar5;
          if (iVar4 < 0) {
            iVar4 = -iVar4;
          }
          goto LAB_000144b0;
        }
LAB_0001455c:
        iVar4 = param_1[0xb] - DAT_0001b304;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
        cVar12 = '\0';
        if (iVar4 < 2) {
          *(char *)((int)param_1 + 0xe) = *(char *)((int)param_1 + 0xe) + '\x01';
        }
        else {
          *(undefined1 *)((int)param_1 + 0xe) = 0;
        }
      }
      else {
        iVar4 = param_1[0xd] - uVar5;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
LAB_000144b0:
        if (DAT_00019140 < iVar4) goto LAB_0001455c;
LAB_000144b4:
        cVar12 = (char)param_1[3] + '\x01';
      }
      *(char *)(param_1 + 3) = cVar12;
      if ((((byte)param_1[3] < 2) || (param_1[0xe] == 2)) && (*(byte *)((int)param_1 + 0xe) < 6))
      goto LAB_00015a66;
      param_1[2] = 2;
    }
    if (param_1[2] == 2) {
      uVar5 = (uint)DAT_0001913c;
      if ((int)(uVar5 << 0x18) < 0) {
        uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
      }
      if ((int)(uVar5 << 0x1e) < 0) {
        __android_log_print(3,"CAMERA RAWCHIP_PROC_AF","afsu [Settle] state, curr_step:%d\n",
                            (int)(short)param_1[9]);
      }
      uVar5 = 0;
      param_1[2] = 4;
      do {
        (&DAT_0001b1d0)[uVar5 * 4] = 0;
        (&DAT_0001b1d4)[uVar5 * 2] = 0;
        uVar5 = uVar5 + 1 & 0xff;
      } while (uVar5 < 0x22);
      DAT_0001b1b4 = 0;
      DAT_0001915c = 0;
      DAT_0001b1c4 = 0;
      DAT_0001b2fc = 0;
    }
    uVar5 = param_1[2];
    if (uVar5 == 4) {
      uVar5 = (uint)DAT_0001913c;
      if ((int)(uVar5 << 0x18) < 0) {
        uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
      }
      if ((int)(uVar5 << 0x1e) < 0) {
        __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",
                            "afsu [Detect][%d] state, curr_step:%d, fv=%d",DAT_0001b1b4,
                            (int)(short)param_1[9],param_1[10]);
      }
      iVar4 = caf_triggered(param_1,param_2);
      bVar11 = DAT_0001b1b4;
      if (iVar4 != 0) goto LAB_00015a66;
      uVar5 = param_1[10];
      uVar8 = (uint)DAT_0001b1b4;
      (&DAT_0001b1d0)[uVar8 * 4] = (short)param_1[9];
      (&DAT_0001b1d4)[uVar8 * 2] = uVar5;
      if (*param_1 == 2) {
        bVar6 = 4;
LAB_000146bc:
        param_2[1] = bVar6;
      }
      else if (*param_1 < 2) {
        bVar6 = 8;
        goto LAB_000146bc;
      }
      if (((uVar8 == 0) || ((short)param_1[9] < 0x19)) || (0x6f < (short)param_1[9])) {
        *param_2 = 0x18 < (short)param_1[9] && (short)param_1[9] < 0x45;
        sVar7 = (short)param_1[9];
        if (sVar7 < 0x19) {
          DAT_0001b2e6 = 0x70 - sVar7;
          bVar11 = 0;
        }
        else {
          if (sVar7 < 0x70) goto LAB_000147b0;
          DAT_0001b2e6 = sVar7 + -0x18;
          bVar11 = 1;
        }
        *param_2 = bVar11;
        param_1[2] = 6;
        if (*param_1 == 2) {
          if (DAT_00019150 < (int)param_1[0xb]) {
LAB_00014756:
            DAT_0001b1cc = 0x12;
            DAT_0001b318 = 8;
          }
          else {
            DAT_0001b318 = 4;
            DAT_0001b1cc = 0x22;
          }
        }
        else if (*param_1 < 2) goto LAB_00014756;
        uVar5 = 0;
        DAT_0001b2f8 = 0;
        do {
          (&DAT_0001b1d0)[uVar5 * 4] = 0;
          (&DAT_0001b1d4)[uVar5 * 2] = 0;
          uVar5 = uVar5 + 1 & 0xff;
        } while (uVar5 < 0x22);
        DAT_0001b1d4 = param_1[10];
        DAT_0001b1d0 = (short)param_1[9];
        DAT_0001b1b4 = 1;
        DAT_0001b2fc = DAT_0001b1d4;
        param_1[0xf] = (int)DAT_0001b1d0;
        DAT_0001b1c4 = 0;
      }
      else {
LAB_000147b0:
        if (uVar8 < 2) {
          DAT_0001b1b4 = bVar11 + 1;
          goto LAB_00015a66;
        }
        if (((uint)(0.0 < (double)DAT_0001b1dc * 1.02) *
             (int)(longlong)((double)DAT_0001b1dc * 1.02) < DAT_0001b1e4) &&
           ((uint)(0.0 < (double)DAT_0001b1d4 * 1.02) * (int)(longlong)((double)DAT_0001b1d4 * 1.02)
            < DAT_0001b1dc)) {
          DAT_0001b1b0 = 1;
        }
        else {
          DAT_0001b1b0 = 0;
          if (*param_2 == 0) {
            *param_2 = 1;
          }
          else {
            *param_2 = 0;
          }
        }
        if (*param_2 == 1) {
          iVar4 = -0x18;
        }
        else {
          iVar4 = -0x70;
        }
        iVar4 = (short)param_1[9] + iVar4;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
        DAT_0001b2e6 = (short)iVar4;
        param_1[2] = 6;
        DAT_0001b2fc = DAT_0001b1b0;
        if (*param_1 == 2) {
          if (DAT_00019150 < (int)param_1[0xb]) {
LAB_000148ac:
            DAT_0001b1cc = 0x12;
            DAT_0001b318 = 8;
          }
          else {
            DAT_0001b318 = 4;
            DAT_0001b1cc = 0x22;
          }
        }
        else if (*param_1 < 2) goto LAB_000148ac;
        if (DAT_0001b1b0 == 0) {
          param_1[0xf] = 0;
          DAT_0001b2f8 = 0;
          uVar5 = 0;
          do {
            (&DAT_0001b1d0)[uVar5 * 4] = 0;
            uVar8 = uVar5 + 1 & 0xff;
            (&DAT_0001b1d4)[uVar5 * 2] = 0;
            uVar5 = uVar8;
          } while (uVar8 < 0x22);
          DAT_0001b1b4 = 0;
        }
        else {
          param_1[0xf] = (int)(short)param_1[9];
          DAT_0001b2fc = DAT_0001b1e4;
          DAT_0001b1b4 = DAT_0001b1b4 + 1;
          DAT_0001b2f8 = 0;
        }
      }
      goto LAB_00015a66;
    }
    if (uVar5 != 6) {
      if (uVar5 == 5) {
        uVar5 = (uint)DAT_0001913c;
        if ((int)(uVar5 << 0x18) < 0) {
          uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
        }
        if ((int)(uVar5 << 0x1e) < 0) {
          __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",
                              "afsu [Video Move] state [PURE], curr_step:%d, remain moveback_dest_step:%d\n"
                              ,(int)(short)param_1[9],(int)DAT_0001b2e6);
        }
        iVar4 = caf_triggered(param_1,param_2);
        if (iVar4 != 0) goto LAB_00015a66;
        if ((8 < DAT_0001b2e6) && (*param_1 == 2)) {
          DAT_0001b2e6 = DAT_0001b2e6 + -8;
          param_2[1] = 8;
          uVar17 = 0;
          uVar13 = 2;
          goto LAB_00015a68;
        }
        param_2[1] = (byte)DAT_0001b2e6;
        DAT_0001b1ce = DAT_0001b2ea;
        param_1[2] = 0;
        DAT_0001b2e6 = 0;
        sVar7 = 2;
        *(undefined1 *)((int)param_1 + 0xd) = 0;
        psVar16 = (short *)&DAT_0001b30a;
LAB_000157ae:
        DAT_0001b2ec = 0;
        *psVar16 = sVar7;
        goto LAB_00015a58;
      }
      if (uVar5 == 7) {
        uVar5 = (uint)DAT_0001913c;
        if ((int)(uVar5 << 0x18) < 0) {
          uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
        }
        if ((int)(uVar5 << 0x1e) < 0) {
          __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",
                              "afsu [Video [Precise] state, curr_step:%d, fv=%d",
                              (int)(short)param_1[9],param_1[10]);
        }
        uVar2 = DAT_0001b2ea;
        if (DAT_0001b1b8 < param_1[10]) {
          param_2[1] = 1;
          *param_2 = DAT_0001914c;
          DAT_0001b1b8 = param_1[10];
          DAT_0001b30c = (int)(short)param_1[9];
          goto LAB_00015a66;
        }
        *param_2 = DAT_0001914c != 1;
        uVar17 = 0;
        param_2[1] = 1;
        DAT_0001b2ec = 0;
        param_1[2] = 0;
        *(undefined1 *)((int)param_1 + 0xd) = 0;
        DAT_0001b1ce = uVar2;
        DAT_0001b30a = 2;
        uVar13 = 1;
      }
      else {
        if (uVar5 == 3) {
          uVar5 = (uint)DAT_0001913c;
          if ((int)(uVar5 << 0x18) < 0) {
            uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
          }
          if ((int)(uVar5 << 0x1e) < 0) {
            __android_log_print(3,"CAMERA RAWCHIP_PROC_AF","afsu [Running] state, curr_step:%d\n",
                                (int)(short)param_1[9]);
          }
          if (param_1[0xe] == 2) {
            uVar17 = 0;
            param_1[2] = 1;
            *(undefined1 *)(param_1 + 3) = 0;
            *(undefined1 *)((int)param_1 + 0xe) = 0;
            if (param_1[8] == 1) {
              *param_2 = 0x70 < (short)param_1[9];
              iVar4 = (short)param_1[9] + -0x70;
              if (iVar4 < 0) {
                iVar4 = -iVar4;
              }
              bVar11 = (byte)iVar4;
              uVar13 = 1;
              goto LAB_000158bc;
            }
            if ((short)param_1[9] < 0x65) {
              *param_2 = 1;
              param_2[1] = (char)param_1[9] - 0x18;
              param_1[2] = 1;
              uVar13 = 1;
            }
            else {
              uVar13 = 2;
            }
            goto LAB_00015a68;
          }
        }
        if (param_1[2] < 8) {
          if ((int)DAT_0001915c < 0) {
            *param_2 = 0x18 < (short)param_1[9];
            iVar4 = (short)param_1[9] + -0x18;
            if (DAT_0001915c == 0xffffffff) {
              if (iVar4 < 0) {
                iVar4 = -iVar4;
              }
              param_2[1] = (byte)iVar4;
            }
            else {
              if (iVar4 < 0) {
                iVar4 = -iVar4;
              }
              param_2[1] = (byte)(iVar4 >> 1);
              if (DAT_0001915c == 0xfffffffe) {
                DAT_00019158 = param_1[10];
              }
            }
            DAT_0001915c = DAT_0001915c + 1;
            DAT_0001b1aa = 0;
            DAT_0001b2e0 = 0;
            DAT_0001b1bc = 0xffffffff;
            iVar4 = 0;
            do {
              iVar15 = iVar4 + 4;
              *(undefined4 *)((int)param_1 + iVar4 + 0x40) = 0;
              iVar4 = iVar15;
            } while (iVar15 != 0x2c);
          }
          else if (((int)DAT_0001915c < 10) && (DAT_0001b1aa < 2)) {
            if (DAT_0001915c != 0) {
              uVar5 = param_1[10];
              if ((int)DAT_0001915c < 5) {
                if ((uint)(0.0 < (double)uVar5 * 0.04) * (int)(longlong)((double)uVar5 * 0.04) +
                    uVar5 < DAT_0001b2e0) {
                  uVar8 = param_1[DAT_0001915c + 0xf];
joined_r0x00015334:
                  if ((uVar5 < uVar8) && (DAT_00019158 <= DAT_0001b2e0)) {
                    DAT_0001b1aa = DAT_0001b1aa + 1;
                    goto LAB_0001534e;
                  }
                }
              }
              else if ((uint)(0.0 < (double)uVar5 * 0.07) * (int)(longlong)((double)uVar5 * 0.07) +
                       uVar5 < DAT_0001b2e0) {
                uVar8 = param_1[DAT_0001915c + 0xf];
                goto joined_r0x00015334;
              }
              DAT_0001b1aa = 0;
            }
LAB_0001534e:
            if ((int)DAT_0001915c < 5) {
              bVar11 = 8;
            }
            else {
              bVar11 = 0xc;
            }
            *param_2 = 0;
            param_2[1] = bVar11;
            uVar5 = DAT_0001b2e0;
            uVar8 = param_1[10];
            param_1[DAT_0001915c + 0x10] = uVar8;
            if (uVar5 < uVar8) {
              DAT_0001b1c4 = DAT_0001915c;
              DAT_0001b2e0 = uVar8;
            }
            if (uVar8 < DAT_0001b1bc) {
              DAT_0001b2f4 = DAT_0001915c;
              DAT_0001b1bc = uVar8;
            }
            DAT_0001915c = DAT_0001915c + 1;
            if (DAT_0001b1c4 < 5) {
              DAT_0001b2e4 = (short)DAT_0001b1c4 * 8 + 0x18;
            }
            else {
              DAT_0001b2e4 = (short)DAT_0001b1c4 * 0xc + 4;
            }
            DAT_0001b30a = 4;
            DAT_0001b1ce = DAT_0001b2ea;
          }
          uVar5 = DAT_0001b1c4;
          if (((int)DAT_0001915c < 10) && (DAT_0001b1aa < 2)) goto LAB_00015a66;
          DAT_0001915c = DAT_0001915c - 1;
          *param_2 = 1;
          cVar12 = (char)DAT_0001915c;
          cVar1 = (char)uVar5;
          if (uVar5 < 5) {
            if ((int)DAT_0001915c < 6) {
              bVar11 = (cVar12 - cVar1) * '\b';
            }
            else {
              bVar11 = cVar12 * '\f' + cVar1 * -8 + -0x14;
            }
          }
          else {
            bVar11 = (cVar12 - cVar1) * '\f';
          }
          param_2[1] = bVar11;
          uVar8 = param_1[uVar5 + 0xf];
          uVar13 = param_1[uVar5 + 0x11];
          if (uVar5 - 1 < 4) {
            if (uVar8 >= uVar13 && uVar8 != uVar13) {
              bVar11 = param_2[1] + 2;
              goto LAB_00015464;
            }
            if (uVar8 < uVar13) {
              bVar11 = param_2[1] - 2;
              goto LAB_00015464;
            }
          }
          else {
            if (uVar13 <= uVar8 && uVar8 != uVar13) {
              bVar11 = param_2[1] + 4;
            }
            else {
              if (uVar13 <= uVar8) goto LAB_00015466;
              bVar11 = param_2[1] - 4;
            }
LAB_00015464:
            param_2[1] = bVar11;
          }
LAB_00015466:
          uVar5 = (uint)DAT_0001913c;
          if ((int)(uVar5 << 0x18) < 0) {
            uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
          }
          if ((int)(uVar5 << 0x1e) < 0) {
            __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",
                                "afsu fv[0]:%d, fv[1]:%d, fv[2]:%d, fv[3]:%d, fv[4]:%d, fv[5]:%d, fv[6]:%d, fv[7]:%d, fv[8]:%d, fv[9]:%d, fv[10]:%d\n"
                                ,param_1[0x10],param_1[0x11],param_1[0x12],param_1[0x13],
                                param_1[0x14],param_1[0x15],param_1[0x16],param_1[0x17],
                                param_1[0x18],param_1[0x19],param_1[0x1a]);
          }
          if (8 < (int)DAT_0001915c) {
            if (param_1[8] == 1) {
              if ((int)((uint)((float)((param_1[DAT_0001b1c4 + 0x10] - param_1[DAT_0001b2f4 + 0x10])
                                      * 100) / (float)param_1[DAT_0001b1c4 + 0x10] < 25.0) << 0x1f)
                  < 0) {
                *param_2 = 0x70 < (short)param_1[9];
                iVar4 = (short)param_1[9] + -0x70;
LAB_00015772:
                if (iVar4 < 0) {
                  iVar4 = -iVar4;
                }
                param_2[1] = (byte)iVar4;
              }
            }
            else if (((int)((uint)((float)((param_1[DAT_0001b1c4 + 0x10] -
                                           param_1[DAT_0001b2f4 + 0x10]) * 100) /
                                   (float)param_1[DAT_0001b1c4 + 0x10] < 25.0) << 0x1f) < 0) &&
                    (0x13b < (int)param_1[0xb])) {
              iVar4 = (short)param_1[9] + -0x18;
              goto LAB_00015772;
            }
          }
          param_1[2] = 0;
          DAT_0001b1ce = DAT_0001b2ea;
          *(undefined1 *)((int)param_1 + 0xd) = 0;
          psVar16 = &DAT_0001b2e4;
          DAT_0001b30a = 4;
          sVar7 = (short)param_1[9] - (short)(char)param_2[1];
          goto LAB_000157ae;
        }
        uVar5 = (uint)DAT_0001913c;
        if ((int)(uVar5 << 0x18) < 0) {
          uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
        }
        if ((int)(uVar5 << 0x1b) < 0) {
          __android_log_print(6,"CAMERA RAWCHIP_PROC_AF",
                              "!!!!! afsu_algorithm state machine error !!!!!");
        }
        uVar17 = 0;
        uVar13 = uVar17;
      }
      goto LAB_00015a68;
    }
    uVar5 = (uint)DAT_0001913c;
    if ((int)(uVar5 << 0x18) < 0) {
      uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
    }
    if ((int)(uVar5 << 0x1e) < 0) {
      __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",
                          "afsu [Video Move] state [VALUE], curr_step:%d, remain moveback_dest_step:%d, fv=%d\n"
                          ,(int)(short)param_1[9],(int)DAT_0001b2e6,param_1[10]);
    }
    iVar4 = caf_triggered(param_1,param_2);
    bVar11 = DAT_0001b318;
    if (iVar4 != 0) goto LAB_00015a66;
    uVar8 = (uint)DAT_0001b1b4;
    param_2[1] = DAT_0001b318;
    uVar5 = param_1[10];
    sVar7 = (short)param_1[9];
    (&DAT_0001b1d4)[uVar8 * 2] = uVar5;
    (&DAT_0001b1d0)[uVar8 * 4] = sVar7;
    if (uVar8 == 0) {
      param_1[0xf] = (int)sVar7;
      DAT_0001b1c4 = uVar8;
      DAT_0001b2fc = uVar5;
    }
    else if ((uVar8 < DAT_0001b1cc) && (DAT_0001b2f8 < 2)) {
      if (DAT_0001b2fc < uVar5) {
        DAT_0001b2fc = uVar5;
        param_1[0xf] = (int)sVar7;
        DAT_0001b1c4 = uVar8;
      }
      if (((uint)(0.0 < (double)uVar5 * 1.04) * (int)(longlong)((double)uVar5 * 1.04) < DAT_0001b2fc
          ) && (uVar5 < *(uint *)(&DAT_0001b1cc + uVar8 * 8))) {
        DAT_0001b2f8 = DAT_0001b2f8 + 1;
      }
      else {
        DAT_0001b2f8 = 0;
      }
      DAT_0001b2e4 = (short)param_1[0xf];
      param_2[1] = bVar11;
    }
    bVar11 = DAT_0001b318;
    DAT_0001b2e6 = DAT_0001b2e6 - (char)param_2[1];
    if ((int)uVar8 < (int)(DAT_0001b1cc - 1)) {
      if (((1 < DAT_0001b2f8) || ((int)((uint)DAT_0001b318 * 2 + 0x70) <= (int)(short)param_1[9]))
         || (uVar13 = 2, (short)param_1[9] < 1)) {
        iVar4 = (int)(short)param_1[9];
        if (((int)((uint)DAT_0001b318 * 2 + 0x70) <= iVar4) || (iVar4 < 1)) goto LAB_00014c04;
        if (1 < DAT_0001b2f8) {
          *param_2 = (int)param_1[0xf] <= iVar4;
          uVar5 = DAT_0001b31c;
          iVar4 = param_1[0xf] - (int)(short)param_1[9];
          if (iVar4 < 0) {
            iVar4 = -iVar4;
          }
          param_2[1] = (byte)iVar4;
          if (uVar5 == 0) {
            if (DAT_0001b1c4 != uVar8) {
              if ((DAT_0001b1c4 == 0) ||
                 (*(uint *)(&DAT_0001b1cc + DAT_0001b1c4 * 8) <= (&DAT_0001b1dc)[DAT_0001b1c4 * 2]))
              {
                bVar11 = -(bVar11 >> 2);
              }
              else {
                bVar11 = bVar11 >> 2;
              }
              param_2[1] = (byte)iVar4 + bVar11;
            }
          }
          else {
            param_1[2] = 7;
            if ((DAT_0001b1c4 == 0) ||
               (*(uint *)(&DAT_0001b1cc + DAT_0001b1c4 * 8) <= (&DAT_0001b1dc)[DAT_0001b1c4 * 2])) {
              DAT_0001914c = false;
              if (*param_2 != 1) {
                DAT_0001914c = true;
              }
            }
            else {
              DAT_0001914c = *param_2 == 1;
            }
            DAT_0001b1b8 = 0;
            DAT_0001b30c = 0;
          }
          DAT_0001b1c0 = '\0';
        }
        uVar13 = 2;
        goto LAB_00014e5e;
      }
LAB_00014f94:
      if (*param_2 == 1) {
        if ((int)(short)param_1[9] - (int)(char)param_2[1] < 0) {
          param_2[1] = (byte)(short)param_1[9];
        }
      }
      else if (*param_2 == 0) {
        iVar4 = (uint)DAT_0001b318 * 2 + 0x70;
        if (iVar4 < (int)(short)param_1[9] + (int)(char)param_2[1]) {
          param_2[1] = (char)iVar4 - (char)(short)param_1[9];
        }
      }
      DAT_0001b1b4 = DAT_0001b1b4 + 1;
    }
    else {
LAB_00014c04:
      if (param_1[8] == 1) {
        *param_2 = 0x70 < (short)param_1[9];
        uVar5 = (uint)DAT_0001913c;
        iVar4 = (short)param_1[9] + -0x70;
        if (iVar4 < 0) {
          iVar4 = -iVar4;
        }
        param_2[1] = (byte)iVar4;
        if ((int)(uVar5 << 0x18) < 0) {
          uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
        }
        if ((int)(uVar5 << 0x1e) < 0) {
          pcVar9 = "afsu_algorithm focus fail, move to macro";
          goto LAB_00014c94;
        }
      }
      else {
        uVar5 = (uint)(short)param_1[9];
        if (uVar5 == param_1[0xf]) {
          param_2[1] = 0;
          uVar5 = (uint)DAT_0001913c;
          if ((int)(uVar5 << 0x18) < 0) {
            uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
          }
          if ((int)(uVar5 << 0x1e) < 0) {
            pcVar9 = "afsu_algorithm focus fail, FV increasing, move to last";
            goto LAB_00014c94;
          }
        }
        else if (DAT_0001b1c0 == '\0') {
          *param_2 = 0x18 < (int)uVar5;
          iVar4 = (short)param_1[9] + -0x18;
          if (iVar4 < 0) {
            iVar4 = -iVar4;
          }
          param_2[1] = (byte)iVar4;
        }
        else if (DAT_0001b1d4 < (&DAT_0001b1d4)[uVar8 * 2]) {
          *param_2 = 0x70 < (int)uVar5;
          iVar4 = (short)param_1[9] + -0x70;
          if (iVar4 < 0) {
            iVar4 = -iVar4;
          }
          param_2[1] = (byte)iVar4;
          uVar5 = (uint)DAT_0001913c;
          if ((int)(uVar5 << 0x18) < 0) {
            uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
          }
          if ((int)(uVar5 << 0x1e) < 0) {
            pcVar9 = "afsu_algorithm focus fail, macro better than infinity, move to macro";
            goto LAB_00014c94;
          }
        }
        else {
          *param_2 = 0x18 < (int)uVar5;
          iVar4 = (short)param_1[9] + -0x18;
          if (iVar4 < 0) {
            iVar4 = -iVar4;
          }
          param_2[1] = (byte)iVar4;
          uVar5 = (uint)DAT_0001913c;
          if ((int)(uVar5 << 0x18) < 0) {
            uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
          }
          if ((int)(uVar5 << 0x1e) < 0) {
            pcVar9 = "afsu_algorithm focus fail, infinity better than macro, move to infinity";
LAB_00014c94:
            __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",pcVar9);
          }
        }
      }
      if ((int)(short)param_1[9] == param_1[0xf]) {
        uVar13 = 1;
        DAT_0001b1c0 = '\0';
      }
      else {
        uVar13 = 1;
        if (DAT_0001b1c0 == '\0') {
          DAT_0001b1c0 = '\x01';
        }
        else {
          DAT_0001b1c0 = '\0';
        }
      }
LAB_00014e5e:
      uVar5 = (uint)DAT_0001913c;
      if ((int)(uVar5 << 0x18) < 0) {
        uVar5 = __htclog_init_mask("CAMERA RAWCHIP_PROC_AF",0xffffffff);
      }
      if ((int)(uVar5 << 0x1e) < 0) {
        __android_log_print(3,"CAMERA RAWCHIP_PROC_AF",
                            "afsu[video] fv[0]:%d, fv[1]:%d, fv[2]:%d, fv[3]:%d, fv[4]:%d, fv[5]:%d, fv[6]:%d, fv[7]:%d, fv[8]:%d, fv[9]:%d, fv[10]:%d, fv[11]:%d, fv[12]:%d, fv[13]:%d\n"
                            ,DAT_0001b1d4,DAT_0001b1dc,DAT_0001b1e4,DAT_0001b1ec,DAT_0001b1f4,
                            DAT_0001b1fc,DAT_0001b204,DAT_0001b20c,DAT_0001b214,DAT_0001b21c,
                            DAT_0001b224,DAT_0001b22c,DAT_0001b234,DAT_0001b23c);
      }
      uVar5 = DAT_0001b31c;
      uVar2 = DAT_0001b2ea;
      DAT_0001b2e4 = (short)param_1[0xf];
      if (DAT_0001b31c == 0) {
        bVar11 = param_2[1];
        if (((char)bVar11 < '\t') || (*param_1 != 2)) {
          DAT_0001b30a = 2;
          uVar13 = 1;
          param_1[2] = 0;
          *(undefined1 *)((int)param_1 + 0xd) = 0;
          DAT_0001b2ec = 0;
          DAT_0001b1ce = uVar2;
          goto LAB_00014f94;
        }
        param_2[1] = 0;
        DAT_0001b2e6 = (short)(char)bVar11;
        param_1[2] = 5;
        DAT_0001b2fc = uVar5;
        DAT_0001b2f8 = 0;
        uVar5 = 0;
        do {
          (&DAT_0001b1d0)[uVar5 * 4] = 0;
          (&DAT_0001b1d4)[uVar5 * 2] = 0;
          uVar5 = uVar5 + 1 & 0xff;
        } while (uVar5 < 0x22);
        DAT_0001b1b4 = 0;
      }
    }
    uVar17 = 0;
  }
LAB_00015a68:
  DAT_0001b304 = param_1[0xb];
  DAT_0001b2f0 = param_1[0xc];
  DAT_0001b1c8 = param_1[10];
  DAT_0001b310 = uVar17;
LAB_00015a92:
  if (local_2c == __stack_chk_guard) {
    return;
  }
                    /* WARNING: Subroutine does not return */
  __stack_chk_fail(uVar13);
}



/* ============================================= */
/* Function: FUN_00015b60 */
/* Address: 0x00015b60 */
/* ============================================= */

uint * FUN_00015b60(uint param_1)

{
  uint *puVar1;
  
  puVar1 = (uint *)0x0;
  if ((((param_1 & 0xff) < 7) &&
      (puVar1 = (uint *)(&DAT_0001b324)[param_1 & 0xff], puVar1 != (uint *)0x0)) &&
     (*puVar1 != param_1)) {
    puVar1 = (uint *)0x0;
  }
  return puVar1;
}



/* ============================================= */
/* Function: rawchip_proc_interface_create */
/* Address: 0x00015b84 */
/* ============================================= */

int rawchip_proc_interface_create(void)

{
  uint uVar1;
  int *piVar2;
  int iVar3;
  
  uVar1 = (uint)DAT_00019160;
  if ((int)(uVar1 << 0x18) < 0) {
    uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
  }
  if ((int)(uVar1 << 0x1d) < 0) {
    __android_log_print(4,"CAMERA RAWCHIP_PROC",&DAT_000169f0,"rawchip_proc_interface_create");
  }
  iVar3 = 0;
  piVar2 = &DAT_0001b320;
  while (piVar2 = piVar2 + 1, *piVar2 != 0) {
    iVar3 = iVar3 + 1;
    if (iVar3 == 8) {
      return 0;
    }
  }
  if (iVar3 != 7) {
    piVar2 = malloc(0x2c);
    (&DAT_0001b324)[iVar3] = piVar2;
    if (piVar2 != (int *)0x0) {
      memset(piVar2,0,0x2c);
      DAT_0001b320 = DAT_0001b320 + 1;
      iVar3 = iVar3 + DAT_0001b320 * 0x100;
      *piVar2 = iVar3;
      return iVar3;
    }
    uVar1 = (uint)DAT_00019160;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar1 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC","malloc rawchip_procCtrl failed\n");
      return 0;
    }
  }
  return 0;
}



/* ============================================= */
/* Function: rawchip_proc_init */
/* Address: 0x00015c64 */
/* ============================================= */

undefined4 rawchip_proc_init(undefined4 param_1,void *param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  uint uVar2;
  undefined4 uVar3;
  
  iVar1 = FUN_00015b60();
  if (iVar1 == 0) {
    uVar2 = (uint)DAT_00019160;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC"," %s: rawchipCtrl is NULL .. Exiting",
                          "rawchip_proc_init",param_4);
    }
    return 0xffffffff;
  }
  uVar2 = (uint)DAT_00019160;
  if ((int)(uVar2 << 0x18) < 0) {
    uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
  }
  if ((int)(uVar2 << 0x1d) < 0) {
    __android_log_print(4,"CAMERA RAWCHIP_PROC",&DAT_000169f0,"rawchip_proc_init");
  }
  memcpy((void *)(iVar1 + 4),param_2,0x28);
  uVar3 = rawchip_init(iVar1);
  return uVar3;
}



/* ============================================= */
/* Function: rawchip_proc_interface_destroy */
/* Address: 0x00015d14 */
/* ============================================= */

void rawchip_proc_interface_destroy
               (uint param_1,undefined4 param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  uint uVar2;
  
  iVar1 = FUN_00015b60();
  uVar2 = (uint)DAT_00019160;
  if ((int)(uVar2 << 0x18) < 0) {
    uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019160,uVar2 << 0x18,param_4)
    ;
  }
  if ((int)(uVar2 << 0x1d) < 0) {
    __android_log_print(4,"CAMERA RAWCHIP_PROC",&DAT_000169f0,"rawchip_proc_interface_destroy");
  }
  if (iVar1 != 0) {
    param_1 = param_1 & 0xff;
    rawchip_destroy();
    if (param_1 < 8) {
      if ((void *)(&DAT_0001b324)[param_1] != (void *)0x0) {
        free((void *)(&DAT_0001b324)[param_1]);
      }
      (&DAT_0001b324)[param_1] = 0;
    }
  }
  return;
}



/* ============================================= */
/* Function: rawchip_proc_get_params */
/* Address: 0x00015d90 */
/* ============================================= */

undefined4 rawchip_proc_get_params(undefined4 param_1,int *param_2)

{
  int iVar1;
  uint uVar2;
  undefined4 uVar3;
  
  iVar1 = FUN_00015b60();
  if (iVar1 == 0) {
    uVar2 = (uint)DAT_00019160;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC"," %s: rawchipCtrl is NULL .. Exiting",
                          "rawchip_proc_get_params");
    }
  }
  else {
    iVar1 = *param_2;
    if (iVar1 == 1) {
      uVar3 = rawchip_get_af_roi_region_num(param_2 + 1);
      return uVar3;
    }
    if (iVar1 == 0) {
      uVar3 = rawchip_get_af_active(param_2 + 1);
      return uVar3;
    }
    if (iVar1 == 2) {
      uVar3 = rawchip_get_af_default_roi_region(param_2 + 1);
      return uVar3;
    }
    uVar2 = (uint)DAT_00019160;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if ((int)(uVar2 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC","Invalid RAWCHIP_PROC Get Param Type");
    }
  }
  return 0xffffffff;
}



/* ============================================= */
/* Function: rawchip_proc_set_params */
/* Address: 0x00015e4c */
/* ============================================= */

undefined4 rawchip_proc_set_params(undefined4 param_1,undefined4 *param_2,undefined4 param_3)

{
  undefined1 uVar1;
  int iVar2;
  uint uVar3;
  undefined4 uVar4;
  int iVar5;
  int iVar6;
  int iVar7;
  undefined4 *puVar8;
  
  puVar8 = param_2;
  iVar2 = FUN_00015b60();
  if (iVar2 == 0) {
    uVar3 = (uint)DAT_00019160;
    if ((int)(uVar3 << 0x18) < 0) {
      uVar3 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019160,uVar3 << 0x18,
                                 param_1,puVar8,param_3);
    }
    if ((int)(uVar3 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC"," %s: rawchipCtrl is NULL .. Exiting",
                          "rawchip_proc_set_params");
    }
LAB_00016002:
    uVar4 = 0xffffffff;
  }
  else {
    switch(*param_2) {
    case 0:
      uVar4 = rawchip_set_af_roi_info(param_2[1],param_2[2]);
      return uVar4;
    case 1:
      uVar4 = rawchip_set_aec_exp_info(param_2[1],param_2[2],param_2[3],param_2[4],param_2[5]);
      break;
    case 2:
      uVar4 = rawchip_set_awb_gain_info(param_2[1]);
      return uVar4;
    case 3:
      uVar4 = rawchip_set_af_behavior(param_2[1]);
      return uVar4;
    case 4:
      uVar4 = rawchip_set_af_mode(param_2[1]);
      return uVar4;
    case 5:
      FocusValue = param_2[1];
      uVar4 = rawchip_set_af_info_for_af();
      return uVar4;
    case 6:
      uVar4 = rawchip_set_aec_info_for_af(param_2[1],param_2[2],param_2[3]);
      return uVar4;
    case 7:
      uVar4 = rawchip_set_gsensor_info_for_af(param_2[1]);
      return uVar4;
    case 8:
      uVar4 = 0;
      if (useDxOAF != '\0') {
        iVar5 = ((uint)(ushort)dxoframeSetting._8_2_ - (uint)(ushort)dxoframeSetting._4_2_) + 1;
        iVar2 = ((uint)(ushort)dxoframeSetting._8_2_ - (uint)(ushort)dxoframeSetting._4_2_) + 0x10;
        if (-1 < iVar5) {
          iVar2 = iVar5;
        }
        iVar5 = __aeabi_idiv(iVar2 >> 4,
                             (int)((uint)(ushort)dxoframeSetting._14_2_ +
                                  (uint)(ushort)dxoframeSetting._16_2_) >> 1);
        iVar6 = ((uint)(ushort)dxoframeSetting._6_2_ - (uint)(ushort)dxoframeSetting._2_2_) + 1;
        iVar2 = ((uint)(ushort)dxoframeSetting._6_2_ - (uint)(ushort)dxoframeSetting._2_2_) + 0x10;
        if (-1 < iVar6) {
          iVar2 = iVar6;
        }
        iVar6 = 0;
        iVar2 = __aeabi_idiv(iVar2 >> 4,
                             (int)((uint)(ushort)dxoframeSetting._10_2_ +
                                  (uint)(ushort)dxoframeSetting._12_2_) >> 1);
        do {
          iVar7 = 0;
          do {
            uVar1 = __aeabi_uidiv(*(undefined4 *)(param_2[1] + (iVar7 + iVar6) * 4),iVar2 * iVar5);
            lumTable[iVar7 + iVar6] = uVar1;
            iVar7 = iVar7 + 1;
          } while (iVar7 != 0x10);
          iVar6 = iVar6 + 0x10;
        } while (iVar6 != 0x100);
        uVar4 = 0;
      }
      break;
    case 9:
      uVar4 = rawchip_set_bestshot_mode(param_2[1]);
      return uVar4;
    case 10:
      uVar4 = rawchip_set_sensor_info(param_2[1]);
      return uVar4;
    default:
      uVar3 = (uint)DAT_00019160;
      if ((int)(uVar3 << 0x18) < 0) {
        uVar3 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019160,*param_2,param_1,
                                   puVar8,param_3);
      }
      if ((int)(uVar3 << 0x1b) < 0) {
        __android_log_print(6,"CAMERA RAWCHIP_PROC","Invalid RAWCHIP_PROC Set Param Type");
      }
      goto LAB_00016002;
    }
  }
  return uVar4;
}



/* ============================================= */
/* Function: rawchip_proc_process */
/* Address: 0x00016040 */
/* ============================================= */

undefined4
rawchip_proc_process(undefined4 param_1,undefined4 *param_2,undefined4 param_3,undefined4 param_4)

{
  int iVar1;
  uint uVar2;
  undefined4 uVar3;
  
  iVar1 = FUN_00015b60();
  if (iVar1 == 0) {
    uVar2 = (uint)DAT_00019160;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff,&DAT_00019160,uVar2 << 0x18,
                                 param_4);
    }
    if ((int)(uVar2 << 0x1b) < 0) {
      __android_log_print(6,"CAMERA RAWCHIP_PROC"," %s: rawchipCtrl is NULL .. Exiting",
                          "rawchip_proc_process");
    }
  }
  else {
    memcpy((void *)(iVar1 + 4),param_2 + 2,0x28);
    switch(*param_2) {
    case 0:
      uVar3 = rawchip_thread_launch(iVar1);
      return uVar3;
    case 1:
      uVar3 = rawchip_thread_release();
      return uVar3;
    case 2:
      uVar3 = rawchip_once_af_start(iVar1);
      return uVar3;
    case 3:
      uVar3 = rawchip_cancel_af(iVar1);
      return uVar3;
    case 4:
      uVar3 = rawchip_update_3A_params(iVar1);
      param_2[3] = *(undefined4 *)(iVar1 + 8);
      return uVar3;
    case 5:
      uVar3 = rawchip_through_focus(iVar1);
      return uVar3;
    }
    uVar2 = (uint)DAT_00019160;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("CAMERA RAWCHIP_PROC",0xffffffff);
    }
    if (-1 < (int)(uVar2 << 0x1b)) {
      return 0xffffffff;
    }
    __android_log_print(6,"CAMERA RAWCHIP_PROC","Invalid RAWCHIP_PROC Process Type");
  }
  return 0xffffffff;
}



/* ============================================= */
/* Function: get_current_product */
/* Address: 0x00016130 */
/* ============================================= */

void get_current_product(void)

{
  size_t __n;
  int iVar1;
  int iVar2;
  char acStack_3c [32];
  int local_1c;
  
  local_1c = __stack_chk_guard;
  if (DAT_00019170 == 0x19) {
    iVar2 = 0;
    __system_property_get("ro.product.device",acStack_3c);
    __n = strlen(acStack_3c);
    do {
      iVar1 = strncmp(acStack_3c,(char *)(&DAT_00018d48)[iVar2 * 2],__n);
      if (iVar1 == 0) {
        DAT_00019170 = (&DAT_00018d4c)[iVar2 * 2];
        break;
      }
      iVar2 = iVar2 + 1;
    } while (iVar2 != 0x19);
  }
  if (local_1c == __stack_chk_guard) {
    return;
  }
                    /* WARNING: Subroutine does not return */
  __stack_chk_fail(DAT_00019170);
}



/* ============================================= */
/* Function: get_current_product_name */
/* Address: 0x000161b4 */
/* ============================================= */

void get_current_product_name(char *param_1)

{
  size_t sVar1;
  int iVar2;
  int iVar3;
  char acStack_3c [32];
  int local_1c;
  
  iVar3 = 0;
  local_1c = __stack_chk_guard;
  __system_property_get("ro.product.device",acStack_3c);
  sVar1 = strlen(acStack_3c);
  do {
    iVar2 = strncmp(acStack_3c,*(char **)((int)&DAT_00018d48 + iVar3),sVar1);
    if (iVar2 == 0) {
      strncpy(param_1,acStack_3c,0x1e);
      sVar1 = strlen(param_1);
      goto LAB_00016204;
    }
    iVar3 = iVar3 + 8;
  } while (iVar3 != 200);
  sVar1 = 0;
LAB_00016204:
  if (local_1c != __stack_chk_guard) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail(sVar1);
  }
  return;
}



/* ============================================= */
/* Function: get_mfg_bootmode */
/* Address: 0x00016224 */
/* ============================================= */

void get_mfg_bootmode(void)

{
  size_t __n;
  int iVar1;
  uint uVar2;
  undefined4 uVar3;
  char acStack_2c [32];
  int local_c;
  
  local_c = __stack_chk_guard;
  __system_property_get("ro.bootmode",acStack_2c);
  __n = strlen(acStack_2c);
  iVar1 = strncmp(acStack_2c,"factory2",__n);
  if (iVar1 == 0) {
    uVar2 = (uint)DAT_0001916c;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("mm-camera-prodcut",0xffffffff);
    }
    if ((int)(uVar2 << 0x1e) < 0) {
      __android_log_print(3,"mm-camera-prodcut","mfg_build %d",1);
    }
    uVar3 = 1;
  }
  else {
    uVar3 = 0;
  }
  if (local_c != __stack_chk_guard) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail(uVar3);
  }
  return;
}



/* ============================================= */
/* Function: get_lens_type */
/* Address: 0x000162b0 */
/* ============================================= */

void get_lens_type(void)

{
  FILE *__stream;
  uint uVar1;
  int iVar2;
  char acStack_48 [52];
  int local_14;
  
  local_14 = __stack_chk_guard;
  memset(acStack_48,0,0x32);
  if (DAT_0001b344 < 1) {
    __stream = fopen("/sys/android_camera/lensinfo","r");
    if (__stream == (FILE *)0x0) {
      uVar1 = (uint)DAT_0001916c;
      if ((int)(uVar1 << 0x18) < 0) {
        uVar1 = __htclog_init_mask("mm-camera-prodcut",0xffffffff);
      }
      if ((int)(uVar1 << 0x1d) < 0) {
        __android_log_print(4,"mm-camera-prodcut","get_lens_type() : can\'t find lensinfo\n");
      }
      DAT_0001b344 = 2;
    }
    else {
      fread(acStack_48,1,0x32,__stream);
      iVar2 = atoi(acStack_48);
      uVar1 = (uint)DAT_0001916c;
      if ((int)(uVar1 << 0x18) < 0) {
        uVar1 = __htclog_init_mask("mm-camera-prodcut",0xffffffff);
      }
      if ((int)(uVar1 << 0x1d) < 0) {
        __android_log_print(4,"mm-camera-prodcut","get_lens_type() : lens_info=%d\n",iVar2);
      }
      if (iVar2 - 1U < 6) {
        DAT_0001b344 = iVar2;
      }
      uVar1 = (uint)DAT_0001916c;
      if ((int)(uVar1 << 0x18) < 0) {
        uVar1 = __htclog_init_mask("mm-camera-prodcut",0xffffffff);
      }
      if ((int)(uVar1 << 0x1d) < 0) {
        __android_log_print(4,"mm-camera-prodcut","get_lens_type() : lens_type=%d\n",DAT_0001b344);
      }
      fclose(__stream);
    }
  }
  if (local_14 == __stack_chk_guard) {
    return;
  }
                    /* WARNING: Subroutine does not return */
  __stack_chk_fail(DAT_0001b344);
}



/* ============================================= */
/* Function: valid_htccb_index */
/* Address: 0x00016404 */
/* ============================================= */

undefined8 valid_htccb_index(uint param_1,undefined4 param_2,undefined4 param_3)

{
  bool bVar1;
  uint uVar2;
  uint uVar3;
  
  bVar1 = param_1 < 9;
  uVar3 = param_1;
  if (bVar1 == 0) {
    uVar2 = (uint)DAT_00019174;
    if ((int)(uVar2 << 0x18) < 0) {
      uVar2 = __htclog_init_mask("htccallback_rawchip",0xffffffff);
    }
    if ((int)(uVar2 << 0x1b) < 0) {
      __android_log_print(6,"htccallback_rawchip",
                          "%s: invalid htccb_status index %d! should be in [0, %d]",
                          "valid_htccb_index",param_1,8,param_3);
      uVar3 = param_1;
    }
  }
  return CONCAT44(uVar3,(uint)bVar1);
}



/* ============================================= */
/* Function: do_htccb */
/* Address: 0x00016460 */
/* ============================================= */

void do_htccb(undefined4 param_1,undefined4 param_2,char *param_3)

{
  uint uVar1;
  int *piVar2;
  char *pcVar3;
  size_t sVar4;
  char acStack_3c [32];
  int local_1c;
  
  local_1c = __stack_chk_guard;
  memset(acStack_3c,0,0x1e);
  sprintf(acStack_3c,"%d0%d%d\n",param_1,param_2,param_3);
  DAT_0001b348 = fopen("/sys/camera_htccallback/htccallback","wb");
  if (DAT_0001b348 == (FILE *)0x0) {
    uVar1 = (uint)DAT_00019174;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("htccallback_rawchip",0xffffffff);
    }
    if ((int)(uVar1 << 0x1b) < 0) {
      piVar2 = (int *)__errno();
      pcVar3 = strerror(*piVar2);
      __android_log_print(6,"htccallback_rawchip","fail to create socket, error [%s]",pcVar3);
    }
  }
  else {
    sVar4 = fwrite(acStack_3c,0x1e,1,DAT_0001b348);
    if (sVar4 == 0) {
      uVar1 = (uint)DAT_00019174;
      if ((int)(uVar1 << 0x18) < 0) {
        uVar1 = __htclog_init_mask("htccallback_rawchip",0xffffffff);
      }
      if ((int)(uVar1 << 0x1b) < 0) {
        piVar2 = (int *)__errno();
        param_3 = strerror(*piVar2);
        __android_log_print(6,"htccallback_rawchip","fail to write [%s] to socket, error [%s]",
                            acStack_3c,param_3);
      }
    }
    uVar1 = (uint)DAT_00019174;
    if ((int)(uVar1 << 0x18) < 0) {
      uVar1 = __htclog_init_mask("htccallback_rawchip",0xffffffff);
    }
    if ((int)(uVar1 << 0x1e) < 0) {
      __android_log_print(3,"htccallback_rawchip","[rawchip] write buf[%s] to socket",acStack_3c,
                          param_3);
    }
    fclose(DAT_0001b348);
  }
  if (local_1c != __stack_chk_guard) {
                    /* WARNING: Subroutine does not return */
    __stack_chk_fail();
  }
  return;
}



/* ============================================= */
/* Function: check_n_update_htccb_status */
/* Address: 0x000165b0 */
/* ============================================= */

char check_n_update_htccb_status(int param_1,int param_2)

{
  char cVar1;
  int iVar2;
  
  iVar2 = valid_htccb_index();
  cVar1 = '\0';
  if (iVar2 != 0) {
    if (param_2 == 0) {
      cVar1 = '\0' < (char)(&DAT_0001b34c)[param_1];
    }
    else if (param_2 < 1) {
      cVar1 = '\0';
    }
    else {
      cVar1 = '\x01' - (&DAT_0001b34c)[param_1];
      if (1 < (byte)(&DAT_0001b34c)[param_1]) {
        cVar1 = '\0';
      }
    }
    (&DAT_0001b34c)[param_1] = (char)param_2;
  }
  return cVar1;
}



/* ============================================= */
/* Function: __on_dlclose */
/* Address: 0x000165fc */
/* ============================================= */

void __on_dlclose(void)

{
  __cxa_finalize(&__dso_handle);
  return;
}



