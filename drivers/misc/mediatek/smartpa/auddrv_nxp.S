#include <asm/assembler.h>

#-----------------------------------
.globl tfa9890_read_proc
.type tfa9890_read_proc, @function
#-----------------------------------
tfa9890_read_proc:

            sub sp,sp,#176
            stp x28,x27,[sp,#80]
            stp x26,x25,[sp,#96]
            stp x24,x23,[sp,#112]
            stp x22,x21,[sp,#128]
            stp x20,x19,[sp,#144]
            stp x29,lr,[sp,#160]
            add x29,sp,#160
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov x20,x1
            mov w1,#192
            movk w1,#320,lsl #16
            str x8,[sp,#72]
            adrp x8, .L_ffffff8009ad56b8
            ldr x0,[x8,:lo12:.L_ffffff8009ad56b8]
            mov w2,#4096
            mov x19,x3
            strb wzr,[sp,#36]
            str wzr,[sp,#32]
            bl kmem_cache_alloc_trace

            cbz x0,.L_ffffff8008947d9c

            adrp x27, .L_ffffff800a0da9d8
            ldr x8,[x27,:lo12:.L_ffffff800a0da9d8]
            mov x24,x0
            adrp x0, .L_ffffff80092017df
            add x0,x0, :lo12:.L_ffffff80092017df
            ldrh w2,[x8,#2]
            mov w1,#83
            str x20,[sp]
            str x19,[sp,#16]
            bl printk

            adrp x21, .L_ffffff8008f4fec8
            mov w25,#1
            adrp x23, .L_ffffff8009201809
            mov x28,xzr
            add x21,x21, :lo12:.L_ffffff8008f4fec8
            mov w19,#65536
            add x20,sp,#28
            movk w25,#2,lsl #16
            add x22,sp,#32
            add x23,x23, :lo12:.L_ffffff8009201809
            str x24,[sp,#8]
.L_ffffff8008947c5c:

            ldrb w26,[x21,x28]
            ldr x8,[x27,:lo12:.L_ffffff800a0da9d8]
            stp xzr,xzr,[sp,#56]
            stp xzr,xzr,[sp,#40]
            strb w26,[sp,#28]
            ldrh w9,[x8,#2]
            stur w19,[sp,#42]
            str x20,[sp,#48]
            strh w9,[sp,#40]
            ldrh w9,[x8,#2]
            stur w25,[sp,#58]
            str x22,[sp,#64]
            strh w9,[sp,#56]
            cbz x8,.L_ffffff8008947ccc

            ldr x0,[x8,#24]
            add x1,sp,#40
            mov w2,#2
            bl i2c_transfer

            cmp w0,#2
            b.eq .L_ffffff8008947cdc

            ldr x8,[x27,:lo12:.L_ffffff800a0da9d8]
            ldrh w3,[sp,#58]
            adrp x0, .L_ffffff8009201872
            mov w1,#248
            ldrh w2,[x8,#2]
            add x0,x0, :lo12:.L_ffffff8009201872
            bl printk

            b .L_ffffff8008947cdc
.L_ffffff8008947ccc:

            adrp x0, .L_ffffff8009201851
            mov w1,#242
            add x0,x0, :lo12:.L_ffffff8009201851
            bl printk
.L_ffffff8008947cdc:

            ldrb w8,[sp,#32]
            ldrb w3,[sp,#33]
            mov x0,x24
            mov x1,x23
            mov w2,w26
            bfi w3,w8,#8,#8
            bl sprintf

            add x28,x28,#1
            cmp x28,#23
            add x24,x24,w0, sxtw
            b.ne .L_ffffff8008947c5c

            ldp x20,x19,[sp,#8]
            ldr x8,[x19]
            sub w9,w24,w20
            sxtw x22,w9
            cmp x8,x22
            b.ge .L_ffffff8008947da8

            mov w2,#1
            mov x0,x20
            mov x1,x22
            bl __check_object_size

            ldr x11,[sp]
            mrs x8,sp_el0
            ldr x9,[x8,#8]
            mov x23,x22
            mov x10,x11
            adds x10,x10,x22
            csel x9,xzr,x9,hi
            csinv x10,x10,xzr,lo
            sbcs xzr,x10,x9
            cset x10,ls
            cbz x10,.L_ffffff8008947d7c

            ldr x8,[x8,#8]
            bics xzr,x11,x8
            csel x0,x11,xzr,eq
            csdb
            mov x1,x20
            mov x2,x22
            bl __arch_copy_to_user

            mov x23,x0
.L_ffffff8008947d7c:

            ldr x8,[x19]
            mov x0,x20
            add x8,x8,x22
            str x8,[x19]
            bl kfree

            cbz w23,.L_ffffff8008947db4

            sxtw x22,w23
            b .L_ffffff8008947db4
.L_ffffff8008947d9c:

            bl kfree

            mov x22,#-12
            b .L_ffffff8008947db4
.L_ffffff8008947da8:

            mov x0,x20
            bl kfree

            mov x22,xzr
.L_ffffff8008947db4:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#72]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008947dec

            mov x0,x22
            ldp x29,lr,[sp,#160]
            ldp x20,x19,[sp,#144]
            ldp x22,x21,[sp,#128]
            ldp x24,x23,[sp,#112]
            ldp x26,x25,[sp,#96]
            ldp x28,x27,[sp,#80]
            add sp,sp,#176
            ret
.L_ffffff8008947dec:

            bl __stack_chk_fail
.size tfa9890_read_proc, . - tfa9890_read_proc
#-----------------------------------
.type NXPExtSpk_i2c_probe, @function
#-----------------------------------
NXPExtSpk_i2c_probe:

            stp x29,lr,[sp,#-16]!
            mov x29,sp
            adrp x8, .L_ffffff800a0da9d8
            str x0,[x8,:lo12:.L_ffffff800a0da9d8]
            adrp x0, .L_ffffff8009201823
            adrp x3, .L_ffffff8009931688
            add x0,x0, :lo12:.L_ffffff8009201823
            add x3,x3, :lo12:.L_ffffff8009931688
            mov w1,#436
            mov x2,xzr
            bl proc_create

            mov w0,wzr
            ldp x29,lr,[sp],#16
            ret
.size NXPExtSpk_i2c_probe, . - NXPExtSpk_i2c_probe
#-----------------------------------
.type NXPExtSpk_i2c_remove, @function
#-----------------------------------
NXPExtSpk_i2c_remove:

            stp x29,lr,[sp,#-16]!
            mov x29,sp
            adrp x8, .L_ffffff800a0da9d8
            str xzr,[x8,:lo12:.L_ffffff800a0da9d8]
            bl i2c_unregister_device

            adrp x0, .L_ffffff80099315a0
            add x0,x0, :lo12:.L_ffffff80099315a0
            bl i2c_del_driver

            mov w0,wzr
            ldp x29,lr,[sp],#16
            ret
.size NXPExtSpk_i2c_remove, . - NXPExtSpk_i2c_remove
#-----------------------------------
.type tfa9890_write_proc, @function
#-----------------------------------
tfa9890_write_proc:

            sub sp,sp,#160
            str x21,[sp,#112]
            stp x20,x19,[sp,#128]
            stp x29,lr,[sp,#144]
            add x29,sp,#144
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            cmp x2,#63
            stur x8,[x29,#-40]
            mov w8,#63
            csel x20,x2,x8,lo
            cmp x20,#65
            stp xzr,xzr,[sp,#88]
            stp xzr,xzr,[sp,#72]
            stp xzr,xzr,[sp,#56]
            stp xzr,xzr,[sp,#40]
            str xzr,[sp,#8]
            strb wzr,[sp,#38]
            strh wzr,[sp,#36]
            b.hs .L_ffffff8008947fa0

            mov x19,x2
            mov x21,x1
            add x0,sp,#40
            mov x1,x20
            mov w2,wzr
            bl __check_object_size

            mrs x8,sp_el0
            ldr x9,[x8,#8]
            mov x10,x21
            adds x10,x10,x20
            csel x9,xzr,x9,hi
            csinv x10,x10,xzr,lo
            sbcs xzr,x10,x9
            cset x10,ls
            cbz x10,.L_ffffff8008947fc4

            ldr x8,[x8,#8]
            bics xzr,x21,x8
            csel x1,x21,xzr,eq
            csdb
            add x0,sp,#40
            mov x2,x20
            bl __arch_copy_from_user

            mov x2,x0
            cbnz x2,.L_ffffff8008947fcc
.L_ffffff8008947f04:

            adrp x1, .L_ffffff800925dd7d
            add x1,x1, :lo12:.L_ffffff800925dd7d
            add x0,sp,#40
            add x2,sp,#12
            add x3,sp,#8
            bl sscanf

            cmp w0,#2
            b.ne .L_ffffff8008947f74

            ldp w9,w8,[sp,#8]
            adrp x10, .L_ffffff800a0da9d8
            ldr x10,[x10,:lo12:.L_ffffff800a0da9d8]
            add x1,sp,#16
            strb w8,[sp,#36]
            lsr w8,w9,#8
            strb w9,[sp,#38]
            strb w8,[sp,#37]
            ldrh w8,[x10,#2]
            ldr x0,[x10,#24]
            mov w9,#3
            mov w2,#1
            strh w8,[sp,#16]
            ldrh w8,[x10]
            strh w9,[sp,#20]
            add x9,sp,#36
            str x9,[sp,#24]
            and w8,w8,#16
            strh w8,[sp,#18]
            bl i2c_transfer
.L_ffffff8008947f74:

            adrp x9, .L_ffffff800984ea88
            ldur x8,[x29,#-40]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008947ff8

            mov x0,x19
            ldp x29,lr,[sp,#144]
            ldp x20,x19,[sp,#128]
            ldr x21,[sp,#112]
            add sp,sp,#160
            ret
.L_ffffff8008947fa0:

            adrp x0, .L_ffffff80090c2574
            adrp x2, .L_ffffff80090c25c0
            add x0,x0, :lo12:.L_ffffff80090c2574
            add x2,x2, :lo12:.L_ffffff80090c25c0
            mov w1,#123
            mov w3,#64
            mov x4,x20
            bl warn_slowpath_fmt

            b .L_ffffff8008947fe0
.L_ffffff8008947fc4:

            mov x2,x20
            cbz x2,.L_ffffff8008947f04
.L_ffffff8008947fcc:

            sub x8,x20,x2
            add x9,sp,#40
            add x0,x9,x8
            mov w1,wzr
            bl __pi_memset
.L_ffffff8008947fe0:

            adrp x0, .L_ffffff800920182b
            add x0,x0, :lo12:.L_ffffff800920182b
            mov w1,#122
            bl printk

            mov x19,#-14
            b .L_ffffff8008947f74
.L_ffffff8008947ff8:

            bl __stack_chk_fail
.size tfa9890_write_proc, . - tfa9890_write_proc
#-----------------------------------
.type AudDrv_nxpspk_probe, @function
#-----------------------------------
AudDrv_nxpspk_probe:

            stp x29,lr,[sp,#-16]!
            mov x29,sp
            adrp x1, .L_ffffff80099315a0
            add x1,x1, :lo12:.L_ffffff80099315a0
            mov x0,xzr
            bl i2c_register_driver

            cbz w0,.L_ffffff800894802c

            adrp x0, .L_ffffff8009201923
            add x0,x0, :lo12:.L_ffffff8009201923
            mov w1,#288
            bl printk

            mov w0,#-1
.L_ffffff800894802c:

            ldp x29,lr,[sp],#16
            ret
.size AudDrv_nxpspk_probe, . - AudDrv_nxpspk_probe
#-----------------------------------
.type AudDrv_nxpspk_read, @function
#-----------------------------------
AudDrv_nxpspk_read:

            sub sp,sp,#80
            stp x22,x21,[sp,#32]
            stp x20,x19,[sp,#48]
            stp x29,lr,[sp,#64]
            add x29,sp,#64
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            cmp x2,#2,lsl #12
            mov w9,#8192
            mov x21,x1
            csel x20,x2,x9,lo
            mov w1,#192
            movk w1,#320,lsl #16
            mov x0,x20
            str x8,[sp,#24]
            bl __kmalloc

            cbz x0,.L_ffffff8008948130

            adrp x8, .L_ffffff800a0da9d8
            ldr x8,[x8,:lo12:.L_ffffff800a0da9d8]
            mov x19,x0
            add x1,sp,#8
            mov w2,#1
            ldrh w9,[x8]
            ldr x0,[x8,#24]
            strh w20,[sp,#12]
            str x19,[sp,#16]
            and w9,w9,#16
            orr w9,w9,#1
            strh w9,[sp,#10]
            ldrh w8,[x8,#2]
            strh w8,[sp,#8]
            bl i2c_transfer

            cmp w0,#1
            csel w22,w20,w0,eq
            tbnz w22,#31,.L_ffffff8008948120

            mov w2,#1
            mov x0,x19
            mov x1,x20
            bl __check_object_size

            mrs x8,sp_el0
            ldr x9,[x8,#8]
            mov x10,x21
            adds x10,x10,x20
            csel x9,xzr,x9,hi
            csinv x10,x10,xzr,lo
            sbcs xzr,x10,x9
            cset x10,ls
            cbz x10,.L_ffffff8008948114

            ldr x8,[x8,#8]
            bics xzr,x21,x8
            csel x0,x21,xzr,eq
            csdb
            mov x1,x19
            mov x2,x20
            bl __arch_copy_to_user

            mov x20,x0
.L_ffffff8008948114:

            cmp x20,#0
            mov w8,#-14
            csel w22,w22,w8,eq
.L_ffffff8008948120:

            mov x0,x19
            bl kfree

            sxtw x0,w22
            b .L_ffffff8008948134
.L_ffffff8008948130:

            mov x0,#-12
.L_ffffff8008948134:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#24]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff800894815c

            ldp x29,lr,[sp,#64]
            ldp x20,x19,[sp,#48]
            ldp x22,x21,[sp,#32]
            add sp,sp,#80
            ret
.L_ffffff800894815c:

            bl __stack_chk_fail
.size AudDrv_nxpspk_read, . - AudDrv_nxpspk_read
#-----------------------------------
.type AudDrv_nxpspk_write, @function
#-----------------------------------
AudDrv_nxpspk_write:

            sub sp,sp,#80
            str x21,[sp,#32]
            stp x20,x19,[sp,#48]
            stp x29,lr,[sp,#64]
            add x29,sp,#64
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov x21,x1
            mov w1,#192
            movk w1,#320,lsl #16
            mov x0,x2
            mov x20,x2
            str x8,[sp,#24]
            bl __kmalloc

            cbz x0,.L_ffffff8008948244

            mov x1,x20
            mov w2,wzr
            mov x19,x0
            bl __check_object_size

            mrs x8,sp_el0
            ldr x9,[x8,#8]
            mov x10,x21
            mov x2,x20
            adds x10,x10,x20
            csel x9,xzr,x9,hi
            csinv x10,x10,xzr,lo
            sbcs xzr,x10,x9
            cset x10,ls
            cbz x10,.L_ffffff80089481f4

            ldr x8,[x8,#8]
            bics xzr,x21,x8
            csel x1,x21,xzr,eq
            csdb
            mov x0,x19
            mov x2,x20
            bl __arch_copy_from_user

            mov x2,x0
.L_ffffff80089481f4:

            cbnz x2,.L_ffffff8008948270

            adrp x8, .L_ffffff800a0da9d8
            ldr x8,[x8,:lo12:.L_ffffff800a0da9d8]
            add x1,sp,#8
            mov w2,#1
            ldrh w9,[x8,#2]
            ldr x0,[x8,#24]
            strh w9,[sp,#8]
            ldrh w8,[x8]
            strh w20,[sp,#12]
            str x19,[sp,#16]
            and w8,w8,#16
            strh w8,[sp,#10]
            bl i2c_transfer

            cmp w0,#1
            csel w20,w20,w0,eq
            mov x0,x19
            bl kfree

            sxtw x0,w20
            b .L_ffffff8008948248
.L_ffffff8008948244:

            mov x0,#-12
.L_ffffff8008948248:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#24]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008948290

            ldp x29,lr,[sp,#64]
            ldp x20,x19,[sp,#48]
            ldr x21,[sp,#32]
            add sp,sp,#80
            ret
.L_ffffff8008948270:

            sub x8,x20,x2
            add x0,x19,x8
            mov w1,wzr
            bl __pi_memset

            mov x0,x19
            bl kfree

            mov x0,#-14
            b .L_ffffff8008948248
.L_ffffff8008948290:

            bl __stack_chk_fail
.size AudDrv_nxpspk_write, . - AudDrv_nxpspk_write
#-----------------------------------
.type AudDrv_nxpspk_ioctl, @function
#-----------------------------------
AudDrv_nxpspk_ioctl:

            cmp w1,#1798
            b.ne .L_ffffff80089482b4

            and x8,x2,#-4
            cmp x8,#52
            b.ne .L_ffffff80089482b4

            adrp x8, .L_ffffff800a0da9d8
            ldr x8,[x8,:lo12:.L_ffffff800a0da9d8]
            strh w2,[x8,#2]
.L_ffffff80089482b4:

            mov x0,xzr
            ret
.size AudDrv_nxpspk_ioctl, . - AudDrv_nxpspk_ioctl
#-----------------------------------
.type AudDrv_nxpspk_open, @function
#-----------------------------------
AudDrv_nxpspk_open:

            stp x29,lr,[sp,#-16]!
            mov x29,sp
            bl nonseekable_open

            ldp x29,lr,[sp],#16
            ret
.size AudDrv_nxpspk_open, . - AudDrv_nxpspk_open

#-----------------------------------
.type AudDrv_nxpspk_mod_init, @function
.globl AudDrv_nxpspk_mod_init
#-----------------------------------
AudDrv_nxpspk_mod_init:

            stp x20,x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            adrp x0, .L_ffffff80092018a0
            add x0,x0, :lo12:.L_ffffff80092018a0
            mov w1,#-1
            bl platform_device_alloc

            adrp x20, .L_ffffff800a0da9e0
            str x0,[x20,:lo12:.L_ffffff800a0da9e0]
            cbz x0,.L_ffffff8009360e48

            bl platform_device_add

            cbz w0,.L_ffffff8009360e50

            mov w19,w0
            ldr x0,[x20,:lo12:.L_ffffff800a0da9e0]
            bl platform_device_put

            b .L_ffffff8009360ea0
.L_ffffff8009360e48:

            mov w19,#-12
            b .L_ffffff8009360ea0
.L_ffffff8009360e50:

            adrp x0, .L_ffffff8009931778
            add x0,x0, :lo12:.L_ffffff8009931778
            mov x1,xzr
            bl __platform_driver_register

            cbz w0,.L_ffffff8009360e78

            mov w19,w0
            adrp x0, .L_ffffff80092018af
            add x0,x0, :lo12:.L_ffffff80092018af
            mov w1,#426
            b .L_ffffff8009360e98
.L_ffffff8009360e78:

            adrp x0, .L_ffffff8009931828
            add x0,x0, :lo12:.L_ffffff8009931828
            bl misc_register

            mov w19,w0
            cbz w0,.L_ffffff8009360ea0

            adrp x0, .L_ffffff80092018e3
            add x0,x0, :lo12:.L_ffffff80092018e3
            mov w1,#433
.L_ffffff8009360e98:

            mov w2,w19
            bl printk
.L_ffffff8009360ea0:

            ldp x29,lr,[sp,#16]
            mov w0,w19
            ldp x20,x19,[sp],#32
            ret
.size AudDrv_nxpspk_mod_init, . - AudDrv_nxpspk_mod_init

.pushsection ".smartpa.data.write", "aw"

.align 8
.L_ffffff8008f4fec8:
          .byte 0x0
          .byte 0x1
          .byte 0x2
          .byte 0x3
          .byte 0x4
          .byte 0x5
          .byte 0x6
          .byte 0x7
          .byte 0x8
          .byte 0x9
          .byte 0xa
          .byte 0xb
          .byte 0xf
          .byte 0x47
          .byte 0x49
          .byte 0x62
          .byte 0x70
          .byte 0x71
          .byte 0x72
          .byte 0x73
          .byte 0x80
          .byte 0x81
          .byte 0x8f
          .byte 0x0
.L_ffffff8008f4fee0:
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x6d
          .byte 0x65
          .byte 0x64
          .byte 0x69
          .byte 0x61
          .byte 0x74
          .byte 0x65
          .byte 0x6b
          .byte 0x2c
          .byte 0x73
          .byte 0x70
          .byte 0x65
          .byte 0x61
          .byte 0x6b
          .byte 0x65
          .byte 0x72
          .byte 0x5f
          .byte 0x61
          .byte 0x6d
          .byte 0x70
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
.L_ffffff8008f50070:
          .string "TFA98XX"
          .zero 56
.align 8
.L_ffffff80090c2574:
          .string "/home3/work/repo/q_a61_release/alps/kernel-4.14/include/linux/thread_info.h"
.align 8
.L_ffffff80090c25c0:
          .string "Buffer overflow detected (%d < %lu)!\n"
.align 8
.L_ffffff80092017df:
          .byte 0x1
          .byte 0x33
          .byte 0x5b
          .byte 0x4e
          .byte 0x58
          .byte 0x50
          .byte 0x5d
          .byte 0x20
          .byte 0x5b
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x65
          .byte 0x3a
          .byte 0x25
          .byte 0x64
          .byte 0x5d
          .byte 0x6e
          .byte 0x65
          .byte 0x77
          .byte 0x5f
          .byte 0x63
          .byte 0x6c
          .byte 0x69
          .byte 0x65
          .byte 0x6e
          .byte 0x74
          .byte 0x2d
          .byte 0x3e
          .byte 0x61
          .byte 0x64
          .byte 0x64
          .byte 0x72
          .byte 0x3a
          .byte 0x30
          .byte 0x78
          .byte 0x25
          .byte 0x30
          .byte 0x34
          .byte 0x78
          .byte 0xa
          .byte 0x0
.align 8
.L_ffffff8009201809:
          .string "reg[0x%x]=0x%04x\n"
.align 8
.L_ffffff800920181b:
          .string "TFA98XX"
.L_ffffff8009201823:
          .string "tfa9890"
.align 8
.L_ffffff800920182b:
          .byte 0x1
          .byte 0x33
          .byte 0x5b
          .byte 0x4e
          .byte 0x58
          .byte 0x50
          .byte 0x5d
          .byte 0x20
          .byte 0x5b
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x65
          .byte 0x3a
          .byte 0x25
          .byte 0x64
          .byte 0x5d
          .byte 0x63
          .byte 0x6f
          .byte 0x70
          .byte 0x79
          .byte 0x5f
          .byte 0x66
          .byte 0x72
          .byte 0x6f
          .byte 0x6d
          .byte 0x5f
          .byte 0x75
          .byte 0x73
          .byte 0x65
          .byte 0x72
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0xa
          .byte 0x0
.align 8
.L_ffffff8009201851:
          .byte 0x1
          .byte 0x33
          .byte 0x5b
          .byte 0x4e
          .byte 0x58
          .byte 0x50
          .byte 0x5d
          .byte 0x20
          .byte 0x5b
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x65
          .byte 0x3a
          .byte 0x25
          .byte 0x64
          .byte 0x5d
          .byte 0x63
          .byte 0x6c
          .byte 0x69
          .byte 0x65
          .byte 0x6e
          .byte 0x74
          .byte 0x20
          .byte 0x69
          .byte 0x73
          .byte 0x20
          .byte 0x6e
          .byte 0x75
          .byte 0x6c
          .byte 0x6c
          .byte 0x21
          .byte 0x0
.align 8
.L_ffffff8009201872:
          .byte 0x1
          .byte 0x33
          .byte 0x5b
          .byte 0x4e
          .byte 0x58
          .byte 0x50
          .byte 0x5d
          .byte 0x20
          .byte 0x5b
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x65
          .byte 0x3a
          .byte 0x25
          .byte 0x64
          .byte 0x5d
          .byte 0x69
          .byte 0x32
          .byte 0x63
          .byte 0x5f
          .byte 0x74
          .byte 0x72
          .byte 0x61
          .byte 0x6e
          .byte 0x73
          .byte 0x66
          .byte 0x65
          .byte 0x72
          .byte 0x20
          .byte 0x65
          .byte 0x72
          .byte 0x72
          .byte 0x6f
          .byte 0x72
          .byte 0x3a
          .byte 0x20
          .byte 0x28
          .byte 0x25
          .byte 0x78
          .byte 0x20
          .byte 0x25
          .byte 0x78
          .byte 0x29
          .byte 0xa
          .byte 0x0
.align 8
.L_ffffff80092018a0:
          .string "AudioMTKNXPSPK"
.align 8
.L_ffffff80092018af:
          .byte 0x1
          .byte 0x33
          .byte 0x5b
          .byte 0x4e
          .byte 0x58
          .byte 0x50
          .byte 0x5d
          .byte 0x20
          .byte 0x5b
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x65
          .byte 0x3a
          .byte 0x25
          .byte 0x64
          .byte 0x5d
          .byte 0x41
          .byte 0x75
          .byte 0x64
          .byte 0x44
          .byte 0x72
          .byte 0x76
          .byte 0x20
          .byte 0x46
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x3a
          .byte 0x25
          .byte 0x64
          .byte 0x20
          .byte 0x2d
          .byte 0x20
          .byte 0x52
          .byte 0x65
          .byte 0x67
          .byte 0x69
          .byte 0x73
          .byte 0x74
          .byte 0x65
          .byte 0x72
          .byte 0x20
          .byte 0x44
          .byte 0x52
          .byte 0x49
          .byte 0x56
          .byte 0x45
          .byte 0x52
          .byte 0x20
          .byte 0xa
          .byte 0x0
.align 8
.L_ffffff80092018e3:
          .byte 0x1
          .byte 0x33
          .byte 0x5b
          .byte 0x4e
          .byte 0x58
          .byte 0x50
          .byte 0x5d
          .byte 0x20
          .byte 0x5b
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x65
          .byte 0x3a
          .byte 0x25
          .byte 0x64
          .byte 0x5d
          .byte 0x41
          .byte 0x75
          .byte 0x64
          .byte 0x44
          .byte 0x72
          .byte 0x76
          .byte 0x5f
          .byte 0x6e
          .byte 0x78
          .byte 0x70
          .byte 0x73
          .byte 0x70
          .byte 0x6b
          .byte 0x5f
          .byte 0x6d
          .byte 0x6f
          .byte 0x64
          .byte 0x5f
          .byte 0x69
          .byte 0x6e
          .byte 0x69
          .byte 0x74
          .byte 0x20
          .byte 0x6d
          .byte 0x69
          .byte 0x73
          .byte 0x63
          .byte 0x5f
          .byte 0x72
          .byte 0x65
          .byte 0x67
          .byte 0x69
          .byte 0x73
          .byte 0x74
          .byte 0x65
          .byte 0x72
          .byte 0x20
          .byte 0x46
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x3a
          .byte 0x25
          .byte 0x64
          .byte 0x20
          .byte 0xa
          .byte 0x0
.align 8
.L_ffffff8009201923:
          .byte 0x1
          .byte 0x33
          .byte 0x5b
          .byte 0x4e
          .byte 0x58
          .byte 0x50
          .byte 0x5d
          .byte 0x20
          .byte 0x5b
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x65
          .byte 0x3a
          .byte 0x25
          .byte 0x64
          .byte 0x5d
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x20
          .byte 0x74
          .byte 0x6f
          .byte 0x20
          .byte 0x61
          .byte 0x64
          .byte 0x64
          .byte 0x20
          .byte 0x64
          .byte 0x65
          .byte 0x76
          .byte 0x69
          .byte 0x63
          .byte 0x65
          .byte 0x20
          .byte 0x69
          .byte 0x6e
          .byte 0x74
          .byte 0x6f
          .byte 0x20
.L_ffffff800920194c:
          .string "i2c"
.L_ffffff8009201950:
          .string "smartpa_i2c"
.align 8
.L_ffffff800925dd7d:
          .string "0x%x 0x%x"
.align 8
.L_ffffff800984ea88:
          .zero 8
.align 8
.L_ffffff80099315a0:
          .zero 16
.L_ffffff80099315b0:
          .quad NXPExtSpk_i2c_probe
.L_ffffff80099315b8:
          .quad NXPExtSpk_i2c_remove
          .zero 32
.L_ffffff80099315e0:
          .quad .L_ffffff800920181b
          .zero 32
.L_ffffff8009931608:
          .quad .L_ffffff8008f4fee0
          .zero 72
.L_ffffff8009931658:
          .quad .L_ffffff8008f50070
          .zero 40
.L_ffffff8009931688:
          .zero 16
.L_ffffff8009931698:
          .quad tfa9890_read_proc
.L_ffffff80099316a0:
          .quad tfa9890_write_proc
          .zero 208
.L_ffffff8009931778:
          .quad AudDrv_nxpspk_probe
          .zero 32
.L_ffffff80099317a0:
          .quad .L_ffffff8009931878
          .zero 128
.align 8
.L_ffffff8009931828:
          .byte 0xff
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
.L_ffffff8009931830:
          .quad .L_ffffff8009201950
.L_ffffff8009931838:
          .quad .L_ffffff8009931888
          .zero 56
.L_ffffff8009931878:
          .string "AudioMTKNXPSPK"
          .zero 1
.L_ffffff8009931888:
          .zero 16
.L_ffffff8009931898:
          .quad AudDrv_nxpspk_read
.L_ffffff80099318a0:
          .quad AudDrv_nxpspk_write
          .zero 40
.L_ffffff80099318d0:
          .quad AudDrv_nxpspk_ioctl
          .zero 16
.L_ffffff80099318e8:
          .quad AudDrv_nxpspk_open
          .zero 136
.align 8
.L_ffffff8009ad56b8:
          .zero 8
.align 8
.L_ffffff800a0da9d8:
          .zero 8
.align 8
.L_ffffff800a0da9e0:
          .zero 8

.popsection // smartpa.data.write
