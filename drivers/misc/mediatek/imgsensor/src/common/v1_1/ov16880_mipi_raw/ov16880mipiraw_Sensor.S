#include <asm/assembler.h>

#-----------------------------------
.globl OV16880_MIPI_RAW_SensorInit
.type OV16880_MIPI_RAW_SensorInit, @function
#-----------------------------------
OV16880_MIPI_RAW_SensorInit:

            cbz x0,.L_ffffff800870874c

            adrp x8, .L_ffffff8009907998
            add x8,x8, :lo12:.L_ffffff8009907998
            str x8,[x0]
.L_ffffff800870874c:

            mov w0,wzr
            ret
.size OV16880_MIPI_RAW_SensorInit, . - OV16880_MIPI_RAW_SensorInit
#-----------------------------------
.type FUN_ffffff8008708754, @function
#-----------------------------------
FUN_ffffff8008708754:
#-----------------------------------
.type open_disambig_0xffffff8008708754_0, @function
#-----------------------------------
open_disambig_0xffffff8008708754_0:

            sub sp,sp,#112
            stp x28,x27,[sp,#16]
            stp x26,x25,[sp,#32]
            stp x24,x23,[sp,#48]
            stp x22,x21,[sp,#64]
            stp x20,x19,[sp,#80]
            stp x29,lr,[sp,#96]
            add x29,sp,#96
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            adrp x19, .L_ffffff800916e0c8
            adrp x0, .L_ffffff8009184c59
            add x19,x19, :lo12:.L_ffffff800916e0c8
            mov w23,#26752
            add x0,x0, :lo12:.L_ffffff8009184c59
            mov x1,x19
            str x8,[sp,#8]
            movk w23,#1,lsl #16
            bl printk

            adrp x0, .L_ffffff8009184c8e
            add x0,x0, :lo12:.L_ffffff8009184c8e
            mov x1,x19
            bl printk

            adrp x24, .L_ffffff8008f29b3c
            adrp x20, .L_ffffff8009c7b64c
            adrp x21, .L_ffffff8009184d16
            mov w22,#108
            add x24,x24, :lo12:.L_ffffff8008f29b3c
            add x20,x20, :lo12:.L_ffffff8009c7b64c
            adrp x25, .L_ffffff8009907a30
            mov w26,#2608
            mov w27,#2864
            mov w28,#3120
            add x21,x21, :lo12:.L_ffffff8009184d16
.L_ffffff80087087dc:

            mov x0,x20
            bl _raw_spin_lock

            mov x0,x20
            strb w22,[x25,:lo12:.L_ffffff8009907a30]
            bl _raw_spin_unlock

            ldrb w4,[x25,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            strh wzr,[sp]
            strh w26,[sp,#4]
            bl iReadRegI2C

            ldrb w4,[x25,:lo12:.L_ffffff8009907a30]
            ldrh w22,[sp]
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            strh wzr,[sp]
            strh w27,[sp,#4]
            bl iReadRegI2C

            ldrh w8,[sp]
            ldrb w4,[x25,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov x2,sp
            lsl w8,w8,#8
            mov w1,#2
            mov w3,#1
            strh wzr,[sp]
            orr w22,w8,w22, lsl #16
            strh w28,[sp,#4]
            bl iReadRegI2C

            ldrh w8,[sp]
            ldrb w2,[x25,:lo12:.L_ffffff8009907a30]
            mov x0,x21
            mov x1,x19
            orr w22,w22,w8
            mov w3,w22
            bl printk

            ldrb w2,[x25,:lo12:.L_ffffff8009907a30]
            cmp w22,w23
            b.eq .L_ffffff8008708954

            mov x0,x21
            mov x1,x19
            mov w3,w22
            bl printk

            ldrb w4,[x25,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            strh wzr,[sp]
            strh w26,[sp,#4]
            bl iReadRegI2C

            ldrb w4,[x25,:lo12:.L_ffffff8009907a30]
            ldrh w22,[sp]
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            strh wzr,[sp]
            strh w27,[sp,#4]
            bl iReadRegI2C

            ldrh w8,[sp]
            ldrb w4,[x25,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov x2,sp
            lsl w8,w8,#8
            mov w1,#2
            mov w3,#1
            strh wzr,[sp]
            orr w22,w8,w22, lsl #16
            strh w28,[sp,#4]
            bl iReadRegI2C

            ldrh w8,[sp]
            ldrb w2,[x25,:lo12:.L_ffffff8009907a30]
            mov x0,x21
            mov x1,x19
            orr w22,w22,w8
            mov w3,w22
            bl printk

            ldrb w2,[x25,:lo12:.L_ffffff8009907a30]
            cmp w22,w23
            b.eq .L_ffffff8008708954

            mov x0,x21
            mov x1,x19
            mov w3,w22
            bl printk

            ldrb w22,[x24],#1
            cmp w22,#255
            b.ne .L_ffffff80087087dc

            mov w0,#16
            b .L_ffffff80087089dc
.L_ffffff8008708954:

            adrp x0, .L_ffffff8009184d66
            adrp x1, .L_ffffff800916e0c8
            mov w3,#26752
            add x0,x0, :lo12:.L_ffffff8009184d66
            add x1,x1, :lo12:.L_ffffff800916e0c8
            movk w3,#1,lsl #16
            bl printk

            bl sensor_init

            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            mov x0,x19
            bl _raw_spin_lock

            mov w10,#34816
            adrp x9, .L_ffffff8009c7b650
            movk w10,#4394,lsl #16
            str w10,[x9,:lo12:.L_ffffff8009c7b650]
            adrp x9, .L_ffffff8009c7b658
            mov w10,#2512
            str w10,[x9,:lo12:.L_ffffff8009c7b658]
            adrp x9, .L_ffffff8009c7b654
            mov w10,#3824
            adrp x8, .L_ffffff8009c7b662
            str w10,[x9,:lo12:.L_ffffff8009c7b654]
            adrp x9, .L_ffffff8009c7b65c
            str w10,[x9,:lo12:.L_ffffff8009c7b65c]
            adrp x9, .L_ffffff8009c7b660
            adrp x10, .L_ffffff8009907a2c
            strb wzr,[x8,:lo12:.L_ffffff8009c7b662]
            mov w8,#300
            mov x0,x19
            strh wzr,[x9,:lo12:.L_ffffff8009c7b660]
            strh w8,[x10,:lo12:.L_ffffff8009907a2c]
            bl _raw_spin_unlock

            mov w0,wzr
.L_ffffff80087089dc:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008708a10

            ldp x29,lr,[sp,#96]
            ldp x20,x19,[sp,#80]
            ldp x22,x21,[sp,#64]
            ldp x24,x23,[sp,#48]
            ldp x26,x25,[sp,#32]
            ldp x28,x27,[sp,#16]
            add sp,sp,#112
            ret
.L_ffffff8008708a10:

            bl __stack_chk_fail
.size open_disambig_0xffffff8008708754_0, . - open_disambig_0xffffff8008708754_0
.size FUN_ffffff8008708754, . - FUN_ffffff8008708754
#-----------------------------------
.type FUN_ffffff8008708a14, @function
#-----------------------------------
FUN_ffffff8008708a14:
#-----------------------------------
.type get_info_disambig_0xffffff8008708a14_0, @function
#-----------------------------------
get_info_disambig_0xffffff8008708a14_0:

            str x21,[sp,#-48]!
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            mov x19,x1
            mov w20,w0
            adrp x0, .L_ffffff8009184dc6
            adrp x1, .L_ffffff80091d8c1c
            add x0,x0, :lo12:.L_ffffff8009184dc6
            add x1,x1, :lo12:.L_ffffff80091d8c1c
            mov w2,w20
            bl printk

            mov w8,#257
            mov w9,#4
            mov x10,#21474836480
            sturh w8,[x19,#13]
            strh w8,[x19,#20]
            mov w8,#1
            strb w9,[x19,#22]
            mov x9,#12884901891
            stp x10,x8,[x19,#24]
            mov x8,#8589934594
            stur x9,[x19,#44]
            mov w9,#2
            stur x8,[x19,#52]
            mov w21,#512
            mov w8,#261
            str wzr,[x19,#156]
            str wzr,[x19,#168]
            str wzr,[x19,#136]
            strh wzr,[x19,#164]
            str w9,[x19,#60]
            strh w21,[x19,#140]
            sturh w8,[x19,#161]
            bl get_boot_mode

            mov w10,#515
            mov w8,#3
            strh w10,[x19,#18]
            mov x10,#26965
            mov w9,#24
            str w8,[x19,#40]
            strb w8,[x19,#17]
            mov w8,w20
            movk x10,#21865,lsl #16
            cmp w0,#4
            strb w9,[x19,#8]
            mov w9,#16777216
            movk x10,#85,lsl #32
            lsl x8,x8,#3
            stur w9,[x19,#150]
            cset w9,ne
            cmp w20,#5
            mov w11,#85
            lsr x8,x10,x8
            lsl w9,w9,#1
            csel w8,w8,w11,lo
            strb wzr,[x19,#173]
            sturh w21,[x19,#15]
            strb wzr,[x19,#143]
            strb w9,[x19,#172]
            str wzr,[x19,#92]
            strb w8,[x19,#144]
            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            mov w0,wzr
            ldr x21,[sp],#48
            ret
.size get_info_disambig_0xffffff8008708a14_0, . - get_info_disambig_0xffffff8008708a14_0
.size FUN_ffffff8008708a14, . - FUN_ffffff8008708a14
#-----------------------------------
.type FUN_ffffff8008708b20, @function
#-----------------------------------
FUN_ffffff8008708b20:
#-----------------------------------
.type get_resolution_disambig_0xffffff8008708b20_0, @function
#-----------------------------------
get_resolution_disambig_0xffffff8008708b20_0:

            str x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            mov x19,x0
            adrp x0, .L_ffffff8009184df3
            adrp x1, .L_ffffff800918265f
            add x0,x0, :lo12:.L_ffffff8009184df3
            add x1,x1, :lo12:.L_ffffff800918265f
            bl printk

            mov x8,#2336
            mov x9,#4272
            movk x8,#1752,lsl #16
            movk x9,#2404,lsl #16
            movk x8,#4672,lsl #32
            movk x9,#1280,lsl #32
            mov w10,#1280
            movk x8,#3504,lsl #48
            movk x9,#720,lsl #48
            movk w10,#720,lsl #16
            stp x8,x9,[x19]
            str w10,[x19,#16]
            ldp x29,lr,[sp,#16]
            mov w0,wzr
            ldr x19,[sp],#32
            ret
.size get_resolution_disambig_0xffffff8008708b20_0, . - get_resolution_disambig_0xffffff8008708b20_0
.size FUN_ffffff8008708b20, . - FUN_ffffff8008708b20
#-----------------------------------
.globl FUN_ffffff8008708b84
.type FUN_ffffff8008708b84, @function
#-----------------------------------
FUN_ffffff8008708b84:
#-----------------------------------
.type feature_control_disambig_0xffffff8008708b84_0, @function
#-----------------------------------
feature_control_disambig_0xffffff8008708b84_0:

            sub sp,sp,#112
            stp x28,x27,[sp,#16]
            stp x26,x25,[sp,#32]
            stp x24,x23,[sp,#48]
            stp x22,x21,[sp,#64]
            stp x20,x19,[sp,#80]
            stp x29,lr,[sp,#96]
            add x29,sp,#96
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov x19,x1
            mov w21,w0
            adrp x0, .L_ffffff8009184e11
            adrp x1, .L_ffffff800918267f
            mov x20,x2
            add x0,x0, :lo12:.L_ffffff8009184e11
            add x1,x1, :lo12:.L_ffffff800918267f
            mov w2,w21
            str x8,[sp,#8]
            bl printk

            sub w8,w21,#3002
            cmp w8,#109
            b.hi .L_ffffff8008709fec

            adrp x9, .L_ffffff8008f29948
            add x9,x9, :lo12:.L_ffffff8008f29948
            adr x10, .L_ffffff8008708bf8
            ldrh w11,[x9,x8,lsl #1]
            add x10,x10,x11, lsl #2
            br x10
.L_ffffff8008708bf8:

            adrp x8, .L_ffffff8009c7b658
            ldr w8,[x8,:lo12:.L_ffffff8009c7b658]
            adrp x9, .L_ffffff8009c7b654
            strh w8,[x19]
            ldr w8,[x9,:lo12:.L_ffffff8009c7b654]
            mov w9,#4
            strh w8,[x19,#2]
            str w9,[x20]
            b .L_ffffff8008709fec

            adrp x21, .L_ffffff8009c7b650
            adrp x8, .L_ffffff8009907a2c
            ldr w2,[x21,:lo12:.L_ffffff8009c7b650]
            ldrh w3,[x8,:lo12:.L_ffffff8009907a2c]
            adrp x0, .L_ffffff8009184e3d
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184e3d
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            ldr w8,[x21,:lo12:.L_ffffff8009c7b650]
            b .L_ffffff80087091cc

            ldr w21,[x19]
            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            mov x0,x19
            bl _raw_spin_lock_irqsave

            mov x1,x0
            and w23,w21,#65535
            adrp x8, .L_ffffff8009907a28
            mov x0,x19
            str w23,[x8,:lo12:.L_ffffff8009907a28]
            bl _raw_spin_unlock_irqrestore

            mov x0,x19
            bl _raw_spin_lock

            adrp x22, .L_ffffff8009c7b65c
            ldr w8,[x22,:lo12:.L_ffffff8009c7b65c]
            add w9,w23,#8
            mov w10,#32767
            adrp x20, .L_ffffff8009c7b654
            sub w11,w8,#8
            cmp w11,w23
            csel w8,w9,w8,lo
            cmp w8,w10
            csel w8,w8,w10,lo
            mov x0,x19
            str w8,[x20,:lo12:.L_ffffff8009c7b654]
            bl _raw_spin_unlock

            adrp x8, .L_ffffff8009c7b662
            ldrb w8,[x8,:lo12:.L_ffffff8009c7b662]
            cmp w23,#4
            mov w9,#4
            csel w21,w21,w9,hi
            cmp w8,#1
            b.ne .L_ffffff8008709488

            adrp x23, .L_ffffff8009c7b650
            adrp x19, .L_ffffff8009c7b658
            ldr w9,[x23,:lo12:.L_ffffff8009c7b650]
            ldr w10,[x19,:lo12:.L_ffffff8009c7b658]
            ldr w8,[x20,:lo12:.L_ffffff8009c7b654]
            udiv w9,w9,w10
            add w9,w9,w9, lsl #2
            lsl w9,w9,#1
            udiv w9,w9,w8
            and w9,w9,#65535
            sub w10,w9,#297
            cmp w10,#8
            b.hi .L_ffffff8008709668

            adrp x0, .L_ffffff800918516a
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff800918516a
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w2,#296
            mov w3,wzr
            bl printk

            ldr w8,[x23,:lo12:.L_ffffff8009c7b650]
            mov w9,#63765
            movk w9,#7084,lsl #16
            lsr w8,w8,#3
            umull x8,w8,w9
            ldr w9,[x19,:lo12:.L_ffffff8009c7b658]
            lsr x8,x8,#34
            b .L_ffffff80087096ac

            ldrh w19,[x19]
            mov w8,#16
            adrp x21, .L_ffffff8009c7b64c
            add x21,x21, :lo12:.L_ffffff8009c7b64c
            lsr w9,w19,#2
            cmp w9,#16
            csel w8,w9,w8,hi
            cmp w8,#248
            mov w9,#248
            mov x0,x21
            csel w20,w8,w9,lo
            bl _raw_spin_lock

            mov x0,x21
            bl _raw_spin_unlock

            adrp x0, .L_ffffff80091851f8
            adrp x1, .L_ffffff8009184790
            add x0,x0, :lo12:.L_ffffff80091851f8
            add x1,x1, :lo12:.L_ffffff8009184790
            mov w2,w19
            mov w3,w20
            bl printk

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2613
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2869
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            b .L_ffffff8008709fdc

            ldp w8,w9,[x19]
            adrp x10, .L_ffffff8009907a30
            ldrb w2,[x10,:lo12:.L_ffffff8009907a30]
            lsr w10,w8,#8
            strb w8,[sp,#5]
            strb w9,[sp,#6]
            strb w10,[sp,#4]
            b .L_ffffff8008709fdc

            ldr w8,[x19]
            adrp x9, .L_ffffff8009907a30
            ldrb w4,[x9,:lo12:.L_ffffff8009907a30]
            mov x0,sp
            lsr w9,w8,#8
            add x2,sp,#4
            mov w1,#2
            mov w3,#2
            strh wzr,[sp,#4]
            strb w9,[sp]
            strb w8,[sp,#1]
            bl iReadRegI2C

            ldrh w8,[sp,#4]
            rev w8,w8
            lsr w8,w8,#16
            str w8,[x19,#4]
            b .L_ffffff8008709fec

            mov w8,#-1
            b .L_ffffff80087091cc

            ldr w20,[x19]
            adrp x0, .L_ffffff8009185232
            adrp x1, .L_ffffff8009184895
            add x0,x0, :lo12:.L_ffffff8009185232
            and w19,w20,#65535
            add x1,x1, :lo12:.L_ffffff8009184895
            mov w2,w19
            bl printk

            cbz w19,.L_ffffff8008709fec

            adrp x0, .L_ffffff8009c7b64c
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            bl _raw_spin_lock

            cmp w19,#150
            b.eq .L_ffffff800870999c

            cmp w19,#300
            b.ne .L_ffffff80087099ac

            adrp x8, .L_ffffff8009c7b662
            ldrb w8,[x8,:lo12:.L_ffffff8009c7b662]
            tbz w8,#0,.L_ffffff80087099ac

            mov w20,#296
            b .L_ffffff80087099ac

            adrp x25, .L_ffffff8008f29b3c
            adrp x20, .L_ffffff8009c7b64c
            adrp x21, .L_ffffff8009184d16
            adrp x22, .L_ffffff80091848a4
            mov w24,#108
            add x25,x25, :lo12:.L_ffffff8008f29b3c
            add x20,x20, :lo12:.L_ffffff8009c7b64c
            adrp x23, .L_ffffff8009907a30
            mov w27,#2864
            mov w28,#3120
            add x21,x21, :lo12:.L_ffffff8009184d16
            add x22,x22, :lo12:.L_ffffff80091848a4
.L_ffffff8008708eb0:

            mov x0,x20
            bl _raw_spin_lock

            mov x0,x20
            strb w24,[x23,:lo12:.L_ffffff8009907a30]
            bl _raw_spin_unlock

            ldrb w4,[x23,:lo12:.L_ffffff8009907a30]
            mov w26,#2608
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w26,[sp]
            bl iReadRegI2C

            ldrb w4,[x23,:lo12:.L_ffffff8009907a30]
            ldrh w24,[sp,#4]
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w27,[sp]
            bl iReadRegI2C

            ldrh w8,[sp,#4]
            ldrb w4,[x23,:lo12:.L_ffffff8009907a30]
            mov x0,sp
            add x2,sp,#4
            lsl w8,w8,#8
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            orr w24,w8,w24, lsl #16
            strh w28,[sp]
            bl iReadRegI2C

            ldrh w8,[sp,#4]
            mov x0,x21
            mov x1,x22
            orr w3,w24,w8
            str w3,[x19]
            ldrb w2,[x23,:lo12:.L_ffffff8009907a30]
            bl printk

            ldr w3,[x19]
            ldrb w2,[x23,:lo12:.L_ffffff8009907a30]
            mov w8,#26752
            movk w8,#1,lsl #16
            cmp w3,w8
            b.eq .L_ffffff8008709460

            mov x0,x21
            mov x1,x22
            bl printk

            ldrb w4,[x23,:lo12:.L_ffffff8009907a30]
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w26,[sp]
            bl iReadRegI2C

            ldrb w4,[x23,:lo12:.L_ffffff8009907a30]
            ldrh w24,[sp,#4]
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w27,[sp]
            bl iReadRegI2C

            ldrh w8,[sp,#4]
            ldrb w4,[x23,:lo12:.L_ffffff8009907a30]
            mov x0,sp
            add x2,sp,#4
            lsl w8,w8,#8
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            orr w24,w8,w24, lsl #16
            strh w28,[sp]
            bl iReadRegI2C

            ldrh w8,[sp,#4]
            mov x0,x21
            mov x1,x22
            orr w3,w24,w8
            str w3,[x19]
            ldrb w2,[x23,:lo12:.L_ffffff8009907a30]
            bl printk

            ldr w3,[x19]
            ldrb w2,[x23,:lo12:.L_ffffff8009907a30]
            mov w8,#26752
            movk w8,#1,lsl #16
            cmp w3,w8
            b.eq .L_ffffff8008709460

            mov w26,#26752
            mov x0,x21
            mov x1,x22
            movk w26,#1,lsl #16
            bl printk

            ldrb w24,[x25],#1
            cmp w24,#255
            b.ne .L_ffffff8008708eb0

            ldr w8,[x19]
            cmp w8,w26
            b.eq .L_ffffff8008709fec

            mov w8,#-1
            str w8,[x19]
            b .L_ffffff8008709fec

            ldrb w8,[x19]
            ldrh w3,[x19,#2]
            adrp x0, .L_ffffff800918525f
            adrp x1, .L_ffffff800918493b
            cmp w8,#0
            cset w19,ne
            add x0,x0, :lo12:.L_ffffff800918525f
            add x1,x1, :lo12:.L_ffffff800918493b
            mov w2,w19
            bl printk

            adrp x20, .L_ffffff8009c7b64c
            add x20,x20, :lo12:.L_ffffff8009c7b64c
            mov x0,x20
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b662
            strb w19,[x8,:lo12:.L_ffffff8009c7b662]
            b .L_ffffff8008709210

            ldrb w19,[x19]
            adrp x0, .L_ffffff80091853ab
            adrp x1, .L_ffffff8009184a42
            add x0,x0, :lo12:.L_ffffff80091853ab
            cmp w19,#0
            cset w2,ne
            add x1,x1, :lo12:.L_ffffff8009184a42
            bl printk

            mov w8,#32850
            strh w8,[sp,#4]
            cbz w19,.L_ffffff80087095c8

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#128
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            b .L_ffffff80087095ec

            ldr w20,[x19]
            ldr w19,[x19,#8]
            adrp x0, .L_ffffff8009185298
            adrp x1, .L_ffffff800918498e
            add x0,x0, :lo12:.L_ffffff8009185298
            add x1,x1, :lo12:.L_ffffff800918498e
            mov w2,w20
            mov w3,w19
            bl printk

            cmp w20,#4
            b.hi .L_ffffff8008709b34

            adrp x8, .L_ffffff8008f29a40
            add x8,x8, :lo12:.L_ffffff8008f29a40
            adr x9, .L_ffffff8008709130
            ldrh w10,[x8,x20,lsl #1]
            add x9,x9,x10, lsl #2
            br x9
.L_ffffff8008709130:

            mov w8,#4096
            movk w8,#8789,lsl #16
            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b64c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            lsl w19,w8,#1
            bl _raw_spin_lock

            mov w8,#23839
            movk w8,#73,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008709c40

            mov w8,#42995
            movk w8,#13357,lsl #16
            umull x8,w19,w8
            lsr x8,x8,#42
            sub w8,w8,#956
            b .L_ffffff8008709c44

            ldr w20,[x19]
            ldr x19,[x19,#8]
            adrp x0, .L_ffffff800918537d
            adrp x1, .L_ffffff80091849f9
            add x0,x0, :lo12:.L_ffffff800918537d
            add x1,x1, :lo12:.L_ffffff80091849f9
            mov w2,w20
            bl printk

            cmp w20,#4
            b.hi .L_ffffff8008709fec

            adrp x8, .L_ffffff8008f29a36
            add x8,x8, :lo12:.L_ffffff8008f29a36
            adr x9, .L_ffffff80087091b8
            ldrh w10,[x8,x20,lsl #1]
            add x9,x9,x10, lsl #2
            br x9
.L_ffffff80087091b8:

            mov w8,#300
            str w8,[x19]
            b .L_ffffff8008709fec

            mov w8,#6777
            movk w8,#65182,lsl #16
.L_ffffff80087091cc:

            mov w9,#4
            str w8,[x19]
            str w9,[x20]
            b .L_ffffff8008709fec

            ldr w2,[x19]
            adrp x0, .L_ffffff8009184e99
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184e99
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            adrp x20, .L_ffffff8009c7b64c
            add x20,x20, :lo12:.L_ffffff8009c7b64c
            mov x0,x20
            bl _raw_spin_lock

            ldr x8,[x19]
            adrp x9, .L_ffffff8009907a2c
            strh w8,[x9,:lo12:.L_ffffff8009907a2c]
.L_ffffff8008709210:

            mov x0,x20
            bl _raw_spin_unlock

            b .L_ffffff8008709fec

            adrp x0, .L_ffffff8009184ec6
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184ec6
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            b .L_ffffff80087095fc

            ldr w2,[x19]
            adrp x0, .L_ffffff8009184f04
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184f04
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            ldr w9,[x19]
            ldr x8,[x19,#8]
            sub w9,w9,#1
            cmp w9,#3
            b.hi .L_ffffff8008709af4

            adrp x10, .L_ffffff8008f29a2e
            add x10,x10, :lo12:.L_ffffff8008f29a2e
            adr x11, .L_ffffff8008709278
            ldrh w12,[x10,x9,lsl #1]
            add x11,x11,x12, lsl #2
            br x11
.L_ffffff8008709278:

            adrp x9, .L_ffffff8008f29b64
            add x9,x9, :lo12:.L_ffffff8008f29b64
            b .L_ffffff8008709b20

            ldrh w2,[x19]
            adrp x0, .L_ffffff8009184f4c
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184f4c
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            ldr w9,[x19]
            ldr x8,[x19,#8]
            cmp w9,#2
            b.eq .L_ffffff8008709624

            cmp w9,#1
            b.ne .L_ffffff8008709630

            adrp x9, .L_ffffff8008f29c20
            add x9,x9, :lo12:.L_ffffff8008f29c20
            b .L_ffffff8008709638

            ldrh w2,[x19]
            ldrh w3,[x19,#8]
            ldrh w4,[x19,#16]
            adrp x0, .L_ffffff800918504f
.L_ffffff80087092d0:

            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff800918504f
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            b .L_ffffff8008709fec

            ldr w2,[x19]
            adrp x0, .L_ffffff8009184f87
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184f87
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            ldr x8,[x19]
            sub x8,x8,#1
            cmp x8,#1
            b.hi .L_ffffff8008709fec

            ldr x0,[x19,#8]
            adrp x1, .L_ffffff8008f29c98
            add x1,x1, :lo12:.L_ffffff8008f29c98
            mov w2,#380
            bl __pi_memcpy

            b .L_ffffff8008709fec

            ldr x2,[x19]
            adrp x0, .L_ffffff8009184fcf
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184fcf
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            ldr x8,[x19]
            cmp x8,#4
            b.hi .L_ffffff8008709360

            adrp x9, .L_ffffff8008f29a24
            add x9,x9, :lo12:.L_ffffff8008f29a24
            adr x10, .L_ffffff8008709360
            ldrh w11,[x9,x8,lsl #1]
            add x10,x10,x11, lsl #2
            br x10
.L_ffffff8008709360:

            ldr x8,[x19,#8]
            str wzr,[x8]
            b .L_ffffff8008709fec

            ldrh w2,[x19]
            adrp x0, .L_ffffff8009185024
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009185024
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            ldrb w8,[x19]
            adrp x9, .L_ffffff8009c7b664
            strb w8,[x9,:lo12:.L_ffffff8009c7b664]
            b .L_ffffff8008709fec

            ldr x22,[x19,#8]
            ldr w20,[x19]
            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            mov x0,x19
            and w23,w22,#65535
            bl _raw_spin_lock_irqsave

            mov x1,x0
            and w21,w20,#65535
            adrp x8, .L_ffffff8009907a28
            mov x0,x19
            str w21,[x8,:lo12:.L_ffffff8009907a28]
            bl _raw_spin_unlock_irqrestore

            mov x0,x19
            bl _raw_spin_lock

            cmp w23,#1
            b.ls .L_ffffff8008709490

            and w8,w22,#65535
            adrp x9, .L_ffffff8009c7b654
            str w8,[x9,:lo12:.L_ffffff8009c7b654]
            b .L_ffffff8008709498

            adrp x0, .L_ffffff8009185099
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009185099
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            mov w0,wzr
            bl streaming_control

            b .L_ffffff8008709fec

            adrp x0, .L_ffffff80091850db
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff80091850db
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            mov w0,#1
            bl streaming_control

            b .L_ffffff8008709fec

            ldr x8,[x19]
            sub x8,x8,#1
            cmp x8,#3
            b.hi .L_ffffff8008709450

            adrp x9, .L_ffffff8008f29a4a
            add x9,x9, :lo12:.L_ffffff8008f29a4a
            adr x10, .L_ffffff8008709450
            ldrb w11,[x9,x8]
            add x10,x10,x11, lsl #2
            br x10
.L_ffffff8008709450:

            ldr x8,[x19,#8]
            mov w9,#44928
            movk w9,#9002,lsl #16
            b .L_ffffff8008709c38
.L_ffffff8008709460:

            adrp x0, .L_ffffff8009184d66
            adrp x1, .L_ffffff80091848a4
            mov w3,#26752
            add x0,x0, :lo12:.L_ffffff8009184d66
            add x1,x1, :lo12:.L_ffffff80091848a4
            movk w3,#1,lsl #16
            bl printk

            ldrb w0,[x23,:lo12:.L_ffffff8009907a30]
            bl ov16880CheckVersion

            b .L_ffffff8008709fec
.L_ffffff8008709488:

            ldr w8,[x20,:lo12:.L_ffffff8009c7b654]
            b .L_ffffff80087097a8
.L_ffffff8008709490:

            adrp x8, .L_ffffff8009c7b654
            ldr w8,[x8,:lo12:.L_ffffff8009c7b654]
.L_ffffff8008709498:

            sub w9,w8,#8
            add w10,w21,#8
            cmp w9,w21
            csel w8,w10,w8,lo
            b.lo .L_ffffff80087094b4

            cmp w8,#8,lsl #12
            b.lo .L_ffffff80087094cc
.L_ffffff80087094b4:

            lsr w9,w8,#15
            mov w10,#32767
            cmp w9,#0
            csel w8,w10,w8,ne
            adrp x9, .L_ffffff8009c7b654
            str w8,[x9,:lo12:.L_ffffff8009c7b654]
.L_ffffff80087094cc:

            adrp x0, .L_ffffff8009c7b64c
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            bl _raw_spin_unlock

            and w8,w20,#65535
            adrp x10, .L_ffffff8009c7b662
            mov w9,#4
            cmp w8,#4
            ldrb w8,[x10,:lo12:.L_ffffff8009c7b662]
            csel w9,w20,w9,hi
            mov w10,#32759
            cmp w10,w9, uxth
            csel w9,w9,w10,hi
            cmp w8,#1
            and w19,w9,#65535
            b.ne .L_ffffff800870957c

            adrp x22, .L_ffffff8009c7b650
            adrp x20, .L_ffffff8009c7b658
            ldr w9,[x22,:lo12:.L_ffffff8009c7b650]
            ldr w10,[x20,:lo12:.L_ffffff8009c7b658]
            adrp x21, .L_ffffff8009c7b654
            ldr w8,[x21,:lo12:.L_ffffff8009c7b654]
            udiv w9,w9,w10
            add w9,w9,w9, lsl #2
            lsl w9,w9,#1
            udiv w9,w9,w8
            and w9,w9,#65535
            sub w10,w9,#297
            cmp w10,#8
            b.hi .L_ffffff8008709858

            adrp x0, .L_ffffff800918516a
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff800918516a
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w2,#296
            mov w3,wzr
            bl printk

            ldr w8,[x22,:lo12:.L_ffffff8009c7b650]
            mov w9,#63765
            movk w9,#7084,lsl #16
            lsr w8,w8,#3
            umull x8,w8,w9
            ldr w9,[x20,:lo12:.L_ffffff8009c7b658]
            lsr x8,x8,#34
            b .L_ffffff800870989c
.L_ffffff800870957c:

            adrp x20, .L_ffffff8009c7b654
            ldr w9,[x20,:lo12:.L_ffffff8009c7b654]
            adrp x21, .L_ffffff8009907a30
            ldrb w2,[x21,:lo12:.L_ffffff8009907a30]
            mov w8,#3640
            strh w8,[sp,#4]
.L_ffffff8008709594:

            lsr w8,w9,#8
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x20,:lo12:.L_ffffff8009c7b654]
            ldrb w2,[x21,:lo12:.L_ffffff8009907a30]
            b .L_ffffff8008709cdc

            ldr x8,[x19,#8]
            mov w9,#30208
            movk w9,#4272,lsl #16
            b .L_ffffff8008709c38
.L_ffffff80087095c8:

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            strh wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            movk w8,#8,lsl #16
.L_ffffff80087095ec:

            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C
.L_ffffff80087095fc:

            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            mov x0,x19
            bl _raw_spin_lock

            mov x0,x19
            bl _raw_spin_unlock

            b .L_ffffff8008709fec

.L_ffffff8008709618:
            mov w8,#1200
            str w8,[x19]
            b .L_ffffff8008709fec
.L_ffffff8008709624:

            adrp x9, .L_ffffff8008f29c5c
            add x9,x9, :lo12:.L_ffffff8008f29c5c
            b .L_ffffff8008709638
.L_ffffff8008709630:

            adrp x9, .L_ffffff8008f29be4
            add x9,x9, :lo12:.L_ffffff8008f29be4
.L_ffffff8008709638:

            ldp x11,x10,[x9,#16]
            ldp x12,x13,[x9]
            str x10,[x8,#24]
            ldr w10,[x9,#56]
            stp x13,x11,[x8,#8]
            ldp x11,x13,[x9,#32]
            ldr x9,[x9,#48]
            str x12,[x8]
            str w10,[x8,#56]
            str x11,[x8,#32]
            stp x13,x9,[x8,#40]
            b .L_ffffff8008709fec
.L_ffffff8008709668:

            sub w9,w9,#147
            cmp w9,#3
            b.hi .L_ffffff80087097a8

            adrp x0, .L_ffffff800918516a
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff800918516a
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w2,#146
            mov w3,wzr
            bl printk

            ldr w8,[x23,:lo12:.L_ffffff8009c7b650]
            mov w9,#14365
            movk w9,#57456,lsl #16
            lsr w8,w8,#1
            umull x8,w8,w9
            ldr w9,[x19,:lo12:.L_ffffff8009c7b658]
            lsr x8,x8,#38
.L_ffffff80087096ac:

            add w8,w8,w8, lsl #2
            adrp x0, .L_ffffff8009c7b64c
            lsl w8,w8,#1
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            udiv w23,w8,w9
            bl _raw_spin_lock

            ldr w8,[x22,:lo12:.L_ffffff8009c7b65c]
            adrp x22, .L_ffffff8009c7b660
            cmp w23,w8
            csel w9,w23,w8,hi
            sub w10,w9,w8
            cmp w9,#8,lsl #12
            str w9,[x20,:lo12:.L_ffffff8009c7b654]
            strh w10,[x22,:lo12:.L_ffffff8009c7b660]
            b.lo .L_ffffff80087096f8

            mov w9,#32767
            sub w8,w9,w8
            str w9,[x20,:lo12:.L_ffffff8009c7b654]
            strh w8,[x22,:lo12:.L_ffffff8009c7b660]
.L_ffffff80087096f8:

            adrp x0, .L_ffffff8009c7b64c
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            bl _raw_spin_unlock

            ldrsh w2,[x22,:lo12:.L_ffffff8009c7b660]
            adrp x0, .L_ffffff80091851ba
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff80091851ba
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x20,:lo12:.L_ffffff8009c7b654]
            adrp x22, .L_ffffff8009907a30
            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            mov w9,#3640
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x20,:lo12:.L_ffffff8009c7b654]
            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            mov w9,#3896
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b658]
            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            mov w9,#3128
            add x0,sp,#4
            lsr w8,w8,#8
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b658]
            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            mov w9,#3384
            b .L_ffffff80087097dc
.L_ffffff80087097a8:

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w9,#3640
            lsr w8,w8,#8
            strh w9,[sp,#4]
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x20,:lo12:.L_ffffff8009c7b654]
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w9,#3896
.L_ffffff80087097dc:

            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            adrp x22, .L_ffffff8009907a30
            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            mov w8,#565
            add x0,sp,#4
            mov w1,#3
            and w19,w21,#65535
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            ubfx w8,w19,#8,#7
            mov w9,#309
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w3,[x20,:lo12:.L_ffffff8009c7b654]
            adrp x0, .L_ffffff800918511c
            adrp x1, .L_ffffff800918515c
            add x0,x0, :lo12:.L_ffffff800918511c
            add x1,x1, :lo12:.L_ffffff800918515c
            b .L_ffffff8008709d58
.L_ffffff8008709858:

            sub w9,w9,#147
            cmp w9,#3
            b.hi .L_ffffff8008709cac

            adrp x0, .L_ffffff800918516a
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff800918516a
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w2,#146
            mov w3,wzr
            bl printk

            ldr w8,[x22,:lo12:.L_ffffff8009c7b650]
            mov w9,#14365
            movk w9,#57456,lsl #16
            lsr w8,w8,#1
            umull x8,w8,w9
            ldr w9,[x20,:lo12:.L_ffffff8009c7b658]
            lsr x8,x8,#38
.L_ffffff800870989c:

            add w8,w8,w8, lsl #2
            adrp x0, .L_ffffff8009c7b64c
            lsl w8,w8,#1
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            udiv w23,w8,w9
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b65c
            ldr w8,[x8,:lo12:.L_ffffff8009c7b65c]
            adrp x22, .L_ffffff8009c7b660
            cmp w23,w8
            csel w9,w23,w8,hi
            sub w10,w9,w8
            cmp w9,#8,lsl #12
            str w9,[x21,:lo12:.L_ffffff8009c7b654]
            strh w10,[x22,:lo12:.L_ffffff8009c7b660]
            b.lo .L_ffffff80087098ec

            mov w9,#32767
            sub w8,w9,w8
            str w9,[x21,:lo12:.L_ffffff8009c7b654]
            strh w8,[x22,:lo12:.L_ffffff8009c7b660]
.L_ffffff80087098ec:

            adrp x0, .L_ffffff8009c7b64c
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            bl _raw_spin_unlock

            ldrsh w2,[x22,:lo12:.L_ffffff8009c7b660]
            adrp x0, .L_ffffff80091851ba
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff80091851ba
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x21,:lo12:.L_ffffff8009c7b654]
            adrp x22, .L_ffffff8009907a30
            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            mov w9,#3640
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x21,:lo12:.L_ffffff8009c7b654]
            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            mov w9,#3896
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x20,:lo12:.L_ffffff8009c7b658]
            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            mov w9,#3128
            add x0,sp,#4
            lsr w8,w8,#8
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x20,:lo12:.L_ffffff8009c7b658]
            ldrb w2,[x22,:lo12:.L_ffffff8009907a30]
            mov w9,#3384
            b .L_ffffff8008709ce0
.L_ffffff800870999c:

            adrp x8, .L_ffffff8009c7b662
            ldrb w8,[x8,:lo12:.L_ffffff8009c7b662]
            tbz w8,#0,.L_ffffff80087099ac

            mov w20,#146
.L_ffffff80087099ac:

            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            adrp x21, .L_ffffff8009907a2c
            mov x0,x19
            strh w20,[x21,:lo12:.L_ffffff8009907a2c]
            bl _raw_spin_unlock

            ldrh w20,[x21,:lo12:.L_ffffff8009907a2c]
            adrp x0, .L_ffffff800918516a
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff800918516a
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w3,#1
            mov w2,w20
            bl printk

            adrp x8, .L_ffffff8009c7b650
            ldr w8,[x8,:lo12:.L_ffffff8009c7b650]
            adrp x21, .L_ffffff8009c7b658
            ldr w9,[x21,:lo12:.L_ffffff8009c7b658]
            mov x0,x19
            udiv w8,w8,w20
            add w8,w8,w8, lsl #2
            lsl w8,w8,#1
            udiv w22,w8,w9
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b65c
            ldr w9,[x8,:lo12:.L_ffffff8009c7b65c]
            adrp x19, .L_ffffff8009c7b654
            adrp x20, .L_ffffff8009c7b660
            cmp w22,w9
            csel w10,w22,w9,hi
            sub w11,w10,w9
            cmp w10,#8,lsl #12
            str w10,[x19,:lo12:.L_ffffff8009c7b654]
            strh w11,[x20,:lo12:.L_ffffff8009c7b660]
            b.lo .L_ffffff8008709a48

            mov w10,#32767
            sub w9,w10,w9
            str w10,[x19,:lo12:.L_ffffff8009c7b654]
            strh w9,[x20,:lo12:.L_ffffff8009c7b660]
.L_ffffff8008709a48:

            adrp x0, .L_ffffff8009c7b64c
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            str w10,[x8,:lo12:.L_ffffff8009c7b65c]
            bl _raw_spin_unlock

            ldrsh w2,[x20,:lo12:.L_ffffff8009c7b660]
            adrp x0, .L_ffffff80091851ba
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff80091851ba
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x19,:lo12:.L_ffffff8009c7b654]
            adrp x20, .L_ffffff8009907a30
            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            mov w9,#3640
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b654]
            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            mov w9,#3896
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x21,:lo12:.L_ffffff8009c7b658]
            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            mov w9,#3128
            add x0,sp,#4
            lsr w8,w8,#8
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x21,:lo12:.L_ffffff8009c7b658]
            b .L_ffffff8008709fcc
.L_ffffff8008709af4:

            adrp x9, .L_ffffff8008f29b44
            add x9,x9, :lo12:.L_ffffff8008f29b44
            b .L_ffffff8008709b20

            adrp x9, .L_ffffff8008f29b84
            add x9,x9, :lo12:.L_ffffff8008f29b84
            b .L_ffffff8008709b20

            adrp x9, .L_ffffff8008f29ba4
            add x9,x9, :lo12:.L_ffffff8008f29ba4
            b .L_ffffff8008709b20

            adrp x9, .L_ffffff8008f29bc4
            add x9,x9, :lo12:.L_ffffff8008f29bc4
.L_ffffff8008709b20:

            ldp x11,x10,[x9,#16]
            ldp x9,x12,[x9]
            stp x11,x10,[x8,#16]
            stp x9,x12,[x8]
            b .L_ffffff8008709fec
.L_ffffff8008709b34:

            mov w8,#34816
            movk w8,#4394,lsl #16
            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b64c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            lsl w19,w8,#1
            bl _raw_spin_lock

            mov w8,#40143
            movk w8,#146,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008709d64

            mov w8,#42995
            movk w8,#13357,lsl #16
            umull x8,w19,w8
            lsr x8,x8,#41
            sub w8,w8,#3824
            b .L_ffffff8008709d68

            mov w8,#34816
            movk w8,#4394,lsl #16
            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b64c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            lsl w19,w8,#1
            bl _raw_spin_lock

            mov w8,#40143
            movk w8,#146,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008709c60

            mov w8,#42995
            movk w8,#13357,lsl #16
            umull x8,w19,w8
            lsr x8,x8,#41
            b .L_ffffff8008709c28

            adrp x8, .L_ffffff8009907a2c
            ldrh w8,[x8,:lo12:.L_ffffff8009907a2c]
            cmp w8,#300
            b.eq .L_ffffff8008709e9c

            cmp w8,#150
            b.ne .L_ffffff8008709e80

            mov w8,#34816
            movk w8,#4394,lsl #16
            b .L_ffffff8008709ea4

            cbz w19,.L_ffffff8008709fec

            mov w8,#4096
            movk w8,#8789,lsl #16
            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b64c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            lsl w19,w8,#1
            bl _raw_spin_lock

            mov w8,#14751
            movk w8,#293,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008709c60

            mov w8,#42995
            movk w8,#13357,lsl #16
            umull x8,w19,w8
            lsr x8,x8,#42
.L_ffffff8008709c28:

            sub w8,w8,#3824
            b .L_ffffff8008709c64

            ldr x8,[x19,#8]
            mov w9,#1
.L_ffffff8008709c38:

            str w9,[x8]
            b .L_ffffff8008709fec
.L_ffffff8008709c40:

            mov w8,wzr
.L_ffffff8008709c44:

            adrp x20, .L_ffffff8009c7b660
            sxth w9,w8
            adrp x19, .L_ffffff8009c7b654
            adrp x10, .L_ffffff8009c7b65c
            strh w8,[x20,:lo12:.L_ffffff8009c7b660]
            add w8,w9,#956
            b .L_ffffff8008709c7c
.L_ffffff8008709c60:

            mov w8,wzr
.L_ffffff8008709c64:

            adrp x20, .L_ffffff8009c7b660
            sxth w9,w8
            adrp x19, .L_ffffff8009c7b654
            adrp x10, .L_ffffff8009c7b65c
            strh w8,[x20,:lo12:.L_ffffff8009c7b660]
            add w8,w9,#3824
.L_ffffff8008709c7c:

            adrp x0, .L_ffffff8009c7b64c
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            str w8,[x19,:lo12:.L_ffffff8009c7b654]
            str w8,[x10,:lo12:.L_ffffff8009c7b65c]
            bl _raw_spin_unlock

            adrp x9, .L_ffffff8009907a28
            ldr w8,[x19,:lo12:.L_ffffff8009c7b654]
            ldr w9,[x9,:lo12:.L_ffffff8009907a28]
            cmp w8,w9
            b.ls .L_ffffff8008709fec

            ldrsh w2,[x20,:lo12:.L_ffffff8009c7b660]
            b .L_ffffff8008709f34
.L_ffffff8008709cac:

            adrp x20, .L_ffffff8009907a30
            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            lsr w8,w8,#8
            mov w9,#3640
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x21,:lo12:.L_ffffff8009c7b654]
            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
.L_ffffff8008709cdc:

            mov w9,#3896
.L_ffffff8008709ce0:

            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            adrp x20, .L_ffffff8009907a30
            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            mov w8,#565
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            lsr w8,w19,#8
            mov w9,#309
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            adrp x8, .L_ffffff8009c7b654
            ldr w3,[x8,:lo12:.L_ffffff8009c7b654]
            adrp x0, .L_ffffff800918511c
            adrp x1, .L_ffffff8009184ab5
            add x0,x0, :lo12:.L_ffffff800918511c
            add x1,x1, :lo12:.L_ffffff8009184ab5
.L_ffffff8008709d58:

            mov w2,w19
            bl printk

            b .L_ffffff8008709fec
.L_ffffff8008709d64:

            mov w8,wzr
.L_ffffff8008709d68:

            adrp x21, .L_ffffff8009c7b660
            sxth w9,w8
            adrp x0, .L_ffffff8009c7b64c
            adrp x19, .L_ffffff8009c7b654
            adrp x10, .L_ffffff8009c7b65c
            strh w8,[x21,:lo12:.L_ffffff8009c7b660]
            add w8,w9,#3824
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            str w8,[x19,:lo12:.L_ffffff8009c7b654]
            str w8,[x10,:lo12:.L_ffffff8009c7b65c]
            bl _raw_spin_unlock

            adrp x9, .L_ffffff8009907a28
            ldr w8,[x19,:lo12:.L_ffffff8009c7b654]
            ldr w9,[x9,:lo12:.L_ffffff8009907a28]
            cmp w8,w9
            b.ls .L_ffffff8008709e64

            ldrsh w2,[x21,:lo12:.L_ffffff8009c7b660]
            adrp x0, .L_ffffff80091851ba
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff80091851ba
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x19,:lo12:.L_ffffff8009c7b654]
            adrp x21, .L_ffffff8009907a30
            ldrb w2,[x21,:lo12:.L_ffffff8009907a30]
            mov w9,#3640
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b654]
            ldrb w2,[x21,:lo12:.L_ffffff8009907a30]
            mov w9,#3896
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            adrp x19, .L_ffffff8009c7b658
            ldr w8,[x19,:lo12:.L_ffffff8009c7b658]
            ldrb w2,[x21,:lo12:.L_ffffff8009907a30]
            mov w9,#3128
            add x0,sp,#4
            lsr w8,w8,#8
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b658]
            ldrb w2,[x21,:lo12:.L_ffffff8009907a30]
            mov w9,#3384
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C
.L_ffffff8008709e64:

            adrp x0, .L_ffffff8009185330
            adrp x1, .L_ffffff800918498e
            add x0,x0, :lo12:.L_ffffff8009185330
            add x1,x1, :lo12:.L_ffffff800918498e
            mov w2,w20
            bl printk

            b .L_ffffff8008709fec
.L_ffffff8008709e80:

            adrp x0, .L_ffffff80091852d6
            adrp x1, .L_ffffff800918498e
            add x0,x0, :lo12:.L_ffffff80091852d6
            add x1,x1, :lo12:.L_ffffff800918498e
            mov w3,#30
            mov w2,w19
            bl printk
.L_ffffff8008709e9c:

            mov w8,#4096
            movk w8,#8789,lsl #16
.L_ffffff8008709ea4:

            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b64c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            lsl w19,w8,#1
            bl _raw_spin_lock

            mov w8,#14751
            movk w8,#293,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008709ee4

            mov w8,#42995
            movk w8,#13357,lsl #16
            umull x8,w19,w8
            lsr x8,x8,#42
            sub w8,w8,#3824
            b .L_ffffff8008709ee8
.L_ffffff8008709ee4:

            mov w8,wzr
.L_ffffff8008709ee8:

            adrp x9, .L_ffffff8009c7b660
            sxth w10,w8
            adrp x0, .L_ffffff8009c7b64c
            adrp x11, .L_ffffff8009c7b654
            adrp x12, .L_ffffff8009c7b65c
            strh w8,[x9,:lo12:.L_ffffff8009c7b660]
            add w8,w10,#3824
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            str w8,[x11,:lo12:.L_ffffff8009c7b654]
            str w8,[x12,:lo12:.L_ffffff8009c7b65c]
            bl _raw_spin_unlock

            adrp x19, .L_ffffff8009c7b654
            adrp x8, .L_ffffff8009907a28
            ldr w9,[x19,:lo12:.L_ffffff8009c7b654]
            ldr w8,[x8,:lo12:.L_ffffff8009907a28]
            cmp w9,w8
            b.ls .L_ffffff8008709fec

            adrp x8, .L_ffffff8009c7b660
            ldrsh w2,[x8,:lo12:.L_ffffff8009c7b660]
.L_ffffff8008709f34:

            adrp x0, .L_ffffff80091851ba
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff80091851ba
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x19,:lo12:.L_ffffff8009c7b654]
            adrp x20, .L_ffffff8009907a30
            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            mov w9,#3640
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b654]
            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            mov w9,#3896
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            adrp x19, .L_ffffff8009c7b658
            ldr w8,[x19,:lo12:.L_ffffff8009c7b658]
            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            mov w9,#3128
            add x0,sp,#4
            lsr w8,w8,#8
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            strb wzr,[sp,#7]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b658]
.L_ffffff8008709fcc:

            ldrb w2,[x20,:lo12:.L_ffffff8009907a30]
            mov w9,#3384
            strh w9,[sp,#4]
            strb w8,[sp,#6]
.L_ffffff8008709fdc:

            add x0,sp,#4
            mov w1,#3
            strb wzr,[sp,#7]
            bl iWriteRegI2C
.L_ffffff8008709fec:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff800870a024

            ldp x29,lr,[sp,#96]
            ldp x20,x19,[sp,#80]
            ldp x22,x21,[sp,#64]
            ldp x24,x23,[sp,#48]
            ldp x26,x25,[sp,#32]
            ldp x28,x27,[sp,#16]
            mov w0,wzr
            add sp,sp,#112
            ret
.L_ffffff800870a024:

            bl __stack_chk_fail
.size feature_control_disambig_0xffffff8008708b84_0, . - feature_control_disambig_0xffffff8008708b84_0
.size FUN_ffffff8008708b84, . - FUN_ffffff8008708b84
#-----------------------------------
.type FUN_ffffff800870a028, @function
#-----------------------------------
FUN_ffffff800870a028:
#-----------------------------------
.type control_disambig_0xffffff800870a028_0, @function
#-----------------------------------
control_disambig_0xffffff800870a028_0:

            stp x20,x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            mov w19,w0
            adrp x0, .L_ffffff8009184dc6
            adrp x1, .L_ffffff80092670e3
            add x0,x0, :lo12:.L_ffffff8009184dc6
            add x1,x1, :lo12:.L_ffffff80092670e3
            mov w2,w19
            bl printk

            adrp x20, .L_ffffff8009c7b64c
            add x20,x20, :lo12:.L_ffffff8009c7b64c
            mov x0,x20
            bl _raw_spin_lock

            mov x0,x20
            bl _raw_spin_unlock

            cmp w19,#4
            b.hi .L_ffffff800870a2d4

            adrp x9, .L_ffffff8008f29a4e
            mov w8,w19
            add x9,x9, :lo12:.L_ffffff8008f29a4e
            adr x10, .L_ffffff800870a08c
            ldrb w11,[x9,x8]
            add x10,x10,x11, lsl #2
            br x10
.L_ffffff800870a08c:

            adrp x0, .L_ffffff8009184da7
            adrp x1, .L_ffffff80091c6b2a
            add x0,x0, :lo12:.L_ffffff8009184da7
            add x1,x1, :lo12:.L_ffffff80091c6b2a
            bl printk

            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            mov x0,x19
            bl _raw_spin_lock

            mov w9,#34816
            adrp x8, .L_ffffff8009c7b650
            movk w9,#4394,lsl #16
            adrp x10, .L_ffffff8009c7b658
            mov w11,#2512
            adrp x12, .L_ffffff8009c7b654
            mov w13,#3824
            adrp x14, .L_ffffff8009c7b65c
            adrp x15, .L_ffffff8009c7b662
            mov x0,x19
            str w9,[x8,:lo12:.L_ffffff8009c7b650]
            str w11,[x10,:lo12:.L_ffffff8009c7b658]
            str w13,[x12,:lo12:.L_ffffff8009c7b654]
            str w13,[x14,:lo12:.L_ffffff8009c7b65c]
            strb wzr,[x15,:lo12:.L_ffffff8009c7b662]
            bl _raw_spin_unlock

            b .L_ffffff800870a2c0

            adrp x0, .L_ffffff8009184df3
            adrp x1, .L_ffffff8009262aba
            add x0,x0, :lo12:.L_ffffff8009184df3
            add x1,x1, :lo12:.L_ffffff8009262aba
            bl printk

            adrp x0, .L_ffffff8009c7b64c
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            bl _raw_spin_lock

            adrp x19, .L_ffffff8009907a2c
            ldrh w2,[x19,:lo12:.L_ffffff8009907a2c]
            cmp w2,#150
            b.eq .L_ffffff800870a37c

            cmp w2,#300
            b.eq .L_ffffff800870a358

            cmp w2,#240
            b.ne .L_ffffff800870a364

            adrp x0, .L_ffffff8009185454
            adrp x1, .L_ffffff8009262aba
            mov w20,#34816
            add x0,x0, :lo12:.L_ffffff8009185454
            add x1,x1, :lo12:.L_ffffff8009262aba
            mov w3,#15
            movk w20,#4394,lsl #16
            bl printk

            b .L_ffffff800870a384

            adrp x0, .L_ffffff8009184df3
            adrp x1, .L_ffffff8009184bd7
            add x0,x0, :lo12:.L_ffffff8009184df3
            add x1,x1, :lo12:.L_ffffff8009184bd7
            bl printk

            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            mov x0,x19
            bl _raw_spin_lock

            mov w9,#4096
            adrp x8, .L_ffffff8009c7b650
            movk w9,#8789,lsl #16
            adrp x10, .L_ffffff8009c7b658
            mov w11,#5024
            adrp x12, .L_ffffff8009c7b654
            mov w13,#3824
            adrp x14, .L_ffffff8009c7b65c
            adrp x15, .L_ffffff8009c7b662
            mov x0,x19
            str w9,[x8,:lo12:.L_ffffff8009c7b650]
            str w11,[x10,:lo12:.L_ffffff8009c7b658]
            str w13,[x12,:lo12:.L_ffffff8009c7b654]
            str w13,[x14,:lo12:.L_ffffff8009c7b65c]
            strb wzr,[x15,:lo12:.L_ffffff8009c7b662]
            bl _raw_spin_unlock

            adrp x8, .L_ffffff8009907a2c
            ldrh w0,[x8,:lo12:.L_ffffff8009907a2c]
            bl normal_video_setting

            b .L_ffffff800870a2c4

            adrp x0, .L_ffffff8009184df3
            adrp x1, .L_ffffff8009184bf9
            add x0,x0, :lo12:.L_ffffff8009184df3
            add x1,x1, :lo12:.L_ffffff8009184bf9
            bl printk

            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            mov x0,x19
            bl _raw_spin_lock

            mov w9,#4096
            adrp x8, .L_ffffff8009c7b650
            movk w9,#8789,lsl #16
            adrp x10, .L_ffffff8009c7b658
            mov w11,#5024
            adrp x12, .L_ffffff8009c7b654
            mov w13,#956
            adrp x14, .L_ffffff8009c7b65c
            adrp x15, .L_ffffff8009c7b660
            adrp x16, .L_ffffff8009c7b662
            mov x0,x19
            str w9,[x8,:lo12:.L_ffffff8009c7b650]
            str w11,[x10,:lo12:.L_ffffff8009c7b658]
            str w13,[x12,:lo12:.L_ffffff8009c7b654]
            str w13,[x14,:lo12:.L_ffffff8009c7b65c]
            strh wzr,[x15,:lo12:.L_ffffff8009c7b660]
            strb wzr,[x16,:lo12:.L_ffffff8009c7b662]
            bl _raw_spin_unlock

            bl hs_video_setting

            b .L_ffffff800870a2c4

            adrp x0, .L_ffffff8009184df3
            adrp x1, .L_ffffff8009184c13
            add x0,x0, :lo12:.L_ffffff8009184df3
            add x1,x1, :lo12:.L_ffffff8009184c13
            bl printk

            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            mov x0,x19
            bl _raw_spin_lock

            mov w9,#4096
            adrp x8, .L_ffffff8009c7b650
            movk w9,#8789,lsl #16
            adrp x10, .L_ffffff8009c7b658
            mov w11,#5024
            adrp x12, .L_ffffff8009c7b654
            mov w13,#956
            adrp x14, .L_ffffff8009c7b65c
            adrp x15, .L_ffffff8009c7b660
            adrp x16, .L_ffffff8009c7b662
            mov x0,x19
            str w9,[x8,:lo12:.L_ffffff8009c7b650]
            str w11,[x10,:lo12:.L_ffffff8009c7b658]
            str w13,[x12,:lo12:.L_ffffff8009c7b654]
            str w13,[x14,:lo12:.L_ffffff8009c7b65c]
            strh wzr,[x15,:lo12:.L_ffffff8009c7b660]
            strb wzr,[x16,:lo12:.L_ffffff8009c7b662]
            bl _raw_spin_unlock

            adrp x0, .L_ffffff8009184da7
            adrp x1, .L_ffffff8009184c1e
            add x0,x0, :lo12:.L_ffffff8009184da7
            add x1,x1, :lo12:.L_ffffff8009184c1e
            bl printk
.L_ffffff800870a2c0:

            bl FUN_ffffff8008712b3c
.L_ffffff800870a2c4:

            mov w0,wzr
.L_ffffff800870a2c8:

            ldp x29,lr,[sp,#16]
            ldp x20,x19,[sp],#32
            ret
.L_ffffff800870a2d4:

            adrp x0, .L_ffffff800918541f
            adrp x1, .L_ffffff80092670e3
            add x0,x0, :lo12:.L_ffffff800918541f
            add x1,x1, :lo12:.L_ffffff80092670e3
            bl printk

            adrp x0, .L_ffffff8009184da7
            adrp x1, .L_ffffff80091c6b2a
            add x0,x0, :lo12:.L_ffffff8009184da7
            add x1,x1, :lo12:.L_ffffff80091c6b2a
            bl printk

            adrp x19, .L_ffffff8009c7b64c
            add x19,x19, :lo12:.L_ffffff8009c7b64c
            mov x0,x19
            bl _raw_spin_lock

            mov w9,#34816
            adrp x8, .L_ffffff8009c7b650
            movk w9,#4394,lsl #16
            adrp x10, .L_ffffff8009c7b658
            mov w11,#2512
            adrp x12, .L_ffffff8009c7b654
            mov w13,#3824
            adrp x14, .L_ffffff8009c7b65c
            adrp x15, .L_ffffff8009c7b662
            mov x0,x19
            str w9,[x8,:lo12:.L_ffffff8009c7b650]
            str w11,[x10,:lo12:.L_ffffff8009c7b658]
            str w13,[x12,:lo12:.L_ffffff8009c7b654]
            str w13,[x14,:lo12:.L_ffffff8009c7b65c]
            strb wzr,[x15,:lo12:.L_ffffff8009c7b662]
            bl _raw_spin_unlock

            bl FUN_ffffff8008712b3c

            mov w0,#4
            b .L_ffffff800870a2c8
.L_ffffff800870a358:

            mov w20,#4096
            movk w20,#8789,lsl #16
            b .L_ffffff800870a384
.L_ffffff800870a364:

            adrp x0, .L_ffffff8009185454
            adrp x1, .L_ffffff8009262aba
            add x0,x0, :lo12:.L_ffffff8009185454
            add x1,x1, :lo12:.L_ffffff8009262aba
            mov w3,#15
            bl printk
.L_ffffff800870a37c:

            mov w20,#34816
            movk w20,#4394,lsl #16
.L_ffffff800870a384:

            adrp x0, .L_ffffff8009c7b64c
            adrp x8, .L_ffffff8009c7b650
            adrp x9, .L_ffffff8009c7b658
            mov w10,#5024
            adrp x11, .L_ffffff8009c7b654
            mov w12,#3824
            adrp x13, .L_ffffff8009c7b65c
            adrp x14, .L_ffffff8009c7b662
            add x0,x0, :lo12:.L_ffffff8009c7b64c
            str w20,[x8,:lo12:.L_ffffff8009c7b650]
            str w10,[x9,:lo12:.L_ffffff8009c7b658]
            str w12,[x11,:lo12:.L_ffffff8009c7b654]
            str w12,[x13,:lo12:.L_ffffff8009c7b65c]
            strb wzr,[x14,:lo12:.L_ffffff8009c7b662]
            bl _raw_spin_unlock

            ldrh w2,[x19,:lo12:.L_ffffff8009907a2c]
            adrp x0, .L_ffffff80091854bc
            adrp x1, .L_ffffff8009262aba
            add x0,x0, :lo12:.L_ffffff80091854bc
            add x1,x1, :lo12:.L_ffffff8009262aba
            bl printk

            ldrh w8,[x19,:lo12:.L_ffffff8009907a2c]
            cmp w8,#150
            b.eq .L_ffffff800870a3f8

            cmp w8,#200
            b.eq .L_ffffff800870a400

            cmp w8,#300
            bl normal_capture_setting

            b .L_ffffff800870a2c4
.L_ffffff800870a3f8:

            bl pip_capture_15fps_setting

            b .L_ffffff800870a2c4
.L_ffffff800870a400:

            bl pip_capture_setting

            b .L_ffffff800870a2c4
.size control_disambig_0xffffff800870a028_0, . - control_disambig_0xffffff800870a028_0
.size FUN_ffffff800870a028, . - FUN_ffffff800870a028
#-----------------------------------
.type FUN_ffffff800870a408, @function
#-----------------------------------
FUN_ffffff800870a408:
#-----------------------------------
.type close_disambig_0xffffff800870a408_0, @function
#-----------------------------------
close_disambig_0xffffff800870a408_0:

            stp x29,lr,[sp,#-16]!
            mov x29,sp
            adrp x0, .L_ffffff8009184da7
            adrp x1, .L_ffffff800916e19a
            add x0,x0, :lo12:.L_ffffff8009184da7
            add x1,x1, :lo12:.L_ffffff800916e19a
            bl printk

            mov w0,wzr
            ldp x29,lr,[sp],#16
            ret
.size close_disambig_0xffffff800870a408_0, . - close_disambig_0xffffff800870a408_0
.size FUN_ffffff800870a408, . - FUN_ffffff800870a408
#-----------------------------------
.type sensor_init, @function
#-----------------------------------
sensor_init:

            sub sp,sp,#112
            stp x28,x27,[sp,#16]
            stp x26,x25,[sp,#32]
            stp x24,x23,[sp,#48]
            stp x22,x21,[sp,#64]
            stp x20,x19,[sp,#80]
            stp x29,lr,[sp,#96]
            add x29,sp,#96
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            adrp x0, .L_ffffff8009184da7
            adrp x1, .L_ffffff80091832e5
            add x0,x0, :lo12:.L_ffffff8009184da7
            add x1,x1, :lo12:.L_ffffff80091832e5
            str x8,[sp,#8]
            bl printk

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7728
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            mov w20,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#769
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            mov w0,#4784
            movk w0,#131,lsl #16
            bl __const_udelay

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w20,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#771
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1027
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3587
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4611
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4867
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7683
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#560
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2352
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4144
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4400
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4656
            movk w8,#65,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4912
            movk w8,#204,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5936
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6192
            movk w8,#218,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6448
            movk w8,#241,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6704
            movk w8,#242,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6960
            movk w8,#150,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7472
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8752
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9008
            movk w8,#176,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10288
            movk w8,#195,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12592
            movk w8,#104,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1845
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2101
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2357
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2613
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2869
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18741
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20021
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20277
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#822
            movk w8,#192,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1078
            movk w8,#44,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1590
            movk w8,#85,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1846
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2614
            movk w8,#109,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3382
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3638
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3894
            movk w8,#68,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8758
            movk w8,#121,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9014
            movk w8,#87,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9270
            movk w8,#80,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11062
            movk w8,#119,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16694
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24630
            movk w8,#192,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w20,#24886
            movk w20,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w20,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w20,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25142
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25398
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25654
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26422
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27190
            movk w8,#84,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27702
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2103
            movk w8,#66,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2359
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6199
            movk w8,#140,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6455
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6711
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9527
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10295
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10807
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11063
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12343
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12599
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12855
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13111
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18487
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26423
            movk w8,#48,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29239
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29495
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29751
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30007
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30263
            movk w8,#49,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30519
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30775
            movk w8,#160,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32567
            movk w8,#49,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36151
            movk w8,#57,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36919
            movk w8,#204,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37175
            movk w8,#204,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39991
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40247
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40503
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40759
            movk w8,#30,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41015
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42039
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45111
            movk w8,#72,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45879
            movk w8,#98,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46647
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47415
            movk w8,#35,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49975
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50743
            movk w8,#53,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51255
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51511
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52279
            movk w8,#147,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4152
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4664
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14136
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34109
            movk w8,#23,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35901
            movk w8,#121,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36157
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63
            movk w8,#80,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34111
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34367
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34623
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40511
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40767
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#23,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#320
            movk w8,#96,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#832
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2112
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3904
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4160
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4416
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4928
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6208
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6464
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6720
            movk w8,#72,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8256
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8768
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9280
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9792
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10304
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10816
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11328
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11840
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12352
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12864
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13376
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13888
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14400
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14912
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15424
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15936
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16448
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16704
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16960
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17216
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17472
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17728
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17984
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18240
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45120
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45376
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45632
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45888
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46144
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46400
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46656
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46912
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21056
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21312
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21568
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21824
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22080
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22336
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22592
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22848
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#578
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#834
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26176
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2373
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1350
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16710
            movk w8,#30,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17734
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#72
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2376
            movk w8,#43,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4680
            movk w8,#43,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4936
            movk w8,#144,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13128
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19272
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20552
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21064
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22088
            movk w8,#88,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22344
            movk w8,#170,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25160
            movk w8,#10,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26440
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26952
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27208
            movk w8,#106,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28232
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28488
            movk w8,#85,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30024
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1355
            movk w8,#147,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1611
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#332
            movk w8,#95,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#77
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#333
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#589
            movk w8,#183,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#845
            movk w8,#202,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1101
            movk w8,#48,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1357
            movk w8,#29,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#79
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#847
            movk w8,#80,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1103
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1359
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2127
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2383
            movk w8,#96,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2639
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2895
            movk w8,#48,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5199
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5455
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5711
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5967
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6223
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6479
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6735
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8271
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8527
            movk w8,#208,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            movk w8,#155,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#336
            movk w8,#78,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#592
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7504
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8272
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8784
            movk w8,#145,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9040
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12368
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22096
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25424
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#82
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#594
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1106
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2386
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3666
            movk w8,#49,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32850
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#84
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#340
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1109
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1365
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1621
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1877
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3157
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3413
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3669
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3925
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5205
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5461
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5717
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5973
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7253
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7509
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7765
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8021
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9301
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9557
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9813
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10069
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11349
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11605
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11861
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12117
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13397
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13653
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13909
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14165
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15445
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15701
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15957
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16213
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17493
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17749
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18005
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18261
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19541
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19797
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20053
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20309
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21589
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21845
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22101
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22357
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23637
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23893
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24149
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24405
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25685
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25941
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26197
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26453
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27733
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27989
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28245
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28501
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29781
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30037
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30293
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30549
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31829
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32085
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32341
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32597
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33877
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34133
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34389
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34645
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35925
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36181
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36437
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36693
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37973
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38229
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38485
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38741
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40021
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40277
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40533
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40789
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42069
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42325
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42581
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42837
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44117
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44373
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44629
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44885
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46165
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46421
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46677
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46933
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48213
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48469
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48725
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48981
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50261
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50517
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50773
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51029
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52309
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53077
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54357
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54613
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54869
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55125
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56405
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56661
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56917
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57173
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58453
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58709
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58965
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59221
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60501
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60757
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61013
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61269
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62549
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62805
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63061
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63317
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64597
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64853
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#65109
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#65365
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#86
            movk w8,#48,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3926
            movk w8,#252,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4182
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4438
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12118
            movk w8,#252,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12374
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12630
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20310
            movk w8,#252,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20566
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20822
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28758
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29014
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37974
            movk w8,#192,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38230
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38486
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38742
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38998
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39254
            movk w8,#112,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39510
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39766
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40022
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40278
            movk w8,#104,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40534
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40790
            movk w8,#104,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41302
            movk w8,#255,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55643
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1117
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1373
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1629
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1885
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4701
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4957
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5213
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5469
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5725
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5981
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6237
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6493
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6749
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7005
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7261
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7517
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7773
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8029
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8285
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8541
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8797
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9053
            movk w8,#160,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10333
            movk w8,#192,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10589
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10845
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11101
            movk w8,#112,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11357
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11613
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11869
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12125
            movk w8,#104,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12381
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12637
            movk w8,#104,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12893
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14429
            movk w8,#112,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32860
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33116
            movk w8,#144,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33372
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33628
            movk w8,#95,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34140
            movk w8,#108,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#342
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#598
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#854
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1110
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1366
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1622
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32859
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33115
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33371
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33627
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33883
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34139
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34395
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w28,#34651
            movk w28,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w28,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34907
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w20,#35163
            movk w20,#40,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w20,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35419
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w21,#35675
            movk w21,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w21,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35931
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36187
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36443
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w22,#36699
            movk w22,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w22,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w23,#36955
            movk w23,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w23,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w24,#37211
            movk w24,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w24,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37467
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37723
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37979
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w25,#38235
            movk w25,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w25,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w26,#38491
            movk w26,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w26,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w27,#38747
            movk w27,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w27,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40
            movk w8,#96,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#296
            movk w8,#76,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#552
            movk w8,#69,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#808
            movk w8,#65,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1064
            movk w8,#59,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1320
            movk w8,#58,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1576
            movk w8,#57,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1832
            movk w8,#55,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2088
            movk w8,#54,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2344
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2600
            movk w8,#59,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2856
            movk w8,#61,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3112
            movk w8,#67,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3368
            movk w8,#74,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3624
            movk w8,#82,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3880
            movk w8,#97,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4136
            movk w8,#71,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4392
            movk w8,#55,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4648
            movk w8,#51,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4904
            movk w8,#46,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5160
            movk w8,#43,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5416
            movk w8,#39,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5672
            movk w8,#38,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5928
            movk w8,#38,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6184
            movk w8,#38,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6440
            movk w8,#39,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6696
            movk w8,#41,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6952
            movk w8,#44,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7208
            movk w8,#48,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7464
            movk w8,#53,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7720
            movk w8,#59,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7976
            movk w8,#76,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8232
            movk w8,#58,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8488
            movk w8,#45,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8744
            movk w8,#41,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9000
            movk w8,#36,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9256
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9512
            movk w8,#29,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9768
            movk w8,#27,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10024
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10280
            movk w8,#27,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10536
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10792
            movk w8,#30,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11048
            movk w8,#33,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11304
            movk w8,#38,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11560
            movk w8,#43,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11816
            movk w8,#48,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12072
            movk w8,#63,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12328
            movk w8,#47,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12584
            movk w8,#36,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12840
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13096
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13352
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13608
            movk w8,#20,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13864
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14120
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14376
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14632
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14888
            movk w8,#21,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15144
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15400
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15656
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15912
            movk w8,#39,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16168
            movk w8,#52,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16424
            movk w8,#40,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16680
            movk w8,#29,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16936
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17192
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17448
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17704
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17960
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18216
            movk w8,#10,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18472
            movk w8,#10,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18728
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18984
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19240
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19496
            movk w8,#21,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19752
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20008
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20264
            movk w8,#44,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20520
            movk w8,#33,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20776
            movk w8,#23,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21032
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21288
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21544
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21800
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22056
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22312
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22568
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22824
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23080
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23336
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23592
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23848
            movk w8,#21,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24104
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24360
            movk w8,#38,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24616
            movk w8,#29,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24872
            movk w8,#20,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25128
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25384
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25640
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25896
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26152
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26408
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26664
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26920
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27176
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27432
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27688
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27944
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28200
            movk w8,#23,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28456
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28712
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28968
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29224
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29480
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29736
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29992
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30248
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30504
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30760
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31016
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31272
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31528
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31784
            movk w8,#10,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32040
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32296
            movk w8,#21,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32552
            movk w8,#33,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32808
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33064
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33320
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33576
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33832
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34088
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34344
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34600
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34856
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35112
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35368
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35624
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35880
            movk w8,#10,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36136
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36392
            movk w8,#21,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36648
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36904
            movk w8,#30,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37160
            movk w8,#21,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37416
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37672
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37928
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38184
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38440
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38696
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38952
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39208
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39464
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39720
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39976
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40232
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40488
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40744
            movk w8,#33,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41000
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41256
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41512
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41768
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42024
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42280
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42536
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42792
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43048
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43304
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43560
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43816
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44072
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44328
            movk w8,#21,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44584
            movk w8,#27,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44840
            movk w8,#38,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45096
            movk w8,#41,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45352
            movk w8,#30,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45608
            movk w8,#25,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45864
            movk w8,#20,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46120
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46376
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46632
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46888
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47144
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47400
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47656
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47912
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48168
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48424
            movk w8,#27,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48680
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48936
            movk w8,#45,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49192
            movk w8,#48,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49448
            movk w8,#38,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49704
            movk w8,#33,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49960
            movk w8,#27,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50216
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50472
            movk w8,#21,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50728
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50984
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51240
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51496
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51752
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52008
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52264
            movk w8,#29,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52520
            movk w8,#35,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52776
            movk w8,#40,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53032
            movk w8,#51,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53288
            movk w8,#60,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53544
            movk w8,#47,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53800
            movk w8,#42,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54056
            movk w8,#38,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54312
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54568
            movk w8,#30,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54824
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55080
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55336
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55592
            movk w8,#29,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55848
            movk w8,#31,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56104
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56360
            movk w8,#39,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56616
            movk w8,#44,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56872
            movk w8,#49,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57128
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57384
            movk w8,#74,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57640
            movk w8,#58,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57896
            movk w8,#52,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58152
            movk w8,#47,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58408
            movk w8,#44,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58664
            movk w8,#40,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58920
            movk w8,#39,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59176
            movk w8,#38,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59432
            movk w8,#39,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59688
            movk w8,#39,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59944
            movk w8,#41,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60200
            movk w8,#45,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60456
            movk w8,#49,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60712
            movk w8,#55,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60968
            movk w8,#61,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61224
            movk w8,#78,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61480
            movk w8,#92,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61736
            movk w8,#77,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61992
            movk w8,#70,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62248
            movk w8,#61,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62504
            movk w8,#58,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62760
            movk w8,#54,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63016
            movk w8,#53,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63272
            movk w8,#51,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63528
            movk w8,#52,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63784
            movk w8,#53,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64040
            movk w8,#55,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64296
            movk w8,#58,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64552
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64808
            movk w8,#72,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#65064
            movk w8,#80,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#65320
            movk w8,#100,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41
            movk w8,#116,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#297
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#553
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#809
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1065
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1321
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1577
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1833
            movk w8,#121,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2089
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2345
            movk w8,#122,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2601
            movk w8,#121,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2857
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3113
            movk w8,#122,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3369
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3625
            movk w8,#121,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3881
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4137
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4393
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4649
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4905
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5161
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5417
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5673
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5929
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6185
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6441
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6697
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6953
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7209
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7465
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7721
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7977
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8233
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8489
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8745
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9001
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9257
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9513
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9769
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10025
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10281
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10537
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10793
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11049
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11305
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11561
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11817
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12073
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12329
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12585
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12841
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13097
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13353
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13609
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13865
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14121
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14377
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14633
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14889
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15145
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15401
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15657
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15913
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16169
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16425
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16681
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16937
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17193
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17449
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17705
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17961
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18217
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18473
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18729
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18985
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19241
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19497
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19753
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20009
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20265
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20521
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20777
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21033
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21289
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21545
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21801
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22057
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22313
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22569
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22825
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23081
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23337
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23593
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23849
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24105
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24361
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24617
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24873
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25129
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25385
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25641
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25897
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26153
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26409
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26665
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26921
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27177
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27433
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27689
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27945
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28201
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28457
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28713
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28969
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29225
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29481
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29737
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29993
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30249
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30505
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30761
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31017
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31273
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31529
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31785
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32041
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32297
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32553
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32809
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33065
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33321
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33577
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33833
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34089
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34345
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34601
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34857
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35113
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35369
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35625
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35881
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36137
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36393
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36649
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36905
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37161
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37417
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37673
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37929
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38185
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38441
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38697
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38953
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39209
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39465
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39721
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39977
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40233
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40489
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40745
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41001
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41257
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41513
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41769
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42025
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42281
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42537
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42793
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43049
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43305
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43561
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43817
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44073
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44329
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44585
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44841
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45097
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45353
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45609
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45865
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46121
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46377
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46633
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46889
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47145
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47401
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47657
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47913
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48169
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48425
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48681
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48937
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49193
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49449
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49705
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49961
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50217
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50473
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50729
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50985
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51241
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51497
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51753
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52009
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52265
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52521
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52777
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53033
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53289
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53545
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53801
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54057
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54313
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54569
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54825
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55081
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55337
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55593
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55849
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56105
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56361
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56617
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56873
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57129
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57385
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57641
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57897
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58153
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58409
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58665
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58921
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59177
            movk w8,#122,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59433
            movk w8,#122,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59689
            movk w8,#123,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59945
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60201
            movk w8,#124,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60457
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60713
            movk w8,#125,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60969
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61225
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61481
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61737
            movk w8,#134,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61993
            movk w8,#134,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62249
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62505
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62761
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63017
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63273
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63529
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63785
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64041
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64297
            movk w8,#135,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64553
            movk w8,#135,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64809
            movk w8,#136,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#65065
            movk w8,#136,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#65321
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#298
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#554
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#810
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1066
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1322
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1578
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1834
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2090
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2346
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2602
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2858
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3114
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3370
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3626
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3882
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4138
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4394
            movk w8,#134,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4650
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4906
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5162
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5418
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5674
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5930
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6186
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6442
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6698
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6954
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7210
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7466
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7722
            movk w8,#134,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#7978
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8234
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8490
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8746
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9002
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9258
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9514
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9770
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10026
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10282
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10538
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10794
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11050
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11306
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11562
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11818
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12074
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12330
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12586
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12842
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13098
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13354
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13610
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13866
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14122
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14378
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14634
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14890
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15146
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15402
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15658
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15914
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16170
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16426
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16682
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16938
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17194
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17450
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17706
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17962
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18218
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18474
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18730
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18986
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19242
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19498
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19754
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20010
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20266
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20522
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20778
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21034
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21290
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21546
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21802
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22058
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22314
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22570
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22826
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23082
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23338
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23594
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23850
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24106
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24362
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24618
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24874
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25130
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25386
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25642
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25898
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26154
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26410
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26666
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26922
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27178
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27434
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27690
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27946
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28202
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28458
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28714
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28970
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29226
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29482
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29738
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29994
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30250
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30506
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30762
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31018
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31274
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31530
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31786
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32042
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32298
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32554
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32810
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33066
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33322
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33578
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33834
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34090
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34346
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34602
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34858
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35114
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35370
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35626
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35882
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36138
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36394
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36650
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36906
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37162
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37418
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37674
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37930
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38186
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38442
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38698
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38954
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39210
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39466
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39722
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39978
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40234
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40490
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#40746
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41002
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41258
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41514
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41770
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42026
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42282
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42538
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42794
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43050
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43306
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43562
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43818
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44074
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44330
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44586
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#44842
            movk w8,#126,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45098
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45354
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45610
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45866
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46122
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46378
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46634
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#46890
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47146
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47402
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47658
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47914
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48170
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48426
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48682
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#48938
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49194
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49450
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49706
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49962
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50218
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50474
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50730
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50986
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51242
            movk w8,#127,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51498
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51754
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52010
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52266
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52522
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52778
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53034
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53290
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53546
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53802
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54058
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54314
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54570
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54826
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55082
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55338
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55594
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55850
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56106
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56362
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56618
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56874
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57130
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57386
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57642
            movk w8,#135,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57898
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58154
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58410
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58666
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58922
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59178
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59434
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59690
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59946
            movk w8,#131,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60202
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60458
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60714
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60970
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61226
            movk w8,#130,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61482
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61738
            movk w8,#134,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61994
            movk w8,#134,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62250
            movk w8,#134,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62506
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62762
            movk w8,#135,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63018
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63274
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63530
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63786
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64042
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64298
            movk w8,#132,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64554
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64810
            movk w8,#133,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#65066
            movk w8,#134,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#65322
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1323
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1579
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1835
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2091
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            movk w8,#155,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#592
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34139
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w28,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w20,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w21,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w22,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w23,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w24,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w25,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w26,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            add x0,sp,#4
            mov w1,#3
            str w27,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#336
            movk w8,#78,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6192
            movk w8,#218,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27702
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16694
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10589
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff80087128c4

            ldp x29,lr,[sp,#96]
            ldp x20,x19,[sp,#80]
            ldp x22,x21,[sp,#64]
            ldp x24,x23,[sp,#48]
            ldp x26,x25,[sp,#32]
            ldp x28,x27,[sp,#16]
            add sp,sp,#112
            ret
.L_ffffff80087128c4:

            bl __stack_chk_fail
.size sensor_init, . - sensor_init
#-----------------------------------
.type streaming_control, @function
#-----------------------------------
streaming_control:

            sub sp,sp,#48
            str x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov w19,w0
            and w2,w0,#1
            adrp x0, .L_ffffff80091853d3
            adrp x1, .L_ffffff8009184aa3
            add x0,x0, :lo12:.L_ffffff80091853d3
            add x1,x1, :lo12:.L_ffffff8009184aa3
            str x8,[sp,#8]
            bl printk

            mov w8,#1
            strh w8,[sp,#4]
            tbz w19,#0,.L_ffffff800871291c

            adrp x9, .L_ffffff8009907a30
            ldrb w2,[x9,:lo12:.L_ffffff8009907a30]
            strh w8,[sp,#6]
            b .L_ffffff8008712928
.L_ffffff800871291c:

            adrp x8, .L_ffffff8009907a30
            ldrb w2,[x8,:lo12:.L_ffffff8009907a30]
            strh wzr,[sp,#6]
.L_ffffff8008712928:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008712b38

            ldp x29,lr,[sp,#32]
            ldr x19,[sp,#16]
            add sp,sp,#48
            ret
.L_ffffff8008712b38:

            bl __stack_chk_fail
.size streaming_control, . - streaming_control
#-----------------------------------
.type FUN_ffffff8008712b3c, @function
#-----------------------------------
FUN_ffffff8008712b3c:
#-----------------------------------
.type preview_setting_disambig_0xffffff8008712b3c_0, @function
#-----------------------------------
preview_setting_disambig_0xffffff8008712b3c_0:

            sub sp,sp,#48
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            adrp x19, .L_ffffff8009907a30
            add x0,sp,#4
            mov w1,#3
            str x8,[sp,#8]
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#515
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24886
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24886
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1350
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16454
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16710
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16710
            movk w8,#30,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17222
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17734
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4936
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4936
            movk w8,#144,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28232
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28232
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#515
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3843
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#309
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#565
            movk w8,#114,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9783
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10039
            movk w8,#68,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10551
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12087
            movk w8,#137,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24631
            movk w8,#144,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41271
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42295
            movk w8,#170,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41527
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41783
            movk w8,#66,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#312
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#568
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#824
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1080
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1336
            movk w8,#95,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1592
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1848
            movk w8,#199,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2104
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2360
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2616
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2872
            movk w8,#216,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3128
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3384
            movk w8,#208,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3640
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3896
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4408
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4920
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5176
            movk w8,#49,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5432
            movk w8,#49,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8248
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8504
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13880
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16696
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1600
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1856
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2368
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20544
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20800
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#325
            movk w8,#60,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14152
            movk w8,#20,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#338
            movk w8,#98,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#850
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1362
            movk w8,#72,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#85
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#341
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#597
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#853
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2133
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2389
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2645
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2901
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4181
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4437
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4693
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4949
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6229
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6485
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6741
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6997
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8277
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8533
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8789
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9045
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10325
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10581
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10837
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11093
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12373
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12629
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12885
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13141
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14421
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14677
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14933
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15189
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16469
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16725
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16981
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17237
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18517
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18773
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19029
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19285
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20565
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20821
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21077
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21333
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22613
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22869
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23125
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23381
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24661
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24917
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25173
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25429
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26709
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26965
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27221
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27477
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28757
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29013
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29269
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29525
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30805
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31061
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31317
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31573
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32853
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33109
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33365
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33621
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34901
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35157
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35413
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35669
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36949
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37205
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37461
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37717
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38997
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39253
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39509
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39765
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41045
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41301
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41557
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41813
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43093
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43349
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43605
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43861
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45141
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45397
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45653
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45909
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47189
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47445
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47701
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47957
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49237
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49493
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49749
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50005
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51285
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51541
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51797
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52053
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53333
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53589
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53845
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54101
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55381
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55637
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55893
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56149
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57429
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57685
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57941
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58197
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59477
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59733
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59989
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60245
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61525
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61781
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62037
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62293
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63573
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63829
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64085
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64341
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36950
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37206
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37462
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37718
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9309
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10077
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14941
            movk w8,#80,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x8, .L_ffffff8009907990
            ldr w8,[x8,:lo12:.L_ffffff8009907990]
            adrp x9, .L_ffffff8009c7b648
            ldr w20,[x9,:lo12:.L_ffffff8009c7b648]
            mov w9,#8248
            cmp w8,#1
            strh w9,[sp,#4]
            b.ne .L_ffffff80087140e8

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#197
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff800871411c

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6
            b .L_ffffff8008714124
.L_ffffff80087140e8:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#129
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff8008714144

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6
            b .L_ffffff800871414c
.L_ffffff800871411c:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2
.L_ffffff8008714124:

            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#87,lsl #16
            b .L_ffffff8008714168
.L_ffffff8008714144:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2
.L_ffffff800871414c:

            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#23,lsl #16
.L_ffffff8008714168:

            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            movk w8,#155,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#592
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34139
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34651
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35163
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35675
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36699
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36955
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37211
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38235
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38491
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38747
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#336
            movk w8,#74,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6192
            movk w8,#90,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27702
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16694
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10589
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2563
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4355
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1590
            movk w8,#85,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1846
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2870
            movk w8,#72,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3126
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8247
            movk w8,#136,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008714428

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            add sp,sp,#48
            ret
.L_ffffff8008714428:

            bl __stack_chk_fail
.size preview_setting_disambig_0xffffff8008712b3c_0, . - preview_setting_disambig_0xffffff8008712b3c_0
.size FUN_ffffff8008712b3c, . - FUN_ffffff8008712b3c
#-----------------------------------
.type normal_capture_setting, @function
#-----------------------------------
normal_capture_setting:

            sub sp,sp,#48
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            adrp x0, .L_ffffff80091854e8
            adrp x1, .L_ffffff8009185508
            add x0,x0, :lo12:.L_ffffff80091854e8
            add x1,x1, :lo12:.L_ffffff8009185508
            str x8,[sp,#8]
            bl printk

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#515
            movk w8,#60,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3843
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#309
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#565
            movk w8,#234,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9783
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10039
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10551
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12087
            movk w8,#146,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24631
            movk w8,#146,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41271
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42295
            movk w8,#150,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41527
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41783
            movk w8,#35,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#312
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#568
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#824
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1080
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1336
            movk w8,#95,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1592
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1848
            movk w8,#199,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2104
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2360
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2616
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2872
            movk w8,#176,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3128
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3384
            movk w8,#160,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3640
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3896
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4152
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4408
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4664
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4920
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5176
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5432
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8248
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8504
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13880
            movk w8,#20,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16696
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1600
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1856
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2368
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20544
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20800
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#325
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14152
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#338
            movk w8,#113,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#850
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1362
            movk w8,#144,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#85
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#341
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#597
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#853
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2133
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2389
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2645
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2901
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4181
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4437
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4693
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4949
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6229
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6485
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6741
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6997
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8277
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8533
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8789
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9045
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10325
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10581
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10837
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11093
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12373
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12629
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12885
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13141
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14421
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14677
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14933
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15189
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16469
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16725
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16981
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17237
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18517
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18773
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19029
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19285
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21077
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21333
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22613
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22869
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23125
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23381
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24661
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24917
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25173
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25429
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26709
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26965
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27221
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27477
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28757
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29013
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29269
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29525
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30805
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31061
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31317
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31573
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32853
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33109
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33365
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33621
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34901
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35157
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35413
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35669
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36949
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37205
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37461
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37717
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38997
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39253
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39509
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39765
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41045
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41301
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41557
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41813
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43093
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43349
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43605
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43861
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45141
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45397
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45653
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45909
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47189
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47445
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47701
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47957
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49237
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49493
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49749
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50005
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51285
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51541
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51797
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52053
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53333
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53589
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53845
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54101
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55381
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55637
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55893
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56149
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57429
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57685
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57941
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58197
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59477
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59733
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59989
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60245
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61525
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61781
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62037
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62293
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63573
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63829
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64085
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64341
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36950
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37206
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37462
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37718
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9309
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10077
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14941
            movk w8,#88,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x8, .L_ffffff8009907990
            ldr w8,[x8,:lo12:.L_ffffff8009907990]
            adrp x9, .L_ffffff8009c7b648
            ldr w20,[x9,:lo12:.L_ffffff8009c7b648]
            mov w9,#8248
            cmp w8,#1
            strh w9,[sp,#4]
            b.ne .L_ffffff8008715754

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#196
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff800871578c

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4
            strh w8,[sp,#6]
            b .L_ffffff8008715794
.L_ffffff8008715754:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#128
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff80087157b0

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4
            strh w8,[sp,#6]
            b .L_ffffff80087157b8
.L_ffffff800871578c:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            strh wzr,[sp,#6]
.L_ffffff8008715794:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#87,lsl #16
            b .L_ffffff80087157d0
.L_ffffff80087157b0:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            strh wzr,[sp,#6]
.L_ffffff80087157b8:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#23,lsl #16
.L_ffffff80087157d0:

            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            movk w8,#155,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#592
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34139
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34651
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35163
            movk w8,#40,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35675
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36699
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36955
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37211
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38235
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38491
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38747
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#336
            movk w8,#78,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6192
            movk w8,#218,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27702
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16694
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10589
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2563
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4355
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1590
            movk w8,#85,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1846
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2870
            movk w8,#72,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3126
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8247
            movk w8,#136,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x8, .L_ffffff8009c7b664
            ldrb w8,[x8,:lo12:.L_ffffff8009c7b664]
            cmp w8,#1
            b.ne .L_ffffff8008715c10

            adrp x0, .L_ffffff800918551f
            adrp x1, .L_ffffff8009185508
            add x0,x0, :lo12:.L_ffffff800918551f
            add x1,x1, :lo12:.L_ffffff8009185508
            bl printk

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#515
            movk w8,#61,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16694
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24886
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1350
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16454
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16710
            movk w8,#35,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17222
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17734
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4936
            movk w8,#152,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28232
            movk w8,#7,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#336
            movk w8,#74,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10589
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            mov x19,#-60
.L_ffffff8008715bfc:

            mov w0,#35160
            movk w0,#65,lsl #16
            bl __const_udelay

            adds x19,x19,#1
            b.lo .L_ffffff8008715bfc
.L_ffffff8008715c10:

            adrp x0, .L_ffffff8009185553
            adrp x1, .L_ffffff8009185508
            add x0,x0, :lo12:.L_ffffff8009185553
            add x1,x1, :lo12:.L_ffffff8009185508
            bl printk

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008715c48

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            add sp,sp,#48
            ret
.L_ffffff8008715c48:

            bl __stack_chk_fail
.size normal_capture_setting, . - normal_capture_setting
#-----------------------------------
.type pip_capture_setting, @function
#-----------------------------------
pip_capture_setting:

            sub sp,sp,#48
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            adrp x0, .L_ffffff8009185575
            adrp x1, .L_ffffff80091855ac
            add x0,x0, :lo12:.L_ffffff8009185575
            add x1,x1, :lo12:.L_ffffff80091855ac
            str x8,[sp,#8]
            bl printk

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#515
            movk w8,#60,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3843
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#309
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#565
            movk w8,#234,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9783
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10039
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10551
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12087
            movk w8,#146,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24631
            movk w8,#146,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41271
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42295
            movk w8,#150,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41527
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41783
            movk w8,#35,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#312
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#568
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#824
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1080
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1336
            movk w8,#95,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1592
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1848
            movk w8,#199,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2104
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2360
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2616
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2872
            movk w8,#176,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3128
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3384
            movk w8,#160,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3640
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3896
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4408
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4920
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5176
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5432
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8248
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8504
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13880
            movk w8,#20,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16696
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1600
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1856
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2368
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20544
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20800
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#325
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14152
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#338
            movk w8,#113,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#850
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1362
            movk w8,#144,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#85
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#341
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#597
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#853
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2133
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2389
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2645
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2901
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4181
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4437
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4693
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4949
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6229
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6485
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6741
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6997
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8277
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8533
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8789
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9045
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10325
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10581
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10837
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11093
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12373
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12629
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12885
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13141
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14421
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14677
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14933
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15189
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16469
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16725
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16981
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17237
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18517
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18773
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19029
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19285
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21077
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21333
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22613
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22869
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23125
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23381
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24661
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24917
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25173
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25429
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26709
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26965
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27221
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27477
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28757
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29013
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29269
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29525
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30805
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31061
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31317
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31573
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32853
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33109
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33365
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33621
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34901
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35157
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35413
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35669
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36949
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37205
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37461
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37717
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38997
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39253
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39509
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39765
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41045
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41301
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41557
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41813
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43093
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43349
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43605
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43861
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45141
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45397
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45653
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45909
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47189
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47445
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47701
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47957
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49237
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49493
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49749
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50005
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51285
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51541
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51797
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52053
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53333
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53589
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53845
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54101
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55381
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55637
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55893
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56149
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57429
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57685
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57941
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58197
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59477
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59733
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59989
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60245
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61525
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61781
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62037
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62293
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63573
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63829
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64085
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64341
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36950
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37206
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37462
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37718
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9309
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10077
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14941
            movk w8,#88,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x8, .L_ffffff8009907990
            ldr w8,[x8,:lo12:.L_ffffff8009907990]
            adrp x9, .L_ffffff8009c7b648
            ldr w20,[x9,:lo12:.L_ffffff8009c7b648]
            mov w9,#8248
            cmp w8,#1
            strh w9,[sp,#4]
            b.ne .L_ffffff8008716f44

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#196
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff8008716f7c

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4
            strh w8,[sp,#6]
            b .L_ffffff8008716f84
.L_ffffff8008716f44:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#128
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff8008716fa0

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4
            strh w8,[sp,#6]
            b .L_ffffff8008716fa8
.L_ffffff8008716f7c:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            strh wzr,[sp,#6]
.L_ffffff8008716f84:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#87,lsl #16
            b .L_ffffff8008716fc0
.L_ffffff8008716fa0:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            strh wzr,[sp,#6]
.L_ffffff8008716fa8:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#23,lsl #16
.L_ffffff8008716fc0:

            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            movk w8,#155,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#592
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34139
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34651
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35163
            movk w8,#40,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35675
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36699
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36955
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37211
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38235
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38491
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38747
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#336
            movk w8,#78,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6192
            movk w8,#218,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27702
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16694
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10589
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2563
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4355
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1590
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1846
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2870
            movk w8,#79,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3126
            movk w8,#31,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8247
            movk w8,#68,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008717294

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            add sp,sp,#48
            ret
.L_ffffff8008717294:

            bl __stack_chk_fail
.size pip_capture_setting, . - pip_capture_setting
#-----------------------------------
.type pip_capture_15fps_setting, @function
#-----------------------------------
pip_capture_15fps_setting:

            sub sp,sp,#48
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            adrp x0, .L_ffffff8009185575
            adrp x1, .L_ffffff80091855c0
            add x0,x0, :lo12:.L_ffffff8009185575
            add x1,x1, :lo12:.L_ffffff80091855c0
            str x8,[sp,#8]
            bl printk

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#515
            movk w8,#60,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3843
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#309
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#565
            movk w8,#234,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9783
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10039
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10551
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12087
            movk w8,#146,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24631
            movk w8,#146,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41271
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42295
            movk w8,#150,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41527
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41783
            movk w8,#35,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#312
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#568
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#824
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1080
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1336
            movk w8,#95,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1592
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1848
            movk w8,#199,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2104
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2360
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2616
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2872
            movk w8,#176,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3128
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3384
            movk w8,#160,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3640
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3896
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4408
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4920
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5176
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5432
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8248
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8504
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13880
            movk w8,#20,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16696
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1600
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1856
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2368
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20544
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20800
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#325
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14152
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#338
            movk w8,#113,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#850
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1362
            movk w8,#144,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#85
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#341
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#597
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#853
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2133
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2389
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2645
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2901
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4181
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4437
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4693
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4949
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6229
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6485
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6741
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6997
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8277
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8533
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8789
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9045
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10325
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10581
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10837
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11093
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12373
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12629
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12885
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13141
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14421
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14677
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14933
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15189
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16469
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16725
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16981
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17237
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18517
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18773
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19029
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19285
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21077
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21333
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22613
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22869
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23125
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23381
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24661
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24917
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25173
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25429
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26709
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26965
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27221
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27477
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28757
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29013
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29269
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29525
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30805
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31061
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31317
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31573
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32853
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33109
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33365
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33621
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34901
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35157
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35413
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35669
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36949
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37205
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37461
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37717
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38997
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39253
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39509
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39765
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41045
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41301
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41557
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41813
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43093
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43349
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43605
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43861
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45141
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45397
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45653
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45909
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47189
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47445
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47701
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47957
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49237
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49493
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49749
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50005
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51285
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51541
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51797
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52053
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53333
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53589
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53845
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54101
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55381
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55637
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55893
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56149
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57429
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57685
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57941
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58197
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59477
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59733
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59989
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60245
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61525
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61781
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62037
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62293
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63573
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63829
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64085
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64341
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36950
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37206
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37462
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37718
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9309
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10077
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14941
            movk w8,#88,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x8, .L_ffffff8009907990
            ldr w8,[x8,:lo12:.L_ffffff8009907990]
            adrp x9, .L_ffffff8009c7b648
            ldr w20,[x9,:lo12:.L_ffffff8009c7b648]
            mov w9,#8248
            cmp w8,#1
            strh w9,[sp,#4]
            b.ne .L_ffffff8008718590

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#196
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff80087185c8

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4
            strh w8,[sp,#6]
            b .L_ffffff80087185d0
.L_ffffff8008718590:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#128
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff80087185ec

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4
            strh w8,[sp,#6]
            b .L_ffffff80087185f4
.L_ffffff80087185c8:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            strh wzr,[sp,#6]
.L_ffffff80087185d0:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#87,lsl #16
            b .L_ffffff800871860c
.L_ffffff80087185ec:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            strh wzr,[sp,#6]
.L_ffffff80087185f4:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#23,lsl #16
.L_ffffff800871860c:

            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            movk w8,#155,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#592
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34139
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34651
            movk w8,#22,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35163
            movk w8,#40,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35675
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36699
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36955
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37211
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38235
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38491
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38747
            movk w8,#26,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#336
            movk w8,#78,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6192
            movk w8,#218,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27702
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16694
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10589
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2563
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4355
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1590
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1846
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2870
            movk w8,#79,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3126
            movk w8,#31,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8247
            movk w8,#68,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff80087188e0

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            add sp,sp,#48
            ret
.L_ffffff80087188e0:

            bl __stack_chk_fail
.size pip_capture_15fps_setting, . - pip_capture_15fps_setting
#-----------------------------------
.type normal_video_setting, @function
#-----------------------------------
normal_video_setting:

            sub sp,sp,#48
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            and w2,w0,#65535
            adrp x0, .L_ffffff80091855da
            adrp x1, .L_ffffff8009184be4
            add x0,x0, :lo12:.L_ffffff80091855da
            add x1,x1, :lo12:.L_ffffff8009184be4
            str x8,[sp,#8]
            bl printk

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#515
            movk w8,#60,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3843
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#309
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#565
            movk w8,#234,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9783
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10039
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10551
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12087
            movk w8,#146,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24631
            movk w8,#146,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41271
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42295
            movk w8,#150,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41527
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41783
            movk w8,#35,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#312
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#568
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#824
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1080
            movk w8,#18,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1336
            movk w8,#95,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1592
            movk w8,#13,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1848
            movk w8,#199,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2104
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2360
            movk w8,#176,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2616
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2872
            movk w8,#100,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3128
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3384
            movk w8,#160,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3640
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3896
            movk w8,#240,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4408
            movk w8,#216,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4664
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4920
            movk w8,#46,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5176
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5432
            movk w8,#17,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8248
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8504
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13880
            movk w8,#20,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16696
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1600
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1856
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2368
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20544
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20800
            movk w8,#15,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#325
            movk w8,#56,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14152
            movk w8,#11,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#338
            movk w8,#113,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#850
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1362
            movk w8,#144,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#85
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#341
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#597
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#853
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2133
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2389
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2645
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2901
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4181
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4437
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4693
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4949
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6229
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6485
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6741
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6997
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8277
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8533
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8789
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9045
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10325
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10581
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10837
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11093
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12373
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12629
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12885
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13141
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14421
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14677
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14933
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15189
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16469
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16725
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16981
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17237
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18517
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18773
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19029
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19285
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21077
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21333
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22613
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22869
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23125
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23381
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24661
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24917
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25173
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25429
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26709
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26965
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27221
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27477
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28757
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29013
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29269
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29525
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30805
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31061
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31317
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31573
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32853
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33109
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33365
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33621
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34901
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35157
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35413
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35669
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36949
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37205
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37461
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37717
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38997
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39253
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39509
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39765
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41045
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41301
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41557
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41813
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43093
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43349
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43605
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43861
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45141
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45397
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45653
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45909
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47189
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47445
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47701
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47957
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49237
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49493
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49749
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50005
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51285
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51541
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51797
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52053
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53333
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53589
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53845
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54101
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55381
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55637
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55893
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56149
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57429
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57685
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57941
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58197
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59477
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59733
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59989
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60245
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61525
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61781
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62037
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62293
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63573
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63829
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64085
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64341
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36950
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37206
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37462
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37718
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9309
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9565
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9821
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10077
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14941
            movk w8,#88,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x8, .L_ffffff8009907990
            ldr w8,[x8,:lo12:.L_ffffff8009907990]
            adrp x9, .L_ffffff8009c7b648
            ldr w20,[x9,:lo12:.L_ffffff8009c7b648]
            mov w9,#8248
            cmp w8,#1
            strh w9,[sp,#4]
            b.ne .L_ffffff8008719bfc

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#196
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff8008719c34

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4
            strh w8,[sp,#6]
            b .L_ffffff8008719c3c
.L_ffffff8008719bfc:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#128
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff8008719c58

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4
            strh w8,[sp,#6]
            b .L_ffffff8008719c60
.L_ffffff8008719c34:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            strh wzr,[sp,#6]
.L_ffffff8008719c3c:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#87,lsl #16
            b .L_ffffff8008719c78
.L_ffffff8008719c58:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            strh wzr,[sp,#6]
.L_ffffff8008719c60:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#23,lsl #16
.L_ffffff8008719c78:

            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            movk w8,#155,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#592
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34139
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34651
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35163
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35675
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36699
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36955
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37211
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38235
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38491
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38747
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#336
            movk w8,#74,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6192
            movk w8,#90,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27702
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16694
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10589
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2563
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4355
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1590
            movk w8,#85,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1846
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2870
            movk w8,#72,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3126
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8247
            movk w8,#136,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x0, .L_ffffff8009185553
            adrp x1, .L_ffffff8009184be4
            add x0,x0, :lo12:.L_ffffff8009185553
            add x1,x1, :lo12:.L_ffffff8009184be4
            bl printk

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008719f4c

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            add sp,sp,#48
            ret
.L_ffffff8008719f4c:

            bl __stack_chk_fail
.size normal_video_setting, . - normal_video_setting
#-----------------------------------
.type hs_video_setting, @function
#-----------------------------------
hs_video_setting:

            sub sp,sp,#48
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            adrp x0, .L_ffffff8009184da7
            adrp x1, .L_ffffff8009184c02
            add x0,x0, :lo12:.L_ffffff8009184da7
            add x1,x1, :lo12:.L_ffffff8009184c02
            str x8,[sp,#8]
            bl printk

            adrp x19, .L_ffffff8009907a30
            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#515
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3843
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#309
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#565
            movk w8,#182,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9783
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10039
            movk w8,#68,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10551
            movk w8,#34,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12087
            movk w8,#137,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24631
            movk w8,#144,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41271
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#42295
            movk w8,#170,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41527
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41783
            movk w8,#66,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#312
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#568
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#824
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1080
            movk w8,#14,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1336
            movk w8,#63,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1592
            movk w8,#9,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1848
            movk w8,#191,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2104
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2360
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2616
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2872
            movk w8,#208,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3128
            movk w8,#19,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3384
            movk w8,#160,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3640
            movk w8,#3,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3896
            movk w8,#188,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4152
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4408
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4664
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4920
            movk w8,#4,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5176
            movk w8,#49,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#5432
            movk w8,#49,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8248
            movk w8,#129,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8504
            movk w8,#6,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13880
            movk w8,#12,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16696
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1600
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1856
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2368
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20544
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20800
            movk w8,#1,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#325
            movk w8,#60,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14152
            movk w8,#20,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#338
            movk w8,#98,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#850
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1362
            movk w8,#72,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#85
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#341
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#597
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#853
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2133
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2389
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2645
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2901
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4181
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4437
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4693
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4949
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6229
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6485
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6741
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6997
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8277
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8533
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8789
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9045
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10325
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10581
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10837
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#11093
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12373
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12629
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#12885
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#13141
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14421
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14677
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14933
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#15189
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16469
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16725
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16981
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#17237
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18517
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#18773
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19029
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#19285
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20565
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#20821
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21077
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#21333
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22613
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#22869
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23125
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#23381
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24661
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#24917
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25173
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#25429
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26709
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#26965
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27221
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27477
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#28757
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29013
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29269
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#29525
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#30805
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31061
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31317
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#31573
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#32853
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33109
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33365
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#33621
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34901
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35157
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35413
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35669
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36949
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37205
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37461
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37717
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38997
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39253
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39509
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#39765
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41045
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41301
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41557
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#41813
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43093
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43349
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43605
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#43861
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45141
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45397
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45653
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#45909
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47189
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47445
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47701
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#47957
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49237
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49493
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#49749
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#50005
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51285
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51541
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#51797
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#52053
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53333
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53589
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#53845
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#54101
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55381
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55637
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#55893
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#56149
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57429
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57685
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#57941
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#58197
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59477
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59733
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#59989
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#60245
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61525
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#61781
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62037
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#62293
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63573
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#63829
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64085
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64341
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36950
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37206
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37462
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37718
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9309
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9565
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#9821
            movk w8,#2,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10077
            movk w8,#8,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#14941
            movk w8,#88,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x8, .L_ffffff8009907990
            ldr w8,[x8,:lo12:.L_ffffff8009907990]
            adrp x9, .L_ffffff8009c7b648
            ldr w20,[x9,:lo12:.L_ffffff8009c7b648]
            mov w9,#8248
            cmp w8,#1
            strh w9,[sp,#4]
            b.ne .L_ffffff800871b410

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#197
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff800871b444

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6
            b .L_ffffff800871b44c
.L_ffffff800871b410:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#129
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            mov w8,#8504
            cmp w20,#1
            strh w8,[sp,#4]
            b.ne .L_ffffff800871b46c

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6
            b .L_ffffff800871b474
.L_ffffff800871b444:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2
.L_ffffff800871b44c:

            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#87,lsl #16
            b .L_ffffff800871b490
.L_ffffff800871b46c:

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2
.L_ffffff800871b474:

            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#64
            movk w8,#23,lsl #16
.L_ffffff800871b490:

            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#80
            movk w8,#155,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#592
            movk w8,#28,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34139
            movk w8,#16,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#34651
            movk w8,#32,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35163
            movk w8,#64,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#35675
            movk w8,#128,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36699
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#36955
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#37211
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38235
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38491
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#38747
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#336
            movk w8,#74,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#6192
            movk w8,#90,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#27702
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#16694
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#10589
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2563
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#4355
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1590
            movk w8,#85,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#1846
            movk w8,#5,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#2870
            movk w8,#72,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#3126
            movk w8,#24,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907a30]
            mov w8,#8247
            movk w8,#136,lsl #16
            add x0,sp,#4
            mov w1,#3
            str w8,[sp,#4]
            bl iWriteRegI2C

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff800871b750

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            add sp,sp,#48
            ret
.L_ffffff800871b750:

            bl __stack_chk_fail
.size hs_video_setting, . - hs_video_setting
.align 2
#-----------------------------------
.globl selective_read_eeprom
.type selective_read_eeprom, @function
#-----------------------------------
selective_read_eeprom:

            sub sp,sp,#32
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov x2,x1
            lsr w9,w0,#8
            mov w1,#2
            str x8,[sp,#8]
            strb w0,[sp,#5]
            add x0,sp,#4
            mov w3,#1
            mov w4,#160
            strb w9,[sp,#4]
            bl iReadRegI2C

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff800871b7b8

            ldp x29,lr,[sp,#16]
            mvn w8,w0
            lsr w0,w8,#31
            add sp,sp,#32
            ret
.L_ffffff800871b7b8:

            bl __stack_chk_fail
.size selective_read_eeprom, . - selective_read_eeprom
.align 2
#-----------------------------------
.globl read_eeprom
.type read_eeprom, @function
#-----------------------------------
read_eeprom:

            sub sp,sp,#112
            stp x28,x27,[sp,#16]
            stp x26,x25,[sp,#32]
            stp x24,x23,[sp,#48]
            stp x22,x21,[sp,#64]
            stp x20,x19,[sp,#80]
            stp x29,lr,[sp,#96]
            add x29,sp,#96
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov x19,x1
            str x8,[sp,#8]
            adrp x8, .L_ffffff80099f551a
            ldrb w8,[x8,:lo12:.L_ffffff80099f551a]
            tbnz w8,#2,.L_ffffff800871b950
.L_ffffff800871b7f8:

            adrp x25, .L_ffffff8009c7c668
            ldrb w8,[x25,:lo12:.L_ffffff8009c7c668]
            adrp x26, .L_ffffff8009c7c66c
            adrp x27, .L_ffffff8009c7c670
            cmp w8,#1
            b.ne .L_ffffff800871b848

            ldr w8,[x26,:lo12:.L_ffffff8009c7c66c]
            cmp w8,#1372
            b.ne .L_ffffff800871b848

            ldr w8,[x27,:lo12:.L_ffffff8009c7c670]
            cmp w8,#2049
            b.ne .L_ffffff800871b848

            adrp x8, .L_ffffff80099f556a
            ldrb w8,[x8,:lo12:.L_ffffff80099f556a]
            tbz w8,#2,.L_ffffff800871b904

            adrp x0, .L_ffffff80099f5548
            adrp x1, .L_ffffff800918571d
            add x0,x0, :lo12:.L_ffffff80099f5548
            add x1,x1, :lo12:.L_ffffff800918571d
            b .L_ffffff800871b988
.L_ffffff800871b848:

            adrp x20, .L_ffffff8009c7b668
            adrp x22, .L_ffffff800918578f
            mov x28,xzr
            add x20,x20, :lo12:.L_ffffff8009c7b668
            adrp x21, .L_ffffff80099f5592
            add x22,x22, :lo12:.L_ffffff800918578f
.L_ffffff800871b860:

            add x23,x28,#2049
            add x24,x20,x28
            ubfx x8,x23,#8,#24
            add x0,sp,#4
            mov w1,#2
            mov w3,#1
            mov w4,#160
            mov x2,x24
            strb w8,[sp,#4]
            strb w23,[sp,#5]
            bl iReadRegI2C

            tbnz w0,#31,.L_ffffff800871b8cc

            ldrb w8,[x21,:lo12:.L_ffffff80099f5592]
            tbnz w8,#2,.L_ffffff800871b8a8
.L_ffffff800871b898:

            add x28,x28,#1
            cmp x28,#1372
            b.lo .L_ffffff800871b860

            b .L_ffffff800871b8e0
.L_ffffff800871b8a8:

            ldrb w4,[x24]
            adrp x0, .L_ffffff80099f5570
            adrp x1, .L_ffffff800918579c
            add x0,x0, :lo12:.L_ffffff80099f5570
            add x1,x1, :lo12:.L_ffffff800918579c
            mov x2,x22
            mov w3,w23
            bl __dynamic_pr_debug

            b .L_ffffff800871b898
.L_ffffff800871b8cc:

            mov w0,wzr
            strb wzr,[x25,:lo12:.L_ffffff8009c7c668]
            str wzr,[x26,:lo12:.L_ffffff8009c7c66c]
            str wzr,[x27,:lo12:.L_ffffff8009c7c670]
            b .L_ffffff800871b91c
.L_ffffff800871b8e0:

            mov w8,#1
            adrp x9, .L_ffffff80099f5542
            strb w8,[x25,:lo12:.L_ffffff8009c7c668]
            ldrb w8,[x9,:lo12:.L_ffffff80099f5542]
            mov w9,#1372
            str w9,[x26,:lo12:.L_ffffff8009c7c66c]
            mov w9,#2049
            str w9,[x27,:lo12:.L_ffffff8009c7c670]
            tbnz w8,#2,.L_ffffff800871b978
.L_ffffff800871b904:

            adrp x1, .L_ffffff8009c7b668
            add x1,x1, :lo12:.L_ffffff8009c7b668
            mov w2,#1372
            mov x0,x19
            bl __pi_memcpy

            mov w0,#1
.L_ffffff800871b91c:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff800871b998

            ldp x29,lr,[sp,#96]
            ldp x20,x19,[sp,#80]
            ldp x22,x21,[sp,#64]
            ldp x24,x23,[sp,#48]
            ldp x26,x25,[sp,#32]
            ldp x28,x27,[sp,#16]
            add sp,sp,#112
            ret
.L_ffffff800871b950:

            adrp x0, .L_ffffff80099f54f8
            adrp x1, .L_ffffff80091856b3
            adrp x2, .L_ffffff8009185790
            add x0,x0, :lo12:.L_ffffff80099f54f8
            add x1,x1, :lo12:.L_ffffff80091856b3
            add x2,x2, :lo12:.L_ffffff8009185790
            mov w3,#2049
            mov w4,#1372
            bl __dynamic_pr_debug

            b .L_ffffff800871b7f8
.L_ffffff800871b978:

            adrp x0, .L_ffffff80099f5520
            adrp x1, .L_ffffff80091856f2
            add x0,x0, :lo12:.L_ffffff80099f5520
            add x1,x1, :lo12:.L_ffffff80091856f2
.L_ffffff800871b988:

            adrp x2, .L_ffffff8009185790
            add x2,x2, :lo12:.L_ffffff8009185790
            bl __dynamic_pr_debug

            b .L_ffffff800871b904
.L_ffffff800871b998:

            bl __stack_chk_fail
.size read_eeprom, . - read_eeprom
.align 2
#-----------------------------------
.globl OV16880_MIPI_read_otp_wb
.type OV16880_MIPI_read_otp_wb, @function
#-----------------------------------
OV16880_MIPI_read_otp_wb:

            sub sp,sp,#48
            str x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov w9,#6912
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            str x8,[sp,#8]
            strb wzr,[sp]
            strh w9,[sp,#4]
            bl iReadRegI2C

            ldrb w8,[sp]
            cmp w8,#1
            b.ne .L_ffffff800871bba0

            mov w8,#7168
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            mov w19,#1
            bl iReadRegI2C

            mov w8,#7424
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#7680
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#7936
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#8192
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#8448
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#8704
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#8960
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#9216
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#9472
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#9728
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            mov w8,#9984
            add x0,sp,#4
            mov x2,sp
            mov w1,#2
            mov w3,#1
            mov w4,#160
            strb wzr,[sp]
            strh w8,[sp,#4]
            bl iReadRegI2C

            b .L_ffffff800871bbb0
.L_ffffff800871bba0:

            adrp x0, .L_ffffff8009185750
            add x0,x0, :lo12:.L_ffffff8009185750
            bl printk

            mov w19,wzr
.L_ffffff800871bbb0:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff800871bbd8

            mov w0,w19
            ldp x29,lr,[sp,#32]
            ldr x19,[sp,#16]
            add sp,sp,#48
            ret
.L_ffffff800871bbd8:

            bl __stack_chk_fail
.size OV16880_MIPI_read_otp_wb, . - OV16880_MIPI_read_otp_wb
.align 2
#-----------------------------------
.globl ov16880CheckVersion
.type ov16880CheckVersion, @function
#-----------------------------------
ov16880CheckVersion:

            sub sp,sp,#128
            stp x26,x25,[sp,#48]
            stp x24,x23,[sp,#64]
            stp x22,x21,[sp,#80]
            stp x20,x19,[sp,#96]
            stp x29,lr,[sp,#112]
            add x29,sp,#112
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            add x0,sp,#12
            add x2,sp,#8
            mov w1,#2
            mov w3,#1
            mov w4,#160
            str x8,[sp,#40]
            strb wzr,[sp,#8]
            stur xzr,[sp,#31]
            stp xzr,xzr,[sp,#16]
            strh wzr,[sp,#12]
            bl iReadRegI2C

            tbnz w0,#31,.L_ffffff800871bc58

            adrp x8, .L_ffffff80099f5592
            ldrb w8,[x8,:lo12:.L_ffffff80099f5592]
            tbnz w8,#2,.L_ffffff800871bd90
.L_ffffff800871bc3c:

            adrp x8, .L_ffffff8009c7c668
            mov w9,#1
            adrp x10, .L_ffffff8009c7c66c
            adrp x11, .L_ffffff8009c7c670
            strb w9,[x8,:lo12:.L_ffffff8009c7c668]
            str w9,[x10,:lo12:.L_ffffff8009c7c66c]
            str wzr,[x11,:lo12:.L_ffffff8009c7c670]
.L_ffffff800871bc58:

            ldrb w8,[sp,#8]
            cbz w8,.L_ffffff800871bce8

            adrp x19, .L_ffffff80099f5570
            adrp x20, .L_ffffff800918579c
            adrp x21, .L_ffffff800918578f
            mov x24,xzr
            add x25,sp,#16
            adrp x26, .L_ffffff80099f5592
            add x19,x19, :lo12:.L_ffffff80099f5570
            add x20,x20, :lo12:.L_ffffff800918579c
            add x21,x21, :lo12:.L_ffffff800918578f
.L_ffffff800871bc84:

            add x22,x24,#1
            add x23,x25,x24
            ubfx x8,x22,#8,#24
            add x0,sp,#12
            mov w1,#2
            mov w3,#1
            mov w4,#160
            mov x2,x23
            strb w8,[sp,#12]
            strb w22,[sp,#13]
            bl iReadRegI2C

            tbnz w0,#31,.L_ffffff800871bd1c

            ldrb w8,[x26,:lo12:.L_ffffff80099f5592]
            tbnz w8,#2,.L_ffffff800871bccc
.L_ffffff800871bcbc:

            add x24,x24,#1
            cmp x24,#23
            b.lo .L_ffffff800871bc84

            b .L_ffffff800871bcfc
.L_ffffff800871bccc:

            ldrb w4,[x23]
            mov x0,x19
            mov x1,x20
            mov x2,x21
            mov w3,w22
            bl __dynamic_pr_debug

            b .L_ffffff800871bcbc
.L_ffffff800871bce8:

            adrp x0, .L_ffffff8009185775
            add x0,x0, :lo12:.L_ffffff8009185775
            bl printk

            mov w0,wzr
            b .L_ffffff800871bd60
.L_ffffff800871bcfc:

            adrp x8, .L_ffffff8009c7c668
            mov w9,#1
            adrp x10, .L_ffffff8009c7c66c
            mov w11,#23
            adrp x12, .L_ffffff8009c7c670
            strb w9,[x8,:lo12:.L_ffffff8009c7c668]
            str w11,[x10,:lo12:.L_ffffff8009c7c66c]
            str w9,[x12,:lo12:.L_ffffff8009c7c670]
.L_ffffff800871bd1c:

            adrp x9, .L_ffffff8009c7628c
            ldrsw x9,[x9,:lo12:.L_ffffff8009c7628c]
            ldrb w8,[sp,#18]
            adrp x11, .L_ffffff800992f548
            mov w10,#156
            add x11,x11, :lo12:.L_ffffff800992f548
            madd x9,x9,x10,x11
            ldrb w12,[sp,#31]
            ldrb w13,[sp,#32]
            ldrb w10,[sp,#33]
            ldrb w11,[sp,#19]
            str w8,[x9,#64]
            ldrb w8,[sp,#27]
            mov w0,#1
            stp w12,w13,[x9,#136]
            str w11,[x9,#100]
            stp w10,w8,[x9,#144]
.L_ffffff800871bd60:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#40]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff800871bdb8

            ldp x29,lr,[sp,#112]
            ldp x20,x19,[sp,#96]
            ldp x22,x21,[sp,#80]
            ldp x24,x23,[sp,#64]
            ldp x26,x25,[sp,#48]
            add sp,sp,#128
            ret
.L_ffffff800871bd90:

            ldrb w4,[sp,#8]
            adrp x0, .L_ffffff80099f5570
            adrp x1, .L_ffffff800918579c
            adrp x2, .L_ffffff800918578f
            add x0,x0, :lo12:.L_ffffff80099f5570
            add x1,x1, :lo12:.L_ffffff800918579c
            add x2,x2, :lo12:.L_ffffff800918578f
            mov w3,wzr
            bl __dynamic_pr_debug

            b .L_ffffff800871bc3c
.L_ffffff800871bdb8:

            bl __stack_chk_fail
.size ov16880CheckVersion, . - ov16880CheckVersion
.align 2

.pushsection ".ov16880.data.write", "aw"
#include "ov16880mipiraw_data.S"
.popsection
