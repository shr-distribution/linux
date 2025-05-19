#include <asm/assembler.h>

#-----------------------------------
.globl IMX219_MIPI_RAW_SensorInit
.type IMX219_MIPI_RAW_SensorInit, @function
#-----------------------------------
IMX219_MIPI_RAW_SensorInit:

            cbz x0,.L_ffffff8008704a88

            adrp x8, .L_ffffff80099078f0
            add x8,x8, :lo12:.L_ffffff80099078f0
            str x8,[x0]
.L_ffffff8008704a88:

            mov w0,wzr
            ret
.size IMX219_MIPI_RAW_SensorInit, . - IMX219_MIPI_RAW_SensorInit
#-----------------------------------
.type FUN_ffffff8008704a90, @function
#-----------------------------------
FUN_ffffff8008704a90:
#-----------------------------------
.type open, @function
#-----------------------------------
open:

            sub sp,sp,#80
            stp x22,x21,[sp,#32]
            stp x20,x19,[sp,#48]
            stp x29,lr,[sp,#64]
            add x29,sp,#64
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            adrp x19, .L_ffffff800916e0c8
            adrp x0, .L_ffffff80091842f0
            add x19,x19, :lo12:.L_ffffff800916e0c8
            add x0,x0, :lo12:.L_ffffff80091842f0
            mov x1,x19
            str x8,[sp,#24]
            bl printk

            adrp x0, .L_ffffff800918431e
            add x0,x0, :lo12:.L_ffffff800918431e
            mov x1,x19
            bl printk

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            mov x0,x19
            bl _raw_spin_lock

            adrp x21, .L_ffffff8009907988
            mov w8,#33
            mov x0,x19
            strb w8,[x21,:lo12:.L_ffffff8009907988]
            bl _raw_spin_unlock

            ldrb w4,[x21,:lo12:.L_ffffff8009907988]
            add x0,sp,#20
            add x2,sp,#12
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#12]
            strh wzr,[sp,#20]
            mov w20,#1
            bl iReadRegI2C

            ldrb w4,[x21,:lo12:.L_ffffff8009907988]
            ldrh w22,[sp,#12]
            mov w19,#256
            add x0,sp,#20
            add x2,sp,#16
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#16]
            strh w19,[sp,#20]
            bl iReadRegI2C

            ldrh w8,[sp,#16]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            orr w3,w8,w22, lsl #8
            cmp w3,#537
            b.eq .L_ffffff8008704cc4

            adrp x0, .L_ffffff80091843e0
            adrp x1, .L_ffffff800916e0c8
            add x0,x0, :lo12:.L_ffffff80091843e0
            add x1,x1, :lo12:.L_ffffff800916e0c8
            bl printk

            ldrb w4,[x21,:lo12:.L_ffffff8009907988]
            add x0,sp,#20
            add x2,sp,#12
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#12]
            strh wzr,[sp,#20]
            bl iReadRegI2C

            ldrb w4,[x21,:lo12:.L_ffffff8009907988]
            ldrh w22,[sp,#12]
            add x0,sp,#20
            add x2,sp,#16
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#16]
            strh w19,[sp,#20]
            bl iReadRegI2C

            ldrh w8,[sp,#16]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            orr w3,w8,w22, lsl #8
            cmp w3,#537
            b.eq .L_ffffff8008704cc4

            adrp x0, .L_ffffff80091843e0
            adrp x1, .L_ffffff800916e0c8
            add x0,x0, :lo12:.L_ffffff80091843e0
            add x1,x1, :lo12:.L_ffffff800916e0c8
            bl printk

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            mov x0,x19
            bl _raw_spin_lock

            mov w8,#32
            mov x0,x19
            strb w8,[x21,:lo12:.L_ffffff8009907988]
            bl _raw_spin_unlock

            ldrb w4,[x21,:lo12:.L_ffffff8009907988]
            add x0,sp,#20
            add x2,sp,#12
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#12]
            strh wzr,[sp,#20]
            bl iReadRegI2C

            ldrb w4,[x21,:lo12:.L_ffffff8009907988]
            ldrh w22,[sp,#12]
            mov w19,#256
            add x0,sp,#20
            add x2,sp,#16
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#16]
            strh w19,[sp,#20]
            bl iReadRegI2C

            ldrh w8,[sp,#16]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            orr w3,w8,w22, lsl #8
            cmp w3,#537
            b.eq .L_ffffff8008704cc4

            adrp x0, .L_ffffff80091843e0
            adrp x1, .L_ffffff800916e0c8
            add x0,x0, :lo12:.L_ffffff80091843e0
            add x1,x1, :lo12:.L_ffffff800916e0c8
            bl printk

            ldrb w4,[x21,:lo12:.L_ffffff8009907988]
            add x0,sp,#20
            add x2,sp,#12
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#12]
            strh wzr,[sp,#20]
            bl iReadRegI2C

            ldrb w4,[x21,:lo12:.L_ffffff8009907988]
            ldrh w22,[sp,#12]
            add x0,sp,#20
            add x2,sp,#16
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#16]
            strh w19,[sp,#20]
            bl iReadRegI2C

            ldrh w8,[sp,#16]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            orr w3,w8,w22, lsl #8
            cmp w3,#537
            b.ne .L_ffffff8008704d7c
.L_ffffff8008704cc4:

            adrp x0, .L_ffffff80091843a0
            adrp x1, .L_ffffff800916e0c8
            add x0,x0, :lo12:.L_ffffff80091843a0
            add x1,x1, :lo12:.L_ffffff800916e0c8
            mov w3,#537
            bl printk

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff80091832e5
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff80091832e5
            bl printk

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            adrp x8, .L_ffffff8009c7b624
            mov x0,x19
            strh wzr,[x8,:lo12:.L_ffffff8009c7b624]
            bl _raw_spin_lock

            adrp x9, .L_ffffff8009c7b630
            strb w20,[x9,:lo12:.L_ffffff8009c7b630]
            adrp x9, .L_ffffff8009c7b638
            strb w20,[x9,:lo12:.L_ffffff8009c7b638]
            adrp x9, .L_ffffff8009c7b634
            mov w10,#1332
            adrp x8, .L_ffffff8009c7b642
            str w10,[x9,:lo12:.L_ffffff8009c7b634]
            adrp x9, .L_ffffff8009c7b63c
            str w10,[x9,:lo12:.L_ffffff8009c7b63c]
            adrp x9, .L_ffffff8009c7b640
            adrp x10, .L_ffffff8009907984
            strb wzr,[x8,:lo12:.L_ffffff8009c7b642]
            mov w8,#300
            mov x0,x19
            strh wzr,[x9,:lo12:.L_ffffff8009c7b640]
            strh w8,[x10,:lo12:.L_ffffff8009907984]
            bl _raw_spin_unlock

            mov w0,wzr
.L_ffffff8008704d54:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#24]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008704d98

            ldp x29,lr,[sp,#64]
            ldp x20,x19,[sp,#48]
            ldp x22,x21,[sp,#32]
            add sp,sp,#80
            ret
.L_ffffff8008704d7c:

            adrp x0, .L_ffffff80091843e0
            adrp x1, .L_ffffff800916e0c8
            add x0,x0, :lo12:.L_ffffff80091843e0
            add x1,x1, :lo12:.L_ffffff800916e0c8
            bl printk

            mov w0,#16
            b .L_ffffff8008704d54
.L_ffffff8008704d98:

            bl __stack_chk_fail
.size open, . - open
.size FUN_ffffff8008704a90, . - FUN_ffffff8008704a90
#-----------------------------------
.type FUN_ffffff8008704d9c, @function
#-----------------------------------
FUN_ffffff8008704d9c:
#-----------------------------------
.type get_info, @function
#-----------------------------------
get_info:

            stp x20,x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            mov x19,x1
            mov w20,w0
            adrp x0, .L_ffffff8009184446
            adrp x1, .L_ffffff80091d8c1c
            add x0,x0, :lo12:.L_ffffff8009184446
            add x1,x1, :lo12:.L_ffffff80091d8c1c
            mov w2,w20
            bl printk

            mov x12,#8589934594
            mov w9,#4
            stur x12,[x19,#44]
            stur x12,[x19,#52]
            stp x12,x12,[x19,#72]
            mov w12,#256
            strb w9,[x19,#22]
            mov w9,#3
            movk w12,#514,lsl #16
            mov w8,#257
            mov x10,#21474836480
            mov w11,#1
            stur w12,[x19,#139]
            mov w12,#24
            strh w9,[x19,#136]
            strb w9,[x19,#161]
            strb w9,[x19,#17]
            sub w9,w20,#1
            sturh w8,[x19,#13]
            strh w8,[x19,#20]
            mov w8,#2
            stp x10,x11,[x19,#24]
            str w11,[x19,#168]
            str w11,[x19,#40]
            mov w10,#512
            strb w12,[x19,#8]
            mov w11,#515
            mov w12,#16777216
            cmp w9,#3
            str wzr,[x19,#156]
            strb wzr,[x19,#138]
            strb wzr,[x19,#162]
            strh wzr,[x19,#164]
            strb wzr,[x19,#143]
            str w8,[x19,#60]
            str w8,[x19,#88]
            sturh w10,[x19,#15]
            strh w11,[x19,#18]
            stur w12,[x19,#150]
            b.hi .L_ffffff8008704e90

            mov w8,w9
            mov x10,#1125917086973956
            mov w11,#8277
            movk x10,#2,lsl #16
            lsl w9,w9,#3
            movk w11,#8224,lsl #16
            lsl x8,x8,#4
            lsr x8,x10,x8
            lsr w9,w11,w9
            b .L_ffffff8008704e94
.L_ffffff8008704e90:

            mov w9,#32
.L_ffffff8008704e94:

            strh w8,[x19,#92]
            strh w8,[x19,#94]
            strb w9,[x19,#144]
            ldp x29,lr,[sp,#16]
            mov w0,wzr
            ldp x20,x19,[sp],#32
            ret
.size get_info, . - get_info
.size FUN_ffffff8008704d9c, . - FUN_ffffff8008704d9c
#-----------------------------------
.type FUN_ffffff8008704eb0, @function
#-----------------------------------
FUN_ffffff8008704eb0:
#-----------------------------------
.type get_resolution, @function
#-----------------------------------
get_resolution:

            str x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            mov x19,x0
            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff800918265f
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff800918265f
            bl printk

            mov x8,#1632
            mov x9,#1280
            mov x10,#1632
            movk x8,#1224,lsl #16
            movk x9,#720,lsl #16
            movk x10,#1224,lsl #16
            movk x8,#3264,lsl #32
            movk x9,#1632,lsl #32
            movk x10,#1632,lsl #32
            movk x8,#2448,lsl #48
            movk x9,#1224,lsl #48
            movk x10,#1224,lsl #48
            stp x8,x8,[x19]
            stp x9,x10,[x19,#16]
            str x10,[x19,#32]
            ldp x29,lr,[sp,#16]
            mov w0,wzr
            ldr x19,[sp],#32
            ret
.size get_resolution, . - get_resolution
.size FUN_ffffff8008704eb0, . - FUN_ffffff8008704eb0
#-----------------------------------
.type FUN_ffffff8008704f20, @function
#-----------------------------------
FUN_ffffff8008704f20:
#-----------------------------------
.type feature_control, @function
#-----------------------------------
feature_control:

            sub sp,sp,#96
            stp x26,x25,[sp,#16]
            stp x24,x23,[sp,#32]
            stp x22,x21,[sp,#48]
            stp x20,x19,[sp,#64]
            stp x29,lr,[sp,#80]
            add x29,sp,#80
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov x19,x1
            mov w21,w0
            adrp x0, .L_ffffff8009184473
            adrp x1, .L_ffffff800918267f
            mov x20,x2
            add x0,x0, :lo12:.L_ffffff8009184473
            add x1,x1, :lo12:.L_ffffff800918267f
            mov w2,w21
            str x8,[sp,#8]
            bl printk

            sub w8,w21,#3002
            cmp w8,#102
            b.hi .L_ffffff8008706198

            adrp x9, .L_ffffff8008f295b8
            add x9,x9, :lo12:.L_ffffff8008f295b8
            adr x10, .L_ffffff8008704f90
            ldrh w11,[x9,x8,lsl #1]
            add x10,x10,x11, lsl #2
            br x10
.L_ffffff8008704f90:

            adrp x8, .L_ffffff8009c7b638
            ldrb w8,[x8,:lo12:.L_ffffff8009c7b638]
            mov w9,#3448
.L_ffffff8008704f9c:

            adrp x10, .L_ffffff8009c7b634
            cmp w8,#0
            csel w8,w9,wzr,ne
            strh w8,[x19]
            ldr w8,[x10,:lo12:.L_ffffff8009c7b634]
            mov w9,#4
            strh w8,[x19,#2]
            str w9,[x20]
            b .L_ffffff8008706198

            adrp x21, .L_ffffff8009c7b630
            ldrb w8,[x21,:lo12:.L_ffffff8009c7b630]
            adrp x9, .L_ffffff8009907984
            ldrh w3,[x9,:lo12:.L_ffffff8009907984]
            mov w22,#47104
            cmp w8,#0
            movk w22,#2636,lsl #16
            adrp x0, .L_ffffff800918449f
            adrp x1, .L_ffffff800918267f
            csel w2,w22,wzr,ne
            add x0,x0, :lo12:.L_ffffff800918449f
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            ldrb w8,[x21,:lo12:.L_ffffff8009c7b630]
            mov w9,#4
            cmp w8,#0
            csel w8,w22,wzr,ne
            b .L_ffffff80087054f0

            ldr w22,[x19]
            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            mov x0,x19
            bl _raw_spin_lock_irqsave

            mov x1,x0
            and w23,w22,#65535
            adrp x8, .L_ffffff8009907980
            mov x0,x19
            str w23,[x8,:lo12:.L_ffffff8009907980]
            bl _raw_spin_unlock_irqrestore

            mov x0,x19
            bl _raw_spin_lock

            adrp x21, .L_ffffff8009c7b63c
            ldr w8,[x21,:lo12:.L_ffffff8009c7b63c]
            add w9,w23,#5
            mov w10,#65535
            adrp x20, .L_ffffff8009c7b634
            sub w11,w8,#5
            cmp w11,w23
            csel w8,w9,w8,lo
            cmp w8,w10
            csel w8,w8,w10,lo
            mov x0,x19
            str w8,[x20,:lo12:.L_ffffff8009c7b634]
            bl _raw_spin_unlock

            adrp x10, .L_ffffff8009c7b642
            cmp w23,#2
            mov w8,#2
            ldrb w10,[x10,:lo12:.L_ffffff8009c7b642]
            mov w9,#65530
            csel w8,w22,w8,hi
            cmp w9,w8, uxth
            mov w9,#-6
            csel w8,w8,w9,hi
            cmp w10,#1
            and w19,w8,#65535
            b.ne .L_ffffff80087056b4

            adrp x22, .L_ffffff8009c7b630
            ldrb w9,[x22,:lo12:.L_ffffff8009c7b630]
            ldr w8,[x20,:lo12:.L_ffffff8009c7b634]
            cmp w9,#0
            mov w9,#42408
            movk w9,#7,lsl #16
            csel w9,w9,wzr,ne
            udiv w9,w9,w8
            and w9,w9,#65535
            sub w10,w9,#297
            cmp w10,#8
            b.hi .L_ffffff8008705808

            adrp x0, .L_ffffff80091846ae
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff80091846ae
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w2,#296
            mov w3,wzr
            bl printk

            ldrb w8,[x22,:lo12:.L_ffffff8009c7b630]
            cmp w8,#0
            mov w8,#1693
            b .L_ffffff800870583c

            adrp x8, .L_ffffff8009c7b624
            ldr x22,[x19]
            ldrh w8,[x8,:lo12:.L_ffffff8009c7b624]
            cmp w8,w22, uxth
            b.eq .L_ffffff8008706198

            adrp x0, .L_ffffff8009184799
            adrp x1, .L_ffffff80091847e3
            add x0,x0, :lo12:.L_ffffff8009184799
            add x1,x1, :lo12:.L_ffffff80091847e3
            and w19,w22,#65535
            bl printk

            adrp x21, .L_ffffff8009907768
            add x21,x21, :lo12:.L_ffffff8009907768
            mov x20,xzr
            mov x8,x21
.L_ffffff8008705138:

            ldrh w3,[x8]
            cmp w3,w22, uxth
            b.hi .L_ffffff800870515c

            b.eq .L_ffffff8008705194

            add x20,x20,#1
            cmp x20,#96
            add x8,x8,#4
            b.lo .L_ffffff8008705138

            ldrh w3,[x8]
.L_ffffff800870515c:

            cmp w3,w22, uxth
            b.eq .L_ffffff800870517c

            adrp x0, .L_ffffff80091847ec
            adrp x1, .L_ffffff80091847e3
            add x0,x0, :lo12:.L_ffffff80091847ec
            add x1,x1, :lo12:.L_ffffff80091847e3
            mov w2,w19
            bl printk
.L_ffffff800870517c:

            adrp x0, .L_ffffff800918482c
            adrp x1, .L_ffffff80091847e3
            add x0,x0, :lo12:.L_ffffff800918482c
            add x1,x1, :lo12:.L_ffffff80091847e3
            bl printk

            sub x20,x20,#1
.L_ffffff8008705194:

            add x8,x21,x20, lsl #2
            ldrh w20,[x8,#2]
            adrp x21, .L_ffffff8009c7b62c
            add x21,x21, :lo12:.L_ffffff8009c7b62c
            mov x0,x21
            bl _raw_spin_lock

            mov x0,x21
            bl _raw_spin_unlock

            adrp x8, .L_ffffff8009907988
            ldrb w2,[x8,:lo12:.L_ffffff8009907988]
            mov w8,#22273
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            adrp x0, .L_ffffff8009184757
            adrp x1, .L_ffffff8009184790
            add x0,x0, :lo12:.L_ffffff8009184757
            add x1,x1, :lo12:.L_ffffff8009184790
            mov w2,w19
            mov w3,w20
            b .L_ffffff8008705b4c

            ldp w8,w9,[x19]
            adrp x10, .L_ffffff8009907988
            ldrb w2,[x10,:lo12:.L_ffffff8009907988]
            lsr w10,w8,#8
            strb w8,[sp,#5]
            strb w10,[sp,#4]
            strb w9,[sp,#6]
            b .L_ffffff800870618c

            ldr w8,[x19]
            adrp x9, .L_ffffff8009907988
            ldrb w4,[x9,:lo12:.L_ffffff8009907988]
            mov x0,sp
            lsr w9,w8,#8
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strb w9,[sp]
            strb w8,[sp,#1]
            bl iReadRegI2C

            ldrh w8,[sp,#4]
            str w8,[x19,#4]
            b .L_ffffff8008706198

            mov w8,#-1
            b .L_ffffff80087054ec

            ldr w20,[x19]
            adrp x0, .L_ffffff8009184869
            adrp x1, .L_ffffff8009184895
            add x0,x0, :lo12:.L_ffffff8009184869
            and w19,w20,#65535
            add x1,x1, :lo12:.L_ffffff8009184895
            mov w2,w19
            bl printk

            cbz w19,.L_ffffff8008706198

            adrp x0, .L_ffffff8009c7b62c
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            bl _raw_spin_lock

            cmp w19,#150
            b.eq .L_ffffff8008705b54

            cmp w19,#300
            b.ne .L_ffffff8008705b64

            adrp x8, .L_ffffff8009c7b642
            ldrb w8,[x8,:lo12:.L_ffffff8009c7b642]
            tbz w8,#0,.L_ffffff8008705b64

            mov w20,#296
            b .L_ffffff8008705b64

            adrp x23, .L_ffffff8008f29800
            adrp x20, .L_ffffff8009c7b62c
            adrp x21, .L_ffffff80091848b5
            adrp x22, .L_ffffff80091848a4
            mov w26,#33
            add x23,x23, :lo12:.L_ffffff8008f29800
            add x20,x20, :lo12:.L_ffffff8009c7b62c
            adrp x24, .L_ffffff8009907988
            mov w25,#256
            add x21,x21, :lo12:.L_ffffff80091848b5
            add x22,x22, :lo12:.L_ffffff80091848a4
.L_ffffff80087052d4:

            mov x0,x20
            bl _raw_spin_lock

            mov x0,x20
            strb w26,[x24,:lo12:.L_ffffff8009907988]
            bl _raw_spin_unlock

            ldrb w4,[x24,:lo12:.L_ffffff8009907988]
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh wzr,[sp]
            bl iReadRegI2C

            ldrb w4,[x24,:lo12:.L_ffffff8009907988]
            ldrh w26,[sp,#4]
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w25,[sp]
            bl iReadRegI2C

            ldrh w8,[sp,#4]
            orr w3,w8,w26, lsl #8
            str w3,[x19]
            ldrb w2,[x24,:lo12:.L_ffffff8009907988]
            cmp w3,#537
            b.eq .L_ffffff800870569c

            mov x0,x21
            mov x1,x22
            bl printk

            ldrb w4,[x24,:lo12:.L_ffffff8009907988]
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh wzr,[sp]
            bl iReadRegI2C

            ldrb w4,[x24,:lo12:.L_ffffff8009907988]
            ldrh w26,[sp,#4]
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w25,[sp]
            bl iReadRegI2C

            ldrh w8,[sp,#4]
            orr w3,w8,w26, lsl #8
            str w3,[x19]
            ldrb w2,[x24,:lo12:.L_ffffff8009907988]
            cmp w3,#537
            b.eq .L_ffffff800870569c

            mov x0,x21
            mov x1,x22
            bl printk

            ldrb w26,[x23],#1
            cmp w26,#255
            b.ne .L_ffffff80087052d4

            ldr w8,[x19]
            cmp w8,#537
            b.eq .L_ffffff8008706198

            mov w8,#-1
            str w8,[x19]
            b .L_ffffff8008706198

            ldrb w8,[x19]
            ldrh w3,[x19,#2]
            adrp x0, .L_ffffff8009184902
            adrp x1, .L_ffffff800918493b
            cmp w8,#0
            cset w19,ne
            add x0,x0, :lo12:.L_ffffff8009184902
            add x1,x1, :lo12:.L_ffffff800918493b
            mov w2,w19
            bl printk

            adrp x20, .L_ffffff8009c7b62c
            add x20,x20, :lo12:.L_ffffff8009c7b62c
            mov x0,x20
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b642
            strb w19,[x8,:lo12:.L_ffffff8009c7b642]
            b .L_ffffff8008705530

            ldrb w8,[x19]
            cmp w8,#0
            cset w0,ne
            bl set_test_pattern_mode

            b .L_ffffff8008706198

            ldr w20,[x19]
            ldr w19,[x19,#8]
            adrp x0, .L_ffffff8009184951
            adrp x1, .L_ffffff800918498e
            add x0,x0, :lo12:.L_ffffff8009184951
            add x1,x1, :lo12:.L_ffffff800918498e
            mov w2,w20
            mov w3,w19
            bl printk

            cmp w20,#9
            b.hi .L_ffffff8008705cfc

            adrp x8, .L_ffffff8008f296a2
            add x8,x8, :lo12:.L_ffffff8008f296a2
            adr x9, .L_ffffff8008705478
            ldrh w10,[x8,x20,lsl #1]
            add x9,x9,x10, lsl #2
            br x9
.L_ffffff8008705478:

            mov w8,#47104
            movk w8,#2636,lsl #16
            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b62c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            lsl w19,w8,#1
            b .L_ffffff8008706064

            ldr w20,[x19]
            ldr x19,[x19,#8]
            adrp x0, .L_ffffff8009184446
            adrp x1, .L_ffffff80091849f9
            add x0,x0, :lo12:.L_ffffff8009184446
            add x1,x1, :lo12:.L_ffffff80091849f9
            mov w2,w20
            bl printk

            cmp w20,#9
            b.hi .L_ffffff8008706198

            adrp x8, .L_ffffff8008f2968e
            add x8,x8, :lo12:.L_ffffff8008f2968e
            adr x9, .L_ffffff80087054d8
            ldrh w10,[x8,x20,lsl #1]
            add x9,x9,x10, lsl #2
            br x9
.L_ffffff80087054d8:

            mov w8,#300
            str w8,[x19]
            b .L_ffffff8008706198

            mov w8,#34332
            movk w8,#40456,lsl #16
.L_ffffff80087054ec:

            mov w9,#4
.L_ffffff80087054f0:

            str w8,[x19]
            str w9,[x20]
            b .L_ffffff8008706198

            ldr w2,[x19]
            adrp x0, .L_ffffff80091844fa
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff80091844fa
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            adrp x20, .L_ffffff8009c7b62c
            add x20,x20, :lo12:.L_ffffff8009c7b62c
            mov x0,x20
            bl _raw_spin_lock

            ldr w8,[x19]
            adrp x9, .L_ffffff8009907984
            strh w8,[x9,:lo12:.L_ffffff8009907984]
.L_ffffff8008705530:

            mov x0,x20
            bl _raw_spin_unlock

            b .L_ffffff8008706198

            ldrb w2,[x19]
            adrp x0, .L_ffffff8009184526
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184526
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            mov x0,x19
            bl _raw_spin_lock

            mov x0,x19
            bl _raw_spin_unlock

            b .L_ffffff8008706198

            ldr w2,[x19]
            adrp x0, .L_ffffff8009184552
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184552
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            ldr w9,[x19]
            ldr x8,[x19,#8]
            sub w9,w9,#1
            cmp w9,#3
            b.hi .L_ffffff8008705c28

            adrp x10, .L_ffffff8008f29686
            add x10,x10, :lo12:.L_ffffff8008f29686
            adr x11, .L_ffffff80087055b4
            ldrh w12,[x10,x9,lsl #1]
            add x11,x11,x12, lsl #2
            br x11
.L_ffffff80087055b4:

            adrp x9, .L_ffffff8008f29828
            add x9,x9, :lo12:.L_ffffff8008f29828
            b .L_ffffff8008705c54

            ldr x22,[x19,#8]
            ldr w20,[x19]
            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            mov x0,x19
            and w23,w22,#65535
            bl _raw_spin_lock_irqsave

            mov x1,x0
            and w21,w20,#65535
            adrp x8, .L_ffffff8009907980
            mov x0,x19
            str w21,[x8,:lo12:.L_ffffff8009907980]
            bl _raw_spin_unlock_irqrestore

            mov x0,x19
            bl _raw_spin_lock

            cmp w23,#1
            b.ls .L_ffffff80087056f0

            and w8,w22,#65535
            adrp x9, .L_ffffff8009c7b634
            str w8,[x9,:lo12:.L_ffffff8009c7b634]
            b .L_ffffff80087056f8

            adrp x0, .L_ffffff80091845e2
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff80091845e2
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            adrp x0, .L_ffffff8009184a58
            adrp x1, .L_ffffff8009184aa3
            add x0,x0, :lo12:.L_ffffff8009184a58
            add x1,x1, :lo12:.L_ffffff8009184aa3
            mov w2,wzr
            bl printk

            adrp x8, .L_ffffff8009907988
            ldrb w2,[x8,:lo12:.L_ffffff8009907988]
            mov w8,#1
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            b .L_ffffff800870618c

            adrp x0, .L_ffffff8009184623
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184623
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            adrp x0, .L_ffffff8009184a58
            adrp x1, .L_ffffff8009184aa3
            add x0,x0, :lo12:.L_ffffff8009184a58
            add x1,x1, :lo12:.L_ffffff8009184aa3
            mov w2,#1
            mov w19,#1
            bl printk

            adrp x8, .L_ffffff8009907988
            ldrb w2,[x8,:lo12:.L_ffffff8009907988]
            strh w19,[sp,#4]
            strb w19,[sp,#6]
            b .L_ffffff800870618c
.L_ffffff800870569c:

            adrp x0, .L_ffffff80091843a0
            adrp x1, .L_ffffff80091848a4
            add x0,x0, :lo12:.L_ffffff80091843a0
            add x1,x1, :lo12:.L_ffffff80091848a4
            mov w3,#537
            b .L_ffffff8008705b4c
.L_ffffff80087056b4:

            ldr w8,[x20,:lo12:.L_ffffff8009c7b634]
            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24577
            lsr w8,w8,#8
.L_ffffff80087056c8:

            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldr w8,[x20,:lo12:.L_ffffff8009c7b634]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24833
            strh w9,[sp,#4]
            b .L_ffffff800870593c
.L_ffffff80087056f0:

            adrp x8, .L_ffffff8009c7b634
            ldr w8,[x8,:lo12:.L_ffffff8009c7b634]
.L_ffffff80087056f8:

            sub w9,w8,#5
            add w10,w21,#5
            cmp w9,w21
            csel w8,w10,w8,lo
            b.lo .L_ffffff8008705714

            cmp w8,#16,lsl #12
            b.lo .L_ffffff800870572c
.L_ffffff8008705714:

            lsr w9,w8,#16
            mov w10,#65535
            cmp w9,#0
            csel w8,w10,w8,ne
            adrp x9, .L_ffffff8009c7b634
            str w8,[x9,:lo12:.L_ffffff8009c7b634]
.L_ffffff800870572c:

            adrp x0, .L_ffffff8009c7b62c
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            bl _raw_spin_unlock

            and w8,w20,#65535
            adrp x11, .L_ffffff8009c7b642
            cmp w8,#2
            ldrb w8,[x11,:lo12:.L_ffffff8009c7b642]
            mov w9,#2
            mov w10,#65530
            csel w9,w20,w9,hi
.L_ffffff8008705754:

            cmp w10,w9, uxth
            mov w10,#-6
            csel w20,w9,w10,hi
            cmp w8,#1
            b.ne .L_ffffff80087057c8

            adrp x21, .L_ffffff8009c7b630
            ldrb w9,[x21,:lo12:.L_ffffff8009c7b630]
            adrp x19, .L_ffffff8009c7b634
            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            cmp w9,#0
            mov w9,#42408
            movk w9,#7,lsl #16
            csel w9,w9,wzr,ne
            udiv w9,w9,w8
            and w9,w9,#65535
            sub w10,w9,#297
            cmp w10,#8
            b.hi .L_ffffff80087059a4

            adrp x0, .L_ffffff80091846ae
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff80091846ae
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w2,#296
            mov w3,wzr
            bl printk

            ldrb w8,[x21,:lo12:.L_ffffff8009c7b630]
            cmp w8,#0
            mov w8,#1693
            b .L_ffffff80087059d8
.L_ffffff80087057c8:

            adrp x19, .L_ffffff8009c7b634
            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24577
            lsr w8,w8,#8
.L_ffffff80087057e0:

            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24833
            strh w9,[sp,#4]
            b .L_ffffff8008705adc
.L_ffffff8008705808:

            sub w9,w9,#147
            cmp w9,#3
            b.hi .L_ffffff8008705cd4

            adrp x0, .L_ffffff80091846ae
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff80091846ae
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w2,#146
            mov w3,wzr
            bl printk

            ldrb w8,[x22,:lo12:.L_ffffff8009c7b630]
            cmp w8,#0
            mov w8,#3432
.L_ffffff800870583c:

            adrp x0, .L_ffffff8009c7b62c
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            csel w22,w8,wzr,ne
            bl _raw_spin_lock

            ldr w8,[x21,:lo12:.L_ffffff8009c7b63c]
            adrp x21, .L_ffffff8009c7b640
            cmp w22,w8
            csel w9,w22,w8,hi
            sub w10,w9,w8
            cmp w9,#16,lsl #12
            str w9,[x20,:lo12:.L_ffffff8009c7b634]
            strh w10,[x21,:lo12:.L_ffffff8009c7b640]
            b.lo .L_ffffff8008705880

            mov w9,#65535
            mvn w8,w8
            str w9,[x20,:lo12:.L_ffffff8009c7b634]
            strh w8,[x21,:lo12:.L_ffffff8009c7b640]
.L_ffffff8008705880:

            adrp x0, .L_ffffff8009c7b62c
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            bl _raw_spin_unlock

            ldrh w2,[x21,:lo12:.L_ffffff8009c7b640]
            adrp x0, .L_ffffff800918470f
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff800918470f
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x20,:lo12:.L_ffffff8009c7b634]
            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24577
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldr w8,[x20,:lo12:.L_ffffff8009c7b634]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24833
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            adrp x22, .L_ffffff8009c7b638
            ldrb w8,[x22,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#25089
            add x0,sp,#4
            cmp w8,#0
            mov w8,#13
            csel w8,w8,wzr,ne
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w8,[x22,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#25345
            strh w9,[sp,#4]
            cmp w8,#0
            mov w8,#120
            csel w8,w8,wzr,ne
.L_ffffff800870593c:

            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w8,#23041
            strh w8,[sp,#4]
            lsr w8,w19,#8
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w8,#23297
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldr w3,[x20,:lo12:.L_ffffff8009c7b634]
            adrp x0, .L_ffffff8009184663
            adrp x1, .L_ffffff80091846a2
            add x0,x0, :lo12:.L_ffffff8009184663
            add x1,x1, :lo12:.L_ffffff80091846a2
            b .L_ffffff8008705b48
.L_ffffff80087059a4:

            sub w9,w9,#147
            cmp w9,#3
            b.hi .L_ffffff8008705ce8

            adrp x0, .L_ffffff80091846ae
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff80091846ae
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w2,#146
            mov w3,wzr
            bl printk

            ldrb w8,[x21,:lo12:.L_ffffff8009c7b630]
            cmp w8,#0
            mov w8,#3432
.L_ffffff80087059d8:

            adrp x0, .L_ffffff8009c7b62c
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            csel w22,w8,wzr,ne
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b63c
            ldr w8,[x8,:lo12:.L_ffffff8009c7b63c]
            adrp x21, .L_ffffff8009c7b640
            cmp w22,w8
            csel w9,w22,w8,hi
            sub w10,w9,w8
            cmp w9,#16,lsl #12
            str w9,[x19,:lo12:.L_ffffff8009c7b634]
            strh w10,[x21,:lo12:.L_ffffff8009c7b640]
            b.lo .L_ffffff8008705a20

            mov w9,#65535
            mvn w8,w8
            str w9,[x19,:lo12:.L_ffffff8009c7b634]
            strh w8,[x21,:lo12:.L_ffffff8009c7b640]
.L_ffffff8008705a20:

            adrp x0, .L_ffffff8009c7b62c
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            bl _raw_spin_unlock

            ldrh w2,[x21,:lo12:.L_ffffff8009c7b640]
            adrp x0, .L_ffffff800918470f
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff800918470f
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24577
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24833
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            adrp x19, .L_ffffff8009c7b638
            ldrb w8,[x19,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#25089
            add x0,sp,#4
            cmp w8,#0
            mov w8,#13
            csel w8,w8,wzr,ne
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w8,[x19,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#25345
            strh w9,[sp,#4]
            cmp w8,#0
            mov w8,#120
            csel w8,w8,wzr,ne
.L_ffffff8008705adc:

            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w8,#23041
            strh w8,[sp,#4]
            lsr w8,w20,#8
            add x0,sp,#4
            mov w1,#3
            and w19,w20,#65535
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w8,#23297
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            adrp x8, .L_ffffff8009c7b634
            ldr w3,[x8,:lo12:.L_ffffff8009c7b634]
            adrp x0, .L_ffffff8009184663
            adrp x1, .L_ffffff8009184ab5
            add x0,x0, :lo12:.L_ffffff8009184663
            add x1,x1, :lo12:.L_ffffff8009184ab5
.L_ffffff8008705b48:

            mov w2,w19
.L_ffffff8008705b4c:

            bl printk

            b .L_ffffff8008706198
.L_ffffff8008705b54:

            adrp x8, .L_ffffff8009c7b642
            ldrb w8,[x8,:lo12:.L_ffffff8009c7b642]
            tbz w8,#0,.L_ffffff8008705b64

            mov w20,#146
.L_ffffff8008705b64:

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            adrp x21, .L_ffffff8009907984
            mov x0,x19
            strh w20,[x21,:lo12:.L_ffffff8009907984]
            bl _raw_spin_unlock

            ldrh w20,[x21,:lo12:.L_ffffff8009907984]
            adrp x0, .L_ffffff80091846ae
            adrp x1, .L_ffffff80091846fd
            add x0,x0, :lo12:.L_ffffff80091846ae
            add x1,x1, :lo12:.L_ffffff80091846fd
            mov w3,#1
            mov w2,w20
            bl printk

            adrp x8, .L_ffffff8009c7b630
            ldrb w8,[x8,:lo12:.L_ffffff8009c7b630]
            mov w9,#47104
            movk w9,#2636,lsl #16
            mov x0,x19
            cmp w8,#0
            csel w8,w9,wzr,ne
            udiv w8,w8,w20
            add w8,w8,w8, lsl #2
            mov w9,#51243
            ubfx w8,w8,#2,#29
            movk w9,#4865,lsl #16
            umull x8,w8,w9
            lsr x21,x8,#37
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b63c
            ldr w9,[x8,:lo12:.L_ffffff8009c7b63c]
            adrp x19, .L_ffffff8009c7b634
            adrp x20, .L_ffffff8009c7b640
            cmp w21,w9
            csel w10,w21,w9,hi
            sub w11,w10,w9
            cmp w10,#16,lsl #12
            str w10,[x19,:lo12:.L_ffffff8009c7b634]
            strh w11,[x20,:lo12:.L_ffffff8009c7b640]
            b.lo .L_ffffff8008705c14

            mov w10,#65535
            mvn w9,w9
            str w10,[x19,:lo12:.L_ffffff8009c7b634]
            strh w9,[x20,:lo12:.L_ffffff8009c7b640]
.L_ffffff8008705c14:

            adrp x0, .L_ffffff8009c7b62c
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            str w10,[x8,:lo12:.L_ffffff8009c7b63c]
            bl _raw_spin_unlock

            b .L_ffffff80087060d8
.L_ffffff8008705c28:

            adrp x9, .L_ffffff8008f29808
            add x9,x9, :lo12:.L_ffffff8008f29808
            b .L_ffffff8008705c54

            adrp x9, .L_ffffff8008f29848
            add x9,x9, :lo12:.L_ffffff8008f29848
            b .L_ffffff8008705c54

            adrp x9, .L_ffffff8008f29868
            add x9,x9, :lo12:.L_ffffff8008f29868
            b .L_ffffff8008705c54

            adrp x9, .L_ffffff8008f29888
            add x9,x9, :lo12:.L_ffffff8008f29888
.L_ffffff8008705c54:

            ldp x11,x10,[x9,#16]
            ldp x9,x12,[x9]
            stp x11,x10,[x8,#16]
            stp x9,x12,[x8]
            ldrh w2,[x19]
            ldrh w3,[x19,#8]
            ldrh w4,[x19,#16]
            adrp x0, .L_ffffff8009184599
            adrp x1, .L_ffffff800918267f
            add x0,x0, :lo12:.L_ffffff8009184599
            add x1,x1, :lo12:.L_ffffff800918267f
            bl printk

            b .L_ffffff8008706198

            mov w8,#47104
            movk w8,#2636,lsl #16
            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b62c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            lsl w19,w8,#1
            bl _raw_spin_lock

            mov w8,#58871
            movk w8,#133,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008705e08

            mov w9,#51243
            lsr w8,w19,#3
            movk w9,#4865,lsl #16
            umull x8,w8,w9
            lsr x8,x8,#37
            sub w8,w8,#2544
            b .L_ffffff8008705e0c
.L_ffffff8008705cd4:

            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            lsr w8,w8,#8
            mov w9,#24577
            b .L_ffffff80087056c8
.L_ffffff8008705ce8:

            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            lsr w8,w8,#8
            mov w9,#24577
            b .L_ffffff80087057e0
.L_ffffff8008705cfc:

            mov w8,#47104
            movk w8,#2636,lsl #16
            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b62c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            lsl w19,w8,#1
            bl _raw_spin_lock

            mov w8,#8663
            movk w8,#70,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008705e38

            mov w9,#51243
            lsr w8,w19,#3
            movk w9,#4865,lsl #16
            umull x8,w8,w9
            lsr x8,x8,#37
            sub w8,w8,#1332
            b .L_ffffff8008705e3c

            cbz w19,.L_ffffff8008706198

            mov w8,#47104
            movk w8,#2636,lsl #16
            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b62c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            lsl w19,w8,#1
            bl _raw_spin_lock

            mov w8,#8663
            movk w8,#70,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008705e28

            mov w9,#51243
            lsr w8,w19,#3
            movk w9,#4865,lsl #16
            umull x8,w8,w9
            lsr x8,x8,#37
            sub w8,w8,#1332
            b .L_ffffff8008705e2c

            mov w8,#47104
            movk w8,#2636,lsl #16
            udiv w8,w8,w19
            adrp x0, .L_ffffff8009c7b62c
            add w8,w8,w8, lsl #2
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            lsl w19,w8,#1
            bl _raw_spin_lock

            mov w8,#58871
            movk w8,#133,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008705f58

            mov w9,#51243
            lsr w8,w19,#3
            movk w9,#4865,lsl #16
            umull x8,w8,w9
            lsr x8,x8,#37
            sub w8,w8,#2544
            b .L_ffffff8008705f5c

            mov w8,#47104
            movk w8,#2636,lsl #16
            udiv w8,w8,w19
            add w8,w8,w8, lsl #2
            lsl w19,w8,#1
            b .L_ffffff800870605c

            mov w8,#200
            str w8,[x19]
            b .L_ffffff8008706198
.L_ffffff8008705e08:

            mov w8,wzr
.L_ffffff8008705e0c:

            adrp x20, .L_ffffff8009c7b640
            and w9,w8,#65535
            adrp x19, .L_ffffff8009c7b634
            adrp x10, .L_ffffff8009c7b63c
            strh w8,[x20,:lo12:.L_ffffff8009c7b640]
            add w8,w9,#2544
            b .L_ffffff80087060b0
.L_ffffff8008705e28:

            mov w8,wzr
.L_ffffff8008705e2c:

            adrp x20, .L_ffffff8009c7b640
            and w9,w8,#65535
            b .L_ffffff80087060a0
.L_ffffff8008705e38:

            mov w8,wzr
.L_ffffff8008705e3c:

            adrp x21, .L_ffffff8009c7b640
            and w9,w8,#65535
            adrp x0, .L_ffffff8009c7b62c
            adrp x19, .L_ffffff8009c7b634
            adrp x10, .L_ffffff8009c7b63c
            strh w8,[x21,:lo12:.L_ffffff8009c7b640]
            add w8,w9,#1332
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            str w8,[x19,:lo12:.L_ffffff8009c7b634]
            str w8,[x10,:lo12:.L_ffffff8009c7b63c]
            bl _raw_spin_unlock

            adrp x9, .L_ffffff8009907980
            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            ldr w9,[x9,:lo12:.L_ffffff8009907980]
            cmp w8,w9
            b.ls .L_ffffff8008705f3c

            ldrh w2,[x21,:lo12:.L_ffffff8009c7b640]
            adrp x0, .L_ffffff800918470f
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff800918470f
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24577
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24833
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            adrp x19, .L_ffffff8009c7b638
            ldrb w8,[x19,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#25089
            add x0,sp,#4
            cmp w8,#0
            mov w8,#13
            csel w8,w8,wzr,ne
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w8,[x19,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#25345
            add x0,sp,#4
            cmp w8,#0
            mov w8,#120
            csel w8,w8,wzr,ne
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C
.L_ffffff8008705f3c:

            adrp x0, .L_ffffff80091849ac
            adrp x1, .L_ffffff800918498e
            add x0,x0, :lo12:.L_ffffff80091849ac
            add x1,x1, :lo12:.L_ffffff800918498e
            mov w2,w20
            bl printk

            b .L_ffffff8008706198
.L_ffffff8008705f58:

            mov w8,wzr
.L_ffffff8008705f5c:

            adrp x21, .L_ffffff8009c7b640
            and w9,w8,#65535
            adrp x0, .L_ffffff8009c7b62c
            adrp x20, .L_ffffff8009c7b634
            adrp x10, .L_ffffff8009c7b63c
            strh w8,[x21,:lo12:.L_ffffff8009c7b640]
            add w8,w9,#2544
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            str w8,[x20,:lo12:.L_ffffff8009c7b634]
            str w8,[x10,:lo12:.L_ffffff8009c7b63c]
            bl _raw_spin_unlock

            adrp x9, .L_ffffff8009907980
            ldr w8,[x20,:lo12:.L_ffffff8009c7b634]
            ldr w9,[x9,:lo12:.L_ffffff8009907980]
            cmp w8,w9
            b.ls .L_ffffff800870605c

            ldrh w2,[x21,:lo12:.L_ffffff8009c7b640]
            adrp x0, .L_ffffff800918470f
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff800918470f
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x20,:lo12:.L_ffffff8009c7b634]
            adrp x21, .L_ffffff8009907988
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24577
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldr w8,[x20,:lo12:.L_ffffff8009c7b634]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#24833
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            adrp x20, .L_ffffff8009c7b638
            ldrb w8,[x20,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#25089
            add x0,sp,#4
            cmp w8,#0
            mov w8,#13
            csel w8,w8,wzr,ne
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w8,[x20,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x21,:lo12:.L_ffffff8009907988]
            mov w9,#25345
            add x0,sp,#4
            cmp w8,#0
            mov w8,#120
            csel w8,w8,wzr,ne
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C
.L_ffffff800870605c:

            adrp x0, .L_ffffff8009c7b62c
            add x0,x0, :lo12:.L_ffffff8009c7b62c
.L_ffffff8008706064:

            bl _raw_spin_lock

            mov w8,#8663
            movk w8,#70,lsl #16
            cmp w19,w8
            b.ls .L_ffffff8008706094

            mov w9,#51243
            lsr w8,w19,#3
            movk w9,#4865,lsl #16
            umull x8,w8,w9
            lsr x8,x8,#37
            sub w8,w8,#1332
            b .L_ffffff8008706098
.L_ffffff8008706094:

            mov w8,wzr
.L_ffffff8008706098:

            and w9,w8,#65535
            adrp x20, .L_ffffff8009c7b640
.L_ffffff80087060a0:

            adrp x19, .L_ffffff8009c7b634
            adrp x10, .L_ffffff8009c7b63c
            strh w8,[x20,:lo12:.L_ffffff8009c7b640]
            add w8,w9,#1332
.L_ffffff80087060b0:

            adrp x0, .L_ffffff8009c7b62c
            add x0,x0, :lo12:.L_ffffff8009c7b62c
            str w8,[x19,:lo12:.L_ffffff8009c7b634]
            str w8,[x10,:lo12:.L_ffffff8009c7b63c]
            bl _raw_spin_unlock

            adrp x9, .L_ffffff8009907980
            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            ldr w9,[x9,:lo12:.L_ffffff8009907980]
            cmp w8,w9
            b.ls .L_ffffff8008706198
.L_ffffff80087060d8:

            ldrh w2,[x20,:lo12:.L_ffffff8009c7b640]
            adrp x0, .L_ffffff800918470f
            adrp x1, .L_ffffff800918474d
            add x0,x0, :lo12:.L_ffffff800918470f
            add x1,x1, :lo12:.L_ffffff800918474d
            mov w3,wzr
            bl printk

            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            adrp x20, .L_ffffff8009907988
            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w9,#24577
            lsr w8,w8,#8
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldr w8,[x19,:lo12:.L_ffffff8009c7b634]
            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w9,#24833
            add x0,sp,#4
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            adrp x19, .L_ffffff8009c7b638
            ldrb w8,[x19,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w9,#25089
            add x0,sp,#4
            cmp w8,#0
            mov w8,#13
            csel w8,w8,wzr,ne
            mov w1,#3
            strh w9,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w8,[x19,:lo12:.L_ffffff8009c7b638]
            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w9,#25345
            strh w9,[sp,#4]
            cmp w8,#0
            mov w8,#120
            csel w8,w8,wzr,ne
            strb w8,[sp,#6]
.L_ffffff800870618c:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C
.L_ffffff8008706198:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff80087061cc

            ldp x29,lr,[sp,#80]
            ldp x20,x19,[sp,#64]
            ldp x22,x21,[sp,#48]
            ldp x24,x23,[sp,#32]
            ldp x26,x25,[sp,#16]
            mov w0,wzr
            add sp,sp,#96
            ret
.L_ffffff80087061cc:

            bl __stack_chk_fail
.size feature_control, . - feature_control
.size FUN_ffffff8008704f20, . - FUN_ffffff8008704f20
#-----------------------------------
.type FUN_ffffff80087061d0, @function
#-----------------------------------
FUN_ffffff80087061d0:
#-----------------------------------
.type control, @function
#-----------------------------------
control:

            sub sp,sp,#96
            str x25,[sp,#16]
            stp x24,x23,[sp,#32]
            stp x22,x21,[sp,#48]
            stp x20,x19,[sp,#64]
            stp x29,lr,[sp,#80]
            add x29,sp,#80
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov w19,w0
            adrp x0, .L_ffffff8009184446
            adrp x1, .L_ffffff80092670e3
            add x0,x0, :lo12:.L_ffffff8009184446
            add x1,x1, :lo12:.L_ffffff80092670e3
            mov w2,w19
            str x8,[sp,#8]
            bl printk

            adrp x20, .L_ffffff8009c7b62c
            add x20,x20, :lo12:.L_ffffff8009c7b62c
            mov x0,x20
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b644
            mov x0,x20
            str w19,[x8,:lo12:.L_ffffff8009c7b644]
            bl _raw_spin_unlock

            cmp w19,#9
            b.hi .L_ffffff800870626c

            adrp x9, .L_ffffff8008f296b6
            mov w8,w19
            add x9,x9, :lo12:.L_ffffff8008f296b6
            adr x10, .L_ffffff8008706258
            ldrh w11,[x9,x8,lsl #1]
            add x10,x10,x11, lsl #2
            br x10
.L_ffffff8008706258:

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff80091c6b2a
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff80091c6b2a
            b .L_ffffff8008707a6c
.L_ffffff800870626c:

            adrp x0, .L_ffffff8009184ace
            adrp x1, .L_ffffff80092670e3
            add x0,x0, :lo12:.L_ffffff8009184ace
            add x1,x1, :lo12:.L_ffffff80092670e3
            bl printk

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff80091c6b2a
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff80091c6b2a
            bl printk

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
.L_ffffff800870629c:

            mov x0,x19
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b630
            mov w9,#1
            adrp x10, .L_ffffff8009c7b638
            adrp x11, .L_ffffff8009c7b634
            mov w12,#1332
            adrp x13, .L_ffffff8009c7b63c
            adrp x14, .L_ffffff8009c7b642
            mov x0,x19
            strb w9,[x8,:lo12:.L_ffffff8009c7b630]
            strb w9,[x10,:lo12:.L_ffffff8009c7b638]
            str w12,[x11,:lo12:.L_ffffff8009c7b634]
            str w12,[x13,:lo12:.L_ffffff8009c7b63c]
            strb wzr,[x14,:lo12:.L_ffffff8009c7b642]
            bl _raw_spin_unlock

            bl FUN_ffffff8008708050

            adrp x19, .L_ffffff8009184b30
            adrp x0, .L_ffffff8009184b02
            add x19,x19, :lo12:.L_ffffff8009184b30
            add x0,x0, :lo12:.L_ffffff8009184b02
            mov x1,x19
            mov w2,wzr
            bl printk

            adrp x0, .L_ffffff8009184b40
            add x0,x0, :lo12:.L_ffffff8009184b40
            mov x1,x19
            bl printk

            adrp x20, .L_ffffff8009907988
            ldrb w4,[x20,:lo12:.L_ffffff8009907988]
            mov w21,#29185
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w21,[sp]
            bl iReadRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#3
            add x0,sp,#4
            mov w1,#3
            strh w21,[sp,#4]
            strb w8,[sp,#6]
            bl iWriteRegI2C

            adrp x0, .L_ffffff8009184b75
            add x0,x0, :lo12:.L_ffffff8009184b75
            mov x1,x19
            bl printk

            mov w0,#4
            b .L_ffffff8008707b40

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff8009262aba
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff8009262aba
            bl printk

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            mov x0,x19
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b630
            mov w22,#1
            adrp x9, .L_ffffff8009c7b638
            adrp x10, .L_ffffff8009c7b634
            mov w11,#2544
            adrp x12, .L_ffffff8009c7b63c
            adrp x13, .L_ffffff8009c7b642
            mov x0,x19
            strb w22,[x8,:lo12:.L_ffffff8009c7b630]
            strb w22,[x9,:lo12:.L_ffffff8009c7b638]
            str w11,[x10,:lo12:.L_ffffff8009c7b634]
            str w11,[x12,:lo12:.L_ffffff8009c7b63c]
            strb wzr,[x13,:lo12:.L_ffffff8009c7b642]
            bl _raw_spin_unlock

            adrp x8, .L_ffffff8009907984
            ldrh w2,[x8,:lo12:.L_ffffff8009907984]
            adrp x0, .L_ffffff8009184bac
            adrp x1, .L_ffffff800918550f
            add x0,x0, :lo12:.L_ffffff8009184bac
            add x1,x1, :lo12:.L_ffffff800918550f
            bl printk

            adrp x20, .L_ffffff8009907988
            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w22,[sp,#4]
            strb wzr,[sp,#6]
            mov w21,#3
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w24,#60208
            mov w19,#5
            add x0,sp,#4
            mov w1,#3
            strh w24,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w23,#12
            add x0,sp,#4
            mov w1,#3
            strh w24,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2608
            mov w25,#255
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2864
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
.L_ffffff8008706480:

            mov w1,#3
            strh w24,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            strh w24,[sp,#4]
            mov w24,#9
            add x0,sp,#4
            mov w1,#3
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#5121
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#10241
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#10753
            strh w8,[sp,#4]
            mov w8,#24
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#11009
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24577
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24833
            strh w8,[sp,#4]
            mov w8,#240
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25089
.L_ffffff8008706564:

            strh w8,[sp,#4]
            mov w8,#13
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25345
            strh w8,[sp,#4]
            mov w8,#120
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25601
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25857
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26113
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26369
            strh w8,[sp,#4]
            mov w8,#207
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26625
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26881
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27137
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27393
.L_ffffff800870666c:

            strh w8,[sp,#4]
            mov w8,#159
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27649
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27905
            strh w8,[sp,#4]
            mov w8,#208
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28161
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28417
            strh w8,[sp,#4]
            mov w8,#160
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28673
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28929
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w23,#29185
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#29697
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#29953
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#35841
            mov w24,#10
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#36097
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#259
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#771
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#1027
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#1283
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#1539
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#1795
            strh w8,[sp,#4]
            mov w8,#54
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2307
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2819
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#3075
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#3331
            strh w8,[sp,#4]
            mov w8,#108
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24133
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#7751
            strh w8,[sp,#4]
            mov w8,#75
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26439
            strh w8,[sp,#4]
            mov w8,#15
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#20551
            mov w19,#20
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#16453
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#46151
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#4935
            strh w8,[sp,#4]
            mov w8,#48
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#35655
            mov w19,#16
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#36679
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#37703
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#38727
            mov w19,#14
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#39751
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w22,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            adrp x19, .L_ffffff8009184b30
            adrp x0, .L_ffffff8009184b02
            add x19,x19, :lo12:.L_ffffff8009184b30
            add x0,x0, :lo12:.L_ffffff8009184b02
            mov x1,x19
            mov w2,wzr
            bl printk

            adrp x0, .L_ffffff8009184b40
            add x0,x0, :lo12:.L_ffffff8009184b40
            mov x1,x19
            bl printk

            ldrb w4,[x20,:lo12:.L_ffffff8009907988]
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w23,[sp]
            bl iReadRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            adrp x0, .L_ffffff8009184b75
            add x0,x0, :lo12:.L_ffffff8009184b75
            mov x1,x19
            bl printk

            adrp x19, .L_ffffff8009c7b628
            ldrb w8,[x19,:lo12:.L_ffffff8009c7b628]
            cbz w8,.L_ffffff8008707b3c

            mov w0,#1
            bl set_test_pattern_mode

            mov w0,wzr
            strb wzr,[x19,:lo12:.L_ffffff8009c7b628]
            b .L_ffffff8008707b40

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff8009184bd7
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff8009184bd7
            bl printk

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            mov x0,x19
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b630
            mov w22,#1
            adrp x9, .L_ffffff8009c7b638
            adrp x10, .L_ffffff8009c7b634
            mov w11,#1332
            adrp x12, .L_ffffff8009c7b63c
            adrp x13, .L_ffffff8009c7b642
            mov x0,x19
            strb w22,[x8,:lo12:.L_ffffff8009c7b630]
            strb w22,[x9,:lo12:.L_ffffff8009c7b638]
            str w11,[x10,:lo12:.L_ffffff8009c7b634]
            str w11,[x12,:lo12:.L_ffffff8009c7b63c]
            strb wzr,[x13,:lo12:.L_ffffff8009c7b642]
            bl _raw_spin_unlock

            adrp x8, .L_ffffff8009907984
            ldrh w2,[x8,:lo12:.L_ffffff8009907984]
            adrp x0, .L_ffffff8009184bac
            adrp x1, .L_ffffff8009184be4
            add x0,x0, :lo12:.L_ffffff8009184bac
            add x1,x1, :lo12:.L_ffffff8009184be4
            bl printk

            adrp x20, .L_ffffff8009907988
            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w22,[sp,#4]
            strb wzr,[sp,#6]
            mov w21,#3
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w23,#60208
            mov w19,#5
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w24,#12
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2608
            mov w25,#255
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2864
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            strh w23,[sp,#4]
            mov w23,#9
            add x0,sp,#4
            mov w1,#3
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#5121
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#10241
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#10753
            strh w8,[sp,#4]
            mov w8,#24
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#11009
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24577
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24833
            strh w8,[sp,#4]
            mov w8,#52
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25089
            strh w8,[sp,#4]
            mov w8,#13
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25345
            strh w8,[sp,#4]
            mov w8,#120
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25601
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25857
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26113
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26369
            strh w8,[sp,#4]
            mov w8,#207
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26625
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26881
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27137
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27393
            strh w8,[sp,#4]
            mov w8,#159
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27649
            strh w8,[sp,#4]
            mov w8,#6
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27905
            strh w8,[sp,#4]
            mov w8,#104
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28161
            strh w8,[sp,#4]
            mov w8,#4
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28417
            strh w8,[sp,#4]
            mov w8,#208
            b .L_ffffff8008707638

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff8009184bf9
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff8009184bf9
            bl printk

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            mov x0,x19
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b630
            mov w22,#1
            adrp x9, .L_ffffff8009c7b638
            adrp x10, .L_ffffff8009c7b634
            mov w11,#2544
            adrp x12, .L_ffffff8009c7b63c
            adrp x13, .L_ffffff8009c7b640
            adrp x14, .L_ffffff8009c7b642
            mov x0,x19
            strb w22,[x8,:lo12:.L_ffffff8009c7b630]
            strb w22,[x9,:lo12:.L_ffffff8009c7b638]
            str w11,[x10,:lo12:.L_ffffff8009c7b634]
            str w11,[x12,:lo12:.L_ffffff8009c7b63c]
            strh wzr,[x13,:lo12:.L_ffffff8009c7b640]
            strb wzr,[x14,:lo12:.L_ffffff8009c7b642]
            bl _raw_spin_unlock

            adrp x8, .L_ffffff8009907984
            ldrh w2,[x8,:lo12:.L_ffffff8009907984]
            adrp x0, .L_ffffff8009184bac
            adrp x1, .L_ffffff8009184c02
            add x0,x0, :lo12:.L_ffffff8009184bac
            add x1,x1, :lo12:.L_ffffff8009184c02
            bl printk

            adrp x20, .L_ffffff8009907988
            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w22,[sp,#4]
            strb wzr,[sp,#6]
            mov w21,#3
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w24,#60208
            mov w19,#5
            add x0,sp,#4
            mov w1,#3
            strh w24,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w23,#12
            add x0,sp,#4
            mov w1,#3
            strh w24,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2608
            mov w25,#255
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2864
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w24,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            strh w24,[sp,#4]
            mov w24,#9
            add x0,sp,#4
            mov w1,#3
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#5121
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#10241
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#10753
            strh w8,[sp,#4]
            mov w8,#24
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#11009
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24577
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24833
            strh w8,[sp,#4]
            mov w8,#240
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25089
            strh w8,[sp,#4]
            mov w8,#13
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25345
            strh w8,[sp,#4]
            mov w8,#120
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25601
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25857
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26113
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26369
            strh w8,[sp,#4]
            mov w8,#207
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26625
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26881
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27137
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27393
            strh w8,[sp,#4]
            mov w8,#159
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27649
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27905
            strh w8,[sp,#4]
            mov w8,#208
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28161
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28417
            strh w8,[sp,#4]
            mov w8,#160
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28673
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28929
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w23,#29185
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#29697
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#29953
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            b .L_ffffff80087076c8

            adrp x19, .L_ffffff8009184428
            add x19,x19, :lo12:.L_ffffff8009184428
            adrp x1, .L_ffffff8009184c13
            add x1,x1, :lo12:.L_ffffff8009184c13
            mov x0,x19
            bl printk

            adrp x20, .L_ffffff8009c7b62c
            add x20,x20, :lo12:.L_ffffff8009c7b62c
            mov x0,x20
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b630
            mov w22,#1
            adrp x9, .L_ffffff8009c7b638
            adrp x10, .L_ffffff8009c7b634
            mov w11,#2544
            adrp x12, .L_ffffff8009c7b63c
            adrp x13, .L_ffffff8009c7b640
            adrp x14, .L_ffffff8009c7b642
            mov x0,x20
            strb w22,[x8,:lo12:.L_ffffff8009c7b630]
            strb w22,[x9,:lo12:.L_ffffff8009c7b638]
            str w11,[x10,:lo12:.L_ffffff8009c7b634]
            str w11,[x12,:lo12:.L_ffffff8009c7b63c]
            strh wzr,[x13,:lo12:.L_ffffff8009c7b640]
            strb wzr,[x14,:lo12:.L_ffffff8009c7b642]
            bl _raw_spin_unlock

            adrp x1, .L_ffffff8009184c1e
            add x1,x1, :lo12:.L_ffffff8009184c1e
            mov x0,x19
            bl printk

            adrp x20, .L_ffffff8009907988
            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w22,[sp,#4]
            strb wzr,[sp,#6]
            mov w21,#3
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w24,#60208
            mov w19,#5
            add x0,sp,#4
            mov w1,#3
            strh w24,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w23,#12
            add x0,sp,#4
            mov w1,#3
            strh w24,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2608
            mov w25,#255
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2864
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w24,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            strh w24,[sp,#4]
            mov w24,#9
            add x0,sp,#4
            mov w1,#3
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#5121
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#10241
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#10753
            strh w8,[sp,#4]
            mov w8,#24
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#11009
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24577
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24833
            strh w8,[sp,#4]
            mov w8,#240
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25089
            strh w8,[sp,#4]
            mov w8,#13
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25345
            strh w8,[sp,#4]
            mov w8,#120
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25601
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#25857
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26113
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26369
            strh w8,[sp,#4]
            mov w8,#207
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26625
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26881
            strh w8,[sp,#4]
            mov w8,#50
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27137
            strh w8,[sp,#4]
            mov w8,#8
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27393
            strh w8,[sp,#4]
            mov w8,#109
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27649
            strh w8,[sp,#4]
            mov w8,#6
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#27905
            strh w8,[sp,#4]
            mov w8,#104
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28161
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28417
            strh w8,[sp,#4]
            mov w8,#158
.L_ffffff8008707638:

            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28673
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#28929
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w23,#29185
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#29697
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#29953
            strh w8,[sp,#4]
            strb w22,[sp,#6]
.L_ffffff80087076c8:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#35841
            mov w24,#10
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#36097
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#259
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#771
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#1027
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#1283
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#1539
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#1795
            strh w8,[sp,#4]
            mov w8,#54
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2307
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#2819
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#3075
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#3331
            strh w8,[sp,#4]
            mov w8,#108
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#24133
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#7751
            strh w8,[sp,#4]
            mov w8,#75
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#26439
            strh w8,[sp,#4]
            mov w8,#15
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#20551
            mov w19,#20
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#16453
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#46151
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#4935
            strh w8,[sp,#4]
            mov w8,#48
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#35655
            mov w19,#16
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#36679
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#37703
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#38727
            mov w19,#14
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#39751
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w19,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w22,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            adrp x19, .L_ffffff8009184b30
            adrp x0, .L_ffffff8009184b02
            add x19,x19, :lo12:.L_ffffff8009184b30
            add x0,x0, :lo12:.L_ffffff8009184b02
            mov x1,x19
            mov w2,wzr
            bl printk

            adrp x0, .L_ffffff8009184b40
            add x0,x0, :lo12:.L_ffffff8009184b40
            mov x1,x19
            bl printk

            ldrb w4,[x20,:lo12:.L_ffffff8009907988]
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w23,[sp]
            bl iReadRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            strh w23,[sp,#4]
            strb w21,[sp,#6]
            b .L_ffffff8008707b20

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff8009184c31
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff8009184c31
            b .L_ffffff8008707a6c

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff8009184c39
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff8009184c39
            b .L_ffffff8008707a6c

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff8009184c41
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff8009184c41
            b .L_ffffff8008707a6c

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff8009184c49
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff8009184c49
            b .L_ffffff8008707a6c

            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff8009184c51
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff8009184c51
.L_ffffff8008707a6c:

            bl printk

            adrp x19, .L_ffffff8009c7b62c
            add x19,x19, :lo12:.L_ffffff8009c7b62c
            mov x0,x19
            bl _raw_spin_lock

            adrp x8, .L_ffffff8009c7b630
            mov w9,#1
            adrp x10, .L_ffffff8009c7b638
            adrp x11, .L_ffffff8009c7b634
            mov w12,#1332
            adrp x13, .L_ffffff8009c7b63c
            adrp x14, .L_ffffff8009c7b642
            mov x0,x19
            strb w9,[x8,:lo12:.L_ffffff8009c7b630]
            strb w9,[x10,:lo12:.L_ffffff8009c7b638]
            str w12,[x11,:lo12:.L_ffffff8009c7b634]
            str w12,[x13,:lo12:.L_ffffff8009c7b63c]
            strb wzr,[x14,:lo12:.L_ffffff8009c7b642]
            bl _raw_spin_unlock

            bl FUN_ffffff8008708050

            adrp x19, .L_ffffff8009184b30
            adrp x0, .L_ffffff8009184b02
            add x19,x19, :lo12:.L_ffffff8009184b30
            add x0,x0, :lo12:.L_ffffff8009184b02
            mov x1,x19
            mov w2,wzr
            bl printk

            adrp x0, .L_ffffff8009184b40
            add x0,x0, :lo12:.L_ffffff8009184b40
            mov x1,x19
            bl printk

            adrp x20, .L_ffffff8009907988
            ldrb w4,[x20,:lo12:.L_ffffff8009907988]
            mov w21,#29185
            mov x0,sp
            add x2,sp,#4
            mov w1,#2
            mov w3,#1
            strh wzr,[sp,#4]
            strh w21,[sp]
            bl iReadRegI2C

            ldrb w2,[x20,:lo12:.L_ffffff8009907988]
            mov w8,#3
            strh w21,[sp,#4]
            strb w8,[sp,#6]
.L_ffffff8008707b20:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            adrp x0, .L_ffffff8009184b75
            add x0,x0, :lo12:.L_ffffff8009184b75
            mov x1,x19
            bl printk
.L_ffffff8008707b3c:

            mov w0,wzr
.L_ffffff8008707b40:

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008707b70

            ldp x29,lr,[sp,#80]
            ldp x20,x19,[sp,#64]
            ldp x22,x21,[sp,#48]
            ldp x24,x23,[sp,#32]
            ldr x25,[sp,#16]
            add sp,sp,#96
            ret
.L_ffffff8008707b70:

            bl __stack_chk_fail
.size control, . - control
.size FUN_ffffff80087061d0, . - FUN_ffffff80087061d0
#-----------------------------------
.type FUN_ffffff8008707b74, @function
#-----------------------------------
FUN_ffffff8008707b74:
#-----------------------------------
.type close, @function
#-----------------------------------
close:

            stp x29,lr,[sp,#-16]!
            mov x29,sp
            adrp x0, .L_ffffff8009184428
            adrp x1, .L_ffffff800916e19a
            add x0,x0, :lo12:.L_ffffff8009184428
            add x1,x1, :lo12:.L_ffffff800916e19a
            bl printk

            mov w0,wzr
            ldp x29,lr,[sp],#16
            ret
.size close, . - close
.size FUN_ffffff8008707b74, . - FUN_ffffff8008707b74
#-----------------------------------
.type set_test_pattern_mode, @function
#-----------------------------------
set_test_pattern_mode:

            sub sp,sp,#80
            stp x24,x23,[sp,#16]
            stp x22,x21,[sp,#32]
            stp x20,x19,[sp,#48]
            stp x29,lr,[sp,#64]
            add x29,sp,#64
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            mov w19,w0
            and w2,w0,#1
            adrp x0, .L_ffffff8009184a1b
            adrp x1, .L_ffffff8009184a42
            add x0,x0, :lo12:.L_ffffff8009184a1b
            add x1,x1, :lo12:.L_ffffff8009184a42
            str x8,[sp,#8]
            bl printk

            adrp x8, .L_ffffff8009c7b644
            ldr w8,[x8,:lo12:.L_ffffff8009c7b644]
            cmp w8,#1
            b.ne .L_ffffff8008707c04

            adrp x8, .L_ffffff8009c7b628
            tbz w19,#0,.L_ffffff8008707d1c

            mov w9,#1
            adrp x19, .L_ffffff8009907988
            strb w9,[x8,:lo12:.L_ffffff8009c7b628]
            b .L_ffffff8008707d24
.L_ffffff8008707c04:

            adrp x8, .L_ffffff8009c7b628
            tbz w19,#0,.L_ffffff8008707e80

            adrp x19, .L_ffffff8009907988
            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w9,#1
            mov w20,#6
            add x0,sp,#4
            mov w1,#3
            strb w9,[x8,:lo12:.L_ffffff8009c7b628]
            strh w20,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#262
            mov w21,#2
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9222
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9478
            mov w22,#104
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9734
            mov w23,#4
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9990
            mov w24,#208
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#10337
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#10593
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#15457
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            b .L_ffffff8008707e28
.L_ffffff8008707d1c:

            adrp x19, .L_ffffff8009907988
            strb wzr,[x8,:lo12:.L_ffffff8009c7b628]
.L_ffffff8008707d24:

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#6
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#262
            mov w20,#2
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9222
            mov w21,#12
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9478
            mov w22,#208
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9734
            mov w23,#9
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9990
            mov w24,#160
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#10337
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#10593
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#15457
            strh w8,[sp,#4]
            strb w21,[sp,#6]
.L_ffffff8008707e28:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#15713
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#15969
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#16225
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            b .L_ffffff8008707fdc
.L_ffffff8008707e80:

            adrp x19, .L_ffffff8009907988
            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w20,#6
            add x0,sp,#4
            mov w1,#3
            strb wzr,[x8,:lo12:.L_ffffff8009c7b628]
            strh w20,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#262
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9222
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9478
            mov w21,#104
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9734
            mov w22,#4
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#9990
            mov w23,#208
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#10337
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#10593
            strh w8,[sp,#4]
            mov w8,#2
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#15457
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#15713
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#15969
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#16225
            strh w8,[sp,#4]
            strb w23,[sp,#6]
.L_ffffff8008707fdc:

            add x0,sp,#4
            mov w1,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#1637
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#1893
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff800870804c

            ldp x29,lr,[sp,#64]
            ldp x20,x19,[sp,#48]
            ldp x22,x21,[sp,#32]
            ldp x24,x23,[sp,#16]
            add sp,sp,#80
            ret
.L_ffffff800870804c:

            bl __stack_chk_fail
.size set_test_pattern_mode, . - set_test_pattern_mode
#-----------------------------------
.type FUN_ffffff8008708050, @function
#-----------------------------------
FUN_ffffff8008708050:
#-----------------------------------
.type preview_setting, @function
#-----------------------------------
preview_setting:

            sub sp,sp,#96
            str x25,[sp,#16]
            stp x24,x23,[sp,#32]
            stp x22,x21,[sp,#48]
            stp x20,x19,[sp,#64]
            stp x29,lr,[sp,#80]
            add x29,sp,#80
            adrp x8, .L_ffffff800984ea88
            ldr x8,[x8,:lo12:.L_ffffff800984ea88]
            adrp x19, .L_ffffff8009907988
            mov w20,#1
            add x0,sp,#4
            str x8,[sp,#8]
            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w1,#3
            strh w20,[sp,#4]
            strb wzr,[sp,#6]
            mov w21,#3
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w23,#60208
            mov w22,#5
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w24,#12
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#2608
            mov w25,#255
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#2864
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w25,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w23,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            strh w23,[sp,#4]
            mov w23,#9
            add x0,sp,#4
            mov w1,#3
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#5121
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#10241
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#10753
            strh w8,[sp,#4]
            mov w8,#24
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#11009
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#24577
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#24833
            strh w8,[sp,#4]
            mov w8,#52
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#25089
            strh w8,[sp,#4]
            mov w8,#13
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#25345
            strh w8,[sp,#4]
            mov w8,#120
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#25601
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#25857
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#26113
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w24,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#26369
            strh w8,[sp,#4]
            mov w8,#207
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#26625
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#26881
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#27137
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#27393
            strh w8,[sp,#4]
            mov w8,#159
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#27649
            strh w8,[sp,#4]
            mov w8,#6
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#27905
            strh w8,[sp,#4]
            mov w8,#104
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#28161
            strh w8,[sp,#4]
            mov w8,#4
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#28417
            strh w8,[sp,#4]
            mov w8,#208
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#28673
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#28929
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#29185
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#29697
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#29953
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#35841
            mov w23,#10
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#36097
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#259
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w22,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#771
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#1027
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#1283
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#1539
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#1795
            strh w8,[sp,#4]
            mov w8,#54
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#2307
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w23,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#2819
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#3075
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#3331
            strh w8,[sp,#4]
            mov w8,#108
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#24133
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#7751
            strh w8,[sp,#4]
            mov w8,#75
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#26439
            strh w8,[sp,#4]
            mov w8,#15
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#20551
            mov w21,#20
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#16453
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb wzr,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#46151
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#4935
            strh w8,[sp,#4]
            mov w8,#48
            add x0,sp,#4
            mov w1,#3
            strb w8,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#35655
            mov w21,#16
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#36679
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#37703
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#38727
            mov w21,#14
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            mov w8,#39751
            add x0,sp,#4
            mov w1,#3
            strh w8,[sp,#4]
            strb w21,[sp,#6]
            bl iWriteRegI2C

            ldrb w2,[x19,:lo12:.L_ffffff8009907988]
            add x0,sp,#4
            mov w1,#3
            strh w20,[sp,#4]
            strb w20,[sp,#6]
            bl iWriteRegI2C

            adrp x9, .L_ffffff800984ea88
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:.L_ffffff800984ea88]
            cmp x9,x8
            b.ne .L_ffffff8008708738

            ldp x29,lr,[sp,#80]
            ldp x20,x19,[sp,#64]
            ldp x22,x21,[sp,#48]
            ldp x24,x23,[sp,#32]
            ldr x25,[sp,#16]
            add sp,sp,#96
            ret
.L_ffffff8008708738:

            bl __stack_chk_fail
.size preview_setting, . - preview_setting
.size FUN_ffffff8008708050, . - FUN_ffffff8008708050
.align 2

.pushsection ".imx219.data.write", "aw"
#include "imx219mipiraw_data.S"
.popsection
