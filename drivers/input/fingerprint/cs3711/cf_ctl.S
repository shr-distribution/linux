#include <asm/assembler.h>

.extern __stack_chk_guard
.extern system_wq

#-----------------------------------
.type cf_probe, @function
#-----------------------------------
cf_probe:

            sub sp,sp,#64
            stp x22,x21,[sp,#16]
            stp x20,x19,[sp,#32]
            stp x29,lr,[sp,#48]
            add x29,sp,#48
            adrp x8, __stack_chk_guard
            ldr x8,[x8,:lo12:__stack_chk_guard]
            mov x21,x0
            adrp x0, .L_ffffff8009228070
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff8009228070
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1528
            str x8,[sp,#8]
            bl printk

            adrp x0, .L_ffffff80092280a8
            add x0,x0, :lo12:.L_ffffff80092280a8
            bl printk

            adrp x2, .L_ffffff80092280d1
            add x2,x2, :lo12:.L_ffffff80092280d1
            mov x0,xzr
            mov x1,xzr
            bl of_find_compatible_node

            cmn x0,#4095
            b.lo .L_ffffff8008a5d74c

            adrp x0, .L_ffffff80092280e6
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff80092280e6
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1538
            bl printk

            mov w20,#-22
            b .L_ffffff8008a5dbac
.L_ffffff8008a5d74c:

            bl of_find_device_by_node

            cmn x0,#4095
            b.lo .L_ffffff8008a5d778

            mov x20,x0
            adrp x0, .L_ffffff8009228114
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff8009228114
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1545
            bl printk

            b .L_ffffff8008a5dbac
.L_ffffff8008a5d778:

            adrp x0, .L_ffffff8009228146
            add x0,x0, :lo12:.L_ffffff8009228146
            bl printk

            mov w2,#32960
            mov w1,#672
            movk w2,#320,lsl #16
            mov x0,x21
            bl devm_kmalloc

            cbz x0,.L_ffffff8008a5d82c

            mov w9,#33920
            adrp x10, .L_ffffff8009965458
            mov x19,x0
            stp x21,x21,[x0]
            str x0,[x21,#152]
            mov w8,#8
            movk w9,#30,lsl #16
            add x10,x10, :lo12:.L_ffffff8009965458
            mov x0,x21
            strh wzr,[x21,#734]
            strb w8,[x21,#733]
            str w9,[x21,#728]
            str x10,[x21,#752]
            bl spi_setup

            mov x0,x19
            bl cf_gpio_init

            adrp x0, .L_ffffff800922850c
            add x0,x0, :lo12:.L_ffffff800922850c
            bl printk

            ldr x0,[x19]
            bl devm_pinctrl_get

            str x0,[x19,#144]
            adrp x0, .L_ffffff8009228541
            add x0,x0, :lo12:.L_ffffff8009228541
            bl printk

            ldr x0,[x19,#144]
            cmn x0,#1,lsl #12
            b.ls .L_ffffff8008a5d84c

            adrp x0, .L_ffffff8009228579
            adrp x1, .L_ffffff80092285af
            add x0,x0, :lo12:.L_ffffff8009228579
            add x1,x1, :lo12:.L_ffffff80092285af
            mov w2,#748
            bl printk

            str xzr,[x19,#144]
            b .L_ffffff8008a5da4c
.L_ffffff8008a5d82c:

            adrp x0, .L_ffffff8009228170
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff8009228170
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1559
.L_ffffff8008a5d840:

            bl printk

            mov w20,#-12
            b .L_ffffff8008a5dbac
.L_ffffff8008a5d84c:

            adrp x20, .L_ffffff8009228623
            add x20,x20, :lo12:.L_ffffff8009228623
            mov x1,x20
            bl pinctrl_lookup_state

            cmn x0,#1,lsl #12
            b.hi .L_ffffff8008a5da30

            adrp x22, .L_ffffff8009965228
            ldrb w8,[x22,:lo12:.L_ffffff8009965228]
            mov x21,x0
            cbz w8,.L_ffffff8008a5d894

            adrp x0, .L_ffffff80092285f4
            adrp x1, .L_ffffff80092285af
            adrp x3, .L_ffffff8009228623
            add x0,x0, :lo12:.L_ffffff80092285f4
            add x1,x1, :lo12:.L_ffffff80092285af
            add x3,x3, :lo12:.L_ffffff8009228623
            mov w2,#764
            bl printk
.L_ffffff8008a5d894:

            ldr x0,[x19,#144]
            adrp x20, .L_ffffff8009228633
            add x20,x20, :lo12:.L_ffffff8009228633
            mov x1,x20
            str x21,[x19,#152]
            bl pinctrl_lookup_state

            cmn x0,#1,lsl #12
            b.hi .L_ffffff8008a5da30

            ldrb w8,[x22,:lo12:.L_ffffff8009965228]
            mov x21,x0
            cbz w8,.L_ffffff8008a5d8e0

            adrp x0, .L_ffffff80092285f4
            adrp x1, .L_ffffff80092285af
            adrp x3, .L_ffffff8009228633
            add x0,x0, :lo12:.L_ffffff80092285f4
            add x1,x1, :lo12:.L_ffffff80092285af
            add x3,x3, :lo12:.L_ffffff8009228633
            mov w2,#764
            bl printk
.L_ffffff8008a5d8e0:

            ldr x0,[x19,#144]
            adrp x20, .L_ffffff8009228642
            add x20,x20, :lo12:.L_ffffff8009228642
            mov x1,x20
            str x21,[x19,#160]
            bl pinctrl_lookup_state

            cmn x0,#1,lsl #12
            b.hi .L_ffffff8008a5da30

            ldrb w8,[x22,:lo12:.L_ffffff8009965228]
            mov x21,x0
            cbz w8,.L_ffffff8008a5d92c

            adrp x0, .L_ffffff80092285f4
            adrp x1, .L_ffffff80092285af
            adrp x3, .L_ffffff8009228642
            add x0,x0, :lo12:.L_ffffff80092285f4
            add x1,x1, :lo12:.L_ffffff80092285af
            add x3,x3, :lo12:.L_ffffff8009228642
            mov w2,#764
            bl printk
.L_ffffff8008a5d92c:

            ldr x0,[x19,#144]
            adrp x20, .L_ffffff800922864e
            add x20,x20, :lo12:.L_ffffff800922864e
            mov x1,x20
            str x21,[x19,#168]
            bl pinctrl_lookup_state

            cmn x0,#1,lsl #12
            b.hi .L_ffffff8008a5da30

            ldrb w8,[x22,:lo12:.L_ffffff8009965228]
            mov x21,x0
            cbz w8,.L_ffffff8008a5d978

            adrp x0, .L_ffffff80092285f4
            adrp x1, .L_ffffff80092285af
            adrp x3, .L_ffffff800922864e
            add x0,x0, :lo12:.L_ffffff80092285f4
            add x1,x1, :lo12:.L_ffffff80092285af
            add x3,x3, :lo12:.L_ffffff800922864e
            mov w2,#764
            bl printk
.L_ffffff8008a5d978:

            ldr x0,[x19,#144]
            adrp x20, .L_ffffff8009228664
            add x20,x20, :lo12:.L_ffffff8009228664
            mov x1,x20
            str x21,[x19,#176]
            bl pinctrl_lookup_state

            cmn x0,#1,lsl #12
            b.hi .L_ffffff8008a5da30

            ldrb w8,[x22,:lo12:.L_ffffff8009965228]
            mov x21,x0
            cbz w8,.L_ffffff8008a5d9c4

            adrp x0, .L_ffffff80092285f4
            adrp x1, .L_ffffff80092285af
            adrp x3, .L_ffffff8009228664
            add x0,x0, :lo12:.L_ffffff80092285f4
            add x1,x1, :lo12:.L_ffffff80092285af
            add x3,x3, :lo12:.L_ffffff8009228664
            mov w2,#764
            bl printk
.L_ffffff8008a5d9c4:

            ldr x0,[x19,#144]
            adrp x20, .L_ffffff8009228678
            add x20,x20, :lo12:.L_ffffff8009228678
            mov x1,x20
            str x21,[x19,#184]
            bl pinctrl_lookup_state

            cmn x0,#1,lsl #12
            b.hi .L_ffffff8008a5da30

            ldrb w8,[x22,:lo12:.L_ffffff8009965228]
            mov x21,x0
            cbz w8,.L_ffffff8008a5da10

            adrp x0, .L_ffffff80092285f4
            adrp x1, .L_ffffff80092285af
            adrp x3, .L_ffffff8009228678
            add x0,x0, :lo12:.L_ffffff80092285f4
            add x1,x1, :lo12:.L_ffffff80092285af
            add x3,x3, :lo12:.L_ffffff8009228678
            mov w2,#764
            bl printk
.L_ffffff8008a5da10:

            ldr x0,[x19,#144]
            adrp x20, .L_ffffff8009228687
            add x20,x20, :lo12:.L_ffffff8009228687
            mov x1,x20
            str x21,[x19,#192]
            bl pinctrl_lookup_state

            cmn x0,#1,lsl #12
            b.ls .L_ffffff8008a5decc
.L_ffffff8008a5da30:

            adrp x0, .L_ffffff80092285c9
            adrp x1, .L_ffffff80092285af
            add x0,x0, :lo12:.L_ffffff80092285c9
            add x1,x1, :lo12:.L_ffffff80092285af
            mov w2,#760
            mov x3,x20
            bl printk
.L_ffffff8008a5da4c:

            mov x0,x19
            bl hw_reset

            adrp x0, .L_ffffff80092286f0
            adrp x1, .L_ffffff800922871d
            add x0,x0, :lo12:.L_ffffff80092286f0
            add x1,x1, :lo12:.L_ffffff800922871d
            mov w2,#515
            bl printk

            ldr x0,[x19,#8]
            bl mt_spi_enable_master_clk

            mov x0,x19
            bl hw_reset

            adrp x0, .L_ffffff8009965458
            mov w8,#16093
            add x0,x0, :lo12:.L_ffffff8009965458
            mov x2,sp
            add x3,sp,#4
            mov w1,#16
            mov w4,#4
            strh w8,[sp]
            bl tee_spi_transfer

            ldrb w4,[sp,#7]
            ldrb w3,[sp,#6]
            adrp x0, .L_ffffff800922872f
            adrp x1, .L_ffffff800922876e
            add x0,x0, :lo12:.L_ffffff800922872f
            add x1,x1, :lo12:.L_ffffff800922876e
            mov w2,#1489
            bl printk

            adrp x0, .L_ffffff80092281b8
            add x0,x0, :lo12:.L_ffffff80092281b8
            bl printk

            adrp x0, .L_ffffff8009228783
            add x0,x0, :lo12:.L_ffffff8009228783
            bl printk

            ldr x0,[x19,#8]
            bl mt_spi_disable_master_clk

            adrp x1, .L_ffffff80092281d6
            adrp x2, .L_ffffff800a0fb390
            add x1,x1, :lo12:.L_ffffff80092281d6
            add x2,x2, :lo12:.L_ffffff800a0fb390
            mov x0,xzr
            bl __class_create

            adrp x3, .L_ffffff80092281e1
            str x0,[x19,#120]
            add x0,x19,#136
            add x3,x3, :lo12:.L_ffffff80092281e1
            mov w2,#1
            mov w1,wzr
            bl alloc_chrdev_region

            cbz w0,.L_ffffff8008a5db34

            mov w20,w0
            adrp x0, .L_ffffff80092281e8
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff80092281e8
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1686
            b .L_ffffff8008a5dba4
.L_ffffff8008a5db34:

            ldr x0,[x19,#120]
            ldr w2,[x19,#136]
            adrp x4, .L_ffffff80090df3f3
            adrp x5, .L_ffffff80092281e1
            add x4,x4, :lo12:.L_ffffff80090df3f3
            add x5,x5, :lo12:.L_ffffff80092281e1
            mov x1,xzr
            mov x3,xzr
            bl device_create

            add x20,x19,#16
            adrp x1, .L_ffffff8008f5ef20
            str x0,[x19,#128]
            add x1,x1, :lo12:.L_ffffff8008f5ef20
            mov x0,x20
            bl cdev_init

            ldr w1,[x19,#136]
            mov w2,#1
            mov x0,x20
            str xzr,[x19,#80]
            mov w21,#1
            bl cdev_add

            cbz w0,.L_ffffff8008a5dbd8

            mov w20,w0
            adrp x0, .L_ffffff8009228229
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff8009228229
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1696
.L_ffffff8008a5dba4:

            mov w3,w20
            bl printk
.L_ffffff8008a5dbac:

            adrp x9, __stack_chk_guard
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:__stack_chk_guard]
            cmp x9,x8
            b.ne .L_ffffff8008a5df78

            mov w0,w20
            ldp x29,lr,[sp,#48]
            ldp x20,x19,[sp,#32]
            ldp x22,x21,[sp,#16]
            add sp,sp,#64
            ret
.L_ffffff8008a5dbd8:

            adrp x1, .L_ffffff800922825f
            adrp x2, .L_ffffff800a0fb391
            add x8,x19,#653
            add x0,x19,#608
            add x1,x1, :lo12:.L_ffffff800922825f
            add x2,x2, :lo12:.L_ffffff800a0fb391
            strh wzr,[x8]
            bl __mutex_init

            add x20,x19,#400
            adrp x1, .L_ffffff800922826d
            add x1,x1, :lo12:.L_ffffff800922826d
            mov x0,x20
            str wzr,[x19,#640]
            bl wakeup_source_prepare

            mov x0,x20
            bl wakeup_source_add

            mov x8,#68719476704
            add x9,x19,#256
            str w21,[x19,#648]
            adrp x21, .L_ffffff8009965228
            str x8,[x19,#248]
            str x9,[x19,#256]
            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            adrp x10, cf_device_event
            add x10,x10, :lo12:cf_device_event
            stp x9,x10,[x19,#264]
            cmp w8,#2
            b.lo .L_ffffff8008a5dc60

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228cf8
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228cf8
            mov w2,#841
            bl printk
.L_ffffff8008a5dc60:

            bl input_allocate_device

            str x0,[x19,#232]
            cbz x0,.L_ffffff8008a5ddd0

            adrp x8, .L_ffffff8009228d3f
            add x8,x8, :lo12:.L_ffffff8009228d3f
            str x8,[x0]
            ldr x8,[x19,#232]
            adrp x0, .L_ffffff80090f9cc8
            add x0,x0, :lo12:.L_ffffff80090f9cc8
            ldr x9,[x8,#40]
            orr x9,x9,#2
            str x9,[x8,#40]
            ldr x8,[x19,#232]
            ldr x9,[x8,#56]
            orr x9,x9,#274877906944
            str x9,[x8,#56]
            ldr x8,[x19,#232]
            ldr x9,[x8,#64]
            orr x9,x9,#2048
            str x9,[x8,#64]
            ldr x8,[x19,#232]
            ldr x9,[x8,#64]
            orr x9,x9,#1073741824
            str x9,[x8,#64]
            ldr x8,[x19,#232]
            ldr x9,[x8,#64]
            orr x9,x9,#1152921504606846976
            str x9,[x8,#64]
            ldr x8,[x19,#232]
            ldr x9,[x8,#64]
            orr x9,x9,#2305843009213693952
            str x9,[x8,#64]
            ldr x8,[x19,#232]
            ldr x9,[x8,#64]
            orr x9,x9,#4611686018427387904
            str x9,[x8,#64]
            ldr x8,[x19,#232]
            ldr x9,[x8,#64]
            orr x9,x9,#-9223372036854775808
            str x9,[x8,#64]
            ldr x8,[x19,#232]
            ldr x9,[x8,#40]
            orr x9,x9,#268435456
            str x9,[x8,#40]
            ldr x8,[x19,#232]
            ldr x9,[x8,#56]
            orr x9,x9,#549755813888
            str x9,[x8,#56]
            ldr x8,[x19,#232]
            ldr x9,[x8,#56]
            orr x9,x9,#2199023255552
            str x9,[x8,#56]
            ldr x8,[x19,#232]
            ldr x9,[x8,#56]
            orr x9,x9,#4398046511104
            str x9,[x8,#56]
            ldr x8,[x19,#232]
            ldr x9,[x8,#56]
            orr x9,x9,#17592186044416
            str x9,[x8,#56]
            ldr x8,[x19,#232]
            ldr x9,[x8,#64]
            orr x9,x9,#32768
            str x9,[x8,#64]
            bl agold_strtol

            ldr x8,[x19,#232]
            add w9,w0,#63
            cmp w0,#0
            csel w9,w9,w0,lt
            asr w10,w9,#6
            add x8,x8,w10, sxtw #3
            ldr x10,[x8,#48]
            and w9,w9,#4294967232
            sub w9,w0,w9
            mov w11,#1
            lsl x9,x11,x9
            orr x9,x9,x10
            str x9,[x8,#48]
            ldr x0,[x19,#232]
            bl input_register_device

            cbz w0,.L_ffffff8008a5de08

            adrp x0, .L_ffffff8009228d47
            adrp x1, .L_ffffff8009228cf8
            add x0,x0, :lo12:.L_ffffff8009228d47
            add x1,x1, :lo12:.L_ffffff8009228cf8
            mov w2,#871
            bl printk

            ldr x0,[x19,#232]
            bl input_free_device

            mov w20,#-19
            str xzr,[x19,#232]
            b .L_ffffff8008a5ddec
.L_ffffff8008a5ddd0:

            adrp x0, .L_ffffff8009228d06
            adrp x1, .L_ffffff8009228cf8
            add x0,x0, :lo12:.L_ffffff8009228d06
            add x1,x1, :lo12:.L_ffffff8009228cf8
            mov w2,#845
            bl printk

            mov w20,#-12
.L_ffffff8008a5ddec:

            adrp x0, .L_ffffff8009228277
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff8009228277
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1719
            bl printk

            b .L_ffffff8008a5dbac
.L_ffffff8008a5de08:

            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            cmp w8,#2
            b.lo .L_ffffff8008a5de2c

            adrp x0, .L_ffffff800922838f
            adrp x1, .L_ffffff8009228cf8
            add x0,x0, :lo12:.L_ffffff800922838f
            add x1,x1, :lo12:.L_ffffff8009228cf8
            mov w2,#877
            bl printk
.L_ffffff8008a5de2c:

            adrp x0, .L_ffffff80092281e1
            add x0,x0, :lo12:.L_ffffff80092281e1
            mov w1,#-1
            bl platform_device_alloc

            str x0,[x19,#280]
            cbz x0,.L_ffffff8008a5de74

            bl platform_device_add

            cbz w0,.L_ffffff8008a5de8c

            mov w20,w0
            adrp x0, .L_ffffff80092282e1
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff80092282e1
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1734
            bl printk

            ldr x0,[x19,#280]
            bl platform_device_del

            b .L_ffffff8008a5dbac
.L_ffffff8008a5de74:

            adrp x0, .L_ffffff80092282aa
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff80092282aa
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1727
            b .L_ffffff8008a5d840
.L_ffffff8008a5de8c:

            ldr x8,[x19,#280]
            adrp x1, .L_ffffff8008f5f010
            add x1,x1, :lo12:.L_ffffff8008f5f010
            str x19,[x8,#168]
            ldr x8,[x19,#280]
            add x0,x8,#32
            bl sysfs_create_group

            cbz w0,.L_ffffff8008a5df00

            mov w20,w0
            adrp x0, .L_ffffff8009228316
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff8009228316
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1746
            bl printk

            b .L_ffffff8008a5dbac
.L_ffffff8008a5decc:

            ldrb w8,[x22,:lo12:.L_ffffff8009965228]
            mov x21,x0
            cbz w8,.L_ffffff8008a5def8

            adrp x0, .L_ffffff80092285f4
            adrp x1, .L_ffffff80092285af
            adrp x3, .L_ffffff8009228687
            add x0,x0, :lo12:.L_ffffff80092285f4
            add x1,x1, :lo12:.L_ffffff80092285af
            add x3,x3, :lo12:.L_ffffff8009228687
            mov w2,#764
            bl printk
.L_ffffff8008a5def8:

            str x21,[x19,#200]
            b .L_ffffff8008a5da4c
.L_ffffff8008a5df00:

            mov x0,x19
            bl cf_irq_init

            cbz w0,.L_ffffff8008a5df2c

            mov w20,w0
            adrp x0, .L_ffffff800922834a
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff800922834a
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1757
            bl printk

            b .L_ffffff8008a5dbac
.L_ffffff8008a5df2c:

            adrp x8, FUN_ffffff8008a5e404
            add x0,x19,#376
            add x8,x8, :lo12:FUN_ffffff8008a5e404
            str x8,[x19,#376]
            bl fb_register_client

            adrp x0, .L_ffffff800922837a
            add x0,x0, :lo12:.L_ffffff800922837a
            bl printk

            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            cmp w8,#2
            b.lo .L_ffffff8008a5df70

            adrp x0, .L_ffffff800922838f
            adrp x1, .L_ffffff800922809f
            add x0,x0, :lo12:.L_ffffff800922838f
            add x1,x1, :lo12:.L_ffffff800922809f
            mov w2,#1775
            bl printk
.L_ffffff8008a5df70:

            mov w20,wzr
            b .L_ffffff8008a5dbac
.L_ffffff8008a5df78:

            bl __stack_chk_fail
.size cf_probe, . - cf_probe
#-----------------------------------
.type cf_remove, @function
#-----------------------------------
cf_remove:

            stp x20,x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            ldr x20,[x0,#152]
            ldr x8,[x20,#376]
            cbz x8,.L_ffffff8008a5dfa0

            add x0,x20,#376
            str xzr,[x0]
            bl fb_unregister_client
.L_ffffff8008a5dfa0:

            ldr x8,[x20,#280]
            adrp x1, .L_ffffff8008f5f010
            add x1,x1, :lo12:.L_ffffff8008f5f010
            add x0,x8,#32
            bl sysfs_remove_group

            ldr x0,[x20,#120]
            bl class_destroy

            add x0,x20,#400
            bl __pm_relax

            adrp x0, .L_ffffff8009228f54
            add x0,x0, :lo12:.L_ffffff8009228f54
            bl printk

            ldr x19,[x20,#224]
            cbz x19,.L_ffffff8008a5e018

            mov x0,x19
            bl regulator_is_enabled

            cbz w0,.L_ffffff8008a5e00c

            mov x0,x19
            bl regulator_disable

            adrp x0, .L_ffffff8009228fa2
            adrp x1, .L_ffffff8009228f97
            adrp x3, .L_ffffff8009228f46
            add x0,x0, :lo12:.L_ffffff8009228fa2
            add x1,x1, :lo12:.L_ffffff8009228f97
            add x3,x3, :lo12:.L_ffffff8009228f46
            mov w2,#357
            bl printk
.L_ffffff8008a5e00c:

            mov x0,x19
            bl regulator_put

            str xzr,[x20,#224]
.L_ffffff8008a5e018:

            adrp x8, .L_ffffff8009965228
            ldrb w8,[x8,:lo12:.L_ffffff8009965228]
            cmp w8,#2
            b.lo .L_ffffff8008a5e040

            adrp x0, .L_ffffff800922838f
            adrp x1, .L_ffffff8009228f4a
            add x0,x0, :lo12:.L_ffffff800922838f
            add x1,x1, :lo12:.L_ffffff8009228f4a
            mov w2,#1809
            bl printk
.L_ffffff8008a5e040:

            ldp x29,lr,[sp,#16]
            mov w0,wzr
            ldp x20,x19,[sp],#32
            ret
.size cf_remove, . - cf_remove
#-----------------------------------
.type cf_gpio_init, @function
#-----------------------------------
cf_gpio_init:

            str x21,[sp,#-48]!
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x2, .L_ffffff80092280d1
            mov x19,x0
            add x2,x2, :lo12:.L_ffffff80092280d1
            mov x0,xzr
            mov x1,xzr
            bl of_find_compatible_node

            cbz x0,.L_ffffff8008a5e12c

            adrp x21, .L_ffffff8009965228
            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            mov x20,x0
            cbz w8,.L_ffffff8008a5e0a4

            adrp x0, .L_ffffff80092283ae
            adrp x1, .L_ffffff80092283e6
            add x0,x0, :lo12:.L_ffffff80092283ae
            add x1,x1, :lo12:.L_ffffff80092283e6
            mov w2,#616
            bl printk
.L_ffffff8008a5e0a4:

            adrp x1, .L_ffffff8009204464
            add x1,x1, :lo12:.L_ffffff8009204464
            mov x0,x20
            mov w2,wzr
            mov x3,xzr
            bl of_get_named_gpio_flags

            mov w2,w0
            tbnz w0,#31,.L_ffffff8008a5e0cc

            str w2,[x19,#592]
            b .L_ffffff8008a5e0e0
.L_ffffff8008a5e0cc:

            adrp x0, .L_ffffff80092283f3
            adrp x1, .L_ffffff8009228422
            add x0,x0, :lo12:.L_ffffff80092283f3
            add x1,x1, :lo12:.L_ffffff8009228422
            bl printk
.L_ffffff8008a5e0e0:

            adrp x1, .L_ffffff8009204401
            add x1,x1, :lo12:.L_ffffff8009204401
            mov x0,x20
            mov w2,wzr
            mov x3,xzr
            bl of_get_named_gpio_flags

            mov w2,w0
            tbnz w0,#31,.L_ffffff8008a5e13c

            str w2,[x19,#588]
            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            cbz w8,.L_ffffff8008a5e12c
.L_ffffff8008a5e10c:

            ldr w3,[x19,#592]
            ldr w4,[x19,#588]
            adrp x0, .L_ffffff80092284ab
            adrp x1, .L_ffffff80092283e6
            add x0,x0, :lo12:.L_ffffff80092284ab
            add x1,x1, :lo12:.L_ffffff80092283e6
            mov w2,#629
            bl printk
.L_ffffff8008a5e12c:

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            ldr x21,[sp],#48
            ret
.L_ffffff8008a5e13c:

            adrp x0, .L_ffffff800922847c
            adrp x1, .L_ffffff8009228422
            add x0,x0, :lo12:.L_ffffff800922847c
            add x1,x1, :lo12:.L_ffffff8009228422
            bl printk

            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            cbnz w8,.L_ffffff8008a5e10c

            b .L_ffffff8008a5e12c
.size cf_gpio_init, . - cf_gpio_init
#-----------------------------------
.type hw_reset, @function
#-----------------------------------
hw_reset:

            str x21,[sp,#-48]!
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            mov x19,x0
            mov w0,#25
            mov w1,#1
            bl agold_gpio_set

            mov w0,#100
            mov w1,#200
            bl usleep_range

            mov w0,#25
            mov w1,wzr
            bl agold_gpio_set

            mov w0,#1000
            mov w1,#1100
            bl usleep_range

            mov w0,#25
            mov w1,#1
            bl agold_gpio_set

            mov w0,#100
            mov w1,#200
            bl usleep_range

            ldr w0,[x19,#588]
            bl gpio_to_desc

            bl gpiod_get_raw_value

            ldr w8,[x19,#592]
            mov w20,w0
            mov w0,w8
            bl gpio_to_desc

            bl gpiod_get_raw_value

            adrp x21, .L_ffffff8009965228
            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            cbz w8,.L_ffffff8008a5e228

            mov w19,w0
            adrp x0, .L_ffffff8009228696
            adrp x1, .L_ffffff8009151a5d
            add x0,x0, :lo12:.L_ffffff8009228696
            add x1,x1, :lo12:.L_ffffff8009151a5d
            mov w2,#406
            mov w3,w20
            bl printk

            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            cbz w8,.L_ffffff8008a5e228

            adrp x0, .L_ffffff80092286c3
            adrp x1, .L_ffffff8009151a5d
            add x0,x0, :lo12:.L_ffffff80092286c3
            add x1,x1, :lo12:.L_ffffff8009151a5d
            mov w2,#407
            mov w3,w19
            bl printk
.L_ffffff8008a5e228:

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            ldr x21,[sp],#48
            ret
.size hw_reset, . - hw_reset
#-----------------------------------
.type cf_device_event, @function
#-----------------------------------
cf_device_event:

            stp x20,x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            adrp x20, .L_ffffff8009965468
            ldr x1,[x20,:lo12:.L_ffffff8009965468]
            mov x19,x0
            adrp x0, .L_ffffff8009228c12
            add x0,x0, :lo12:.L_ffffff8009228c12
            bl printk

            ldr x8,[x19,#32]
            ldr x2,[x20,:lo12:.L_ffffff8009965468]
            mov x1,xzr
            add x0,x8,#32
            bl sysfs_notify

            adrp x0, .L_ffffff8009228c40
            add x0,x0, :lo12:.L_ffffff8009228c40
            bl printk

            ldp x29,lr,[sp,#16]
            ldp x20,x19,[sp],#32
            ret
.size cf_device_event, . - cf_device_event
#-----------------------------------
.type cf_irq_init, @function
#-----------------------------------
cf_irq_init:

            sub sp,sp,#64
            stp x22,x21,[sp,#16]
            stp x20,x19,[sp,#32]
            stp x29,lr,[sp,#48]
            add x29,sp,#48
            adrp x8, __stack_chk_guard
            ldr x8,[x8,:lo12:__stack_chk_guard]
            adrp x2, .L_ffffff80092280d1
            mov x19,x0
            add x2,x2, :lo12:.L_ffffff80092280d1
            str x8,[sp,#8]
            str xzr,[sp]
            ldr x20,[x0]
            mov x0,xzr
            mov x1,xzr
            bl of_find_compatible_node

            cbz x0,.L_ffffff8008a5e378

            adrp x1, .L_ffffff80090f7c05
            add x1,x1, :lo12:.L_ffffff80090f7c05
            mov x2,sp
            mov w3,#2
            mov x4,xzr
            mov x21,x0
            bl of_property_read_variable_u32_array

            tbnz w0,#31,.L_ffffff8008a5e314

            adrp x0, .L_ffffff8009228d7c
            adrp x1, .L_ffffff8009228da8
            add x0,x0, :lo12:.L_ffffff8009228d7c
            add x1,x1, :lo12:.L_ffffff8009228da8
            mov w2,#800
            bl printk

            ldp w0,w22,[sp]
            bl gpio_to_desc

            mov w1,w22
            bl gpiod_set_debounce
.L_ffffff8008a5e314:

            mov x0,x21
            mov w1,wzr
            bl irq_of_parse_and_map

            str w0,[x19,#584]
            cbz w0,.L_ffffff8008a5e398

            adrp x3, cf_irq_handler
            adrp x5, .L_ffffff8009228dec
            mov w1,w0
            add x3,x3, :lo12:cf_irq_handler
            add x5,x5, :lo12:.L_ffffff8009228dec
            mov w4,#24577
            mov x0,x20
            mov x2,xzr
            mov x6,x19
            bl devm_request_threaded_irq

            ldr w3,[x19,#584]
            mov w20,w0
            cbz w0,.L_ffffff8008a5e3b8

            adrp x0, .L_ffffff8009228df3
            adrp x1, .L_ffffff8009228da8
            add x0,x0, :lo12:.L_ffffff8009228df3
            add x1,x1, :lo12:.L_ffffff8009228da8
            mov w2,#825
            bl printk

            b .L_ffffff8008a5e3d4
.L_ffffff8008a5e378:

            adrp x0, .L_ffffff80092280e6
            adrp x1, .L_ffffff8009228da8
            add x0,x0, :lo12:.L_ffffff80092280e6
            add x1,x1, :lo12:.L_ffffff8009228da8
            mov w2,#815
            bl printk

            mov w20,#-19
            b .L_ffffff8008a5e3d4
.L_ffffff8008a5e398:

            adrp x0, .L_ffffff8009228db4
            adrp x1, .L_ffffff8009228da8
            add x0,x0, :lo12:.L_ffffff8009228db4
            add x1,x1, :lo12:.L_ffffff8009228da8
            mov w2,#808
            bl printk

            mov w20,#-22
            b .L_ffffff8008a5e3d4
.L_ffffff8008a5e3b8:

            adrp x0, .L_ffffff8009228e26
            add x0,x0, :lo12:.L_ffffff8009228e26
            mov w1,w3
            bl printk

            ldr w0,[x19,#584]
            mov w1,#1
            bl irq_set_irq_wake
.L_ffffff8008a5e3d4:

            adrp x9, __stack_chk_guard
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:__stack_chk_guard]
            cmp x9,x8
            b.ne .L_ffffff8008a5e400

            mov w0,w20
            ldp x29,lr,[sp,#48]
            ldp x20,x19,[sp,#32]
            ldp x22,x21,[sp,#16]
            add sp,sp,#64
            ret
.L_ffffff8008a5e400:

            bl __stack_chk_fail
.size cf_irq_init, . - cf_irq_init
#-----------------------------------
.type FUN_ffffff8008a5e404, @function
#-----------------------------------
FUN_ffffff8008a5e404:
#-----------------------------------
.type fb_notifier_callback_disambig_0xffffff8008a5e404_0, @function
#-----------------------------------
fb_notifier_callback_disambig_0xffffff8008a5e404_0:

            str x21,[sp,#-48]!
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            mov x20,x0
            adrp x0, .L_ffffff8009228e64
            add x0,x0, :lo12:.L_ffffff8009228e64
            mov x19,x2
            mov x21,x1
            bl printk

            cbz x19,.L_ffffff8008a5e568

            cmp x21,#9
            b.ne .L_ffffff8008a5e568

            sub x20,x20,#376
            cbz x20,.L_ffffff8008a5e568

            ldr x8,[x19,#8]
            cbz x8,.L_ffffff8008a5e568

            ldr w8,[x8]
            cmp w8,#4
            b.eq .L_ffffff8008a5e4b4

            cbnz w8,.L_ffffff8008a5e568

            adrp x19, .L_ffffff8009965228
            ldrb w8,[x19,:lo12:.L_ffffff8009965228]
            cbz w8,.L_ffffff8008a5e50c

            adrp x0, .L_ffffff8009228eb1
            adrp x1, .L_ffffff800922200c
            add x0,x0, :lo12:.L_ffffff8009228eb1
            add x1,x1, :lo12:.L_ffffff800922200c
            mov w2,#1427
            bl printk

            ldrb w8,[x19,:lo12:.L_ffffff8009965228]
            mov w9,#1
            strb w9,[x20,#664]
            str wzr,[x20,#668]
            cmp w8,#2
            b.lo .L_ffffff8008a5e4ac

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228ef4
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228ef4
            mov w2,#559
            bl printk
.L_ffffff8008a5e4ac:

            dmb ishld
            b .L_ffffff8008a5e530
.L_ffffff8008a5e4b4:

            adrp x19, .L_ffffff8009965228
            ldrb w8,[x19,:lo12:.L_ffffff8009965228]
            cbz w8,.L_ffffff8008a5e520

            adrp x0, .L_ffffff8009228ed2
            adrp x1, .L_ffffff800922200c
            add x0,x0, :lo12:.L_ffffff8009228ed2
            add x1,x1, :lo12:.L_ffffff800922200c
            mov w2,#1435
            bl printk

            ldrb w8,[x19,:lo12:.L_ffffff8009965228]
            mov w9,#1
            strb w9,[x20,#664]
            str w9,[x20,#668]
            cmp w8,#2
            b.lo .L_ffffff8008a5e52c

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228ef4
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228ef4
            mov w2,#559
            bl printk

            b .L_ffffff8008a5e52c
.L_ffffff8008a5e50c:

            mov w8,#1
            strb w8,[x20,#664]
            str wzr,[x20,#668]
            dmb ishld
            b .L_ffffff8008a5e530
.L_ffffff8008a5e520:

            mov w8,#1
            strb w8,[x20,#664]
            str w8,[x20,#668]
.L_ffffff8008a5e52c:

            dmb ishld
.L_ffffff8008a5e530:

            adrp x0, .L_ffffff8009228f01
            add x0,x0, :lo12:.L_ffffff8009228f01
            bl printk

            ldr w8,[x20,#648]
            cbz w8,.L_ffffff8008a5e554

            add x0,x20,#400
            mov w1,#250
            mov w2,wzr
            bl pm_wakeup_ws_event
.L_ffffff8008a5e554:

            adrp x8, system_wq
            ldr x1,[x8,:lo12:system_wq]
            add x2,x20,#248
            mov w0,#8
            bl queue_work_on
.L_ffffff8008a5e568:

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            mov w0,wzr
            ldr x21,[sp],#48
            ret
.size fb_notifier_callback_disambig_0xffffff8008a5e404_0, . - fb_notifier_callback_disambig_0xffffff8008a5e404_0
.size FUN_ffffff8008a5e404, . - FUN_ffffff8008a5e404
#-----------------------------------
.type cf_write, @function
#-----------------------------------
cf_write:

            stp x22,x21,[sp,#-48]!
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x21, .L_ffffff8009965228
            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            mov x19,x2
            cbz w8,.L_ffffff8008a5e640

            ldr x22,[x0,#200]
            ldr w20,[x22,#588]
            mov w0,w20
            bl gpio_to_desc

            bl gpiod_to_irq

            ldr w5,[x22,#584]
            mov w4,w0
            adrp x0, .L_ffffff80092287ce
            adrp x1, .L_ffffff8009228812
            add x0,x0, :lo12:.L_ffffff80092287ce
            add x1,x1, :lo12:.L_ffffff8009228812
            mov w2,#1017
            mov w3,w20
            bl printk

            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            cbz w8,.L_ffffff8008a5e640

            ldr w0,[x22,#592]
            bl gpio_to_desc

            bl gpiod_get_raw_value

            ldr w8,[x22,#588]
            mov w20,w0
            mov w0,w8
            bl gpio_to_desc

            bl gpiod_get_raw_value

            mov w4,w0
            adrp x0, .L_ffffff800922881b
            adrp x1, .L_ffffff8009228812
            add x0,x0, :lo12:.L_ffffff800922881b
            add x1,x1, :lo12:.L_ffffff8009228812
            mov w2,#1018
            mov w3,w20
            bl printk

            ldrb w8,[x21,:lo12:.L_ffffff8009965228]
            cmp w8,#2
            b.lo .L_ffffff8008a5e640

            adrp x0, .L_ffffff800922838f
            adrp x1, .L_ffffff8009228812
            add x0,x0, :lo12:.L_ffffff800922838f
            add x1,x1, :lo12:.L_ffffff8009228812
            mov w2,#1019
            bl printk
.L_ffffff8008a5e640:

            mov x0,x19
            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            ldp x22,x21,[sp],#48
            ret
.size cf_write, . - cf_write
#-----------------------------------
.type cf_ioctl, @function
#-----------------------------------
cf_ioctl:

            sub sp,sp,#80
            str x23,[sp,#16]
            stp x22,x21,[sp,#32]
            stp x20,x19,[sp,#48]
            stp x29,lr,[sp,#64]
            add x29,sp,#64
            adrp x8, __stack_chk_guard
            ldr x8,[x8,:lo12:__stack_chk_guard]
            adrp x23, .L_ffffff8009965228
            mov x21,x2
            mov w22,w1
            str x8,[sp,#8]
            ldrb w8,[x23,:lo12:.L_ffffff8009965228]
            ldr x20,[x0,#200]
            cmp w8,#2
            b.lo .L_ffffff8008a5e6ac

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1040
            bl printk
.L_ffffff8008a5e6ac:

            adrp x0, .L_ffffff8009228867
            add x0,x0, :lo12:.L_ffffff8009228867
            bl printk

            and w8,w22,#65280
            mov w9,#27392
            cmp w8,w9
            b.ne .L_ffffff8008a5e7d4

            tbz w22,#31,.L_ffffff8008a5e6f4

            mrs x8,sp_el0
            ldr x8,[x8,#8]
            mov x10,x21
            ubfx w9,w22,#16,#14
            adds x10,x10,x9
            csel x8,xzr,x8,hi
            csinv x10,x10,xzr,lo
            sbcs xzr,x10,x8
            cset x10,ls
            cbz x10,.L_ffffff8008a5e7dc
.L_ffffff8008a5e6f4:

            tbz w22,#30,.L_ffffff8008a5e720

            mrs x8,sp_el0
            ldr x8,[x8,#8]
            mov x10,x21
            ubfx w9,w22,#16,#14
            adds x10,x10,x9
            csel x8,xzr,x8,hi
            csinv x10,x10,xzr,lo
            sbcs xzr,x10,x8
            cset x10,ls
            cbz x10,.L_ffffff8008a5e7dc
.L_ffffff8008a5e720:

            add x19,x20,#608
            mov x0,x19
            bl mutex_lock

            mov w8,#27402
            cmp w22,w8
            b.le .L_ffffff8008a5e7e4

            mov w8,#-27403
            add w8,w22,w8
            cmp w8,#85
            b.hi .L_ffffff8008a5e864

            adrp x9, .L_ffffff8008f5eda8
            add x9,x9, :lo12:.L_ffffff8008f5eda8
            adr x10, .L_ffffff8008a5e764
            ldrh w11,[x9,x8,lsl #1]
            add x10,x10,x11, lsl #2
            mov w21,wzr
            br x10
.L_ffffff8008a5e764:

            ldrb w8,[x23,:lo12:.L_ffffff8009965228]
            cbz w8,.L_ffffff8008a5e7a8

            adrp x0, .L_ffffff8009228960
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff8009228960
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1087
            bl printk

            ldrb w8,[x23,:lo12:.L_ffffff8009965228]
            cmp w8,#2
            b.lo .L_ffffff8008a5e7a8

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228aef
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228aef
            mov w2,#477
            bl printk
.L_ffffff8008a5e7a8:

            adrp x0, .L_ffffff8009228afd
            add x0,x0, :lo12:.L_ffffff8009228afd
            bl printk

            ldrb w8,[x20,#652]
            cbnz w8,.L_ffffff8008a5ed44

            ldr w0,[x20,#584]
            bl enable_irq

            mov w8,#1
            mov w21,wzr
            strb w8,[x20,#652]
            b .L_ffffff8008a5ee08
.L_ffffff8008a5e7d4:

            mov x0,#-25
.L_ffffff8008a5e7d8:

            b .L_ffffff8008a5ee14
.L_ffffff8008a5e7dc:

            mov x0,#-14
            b .L_ffffff8008a5ee14
.L_ffffff8008a5e7e4:

            mov w8,#27424
            movk w8,#32776,lsl #16
            cmp w22,w8
            b.eq .L_ffffff8008a5eaec

            mov w8,#27504
            movk w8,#32776,lsl #16
            cmp w22,w8
            b.ne .L_ffffff8008a5e92c

            str wzr,[sp]
            mrs x8,sp_el0
            ldr x9,[x8,#8]
            mov x10,x21
            adds x10,x10,#4
            csel x9,xzr,x9,hi
            csinv x10,x10,xzr,lo
            sbcs xzr,x10,x9
            cset x10,ls
            cbz x10,.L_ffffff8008a5e84c

            ldr x8,[x8,#8]
            bics xzr,x21,x8
            csel x0,x21,xzr,eq
            csdb
            mov x1,sp
            mov w2,#4
            bl __arch_copy_to_user

            cbz x0,.L_ffffff8008a5ed44
.L_ffffff8008a5e84c:

            adrp x0, .L_ffffff8009228a77
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff8009228a77
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1260
            b .L_ffffff8008a5ee00
.L_ffffff8008a5e864:

            mov w8,#27400
            movk w8,#16385,lsl #16
            cmp w22,w8
            b.eq .L_ffffff8008a5eb68

            mov w8,#27399
            movk w8,#16392,lsl #16
            cmp w22,w8
            b.ne .L_ffffff8008a5e92c

            mrs x8,sp_el0
            ldr x9,[x8,#8]
            mov x10,x21
            adds x10,x10,#8
            csel x9,xzr,x9,hi
            csinv x10,x10,xzr,lo
            sbcs xzr,x10,x9
            cset x10,ls
            cbz x10,.L_ffffff8008a5ed4c

            ldr x8,[x8,#8]
            bics xzr,x21,x8
            csel x1,x21,xzr,eq
            csdb
            mov x0,sp
            mov w2,#8
            bl __arch_copy_from_user

            cbnz x0,.L_ffffff8008a5ed54

            ldrb w8,[x23,:lo12:.L_ffffff8009965228]
            ldr x20,[x20,#232]
            cmp w8,#2
            b.lo .L_ffffff8008a5e8f0

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228b93
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228b93
            mov w2,#916
            bl printk
.L_ffffff8008a5e8f0:

            ldr w8,[sp]
            sub w8,w8,#1
            cmp w8,#12
            b.hi .L_ffffff8008a5ec30

            adrp x9, .L_ffffff8008f5ee54
            add x9,x9, :lo12:.L_ffffff8008f5ee54
            adr x10, .L_ffffff8008a5e918
            ldrb w11,[x9,x8]
            add x10,x10,x11, lsl #2
            br x10
.L_ffffff8008a5e918:

            adrp x0, .L_ffffff80090f9cc8
            add x0,x0, :lo12:.L_ffffff80090f9cc8
            bl agold_strtol

            mov x21,x0
            b .L_ffffff8008a5ec8c
.L_ffffff8008a5e92c:

            mov w21,#-22
            b .L_ffffff8008a5ee08

            ldrb w8,[x23,#552]
            cbz w8,.L_ffffff8008a5e978

            adrp x0, .L_ffffff800922898c
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff800922898c
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1092
            bl printk

            ldrb w8,[x23,#552]
            cmp w8,#2
            b.lo .L_ffffff8008a5e978

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228b40
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228b40
            mov w2,#465
            bl printk
.L_ffffff8008a5e978:

            adrp x0, .L_ffffff8009228b4f
            add x0,x0, :lo12:.L_ffffff8009228b4f
            bl printk

            ldrb w8,[x20,#652]
            cbz w8,.L_ffffff8008a5ed44

            ldr w0,[x20,#584]
            bl disable_irq_nosync

            mov w21,wzr
            strb wzr,[x20,#652]
            b .L_ffffff8008a5ee08

            ldrb w8,[x23,#552]
            cbz w8,.L_ffffff8008a5e9c0

            adrp x0, .L_ffffff80092288a8
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff80092288a8
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1055
            bl printk
.L_ffffff8008a5e9c0:

            mov x0,x20
            bl cf_gpio_init

            mov w21,wzr
            b .L_ffffff8008a5ee08

            ldrb w8,[x23,#552]
            cbz w8,.L_ffffff8008a5e9f0

            adrp x0, .L_ffffff80092289b9
            adrp x1, .L_ffffff800922885e
.L_ffffff8008a5e9e0:

            add x0,x0, :lo12:.L_ffffff80092289b9
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1097
            bl printk
.L_ffffff8008a5e9f0:

            mov x0,x20
            bl cf_irq_init

            mov w21,w0
            b .L_ffffff8008a5ee08

            ldrb w8,[x23,#552]
            cbz w8,.L_ffffff8008a5ea20

            adrp x0, .L_ffffff80092288d3
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff80092288d3
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1060
            bl printk
.L_ffffff8008a5ea20:

            ldr w0,[x20,#588]
            cmp w0,#511
            b.hi .L_ffffff8008a5ea30

            bl gpio_free
.L_ffffff8008a5ea30:

            ldr w0,[x20,#584]
            cbz w0,.L_ffffff8008a5ea44

            mov x1,x20
            bl free_irq

            str wzr,[x20,#584]
.L_ffffff8008a5ea44:

            ldr w0,[x20,#592]
            cmp w0,#511
            b.hi .L_ffffff8008a5ea54

            bl gpio_free
.L_ffffff8008a5ea54:

            ldr w0,[x20,#596]
            cmp w0,#511
            b.hi .L_ffffff8008a5ea64

            bl gpio_free
.L_ffffff8008a5ea64:

            ldr x0,[x20,#144]
            cbz x0,.L_ffffff8008a5ed44

            bl devm_pinctrl_put

            mov w21,wzr
            b .L_ffffff8008a5ee08

            adrp x0, .L_ffffff80092286f0
            adrp x1, .L_ffffff800922871d
            add x0,x0, :lo12:.L_ffffff80092286f0
            add x1,x1, :lo12:.L_ffffff800922871d
            mov w2,#515
            bl printk

            ldr x0,[x20,#8]
            bl mt_spi_enable_master_clk

            ldrb w8,[x23,#552]
            cbz w8,.L_ffffff8008a5ed44

            adrp x0, .L_ffffff80092289e2
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff80092289e2
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1104
            b .L_ffffff8008a5ed40

            adrp x0, .L_ffffff8009228783
            add x0,x0, :lo12:.L_ffffff8009228783
            bl printk

            ldr x0,[x20,#8]
            bl mt_spi_disable_master_clk

            ldrb w8,[x23,#552]
            cbz w8,.L_ffffff8008a5ed44

            adrp x0, .L_ffffff8009228a12
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff8009228a12
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1111
            b .L_ffffff8008a5ed40
.L_ffffff8008a5eaec:

            adrp x0, .L_ffffff800a0fb392
            adrp x1, .L_ffffff8009228ba7
            add x0,x0, :lo12:.L_ffffff800a0fb392
            add x1,x1, :lo12:.L_ffffff8009228ba7
            mov w2,#32
            bl strncpy

            strb wzr,[x0,#31]
            mrs x8,sp_el0
            ldr x9,[x8,#8]
            mov x10,x21
            adds x10,x10,#32
            csel x9,xzr,x9,hi
            csinv x10,x10,xzr,lo
            sbcs xzr,x10,x9
            cset x10,ls
            cbz x10,.L_ffffff8008a5eb50

            ldr x8,[x8,#8]
            bics xzr,x21,x8
            csel x0,x21,xzr,eq
            csdb
            adrp x1, .L_ffffff800a0fb392
            add x1,x1, :lo12:.L_ffffff800a0fb392
            mov w2,#32
            bl __arch_copy_to_user

            cbz x0,.L_ffffff8008a5ed44
.L_ffffff8008a5eb50:

            adrp x0, .L_ffffff8009228a77
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff8009228a77
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1141
            b .L_ffffff8008a5ee00
.L_ffffff8008a5eb68:

            ldrb w8,[x23,:lo12:.L_ffffff8009965228]
            cbz w8,.L_ffffff8008a5eb88

            adrp x0, .L_ffffff8009228900
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff8009228900
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1065
            bl printk
.L_ffffff8008a5eb88:

            adrp x8, .L_ffffff800a0fb38c
            ldr w9,[x8,:lo12:.L_ffffff800a0fb38c]
            cmp w9,#1
            b.ne .L_ffffff8008a5ebd4

            str wzr,[x8,:lo12:.L_ffffff800a0fb38c]
            ldr x22,[x20,#232]
            adrp x0, .L_ffffff80090f9cc8
            add x0,x0, :lo12:.L_ffffff80090f9cc8
            bl agold_strtol

            mov x2,x0
            mov w1,#1
            mov x0,x22
            mov w3,wzr
            bl input_event

            ldr x0,[x20,#232]
            mov w1,wzr
            mov w2,wzr
            mov w3,wzr
            bl input_event
.L_ffffff8008a5ebd4:

            mrs x10,sp_el0
            ldr x8,[x10,#8]
            mov x9,x21
            adds x9,x9,#1
            csel x8,xzr,x8,hi
            csinv x9,x9,xzr,lo
            sbcs xzr,x9,x8
            cset x9,ls
            cbz x9,.L_ffffff8008a5edec

            ldr x9,[x10,#8]
            bics xzr,x21,x9
            csel x8,x21,xzr,eq
            csdb
            adrp x9, .L_ffffff8009a45544
            ldr w11,[x9,:lo12:.L_ffffff8009a45544]
            cmp w11,#1
            b.lt .L_ffffff8008a5ed88

            adrp x11, .L_ffffff8009a454e0
            ldr w11,[x11,:lo12:.L_ffffff8009a454e0]
            cmp w11,#0
            b.le .L_ffffff8008a5ed94
.L_ffffff8008a5ec28:

            nop
            nop
            nop
            nop
            b .L_ffffff8008a5edc0
.L_ffffff8008a5ec30:

            mov w21,#240
            b .L_ffffff8008a5ec8c

            mov w21,#102
            b .L_ffffff8008a5ec8c

            mov w21,#139
            b .L_ffffff8008a5ec8c

            mov w21,#158
            b .L_ffffff8008a5ec8c

            mov w21,#190
            b .L_ffffff8008a5ec8c

            mov w21,#191
            b .L_ffffff8008a5ec8c

            mov w21,#28
            b .L_ffffff8008a5ec8c

            mov w21,#103
            b .L_ffffff8008a5ec8c

            mov w21,#105
            b .L_ffffff8008a5ec8c

            mov w21,#106
            b .L_ffffff8008a5ec8c

            mov w21,#108
            b .L_ffffff8008a5ec8c

            mov w21,#143
.L_ffffff8008a5ec8c:

            ldr w8,[sp,#4]
            cmp w8,#2
            b.eq .L_ffffff8008a5ecac

            cmp w8,#1
            b.eq .L_ffffff8008a5ece8

            cbnz w8,.L_ffffff8008a5ecf4

            mov w9,#2
            b .L_ffffff8008a5ecec
.L_ffffff8008a5ecac:

            mov w1,#1
            mov w3,#1
            mov x0,x20
            mov w2,w21
            bl input_event

            mov x0,x20
            mov w1,wzr
            mov w2,wzr
            mov w3,wzr
            bl input_event

            mov w1,#1
            mov x0,x20
            mov w2,w21
            mov w3,wzr
            b .L_ffffff8008a5ed08
.L_ffffff8008a5ece8:

            mov w9,w8
.L_ffffff8008a5ecec:

            adrp x10, .L_ffffff800a0fb38c
            str w9,[x10,:lo12:.L_ffffff800a0fb38c]
.L_ffffff8008a5ecf4:

            cmp w8,#0
            cset w3,ne
            mov w1,#1
            mov x0,x20
            mov w2,w21
.L_ffffff8008a5ed08:

            bl input_event

            mov x0,x20
            mov w1,wzr
            mov w2,wzr
            mov w3,wzr
            bl input_event

            ldrb w8,[x23,:lo12:.L_ffffff8009965228]
            cmp w8,#2
            b.lo .L_ffffff8008a5ed44

            adrp x0, .L_ffffff800922838f
            adrp x1, .L_ffffff8009228b93
            add x0,x0, :lo12:.L_ffffff800922838f
            add x1,x1, :lo12:.L_ffffff8009228b93
            mov w2,#959
.L_ffffff8008a5ed40:

            bl printk
.L_ffffff8008a5ed44:

            mov w21,wzr
            b .L_ffffff8008a5ee08
.L_ffffff8008a5ed4c:

            mov w2,#8
            b .L_ffffff8008a5ed58
.L_ffffff8008a5ed54:

            mov x2,x0
.L_ffffff8008a5ed58:

            mov w8,#8
            sub x8,x8,x2
            mov x9,sp
            add x0,x9,x8
            mov w1,wzr
            bl __pi_memset

            adrp x0, .L_ffffff8009228a43
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff8009228a43
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1131
            b .L_ffffff8008a5ee00
.L_ffffff8008a5ed88:

            adrp x11, .L_ffffff8009a454c8
            ldr x11,[x11,:lo12:.L_ffffff8009a454c8]
            tbnz w11,#4,.L_ffffff8008a5ec28
.L_ffffff8008a5ed94:

            mrs x11,daif
            msr daifset,#2
            ldr x10,[x10,#16]
            mrs x12,ttbr1_el1
            mov x13,x10
            bfxil x13,x12,#0,#48
            msr ttbr1_el1,x13
            isb
            msr ttbr0_el1,x10
            isb
            msr daif,x11
.L_ffffff8008a5edc0:

            mov w10,wzr
            ldrb w21,[x8]
.L_ffffff8008a5edc8:

            ldr w8,[x9,:lo12:.L_ffffff8009a45544]
            cmp w8,#1
            b.lt .L_ffffff8008a5ee40

            adrp x8, .L_ffffff8009a454e0
            ldr w8,[x8,:lo12:.L_ffffff8009a454e0]
            cmp w8,#0
            b.le .L_ffffff8008a5ee4c
.L_ffffff8008a5ede4:

            nop
            nop
            nop
            nop
            cbz w10,.L_ffffff8008a5ee78
.L_ffffff8008a5edec:

            adrp x0, .L_ffffff800922892e
            adrp x1, .L_ffffff800922885e
            add x0,x0, :lo12:.L_ffffff800922892e
            add x1,x1, :lo12:.L_ffffff800922885e
            mov w2,#1078
.L_ffffff8008a5ee00:

            bl printk

            mov w21,#-14
.L_ffffff8008a5ee08:

            mov x0,x19
            bl mutex_unlock

            sxtw x0,w21
.L_ffffff8008a5ee14:

            adrp x9, __stack_chk_guard
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:__stack_chk_guard]
            cmp x9,x8
            b.ne .L_ffffff8008a5eee0

            ldp x29,lr,[sp,#64]
            ldp x20,x19,[sp,#48]
            ldp x22,x21,[sp,#32]
            ldr x23,[sp,#16]
            add sp,sp,#80
            ret
.L_ffffff8008a5ee40:

            adrp x8, .L_ffffff8009a454c8
            ldr x8,[x8,:lo12:.L_ffffff8009a454c8]
            tbnz w8,#4,.L_ffffff8008a5ede4
.L_ffffff8008a5ee4c:

            mrs x8,daif
            msr daifset,#2
            mrs x9,ttbr1_el1
            and x9,x9,#281474976710655
            add x11,x9,#2,lsl #12
            msr ttbr0_el1,x11
            isb
            msr ttbr1_el1,x9
            isb
            msr daif,x8
            cbnz w10,.L_ffffff8008a5edec
.L_ffffff8008a5ee78:

            ldrb w8,[x23,:lo12:.L_ffffff8009965228]
            cmp w8,#2
            b.lo .L_ffffff8008a5ee9c

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228aa9
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228aa9
            mov w2,#885
            bl printk
.L_ffffff8008a5ee9c:

            ldr w8,[x20,#592]
            cbnz w8,.L_ffffff8008a5eebc

            adrp x0, .L_ffffff8009228ac1
            adrp x1, .L_ffffff8009228aa9
            add x0,x0, :lo12:.L_ffffff8009228ac1
            add x1,x1, :lo12:.L_ffffff8009228aa9
            mov w2,#889
            bl printk
.L_ffffff8008a5eebc:

            mov w0,#25
            tst w21,#255
            b.eq .L_ffffff8008a5eed0

            mov w1,#1
            b .L_ffffff8008a5eed4
.L_ffffff8008a5eed0:

            mov w1,wzr
.L_ffffff8008a5eed4:

            bl agold_gpio_set

            mov w21,wzr
            b .L_ffffff8008a5ee08
.L_ffffff8008a5eee0:

            bl __stack_chk_fail
.size cf_ioctl, . - cf_ioctl
#-----------------------------------
.type cf_open, @function
#-----------------------------------
cf_open:

            stp x20,x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            adrp x8, .L_ffffff8009965228
            ldrb w8,[x8,:lo12:.L_ffffff8009965228]
            mov x19,x1
            mov x20,x0
            cmp w8,#2
            b.lo .L_ffffff8008a5ef20

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228bbe
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228bbe
            mov w2,#976
            bl printk
.L_ffffff8008a5ef20:

            ldr x8,[x20,#544]
            mov w0,wzr
            sub x8,x8,#16
            str x8,[x19,#200]
            ldp x29,lr,[sp,#16]
            ldp x20,x19,[sp],#32
            ret
.size cf_open, . - cf_open
#-----------------------------------
.type cf_release, @function
#-----------------------------------
cf_release:

            mov w0,wzr
            ret
.size cf_release, . - cf_release
#-----------------------------------
.type cf_fasync, @function
#-----------------------------------
cf_fasync:

            str x21,[sp,#-48]!
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x8, .L_ffffff8009965228
            ldrb w8,[x8,:lo12:.L_ffffff8009965228]
            mov w19,w2
            mov x20,x1
            mov w21,w0
            cmp w8,#2
            b.lo .L_ffffff8008a5ef88

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228bc6
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228bc6
            mov w2,#1281
            bl printk
.L_ffffff8008a5ef88:

            adrp x0, .L_ffffff8009228bd0
            add x0,x0, :lo12:.L_ffffff8009228bd0
            bl printk

            ldr x8,[x20,#200]
            mov w0,w21
            mov x1,x20
            mov w2,w19
            add x3,x8,#240
            bl fasync_helper

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            ldr x21,[sp],#48
            ret
.size cf_fasync, . - cf_fasync
#-----------------------------------
.type irq_get, @function
#-----------------------------------
irq_get:

            str x21,[sp,#-48]!
            stp x20,x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            ldr x21,[x0,#152]
            mov x19,x2
            ldr w0,[x21,#588]
            bl gpio_to_desc

            bl gpiod_get_raw_value

            mov w20,w0
            adrp x0, .L_ffffff8009228c6b
            add x0,x0, :lo12:.L_ffffff8009228c6b
            bl printk

            ldrb w8,[x21,#664]
            cbz w20,.L_ffffff8008a5f000

            cbnz w8,.L_ffffff8008a5f020

            b .L_ffffff8008a5f024
.L_ffffff8008a5f000:

            cbz w8,.L_ffffff8008a5f050

            ldr w8,[x21,#668]
            mov w9,#3
            cmp w8,#1
            csel w9,w9,wzr,eq
            cmp w8,#0
            mov w8,#2
            csel w20,w8,w9,eq
.L_ffffff8008a5f020:

            strb wzr,[x21,#664]
.L_ffffff8008a5f024:

            adrp x2, .L_ffffff80090df631
            add x2,x2, :lo12:.L_ffffff80090df631
            mov w1,#4096
            mov x0,x19
            mov w3,w20
            bl scnprintf

            ldp x29,lr,[sp,#32]
            ldp x20,x19,[sp,#16]
            sxtw x0,w0
            ldr x21,[sp],#48
            ret
.L_ffffff8008a5f050:

            mov w20,wzr
            b .L_ffffff8008a5f024
.size irq_get, . - irq_get
#-----------------------------------
.type irq_ack, @function
#-----------------------------------
irq_ack:

            str x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            adrp x8, .L_ffffff8009965228
            ldrb w8,[x8,:lo12:.L_ffffff8009965228]
            mov x19,x3
            cbz w8,.L_ffffff8008a5f08c

            adrp x0, .L_ffffff8009228cab
            adrp x1, .L_ffffff8009228cf0
            add x0,x0, :lo12:.L_ffffff8009228cab
            add x1,x1, :lo12:.L_ffffff8009228cf0
            mov w2,#449
            bl printk
.L_ffffff8008a5f08c:

            ldp x29,lr,[sp,#16]
            mov x0,x19
            ldr x19,[sp],#32
            ret
.size irq_ack, . - irq_ack
#-----------------------------------
.type cf_irq_handler, @function
#-----------------------------------
cf_irq_handler:

            str x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            adrp x8, .L_ffffff8009965228
            ldrb w8,[x8,:lo12:.L_ffffff8009965228]
            mov x19,x1
            cmp w8,#2
            b.lo .L_ffffff8008a5f0d4

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228e4d
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228e4d
            mov w2,#538
            bl printk
.L_ffffff8008a5f0d4:

            dmb ishld
            ldr w8,[x19,#648]
            cbz w8,.L_ffffff8008a5f0f0

            add x0,x19,#400
            mov w1,#250
            mov w2,wzr
            bl pm_wakeup_ws_event
.L_ffffff8008a5f0f0:

            adrp x0, .L_ffffff8009228e5c
            adrp x1, .L_ffffff8009228e4d
            add x0,x0, :lo12:.L_ffffff8009228e5c
            add x1,x1, :lo12:.L_ffffff8009228e4d
            bl printk

            adrp x8, system_wq
            ldr x1,[x8,:lo12:system_wq]
            add x2,x19,#248
            mov w0,#8
            bl queue_work_on

            ldp x29,lr,[sp,#16]
            mov w0,#1
            ldr x19,[sp],#32
            ret
.size cf_irq_handler, . - cf_irq_handler
#-----------------------------------
.type cf_suspend, @function
#-----------------------------------
cf_suspend:

            mov w0,wzr
            ret
.size cf_suspend, . - cf_suspend
#-----------------------------------
.type cf_resume, @function
#-----------------------------------
cf_resume:

            mov w0,wzr
            ret
.size cf_resume, . - cf_resume
.align 3
#-----------------------------------
.globl cf_sfr_read
.type cf_sfr_read, @function
#-----------------------------------
cf_sfr_read:

            sub sp,sp,#48
            str x19,[sp,#16]
            stp x29,lr,[sp,#32]
            add x29,sp,#32
            adrp x9, __stack_chk_guard
            ldr x9,[x9,:lo12:__stack_chk_guard]
            mov x8,x2
            mov w10,#221
            and w4,w3,#65535
            str x9,[sp,#8]
            strh w10,[sp,#4]
            strb w1,[sp,#5]
            add x1,sp,#4
            mov w2,#2
            mov x3,x8
            bl cf_spi_read_and_write

            mov w19,w0
            cbz w0,.L_ffffff8008a5f190

            adrp x0, .L_ffffff8009229089
            add x0,x0, :lo12:.L_ffffff8009229089
            mov w1,w19
            bl printk
.L_ffffff8008a5f190:

            adrp x9, __stack_chk_guard
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:__stack_chk_guard]
            cmp x9,x8
            b.ne .L_ffffff8008a5f1b8

            mov w0,w19
            ldp x29,lr,[sp,#32]
            ldr x19,[sp,#16]
            add sp,sp,#48
            ret
.L_ffffff8008a5f1b8:

            bl __stack_chk_fail
.size cf_sfr_read, . - cf_sfr_read
#-----------------------------------
.type cf_spi_read_and_write, @function
#-----------------------------------
cf_spi_read_and_write:

            sub sp,sp,#352
            stp x28,x27,[sp,#256]
            stp x26,x25,[sp,#272]
            stp x24,x23,[sp,#288]
            stp x22,x21,[sp,#304]
            stp x20,x19,[sp,#320]
            stp x29,lr,[sp,#336]
            add x29,sp,#336
            adrp x8, __stack_chk_guard
            ldr x8,[x8,:lo12:__stack_chk_guard]
            stur x8,[x29,#-96]
            stp xzr,xzr,[sp,#88]
            stp xzr,xzr,[sp,#72]
            stp xzr,xzr,[sp,#56]
            stp xzr,xzr,[sp,#40]
            stp xzr,xzr,[sp,#24]
            stp xzr,xzr,[sp,#8]
            cbz x0,.L_ffffff8008a5f364

            add w8,w4,w2
            mov w9,#1023
            tst w8,#1023
            add w10,w8,#1024
            ccmp w8,w9,#0,ne
            and w9,w10,#4294966272
            csel w25,w9,w8,hi
            mov w20,w4
            mov x19,x3
            mov w24,w2
            mov x23,x1
            mov x22,x0
            cmp w25,#32
            b.hi .L_ffffff8008a5f258

            adrp x0, .L_ffffff8009965498
            add x0,x0, :lo12:.L_ffffff8009965498
            bl mutex_trylock

            cbz w0,.L_ffffff8008a5f258

            adrp x21, .L_ffffff800a0fb3b8
            add x21,x21, :lo12:.L_ffffff800a0fb3b8
            b .L_ffffff8008a5f278
.L_ffffff8008a5f258:

            cmp w25,#32
            mov w8,#32
            mov w1,#192
            csel w0,w25,w8,hi
            movk w1,#320,lsl #16
            bl __kmalloc

            mov x21,x0
            cbz x0,.L_ffffff8008a5f3d8
.L_ffffff8008a5f278:

            add x26,sp,#104
            add x8,x26,#88
            mov w24,w24
            stp xzr,xzr,[sp,#104]
            stp xzr,xzr,[sp,#184]
            stp xzr,xzr,[sp,#168]
            stp xzr,xzr,[sp,#152]
            stp xzr,xzr,[sp,#136]
            stp xzr,xzr,[sp,#120]
            str x26,[sp,#104]
            str x8,[sp,#192]
            str x8,[sp,#200]
            add x8,sp,#8
            mov x0,x21
            mov x1,x23
            mov x2,x24
            add x27,x8,#80
            stp xzr,xzr,[sp,#88]
            stp xzr,xzr,[sp,#72]
            stp xzr,xzr,[sp,#56]
            stp xzr,xzr,[sp,#40]
            stp xzr,xzr,[sp,#24]
            stp xzr,xzr,[sp,#8]
            bl __pi_memcpy

            mov w8,#1
            mov w9,#50880
            adrp x23, .L_ffffff800a0fb3b4
            movk w9,#45,lsl #16
            sub x10,x29,#128
            strh w8,[sp,#82]
            adrp x8, cf_spi_complete
            add x23,x23, :lo12:.L_ffffff800a0fb3b4
            add x8,x8, :lo12:cf_spi_complete
            str w9,[sp,#84]
            add x9,x10,#16
            mov x0,x23
            strb wzr,[sp,#80]
            str w25,[sp,#24]
            stp x21,x21,[sp,#8]
            str x27,[sp,#112]
            stp x26,x26,[sp,#88]
            str x27,[sp,#104]
            stur wzr,[x29,#-120]
            stur wzr,[x29,#-128]
            stp x9,x9,[x29,#-112]
            stp x8,x10,[sp,#136]
            bl _raw_spin_lock_irq

            add x1,sp,#104
            mov x0,x22
            bl spi_async

            mov w22,w0
            mov x0,x23
            bl _raw_spin_unlock_irq

            cbz w22,.L_ffffff8008a5f378

            adrp x0, .L_ffffff8009229239
            add x0,x0, :lo12:.L_ffffff8009229239
            mov w1,w22
            bl printk

            b .L_ffffff8008a5f388
.L_ffffff8008a5f364:

            adrp x0, .L_ffffff80092291e8
            add x0,x0, :lo12:.L_ffffff80092291e8
            bl printk

            mov w22,#-22
            b .L_ffffff8008a5f3e8
.L_ffffff8008a5f378:

            sub x0,x29,#128
            bl wait_for_completion

            ldr w22,[sp,#160]
            cbz w22,.L_ffffff8008a5f398
.L_ffffff8008a5f388:

            adrp x0, .L_ffffff8009229209
            add x0,x0, :lo12:.L_ffffff8009229209
            mov w1,w22
            bl printk
.L_ffffff8008a5f398:

            add x1,x21,x24
            mov w2,w20
            mov x0,x19
            bl __pi_memcpy

            ldr x8,[sp,#8]
            adrp x9, .L_ffffff800a0fb3b8
            add x9,x9, :lo12:.L_ffffff800a0fb3b8
            cmp x8,x9
            b.eq .L_ffffff8008a5f3c8

            mov x0,x21
            bl kfree

            b .L_ffffff8008a5f3e8
.L_ffffff8008a5f3c8:

            adrp x0, .L_ffffff8009965498
            add x0,x0, :lo12:.L_ffffff8009965498
            bl mutex_unlock

            b .L_ffffff8008a5f3e8
.L_ffffff8008a5f3d8:

            adrp x0, .L_ffffff80092290b4
            add x0,x0, :lo12:.L_ffffff80092290b4
            bl printk

            mov w22,#-12
.L_ffffff8008a5f3e8:

            adrp x9, __stack_chk_guard
            ldur x8,[x29,#-96]
            ldr x9,[x9,:lo12:__stack_chk_guard]
            cmp x9,x8
            b.ne .L_ffffff8008a5f420

            mov w0,w22
            ldp x29,lr,[sp,#336]
            ldp x20,x19,[sp,#320]
            ldp x22,x21,[sp,#304]
            ldp x24,x23,[sp,#288]
            ldp x26,x25,[sp,#272]
            ldp x28,x27,[sp,#256]
            add sp,sp,#352
            ret
.L_ffffff8008a5f420:

            bl __stack_chk_fail
.size cf_spi_read_and_write, . - cf_spi_read_and_write
.align 2
#-----------------------------------
.globl cf_sfr_write
.type cf_sfr_write, @function
#-----------------------------------
cf_sfr_write:

            stp x24,x23,[sp,#-64]!
            stp x22,x21,[sp,#16]
            stp x20,x19,[sp,#32]
            stp x29,lr,[sp,#48]
            add x29,sp,#48
            and w8,w3,#65535
            mov w24,w1
            add w20,w8,#2
            mov w1,#192
            mov x21,x0
            movk w1,#320,lsl #16
            mov x0,x20
            mov w23,w3
            mov x22,x2
            bl __kmalloc

            cbz x0,.L_ffffff8008a5f4c0

            mov x19,x0
            mov w8,#204
            strb w24,[x0,#1]
            add x0,x0,#2
            and x2,x23,#65535
            mov x1,x22
            strb w8,[x19]
            bl __pi_memcpy

            mov x0,x21
            mov x1,x19
            mov w2,w20
            mov x3,xzr
            mov w4,wzr
            bl cf_spi_read_and_write

            mov w20,w0
            cbz w0,.L_ffffff8008a5f4b4

            adrp x0, .L_ffffff80092290d1
            add x0,x0, :lo12:.L_ffffff80092290d1
            mov w1,w20
            bl printk
.L_ffffff8008a5f4b4:

            mov x0,x19
            bl kfree

            b .L_ffffff8008a5f4d0
.L_ffffff8008a5f4c0:

            adrp x0, .L_ffffff80092290b4
            add x0,x0, :lo12:.L_ffffff80092290b4
            bl printk

            mov w20,#-12
.L_ffffff8008a5f4d0:

            mov w0,w20
            ldp x29,lr,[sp,#48]
            ldp x20,x19,[sp,#32]
            ldp x22,x21,[sp,#16]
            ldp x24,x23,[sp],#64
            ret
.size cf_sfr_write, . - cf_sfr_write
.align 3
#-----------------------------------
.globl cf_sram_read
.type cf_sram_read, @function
#-----------------------------------
cf_sram_read:

            sub sp,sp,#80
            stp x24,x23,[sp,#16]
            stp x22,x21,[sp,#32]
            stp x20,x19,[sp,#48]
            stp x29,lr,[sp,#64]
            add x29,sp,#64
            adrp x8, __stack_chk_guard
            ldr x8,[x8,:lo12:__stack_chk_guard]
            and w9,w3,#65535
            mov w23,w1
            add w24,w9,#1
            mov w1,#192
            mov x22,x0
            movk w1,#320,lsl #16
            mov x0,x24
            mov w21,w3
            mov x19,x2
            str x8,[sp,#8]
            strb wzr,[sp,#6]
            strh wzr,[sp,#4]
            bl __kmalloc

            cbz x0,.L_ffffff8008a5f58c

            mov x20,x0
            mov w8,#187
            lsr w9,w23,#8
            add x1,sp,#4
            mov w2,#3
            mov x0,x22
            mov x3,x20
            mov w4,w24
            strb w8,[sp,#4]
            strb w9,[sp,#5]
            strb w23,[sp,#6]
            bl cf_spi_read_and_write

            mov w22,w0
            cbz w0,.L_ffffff8008a5f5a0

            adrp x0, .L_ffffff8009229107
            add x0,x0, :lo12:.L_ffffff8009229107
            mov w1,w22
            bl printk

            b .L_ffffff8008a5f5b0
.L_ffffff8008a5f58c:

            adrp x0, .L_ffffff80092290b4
            add x0,x0, :lo12:.L_ffffff80092290b4
            bl printk

            mov w22,#-12
            b .L_ffffff8008a5f5b8
.L_ffffff8008a5f5a0:

            add x1,x20,#1
            and x2,x21,#65535
            mov x0,x19
            bl __pi_memcpy
.L_ffffff8008a5f5b0:

            mov x0,x20
            bl kfree
.L_ffffff8008a5f5b8:

            adrp x9, __stack_chk_guard
            ldr x8,[sp,#8]
            ldr x9,[x9,:lo12:__stack_chk_guard]
            cmp x9,x8
            b.ne .L_ffffff8008a5f5e8

            mov w0,w22
            ldp x29,lr,[sp,#64]
            ldp x20,x19,[sp,#48]
            ldp x22,x21,[sp,#32]
            ldp x24,x23,[sp,#16]
            add sp,sp,#80
            ret
.L_ffffff8008a5f5e8:

            bl __stack_chk_fail
.size cf_sram_read, . - cf_sram_read
.align 2
#-----------------------------------
.globl cf_sram_write
.type cf_sram_write, @function
#-----------------------------------
cf_sram_write:

            stp x24,x23,[sp,#-64]!
            stp x22,x21,[sp,#16]
            stp x20,x19,[sp,#32]
            stp x29,lr,[sp,#48]
            add x29,sp,#48
            and w8,w3,#65535
            mov w24,w1
            add w20,w8,#3
            mov w1,#192
            mov x21,x0
            movk w1,#320,lsl #16
            mov x0,x20
            mov w23,w3
            mov x22,x2
            bl __kmalloc

            cbz x0,.L_ffffff8008a5f690

            mov x19,x0
            mov w8,#170
            lsr w9,w24,#8
            strb w24,[x0,#2]
            add x0,x0,#3
            and x2,x23,#65535
            mov x1,x22
            strb w8,[x19]
            strb w9,[x19,#1]
            bl __pi_memcpy

            mov x0,x21
            mov x1,x19
            mov w2,w20
            mov x3,xzr
            mov w4,wzr
            bl cf_spi_read_and_write

            mov w20,w0
            cbz w0,.L_ffffff8008a5f684

            adrp x0, .L_ffffff8009229107
            add x0,x0, :lo12:.L_ffffff8009229107
            mov w1,w20
            bl printk
.L_ffffff8008a5f684:

            mov x0,x19
            bl kfree

            b .L_ffffff8008a5f6a0
.L_ffffff8008a5f690:

            adrp x0, .L_ffffff80092290b4
            add x0,x0, :lo12:.L_ffffff80092290b4
            bl printk

            mov w20,#-12
.L_ffffff8008a5f6a0:

            mov w0,w20
            ldp x29,lr,[sp,#48]
            ldp x20,x19,[sp,#32]
            ldp x22,x21,[sp,#16]
            ldp x24,x23,[sp],#64
            ret
.size cf_sram_write, . - cf_sram_write
.align 3
#-----------------------------------
.globl cf_spi_cmd
.type cf_spi_cmd, @function
#-----------------------------------
cf_spi_cmd:

            str x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            and w2,w2,#65535
            mov x3,xzr
            mov w4,wzr
            bl cf_spi_read_and_write

            mov w19,w0
            cbz w0,.L_ffffff8008a5f6ec

            adrp x0, .L_ffffff8009229107
            add x0,x0, :lo12:.L_ffffff8009229107
            mov w1,w19
            bl printk
.L_ffffff8008a5f6ec:

            ldp x29,lr,[sp,#16]
            mov w0,w19
            ldr x19,[sp],#32
            ret
.size cf_spi_cmd, . - cf_spi_cmd
.align 2
#-----------------------------------
.globl cf_write_configs
.type cf_write_configs, @function
#-----------------------------------
cf_write_configs:

            stp x28,x27,[sp,#-96]!
            stp x26,x25,[sp,#16]
            stp x24,x23,[sp,#32]
            stp x22,x21,[sp,#48]
            stp x20,x19,[sp,#64]
            stp x29,lr,[sp,#80]
            add x29,sp,#80
            cmp w2,#1
            b.lt .L_ffffff8008a5f840

            mov x19,x0
            mov x26,xzr
            sxtw x27,w2
            add x28,x1,#2
            adrp x20, .L_ffffff8009ad5688
            mov w21,#204
.L_ffffff8008a5f738:

            ldurb w8,[x28,#-2]
            ldrh w22,[x28]
            ldrh w23,[x28,#2]
            cmp w8,#170
            b.eq .L_ffffff8008a5f7b8

            cmp w8,#204
            b.ne .L_ffffff8008a5f830

            ldr x0,[x20,:lo12:.L_ffffff8009ad5688]
            mov w1,#192
            movk w1,#320,lsl #16
            mov w2,#3
            bl kmem_cache_alloc_trace

            cbz x0,.L_ffffff8008a5f848

            mov x25,x0
            strb w21,[x0]
            strb w22,[x0,#1]
            strb w23,[x0,#2]
            mov w2,#3
            mov x0,x19
            mov x1,x25
            mov x3,xzr
            mov w4,wzr
            bl cf_spi_read_and_write

            cbnz w0,.L_ffffff8008a5f85c

            mov x0,x25
            bl kfree

            adrp x0, .L_ffffff800922916a
            and w3,w23,#255
            mov w1,#204
            add x0,x0, :lo12:.L_ffffff800922916a
            mov w2,w22
            b .L_ffffff8008a5f82c
.L_ffffff8008a5f7b8:

            ldr x0,[x20,:lo12:.L_ffffff8009ad5688]
            mov w1,#192
            movk w1,#320,lsl #16
            mov w2,#5
            bl kmem_cache_alloc_trace

            cbz x0,.L_ffffff8008a5f848

            mov w9,#170
            mov x25,x0
            lsr w8,w23,#8
            strb w9,[x0]
            lsr w9,w22,#8
            strb w22,[x0,#2]
            strb w23,[x0,#3]
            strb w9,[x0,#1]
            strb w8,[x0,#4]
            mov w2,#5
            mov x0,x19
            mov x1,x25
            mov x3,xzr
            mov w4,wzr
            bl cf_spi_read_and_write

            cbnz w0,.L_ffffff8008a5f86c

            mov x0,x25
            bl kfree

            adrp x0, .L_ffffff80092291a9
            mov w1,#170
            add x0,x0, :lo12:.L_ffffff80092291a9
            mov w2,w22
            mov w3,w23
.L_ffffff8008a5f82c:

            bl printk
.L_ffffff8008a5f830:

            add x26,x26,#1
            cmp x26,x27
            add x28,x28,#6
            b.lt .L_ffffff8008a5f738
.L_ffffff8008a5f840:

            mov w24,wzr
            b .L_ffffff8008a5f898
.L_ffffff8008a5f848:

            adrp x0, .L_ffffff80092290b4
            add x0,x0, :lo12:.L_ffffff80092290b4
            bl printk

            mov w24,#-12
            b .L_ffffff8008a5f888
.L_ffffff8008a5f85c:

            mov w24,w0
            adrp x0, .L_ffffff80092290d1
            add x0,x0, :lo12:.L_ffffff80092290d1
            b .L_ffffff8008a5f878
.L_ffffff8008a5f86c:

            mov w24,w0
            adrp x0, .L_ffffff8009229107
            add x0,x0, :lo12:.L_ffffff8009229107
.L_ffffff8008a5f878:

            mov w1,w24
            bl printk

            mov x0,x25
            bl kfree
.L_ffffff8008a5f888:

            adrp x0, .L_ffffff800922913d
            add x0,x0, :lo12:.L_ffffff800922913d
            mov w1,w24
            bl printk
.L_ffffff8008a5f898:

            mov w0,w24
            ldp x29,lr,[sp,#80]
            ldp x20,x19,[sp,#64]
            ldp x22,x21,[sp,#48]
            ldp x24,x23,[sp,#32]
            ldp x26,x25,[sp,#16]
            ldp x28,x27,[sp],#96
            ret
.size cf_write_configs, . - cf_write_configs
#-----------------------------------
.type cf_spi_complete, @function
#-----------------------------------
cf_spi_complete:

            stp x29,lr,[sp,#-16]!
            mov x29,sp
            bl complete

            ldp x29,lr,[sp],#16
            ret
.size cf_spi_complete, . - cf_spi_complete
.align 2
#-----------------------------------
.type cf_driver_init, @function
.globl cf_driver_init
#-----------------------------------
cf_driver_init:

            str x19,[sp,#-32]!
            stp x29,lr,[sp,#16]
            add x29,sp,#16
            adrp x8, .L_ffffff8009965228
            ldrb w8,[x8,:lo12:.L_ffffff8009965228]
            cmp w8,#2
            b.lo .L_ffffff8009365794

            adrp x0, .L_ffffff8009228035
            adrp x1, .L_ffffff8009228fc8
            add x0,x0, :lo12:.L_ffffff8009228035
            add x1,x1, :lo12:.L_ffffff8009228fc8
            mov w2,#1876
            bl printk
.L_ffffff8009365794:

            adrp x0, .L_ffffff8009228fd7
            add x0,x0, :lo12:.L_ffffff8009228fd7
            bl printk

            adrp x0, .L_ffffff80093f3a38
            add x0,x0, :lo12:.L_ffffff80093f3a38
            mov w1,#1
            bl spi_register_board_info

            adrp x1, .L_ffffff8009965230
            add x1,x1, :lo12:.L_ffffff8009965230
            mov x0,xzr
            bl __spi_register_driver

            mov w19,w0
            cbz w0,.L_ffffff80093657e8

            adrp x0, .L_ffffff8009229046
            adrp x1, .L_ffffff8009228fc8
            add x0,x0, :lo12:.L_ffffff8009229046
            add x1,x1, :lo12:.L_ffffff8009228fc8
            mov w2,#1889
            mov w3,w19
            bl printk

            b .L_ffffff8009365800
.L_ffffff80093657e8:

            adrp x0, .L_ffffff800922900f
            adrp x1, .L_ffffff8009228fc8
            add x0,x0, :lo12:.L_ffffff800922900f
            add x1,x1, :lo12:.L_ffffff8009228fc8
            mov w2,#1887
            bl printk
.L_ffffff8009365800:

            ldp x29,lr,[sp,#16]
            mov w0,w19
            ldr x19,[sp],#32
            ret
.size cf_driver_init, . - cf_driver_init

.pushsection ".cs3711.data.write", "aw"
#include "cf_data.S"
.popsection
