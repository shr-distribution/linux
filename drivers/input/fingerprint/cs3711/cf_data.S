// Extracted symbols from assembly files
// References from: cs3711.S
// Definitions from: titan-kernel-stock.asm

.align 8
.L_ffffff8008f5eda8:
          .short (.L_ffffff8008a5e764-.L_ffffff8008a5e764)/4
          .short (.L_ffffff8008a5e7d8-.L_ffffff8008a5e764)/4
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x8f
          .byte 0x0
          .byte 0x9b
          .byte 0x0
          .byte 0xa7
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0xc5
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0xd5
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0xa9
          .byte 0x1
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0x72
          .byte 0x0
          .byte 0xa9
          .byte 0x1
.L_ffffff8008f5ee54:
          .byte (.L_ffffff8008a5e9e0-.L_ffffff8008a5e918)/4
          .byte 0xca
          .byte 0xcc
          .byte 0x0
          .byte 0x0
          .byte 0xce
          .byte 0xd0
          .byte 0xd2
          .byte 0xd4
          .byte 0xd6
          .byte 0xd8
          .byte 0xda
          .byte 0xdc
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
.L_ffffff8008f5ee68:
          .zero 16
.L_ffffff8008f5ee78:
          .quad cf_suspend
.L_ffffff8008f5ee80:
          .quad cf_resume
          .zero 152
.L_ffffff8008f5ef20:
          .zero 24
.L_ffffff8008f5ef38:
          .quad cf_write
          .zero 40
.L_ffffff8008f5ef68:
          .quad cf_ioctl
          .zero 16
.L_ffffff8008f5ef80:
          .quad cf_open
          .zero 8
.L_ffffff8008f5ef90:
          .quad cf_release
          .zero 8
.L_ffffff8008f5efa0:
          .quad cf_fasync
          .zero 104
.L_ffffff8008f5f010:
          .zero 24
.L_ffffff8008f5f028:
          .quad .L_ffffff8009965488
          .zero 8

.align 8
.L_ffffff80090df3f3:
          .string "%s"
.L_ffffff80090df3f6:
          .string "\nTgid:\t"
.L_ffffff80090df3fe:
          .string "\nNgid:\t"
.L_ffffff80090df406:
          .string "\nPid:\t"
.L_ffffff80090df40d:
          .string "\nPPid:\t"
.L_ffffff80090df415:
          .string "\nTracerPid:\t"
.L_ffffff80090df422:
          .string "\nUid:\t"
.L_ffffff80090df429:
          .string "\nGid:\t"
.L_ffffff80090df430:
          .string "\nFDSize:\t"
.L_ffffff80090df43a:
          .string "\nGroups:\t"
.L_ffffff80090df444:
          .string "R (running)"
.L_ffffff80090df450:
          .string "S (sleeping)"
.L_ffffff80090df45d:
          .string "D (disk sleep)"
.L_ffffff80090df46c:
          .string "T (stopped)"
.L_ffffff80090df478:
          .string "t (tracing stop)"
.L_ffffff80090df489:
          .string "X (dead)"
.L_ffffff80090df492:
          .string "Z (zombie)"
.L_ffffff80090df49d:
          .string "P (parked)"
.L_ffffff80090df4a8:
          .string "I (idle)"
.L_ffffff80090df4b1:
          .string "Threads:\t"
.L_ffffff80090df4bb:
          .string "\nSigQ:\t"
.L_ffffff80090df4c3:
          .string "\nSigPnd:\t"
.L_ffffff80090df4cd:
          .string "ShdPnd:\t"
.L_ffffff80090df4d6:
          .string "SigBlk:\t"
.L_ffffff80090df4df:
          .string "SigIgn:\t"
.L_ffffff80090df4e8:
          .string "SigCgt:\t"
.L_ffffff80090df4f1:
          .string "CapInh:\t"
.L_ffffff80090df4fa:
          .string "CapPrm:\t"
.L_ffffff80090df503:
          .string "CapEff:\t"
.L_ffffff80090df50c:
          .string "CapBnd:\t"
.L_ffffff80090df515:
          .string "CapAmb:\t"
.L_ffffff80090df51e:
          .string "NoNewPrivs:\t"
.L_ffffff80090df52b:
          .string "\nSeccomp:\t"
.L_ffffff80090df536:
          .string "\nSpeculation_Store_Bypass:\t"
.L_ffffff80090df552:
          .string "not vulnerable"
.L_ffffff80090df561:
          .string "thread force mitigated"
.L_ffffff80090df578:
          .string "thread mitigated"
.L_ffffff80090df589:
          .ascii "thread "
.L_ffffff80090df590:
          .string "vulnerable"
.L_ffffff80090df59b:
          .string "globally mitigated"
.L_ffffff80090df5ae:
          .string "Cpus_allowed:\t%*pb\n"
.L_ffffff80090df5c2:
          .string "Cpus_allowed_list:\t%*pbl\n"
.L_ffffff80090df5dc:
          .ascii "\nnon"
.L_ffffff80090df5e0:
          .string "voluntary_ctxt_switches:\t"
.L_ffffff80090df5fa:
          .ascii "%d (%s)"
.L_ffffff80090df601:
          .ascii " "
.L_ffffff80090df602:
          .ascii "%"
.L_ffffff80090df603:
          .string "c"
.L_ffffff80090df605:
          .ascii " 0 0 0 0 0 0"
.L_ffffff80090df611:
          .string " 0"
.L_ffffff80090df614:
          .ascii "pos:\t%lli\nflags:\t0%o\nmnt_id:\t"
.L_ffffff80090df631:
          .string "%i\n"

.align 8
.L_ffffff80090f7c05:
          .string "debounce"

.align 8
.L_ffffff80090f9cc8:
          .string "172"

.align 8
.L_ffffff8009151a5d:
          .string "hw_reset"

.align 8
.L_ffffff800915726f:
          .string "irq"

.align 8
.L_ffffff8009204401:
          .string "gpio-irq-std"
.L_ffffff800920440e:
          .byte 0x1
          .byte 0x33
          .byte 0x25
          .byte 0x73
          .byte 0x20
          .byte 0x3a
          .byte 0x20
          .byte 0x67
          .byte 0x70
          .byte 0x69
          .byte 0x6f
          .byte 0x5f
          .byte 0x72
          .byte 0x65
          .byte 0x71
          .byte 0x75
          .byte 0x65
          .byte 0x73
          .byte 0x74
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x65
          .byte 0x64
          .byte 0xa
          .byte 0x0
.L_ffffff800920442a:
          .byte 0x1
          .byte 0x36
          .byte 0x25
          .byte 0x73
          .byte 0x20
          .byte 0x3a
          .byte 0x20
          .byte 0x49
          .byte 0x52
          .byte 0x51
          .byte 0x20
          .byte 0x47
          .byte 0x50
          .byte 0x49
          .byte 0x4f
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x64
          .byte 0xa
          .byte 0x0
.L_ffffff8009204440:
          .byte 0x1
          .byte 0x33
          .byte 0x25
          .byte 0x73
          .byte 0x20
          .byte 0x3a
          .byte 0x20
          .byte 0x67
          .byte 0x70
          .byte 0x69
          .byte 0x6f
          .byte 0x5f
          .byte 0x64
          .byte 0x69
          .byte 0x72
          .byte 0x65
          .byte 0x63
          .byte 0x74
          .byte 0x69
          .byte 0x6f
          .byte 0x6e
          .byte 0x5f
          .byte 0x69
          .byte 0x6e
          .byte 0x70
          .byte 0x75
          .byte 0x74
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x65
          .byte 0x64
          .byte 0xa
          .byte 0x0
.L_ffffff8009204464:
          .string "gpio-rst-std"

.align 8
.L_ffffff800922200c:
          .string "fb_notifier_callback"

.align 8
.L_ffffff8009228035:
          .string "[chipsailing]%s line:%d  entry\n"
.L_ffffff8009228055:
          .string "cf_driver_exit"
.L_ffffff8009228064:
          .string "chipsailing"
.L_ffffff8009228070:
          .string "[chipsailing]%s line:%d  chipsailing cf_probe\n"
.L_ffffff800922809f:
          .string "cf_probe"
.L_ffffff80092280a8:
          .string "chipsailing cs_finger cf_probe start!!!\n"
.L_ffffff80092280d1:
          .string "mediatek,fingerprint"
.L_ffffff80092280e6:
          .string "[chipsailing]%s line:%d  device node is null\n"
.L_ffffff8009228114:
          .string "[chipsailing]%s line:%d  platform device is null\n"
.L_ffffff8009228146:
          .string "chipsailing cs_finger cf_probe start2!!!\n"
.L_ffffff8009228170:
          .string "[chipsailing]%s line:%d  failed to allocate memory for struct cf_device"
.L_ffffff80092281b8:
          .string "chipsailing read id success!\n"
.L_ffffff80092281d6:
          .string "csfpsensor"
.L_ffffff80092281e1:
          .string "cs_spi"
.L_ffffff80092281e8:
          .string "[chipsailing]%s line:%d  alloc_chrdev_region failed, error = %d\n"
.L_ffffff8009228229:
          .string "[chipsailing]%s line:%d  cdev_add failed, error = %d\n"
.L_ffffff800922825f:
          .string "&cf_dev->lock"
.L_ffffff800922826d:
          .string "cf_ttw_wl"
.L_ffffff8009228277:
          .string "[chipsailing]%s line:%d  could not register input\n"
.L_ffffff80092282aa:
          .string "[chipsailing]%s line:%d  platform_device_alloc failed\n"
.L_ffffff80092282e1:
          .string "[chipsailing]%s line:%d  platform_device_add failed\n"
.L_ffffff8009228316:
          .string "[chipsailing]%s line:%d  sysfs_create_group failed\n"
.L_ffffff800922834a:
          .string "[chipsailing]%s line:%d  could not request irq\n"
.L_ffffff800922837a:
          .string "chipsaling probe ok\n"
.L_ffffff800922838f:
          .string "[chipsailing]%s line:%d  exit\n"
.L_ffffff80092283ae:
          .string "[chipsailing]%s line:%d  of_find_compatible_node start\n"
.L_ffffff80092283e6:
          .string "cf_gpio_init"
.L_ffffff80092283f3:
          .byte 0x1
          .byte 0x33
          .byte 0x25
          .byte 0x73
          .byte 0x3a
          .byte 0x20
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x20
          .byte 0x67
          .byte 0x65
          .byte 0x74
          .byte 0x20
          .byte 0x4e
          .byte 0x46
          .byte 0x43
          .byte 0x20
          .byte 0x52
          .byte 0x53
          .byte 0x54
          .byte 0x20
          .byte 0x47
          .byte 0x50
          .byte 0x49
          .byte 0x4f
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x65
          .byte 0x64
          .byte 0x20
          .byte 0x28
          .byte 0x25
          .byte 0x64
          .byte 0x29
          .byte 0x0
.L_ffffff8009228422:
          .string "/home3/work/repo/q_a61_release/alps/kernel-4.14/drivers/input/fingerprint/cs3711/cf_ctl.c"
.L_ffffff800922847c:
          .byte 0x1
          .byte 0x33
          .byte 0x25
          .byte 0x73
          .byte 0x3a
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x20
          .byte 0x20
          .byte 0x67
          .byte 0x65
          .byte 0x74
          .byte 0x20
          .byte 0x4e
          .byte 0x46
          .byte 0x43
          .byte 0x20
          .byte 0x49
          .byte 0x52
          .byte 0x51
          .byte 0x20
          .byte 0x47
          .byte 0x50
          .byte 0x49
          .byte 0x4f
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x65
          .byte 0x64
          .byte 0x20
          .byte 0x28
          .byte 0x25
          .byte 0x64
          .byte 0x29
          .byte 0x0
.L_ffffff80092284ab:
          .string "[chipsailing]%s line:%d  of_find_compatible_node cf_dev->rst_gpio = %d cf_dev->irq_gpio = %dend\n"
.L_ffffff800922850c:
          .string "chipsailing cs_finger cf_pinctrl_look_up_states1!!!\n"
.L_ffffff8009228541:
          .string "chipsailing cs_finger cf_pinctrl_look_up_states1111!!!\n"
.L_ffffff8009228579:
          .string "[chipsailing]%s line:%d  Target does not use pinctrl\n"
.L_ffffff80092285af:
          .string "cf_pinctrl_look_up_states"
.L_ffffff80092285c9:
          .string "[chipsailing]%s line:%d  cannot find '%s'\n"
.L_ffffff80092285f4:
          .string "[chipsailing]%s line:%d  found pin control %s\n"
.L_ffffff8009228623:
          .string "finger_rst_high"
.L_ffffff8009228633:
          .string "finger_rst_low"
.L_ffffff8009228642:
          .string "finger_eint"
.L_ffffff800922864e:
          .string "finger_clk_as_spi_clk"
.L_ffffff8009228664:
          .string "finger_cs_as_spi_cs"
.L_ffffff8009228678:
          .string "miso_as_spi_mi"
.L_ffffff8009228687:
          .string "mosi_as_spi_mo"
.L_ffffff8009228696:
          .string "[chipsailing]%s line:%d  IRQ after reset %d\n"
.L_ffffff80092286c3:
          .string "[chipsailing]%s line:%d  RST after reset %d\n"
.L_ffffff80092286f0:
          .string "[chipsailing]%s line:%d  cf_spi_enable_clk \n"
.L_ffffff800922871d:
          .string "cf_spi_enable_clk"
.L_ffffff800922872f:
          .string "[chipsailing]%s line:%d  cf_check_chipid , rx[0] %x, rx[1] %x\n"
.L_ffffff800922876e:
          .string "get_and_check_chipid"
.L_ffffff8009228783:
          .string "chipsailing cs_finger cf_spi_disable_clk------------------------------!!!\n"
.L_ffffff80092287ce:
          .string "[chipsailing]%s line:%d  irq_gpio = %d, goio_to_irq = %d, irq = %d\n"
.L_ffffff8009228812:
          .string "cf_write"
.L_ffffff800922881b:
          .string "[chipsailing]%s line:%d  rst_gpio_value = %d, irq_gpio_value = %d\n"
.L_ffffff800922885e:
          .string "cf_ioctl"
.L_ffffff8009228867:
          .string "chipsailing cs_finger cf_ioctl------------------------------!!!\n"
.L_ffffff80092288a8:
          .string "[chipsailing]%s line:%d  CF_IOC_INIT_GPIO\n"
.L_ffffff80092288d3:
          .string "[chipsailing]%s line:%d  CF_IOC_DEINIT_GPIO\n"
.L_ffffff8009228900:
          .string "[chipsailing]%s line:%d  CF_IOC_RESET_DEVICE\n"
.L_ffffff800922892e:
          .string "[chipsailing]%s line:%d  copy_from_user(..) fail\n"
.L_ffffff8009228960:
          .string "[chipsailing]%s line:%d  CF_IOC_ENABLE_IRQ\n"
.L_ffffff800922898c:
          .string "[chipsailing]%s line:%d  CF_IOC_DISABLE_IRQ\n"
.L_ffffff80092289b9:
          .string "[chipsailing]%s line:%d  CF_IOC_REQ_IRQ\n"
.L_ffffff80092289e2:
          .string "[chipsailing]%s line:%d  CF_IOC_ENABLE_SPI_CLK\n"
.L_ffffff8009228a12:
          .string "[chipsailing]%s line:%d  CF_IOC_DISABLE_SPI_CLK\n"
.L_ffffff8009228a43:
          .string "[chipsailing]%s line:%d  copy_from_user(..) failed\n"
.L_ffffff8009228a77:
          .string "[chipsailing]%s line:%d  copy_to_user(..) failed\n"
.L_ffffff8009228aa9:
          .string "cf_reset_gpio_set_value"
.L_ffffff8009228ac1:
          .string "[chipsailing]%s line:%d  rst_gpio is not get\n"
.L_ffffff8009228aef:
          .string "cf_enable_irq"
.L_ffffff8009228afd:
          .string "chipsailing cs_finger enable_irq------------------------------!!!\n"
.L_ffffff8009228b40:
          .string "cf_disable_irq"
.L_ffffff8009228b4f:
          .string "chipsailing cs_finger disable_irq------------------------------!!!\n"
.L_ffffff8009228b93:
          .string "cf_report_key_event"
.L_ffffff8009228ba7:
          .string "Driver_v3.1.1-20181128"
.L_ffffff8009228bbe:
          .string "cf_open"
.L_ffffff8009228bc6:
          .string "cf_fasync"
.L_ffffff8009228bd0:
          .string "chipsailing cs_finger cf_fasync------------------------------!!!\n"
.L_ffffff8009228c12:
          .string "chipsailing cs_finger cf_device_event1 %s!!!\n"
.L_ffffff8009228c40:
          .string "chipsailing cs_finger cf_device_event2!!!\n"
.L_ffffff8009228c6b:
          .string "chipsailing cs_finger irq_get------------------------------!!!\n"
.L_ffffff8009228cab:
          .string "[chipsailing]%s line:%d  irq_ack-----------------------------------\n"
.L_ffffff8009228cf0:
          .string "irq_ack"
.L_ffffff8009228cf8:
          .string "cf_input_init"
.L_ffffff8009228d06:
          .string "[chipsailing]%s line:%d  input_allocate_device(..) fail\n"
.L_ffffff8009228d3f:
          .string "cf-keys"
.L_ffffff8009228d47:
          .string "[chipsailing]%s line:%d  input_register_device fail\n"
.L_ffffff8009228d7c:
          .string "[chipsailing]%s line:%d  gpio_set_debounce\n"
.L_ffffff8009228da8:
          .string "cf_irq_init"
.L_ffffff8009228db4:
          .string "[chipsailing]%s line:%d  irq_of_parse_and_map(..) fail\n"
.L_ffffff8009228dec:
          .string "cf_irq"
.L_ffffff8009228df3:
          .string "[chipsailing]%s line:%d  could not request irq %d\n"
.L_ffffff8009228e26:
          .string "chipsailing find irq cf_dev->irq = %d\n"
.L_ffffff8009228e4d:
          .string "cf_irq_handler"
.L_ffffff8009228e5c:
          .string "%s 111\n"
.L_ffffff8009228e64:
          .string "chipsailing cs_finger fb_notifier_callback------------------------------!!!\n"
.L_ffffff8009228eb1:
          .string "[chipsailing]%s line:%d  LCD on\n"
.L_ffffff8009228ed2:
          .string "[chipsailing]%s line:%d  LCD off\n"
.L_ffffff8009228ef4:
          .string "cf_fb_notify"
.L_ffffff8009228f01:
          .string "chipsailing cs_finger cf_fb_notify------------------------------!!!\n"
.L_ffffff8009228f46:
          .string "VDD"
.L_ffffff8009228f4a:
          .string "cf_remove"
.L_ffffff8009228f54:
          .string "chipsailing cs_finger vreg_setup------------------------------!!!\n"
.L_ffffff8009228f97:
          .string "vreg_setup"
.L_ffffff8009228fa2:
          .string "[chipsailing]%s line:%d  disabled %s\n"
.L_ffffff8009228fc8:
          .string "cf_driver_init"
.L_ffffff8009228fd7:
          .string "chipsailing cs_finger spi_register_board_info start!!!\n"
.L_ffffff800922900f:
          .string "[chipsailing]%s line:%d  spi_register_driver(..) pass\n"
.L_ffffff8009229046:
          .string "[chipsailing]%s line:%d  spi_register_driver(..) fail, error = %d\n"
.L_ffffff8009229089:
          .byte 0x1
          .byte 0x33
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x20
          .byte 0x74
          .byte 0x6f
          .byte 0x20
          .byte 0x72
          .byte 0x65
          .byte 0x61
          .byte 0x64
          .byte 0x20
          .byte 0x73
          .byte 0x66
          .byte 0x72
          .byte 0x2c
          .byte 0x65
          .byte 0x72
          .byte 0x72
          .byte 0x6f
          .byte 0x72
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x64
          .byte 0x0
.L_ffffff80092290b4:
          .byte 0x1
          .byte 0x33
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x73
          .byte 0x68
          .byte 0x6f
          .byte 0x72
          .byte 0x74
          .byte 0x20
          .byte 0x6f
          .byte 0x66
          .byte 0x20
          .byte 0x6d
          .byte 0x65
          .byte 0x6d
          .byte 0xa
          .byte 0x0
.L_ffffff80092290d1:
          .byte 0x1
          .byte 0x33
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x63
          .byte 0x66
          .byte 0x5f
          .byte 0x73
          .byte 0x70
          .byte 0x69
          .byte 0x5f
          .byte 0x72
          .byte 0x65
          .byte 0x61
          .byte 0x64
          .byte 0x5f
          .byte 0x61
          .byte 0x6e
          .byte 0x64
          .byte 0x5f
          .byte 0x77
          .byte 0x72
          .byte 0x69
          .byte 0x74
          .byte 0x65
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x2c
          .byte 0x65
          .byte 0x72
          .byte 0x72
          .byte 0x6f
          .byte 0x72
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x64
          .byte 0xa
          .byte 0x0
.L_ffffff8009229107:
          .byte 0x1
          .byte 0x33
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x63
          .byte 0x66
          .byte 0x5f
          .byte 0x73
          .byte 0x70
          .byte 0x69
          .byte 0x5f
          .byte 0x72
          .byte 0x65
          .byte 0x61
          .byte 0x64
          .byte 0x5f
          .byte 0x61
          .byte 0x6e
          .byte 0x64
          .byte 0x5f
          .byte 0x77
          .byte 0x72
          .byte 0x69
          .byte 0x74
          .byte 0x65
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x2c
          .byte 0x20
          .byte 0x65
          .byte 0x72
          .byte 0x72
          .byte 0x6f
          .byte 0x72
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x64
          .byte 0x0
.L_ffffff800922913d:
          .byte 0x1
          .byte 0x33
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x63
          .byte 0x66
          .byte 0x5f
          .byte 0x73
          .byte 0x66
          .byte 0x72
          .byte 0x5f
          .byte 0x77
          .byte 0x72
          .byte 0x69
          .byte 0x74
          .byte 0x65
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x2c
          .byte 0x20
          .byte 0x65
          .byte 0x72
          .byte 0x72
          .byte 0x6f
          .byte 0x72
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x64
          .byte 0x0
.L_ffffff800922916a:
          .byte 0x1
          .byte 0x33
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x70
          .byte 0x61
          .byte 0x72
          .byte 0x61
          .byte 0x6d
          .byte 0x2e
          .byte 0x63
          .byte 0x6d
          .byte 0x64
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x78
          .byte 0x2c
          .byte 0x70
          .byte 0x61
          .byte 0x72
          .byte 0x61
          .byte 0x6d
          .byte 0x2e
          .byte 0x61
          .byte 0x64
          .byte 0x64
          .byte 0x72
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x78
          .byte 0x2c
          .byte 0x70
          .byte 0x61
          .byte 0x72
          .byte 0x61
          .byte 0x6d
          .byte 0x2e
          .byte 0x64
          .byte 0x61
          .byte 0x74
          .byte 0x61
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x78
          .byte 0xa
          .byte 0x0
.L_ffffff80092291a9:
          .byte 0x1
          .byte 0x36
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x70
          .byte 0x61
          .byte 0x72
          .byte 0x61
          .byte 0x6d
          .byte 0x2e
          .byte 0x63
          .byte 0x6d
          .byte 0x64
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x78
          .byte 0x2c
          .byte 0x70
          .byte 0x61
          .byte 0x72
          .byte 0x61
          .byte 0x6d
          .byte 0x2e
          .byte 0x61
          .byte 0x64
          .byte 0x64
          .byte 0x72
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x78
          .byte 0x2c
          .byte 0x70
          .byte 0x61
          .byte 0x72
          .byte 0x61
          .byte 0x6d
          .byte 0x2e
          .byte 0x64
          .byte 0x61
          .byte 0x74
          .byte 0x61
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x78
          .byte 0xa
          .byte 0x0
.L_ffffff80092291e8:
          .byte 0x1
          .byte 0x33
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x69
          .byte 0x6e
          .byte 0x76
          .byte 0x61
          .byte 0x6c
          .byte 0x69
          .byte 0x64
          .byte 0x20
          .byte 0x61
          .byte 0x72
          .byte 0x67
          .byte 0x75
          .byte 0x6d
          .byte 0x65
          .byte 0x6e
          .byte 0x74
          .byte 0xa
          .byte 0x0
.L_ffffff8009229209:
          .byte 0x1
          .byte 0x33
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x20
          .byte 0x74
          .byte 0x6f
          .byte 0x20
          .byte 0x73
          .byte 0x79
          .byte 0x6e
          .byte 0x63
          .byte 0x20
          .byte 0x6d
          .byte 0x65
          .byte 0x73
          .byte 0x73
          .byte 0x61
          .byte 0x67
          .byte 0x65
          .byte 0x2c
          .byte 0x20
          .byte 0x65
          .byte 0x72
          .byte 0x72
          .byte 0x6f
          .byte 0x72
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x64
          .byte 0x0
.L_ffffff8009229239:
          .byte 0x1
          .byte 0x33
          .byte 0x63
          .byte 0x68
          .byte 0x69
          .byte 0x70
          .byte 0x73
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x3a
          .byte 0x20
          .byte 0x66
          .byte 0x61
          .byte 0x69
          .byte 0x6c
          .byte 0x20
          .byte 0x74
          .byte 0x6f
          .byte 0x20
          .byte 0x61
          .byte 0x73
          .byte 0x79
          .byte 0x6e
          .byte 0x63
          .byte 0x20
          .byte 0x6d
          .byte 0x65
          .byte 0x73
          .byte 0x73
          .byte 0x61
          .byte 0x67
          .byte 0x65
          .byte 0x2c
          .byte 0x20
          .byte 0x65
          .byte 0x72
          .byte 0x72
          .byte 0x6f
          .byte 0x72
          .byte 0x20
          .byte 0x3d
          .byte 0x20
          .byte 0x25
          .byte 0x64
          .byte 0x0

.align 8
.L_ffffff80093f3a38:
          .string "chipsailing"
          .zero 36

.align 8
.L_ffffff8009965228:
          .byte 0x2
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0

.L_ffffff8009965230:
.globl cf_driver
.type cf_driver, %object
cf_driver:
          .zero 8
.L_ffffff8009965238:
          .quad cf_probe
.L_ffffff8009965240:
          .quad cf_remove
          .zero 8
.L_ffffff8009965250:
          .quad .L_ffffff8009228064
          .zero 32
.L_ffffff8009965278:
          .quad .L_ffffff80099652c8
          .zero 56
.L_ffffff80099652b8:
          .quad .L_ffffff8008f5ee68
          .zero 8
.L_ffffff80099652c8:
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
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
          .byte 0x66
          .byte 0x69
          .byte 0x6e
          .byte 0x67
          .byte 0x65
          .byte 0x72
          .byte 0x70
          .byte 0x72
          .byte 0x69
          .byte 0x6e
          .byte 0x74
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
.L_ffffff8009965458:
          .byte 0x1
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x1
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
.L_ffffff8009965468:
          .quad .L_ffffff800915726f
          .byte 0xb4
          .byte 0x1
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
          .byte 0x0
.L_ffffff8009965478:
          .quad irq_get
.L_ffffff8009965480:
          .quad irq_ack
.L_ffffff8009965488:
          .quad .L_ffffff8009965468
          .zero 8
.L_ffffff8009965498:
          .zero 16

.align 8
.L_ffffff8009a454c8:
          .zero 8
.L_ffffff8009a454d0:
          .zero 16
.L_ffffff8009a454e0:
          .zero 8
.L_ffffff8009a454e8:
          .zero 12
.L_ffffff8009a454f4:
          .zero 12
.L_ffffff8009a45500:
          .zero 4
.L_ffffff8009a45504:
          .zero 12
.L_ffffff8009a45510:
          .zero 28
.L_ffffff8009a4552c:
          .zero 4
.L_ffffff8009a45530:
          .zero 20
.L_ffffff8009a45544:
          .zero 4

.align 8
.L_ffffff8009ad5688:
          .zero 8

.align 8
.L_ffffff800a0fb38c:
          .zero 4
.L_ffffff800a0fb390:
          .zero 1
.L_ffffff800a0fb391:
          .zero 1
.L_ffffff800a0fb392:
          .zero 34
.L_ffffff800a0fb3b4:
          .zero 4
.L_ffffff800a0fb3b8:
          .zero 32
