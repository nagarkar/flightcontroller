SET PATH=C:\tools\Keil_v5\ARM\ARMCC\Bin;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0;C:\ProgramData\Oracle\Java\javapath;C:\qp\qtools\bin;C:\utils\;C:\Users\miro\AppData\Local\Pandoc\
SET ARMCC5_ASMOPT=--diag_suppress=9931
SET ARMCC5_CCOPT=--diag_suppress=9931
SET ARMCC5_LINKOPT=--diag_suppress=9931
SET CPU_TYPE=STM32F746NGHx
SET CPU_VENDOR=STMicroelectronics
SET UV2_TARGET=dpp-spy
SET CPU_CLOCK=0x00B71B00
cmd /c "if exist .\spy\qstamp.o del .\spy\qstamp.o"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\bsp.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\main.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\philo.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\table.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qstamp.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via ".\spy\startup_stm32f746xx._ia"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32746g_discovery.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal_msp.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\system_stm32f7xx.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal_cortex.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal_gpio.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal_pwr_ex.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal_rcc.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal_rcc_ex.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal_uart.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal_dma.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\stm32f7xx_hal_i2c.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qep_hsm.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qep_msm.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_act.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_actq.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_defer.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_dyn.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_mem.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_ps.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_qact.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_qeq.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_qmact.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qf_time.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qk.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qk_mutex.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmAsm" --Via ".\spy\qk_port._ia"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qs.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qs_64bit.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qs_fp.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmCC" --Via ".\spy\qs_rx.__i"
"C:\tools\Keil_v5\ARM\ARMCC\Bin\ArmLink" --Via ".\spy\dpp-qk.lnp"
fromelf --bin --output .\spy\dpp-qk.bin .\spy\dpp-qk.axf
