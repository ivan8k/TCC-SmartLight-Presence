#include <soc/rtc_cntl_reg.h>
#include <soc/rtc_io_reg.h>
#include <soc/soc_ulp.h>

    .set samples_log, 4
    .set samples, 16

    .bss
    .global pir_sensor_reading
pir_sensor_reading:
    .long 0

    .global ldr_sensor_readings
ldr_sensor_readings:
    .long 0

    .global pir_io_number
pir_io_number:
    .long 0

    .text
    .global start
start:
    move r3, pir_io_number
    ld r3, r3, 0

    move r0, r3
    jumpr read_pir_io_high, 16, ge

read_pir_io_low:
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S, 16)
    rsh r0, r0, r3
    jump read_pir_done

read_pir_io_high:
    READ_RTC_REG(RTC_GPIO_IN_REG, RTC_GPIO_IN_NEXT_S+16, 2)
    sub r3, r3, 16
    rsh r0, r0, r3

read_pir_done:
    and r0, r0, 1
    move r2, pir_sensor_reading
    st r0, r2, 0

setup_ldr_reading:
    move r1, 0
    stage_rst

read_ldr:
    move r0, 0
    adc r0, 0, 6
    add r1, r1, r0
    stage_inc 1
    //wait 8000
    //jumps read_ldr, samples, LT
    //rsh r1, r1, samples_log
    move r2, ldr_sensor_readings
    st r1, r2, 0


wake_up:
    READ_RTC_FIELD(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP)
    and r0, r0, 1
    jump wake_up, eq

    wake
    halt