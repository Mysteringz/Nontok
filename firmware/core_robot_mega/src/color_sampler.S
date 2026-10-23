/* ATmega2560, D32/PC5. Call with interrupts disabled; restore SREG afterwards.
 * AVR ABI: r18-r20 and r24 are call-clobbered; uint8_t result returned in r24.
 * All loop paths take 9 cycles: SBRC + INC together always take 2.
 * Initial eight NOPs give the first observation the same 9-cycle spacing.
 * Exposure: 128 * 9 = 1152 cycles = 72 us at 16 MHz.
 * Body including RET: 1166 cycles. CALL/wrapper add a few cycles; the blackout
 * is below one 115200-baud 8N1 byte (86.8 us). No port/timer writes or edge ISRs.
 */
#include <avr/io.h>
#if !defined(__AVR_ATmega2560__)
#error "Colour sampler requires ATmega2560 D32/PC5"
#endif
.text
.global colorPulseBurst
.type colorPulseBurst, @function
colorPulseBurst:
    clr r24
    ldi r20, 128
    in r18, _SFR_IO_ADDR(PINC)
    nop
    nop
    nop
    nop
    nop
    nop
    nop
    nop
.Lcolor_loop:
    in r19, _SFR_IO_ADDR(PINC)
    eor r18, r19
    and r18, r19
    sbrc r18, PC5
    inc r24
    mov r18, r19
    dec r20
    brne .Lcolor_loop
    ret
.size colorPulseBurst, .-colorPulseBurst
