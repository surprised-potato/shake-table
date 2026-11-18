// Shake-table controller for ATmega328P (Arduino Uno / Nano), 57600 baud.
// Servo A on pin 9 (X axis), servo B on pin 10 (Y axis), LED on pin 13.
//
// Every command is one line ending in '\n'. Angles are limited to
// SERVO_LIMIT_LO..SERVO_LIMIT_HI (centre +/- 90 deg = 45-225 deg), where the
// arm's push on the table is one-directional, so the table never reverses
// direction partway through a servo move.
//
//   1 / 0 / t          LED on / off / toggle
//   A<deg> / B<deg>    jump servo A / B to <deg> (stops any motion)
//   H9 L9 H10 L10      pin test: hold pin HIGH / LOW (servo PWM off on that pin)
//   OA <min> <max> <speed>   axis A oscillation range (deg) and speed (deg/s)
//   OB <min> <max> <speed>   same for axis B (speed 0 = axis stays still)
//   GO                 start oscillating both axes
//   STOP               stop oscillating: both axes glide to the centre of their range
//   M <a> <b> <speed>  glide to angles a, b (TENTHS of a degree) at <speed> deg/s
//   T <a> <b> <ms>     timed move for time-series playback: reach a, b (tenths
//                      of a degree) in <ms> milliseconds, both axes arriving
//                      together. Silent: no MOVING/STOP replies.
//   ?                  report everything
//
// Replies (one per line):
//   ON / OFF, A<deg.d> / B<deg.d>, P9=H|L|S / P10=H|L|S,
//   OA <min> <max> <speed> / OB ..., RUN / MOVING / STOP, READY at boot.
//   While moving, A and B positions are streamed 10 times a second.
//
// Timing: Timer1 runs the servo PWM at 50 Hz (OC1A = pin 9, OC1B = pin 10).
// Its overflow interrupt fires once per PWM period and advances the motion,
// so each servo gets exactly one new position per pulse.
#define F_CPU 16000000UL
#define BAUD 57600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <stdint.h>
#include <string.h>

// 270-degree servo: 500-2500 us pulse = 0-270 deg, centre (level) at 135 deg.
// For a 180-degree servo use 180 / 544 / 2400.
#define SERVO_RANGE 270
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_CENTRE (SERVO_RANGE / 2)
#define SERVO_LIMIT_LO (SERVO_CENTRE - 90)   // 45
#define SERVO_LIMIT_HI (SERVO_CENTRE + 90)   // 225
#define TICK_HZ 50
#define TICK_MS (1000 / TICK_HZ)
#define MAX_SPEED 1000          // deg/s cap
#define HOMING_MIN_SPEED 30     // deg/s floor when gliding home

enum { IDLE, RUNNING, MOVING };

// Per-axis state; index 0 = A (pin 9), 1 = B (pin 10). Positions in millidegrees.
static volatile int32_t pos[2] = { SERVO_CENTRE * 1000L, SERVO_CENTRE * 1000L };
static volatile int8_t dir[2] = { 1, 1 };
static volatile uint16_t lo[2] = { SERVO_CENTRE - 20, SERVO_CENTRE - 20 }, hi[2] = { SERVO_CENTRE + 20, SERVO_CENTRE + 20 };
static volatile uint16_t speed[2] = { 120, 120 };
static volatile int32_t target[2];         // MOVING: where each axis is gliding to
static volatile int32_t move_step[2];      // MOVING: millidegrees per tick
static volatile uint8_t mode = IDLE;
static volatile uint8_t quiet = 0;         // current move came from T: don't report MOVING/STOP
static volatile uint8_t report_due = 0;    // set by ISR: stream positions
static volatile uint8_t mode_changed = 0;  // set by ISR: glide finished

// ---- UART with interrupt-driven receive ----------------------------------
#define RX_SIZE 128
static volatile char rx_buf[RX_SIZE];
static volatile uint8_t rx_head = 0, rx_tail = 0;

ISR(USART_RX_vect) {
    char c = UDR0;
    uint8_t next = (rx_head + 1) % RX_SIZE;
    if (next != rx_tail) { rx_buf[rx_head] = c; rx_head = next; }
}

static int rx_read(void) {
    if (rx_head == rx_tail) return -1;
    char c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_SIZE;
    return (uint8_t)c;
}

static void uart_init(void) {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= (1 << U2X0);
#else
    UCSR0A &= ~(1 << U2X0);
#endif
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

static void uart_putc(char c) {
    while (!(UCSR0A & (1 << UDRE0))) {}
    UDR0 = c;
}

static void uart_puts(const char *s) {
    while (*s) uart_putc(*s++);
}

static void uart_putu(uint16_t v) {
    char buf[6];
    uint8_t i = 0;
    do { buf[i++] = '0' + v % 10; v /= 10; } while (v);
    while (i) uart_putc(buf[--i]);
}

// ---- Limits ---------------------------------------------------------------
static uint16_t clamp_deg(uint16_t d) {
    if (d < SERVO_LIMIT_LO) return SERVO_LIMIT_LO;
    if (d > SERVO_LIMIT_HI) return SERVO_LIMIT_HI;
    return d;
}

static uint16_t clamp_tenths(uint16_t t) {
    if (t < SERVO_LIMIT_LO * 10) return SERVO_LIMIT_LO * 10;
    if (t > SERVO_LIMIT_HI * 10) return SERVO_LIMIT_HI * 10;
    return t;
}

// ---- Servo output ---------------------------------------------------------
static uint16_t mdeg_to_ticks(int32_t mdeg) {
    if (mdeg < SERVO_LIMIT_LO * 1000L) mdeg = SERVO_LIMIT_LO * 1000L;
    if (mdeg > SERVO_LIMIT_HI * 1000L) mdeg = SERVO_LIMIT_HI * 1000L;
    uint32_t us = SERVO_MIN_US + (uint32_t)mdeg * (SERVO_MAX_US - SERVO_MIN_US) / (SERVO_RANGE * 1000UL);
    return (uint16_t)(us * 2);  // 0.5 us per tick
}

static void write_servo(uint8_t i, int32_t mdeg) {
    if (i == 0) OCR1A = mdeg_to_ticks(mdeg);
    else        OCR1B = mdeg_to_ticks(mdeg);
}

static void servo_init(void) {
    DDRB |= (1 << PB1) | (1 << PB2);  // pins 9 and 10 as outputs
    // Fast PWM mode 14 (TOP = ICR1), non-inverting on OC1A and OC1B, prescaler 8.
    TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = 39999;                     // 16 MHz / 8 / 40000 = 50 Hz
    write_servo(0, pos[0]);
    write_servo(1, pos[1]);
    TIMSK1 = (1 << TOIE1);            // overflow interrupt once per PWM period
}

// Move axis i one tick towards target at `step` millidegrees/tick. Returns 1 when there.
static uint8_t approach(uint8_t i, int32_t tgt, int32_t step) {
    int32_t p = pos[i];
    if (p < tgt) { p += step; if (p > tgt) p = tgt; }
    else if (p > tgt) { p -= step; if (p < tgt) p = tgt; }
    pos[i] = p;
    return p == tgt;
}

ISR(TIMER1_OVF_vect) {
    static uint8_t n = 0;
    if (mode == IDLE) return;

    uint8_t home = 1;
    for (uint8_t i = 0; i < 2; i++) {
        if (mode == RUNNING) {
            int32_t step = (int32_t)speed[i] * 1000 / TICK_HZ;
            int32_t l = (int32_t)lo[i] * 1000, h = (int32_t)hi[i] * 1000;
            if (dir[i] > 0) {
                if (pos[i] > h) dir[i] = -1;           // above range: head down into it
                else { pos[i] += step; if (pos[i] >= h) { pos[i] = h; dir[i] = -1; } }
            } else {
                if (pos[i] < l) dir[i] = 1;            // below range: head up into it
                else { pos[i] -= step; if (pos[i] <= l) { pos[i] = l; dir[i] = 1; } }
            }
        } else { // MOVING: glide to target
            if (!approach(i, target[i], move_step[i])) home = 0;
        }
        write_servo(i, pos[i]);
    }
    if (mode == MOVING && home) { mode = IDLE; if (!quiet) mode_changed = 1; }
    if (++n >= TICK_HZ / 10) { n = 0; report_due = 1; }
}

// ---- Reports --------------------------------------------------------------
static int32_t get_pos(uint8_t i) {
    cli(); int32_t p = pos[i]; sei();
    return p;
}

static void report_led(void) {
    uart_puts((PORTB & (1 << PB5)) ? "ON\n" : "OFF\n");
}

// "A90.5" - angle with one decimal
static void report_servo(uint8_t i) {
    uint16_t tenths = (uint16_t)((get_pos(i) + 50) / 100);
    uart_putc(i == 0 ? 'A' : 'B');
    uart_putu(tenths / 10);
    uart_putc('.');
    uart_putc('0' + tenths % 10);
    uart_putc('\n');
}

static void report_pin(uint8_t pin) {
    uint8_t bit = (pin == 9) ? PB1 : PB2;
    uint8_t com = (pin == 9) ? COM1A1 : COM1B1;
    uart_puts(pin == 9 ? "P9=" : "P10=");
    if (TCCR1A & (1 << com)) uart_putc('S');
    else uart_putc((PINB & (1 << bit)) ? 'H' : 'L');  // actual pin level
    uart_putc('\n');
}

static void report_osc(uint8_t i) {
    uart_puts(i == 0 ? "OA " : "OB ");
    uart_putu(lo[i]); uart_putc(' ');
    uart_putu(hi[i]); uart_putc(' ');
    uart_putu(speed[i]); uart_putc('\n');
}

static void report_mode(void) {
    uart_puts(mode == RUNNING ? "RUN\n" : mode == MOVING ? "MOVING\n" : "STOP\n");
}

static void report_all(void) {
    report_led();
    report_servo(0); report_servo(1);
    report_pin(9); report_pin(10);
    report_osc(0); report_osc(1);
    report_mode();
}

// ---- Commands -------------------------------------------------------------
static void pwm_on(uint8_t i) {
    TCCR1A |= (1 << (i == 0 ? COM1A1 : COM1B1));
}

static uint16_t parse_uint(const char **p, uint8_t *ok) {
    while (**p == ' ') (*p)++;
    if (**p < '0' || **p > '9') { *ok = 0; return 0; }
    uint16_t v = 0;
    while (**p >= '0' && **p <= '9') {
        if (v < 10000) v = v * 10 + (**p - '0');
        (*p)++;
    }
    return v;
}

static void stop_motion(void) {
    if (mode != IDLE) { mode = IDLE; if (!quiet) report_mode(); }
}

static void handle(const char *line) {
    uint8_t ok = 1;
    const char *p = line + 1;

    if (!strcmp(line, "1")) { PORTB |=  (1 << PB5); report_led(); return; }
    if (!strcmp(line, "0")) { PORTB &= ~(1 << PB5); report_led(); return; }
    if (!strcmp(line, "t")) { PORTB ^=  (1 << PB5); report_led(); return; }
    if (!strcmp(line, "?")) { report_all(); return; }

    if (!strcmp(line, "GO")) {
        cli();
        for (uint8_t i = 0; i < 2; i++) dir[i] = (pos[i] > (int32_t)hi[i] * 1000) ? -1 : 1;
        mode = RUNNING;
        quiet = 0;
        sei();
        pwm_on(0); pwm_on(1);
        report_mode(); report_pin(9); report_pin(10);
        return;
    }
    if (!strcmp(line, "STOP")) {
        if (mode == RUNNING) {
            cli();
            for (uint8_t i = 0; i < 2; i++) {
                target[i] = ((int32_t)lo[i] + hi[i]) * 500;   // centre of range, millideg
                uint16_t s = speed[i] < HOMING_MIN_SPEED ? HOMING_MIN_SPEED : speed[i];
                move_step[i] = (int32_t)s * 1000 / TICK_HZ;
            }
            mode = MOVING;
            quiet = 0;
            sei();
            report_mode();
        }
        return;
    }

    if (line[0] == 'M' && line[1] == ' ') {
        uint16_t a = parse_uint(&p, &ok), b = parse_uint(&p, &ok), s = parse_uint(&p, &ok);
        if (!ok) return;
        a = clamp_tenths(a);
        b = clamp_tenths(b);
        if (s < 1) s = 1;
        if (s > MAX_SPEED) s = MAX_SPEED;
        cli();
        target[0] = (int32_t)a * 100;   // tenths of a degree -> millidegrees
        target[1] = (int32_t)b * 100;
        move_step[0] = move_step[1] = (int32_t)s * 1000 / TICK_HZ;
        mode = MOVING;
        quiet = 0;
        sei();
        pwm_on(0); pwm_on(1);
        report_mode();
        return;
    }

    if (line[0] == 'T' && line[1] == ' ') {
        uint16_t a = parse_uint(&p, &ok), b = parse_uint(&p, &ok), ms = parse_uint(&p, &ok);
        if (!ok) return;
        a = clamp_tenths(a);
        b = clamp_tenths(b);
        int32_t ticks = ms / TICK_MS;
        if (ticks < 1) ticks = 1;
        cli();
        target[0] = (int32_t)a * 100;
        target[1] = (int32_t)b * 100;
        for (uint8_t i = 0; i < 2; i++) {
            int32_t d = target[i] - pos[i];
            if (d < 0) d = -d;
            int32_t step = (d + ticks - 1) / ticks;   // arrive in `ticks` PWM periods
            move_step[i] = step > 0 ? step : 1;
        }
        quiet = 1;
        mode = MOVING;
        sei();
        pwm_on(0); pwm_on(1);
        return;
    }

    if (line[0] == 'O' && (line[1] == 'A' || line[1] == 'B')) {
        uint8_t i = line[1] == 'A' ? 0 : 1;
        p = line + 2;
        uint16_t a = parse_uint(&p, &ok), b = parse_uint(&p, &ok), s = parse_uint(&p, &ok);
        if (!ok) return;
        a = clamp_deg(a);
        b = clamp_deg(b);
        if (a > b) { uint16_t t = a; a = b; b = t; }
        if (s > MAX_SPEED) s = MAX_SPEED;
        cli(); lo[i] = a; hi[i] = b; speed[i] = s; sei();
        report_osc(i);
        return;
    }

    if (line[0] == 'A' || line[0] == 'B') {
        uint16_t angle = parse_uint(&p, &ok);
        if (!ok || *p) return;
        angle = clamp_deg(angle);
        uint8_t i = line[0] == 'A' ? 0 : 1;
        quiet = 0;
        stop_motion();
        cli(); pos[i] = (int32_t)angle * 1000; sei();
        write_servo(i, (int32_t)angle * 1000);
        pwm_on(i);
        report_servo(i);
        report_pin(i == 0 ? 9 : 10);
        return;
    }

    if (line[0] == 'H' || line[0] == 'L') {
        uint16_t pin = parse_uint(&p, &ok);
        if (!ok || *p || (pin != 9 && pin != 10)) return;
        uint8_t bit = (pin == 9) ? PB1 : PB2;
        TCCR1A &= ~(1 << ((pin == 9) ? COM1A1 : COM1B1));
        if (line[0] == 'H') PORTB |= (1 << bit); else PORTB &= ~(1 << bit);
        report_pin(pin);
        return;
    }
}

int main(void) {
    DDRB |= (1 << PB5);      // pin 13 (onboard LED) as output
    PORTB &= ~(1 << PB5);    // start off
    servo_init();            // both servos start at the centre (135)
    uart_init();
    sei();
    uart_puts("READY\n");
    report_all();

    char line[32];
    uint8_t len = 0;

    for (;;) {
        int c = rx_read();
        if (c >= 0) {
            if (c == '\n' || c == '\r') {
                if (len) { line[len] = 0; handle(line); }
                len = 0;
            } else if (len < sizeof(line) - 1) {
                line[len++] = (char)c;
            }
        }
        if (report_due) {
            report_due = 0;
            report_servo(0);
            report_servo(1);
        }
        if (mode_changed) {
            mode_changed = 0;
            report_mode();
            report_servo(0);
            report_servo(1);
        }
    }
}
