/*==============================================================================
 Project: SUMO Bot

 Ultrasonic-guided sumo robot for the UBMP4.2 (PIC16F1459, 48 MHz, XC8).

 A single forward-facing ultrasonic sensor (trigger H5OUT, echo H1IN) ranges the
 opponent. On power-up the bot sits in a MODE-SELECT menu (LED1 heartbeat-blinks)
 and waits for a button; it then beeps back 1/2/3 times and runs that mode:

   SW2 -> ADVANCED   : stop-and-scan search, lock + soft-start charge, grace on
                       dropout. The polished tracker. (beeps x1)
   SW3 -> SIMPLE     : slow nudge-and-pause spin (pauses so the sensor can
                       actually get an echo back); ram full power, no ramp, the
                       instant anything is inside the threshold. (beeps x2)
   SW4 -> EXPERIMENTAL: "snake" tracker -- drive forward while gently turning;
                       flip the turn direction every time the target leaves the
                       beam, so the heading weaves around the target and keeps a
                       MOVING opponent centred. Falls back to a spin-search if the
                       target is truly lost. (beeps x3)

 Press SW1 at any time to reset back to the menu (it is the bootloader button).

 HARDWARE NOTE -- the sensor sits ~10 cm AHEAD of the wheel axle (on the ramp).
 The bot pivots about the axle, so the sensor swings on a long lever arm: a small
 body yaw throws the beam well off a target. That is why the snake turn must stay
 GENTLE, and why every search turn is a small step.

 HC-SR04 NOTE -- the sensor needs ~60 ms between triggers or the previous ping's
 echo (still ringing in the air) corrupts the next reading. The drive/settle/pause
 burst lengths below keep pings about that far apart. Pinging faster than this was
 the original "misses targets while spinning" bug.

 Motor speed is software PWM (no hardware PWM is wired to the motor pins).
==============================================================================*/

#include    "xc.h"              // Microchip XC8 compiler include file
#include    "stdint.h"          // Include integer definitions
#include    "stdbool.h"         // Include Boolean (true/false) definitions

#include    "UBMP420.h"         // Include UBMP4.2 constants and functions
#include <builtins.h>

// Motor pin map (H-bridge, active-high outputs on PORTC):
//   left  motor: forward H8OUT, reverse H7OUT
//   right motor: forward H3OUT, reverse H6OUT
// Ultrasonic: trigger H5OUT (out), echo H1IN (in).

// ---- Range / ping (shared by all modes) -------------------------------------
#define ECHO_RISE_TIMEOUT  1200  // max 2us-iters waiting for echo to rise (~2.4ms) -> "no echo"
#define ECHO_HIGH_TIMEOUT  600   // max 10us-ticks of echo width (clamps far / stuck-high, ~100cm)
#define TICKS_NEAR         350   // lock/detect threshold in echo ticks (~60cm)  -- CALIBRATE
#define TICKS_CONTACT      45    // point-blank -> full-power ram (~8cm)          -- CALIBRATE

// ---- Software PWM -----------------------------------------------------------
#define PWM_QUANTUM_US     100   // software-PWM time slice (constant __delay_us arg)
#define PWM_PERIOD_QUANTA  10    // quanta per PWM period (1ms period, duty in 10% steps)
#define DUTY_MAX           10    // full power (== PWM_PERIOD_QUANTA)
#define STALL_FLOOR        2     // a driven wheel is never commanded below this

#define CHARGE_PERIODS     60    // ~ms of forward drive per burst (also paces pings to ~60ms)

// ---- ADVANCED (SW2): stop-and-scan search + soft-start charge ---------------
#define SEARCH_KICK_PERIODS 10   // ~ms of spin per scan step  -- CALIBRATE (too big -> skips targets)
#define SEARCH_KICK_DUTY    9    // spin duty during a scan kick (must break stiction)
#define SETTLE_MS           50   // stop-and-settle before pinging (clean heading + ping gap)
#define RAMP_MAX            2     // soft-start steps (limits current inrush -> fewer brownouts)
static const uint8_t rampDuty[RAMP_MAX + 1] = {7, 9, 10};  // soft-start -> full
#define GRACE_CYCLES        2     // charge-straight bursts kept after a brief dropout

// ---- SIMPLE (SW3): slow nudge-and-pause spin + full-power ram ---------------
#define SIMPLE_KICK_DUTY    8    // firm nudge (breaks stiction) ...
#define SIMPLE_KICK_PERIODS 5    // ... but very short, so each step is a small turn
#define SIMPLE_PAUSE_MS     70   // long pause: lets the chassis stop AND the sensor echo back

// ---- EXPERIMENTAL (SW4): "snake" / bang-bang edge tracker -------------------
// Drive forward while gently turning; flip the turn direction whenever the target
// drops out of the beam. The target always exits the side OPPOSITE the turn, so
// flipping always steers back toward it -> the heading oscillates around the
// target bearing. Keep the turn GENTLE so a 60 ms blind ping-gap can't overshoot
// the beam in one step (that would make the weave coarse/unstable).
#define SNAKE_TURN_CUT      3    // inner-wheel duty cut while curving  -- CALIBRATE (bigger = sharper weave)
#define LOST_LIMIT          5    // consecutive missed pings before giving up to a spin-search

// ---- Audio / indicators -----------------------------------------------------
#define LOCK_BEEP_MS       120   // "target locked" tone
#define LOSS_BEEP_MS       60    // "lost target" tone
#define BOOT_BEEP_MS       40    // power-on / menu-confirm chirp

// ---- Misc -------------------------------------------------------------------
#define SEARCH_CW          true  // default search spin direction (true = clockwise)
// SW1 (RA3) bootloader/kill button: require a SUSTAINED low before resetting so a
// single motor-noise/brownout glitch on the pin can't randomly reboot the bot.
#define KILL_DEBOUNCE      200   // consecutive low samples before RESET() is real

// TODO Set linker ROM ranges to 'default,-0-7FF' under "Memory model" pull-down.
// TODO Set linker code offset to '800' under "Additional options" pull-down.

typedef enum { MODE_ADVANCED = 1, MODE_SIMPLE = 2, MODE_EXPERIMENTAL = 3 } Mode;

static uint16_t killCount = 0;  // SW1 debounce accumulator (file scope: persists)

//------------------------------------------------------------------------------
// Debounced bootloader/kill check. Only resets after KILL_DEBOUNCE consecutive
// low reads, so electrical noise on RA3 can't trigger a spurious reset. A reset
// restarts the firmware -> back to the mode-select menu.
//------------------------------------------------------------------------------
void pollKill(void)
{
    if (SW1 == 0)
    {
        if (++killCount >= KILL_DEBOUNCE)
        {
            RESET();
        }
    }
    else
    {
        killCount = 0;
    }
}

//------------------------------------------------------------------------------
// Drive all four motor pins low (coast/stop).
//------------------------------------------------------------------------------
void motorsStop(void)
{
    H8OUT = 0; H7OUT = 0;   // left off
    H6OUT = 0; H3OUT = 0;   // right off
}

//------------------------------------------------------------------------------
// Software-PWM drive window: `periods` PWM periods at the requested per-wheel
// duty. leftOn/rightOn are on-quanta (0..PWM_PERIOD_QUANTA); *Fwd selects motor
// direction. Always writes all four pins every quantum, so there is never stale
// pin state or an H-bridge shoot-through. SW1 is debounced each quantum.
//------------------------------------------------------------------------------
void drive(uint8_t leftOn, uint8_t rightOn, bool leftFwd, bool rightFwd, uint8_t periods)
{
    for (uint8_t p = 0; p < periods; p++)
    {
        for (uint8_t q = 0; q < PWM_PERIOD_QUANTA; q++)
        {
            bool lOn = (q < leftOn);
            bool rOn = (q < rightOn);

            H8OUT = (lOn &&  leftFwd)  ? 1 : 0;   // left forward
            H7OUT = (lOn && !leftFwd)  ? 1 : 0;   // left reverse
            H3OUT = (rOn &&  rightFwd) ? 1 : 0;   // right forward
            H6OUT = (rOn && !rightFwd) ? 1 : 0;   // right reverse

            __delay_us(PWM_QUANTUM_US);
            pollKill();
        }
    }
}

//------------------------------------------------------------------------------
// Spin in place: cw == true turns clockwise (left fwd / right rev).
//------------------------------------------------------------------------------
void spin(bool cw, uint8_t duty, uint8_t periods)
{
    drive(duty, duty, cw, !cw, periods);
}

//------------------------------------------------------------------------------
// Drive forward while gently curving. cw == true curves right (CW), cutting the
// right (inner) wheel; cw == false curves left (CCW), cutting the left wheel.
//------------------------------------------------------------------------------
void curveForward(bool cw, uint8_t fwd)
{
    uint8_t inner = (fwd > SNAKE_TURN_CUT + STALL_FLOOR)
                    ? (uint8_t)(fwd - SNAKE_TURN_CUT) : STALL_FLOOR;
    if (cw)
    {
        drive(fwd, inner, true, true, CHARGE_PERIODS);   // forward-right (CW)
    }
    else
    {
        drive(inner, fwd, true, true, CHARGE_PERIODS);   // forward-left  (CCW)
    }
}

//------------------------------------------------------------------------------
// Blocking delay in 1 ms steps with motors stopped, debouncing SW1 throughout.
// Lets the chassis come to rest before a ranging ping and spaces pings out to
// the HC-SR04 recovery time.
//------------------------------------------------------------------------------
void settle(uint16_t ms)
{
    motorsStop();
    for (uint16_t i = 0; i < ms; i++)
    {
        __delay_ms(1);
        pollKill();
    }
}

//------------------------------------------------------------------------------
// Fire one ultrasonic ping and return the echo width in 10us ticks. Returns 0 if
// no echo arrives (sensor disconnected / nothing in range). Both waits bounded.
//------------------------------------------------------------------------------
uint16_t ping(void)
{
    uint16_t ticks = 0;
    uint16_t guard = 0;

    H5OUT = 0;              // trigger pulse
    __delay_us(5);
    H5OUT = 1;
    __delay_us(10);
    H5OUT = 0;

    while (H1IN == 0)       // bounded wait for echo to rise (0 = no echo)
    {
        if (++guard >= ECHO_RISE_TIMEOUT)
        {
            return 0;
        }
        __delay_us(2);
    }

    while (H1IN == 1)       // bounded measure of echo high-time, in 10us ticks
    {
        __delay_us(10);
        if (++ticks >= ECHO_HIGH_TIMEOUT)
        {
            break;
        }
    }
    return ticks;
}

//------------------------------------------------------------------------------
// Square-wave tone on the piezo for `ms` milliseconds (~2 kHz).
//------------------------------------------------------------------------------
void beepMs(uint16_t ms)
{
    uint16_t cycles = ms * 2;           // 2 kHz -> 2 cycles per millisecond
    for (uint16_t i = 0; i < cycles; i++)
    {
        BEEPER = 1;
        __delay_us(250);
        BEEPER = 0;
        __delay_us(250);
    }
}

//==============================================================================
// MODE: ADVANCED  (SW2)
// Stop-and-scan search; lock + beep; soft-start charge dead-straight; grace on a
// brief dropout. Locks on the first in-range ping.
//==============================================================================
void runAdvanced(void)
{
    uint8_t ramp      = 0;
    uint8_t graceLeft = 0;
    bool    locked    = false;
    uint8_t blink     = 0;

    while (1)
    {
        uint16_t ticks   = ping();
        bool     present = (ticks > 0 && ticks < TICKS_NEAR);

        if (present)
        {
            if (!locked)
            {
                beepMs(LOCK_BEEP_MS);   // "target locked"
                locked = true;
            }
            graceLeft = GRACE_CYCLES;
            LED1      = 0;              // solid = locked
            if (ramp < RAMP_MAX)
            {
                ramp++;
            }
            uint8_t duty = (ticks < TICKS_CONTACT) ? DUTY_MAX : rampDuty[ramp];
            drive(duty, duty, true, true, CHARGE_PERIODS);   // charge straight
        }
        else if (locked && graceLeft > 0)
        {
            graceLeft--;               // brief dropout: keep charging on last heading
            LED1 = 0;
            uint8_t duty = rampDuty[ramp];
            drive(duty, duty, true, true, CHARGE_PERIODS);
        }
        else
        {
            if (locked)
            {
                beepMs(LOSS_BEEP_MS);  // "lost target"
            }
            locked = false;
            ramp   = 0;
            blink ^= 1;
            LED1 = blink ? 0 : 1;      // slow blink = searching

            spin(SEARCH_CW, SEARCH_KICK_DUTY, SEARCH_KICK_PERIODS);  // small scan step
            settle(SETTLE_MS);                                       // stop, settle, then re-ping
        }
    }
}

//==============================================================================
// MODE: SIMPLE  (SW3)
// Slow nudge-and-pause spin: a short firm kick, then a long pause so the chassis
// stops and the sensor can actually echo back (this is what stops it missing
// targets). The instant anything is inside the threshold, ram full power -- no
// soft-start, no scan, no grace.
//==============================================================================
void runSimple(void)
{
    bool ramming = false;

    while (1)
    {
        uint16_t ticks   = ping();
        bool     present = (ticks > 0 && ticks < TICKS_NEAR);

        if (present)
        {
            if (!ramming)
            {
                beepMs(LOCK_BEEP_MS);  // target detected
                ramming = true;
            }
            LED1 = 0;
            drive(DUTY_MAX, DUTY_MAX, true, true, CHARGE_PERIODS);   // full-power ram, no ramp
        }
        else
        {
            ramming = false;
            LED1 = 1;
            spin(SEARCH_CW, SIMPLE_KICK_DUTY, SIMPLE_KICK_PERIODS);  // small slow nudge
            settle(SIMPLE_PAUSE_MS);                                 // pause -> sensor can respond
        }
    }
}

//==============================================================================
// MODE: EXPERIMENTAL  (SW4)  --  "snake" / bang-bang edge tracker
//
// Drive forward while gently turning in the current direction. Each time the
// target leaves the beam, INSTANTLY flip the turn direction. Because a target
// exits the beam on the side opposite the turn, flipping always steers back
// toward it -- so the heading snakes around the target bearing while the bot
// keeps advancing, naturally keeping a MOVING opponent centred.
//
//   target ahead, curving CW  -> target drifts to LEFT edge -> drops out
//   flip to CCW                -> curve back left            -> target returns
//   target drifts to RIGHT edge-> drops out -> flip to CW    -> ... repeat
//
// If the target is truly gone (the flip doesn't bring it back within LOST_LIMIT
// pings), fall back to an in-place spin-search until it is re-found. The forward
// speed soft-starts (rampDuty) -- it "accelerates forward" as you described.
//==============================================================================
void runExperimental(void)
{
    bool    turnCW     = SEARCH_CW;   // current curve direction (true = CW / right)
    bool    acquired   = false;       // are we currently tracking a target?
    bool    wasPresent = false;       // target seen on the previous ping?
    uint8_t lostRun    = 0;           // consecutive missed pings while tracking
    uint8_t ramp       = 0;           // forward soft-start position

    while (1)
    {
        uint16_t ticks   = ping();
        bool     present = (ticks > 0 && ticks < TICKS_NEAR);

        if (present)
        {
            // ---- Target in beam: advance, weaving toward it ----
            if (!acquired)
            {
                beepMs(LOCK_BEEP_MS);
                acquired = true;
            }
            lostRun = 0;
            LED1    = 0;                          // solid = tracking
            if (ramp < RAMP_MAX)
            {
                ramp++;
            }

            if (ticks < TICKS_CONTACT)
            {
                drive(DUTY_MAX, DUTY_MAX, true, true, CHARGE_PERIODS);  // point-blank: ram straight
            }
            else
            {
                curveForward(turnCW, rampDuty[ramp]);                   // forward + gentle turn
            }
        }
        else if (acquired)
        {
            // ---- Just lost the beam while tracking ----
            if (wasPresent)
            {
                turnCW = !turnCW;        // loss EDGE -> instantly flip turn direction
            }
            lostRun++;

            if (lostRun <= LOST_LIMIT)
            {
                LED1 = 0;
                curveForward(turnCW, rampDuty[ramp]);   // keep advancing the new way to re-find it
            }
            else
            {
                acquired = false;        // really gone -> drop to search
                ramp     = 0;
                beepMs(LOSS_BEEP_MS);
            }
        }
        else
        {
            // ---- Searching: in-place stop-and-scan spin until a target appears ----
            LED1 = 1;
            spin(turnCW, SEARCH_KICK_DUTY, SEARCH_KICK_PERIODS);
            settle(SETTLE_MS);
        }

        wasPresent = present;
    }
}

//==============================================================================
// Mode-select menu: confirm a button, beep 1/2/3 times, wait for release, run it.
//==============================================================================
Mode confirmMode(Mode m)
{
    beepMs(BOOT_BEEP_MS);                          // beep count == mode number
    for (uint8_t b = 1; b < (uint8_t)m; b++)
    {
        settle(120);
        beepMs(BOOT_BEEP_MS);
    }
    while (SW2 == 0 || SW3 == 0 || SW4 == 0)       // wait for release
    {
        __delay_ms(1);
        pollKill();
    }
    LED1 = 0;                                      // solid = running
    return m;
}

// Block (heartbeat-blinking LED1) until SW2/SW3/SW4 is pressed; return the mode.
Mode selectMode(void)
{
    uint16_t hb = 0;
    LED1 = 1;
    while (1)
    {
        if (SW2 == 0) { __delay_ms(25); if (SW2 == 0) return confirmMode(MODE_ADVANCED); }
        if (SW3 == 0) { __delay_ms(25); if (SW3 == 0) return confirmMode(MODE_SIMPLE); }
        if (SW4 == 0) { __delay_ms(25); if (SW4 == 0) return confirmMode(MODE_EXPERIMENTAL); }

        __delay_ms(1);
        pollKill();
        if (++hb >= 400)                            // ~0.4s heartbeat = "pick a mode"
        {
            hb = 0;
            LED1 ^= 1;
        }
    }
}

// The main function is required, and the program begins executing from here.
int main(void)
{
    OSC_config();               // Configure internal oscillator for 48 MHz
    UBMP4_config();             // Configure on-board UBMP4 I/O devices

    motorsStop();
    beepMs(BOOT_BEEP_MS);       // chirp so you know the firmware booted

    while (1)                   // pick a mode, run it; SW1 reset returns here
    {
        switch (selectMode())
        {
            case MODE_ADVANCED:     runAdvanced();     break;
            case MODE_SIMPLE:       runSimple();       break;
            case MODE_EXPERIMENTAL: runExperimental(); break;
        }
    }
}
