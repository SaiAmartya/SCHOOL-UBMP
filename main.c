/*==============================================================================
 Project: SUMO Bot

 Ultrasonic-guided sumo robot for the UBMP4.2 (PIC16F1459, 48 MHz, XC8).

 A single forward-facing ultrasonic sensor (trigger H5OUT, echo H1IN) ranges the
 opponent. The bot:
   - SEARCHES with a "stop-and-scan" spin: kick a small turn, stop, let the
     chassis settle, then take ONE clean range reading. Repeat. This is slow but
     reliable -- the bot is stationary at the instant it pings, so the reading
     belongs to a known heading.
   - LOCKS ON when a confirmed target is inside range, beeps, and CHARGES
     DEAD-STRAIGHT with a short soft-start ramp.
   - RAMS at full power once the opponent is at contact range.

 DESIGN NOTE -- why it just charges straight (no steering/tracking):
 There is ONE fixed forward sensor and no servo, so the bot has no left/right
 bearing information at all. The only honest thing it can know is "something is
 (or isn't) ahead, this far away." So the strategy is: scan until the target is
 in front, then drive straight at it; if it slips out of the beam, coast briefly
 (grace) and otherwise go back to scanning. An earlier version tried to fake a
 bearing by weaving the body and comparing readings ("sequential lobing"); with
 a 3 ms weave that produces no real rotation, so it was just steering on noise.

 HARDWARE NOTE -- the sensor sits ~10 cm AHEAD of the wheel axle (on the ramp).
 The bot pivots about the axle, so the sensor swings on a long lever arm. That is
 why we never steer while locked (a small yaw throws the beam right off the
 target) and why the search spin must be SMALL steps (see SEARCH_KICK_*).

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

// ---- Tunable constants (one place to tune; calibrate on the bench) -----------
#define ECHO_RISE_TIMEOUT  1200  // max 2us-iters waiting for echo to rise (~2.4ms) -> "no echo"
#define ECHO_HIGH_TIMEOUT  600   // max 10us-ticks of echo width (clamps far / stuck-high)
#define TICKS_NEAR         175   // lock threshold in echo ticks (~30cm)  -- CALIBRATE
#define TICKS_CONTACT      45    // point-blank -> full-power ram (~8cm)   -- CALIBRATE

#define PWM_QUANTUM_US     100   // software-PWM time slice (constant __delay_us arg)
#define PWM_PERIOD_QUANTA  10    // quanta per PWM period (1ms period, duty in 10% steps)

// HC-SR04 needs >= ~60ms between triggers or the previous ping's echo, still
// ringing in the air, corrupts the next reading. PING_GAP_MS enforces that gap.
// Pinging faster than this was the main reason the old code "missed" targets
// while spinning -- the readings were garbage, not absent.
#define PING_GAP_MS        60    // min time from one ping to the next

// Charge: drive straight in ~PING_GAP_MS bursts so the motors stay powered right
// up to the next clean ping (no wasted coasting between readings).
#define CHARGE_PERIODS     60    // PWM periods (~ms) of forward drive per charge burst

// Search "stop-and-scan": a short, firm kick turns the bot a small angle, then we
// stop and settle before pinging. Firm duty (beats stiction + the dragging ramp);
// short time (small angle so the narrow beam can't skip over a target).
//   Too-long kick / too-fast spin  -> angular gaps -> misses targets (old bug).
//   Too-short kick / too-low duty   -> stiction wins -> bot buzzes but won't turn.
#define SEARCH_KICK_PERIODS 10   // PWM periods (~ms) of spin per scan step  -- CALIBRATE
#define SEARCH_KICK_DUTY    9    // spin duty during the kick (must break stiction)
#define SETTLE_MS           50   // stop-and-settle before pinging (also covers ping gap)

#define DUTY_MAX           10    // full power (== PWM_PERIOD_QUANTA)
#define RAMP_MAX           2     // soft-start steps (limits current inrush -> fewer brownouts)
static const uint8_t rampDuty[RAMP_MAX + 1] = {7, 9, 10};  // soft-start -> full

#define GRACE_CYCLES       2     // charge-straight burst kept after a brief dropout
#define LOCK_CONFIRM       2     // consecutive in-range pings required to commit (ghost guard)

#define LOCK_BEEP_MS       120   // "target locked" tone (user-requested)
#define LOSS_BEEP_MS       60    // "lost target" tone
#define BOOT_BEEP_MS       40    // power-on / firmware-alive chirp

// SW1 (RA3) is the bootloader/kill button. It is polled often; require a SUSTAINED
// low before resetting so a single motor-noise/brownout glitch on the pin can't
// randomly reboot the bot mid-match (a major source of erratic behaviour).
#define KILL_DEBOUNCE      200   // consecutive low samples before RESET() is real

#define SEARCH_CW          true  // search spin direction (true = clockwise)

// TODO Set linker ROM ranges to 'default,-0-7FF' under "Memory model" pull-down.
// TODO Set linker code offset to '800' under "Additional options" pull-down.

static uint16_t killCount = 0;  // SW1 debounce accumulator (file scope: persists)

//------------------------------------------------------------------------------
// Debounced bootloader/kill check. Only resets after KILL_DEBOUNCE consecutive
// low reads, so electrical noise on RA3 can't trigger a spurious reset.
//------------------------------------------------------------------------------
void pollKill(void)
{
    if (SW1 == 0)
    {
        if (++killCount >= KILL_DEBOUNCE)
        {
            RESET();            // sustained press -> enter bootloader
        }
    }
    else
    {
        killCount = 0;          // released / noise glitch -> forget it
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
// pin state or an H-bridge shoot-through. At full duty (on == period) the pins
// simply stay high (max torque, no switching ripple). SW1 is debounced each
// quantum so the kill button stays responsive without false resets mid-charge.
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
// Blocking delay in 1 ms steps with motors stopped, debouncing SW1 throughout.
// Used to let the chassis come to rest before a ranging ping (and to space pings
// out to the HC-SR04 recovery time).
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
// Fire one ultrasonic ping and return the echo width in 10us ticks.
// Returns 0 if no echo arrives (sensor disconnected / nothing in range) instead
// of hanging forever. Both echo waits are bounded.
//------------------------------------------------------------------------------
uint16_t ping(void)
{
    uint16_t ticks = 0;
    uint16_t guard = 0;

    // Trigger pulse
    H5OUT = 0;
    __delay_us(5);
    H5OUT = 1;
    __delay_us(10);
    H5OUT = 0;

    // Bounded wait for the echo line to go high (0 = no echo this ping)
    while (H1IN == 0)
    {
        if (++guard >= ECHO_RISE_TIMEOUT)
        {
            return 0;
        }
        __delay_us(2);
    }

    // Bounded measure of the echo high-time, in 10us ticks
    while (H1IN == 1)
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
// Square-wave tone on the piezo for `ms` milliseconds (~2 kHz). Motors are off
// while beeping, so the chassis also stops -- the lock beep doubles as a brief
// "kill the search-spin momentum" pause before the charge.
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

// The main function is required, and the program begins executing from here.
int main(void)
{
    // Configure oscillator and I/O ports. These functions run once at start-up.
    OSC_config();               // Configure internal oscillator for 48 MHz
    UBMP4_config();             // Configure on-board UBMP4 I/O devices

    // Persistent state (survives across loop iterations).
    uint8_t ramp      = 0;      // soft-start position 0..RAMP_MAX
    uint8_t graceLeft = 0;      // charge-straight bursts left after a dropout
    uint8_t confirm   = 0;      // consecutive in-range pings (lock debounce)
    bool    locked    = false;  // currently charging a target
    uint8_t blink     = 0;      // search LED toggle

    motorsStop();
    beepMs(BOOT_BEEP_MS);       // chirp so you know the firmware booted

    // Code in this while loop runs repeatedly.
    while (1)
    {
        uint16_t ticks   = ping();
        bool     present = (ticks > 0 && ticks < TICKS_NEAR);

        if (present && (locked || ++confirm >= LOCK_CONFIRM))
        {
            // ---- Confirmed target: charge DEAD-STRAIGHT (no steering -- see
            //      design note; single fixed sensor gives no bearing). ----
            if (!locked)
            {
                beepMs(LOCK_BEEP_MS);   // "target locked" (also stops spin momentum)
                locked = true;
            }
            confirm   = LOCK_CONFIRM;   // saturate while locked
            graceLeft = GRACE_CYCLES;
            LED1      = 0;              // solid (active-low) = locked
            if (ramp < RAMP_MAX)
            {
                ramp++;                 // soft-start accelerates while locked
            }

            uint8_t duty = (ticks < TICKS_CONTACT) ? DUTY_MAX : rampDuty[ramp];
            drive(duty, duty, true, true, CHARGE_PERIODS);  // ~PING_GAP_MS of forward
        }
        else if (present)
        {
            // ---- First sighting, not yet confirmed: hold still and re-ping to
            //      reject a single ghost echo before committing to a charge. ----
            settle(SETTLE_MS);
        }
        else if (locked && graceLeft > 0)
        {
            // ---- Brief dropout: target slipped out of the narrow beam for one
            //      ping. Keep charging straight on the last heading; don't reset
            //      the soft-start ramp. ----
            graceLeft--;
            LED1 = 0;
            uint8_t duty = rampDuty[ramp];
            drive(duty, duty, true, true, CHARGE_PERIODS);
        }
        else
        {
            // ---- No target (confirmed loss or never locked): stop-and-scan. ----
            if (locked)
            {
                beepMs(LOSS_BEEP_MS);   // chirp on confirmed loss
            }
            locked  = false;
            confirm = 0;
            ramp    = 0;                // re-acquisition restarts soft

            blink ^= 1;
            LED1 = blink ? 0 : 1;       // slow blink = searching

            // Kick a small turn, then stop & settle so the NEXT ping is taken with
            // the bot stationary at a known heading (this is what stops it from
            // sweeping past targets). PING_GAP is satisfied by SETTLE_MS.
            drive(SEARCH_KICK_DUTY, SEARCH_KICK_DUTY,
                  SEARCH_CW, !SEARCH_CW, SEARCH_KICK_PERIODS);
            settle(SETTLE_MS);
        }
    }
}
