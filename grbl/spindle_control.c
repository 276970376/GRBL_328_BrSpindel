/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


void spindle_init()
{
  #ifdef VARIABLE_SPINDLE

    // Configure variable spindle PWM and enable pin, if requried. On the Uno, PWM and enable are
    // combined unless configured otherwise.
    SPINDLE_PWM_DDR |= (1<<SPINDLE_PWM_BIT); // Configure as PWM output pin.
    SPINDLE_TCCRA_REGISTER = SPINDLE_TCCRA_INIT_MASK; // Configure PWM output compare timer
    SPINDLE_TCCRB_REGISTER = SPINDLE_TCCRB_INIT_MASK;
      #ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
      SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    #else
      SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.
    #endif
  // BlueOrangeLive
      #ifdef BRUSHLESS_SPINDEL
        SPINDLE_OCR_REGISTER = OUT_BRSP_Steht; // Set PWM pin output auf 0,9 - 1 ms. The ESC need this as minium Signal.
        SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
      #endif
  // BlueOrangeLive
  #else

    // Configure no variable spindle and only enable pin.
    SPINDLE_ENABLE_DDR |= (1<<SPINDLE_ENABLE_BIT); // Configure as output pin.
    SPINDLE_DIRECTION_DDR |= (1<<SPINDLE_DIRECTION_BIT); // Configure as output pin.

  #endif

  spindle_stop();
}


void spindle_stop()
{
  // On the Uno, spindle enable and PWM are shared. Other CPUs have seperate enable pin.
  #ifdef VARIABLE_SPINDLE
// Begin BlueOrangeLive
      #ifdef BRUSHLESS_SPINDEL
        SPINDLE_OCR_REGISTER = OUT_BRSP_Steht; // Set PWM pin output auf 0,9 - 1 ms. The ESC need this as minium Signal.
        SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.
      #else
        SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
        SPINDLE_TCCRA_REGISTER &= ~(1<<SPINDLE_COMB_BIT); // Disable PWM. Output voltage is zero.
      #endif
  // Ende BlueOrangeLive
    #if defined(CPU_MAP_ATMEGA2560) || defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
      #ifdef INVERT_SPINDLE_ENABLE_PIN
        SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
      #else
        SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
      #endif
    #endif
  #else
    #ifdef INVERT_SPINDLE_ENABLE_PIN
      SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);  // Set pin to high
    #else
      SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT); // Set pin to low
    #endif
  #endif
}

void spindle_set_state(uint8_t state, float rpm)
{
  if (sys.abort) { return; } // Block during abort.

  // Halt or set spindle direction and rpm.
  if (state == SPINDLE_DISABLE) {

    spindle_stop();

  } else {

    #ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
      if (state == SPINDLE_ENABLE_CW) {
        SPINDLE_DIRECTION_PORT &= ~(1<<SPINDLE_DIRECTION_BIT);
      } else {
        SPINDLE_DIRECTION_PORT |= (1<<SPINDLE_DIRECTION_BIT);
      }
    #endif

    #ifdef VARIABLE_SPINDLE

      // TODO: Install the optional capability for frequency-based output for servos.
      // Test BlueOrangeLive ESC-Servos Brushless Spindel Motor Drivers 
      uint8_t current_pwm;  // 328p PWM register is 8-bit.
      // Calculate PWM register value based on rpm max/min settings and programmed rpm.
      if (rpm <= 0.0) { spindle_stop(); } // RPM should never be negative, but check anyway.
      else
      {
    // Begin BlueOrangeLive
      #ifdef BRUSHLESS_SPINDEL
        if (rpm > settings.rpm_max) { rpm = settings.rpm_max; }
        if (rpm < settings.rpm_min) { rpm = settings.rpm_min; }
        if (rpm > IN_BRSP_START)
        {
           int in_brsp_range = IN_BRSP_END - IN_BRSP_START;
           int out_brsp_range = OUT_BRSP_END - OUT_BRSP_START;
           current_pwm= (rpm - IN_BRSP_START)*out_brsp_range / in_brsp_range + OUT_BRSP_START;
           if (current_pwm >= (OUT_BRSP_END)) {current_pwm = (OUT_BRSP_END-1); }  // unter 24000 bleiben bzw unter max
         }
         else
          {
            if (rpm > 1) {current_pwm = OUT_BRSP_START;}
            else
              {current_pwm = OUT_BRSP_Steht;}  // ungefähr 0,9 - 1 milliseconds
           }
  #else
    //  Ende  BlueOrangeLive
        if (settings.rpm_max <= settings.rpm_min) { current_pwm = SPINDLE_PWM_MAX_VALUE; } // No PWM range possible. Set simple on/off spindle control pin state.
        else
        {  if (rpm > settings.rpm_max) { rpm = settings.rpm_max; }
           if (rpm < settings.rpm_min) { rpm = settings.rpm_min; }
           #ifdef SPINDLE_MINIMUM_PWM
              float pwm_gradient = (SPINDLE_PWM_MAX_VALUE-SPINDLE_MINIMUM_PWM)/(settings.rpm_max-settings.rpm_min);
              current_pwm = floor( (rpm-settings.rpm_min)*pwm_gradient + (SPINDLE_MINIMUM_PWM+0.5));
            #else
              float pwm_gradient = (SPINDLE_PWM_MAX_VALUE)/(settings.rpm_max-settings.rpm_min);
              current_pwm = floor( (rpm-settings.rpm_min)*pwm_gradient + 0.5);
            #endif
         }
      #endif // hjh
        SPINDLE_OCR_REGISTER = current_pwm; // Set PWM output level.
        SPINDLE_TCCRA_REGISTER |= (1<<SPINDLE_COMB_BIT); // Ensure PWM output is enabled.

        // On the Uno, spindle enable and PWM are shared, unless otherwise specified.
        #if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN)
          #ifdef INVERT_SPINDLE_ENABLE_PIN
            SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
          #else
            SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
          #endif
        #endif
      }

    #else

     // NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
     // if the spindle speed value is zero, as its ignored anyhow.
     #ifdef INVERT_SPINDLE_ENABLE_PIN
       SPINDLE_ENABLE_PORT &= ~(1<<SPINDLE_ENABLE_BIT);
     #else
       SPINDLE_ENABLE_PORT |= (1<<SPINDLE_ENABLE_BIT);
     #endif

    #endif

  }
}


void spindle_run(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
  spindle_set_state(state, rpm);
}
