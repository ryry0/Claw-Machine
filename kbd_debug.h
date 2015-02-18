//This file contains the functions used for reading the keyboard
#ifndef KBD_DEBUG_H_
#define KBD_DEBUG_H_
void readKeyboardDebug(bool &touchpad_enabled) {
  char incoming_byte;
  if (Serial.available()) {
    incoming_byte = Serial.read();
    switch(incoming_byte) {
      case 'l': //move r motor on shelf towards
        motors[MOTOR_X].command_position += 200;
        break;

      case 'o': //move r motor on shelf away
        motors[MOTOR_X].command_position -= 200;
        break;

      case 'i': //move l motor on shelf away
        motors[MOTOR_X2].command_position -= 200;
        break;

      case 'k': //move l motor on shelf towards
        motors[MOTOR_X2].command_position += 200;
        break;

      case 'R':
        game_state = WAIT_FOR_START;
        break;

      case 'P':
        game_state = PLAY;
        break;

      case 'S': //set position and restart the game
        game_state = WAIT_FOR_START;
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_position = 0;
          motors[i].command_velocity = 0;
          encoder_counts[i] = 0;
        }
        break;

      case 's': //move system towards
        motors[MOTOR_X].command_velocity += 800;
        motors[MOTOR_X2].command_velocity += 800;
        break;

      case 'w': //move system away
        motors[MOTOR_X].command_velocity -= 800;
        motors[MOTOR_X2].command_velocity -= 800;
        break;

      case 'a': //move system left
        motors[MOTOR_Y].command_velocity -= 800;
        break;

      case 'd': //move system right
        motors[MOTOR_Y].command_velocity += 800;
        break;

      case '<': //move cart left
        motors[MOTOR_Y].command_position -= 200;
        break;

      case '>': //move cart right
        motors[MOTOR_Y].command_position += 200;
        break;

      case 'j': //release claw
        motors[MOTOR_WINCH].command_velocity += 3200;
        break;

      case 'u': //reel in claw
        motors[MOTOR_WINCH].command_velocity -= 3200;
        break;

      case 'h': //close claw
        analogWrite(CLAW_MOTOR_PIN1, 255);
        analogWrite(CLAW_MOTOR_PIN2,  0);
        break;

      case 'y': //open claw
        analogWrite(CLAW_MOTOR_PIN2, 255);
        analogWrite(CLAW_MOTOR_PIN1,  0);
        break;

      case ' ': //reset encoder data
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_position = 0;
          motors[i].command_velocity = 0;
          encoder_counts[i] = 0;
        }
        break;

      case '.':
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_position = encoder_counts[i];
          motors[i].command_velocity = 0;
        }
        break;

      case 't':
        touchpad_enabled = !touchpad_enabled;
        break;

      case 'r': //relax claw
        analogWrite(CLAW_MOTOR_PIN2, 0);
        analogWrite(CLAW_MOTOR_PIN1, 0);
        break;
    } //end switch
  } //end if
} //end readKeyboardDebug

void readKeyboardInput() {
  char incoming_byte;
  if (Serial.available()) {
    incoming_byte = Serial.read();
    switch(incoming_byte) {
      case 'w':
        motors[MOTOR_X].command_velocity -= 800;
        motors[MOTOR_X2].command_velocity -= 800;
        break;

      case 's':
        motors[MOTOR_X].command_velocity += 800;
        motors[MOTOR_X2].command_velocity += 800;
        break;

      case 'a':
        motors[MOTOR_Y].command_velocity -= 800;
        break;

      case 'd':
        motors[MOTOR_Y].command_velocity += 800;
        break;

      case 'u':
        motors[MOTOR_WINCH].command_velocity += 3200;
        break;

      case 'j':
        motors[MOTOR_WINCH].command_velocity -= 3200;
        break;

      case 'D':
        game_state = DEBUG;
        for (int i = 0; i < NUM_MOTORS; ++i) {
          motors[i].command_velocity = 0;
        }
        break;

      case 'G':
        game_state = RETRIEVE_PRIZE;
        break;

      case 'R':
        game_state = RETURN_HOME;
        break;
    } //end switch
  } //end if
} //end readKeyboardInput


#endif
