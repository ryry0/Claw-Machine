//This file contains the functions used for reading the keyboard
#ifndef KBD_DEBUG_H_
#define KBD_DEBUG_H_
void readKeyboardDebug(bool &touchpad_enabled) {
  char incoming_byte;
  if (Serial.available()) {
    incoming_byte = Serial.read();
    switch(incoming_byte) {
      case 'w':
        motors[MOTOR_X].command_position += 200;
        break;

      case 's':
        motors[MOTOR_X].command_position -= 200;
        break;

      case 'a':
        motors[MOTOR_X2].command_position -= 200;
        break;

      case 'q':
        motors[MOTOR_X2].command_position += 200;
        break;

      case 'r':
        game_state = WAIT_FOR_START;
        break;

      case 'p':
        game_state = PLAY;
        break;

      case 'y':
        motors[MOTOR_X].command_velocity += 200;
        motors[MOTOR_X2].command_velocity += 200;
        break;

      case 'h':
        motors[MOTOR_X].command_velocity += 200;
        motors[MOTOR_X2].command_velocity += 200;
        break;

      case 'k':
        motors[MOTOR_X].command_velocity += 800;
        motors[MOTOR_X2].command_velocity += 800;
        break;

      case 'i':
        motors[MOTOR_X].command_velocity -= 800;
        motors[MOTOR_X2].command_velocity -= 800;
        break;

      case 'j':
        motors[MOTOR_Y].command_velocity -= 800;
        break;

      case 'l':
        motors[MOTOR_Y].command_velocity += 800;
        break;

      case '>':
        motors[MOTOR_Y].command_position -= 200;
        break;

      case '<':
        motors[MOTOR_Y].command_position += 200;
        break;

      case 'D':
        motors[MOTOR_WINCH].command_velocity += 3200;
        break;

      case 'U':
        motors[MOTOR_WINCH].command_velocity -= 3200;
        break;

      case 'C':
        analogWrite(CLAW_MOTOR_PIN1, 255);
        analogWrite(CLAW_MOTOR_PIN2,  0);
        break;

      case 'O':
        analogWrite(CLAW_MOTOR_PIN2, 255);
        analogWrite(CLAW_MOTOR_PIN1,  0);
        break;

      case ' ':
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

      case 'T':
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
      case 'i':
        motors[MOTOR_X].command_velocity -= 800;
        motors[MOTOR_X2].command_velocity -= 800;
        break;

      case 'k':
        motors[MOTOR_X].command_velocity += 800;
        motors[MOTOR_X2].command_velocity += 800;
        break;

      case 'j':
        motors[MOTOR_Y].command_velocity -= 800;
        break;

      case 'l':
        motors[MOTOR_Y].command_velocity += 800;
        break;

      case 'w':
        motors[MOTOR_WINCH].command_velocity += 3200;
        break;

      case 's':
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
