#include QMK_KEYBOARD_H
#include "version.h"
#define MOON_LED_LEVEL LED_LEVEL
#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
  ST_MACRO_0,
  ST_MACRO_1,
  ST_MACRO_2,
  ST_MACRO_3,
  ST_MACRO_4,
  ST_MACRO_5,
  ST_MACRO_6,
  ST_MACRO_7,
  ST_MACRO_8,
  ST_MACRO_9,
  ST_MACRO_10,
  ST_MACRO_11,
  ST_MACRO_12,
  ST_MACRO_13,
  ST_MACRO_14,
  ST_MACRO_15,
  ST_MACRO_16,
  ST_MACRO_17,
  ST_MACRO_18,
  ST_MACRO_19,
  ST_MACRO_20,
  ST_MACRO_21,
  ST_MACRO_22,
  ST_MACRO_23,
  ST_MACRO_24,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
};


const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    TD(DANCE_0),    TO(3),          TD(DANCE_1),    RGUI(RSFT(KC_C)),RALT(RGUI(KC_J)),RALT(RGUI(KC_5)),                                KC_LEFT,        KC_RIGHT,       KC_MEDIA_PLAY_PAUSE,KC_AUDIO_MUTE,  ST_MACRO_0,     ST_MACRO_1,     
    TD(DANCE_2),    KC_Q,           KC_W,           KC_F,           KC_P,           KC_B,                                           KC_J,           KC_L,           KC_U,           KC_Y,           KC_QUOTE,       KC_COLN,        
    ALL_T(KC_ESCAPE),MT(MOD_LCTL, KC_A),MT(MOD_LALT, KC_R),MT(MOD_LGUI, KC_S),MT(MOD_LSFT, KC_T),KC_G,                                           KC_M,           MT(MOD_RSFT, KC_N),MT(MOD_RGUI, KC_E),LT(4, KC_I),    MT(MOD_RCTL, KC_O),ALL_T(KC_EQUAL),
    MEH_T(KC_GRAVE),LT(2, KC_Z),    KC_X,           KC_C,           KC_D,           KC_V,                                           KC_K,           KC_H,           KC_COMMA,       KC_DOT,         LT(5, KC_SLASH),MEH_T(KC_MINUS),
                                                    LT(1, KC_SPACE),KC_SPACE,                                       KC_ENTER,       LT(3, KC_BSPC)
  ),
  [1] = LAYOUT_voyager(
    QK_BOOT,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_LLCK,                                        KC_TRANSPARENT, KC_TRANSPARENT, ST_MACRO_2,     LGUI(KC_MINUS), RGUI(KC_0),     LGUI(KC_EQUAL), 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 RGUI(RSFT(KC_E)),ST_MACRO_3,     ST_MACRO_4,     ST_MACRO_5,     ST_MACRO_6,     LCTL(LSFT(KC_SPACE)),
    KC_TRANSPARENT, OSM(MOD_LCTL),  OSM(MOD_LALT),  OSM(MOD_LGUI),  OSM(MOD_LSFT),  KC_TRANSPARENT,                                 RGUI(RSFT(KC_F)),KC_LEFT,        KC_DOWN,        KC_UP,          KC_RIGHT,       CW_TOGG,        
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 RCTL(RSFT(KC_G)),KC_HOME,        KC_PGDN,        KC_PAGE_UP,     KC_END,         KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_DELETE
  ),
  [2] = LAYOUT_voyager(
    QK_BOOT,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_LLCK,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, RGB_MODE_FORWARD,RGB_HUI,        KC_BRIGHTNESS_UP,RGB_VAI,        KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_MEDIA_NEXT_TRACK,KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_MEDIA_STOP,  RGB_TOG,        RGB_HUD,        KC_BRIGHTNESS_DOWN,RGB_VAD,        KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_AUDIO_MUTE,  KC_MEDIA_PLAY_PAUSE
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT, TO(0),          KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 QK_LLCK,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_BOOT,        
    KC_TRANSPARENT, KC_LBRC,        KC_7,           KC_8,           KC_9,           KC_RBRC,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_SCLN,        KC_4,           KC_5,           KC_6,           KC_EQUAL,                                       KC_TRANSPARENT, OSM(MOD_RSFT),  OSM(MOD_RGUI),  OSM(MOD_RALT),  OSM(MOD_RCTL),  KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_GRAVE,       KC_1,           KC_2,           KC_3,           KC_BSLS,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_0,           KC_MINUS,                                       KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 QK_LLCK,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, QK_BOOT,        
    KC_TRANSPARENT, KC_LCBR,        KC_AMPR,        KC_ASTR,        KC_LPRN,        KC_RCBR,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_COLN,        KC_DLR,         KC_PERC,        KC_CIRC,        KC_PLUS,                                        KC_TRANSPARENT, OSM(MOD_RSFT),  OSM(MOD_RGUI),  KC_TRANSPARENT, OSM(MOD_RCTL),  KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TILD,        KC_EXLM,        KC_AT,          KC_HASH,        KC_PIPE,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_RPRN,        KC_UNDS,                                        KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [5] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, ST_MACRO_7,     ST_MACRO_8,     ST_MACRO_9,     ST_MACRO_10,    ST_MACRO_11,                                    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, ST_MACRO_12,    ST_MACRO_13,    ST_MACRO_14,    ST_MACRO_15,    ST_MACRO_16,                                    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, ST_MACRO_17,    ST_MACRO_18,    ST_MACRO_19,    ST_MACRO_20,    ST_MACRO_21,                                    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    ST_MACRO_22,    ST_MACRO_23,                                    KC_TRANSPARENT, KC_TRANSPARENT
  ),
};


const uint16_t PROGMEM combo0[] = { KC_Q, MT(MOD_LCTL, KC_A), COMBO_END};
const uint16_t PROGMEM combo1[] = { KC_J, KC_M, COMBO_END};
const uint16_t PROGMEM combo2[] = { KC_F, KC_P, COMBO_END};
const uint16_t PROGMEM combo3[] = { MT(MOD_LSFT, KC_T), KC_D, COMBO_END};
const uint16_t PROGMEM combo4[] = { KC_W, MT(MOD_LALT, KC_R), COMBO_END};
const uint16_t PROGMEM combo5[] = { MT(MOD_LALT, KC_R), KC_X, COMBO_END};

combo_t key_combos[COMBO_COUNT] = {
    COMBO(combo0, ST_MACRO_24),
    COMBO(combo1, RGUI(RSFT(KC_SPACE))),
    COMBO(combo2, RALT(RGUI(KC_V))),
    COMBO(combo3, RGUI(KC_V)),
    COMBO(combo4, RGUI(KC_C)),
    COMBO(combo5, RGUI(KC_X)),
};

uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case TD(DANCE_2):
            return TAPPING_TERM -20;
        case ALL_T(KC_ESCAPE):
            return TAPPING_TERM -20;
        case MT(MOD_LSFT, KC_T):
            return TAPPING_TERM -20;
        case LT(1, KC_SPACE):
            return TAPPING_TERM -20;
        case KC_COLN:
            return TAPPING_TERM -20;
        case MT(MOD_RSFT, KC_N):
            return TAPPING_TERM -20;
        case LT(3, KC_BSPC):
            return TAPPING_TERM -20;
        default:
            return TAPPING_TERM;
    }
}





typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,      
    SINGLE_HOLD,         
    DOUBLE_TAP,          
    DOUBLE_HOLD,         
    DOUBLE_SINGLE_TAP,   
    MORE_TAPS            
};

static tap dance_state[3];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(KC_SPACE));
        tap_code16(LALT(KC_SPACE));
        tap_code16(LALT(KC_SPACE));
    }
    if(state->count > 3) {
        tap_code16(LALT(KC_SPACE));
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(LALT(KC_SPACE)); break;
        case DOUBLE_TAP: register_code16(KC_ESCAPE); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(KC_SPACE)); register_code16(LALT(KC_SPACE));
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(LALT(KC_SPACE)); break;
        case DOUBLE_TAP: unregister_code16(KC_ESCAPE); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(KC_SPACE)); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(LALT(LCTL(LSFT(KC_BSPC))));
        tap_code16(LALT(LCTL(LSFT(KC_BSPC))));
        tap_code16(LALT(LCTL(LSFT(KC_BSPC))));
    }
    if(state->count > 3) {
        tap_code16(LALT(LCTL(LSFT(KC_BSPC))));
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(LALT(LCTL(LSFT(KC_BSPC)))); break;
        case DOUBLE_TAP: register_code16(KC_WWW_REFRESH); break;
        case DOUBLE_SINGLE_TAP: tap_code16(LALT(LCTL(LSFT(KC_BSPC)))); register_code16(LALT(LCTL(LSFT(KC_BSPC))));
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(LALT(LCTL(LSFT(KC_BSPC)))); break;
        case DOUBLE_TAP: unregister_code16(KC_WWW_REFRESH); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(LALT(LCTL(LSFT(KC_BSPC)))); break;
    }
    dance_state[1].step = 0;
}
void on_dance_2(tap_dance_state_t *state, void *user_data);
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void on_dance_2(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
        tap_code16(KC_TAB);
    }
    if(state->count > 3) {
        tap_code16(KC_TAB);
    }
}

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case SINGLE_TAP: register_code16(KC_TAB); break;
        case DOUBLE_TAP: register_code16(LALT(LSFT(KC_SPACE))); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_TAB); register_code16(KC_TAB);
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case SINGLE_TAP: unregister_code16(KC_TAB); break;
        case DOUBLE_TAP: unregister_code16(LALT(LSFT(KC_SPACE))); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_TAB); break;
    }
    dance_state[2].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_2, dance_2_finished, dance_2_reset),
};

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_LALT(SS_LCTL(SS_LGUI(SS_LSFT(SS_TAP(X_T)))))SS_DELAY(100)  SS_LGUI(SS_TAP(X_O))SS_DELAY(100)  SS_LALT(SS_LCTL(SS_LGUI(SS_LSFT(SS_TAP(X_O))))));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_LALT(SS_LCTL(SS_LGUI(SS_LSFT(SS_TAP(X_T)))))SS_DELAY(100)  SS_RGUI(SS_TAP(X_S)));
    }
    break;
    case ST_MACRO_2:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_A));
    }
    break;
    case ST_MACRO_3:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_LBRC)SS_DELAY(100)  SS_TAP(X_C));
    }
    break;
    case ST_MACRO_4:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_LBRC)SS_DELAY(100)  SS_TAP(X_D));
    }
    break;
    case ST_MACRO_5:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_RBRC)SS_DELAY(100)  SS_TAP(X_D));
    }
    break;
    case ST_MACRO_6:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_SPACE)SS_DELAY(100)  SS_TAP(X_H)SS_DELAY(100)  SS_TAP(X_P));
    }
    break;
    case ST_MACRO_7:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_LBRC));
    }
    break;
    case ST_MACRO_8:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_7));
    }
    break;
    case ST_MACRO_9:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_8));
    }
    break;
    case ST_MACRO_10:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_9));
    }
    break;
    case ST_MACRO_11:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_F)SS_DELAY(100)  SS_TAP(X_I)SS_DELAY(100)  SS_TAP(X_L)SS_DELAY(100)  SS_TAP(X_I)SS_DELAY(100)  SS_TAP(X_P));
    }
    break;
    case ST_MACRO_12:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_O));
    }
    break;
    case ST_MACRO_13:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_4));
    }
    break;
    case ST_MACRO_14:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_5));
    }
    break;
    case ST_MACRO_15:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_6));
    }
    break;
    case ST_MACRO_16:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_F));
    }
    break;
    case ST_MACRO_17:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_6)SS_DELAY(100)  SS_TAP(X_0)SS_DELAY(100)  SS_TAP(X_4)SS_DELAY(100)  SS_TAP(X_7)SS_DELAY(100)  SS_TAP(X_7));
    }
    break;
    case ST_MACRO_18:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_KP_PLUS)SS_DELAY(100)  SS_TAP(X_4)SS_DELAY(100)  SS_TAP(X_2)SS_DELAY(100)  SS_TAP(X_0));
    }
    break;
    case ST_MACRO_19:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_2));
    }
    break;
    case ST_MACRO_20:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_3));
    }
    break;
    case ST_MACRO_21:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_R));
    }
    break;
    case ST_MACRO_22:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_0));
    }
    break;
    case ST_MACRO_23:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_T));
    }
    break;
    case ST_MACRO_24:
    if (record->event.pressed) {
      SEND_STRING(SS_LGUI(SS_TAP(X_A))SS_DELAY(100)  SS_RGUI(SS_TAP(X_C)));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}
