#pragma once

#include <M5Cardputer.h>
#include <ctype.h>

namespace cardputer_keyboard {

using KeysState = Keyboard_Class::KeysState;

constexpr uint8_t HID_1         = 0x1E;
constexpr uint8_t HID_0         = 0x27;
constexpr uint8_t HID_MINUS     = 0x2D;
constexpr uint8_t HID_EQUALS    = 0x2E;
constexpr uint8_t HID_BACKSPACE = 0x2A;
constexpr uint8_t HID_COMMA     = 0x36;
constexpr uint8_t HID_PERIOD    = 0x37;
constexpr uint8_t HID_SEMICOLON = 0x33;
constexpr uint8_t HID_SLASH     = 0x38;
constexpr uint8_t HID_BACKTICK  = 0x35;

inline char to_lower(char c) {
    return static_cast<char>(tolower(static_cast<unsigned char>(c)));
}

inline bool has_word(const KeysState& s, char c) {
    for (char v : s.word) if (v == c) return true;
    return false;
}

inline bool has_word_ci(const KeysState& s, char c) {
    char want = to_lower(c);
    for (char v : s.word) {
        if (to_lower(v) == want) return true;
    }
    return false;
}

inline bool has_hid(const KeysState& s, uint8_t hid) {
    for (uint8_t v : s.hid_keys) if (v == hid) return true;
    return false;
}

inline bool pressed_word_ci(const KeysState& cur, const KeysState& prev, char c) {
    return has_word_ci(cur, c) && !has_word_ci(prev, c);
}

inline bool pressed_hid(const KeysState& cur, const KeysState& prev, uint8_t hid) {
    return has_hid(cur, hid) && !has_hid(prev, hid);
}

inline bool pressed_fn_hid(const KeysState& cur, const KeysState& prev, uint8_t hid) {
    bool active_now  = cur.fn  && has_hid(cur, hid);
    bool active_prev = prev.fn && has_hid(prev, hid);
    return active_now && !active_prev;
}

inline bool pressed_digit(const KeysState& cur, const KeysState& prev, char digit) {
    if (digit >= '1' && digit <= '9') {
        return pressed_hid(cur, prev, static_cast<uint8_t>(HID_1 + (digit - '1')));
    }
    if (digit == '0') {
        return pressed_hid(cur, prev, HID_0);
    }
    return false;
}

inline bool pressed_enter(const KeysState& cur, const KeysState& prev) {
    return cur.enter && !prev.enter;
}

inline bool pressed_space(const KeysState& cur, const KeysState& prev) {
    return cur.space && !prev.space;
}

inline bool pressed_backspace(const KeysState& cur, const KeysState& prev) {
    return cur.del && !cur.fn && !prev.del;
}

inline bool pressed_escape(const KeysState& cur, const KeysState& prev) {
    return pressed_fn_hid(cur, prev, HID_BACKTICK);
}

inline bool pressed_arrow_up(const KeysState& cur, const KeysState& prev) {
    return pressed_fn_hid(cur, prev, HID_SEMICOLON);
}

inline bool pressed_arrow_left(const KeysState& cur, const KeysState& prev) {
    return pressed_fn_hid(cur, prev, HID_COMMA);
}

inline bool pressed_arrow_down(const KeysState& cur, const KeysState& prev) {
    return pressed_fn_hid(cur, prev, HID_PERIOD);
}

inline bool pressed_arrow_right(const KeysState& cur, const KeysState& prev) {
    return pressed_fn_hid(cur, prev, HID_SLASH);
}

inline bool pressed_forward_delete(const KeysState& cur, const KeysState& prev) {
    return pressed_fn_hid(cur, prev, HID_BACKSPACE);
}

inline bool pressed_nav_up(const KeysState& cur, const KeysState& prev) {
    return pressed_hid(cur, prev, HID_SEMICOLON) || pressed_arrow_up(cur, prev);
}

inline bool pressed_nav_left(const KeysState& cur, const KeysState& prev) {
    return pressed_hid(cur, prev, HID_COMMA) || pressed_arrow_left(cur, prev);
}

inline bool pressed_nav_down(const KeysState& cur, const KeysState& prev) {
    return pressed_hid(cur, prev, HID_PERIOD) || pressed_arrow_down(cur, prev);
}

inline bool pressed_nav_right(const KeysState& cur, const KeysState& prev) {
    return pressed_hid(cur, prev, HID_SLASH) || pressed_arrow_right(cur, prev);
}

}  // namespace cardputer_keyboard
