// The QRS front panel for the kit's hardware mode (kit::PanelLayout), measured from a straight-on
// reference photo (local reference only: nothing of it is shipped, this is a vector drawing at its
// positions). 19" 2U, millimetres.
//
// Two panels behind one drawing. With the COP PROMs (card 1, U308 / U309) the keys, knob, displays and
// LEDs are the unit's controller and panel boards at gate level (qrs/core/cop_panel.hpp): its memories,
// copying, blinking, Kill and Enter included. Without them a hand-written panel follows the owner's
// manual (pp 9-14, 22-23): each parameter's [-] / [+] steps it and assigns the central knob to it;
// MEMORY recalls the factory programs (files 7 and 8, locations 2-8): "*" toggles File mode, where a
// digit picks the file (LOC back to 1), and a digit outside it the location. There copying ("=") and
// the user memories are not modelled, and ENHANCE is drawn only.
//
// Ids:
//   keys     1-16 the parameters' [-]/[+] (2p+1 = [-], 2p+2 = [+], p in Param order: 1st reflection
//            delay, level, reverb delay, level, high end, reverb time, low end, room size),
//            20 ENT, 21 MUTE, 22 FRE, 23 ENH and 24 power (drawn only: no action); 31-38 memory
//            digits 1-8, 39 "*", 40 "=" (no action)
//   LEDs     0 FREEZE, 1-60 the meters (6 bars x 10, top first: inputs L, R, outputs 1-4), 61 ENHANCE
//   digits   0-23 the eight 3-digit displays in Param order, 24 FILE, 25 LOC
//   input    1 the central knob (0..1; its movement steps the assigned parameter)
#pragma once
#include "../../shared/core/panel.hpp"

namespace qrsplug {

enum QrsPanelParam { PReflDelay, PReflLevel, PRevDelay, PRevLevel, PHigh, PTime, PLow, PRoom, NumPanelParams };
enum QrsKey { KeyEnter = 20, KeyMute = 21, KeyFreeze = 22, KeyEnhance = 23, KeyPower = 24, KeyMem1 = 31, KeyStar = 39,
              KeyEquals = 40 };
enum QrsLed { LedFreeze = 0, LedMeter0 = 1, NumMeterLeds = 10, LedEnhance = 1 + 6 * NumMeterLeds, NumQrsLeds };
enum QrsDigit { DigitFile = 24, DigitLoc = 25, NumQrsDigits = 26 };
constexpr int kKnobInput = 1;
// the meters' scale, top first (0 dB = nominal = 12 dB below full scale, manual pp 3, 35)
constexpr const char* kMeterScale[NumMeterLeds] = {"CLIP", "+12", "+6", "0", "-12", "-24", "-36", "-48", "-60", "-72"};

inline kit::PanelLayout qrs_layout() {
    using E = kit::PanelElement;
    kit::PanelLayout L;
    L.width = 483.0f;
    L.height = 88.9f;
    L.face = 0xff1d2032;   // the dark blue-black face
    L.ink = 0xffe6e6e0;
    L.leds = NumQrsLeds;
    L.digits = NumQrsDigits;
    L.inputs = {{kKnobInput, "Knob", 0.0f, 1.0f, 0.5f}};
    auto add = [&](E e) { L.elements.push_back(std::move(e)); };
    auto text = [&](float x, float y, const char* s, float size = 2.3f, int just = E::Centre) {
        E e;
        e.kind = E::Text;
        e.x = x, e.y = y, e.text = s, e.size = size, e.style = just;
        add(e);
    };
    auto hole = [&](float cx, float cy, int style) {
        E e;
        e.kind = E::Hole;
        float w = style == 0 ? 6.4f : 4.4f, h = style == 0 ? 4.4f : w;
        e.x = cx - w / 2, e.y = cy - h / 2, e.w = w, e.h = h, e.style = style;
        add(e);
    };
    auto frame = [&](float x0, float y0, float x1, float y1, const char* caption) {
        E e;
        e.kind = E::Frame;
        e.x = x0, e.y = y0, e.w = x1 - x0, e.h = y1 - y0, e.text = caption, e.size = 2.5f, e.colour = 0xffd8d8d2;
        add(e);
    };
    auto window = [&](float x0, float y0, float x1, float y1) {
        E e;
        e.kind = E::Window;
        e.x = x0, e.y = y0, e.w = x1 - x0, e.h = y1 - y0, e.colour = 0xff200608;
        add(e);
    };
    auto led = [&](float cx, float cy, int id, float d, uint32_t col) {
        E e;
        e.kind = E::Led;
        e.x = cx - d / 2, e.y = cy - d / 2, e.w = e.h = d, e.id = id, e.colour = col;
        add(e);
    };
    auto digit = [&](float cx, float cy, int id) {
        E d;
        d.kind = E::Digit;
        d.w = 4.6f, d.h = 6.8f, d.x = cx - d.w / 2, d.y = cy - d.h / 2, d.id = id, d.colour = 0xffff2a1a;
        d.size = 1.8f;
        add(d);
    };
    auto key = [&](float cx, float cy, int id, const char* legend, const char* hot, int style = E::DarkSquare,
                   float w = 14.6f, float h = 14.6f) {
        E e;
        e.kind = E::Key;
        e.w = w, e.h = h, e.x = cx - w / 2, e.y = cy - h / 2, e.id = id, e.text = legend, e.style = style, e.hotkey = hot;
        e.colour = 0xfff2f2ec;
        add(e);
    };

    // rack ears and the handle screws
    hole(4.9f, 5.4f, 0), hole(478.1f, 5.4f, 0), hole(4.9f, 83.5f, 0), hole(478.1f, 83.5f, 0);
    hole(25.4f, 13.9f, 1), hole(25.4f, 69.7f, 1), hole(447.6f, 13.9f, 1), hole(447.6f, 69.7f, 1);

    // meters: inputs L R, outputs 1-4
    frame(38.7f, 7.1f, 108.7f, 79.4f, "");
    text(53.0f, 15.4f, "INPUTS", 2.2f), text(87.1f, 15.4f, "OUTPUTS", 2.2f), text(65.6f, 19.2f, "dB", 2.0f);
    const float bx[6] = {47.9f, 57.6f, 73.7f, 83.4f, 91.9f, 100.6f};
    const char* bl[6] = {"L", "R", "1", "2", "3", "4"};
    for (int b = 0; b < 6; b++) {
        window(bx[b] - 2.6f, 21.0f, bx[b] + 2.6f, 68.0f);
        for (int i = 0; i < NumMeterLeds; i++)
            led(bx[b], 24.0f + 4.6f * i, LedMeter0 + b * NumMeterLeds + i, 2.4f, 0xffff3020);
        text(bx[b], 72.6f, bl[b], 2.0f);
    }
    for (int i = 0; i < NumMeterLeds; i++) text(65.6f, 24.8f + 4.6f * i, kMeterScale[i], 1.6f);

    // effects
    frame(116.0f, 6.4f, 141.8f, 79.4f, "EFFECTS");
    // small legends, as on the unit; the FREEZE and ENHANCE LEDs sit in their keys' top-left corners
    key(127.4f, 19.3f, KeyEnter, "ENT", "e"), L.elements.back().size = 3.2f;
    key(127.4f, 35.4f, KeyMute, "MUTE", "m"), L.elements.back().size = 3.2f;
    key(127.4f, 51.1f, KeyFreeze, "FRE", "f"), L.elements.back().size = 3.2f;
    key(127.4f, 66.5f, KeyEnhance, "ENH", ""), L.elements.back().size = 3.2f;
    led(122.4f, 46.3f, LedFreeze, 1.6f, 0xffff3020);
    led(122.4f, 61.7f, LedEnhance, 1.6f, 0xffff3020);

    // parameters
    frame(147.8f, 6.4f, 336.5f, 79.4f, "PARAMETERS");
    frame(150.6f, 9.8f, 219.6f, 43.4f, "1st REFLECTION");
    frame(150.6f, 43.4f, 219.6f, 78.2f, "");
    frame(263.2f, 9.8f, 334.4f, 58.4f, "REVERB TIME");
    frame(263.2f, 58.4f, 334.4f, 78.2f, "");
    text(200.5f, 76.0f, "REVERB", 2.1f);
    text(282.1f, 76.0f, "ROOM SIZE", 2.1f);
    const float row[4] = {19.3f, 35.4f, 51.1f, 66.5f};
    const float win[4] = {17.6f, 33.7f, 49.4f, 65.0f};
    // left: 1st reflection delay, level; reverb delay, level
    const char* lunit[4] = {"ms", "db", "ms", "db"};
    const char* llab[4] = {"DELAY", "LEVEL", "DELAY", "LEVEL"};
    for (int r = 0; r < 4; r++) {
        key(159.0f, row[r], 1 + 2 * r, "-", "");
        key(174.7f, row[r], 2 + 2 * r, "+", "");
        window(184.4f, win[r] - 4.6f, 213.4f, win[r] + 4.6f);
        text(189.0f, win[r] + 0.8f, lunit[r], 1.7f);
        for (int d = 0; d < 3; d++) digit(196.6f + 7.0f * d, win[r], 3 * r + d);
        text(200.5f, win[r] + 6.7f, llab[r], 1.8f);
    }
    // right: high end, reverb time, low end, room size
    const char* runit[4] = {"x", "SEC", "x", "m3"};
    const char* rlab[4] = {"HIGH END", "", "LOW END", ""};
    for (int r = 0; r < 4; r++) {
        window(267.1f, win[r] - 4.6f, 297.8f, win[r] + 4.6f);
        for (int d = 0; d < 3; d++) digit(273.5f + 7.0f * d, win[r], 12 + 3 * r + d);
        text(294.6f, win[r] + 0.8f, runit[r], 1.7f);
        if (*rlab[r]) text(282.1f, win[r] + 6.7f, rlab[r], 1.8f);
        key(309.4f, row[r], 9 + 2 * r, "-", "");
        key(325.5f, row[r], 10 + 2 * r, "+", "");
    }
    // the central knob, assigned to the last parameter touched
    {
        E plate;
        plate.kind = E::Window;
        plate.x = 224.1f, plate.y = 12.4f, plate.w = 35.5f, plate.h = 58.4f, plate.colour = 0xff3a3d48;
        add(plate);
        text(229.0f, 29.6f, "-", 3.0f), text(254.0f, 29.6f, "+", 3.0f);
        E k;
        k.kind = E::Pot;
        k.x = 241.7f - 11.8f, k.y = 41.8f - 11.8f, k.w = k.h = 23.6f, k.id = kKnobInput, k.style = E::Dial300;
        add(k);
    }

    // memory
    frame(344.4f, 6.4f, 400.3f, 79.4f, "MEMORY");
    window(352.0f, 13.6f, 360.4f, 23.0f);
    digit(356.2f, 18.3f, DigitFile);
    text(356.2f, 26.0f, "FILE", 1.8f);
    window(352.0f, 44.7f, 360.4f, 54.1f);
    digit(356.2f, 49.4f, DigitLoc);
    text(356.2f, 57.1f, "LOC", 1.8f);
    key(356.2f, row[1], KeyStar, "*", "*", E::DarkSquare);
    key(356.2f, row[3], KeyEquals, "=", "=", E::DarkSquare);
    const int dig[4][2] = {{4, 5}, {3, 6}, {2, 7}, {1, 8}};
    static const char* hk[9] = {"", "1", "2", "3", "4", "5", "6", "7", "8"};
    static const char* lg[9] = {"", "1", "2", "3", "4", "5", "6", "7", "8"};
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 2; c++) {
            const int n = dig[r][c];
            key(c == 0 ? 373.4f : 389.1f, row[r], KeyMem1 + n - 1, lg[n], hk[n]);
        }

    // power (drawn only)
    key(428.2f, 52.6f, KeyPower, "O", "", E::DarkSquare, 15.0f, 19.0f);
    L.elements.back().colour = 0xffffd0d0;

    L.notes = "Each parameter's - and + step it and assign the central knob to it. ENT opens the input while held "
              "in FREEZE; MUTE clears the reverb memory. MEMORY: * toggles File mode, where a digit picks the file; "
              "outside it a digit recalls that location (files 7 and 8 hold the factory programs, 2-8). Location 1 "
              "is the one you edit. With the COP ROMs this is the unit's own controller and panel, memories and "
              "copying (=) included; without them, a panel written from the manual (no copying or user memories, "
              "ENHANCE drawn only). The meters are the plugin's own (0 dB = 12 dB below converter full scale, as on "
              "the unit).";
    return L;
}

}  // namespace qrsplug
