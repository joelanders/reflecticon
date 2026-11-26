// The Quantec QRS in real time: a C++ port of the project's Python model (emu/model/qrs_model.py,
// converters.py, render.py), header only, no allocation after construction.
//
//   80 kHz in -> AD front end (kAdSos, kAdShFir) -> every 4th sample: the AD law (sign-magnitude,
//   12-bit mantissa, x1/8 range) -> DY (pre-delay and 1st reflection lines) -> SIP <-> MY (16 comb
//   groups, bit-exact SIP) -> DIS (Walsh sum, fold-back) -> four DAC words -> the DA law -> a
//   zero-order hold per output (OUT n loaded n quarter-frames after the latch) -> kDaSos -> 80 kHz out.
//
// Bit-exact with the Python model from ADC words to DAC words (tools: cpp/run.cpp + emu/cmp_cpp.py).
// The converters follow converters.py: the DA path is the same filter at the same rate; the AD front
// end is the bilinear transform of the same network plus an FIR fit of the switched S/H (within
// 0.33 dB of the exact response below 9 kHz, see qrs_consts.hpp).
//
// ROMs (2716, 2048 bytes each), supplied by the user: MY U426 (room increments), SIP U508 (coefficient
// tables; the dump has seven rotted bytes, repaired by the caller), DY U633 (level table at 0x300).
#pragma once
#include "qrs_consts.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace qrs {

constexpr double FS = 20000.0;
constexpr int UP = 4;                       // the converters run at 4 x FS
constexpr double FS4 = FS * UP;
constexpr int RT_CEILING[7] = {40, 46, 54, 60, 66, 74, 80};   // COP word-6 limits per room (r units)
constexpr int RT_FLOOR[7] = {20, 20, 20, 20, 26, 34, 40};

inline int64_t sx(int64_t v, int bits) {
    const uint64_t m = (uint64_t(1) << bits) - 1;
    uint64_t u = uint64_t(v) & m;
    return (u >> (bits - 1)) ? int64_t(u) - int64_t(m) - 1 : int64_t(u);
}
inline int64_t tc(uint64_t v, int bits) { return sx(int64_t(v), bits); }
inline uint8_t rev8(uint8_t b) {
    uint8_t r = 0;
    for (int i = 0; i < 8; i++) r = uint8_t(r | (((b >> i) & 1) << (7 - i)));
    return r;
}

struct Roms {
    std::array<uint8_t, 2048> my{}, sip{}, dy{};
};

// Board settings, as the COP sends them (qrs_model.record_settings).
struct Settings {
    int room = 3;          // 0..6 = 1 m3 .. 10^6 m3
    int m = 0;             // SIP index (r - ceiling[room]) mod 128
    int h = 0, l = 0;      // 4-bit two's complement offsets
    int rev_delay = 19;    // DY register: ms - 1 (0..199)
    int rev_level = 31;    // 0 = OFF, 31 = 0 dB
    int refl_delay = 9;
    int refl_level = 31;
    bool freeze = false;   // Gain=1: lossless, flat hold
    bool input_on = true;  // the input gate (off in FREEZE unless ENTER is held)
};

// The COP parameter record (8 words) -> board settings (gate-checked against tb_cop / tb_full).
inline Settings record_settings(const int rec[8]) {
    Settings s;
    s.room = std::clamp(rec[7], 0, 6);
    s.m = ((rec[6] - RT_CEILING[s.room]) % 128 + 128) % 128;
    s.l = rec[4] & 15;
    s.h = rec[5] & 15;
    s.refl_level = rec[0] - 0x60;
    s.rev_level = rec[1] - 0x60;
    s.refl_delay = ((rec[2] - 0x81) % 256 + 256) % 256;
    s.rev_delay = ((rec[3] - 0x81) % 256 + 256) % 256;
    return s;
}

// Front-panel values -> the record (qrs_model.panel_record). Levels: 0..-30 dB, or < -30 = OFF.
inline void panel_record(int rec[8], int room, int r, int low, int high, int rev_delay_ms, int refl_delay_ms,
                         int rev_level_db, int refl_level_db) {
    auto lv = [](int db) { return db < -30 ? 0x60 : 0x7f + db; };
    rec[0] = lv(refl_level_db);
    rec[1] = lv(rev_level_db);
    rec[2] = (refl_delay_ms + 0x80) % 256;
    rec[3] = (rev_delay_ms + 0x80) % 256;
    rec[4] = 0x60 + low;
    rec[5] = 0x60 + high;
    rec[6] = r;
    rec[7] = room;
}

// ---------------------------------------------------------------- SIP (bit-exact, qrs_model.SIP)
class Sip {
public:
    void set(const uint8_t* sip, int m, int h, int l, bool freeze) {
        m &= 127;
        const int fb = m ^ 64;
        for (int g = 0; g < 16; g++) {
            const uint8_t* p = sip + g * 128;
            b_[g][0] = p[(fb + 4 * int(sx(l, 4)) + 512) % 128];
            b_[g][1] = p[fb];
            b_[g][2] = p[(fb + 4 * int(sx(h, 4)) + 512) % 128];
            b_[g][3] = p[m];
        }
        freeze_ = freeze;
    }
    int gin(int g) const { return b_[g][3]; }
    void clear() {
        S_.fill(0), W_.fill(0);
        pL_ = pM_ = pH_ = 0;
        sM_ = sH_ = sS_ = d7_ = 0;
    }

    // u = 39 A - 40 B, gx = input-gain byte x input word. Out: y (29 bit), the DIS wet word (32 bit).
    void group(int g, int64_t u, int64_t gx, int64_t& y, int64_t& wet) {
        const uint8_t bL = b_[g][0], bM = b_[g][1], bH = b_[g][2], bG = b_[g][3];
        int64_t pL = 0, pM = 0, pH = 0;
        if (!freeze_) {
            pL = prod(bL, bG & 1, u, false);
            pM = prod(bM, bL & 1, u, true);
            pH = prod(bH, bM & 1, u, true);
        }
        const uint64_t sL = (uint64_t(pL_ >> 56) & 0xff) | (uint64_t(pL) << 8);
        const uint64_t sM = (uint64_t(pM_ >> 48) & 0xffff) | (uint64_t(pM) << 16);
        const uint64_t sH = (uint64_t(pH_ >> 40) & 0xffffff) | (uint64_t(pH) << 24);
        const uint64_t q = (sL - sM) << 1;
        uint64_t v = (S_[g] >> 26) & 0xffffffffULL;   // U518: S_prev bits 26..57, sign held
        if (v >> 31) v |= ~0xffffffffULL;
        const uint64_t S = ((((v << 24) - (v << 20)) << 1) + q) << 1;
        uint64_t pMd, pHd;
        if (freeze_) {
            pMd = uint64_t(u) << 19;
            pHd = uint64_t(u) << 27;
        } else {
            pMd = (sM << 1) | (sM_ >> 63);   // U536_2
            pHd = (sH << 1) | (sH_ >> 63);   // U516_4
        }
        const uint64_t m6 = pMd << 7;                          // U516_6
        const uint64_t s10 = (S << 2) | ((sS_ >> 62) & 1);     // U512_10
        const uint64_t u5 = (m6 + s10) << 1;                   // U516_5
        const uint64_t V = (u5 - pHd) << 1;
        const uint64_t W = (u5 + pHd) << 1;
        const uint64_t w4 = ((W_[g] >> 31) & 0xffffffffULL) | ((m6 >> 32) << 32);   // U515_4
        const uint64_t d2 = ((uint64_t(gx) << 1) - w4) << 1;  // U515_2
        const uint64_t d7 = d2 << 1;                           // U536_7
        const uint64_t d15 = (d7 << 29) | (d7_ >> 35);         // U516_15
        y = tc(((d15 - V) << 1) >> 33, 29);                    // U516_12
        wet = tc(d7 >> 31, 32);
        S_[g] = S, W_[g] = W;
        pL_ = pL, pM_ = pM, pH_ = pH;
        sM_ = sM, sH_ = sH, sS_ = S, d7_ = d7;
    }

private:
    static int64_t prod(uint8_t b, int r0, int64_t u, bool late) {
        int64_t Y, K, Y0, K0;
        if (b & 1) Y = u >> 1, K = 7 * u;
        else Y = 4 * u, K = 0;
        if (r0) Y0 = u >> 1, K0 = 7 * u;
        else Y0 = 4 * u, K0 = 0;
        if (late) Y >>= 8, K >>= 8, Y0 >>= 8, K0 >>= 8;
        Y = Y - (Y & 1) + (Y0 & 1);
        K = K - (K & 1) + (K0 & 1);
        return int64_t(b & 0xfe) * Y + 128 * K;
    }
    uint8_t b_[16][4] = {};
    bool freeze_ = false;
    std::array<uint64_t, 16> S_{}, W_{};
    int64_t pL_ = 0, pM_ = 0, pH_ = 0;
    uint64_t sM_ = 0, sH_ = 0, sS_ = 0, d7_ = 0;
};

// ---------------------------------------------------------------- DY (pre-delay, 1st reflection)
class Dy {
public:
    void set(const uint8_t* dy, const Settings& s) {
        dv_ = 10 + 20 * std::clamp(s.rev_delay, 0, 199);
        dr_ = 10 + 20 * std::clamp(s.refl_delay, 0, 199);
        lv_ = dy[0x300 + std::clamp(s.rev_level, 0, 31)];
        lr_ = dy[0x300 + std::clamp(s.refl_level, 0, 31)];
    }
    void clear() { hl_.fill(0), hr_.fill(0); }
    // ADC words of this frame -> the reverb feed per board group G (clocks 0-31, the SIP input) and the
    // reflection word per group (clocks 32-63, DIS "Direct"). Exact against the gate sim (the project's
    // notes 2026-11-26 whole-chain-bit-exact): U628 multiplies the word read back through the U601 bit
    // buffer, whose bit 0 comes from the cell the R DRAM has just written.
    void frame(int xl, int xr, int sip[16], int refl[16]) {
        n_++;
        hl_[n_ & kMask] = xl, hr_[n_ & kMask] = xr;
        for (int G = 0; G < 16; G++) {
            // reverb feed: multiplied in group G - 1 (G = 0: group 15 of the previous frame)
            const int g = G ? G - 1 : 15;
            const uint32_t ng = G ? n_ : n_ - 1;
            sip[G] = word(G >= 8 ? n_ - dv_ : n_ - 1 - dv_, G % 2 == 0, g, ng, dv_, lv_);
            // reflection: multiplied in group G itself
            refl[G] = word(G >= 8 ? n_ - dr_ : n_ - 1 - dr_, G % 2 == 1, G, n_, dr_, lr_);
        }
    }

private:
    static constexpr int kSize = 8192, kMask = kSize - 1;   // > 10 + 20 x 200 frames
    // the level-scaled word for sample `frame` of channel ch (true = R), multiplied in group g of frame ng:
    // bits 1-15 are the word's own; bit 0 is bit (g + 9) mod 16 of the R word one sample newer (from
    // group 7 on, two)
    int word(uint32_t frame, bool ch, int g, uint32_t ng, int delay, int level) const {
        const int w = (ch ? hr_ : hl_)[frame & kMask];
        const int fresh = hr_[(ng - uint32_t(delay) + (g >= 7 ? 1u : 0u)) & kMask];
        const int y = int(sx((w & ~1) | ((fresh >> ((g + 9) % 16)) & 1), 16));
        return (level * y) >> 8;
    }
    std::array<int, kSize> hl_{}, hr_{};
    uint32_t n_ = 0;
    int dv_ = 390, dr_ = 190, lv_ = 0, lr_ = 0;
};

// ---------------------------------------------------------------- DIS
inline int dis_word(int64_t acc, bool invert) {
    acc = sx(acc, 32);
    uint32_t lo = uint32_t(acc >> 10) & 0xffff;
    if ((lo >> 15) != (acc < 0 ? 1u : 0u)) lo ^= 0xffff;
    if (invert) lo ^= 0xffff;
    return (lo >> 15) ? int(lo) - 65536 : int(lo);
}

struct DisSigns {
    int s[16][4];
    constexpr DisSigns() : s{} {
        const int mask[4] = {8, 4, 2, 15}, pol[4] = {-1, 1, -1, -1};
        for (int g = 0; g < 16; g++)
            for (int k = 0; k < 4; k++) {
                int x = ((g + 1) % 16) & mask[k], pc = 0;
                while (x) pc += x & 1, x >>= 1;
                s[g][k] = pol[k] * ((pc & 1) ? -1 : 1);
            }
    }
};
inline constexpr DisSigns kDis{};
constexpr int DIS_OFFSET[4] = {0, 0, 0, 4};
// Direct (DY's reflection slot) reaches outputs 1 / 2 through U713 (LS164) and U715 (LS253) at half
// weight (x 512) in these board groups: (group, from the previous frame?, sign).
struct DisDirect { int group, prev, sign; };
constexpr DisDirect kDisDirect[2][6] = {
    {{8, 1, 1}, {10, 1, 1}, {12, 1, 1}, {14, 1, 1}, {0, 0, -1}, {2, 0, -1}},     // output 1 (L)
    {{1, 0, 1}, {3, 0, 1}, {9, 1, 1}, {11, 1, 1}, {13, 1, -1}, {15, 1, -1}},     // output 2 (R)
};

// ---------------------------------------------------------------- the digital machine (one frame)
class Core {
public:
    void init(const Roms& r) {
        roms_ = &r;
        clear();
    }
    void set(const Settings& s) {
        if (!roms_) return;
        const uint8_t* my = roms_->my.data() + std::clamp(s.room, 0, 6) * 128;
        for (int i = 0; i < 64; i++) inc_[i] = uint16_t(rev8(my[2 * i]) | (rev8(my[2 * i + 1]) << 8));
        sip_.set(roms_->sip.data(), s.m, s.h, s.l, s.freeze);
        dy_.set(roms_->dy.data(), s);
        inp_on_ = s.input_on;
    }
    void clear() {   // MUTE: the reverb memory and every register in the loop
        mem_.fill(0);
        A_.fill(0), Bp_.fill(0), wet_prev_.fill(0), refl_prev_.fill(0);
        acc_ = 0;
        yprev15_ = 0;
        sip_.clear();
        dy_.clear();
    }

    // One 20 kHz frame: ADC words in (input L, input R as DY sees them), DAC words out (outputs 1-4).
    void frame(int xl, int xr, int dac[4]) {
        int xs[16], refl[16];
        dy_.frame(xl, xr, xs, refl);
        int64_t wet[16];
        std::array<int32_t, 16> Bn{};
        uint32_t acc = acc_;
        int64_t ylast = yprev15_;
        int j = 0;
        int32_t a0next = 0;
        for (int g = 0; g < 16; g++) {
            mem_[acc] = int32_t(sx(ylast >> 3, 26));                    // c0: write y of the previous group
            acc = (acc + inc_[j++]) & 0xffff;
            Bn[g] = mem_[acc];                                          // c1: read B_g (used next frame)
            acc = (acc + inc_[j++]) & 0xffff;
            const int64_t a = A_[g], b = Bp_[g];
            const int64_t s2 = 5 * a + 8 * b;
            const int64_t u = 39 * a - 40 * b;
            int64_t y;
            sip_.group(g, u, inp_on_ ? int64_t(sip_.gin(g)) * xs[g] : 0, y, wet[g]);
            mem_[acc] = int32_t(sx(s2 >> 3, 26));                       // c2: write s2
            acc = (acc + inc_[j++]) & 0xffff;
            const int32_t nxt = mem_[acc];                              // c3: read A of the next group
            acc = (acc + inc_[j++]) & 0xffff;
            if (g < 15) A_[g + 1] = nxt;
            else a0next = nxt;
            ylast = y;
        }
        A_[0] = a0next;
        yprev15_ = ylast;
        Bp_ = Bn;
        acc_ = acc;
        for (int k = 0; k < 4; k++) {
            int64_t sum = DIS_OFFSET[k];
            if (k < 2)
                for (const auto& d : kDisDirect[k]) sum += 512 * d.sign * int64_t(d.prev ? refl_prev_[d.group] : refl[d.group]);
            for (int g = 0; g < 16; g++) sum += kDis.s[g][k] * 4 * sx(g >= 7 ? wet_prev_[g] : wet[g], 28);
            dac[k] = dis_word(sum, k == 1 || k == 3);
        }
        for (int g = 0; g < 16; g++) wet_prev_[g] = wet[g], refl_prev_[g] = refl[g];
    }

private:
    const Roms* roms_ = nullptr;
    std::array<int32_t, 65536> mem_{};
    std::array<uint16_t, 64> inc_{};
    std::array<int32_t, 16> A_{}, Bp_{};
    std::array<int64_t, 16> wet_prev_{};
    std::array<int, 16> refl_prev_{};
    uint32_t acc_ = 0;
    int64_t yprev15_ = 0;
    bool inp_on_ = true;
    Sip sip_;
    Dy dy_;
};

// ---------------------------------------------------------------- converters
constexpr double AD_DELTA = 0.5 / (3.3214e-3 / 4095 * 1e6);   // R815 0.5 uA in DAC steps (~0.62)

// x: the S/H value, full scale = 1 (3.32 V) -> the 16-bit word DY receives (converters.ad_law).
inline int ad_law(double x) {
    const double a = std::fabs(x) * 8 * 4095;
    const int s = a > 4095 + AD_DELTA ? 8 : 1;
    const double m = std::clamp(std::ceil(a / s - AD_DELTA), 0.0, 4095.0);
    const int w = s * int(m);
    return x < 0 ? -w : (x > 0 ? w : 0);
}

// a DAC word -> volts, full scale = 1 (converters.da_law / 32768)
inline double da_law(int w) {
    const int c = w & 0x7fff, k = c >> 12, n = c & 4095;
    return (w >= 0 ? 4095.0 * k + n : -(4095.0 * (8 - k) - n)) / 32768.0;
}

template <int N>
struct Sos {   // direct form II transposed
    double z[N][2] = {};
    double run(double x, const double (&c)[N][6]) {
        for (int i = 0; i < N; i++) {
            const double y = c[i][0] * x + z[i][0];
            z[i][0] = c[i][1] * x - c[i][4] * y + z[i][1];
            z[i][1] = c[i][2] * x - c[i][5] * y;
            x = y;
        }
        return x;
    }
};

constexpr int kAdN = int(sizeof(kAdSos) / sizeof(kAdSos[0]));
constexpr int kDaN = int(sizeof(kDaSos) / sizeof(kDaSos[0]));
constexpr int kShN = int(sizeof(kAdShFir) / sizeof(kAdShFir[0]));

// The whole unit at 80 kHz: two inputs (full scale 1 = ADC full scale), four outputs (DAC full scale = 1).
class Unit {
public:
    void init(const Roms& r) { core_.init(r); }
    void set(const Settings& s) { core_.set(s); }
    void clear() { core_.clear(); }
    // crossover: true = input L reaches DY In Right (the manual's routing: L's reflection on output 2)
    void set_crossover(bool c) { cross_ = c; }

    void step(double in_l, double in_r, double out[4]) {
        double v[2] = {in_l, in_r};
        for (int c = 0; c < 2; c++) {
            sh_[c][pos_] = ad_[c].run(v[c], kAdSos);
            double acc = 0;
            for (int i = 0; i < kShN; i++) acc += kAdShFir[i] * sh_[c][(pos_ - i + kShN) % kShN];
            held_in_[c] = acc;
        }
        pos_ = (pos_ + 1) % kShN;
        if (sub_ == 0) {
            const int wl = ad_law(held_in_[0]), wr = ad_law(held_in_[1]);
            int dac[4];
            if (cross_) core_.frame(wr, wl, dac);
            else core_.frame(wl, wr, dac);
            for (int k = 0; k < 4; k++) words_[k] = dac[k];
        }
        for (int k = 0; k < 4; k++) {
            if (sub_ == k) hold_[k] = da_law(words_[k]);   // OUT k+1 loaded k quarter-frames after the latch
            out[k] = da_[k].run(hold_[k], kDaSos);
        }
        sub_ = (sub_ + 1) % UP;
    }
    const int* words() const { return words_; }

private:
    Core core_;
    Sos<kAdN> ad_[2];
    Sos<kDaN> da_[4];
    double sh_[2][kShN] = {};
    double held_in_[2] = {};
    double hold_[4] = {};
    int words_[4] = {};
    int pos_ = 0, sub_ = 0;
    bool cross_ = true;
};

}  // namespace qrs
