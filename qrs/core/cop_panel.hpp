// The QRS controller (COP, sheet 2) and front panel (sheet 6) at gate level: the generated model
// (cop_panel_gen.hpp, from the patched netlists by quantec/emu/hdl/gen_cpp.py, checked bit for bit
// against the gate sim by tools_cmp_panel.py) plus what a host needs around it:
//   - keys S101..S130 and the knob's two contacts in, the panel's digits and LEDs out;
//   - the battery RAM as program pages: page = (file - 1) + 8 (location - 1), 8 words each;
//   - the Kill / Enter one-shots and the Freeze / Enhance / File toggles;
//   - idle skipping: when the whole machine is the same at two successive COP cycle starts
//     (16384 counts, 3.2 ms) except for one-shot countdowns, whole cycles are skipped (only the
//     countdowns move) until an input changes or a one-shot is about to end. A panel nobody
//     touches costs nothing.
// The COP ROMs (U308 display, U309 microcode; 2716) are the caller's; nothing of them is here.
#pragma once
#include "cop_panel_gen.hpp"
#include <cstdint>
#include <cstring>

namespace qrs {

class CopPanel {
public:
    static constexpr long kCycle = 16384;                 // COP counts per cycle
    static constexpr double kCountsPerSecond = 5.12e6;    // the COP counter (20.48 MHz / 4)

    // U308 = display ROM (card 1, 10052635), U309 = microcode (card 1, 20023330), 2048 bytes each.
    void boot(const uint8_t* u308, const uint8_t* u309) {
        m_ = CopPanelGen();
        m_.cop_u308_mem = u308;
        m_.cop_u309_mem = u309;
        m_.init();
        std::memset(seg_, 0xff, sizeof seg_);
        skipping_ = have_snap_ = false;
        owed_ = 0;
        last10k_ = m_.v[CopPanelGen::te_n__10k];
        // Power-on: the toggle clocks' filters charge in the first counts, which the toggles take
        // as a press (the gate sim does the same with its x -> 1). Let that pass, then leave the
        // battery-backed toggles as a unit in use would have them: File, Freeze, Enhance off.
        for (long n = 0; n < 4 * kCycle; n++) step();
        set_file_mode(false), set_freeze(false), set_enhance(false);
        stepped_ = skipped_ = 0;
    }

    // ---- inputs (each one wakes the machine from idle skipping first) ----
    void key(int s, bool down) {   // s = 101..130 (the panel's S101..S130)
        if (s < 101 || s > 130 || m_.key[s - 101] == uint8_t(down)) return;
        wake();
        m_.key[s - 101] = uint8_t(down);
    }
    void knob(bool a, bool b) {    // the knob's contacts (RV101): a quadrature pair
        if (m_.knob_a == uint8_t(a) && m_.knob_b == uint8_t(b)) return;
        wake();
        m_.knob_a = uint8_t(a), m_.knob_b = uint8_t(b);
    }
    void blink(bool show) {        // the blink oscillator's phase (not in the netlist: the caller's clock)
        if (m_.blink == uint8_t(show)) return;
        wake();
        m_.blink = uint8_t(show);
    }

    // ---- time ----
    void run(long counts) {
        while (counts > 0) {
            if (skipping_) {
                owed_ += counts;
                counts = 0;
                long periods = owed_ / kCycle;
                const uint32_t left = m_.oneshot_min();
                if (left != 0xffffffffu) {   // stop a cycle short of a one-shot's end and step through it
                    const long ok = long(left) > 2 * kCycle ? (long(left) - kCycle - 1) / kCycle : 0;
                    if (ok < periods) periods = ok;
                }
                if (periods > 0) {
                    m_.oneshot_advance(uint32_t(periods * kCycle));
                    owed_ -= periods * kCycle;
                    skipped_ += periods * kCycle;
                }
                const bool ends = left != 0xffffffffu && owed_ + kCycle >= long(left) - periods * kCycle;
                if (owed_ >= kCycle || ends) {   // a one-shot ends within what is owed: step it all
                    counts = owed_;
                    owed_ = 0;
                    skipping_ = false;
                }
                continue;
            }
            step();
            counts--;
            if (m_.c == 0 && allow_skip) at_cycle_start();
        }
    }
    bool allow_skip = true;   // false: step every count (tests)
    // counts stepped / skipped so far (for tests and cost checks)
    long long stepped() const { return stepped_; }
    long long skipped() const { return skipped_; }
    bool idle() const { return skipping_; }
    // machine time in counts since boot (stepped, skipped and owed)
    long long now() const { return stepped_ + skipped_ + owed_; }

    // ---- the battery RAM: 64 pages of 8 words ----
    static int page_of(int file, int loc) { return (file - 1) + 8 * (loc - 1); }   // 1..8, 1..8
    int page() const {   // the program page the COP is on (the address latches U321 / U344)
        return (m_.cop_u321_q & 7) | (m_.cop_u344_q & 7) << 3;
    }
    int file() const { return (m_.cop_u323_q & 7) + 1; }   // the FILE and LOC latches (displayed)
    int loc() const { return (m_.cop_u346_q & 7) + 1; }
    void read_page(int page, int rec[8]) const {
        for (int w = 0; w < 8; w++) {
            int r = 0;
            for (int i = 0; i < 8; i++) r |= (m_.cop_u343_mem[addr(page, w, i)] & 1) << i;
            rec[w] = r;
        }
    }
    void write_page(int page, const int rec[8]) {
        wake();
        for (int w = 0; w < 8; w++)
            for (int i = 0; i < 8; i++) m_.cop_u343_mem[addr(page, w, i)] = uint8_t(rec[w] >> i & 1);
    }
    // A program written into the page as the panel would leave it. On the page it is on, the COP
    // clamps REVERB TIME (word 6) to the range of the room it holds, which is word 7 as it read it a
    // cycle earlier (after boot or a location change, another page's). So the record goes in, the
    // COP gets 2 cycles (6.4 ms of machine time) to take the room, and the record goes in again.
    void write_program(int page, const int rec[8]) {
        write_page(page, rec);
        if (page != this->page()) return;
        run(2 * kCycle);
        int now[8];
        read_page(page, now);
        for (int w = 0; w < 8; w++)
            if (now[w] != rec[w]) {
                write_page(page, rec);
                run(2 * kCycle);
                return;
            }
    }
    // the whole RAM as 512 bytes (bit a at byte a / 8, bit a % 8): the unit's battery-backed state
    void save_ram(uint8_t out[512]) const {
        std::memset(out, 0, 512);
        for (int a = 0; a < 4096; a++) out[a >> 3] |= uint8_t((m_.cop_u343_mem[a] & 1) << (a & 7));
    }
    void load_ram(const uint8_t in[512]) {
        wake();
        for (int a = 0; a < 4096; a++) m_.cop_u343_mem[a] = uint8_t(in[a >> 3] >> (a & 7) & 1);
    }

    // ---- outputs ----
    bool kill() const { return !m_.v[CopPanelGen::cop_n_Kill]; }    // U341 Q2~ low: output muted, memory cleared
    bool enter() const { return m_.cop_u341_q1 != 0; }             // the input gate open in FREEZE
    bool freeze() const { return m_.cop_u339_q1 != 0; }
    bool enhance() const { return m_.cop_u339_q2 != 0; }
    bool file_mode() const { return m_.cop_u342_q1 != 0; }         // the red point in FILE
    // the 32 scan positions as the last full scan lit them: bit 0 a .. bit 6 g, bit 7 dp (1 = lit),
    // as the viewer reads them (FILE and LOC are mounted upside down). Positions 4, 8, ..., 28 are
    // not fitted.
    void scan_digits(uint8_t out[32]) const {
        for (int i = 0; i < 32; i++) {
            uint8_t d = 0;
            for (int b = 0; b < 8; b++)
                if (kScanSeg[i][b] >= 0 && !(seg_[i] >> b & 1)) d |= uint8_t(1u << kScanSeg[i][b]);
            out[i] = d;
        }
    }

    // ---- setting state the way a user would have left it (each settles the machine) ----
    void set_freeze(bool on) { poke(m_.cop_u339_q1, on); }
    void set_enhance(bool on) { poke(m_.cop_u339_q2, on); }
    void set_file_mode(bool on) { poke(m_.cop_u342_q1, on); }
    // as after recalling `loc` in `file`: the FILE / LOC latches and both pairs of address latches
    void set_location(int file, int loc) {
        wake();
        const uint8_t f = uint8_t((file - 1) & 7), l = uint8_t((loc - 1) & 7);
        m_.cop_u323_q = f, m_.cop_u346_q = l;
        m_.cop_u321_q = m_.cop_u322_q = f;
        m_.cop_u344_q = m_.cop_u345_q = l;
        m_.settle();
    }

    const CopPanelGen& gen() const { return m_; }

private:
    // U207 bit -> segment (0 a .. 6 g, 7 dp) per scan position, traced through the sheet 6 netlist
    // (quantec/emu/hdl/tools_panel_disp.py --cpp)
    static constexpr int8_t kScanSeg[32][8] = {
        {7, 5, 6, 2, 1, 0, 3, 4}, {1, 0, 3, 4, 5, 6, 2, 7}, {1, 0, 3, 4, 5, 6, 2, 7}, {1, 0, 3, 4, 5, 6, 2, 7},
        {-1, -1, -1, -1, -1, -1, -1, -1}, {1, 0, 3, 4, 5, 6, 2, 7}, {1, 0, 3, 4, 5, 6, 2, 7}, {1, 0, 3, 4, 5, 6, 2, 7},
        {-1, -1, -1, -1, -1, -1, -1, -1}, {1, 0, 3, 4, 5, 6, 2, 7}, {1, 0, 3, 4, 5, 6, 2, 7}, {1, 0, 3, 4, 5, 6, 2, 7},
        {-1, -1, -1, -1, -1, -1, -1, -1}, {1, 0, 3, 4, 5, 6, 2, 7}, {1, 0, 3, 4, 5, 6, 2, 7}, {1, 0, 3, 4, 5, 6, 2, 7},
        {7, 5, 6, 2, 1, 0, 3, 4}, {5, 6, 2, 7, 1, 0, 3, 4}, {5, 6, 2, 7, 1, 0, 3, 4}, {5, 6, 2, 7, 1, 0, 3, 4},
        {-1, -1, -1, -1, -1, -1, -1, -1}, {5, 6, 2, 7, 1, 0, 3, 4}, {5, 6, 2, 7, 1, 0, 3, 4}, {5, 6, 2, 7, 1, 0, 3, 4},
        {-1, -1, -1, -1, -1, -1, -1, -1}, {5, 6, 2, 7, 1, 0, 3, 4}, {5, 6, 2, 7, 1, 0, 3, 4}, {5, 6, 2, 7, 1, 0, 3, 4},
        {-1, -1, -1, -1, -1, -1, -1, -1}, {5, 6, 2, 7, 1, 0, 3, 4}, {5, 6, 2, 7, 1, 0, 3, 4}, {5, 6, 2, 7, 1, 0, 3, 4}};
    // RAM address of bit i of word w in a page (A0..A5 from the counter, as the COP's word scan
    // visits them; A6..A8 file, A9..A11 location)
    static int addr(int page, int w, int i) {
        static const int base[8] = {0x3d, 0x39, 0x3c, 0x38, 0x3f, 0x3b, 0x3e, 0x3a};
        static const int off[8] = {0, 0x10, 0x20, 0x30, 8, 0x18, 0x28, 0x38};
        return (page & 63) << 6 | (base[w] - off[i]);
    }

    void step() {
        m_.step();
        stepped_++;
        const uint8_t k10 = m_.v[CopPanelGen::te_n__10k];
        if (!k10 && last10k_) {   // the digit clock fell: the selected digit shows U207's outputs
            const uint32_t dsel = uint32_t(m_.te_u224_q) << 24 | uint32_t(m_.te_u223_q) << 16 | uint32_t(m_.te_u222_q) << 8 | m_.te_u221_q;
            for (int i = 0; i < 32; i++)
                if (dsel >> i & 1) seg_[i] = m_.te_u207_st;
        }
        last10k_ = k10;
    }
    void at_cycle_start() {
        std::memcpy(static_cast<void*>(&tmp_), &m_, sizeof m_);
        tmp_.oneshot_zero();
        if (have_snap_ && std::memcmp(&tmp_, &snap_, sizeof m_) == 0) skipping_ = true;
        std::memcpy(static_cast<void*>(&snap_), &tmp_, sizeof m_);
        have_snap_ = true;
    }
    // leave idle skipping: run the counts owed (less than a cycle), so the next input lands at
    // the right count
    void wake() {
        if (!skipping_) return;
        skipping_ = false;
        long n = owed_;
        owed_ = 0;
        while (n-- > 0) {
            step();
            if (m_.c == 0) at_cycle_start();
        }
        skipping_ = false;
    }
    void poke(uint8_t& q, bool on) {
        if (q == uint8_t(on)) return;
        wake();
        q = uint8_t(on);
        m_.settle();
    }

    CopPanelGen m_, snap_, tmp_;
    uint8_t seg_[32];
    uint8_t last10k_ = 0;
    bool skipping_ = false, have_snap_ = false;
    long owed_ = 0;
    long long stepped_ = 0, skipped_ = 0;
};

}  // namespace qrs
