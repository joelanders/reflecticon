#include "qrs_machine.hpp"
#include "qrs_panel.hpp"
#include "../core/qrs.hpp"
#include "../../shared/core/mailbox.hpp"
#include "../../shared/core/rate_bridge.hpp"
#include "../../shared/core/sha256.hpp"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>

namespace qrsplug {
namespace {

// The panel's eight parameters (the stored program), then the effects keys, then the plugin's own.
enum P { Room, Time, High, Low, RevDelay, RevLevel, ReflDelay, ReflLevel, Freeze, Enter, Mute, Input, Output,
         Outputs, Polarity, NumParams };

constexpr int kLevelOff = -31;         // the level parameters' OFF step (below -30 dB)
constexpr double kMuteSeconds = 0.5;   // room-size change: "mutes the reverb signal for 0.5 s" (manual p 15);
                                       // MUTE "briefly" (p 23): taken as the same

// The panel's HIGH END / LOW END steps: factor 10^(k/5), k = index - 5 (LIN = 1)
const char* const kFactor[11] = {"0.1", "0.16", "0.25", "0.4", "0.63", "LIN", "1.6", "2.5", "4", "6.3", "10"};
const char* const kRoomName[7] = {"1 m³", "10 m³", "100 m³", "1,000 m³", "10,000 m³", "100,000 m³", "1,000,000 m³"};
const char* const kRoomShown[7] = {"1", "10", "10.2", "10.3", "10.4", "10.5", "10.6"};   // the display (m³)

// REVERB TIME r = 20 log10(RT / 10 ms), shown as the R20 preferred numbers (manual pp 17, 46)
std::string rt_text(int r, bool display) {
    static const char* d0[20] = {"0.1", "0.11", "0.12", "0.14", "0.16", "0.18", "0.2", "0.22", "0.25", "0.28",
                                 "0.32", "0.36", "0.4", "0.45", "0.5", "0.56", "0.63", "0.71", "0.8", "0.9"};
    static const char* d1[20] = {"1.0", "1.1", "1.25", "1.4", "1.6", "1.8", "2.0", "2.2", "2.5", "2.8",
                                 "3.2", "3.6", "4.0", "4.5", "5.0", "5.6", "6.3", "7.1", "8.0", "9.0"};
    static const char* d2[20] = {"10", "11", "12.5", "14", "16", "18", "20", "22", "25", "28",
                                 "32", "36", "40", "45", "50", "56", "63", "71", "80", "90"};
    r = std::clamp(r, 20, 80);
    const int d = (r - 20) / 20, i = (r - 20) % 20;
    if (d == 3) return "100";
    if (d == 0 && i == 2 && !display) return "0.125";
    return (d == 0 ? d0 : d == 1 ? d1 : d2)[i];
}

std::string fmt(const char* f, double v) {
    char b[48];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

struct Ctl {   // control -> audio
    qrs::Settings s;
    float in_gain = 1, out_gain = 1;
    int outputs = 0;
    bool invert24 = true;
    uint32_t mute_serial = 0;
};

// The factory programs, files 7 ("Loudspeakers") and 8 ("Headphones"), locations 2-8 (manual p 46): HIGH,
// TIME (s), LOW, room. The table gives no delays or levels: these use the panel photo's (p 51): REVERB
// DELAY 1 ms at -0 dB, 1ST REFLECTION off. Names as in the table, the maker's names left out.
struct Factory {
    int file, loc;
    const char* name;
    double high, time, low;
    int room;
};
const Factory kFactory[14] = {
    {7, 2, "Plate", 0.63, 3.2, 0.63, 2},
    {7, 3, "Chamber music studio", 0.63, 2.2, 1.6, 3},
    {7, 4, "Concert hall, full", 0.4, 2.8, 1.0, 4},
    {7, 5, "Concert hall, empty", 0.63, 3.2, 1.6, 4},
    {7, 6, "Church", 0.25, 4.5, 2.5, 5},
    {7, 7, "Cathedral", 0.16, 12.5, 1.6, 6},
    {7, 8, "Large dome", 0.1, 45, 1.0, 6},
    {8, 2, "Wardrobe, full", 0.1, 0.16, 0.1, 0},
    {8, 3, "Wardrobe, empty", 0.4, 0.25, 0.1, 0},
    {8, 4, "1,000 l oil tank", 0.1, 0.5, 10, 0},
    {8, 5, "Ship / submarine", 0.1, 1.0, 10, 2},
    {8, 6, "Living room, curtains closed", 0.16, 0.4, 0.16, 2},
    {8, 7, "Living room, curtains open", 0.63, 0.4, 0.16, 2},
    {8, 8, "Living room, unfurnished", 1.0, 0.63, 2.5, 2},
};
std::vector<float> factory_values(const Factory& f) {
    const int r = int(std::lround(20 * std::log10(f.time / 0.01)));
    const int hi = int(std::lround(5 * std::log10(f.high))) + 5, lo = int(std::lround(5 * std::log10(f.low))) + 5;
    return {float(f.room), float(r), float(hi), float(lo), 1.0f, 0.0f, 1.0f, float(kLevelOff)};
}

// ---- the PROM images (no ROM bytes here: sizes, SHA-256 hashes, page-hash prefixes, repair masks) ----
enum Role { RoleMy, RoleSip, RoleDy, NumRoles, RoleUnknown = -1 };
const char* const kRoleName[NumRoles] = {"MY U426", "SIP U508", "DY U633"};
const char* const kMyChip = "affdbf36b8ebe1f534741c074460df143fc4b8b3007ddf014c2a9e10f0fe946f";
const char* const kDyChip = "2fce7e20a2f3ce18e089a663122747e54d11ae0bd1d1524521e0303ebcff799d";
// SIP U508: the known dump has seven rotted bytes (QRS project notes 2026-11-12-1600): the dump's hash,
// the repaired chip's hash, and the bits to flip. Six are pure 1 -> 0 losses (the chip has 1s under the
// mask); 0x540 is not (the dump has a 1 under its mask), so only the dump's own pattern repairs it.
const char* const kSipDump = "e1f39c97025361c5a8a434a1740208d413a4c2e570ac1d0cfd9bae54655acf33";
const char* const kSipRepaired = "72954733778de7ff01498dc3520502a5ddc5487614e183a245b050f3e8e8df0d";
struct Repair {
    int at;
    uint8_t mask;
    bool pure;
};
const Repair kSipRepairs[7] = {{0x440, 0xc8, true}, {0x460, 0x5f, true}, {0x4f0, 0x0d, true}, {0x540, 0xf0, false},
                               {0x560, 0x41, true}, {0x6ef, 0x02, true}, {0x7ef, 0x02, true}};
// the first 32 bits of each 128-byte page's SHA-256 in the known chips (the SIP repaired): enough to say
// which pages of an unknown image differ, and nothing of the bytes. MY page = room (7-15 unused, blank);
// SIP page = group; DY page 6 holds the level table (0x300).
const uint32_t kPageHash[NumRoles][16] = {
    {0x70205e94u, 0xd1467764u, 0xbdb780a9u, 0x5ede761du, 0x8d8e11d1u, 0xaf659c5eu, 0xae28c825u, 0xe9175db6u,
     0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u},
    {0xae01a179u, 0xb95ebedfu, 0x9ce1bdbeu, 0x5bf1c33du, 0x0d9d671au, 0x1a1ec43eu, 0x6953eafcu, 0xe216ef85u,
     0x09b93f2bu, 0x4b82b002u, 0xecb85b63u, 0xc7ad929au, 0x82f8190au, 0xf340392cu, 0x7d03d785u, 0x690a8444u},
    {0xe9175db6u, 0xe9175db6u, 0xd309cc5fu, 0xfe730275u, 0x8162309eu, 0x98706cbeu, 0x3e9f0e59u, 0x38723a2eu,
     0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u, 0xe9175db6u},
};
using Image = std::array<uint8_t, 2048>;
std::string sha(const Image& b) { return kit::Sha256::of(b.data(), b.size()); }

// a role from the image's hash, else from its contents: DY has the level table at 0x300-0x31f (rising
// from 0 to 0xff); MY has seven pages of increment tables and the rest blank (0xff).
Role role_of(const Image& b) {
    const std::string h = sha(b);
    if (h == kMyChip) return RoleMy;
    if (h == kDyChip) return RoleDy;
    if (h == kSipDump || h == kSipRepaired) return RoleSip;
    bool dy = b[0x300] == 0 && b[0x31f] == 0xff;
    for (int i = 0x300; i < 0x31f && dy; i++) dy = b[size_t(i)] <= b[size_t(i) + 1];
    if (dy) return RoleDy;
    bool my = true;
    for (int i = 7 * 128; i < 2048 && my; i++) my = b[size_t(i)] == 0xff;
    for (int pg = 0; pg < 7 && my; pg++) my = !std::all_of(b.begin() + pg * 128, b.begin() + pg * 128 + 128, [](uint8_t x) { return x == 0xff; });
    return my ? RoleMy : RoleUnknown;
}

// the pages of `b` whose hash prefix differs from the known chip's
std::vector<int> pages_differing(const Image& b, Role r) {
    std::vector<int> out;
    for (int pg = 0; pg < 16; pg++) {
        const std::string h = kit::Sha256::of(b.data() + pg * 128, 128);
        if (uint32_t(std::stoul(h.substr(0, 8), nullptr, 16)) != kPageHash[r][pg]) out.push_back(pg);
    }
    return out;
}
std::string list(const std::vector<int>& v) {
    std::string s;
    for (size_t i = 0; i < v.size(); i++) s += (i ? (i + 1 == v.size() ? " and " : ", ") : "") + std::to_string(v[i]);
    return s;
}
std::string hex3(int a) {
    char b[8];
    std::snprintf(b, sizeof b, "0x%03x", a);
    return b;
}

// The SIP: the known dump or chip, or a near miss of the known rot: at some of the seven addresses the
// byte is the chip's, at others a pure 1 -> 0 loss under the mask (repaired by setting the mask bits) or
// the dump's own pattern (repaired by the mask). Tries every combination (at most 3^7) against the
// chip's hash. Returns the repaired addresses, or -1 if no combination gives the chip.
int repair_sip(Image& b, std::string& how) {
    const std::string h = sha(b);
    if (h == kSipRepaired) return 0;
    Image t = b;
    int choice[7] = {};
    std::vector<uint8_t> cand[7];
    for (int i = 0; i < 7; i++) {
        const auto& r = kSipRepairs[i];
        const uint8_t u = b[size_t(r.at)];
        cand[i] = {u};
        if (r.pure && uint8_t(u | r.mask) != u) cand[i].push_back(uint8_t(u | r.mask));
        if (std::find(cand[i].begin(), cand[i].end(), uint8_t(u ^ r.mask)) == cand[i].end()) cand[i].push_back(uint8_t(u ^ r.mask));
    }
    for (;;) {
        for (int i = 0; i < 7; i++) t[size_t(kSipRepairs[i].at)] = cand[i][size_t(choice[i])];
        if (sha(t) == kSipRepaired) break;
        int i = 0;
        while (i < 7 && ++choice[i] == int(cand[i].size())) choice[i++] = 0;
        if (i == 7) return -1;
    }
    std::vector<std::string> fixed;
    for (int i = 0; i < 7; i++)
        if (const uint8_t was = b[size_t(kSipRepairs[i].at)], now = t[size_t(kSipRepairs[i].at)]; now != was)
            fixed.push_back(hex3(kSipRepairs[i].at) + ((now & was) == was ? "" : " (as in the known dump)"));
    b = t;
    how.clear();
    for (size_t i = 0; i < fixed.size(); i++) how += (i ? ", " : "") + fixed[i];
    return int(fixed.size());
}

// The three images into roles (swapped files are put back by hash or contents), the SIP repaired where
// it can be, and a note for the status line. false (and *err) when the set can't be sorted out.
bool take_roms(const kit::RomSet& roms, qrs::Roms& out, std::string& note, std::string* err) {
    Image img[NumRoles];
    for (int i = 0; i < NumRoles; i++) {
        const auto& b = roms.bytes(size_t(i));
        if (b.size() != 2048) {
            *err = std::string(kRoleName[i]) + ": " + std::to_string(b.size()) + " bytes; the QRS PROMs (2716) are 2048 bytes";
            return false;
        }
        std::copy(b.begin(), b.end(), img[i].begin());
    }
    Role role[NumRoles];
    for (int i = 0; i < NumRoles; i++) role[i] = role_of(img[i]);
    // an unrecognised image takes its own slot's role if nothing else claims it, else the one left over
    auto claimed = [&](int r) { return std::count(role, role + NumRoles, Role(r)); };
    for (int i = 0; i < NumRoles; i++)
        if (role[i] == RoleUnknown && claimed(i) == 0) role[i] = Role(i);
    for (int i = 0; i < NumRoles; i++)
        if (role[i] == RoleUnknown)
            for (int r = 0; r < NumRoles; r++)
                if (claimed(r) == 0) {
                    role[i] = Role(r);
                    break;
                }
    for (int pass = 0; pass < 2; pass++)   // two files for one role first: the likelier mistake to name
    for (int r = 0; r < NumRoles; r++)
        if (pass == 0 ? claimed(r) > 1 : claimed(r) == 0) {
            std::vector<int> who;
            for (int i = 0; i < NumRoles; i++)
                if (role[i] == r) who.push_back(i);
            *err = claimed(r) ? "the files in the " + std::string(kRoleName[who[0]]) + " and " + kRoleName[who[1]] +
                                    " slots both look like the " + kRoleName[r] + " image"
                              : std::string("no file looks like the ") + kRoleName[r] + " image";
            return false;
        }
    std::vector<std::string> notes;
    {
        std::vector<std::string> moved;
        for (int i = 0; i < NumRoles; i++)
            if (role[i] != i) moved.push_back(std::string("the ") + kRoleName[role[i]] + " image was in the " + kRoleName[i] + " slot");
        if (!moved.empty()) {
            std::string m;
            for (size_t i = 0; i < moved.size(); i++) m += (i ? "; " : "") + moved[i];
            notes.push_back(m + " (taken by its contents)");
        }
    }
    Image my, sip, dy;
    for (int i = 0; i < NumRoles; i++) (role[i] == RoleMy ? my : role[i] == RoleSip ? sip : dy) = img[i];
    if (sha(my) != kMyChip) {
        const auto d = pages_differing(my, RoleMy);
        std::vector<int> rooms, rest;
        for (int pg : d) (pg < 7 ? rooms : rest).push_back(pg);
        notes.push_back("MY U426 is not the known chip (" + (rooms.empty() ? std::string() : "room tables " + list(rooms)) +
                        (rooms.empty() || rest.empty() ? "" : "; ") + (rest.empty() ? std::string() : "unused pages " + list(rest)) +
                        " differ): used as it is");
    }
    if (sha(dy) != kDyChip)
        notes.push_back("DY U633 is not the known chip (pages " + list(pages_differing(dy, RoleDy)) + " of 16 differ): used as it is");
    std::string how;
    if (sha(sip) == kSipDump) {
        for (auto& r : kSipRepairs) sip[size_t(r.at)] ^= r.mask;
        if (sha(sip) != kSipRepaired) {
            *err = "the SIP repairs did not give the expected chip";
            return false;
        }
        notes.push_back("SIP U508: the known dump, its 7 rotted bytes repaired at load");
    } else if (const int n = repair_sip(sip, how); n > 0) {
        notes.push_back("SIP U508: " + std::to_string(n) + " of the known dump's 7 rotted bytes found and repaired (" + how + ")");
    } else if (n < 0) {
        const auto d = pages_differing(sip, RoleSip);
        notes.push_back("SIP U508 is not a known dump: " + std::to_string(d.size()) + " of 16 pages differ from the repaired chip (groups " +
                        list(d) + "); used as it is");
    }
    out.my = my, out.sip = sip, out.dy = dy;
    note.clear();
    for (size_t i = 0; i < notes.size(); i++) note += (i ? ". " : "") + notes[i];
    return true;
}

uint8_t seg(char c) {
    switch (c) {
        case '0': case 'O': return 0x3F;
        case '1': return 0x06;
        case '2': return 0x5B;
        case '3': return 0x4F;
        case '4': return 0x66;
        case '5': return 0x6D;
        case '6': return 0x7D;
        case '7': return 0x07;
        case '8': return 0x7F;
        case '9': return 0x6F;
        case '-': return 0x40;
        case 'F': return 0x71;
        case 'L': return 0x38;
        case 'I': return 0x30;
        case 'N': return 0x37;
        default: return 0;
    }
}
// a display's text ("10.4", "OFF", "-30") into n digits, right-aligned; '.' lights the previous digit's point
void put(std::vector<uint8_t>& d, int at, int n, const std::string& s) {
    std::vector<uint8_t> cells;
    for (char c : s) {
        if (c == '.' && !cells.empty()) cells.back() |= 0x80;
        else cells.push_back(seg(c));
    }
    for (int i = 0; i < n; i++) {
        const int k = int(cells.size()) - n + i;
        d[size_t(at + i)] = k >= 0 ? cells[size_t(k)] : 0;
    }
}

class QrsMachine final : public kit::Machine {
public:
    bool boot(const kit::RomSet& roms, std::string* err) {
        roms_ = std::make_unique<qrs::Roms>();
        if (!take_roms(roms, *roms_, rom_note_, err)) return false;
        unit_ = std::make_unique<qrs::Unit>();
        unit_->init(*roms_);
        ctl_ = ctl_for(defaults());
        unit_->set(ctl_.s);
        unit_->set_crossover(true);   // the manual's routing: input L's 1st reflection on output 2 (p 42)
        cur_ = ctl_;
        return true;
    }

    void prepare(int host_rate, int max_block) override {
        bridge_.setup(host_rate, max_block, long(qrs::FS4), 1, 0.0, 2);
    }
    long latency() const override { return bridge_.latency(); }
    bool live_readouts() const override { return true; }   // the meters

    void set_params(const std::vector<float>& v) override {
        if (v.size() < NumParams) return;
        v_ = v;
        Ctl c = ctl_for(v);
        if (v[Mute] > 0.5f && !mute_held_) mute_serial_++;
        mute_held_ = v[Mute] > 0.5f;
        c.mute_serial = mute_serial_;
        ctl_ = c;
        ctls_.post(c);
        for (auto it = pending_.begin(); it != pending_.end();)   // the host has taken a panel setting
            it = std::abs(v_[size_t(it->first)] - it->second) < 1e-4f ? pending_.erase(it) : std::next(it);
    }

    kit::Readouts readouts() const override {
        kit::Readouts o;
        o.param.assign(NumParams, "");
        if (v_.size() < NumParams) return o;
        const qrs::Settings& s = ctl_.s;
        const int room = s.room, r = time_r(v_);
        o.param[Room] = std::string(kRoomName[room]) + ", reverb time " + rt_text(qrs::RT_FLOOR[room], false) + "-" +
                        rt_text(qrs::RT_CEILING[room], false) + " s";
        o.param[Time] = rt_text(r, false) + " s" + (r != int(std::lround(v_[Time])) ? " (the room's limit)" : "");
        const int hi = idx(v_[High], 7), lo = idx(v_[Low], 10);
        o.param[High] = hi == 5 ? std::string("LIN (as the mid band)") : std::string("x") + kFactor[hi] + " at 8 kHz";
        o.param[Low] = lo == 5 ? std::string("LIN (as the mid band)") : std::string("x") + kFactor[lo] + " at 40 Hz";
        o.param[RevDelay] = fmt("%.0f ms", s.rev_delay + 1);
        o.param[ReflDelay] = fmt("%.0f ms", s.refl_delay + 1);
        o.param[RevLevel] = level_text(s.rev_level);
        o.param[ReflLevel] = level_text(s.refl_level);
        o.param[Freeze] = s.freeze ? "frozen: lossless hold" : "off";
        o.param[Enter] = !s.freeze ? "acts in freeze" : s.input_on ? "input open" : "input closed";
        o.param[Mute] = v_[Mute] > 0.5f ? "clearing" : "ready";
        o.param[Input] = fmt("%+.1f dB", v_[Input]);
        o.param[Output] = fmt("%+.1f dB", v_[Output]);
        o.param[Outputs] = idx(v_[Outputs], 2) == 0 ? "outputs 1 / 2: with the 1st reflection"
                           : idx(v_[Outputs], 2) == 1 ? "outputs 3 / 4: reverb only"
                                                      : "1 + 3 / 2 + 4";
        o.param[Polarity] = ctl_.invert24 ? "outputs 2 and 4 inverted, as the unit" : "all in phase";
        o.panel = std::string(kRoomShown[room]) + " m3  " + rt_text(r, true) + " s  high " + kFactor[hi] + "  low " +
                  kFactor[lo] + "  reverb " + o.param[RevDelay] + " " + o.param[RevLevel] + "  1st refl " +
                  o.param[ReflDelay] + " " + o.param[ReflLevel] + (s.freeze ? "  FREEZE" : "");
        o.status = rom_note_;
        return o;
    }

    std::vector<kit::Preset> presets() const override {
        std::vector<kit::Preset> out;
        for (const Factory& f : kFactory) {
            kit::Preset p;
            p.name = std::to_string(f.file) + "." + std::to_string(f.loc) + " " + f.name;
            p.values = factory_values(f);
            out.push_back(std::move(p));
        }
        return out;
    }

    // ---- hardware panel ----
    void panel_key(int id, bool down) override {
        if (v_.size() < NumParams) return;
        if (id >= 1 && id <= 2 * NumPanelParams) {
            if (!down) return;
            const int p = (id - 1) / 2;
            knob_ = p;
            step(p, id % 2 == 0 ? +1 : -1);
            return;
        }
        if (id == KeyFreeze && down) set_from_panel(Freeze, cur(Freeze) > 0.5f ? 0.0f : 1.0f);
        if (id == KeyEnter) set_from_panel(Enter, down ? 1.0f : 0.0f);
        if (id == KeyMute) set_from_panel(Mute, down ? 1.0f : 0.0f);
        if (id == KeyStar && down) star_ = !star_;
        if (id >= KeyMem1 && id < KeyMem1 + 8 && down) {
            const int n = id - KeyMem1 + 1;
            if (star_) file_ = n, star_ = false;
            else loc_ = n;
            for (const Factory& f : kFactory)
                if (f.file == file_ && f.loc == loc_) {
                    const auto vals = factory_values(f);
                    for (size_t i = 0; i < vals.size(); i++) set_from_panel(int(i), vals[i]);
                }
        }
    }

    void panel_input(int id, float value) override {
        if (id != kKnobInput) return;
        if (!knob_seen_) {
            knob_seen_ = true;
            knob_at_ = value;
            return;
        }
        knob_acc_ += (value - knob_at_) * kKnobSteps;
        knob_at_ = value;
        while (knob_acc_ >= 1.0f) step(knob_, +1), knob_acc_ -= 1.0f;
        while (knob_acc_ <= -1.0f) step(knob_, -1), knob_acc_ += 1.0f;
    }

    kit::PanelState panel() const override {
        kit::PanelState s;
        if (v_.size() < NumParams || !unit_) return s;
        s.valid = true;
        std::vector<float> v = v_;
        for (auto& [i, x] : pending_) v[size_t(i)] = x;
        const Ctl c = ctl_for(v);
        s.digits.assign(NumQrsDigits, 0);
        auto lvl = [](int k) { return k == 0 ? std::string("OFF") : "-" + std::to_string(31 - k); };
        put(s.digits, 3 * PReflDelay, 3, std::to_string(c.s.refl_delay + 1));
        put(s.digits, 3 * PReflLevel, 3, lvl(c.s.refl_level));
        put(s.digits, 3 * PRevDelay, 3, std::to_string(c.s.rev_delay + 1));
        put(s.digits, 3 * PRevLevel, 3, lvl(c.s.rev_level));
        put(s.digits, 3 * PHigh, 3, kFactor[idx(v[High], 7)]);
        put(s.digits, 3 * PTime, 3, rt_text(time_r(v), true));
        put(s.digits, 3 * PLow, 3, kFactor[idx(v[Low], 10)]);
        put(s.digits, 3 * PRoom, 3, kRoomShown[c.s.room]);
        s.digits[DigitFile] = uint8_t(seg(char('0' + file_)) | (star_ ? 0x80 : 0));
        s.digits[DigitLoc] = seg(char('0' + loc_));
        s.leds.assign(NumQrsLeds, 0);
        s.leds[LedFreeze] = c.s.freeze ? 255 : 0;
        // the meters: peak since the last look, falling 20 dB/s of audio processed (no wall clock: the
        // web worklet has none); 0 dB = 12 dB below full scale
        const float dt = float(meter_frames_.exchange(0, std::memory_order_relaxed) / qrs::FS4);
        static const float thr[NumMeterLeds] = {-0.2f, -2, -6, -12, -24, -36, -48, -60, -72, -84};   // dBFS
        for (int b = 0; b < 6; b++) {
            const float pk = meter_[b].exchange(0.0f, std::memory_order_relaxed);
            const float db = pk > 0 ? 20.0f * std::log10(pk) : -120.0f;
            meter_db_[b] = std::max(db, meter_db_[b] - 20.0f * dt);
            for (int i = 0; i < NumMeterLeds; i++)
                s.leds[size_t(LedMeter0 + b * NumMeterLeds + i)] = meter_db_[b] >= thr[i] ? 255 : 0;
        }
        if (!pending_.empty()) {
            s.adopt.assign(NumParams, NAN);
            for (auto& [i, x] : pending_) s.adopt[size_t(i)] = x;
        }
        s.adopt_serial = adopt_serial_;
        return s;
    }

    void process(const float* const* in, float* out_l, float* out_r, int n) override {
        if (!bridge_.ready() || !unit_) {
            std::fill(out_l, out_l + n, 0.0f);
            std::fill(out_r, out_r + n, 0.0f);
            return;
        }
        Ctl c;
        if (ctls_.take(c)) {
            if (c.s.room != cur_.s.room) mute_left_ = long(kMuteSeconds * qrs::FS4);
            if (c.mute_serial != cur_.mute_serial) {
                unit_->clear();
                mute_left_ = long(kMuteSeconds * qrs::FS4);
            }
            unit_->set(c.s);
            cur_ = c;
        }
        qrs::Unit* u = unit_.get();
        const Ctl& k = cur_;
        float pk[6] = {};
        long& mute = mute_left_;
        long frames = 0;
        bridge_.process(in, out_l, out_r, n, [&](const float* const* x, float* l, float* r, int count) {
            for (int i = 0; i < count; i++) {
                const double xl = x[0][i] * k.in_gain, xr = x[1][i] * k.in_gain;
                pk[0] = std::max(pk[0], float(std::fabs(xl)));
                pk[1] = std::max(pk[1], float(std::fabs(xr)));
                double o[4];
                u->step(xl, xr, o);
                const int* w = u->words();
                for (int j = 0; j < 4; j++) pk[2 + j] = std::max(pk[2 + j], std::fabs(float(w[j])) / 32768.0f);
                if (!k.invert24) o[1] = -o[1], o[3] = -o[3];
                double a, b;
                if (k.outputs == 0) a = o[0], b = o[1];
                else if (k.outputs == 1) a = o[2], b = o[3];
                else a = (o[0] + o[2]) * M_SQRT1_2, b = (o[1] + o[3]) * M_SQRT1_2;
                const double g = mute > 0 ? 0.0 : k.out_gain;
                if (mute > 0) mute--;
                l[i] = float(a * g);
                r[i] = float(b * g);
            }
            frames += count;
        });
        meter_frames_.fetch_add(frames, std::memory_order_relaxed);
        for (int b = 0; b < 6; b++) {
            float was = meter_[b].load(std::memory_order_relaxed);
            while (pk[b] > was && !meter_[b].compare_exchange_weak(was, pk[b], std::memory_order_relaxed)) {}
        }
    }

private:
    static std::vector<float> defaults();
    static int idx(float v, int max) { return std::clamp(int(std::lround(v)), 0, max); }
    // the REVERB TIME the COP holds: the stored r, clamped to the room's range
    static int time_r(const std::vector<float>& v) {
        const int room = idx(v[Room], 6);
        return std::clamp(int(std::lround(v[Time])), qrs::RT_FLOOR[room], qrs::RT_CEILING[room]);
    }
    static std::string level_text(int k) { return k == 0 ? std::string("off") : fmt("%.0f dB", k - 31.0); }
    static Ctl ctl_for(const std::vector<float>& v) {
        const int room = idx(v[Room], 6);
        auto lvl = [](float x) { return int(std::lround(x)) <= kLevelOff ? -99 : std::clamp(int(std::lround(x)), -30, 0); };
        int rec[8];
        qrs::panel_record(rec, room, time_r(v), idx(v[Low], 10) - 5, idx(v[High], 7) - 5,
                          std::clamp(int(std::lround(v[RevDelay])), 1, 200), std::clamp(int(std::lround(v[ReflDelay])), 1, 200),
                          lvl(v[RevLevel]), lvl(v[ReflLevel]));
        Ctl c;
        c.s = qrs::record_settings(rec);
        c.s.freeze = v[Freeze] > 0.5f;
        c.s.input_on = !c.s.freeze || v[Enter] > 0.5f;   // FREEZE closes the input gate; ENTER opens it while held
        c.in_gain = float(std::pow(10.0, v[Input] / 20.0));
        c.out_gain = float(std::pow(10.0, v[Output] / 20.0));
        c.outputs = idx(v[Outputs], 2);
        c.invert24 = idx(v[Polarity], 1) == 0;
        return c;
    }
    // one step of a panel parameter, as its [-]/[+] key or the knob gives it
    void step(int p, int dir) {
        static const int host[NumPanelParams] = {ReflDelay, ReflLevel, RevDelay, RevLevel, High, Time, Low, Room};
        const int hp = host[p];
        std::vector<float> v = v_;
        for (auto& [i, x] : pending_) v[size_t(i)] = x;
        int x = int(std::lround(v[size_t(hp)]));
        if (hp == Time) {
            const int room = idx(v[Room], 6);
            x = std::clamp(time_r(v) + dir, qrs::RT_FLOOR[room], qrs::RT_CEILING[room]);
        } else {
            const auto& prm = qrsplug::machine_type().info.params[size_t(hp)];
            x = std::clamp(x + dir, int(prm.min), int(prm.max));
        }
        set_from_panel(hp, float(x));
    }
    void set_from_panel(int param, float value) const {
        pending_[param] = value;
        adopt_serial_++;
    }
    float cur(int param) const {
        auto it = pending_.find(param);
        return it != pending_.end() ? it->second : v_[size_t(param)];
    }
    static kit::Info make_info();
    friend const kit::MachineType& qrsplug::machine_type();

    std::unique_ptr<qrs::Roms> roms_;
    std::unique_ptr<qrs::Unit> unit_;
    kit::RateBridgeT<32> bridge_;
    kit::Mailbox<Ctl> ctls_;
    Ctl ctl_, cur_;            // control thread / audio thread
    long mute_left_ = 0;       // audio thread: samples (80 kHz) of muted output left
    std::vector<float> v_;
    std::string rom_note_;
    uint32_t mute_serial_ = 0;
    bool mute_held_ = false;

    // the panel (control thread)
    static constexpr float kKnobSteps = 60.0f;   // steps per full turn of the drawn knob
    mutable std::map<int, float> pending_;
    mutable uint32_t adopt_serial_ = 0;
    int knob_ = PTime;
    bool knob_seen_ = false;
    float knob_at_ = 0.5f, knob_acc_ = 0;
    int file_ = 7, loc_ = 1;
    bool star_ = false;
    mutable std::atomic<long> meter_frames_{0};   // 80 kHz frames since the panel last looked
    mutable float meter_db_[6] = {-120, -120, -120, -120, -120, -120};
    mutable std::atomic<float> meter_[6] = {};
};

std::unique_ptr<kit::Machine> boot(const kit::RomSet& roms, const std::string&, std::string* err) {
    if (!roms.complete()) {
        *err = "ROMs missing";
        return nullptr;
    }
    auto m = std::make_unique<QrsMachine>();
    if (!m->boot(roms, err)) return nullptr;
    return m;
}

}  // namespace

kit::Info QrsMachine::make_info() {
    kit::Info i;
    i.name = "Reflecticon QRS";
    i.state_tag = "QRSKIT";
    i.settings_name = "ReflecticonQRS";
    i.rom_env = "QRS_ROMPATH";
    i.inputs = 2;
    i.tail_seconds = 20;
    using K = kit::Param;
    std::vector<std::string> rooms(std::begin(kRoomName), std::end(kRoomName));
    std::vector<std::string> high(kFactor, kFactor + 8), low(kFactor, kFactor + 11);
    i.params = {
        {"room", "Room size", K::Choice, 0, 6, 3, rooms, ""},
        {"time", "Reverb time", K::Int, 20, 80, 46, {}, ""},
        {"high", "High end", K::Choice, 0, 7, 5, high, ""},
        {"low", "Low end", K::Choice, 0, 10, 5, low, ""},
        {"revDelay", "Reverb delay", K::Int, 1, 200, 20, {}, "ms"},
        {"revLevel", "Reverb level", K::Int, float(kLevelOff), 0, 0, {}, "dB"},
        {"reflDelay", "1st reflection delay", K::Int, 1, 200, 10, {}, "ms"},
        {"reflLevel", "1st reflection level", K::Int, float(kLevelOff), 0, float(kLevelOff), {}, "dB"},
        {"freeze", "Freeze", K::Bool, 0, 1, 0, {}, ""},
        {"enter", "Enter", K::Bool, 0, 1, 0, {}, ""},
        {"mute", "Mute", K::Bool, 0, 1, 0, {}, ""},
        {"input", "Input", K::Float, -30, 12, 0, {}, "dB"},
        {"output", "Output", K::Float, -30, 18, 0, {}, "dB"},
        {"outputs", "Outputs", K::Choice, 0, 2, 0, {"1 / 2", "3 / 4", "1+3 / 2+4"}, ""},
        {"polarity", "Polarity", K::Choice, 0, 1, 0, {"as the unit", "all in phase"}, ""},
    };
    kit::group(i.params, "Reverb", {{"room", "Room size"}, {"time", "Reverb time"}, {"high", "High end"}, {"low", "Low end"}});
    kit::group(i.params, "Delays", {{"revDelay", "Reverb delay"}, {"revLevel", "Reverb level"},
                                    {"reflDelay", "1st refl. delay"}, {"reflLevel", "1st refl. level"}});
    kit::group(i.params, "Effects", {{"freeze", "Freeze"}, {"enter", "Enter"}, {"mute", "Mute"}});
    kit::group(i.params, "In / Out", {{"input", "Input"}, {"output", "Output"}, {"outputs", "Outputs"}, {"polarity", "Polarity"}});
    i.groups = {"Reverb", "Delays", "Effects", "In / Out"};
    i.mix_group = "In / Out";
    kit::momentary(i.params, {{"enter", "Enter"}, {"mute", "Mute"}});
    i.panel = qrs_layout();
    kit::describe(i.params, {
        {"room", "Room size: sets the reverb's build-up, echo density and the spacing of the room's resonances, "
                 "and the range of reverb times. Changing it mutes the reverb for half a second, as on the unit."},
        {"time", "Reverb time (to -60 dB) in the mid band, in the unit's 1/20-decade steps; limited to the room's range."},
        {"high", "Reverb time near 8 kHz as a factor of the reverb time (LIN = the same)."},
        {"low", "Reverb time near 40 Hz as a factor of the reverb time (LIN = the same)."},
        {"revDelay", "Delay before the reverb, on top of the room's own."},
        {"revLevel", "The reverb's level, 0 to -30 dB, or off (the lowest step)."},
        {"reflDelay", "Delay of the 1st reflection: a stereo echo, swapped left for right, on outputs 1 and 2."},
        {"reflLevel", "The 1st reflection's level, 0 to -30 dB, or off (the lowest step)."},
        {"freeze", "Holds the room with no loss at any frequency; the input is closed except while Enter is held."},
        {"enter", "In freeze, opens the input while held, to load the room."},
        {"mute", "Clears the reverb memory and mutes the output briefly."},
        {"input", "Input level: 0 dB puts the plugin's full scale at the converter's full scale."},
        {"output", "Output level."},
        {"outputs", "Which of the unit's four outputs this plugin's left and right are: 1 / 2 carry the 1st reflection."},
        {"polarity", "The unit sends outputs 2 and 4 inverted; this can put them back in phase."},
    });
    i.roms = {
        {"MY U426", {2048}, {{kMyChip, "card 2 (0005dbe4)"}},
         {"0005dbe4", "u426"}},
        {"SIP U508", {2048},
         {{kSipDump, "card 3 (0003ce18)"}, {kSipRepaired, "card 3, repaired"}},
         {"0003ce18", "u508"}},
        {"DY U633", {2048}, {{kDyChip, "card 4 (0005da61)"}},
         {"0005da61", "u633", "u635"}},
    };
    i.about = "An emulation of a 1982 digital reverb from its schematics and ROMs. Bring your own ROM images "
              "(MY U426, SIP U508, DY U633: 2716s). Not affiliated with or endorsed by the maker.";
    return i;
}

std::vector<float> QrsMachine::defaults() {
    std::vector<float> v;
    for (auto& p : make_info().params) v.push_back(p.def);
    return v;
}

const kit::MachineType& machine_type() {
    static const kit::MachineType t{QrsMachine::make_info(), &boot};
    return t;
}

}  // namespace qrsplug
