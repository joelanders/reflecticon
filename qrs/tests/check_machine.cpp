// check_machine ROM_DIR: the QRS machine, JUCE-free.
//   1. readouts for the defaults and a few settings;
//   2. the factory programs: an impulse through the core (20 kHz, no converters), the 1 kHz-octave
//      reverb time, against the QRS project's measurement of its Python model;
//   3. an impulse through the host at 48 kHz: reverb delay, the 1st reflection (swapped to the right),
//      freeze holds, mute clears;
//   4. the hardware panel: layout, a [+] key, the knob, a memory recall;
//   5. the kit's host checks (block sizes, no allocation, a change reaches the machine, instances);
//   6. a live change while audio runs; 7. what it costs;
//   8. ROM handling, on images made at run time from the user's ROMs by flipping bits (no ROM bytes
//      here): the dump, the chip, partial and extra rot, swapped slots, wrong size, MY/DY changes;
//   9. behaviour: a reverb time and a room change while a tail rings, extreme settings, levels off,
//      delays 1 and 200 ms; 10. host rates 44.1-192 kHz (latency as reported), block sizes 1-4096.
#define KIT_COUNT_ALLOCATIONS
#include "../plugin/qrs_machine.hpp"
#include "../plugin/qrs_panel.hpp"
#include "../core/qrs.hpp"
#include "../../shared/tests/host_checks.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iterator>

static kit::RomSet load_roms(const kit::MachineType& t, const std::string& dir) {
    std::vector<kit::RomFile> files;
    for (auto& e : std::filesystem::directory_iterator(dir)) {
        if (!e.is_regular_file() || !kit::any_size_fits(t.info.roms, size_t(e.file_size()))) continue;
        std::ifstream f(e.path(), std::ios::binary);
        files.push_back({e.path().filename().string(), e.path().string(),
                         std::vector<uint8_t>(std::istreambuf_iterator<char>(f), {})});
    }
    return kit::recognise(t.info.roms, files);
}

static std::string fmt1(double v) {
    char b[32];
    std::snprintf(b, sizeof b, "%.1f", v);
    return b;
}

enum { Room, Time, High, Low, RevDelay, RevLevel, ReflDelay, ReflLevel, Freeze, Enter, Mute, Input, Output, Outputs, Polarity };

// An impulse (0.5 full scale) into the left input through a fresh host at 48 kHz: both outputs.
struct Run {
    std::vector<float> l, r;
    long latency = 0;
};
static Run impulse(const kit::MachineType& t, const kit::RomSet& roms, kit_test::Params& p, double seconds, int at = 100,
                   std::function<void(kit::Host&, kit_test::Params&, int)> during = nullptr, int rate = 48000) {
    auto host = std::make_unique<kit::Host>(t, p.ptrs());
    std::string err;
    auto m = t.boot(roms, "", &err);
    if (!m) {
        std::fprintf(stderr, "boot failed: %s\n", err.c_str());
        std::exit(1);
    }
    host->prepare(rate, 512);
    host->install(std::move(m));
    Run o;
    o.latency = host->latency();
    int total = int(seconds * rate);
    std::vector<float> in(size_t(total), 0.0f), z(size_t(total), 0.0f);
    o.l.assign(size_t(total), 0.0f), o.r.assign(size_t(total), 0.0f);
    if (at >= 0) in[size_t(at)] = 0.5f;
    for (int k = 0; k * 512 < total; k++) {
        const int a = k * 512, n = std::min(512, total - a);
        if (during) during(*host, p, a);
        const float* ins[2] = {in.data() + a, z.data() + a};
        host->process(ins, o.l.data() + a, o.r.data() + a, n);
    }
    return o;
}
static int onset(const std::vector<float>& v, size_t from = 0, float frac = 0.1f) {
    float pk = 0;
    for (size_t i = from; i < v.size(); i++) pk = std::max(pk, std::fabs(v[i]));
    for (size_t i = from; i < v.size(); i++)
        if (std::fabs(v[i]) > frac * pk) return int(i);
    return -1;
}
static double energy(const std::vector<float>& v, double t0, double t1) {
    double e = 0;
    for (size_t i = size_t(t0 * 48000); i < std::min(v.size(), size_t(t1 * 48000)); i++) e += double(v[i]) * v[i];
    return e;
}

// 1 kHz octave band (two RBJ band-pass biquads at 20 kHz) over the four outputs; the decay of 50 ms
// block energies fitted from 5 dB under the peak to 25 dB under it (or 10 dB over the floor).
static double band_rt(const std::vector<std::array<int, 4>>& out, double T) {
    const double fs = 20000, f0 = 1000, q = 1.414, w = 2 * M_PI * f0 / fs, al = std::sin(w) / (2 * q);
    const double b0 = al, b2 = -al, a0 = 1 + al, a1 = -2 * std::cos(w), a2 = 1 - al;
    const size_t blk = size_t(std::clamp(T / 40.0, 0.004, 0.05) * 20000);   // 40 blocks per TIME, 4-50 ms
    std::vector<double> e(out.size() / blk + 1, 0.0);
    for (int ch = 0; ch < 4; ch++) {
        double z[2][2] = {};
        for (size_t i = 0; i < out.size(); i++) {
            double x = out[i][size_t(ch)];
            for (auto& s : z) {
                const double y = (b0 / a0) * x + s[0];
                s[0] = -(a1 / a0) * y + s[1];
                s[1] = (b2 / a0) * x - (a2 / a0) * y;
                x = y;
            }
            e[i / blk] += x * x;
        }
    }
    std::vector<double> db;
    for (double v : e) db.push_back(10 * std::log10(v + 1e-9));
    db.pop_back();
    const size_t ip = size_t(std::max_element(db.begin(), db.end()) - db.begin());
    std::vector<double> tail(db.begin() + long(db.size() * 4 / 5), db.end());
    std::sort(tail.begin(), tail.end());
    const double floor = tail[tail.size() / 2], lo = std::max(db[ip] - 25, floor + 10);
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    int n = 0;
    for (size_t i = ip; i < db.size(); i++) {
        if (db[i] < lo) break;
        if (db[i] > db[ip] - 5) continue;
        const double tt = double(i) * double(blk) / 20000.0;
        sx += tt, sy += db[i], sxx += tt * tt, sxy += tt * db[i], n++;
    }
    if (n < 3) return -1;
    const double slope = (n * sxy - sx * sy) / (n * sxx - sx * sx);
    return -60.0 / slope;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s ROM_DIR\n", argv[0]);
        return 2;
    }
    const auto& t = qrsplug::machine_type();
    auto roms = load_roms(t, argv[1]);
    std::printf("ROMs: %s\n", roms.describe(t.info.roms).c_str());
    if (!roms.complete()) return 2;
    int fails = 0;
    auto check = [&](bool ok, const std::string& what) {
        std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
        if (!ok) fails++;
    };

    std::printf("1. readouts\n");
    {
        kit_test::Params p(t.info);
        kit::Host host(t, p.ptrs());
        std::string err;
        host.prepare(48000, 512);
        host.install(t.boot(roms, "", &err));
        auto show = [&](const char* what) {
            auto r = host.readouts();
            std::printf("  %s: panel \"%s\"\n   ", what, r.panel.c_str());
            for (size_t i = 0; i < r.param.size(); i++) std::printf(" %s=%s;", t.info.params[i].id.c_str(), r.param[i].c_str());
            std::printf("\n   status: %s\n", r.status.c_str());
            return r;
        };
        auto r = show("defaults");
        check(r.param[Time] == "2.0 s", "default reverb time 2.0 s: " + r.param[Time]);
        check(r.param[ReflLevel] == "off", "1st reflection off by default");
        p.v[Room].store(0.0f);
        p.v[Time].store(60.0f);
        p.v[High].store(0.0f);
        host.sync();
        r = show("room 1 m3, time 10 s asked");
        check(r.param[Time] == "1.0 s (the room's limit)", "1 m3 limits the time to 1.0 s: " + r.param[Time]);
        p.v[Room].store(6.0f);
        p.v[Time].store(52.0f);
        host.sync();
        r = show("10^6 m3, 4.0 s");
        check(r.param[Time] == "4.0 s", "R20 display 4.0 s: " + r.param[Time]);
        p.v[Time].store(42.0f);
        host.sync();
        check(host.readouts().param[Time] == "1.25 s (the room's limit)" || host.readouts().param[Time] == "1.25 s",
              "10^6 m3, r 42: " + host.readouts().param[Time] + " (the COP's floor is r 40 = 1.0 s)");
    }

    std::printf("2. factory programs: 1 kHz-octave reverb time against TIME (core at 20 kHz)\n");
    {
        qrs::Roms rr;
        auto copy = [&](int i, std::array<uint8_t, 2048>& out) { std::copy(roms.bytes(size_t(i)).begin(), roms.bytes(size_t(i)).end(), out.begin()); };
        copy(0, rr.my), copy(1, rr.sip), copy(2, rr.dy);
        // the machine repairs the SIP dump at boot; here the same (the masks are in qrs_machine.cpp)
        const std::pair<int, uint8_t> rep[7] = {{0x440, 0xc8}, {0x460, 0x5f}, {0x4f0, 0x0d}, {0x540, 0xf0}, {0x560, 0x41}, {0x6ef, 0x02}, {0x7ef, 0x02}};
        if (roms.slots[1].label.find("repaired") == std::string::npos)
            for (auto& [at, m] : rep) rr.sip[size_t(at)] ^= m;
        std::string err;
        auto mach = t.boot(roms, "", &err);
        // the QRS project's measurement of the same programs (notes 2026-11-12-2330, 1 kHz column; 0 = n/a)
        const double project[14] = {2.92, 2.23, 2.92, 3.46, 4.74, 12.32, 38.1, 0, 0, 0.57, 0.92, 0, 0, 0.64};
        int agree = 0, compared = 0, k = 0;
        for (auto& pr : mach->presets()) {
            const int room = int(pr.values[Room]), r = int(pr.values[Time]);
            const double T = std::pow(10.0, (r - 20) / 20.0) * 0.1;
            int rec[8];
            qrs::panel_record(rec, room, r, int(pr.values[Low]) - 5, int(pr.values[High]) - 5, 1, 1, 0, -99);
            auto core = std::make_unique<qrs::Core>();
            core->init(rr);
            core->set(qrs::record_settings(rec));
            const size_t frames = size_t((0.6 * T + 0.6) * 20000);
            std::vector<std::array<int, 4>> out(frames);
            for (size_t i = 0; i < frames; i++) {
                int dac[4];
                core->frame(i == 1 ? 16384 : 0, 0, dac);
                out[i] = {dac[0], dac[1], dac[2], dac[3]};
            }
            const double rt = band_rt(out, T), ref = project[k++];
            const bool ok = ref == 0 || std::fabs(rt / ref - 1) < 0.06;
            std::printf("    %-34s TIME %6.2f s  measured %6.2f s  project %s %s\n", pr.name.c_str(), T, rt,
                        ref ? (std::to_string(ref).substr(0, 5) + " s").c_str() : "n/a   ", ok ? "" : "(differs)");
            if (ref) agree += ok, compared++;
        }
        check(agree == compared, std::to_string(agree) + " of " + std::to_string(compared) +
                                     " agree with the project's Python measurement within 6 % (block sizes differ)");
    }

    std::printf("3. an impulse through the host at 48 kHz\n");
    {
        kit_test::Params p20(t.info), p120(t.info), prefl(t.info);
        p120.v[RevDelay].store(120.0f);
        prefl.v[ReflLevel].store(0.0f);
        prefl.v[ReflDelay].store(50.0f);
        prefl.v[RevLevel].store(float(-31));
        auto a = impulse(t, roms, p20, 1.5), b = impulse(t, roms, p120, 1.5), c = impulse(t, roms, prefl, 0.5);
        const double shift = (onset(b.l) - onset(a.l)) / 48.0;
        check(std::fabs(shift - 100.0) < 2.0, "reverb delay 20 -> 120 ms moves the reverb by " + std::to_string(shift) + " ms");
        const double refl_ms = (onset(c.r) - 100 - c.latency) / 48.0;
        check(std::fabs(refl_ms - 50.0) < 1.0, "1st reflection 50 ms, reverb off: input L's echo on the right at " +
                                                   std::to_string(refl_ms) + " ms after the latency (" + std::to_string(c.latency) + ")");
        check(energy(c.l, 0, 0.5) < 1e-6 * energy(c.r, 0, 0.5), "and nothing on the left");
        std::printf("  reverb onset %.1f ms after the latency at reverb delay 20 ms (room 10^3 m3)\n",
                    (onset(a.l) - 100 - a.latency) / 48.0);
        double pk = 0;
        for (float x : a.l) pk = std::max(pk, double(std::fabs(x)));
        std::printf("  peak output for a 0.5 impulse: %.4f (%.1f dB)\n", pk, 20 * std::log10(pk / 0.5));
        // freeze: loaded for 0.5 s, then held with the input closed; mute clears
        kit_test::Params pf(t.info);
        pf.v[Time].store(60.0f);
        auto f = impulse(t, roms, pf, 4.0, 100, [](kit::Host& h, kit_test::Params& p, int at) {
            if (at >= 24000 && p.v[Freeze].load() < 0.5f) p.v[Freeze].store(1.0f), h.sync();
            if (at >= 3 * 48000 && p.v[Mute].load() < 0.5f) p.v[Mute].store(1.0f), h.sync();
        });
        const double e1 = energy(f.l, 1.0, 1.5), e2 = energy(f.l, 2.5, 3.0);
        check(e1 > 0 && std::fabs(10 * std::log10(e2 / e1)) < 1.5,
              "freeze holds: 1.0-1.5 s vs 2.5-3.0 s " + std::to_string(10 * std::log10(e2 / e1)) + " dB");
        check(energy(f.l, 3.1, 4.0) < 1e-9 * e1, "mute clears the memory (and the output stays silent)");
    }

    std::printf("4. the hardware panel\n");
    fails += kit_test::run_panel_layout_checks(t, roms);
    {
        kit_test::Params p(t.info);
        kit::Host* hp = nullptr;
        kit_test::render(t, roms, p, 48000, 0.3, [](int) { return 256; }, &hp);
        std::unique_ptr<kit::Host> h(hp);
        kit_test::press(*h, {2 * qrsplug::PRoom + 2});   // room size [+]
        check(kit_test::adopt(*h, p) && std::lround(p.v[Room].load()) == 4, "room size [+]: 10^3 -> 10^4 m3");
        kit_test::run_audio(*h, 0.1);
        auto s = h->panel();
        check(s.digits[3 * qrsplug::PRoom + 0] == 0x06 && s.digits[3 * qrsplug::PRoom + 1] == (0x3F | 0x80) &&
                  s.digits[3 * qrsplug::PRoom + 2] == 0x66,
              "the room display shows 10.4");
        h->panel_input(qrsplug::kKnobInput, 0.5f);
        h->panel_input(qrsplug::kKnobInput, 0.6f);   // the knob, now on room size: +6 steps, held at 10^6
        kit_test::run_audio(*h, 0.1);
        check(kit_test::adopt(*h, p) && std::lround(p.v[Room].load()) == 6, "the knob follows the last key: room size up to 10^6 m3");
        kit_test::press(*h, {qrsplug::KeyStar, qrsplug::KeyMem1 + 7, qrsplug::KeyMem1 + 5});   // * 8 6: file 8, loc 6
        check(kit_test::adopt(*h, p) && std::lround(p.v[Room].load()) == 2 && std::lround(p.v[Time].load()) == 32,
              "memory * 8 6 recalls 8.6 (living room: 10^2 m3, 0.4 s)");
        kit_test::press(*h, {qrsplug::KeyFreeze});
        check(kit_test::adopt(*h, p) && p.v[Freeze].load() > 0.5f && h->panel().leds[qrsplug::LedFreeze] == 255,
              "FRE turns freeze on, and its LED");
    }

    std::printf("5. host checks\n");
    fails += kit_test::run_host_checks(t, roms, Room, 5.0f, 0.0);

    std::printf("6. a live change while audio runs\n");
    {
        kit_test::Params p(t.info);
        kit::Host* h = nullptr;
        kit_test::render(t, roms, p, 48000, 1.0, [](int) { return 256; }, &h);
        std::unique_ptr<kit::Host> host(h);
        p.v[Time].store(30.0f);
        p.v[High].store(1.0f);
        host->sync();
        std::vector<float> z(256, 0.0f), ol(256), orr(256);
        const float* zi[2] = {z.data(), z.data()};
        kit_test::allocations = 0;
        kit_test::counting = true;
        for (int k = 0; k < 40; k++) host->process(zi, ol.data(), orr.data(), 256);
        kit_test::counting = false;
        check(kit_test::allocations == 0, "taking new settings does not allocate");
    }

    std::printf("7. cost: 20 s of audio at 48 kHz in 512-sample blocks\n");
    {
        kit_test::Params p(t.info);
        auto t0 = std::chrono::steady_clock::now();
        kit_test::render(t, roms, p, 48000, 20.0, [](int) { return 512; });
        double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        std::printf("  %.2f s of CPU on one thread: %.1fx real time\n", s, 20.0 / s);
    }

    // ---- 8. ROM handling ----
    std::printf("8. ROM handling (images made here from yours by flipping bits)\n");
    using Img = std::vector<uint8_t>;
    const std::pair<int, uint8_t> rep[7] = {{0x440, 0xc8}, {0x460, 0x5f}, {0x4f0, 0x0d}, {0x540, 0xf0}, {0x560, 0x41}, {0x6ef, 0x02}, {0x7ef, 0x02}};
    const Img my0 = roms.bytes(0), dy0 = roms.bytes(2);
    Img chip = roms.bytes(1), dump;
    if (roms.slots[1].label.find("repaired") == std::string::npos) {
        dump = chip;
        for (auto& [at, m] : rep) chip[size_t(at)] ^= m;
    } else {
        dump = chip;
        for (auto& [at, m] : rep) dump[size_t(at)] ^= m;
    }
    auto set_of = [](const Img& a, const Img& b, const Img& c) {
        kit::RomSet s;
        s.slots.resize(3);
        const Img* im[3] = {&a, &b, &c};
        for (int i = 0; i < 3; i++) s.slots[size_t(i)] = {true, false, "synthetic", {"synthetic", "synthetic", *im[i]}};
        return s;
    };
    auto boot_note = [&](const kit::RomSet& s, std::string& err) {
        auto m = t.boot(s, "", &err);
        if (!m) return std::string("(refused)");
        std::vector<float> v;
        for (auto& prm : t.info.params) v.push_back(prm.def);
        m->set_params(v);
        return m->readouts().status;
    };
    auto out_of = [&](const kit::RomSet& s) {
        kit_test::Params p(t.info);
        p.v[ReflLevel].store(0.0f);
        return kit_test::render(t, s, p, 48000, 0.4, [](int) { return 512; });
    };
    {
        const auto ref = out_of(set_of(my0, chip, dy0));
        auto expect = [&](const char* what, const kit::RomSet& s, std::initializer_list<const char*> has, bool same_as_chip) {
            std::string err;
            const std::string note = boot_note(s, err);
            bool ok = err.empty();
            for (const char* h : has) ok = ok && note.find(h) != std::string::npos;
            if (ok && same_as_chip) ok = out_of(s) == ref;
            check(ok, std::string(what) + ": \"" + note + "\"" + (same_as_chip ? ", output = the chip's" : ""));
        };
        expect("the known dump", set_of(my0, dump, dy0), {"7 rotted bytes repaired"}, true);
        expect("the repaired chip", set_of(my0, chip, dy0), {}, true);
        {
            std::string err;
            check(boot_note(set_of(my0, chip, dy0), err).empty(), "  and it gives no note");
        }
        Img part = chip;
        part[0x440] &= uint8_t(~0x40);   // one bit of 0x440's rot
        part[0x6ef] &= uint8_t(~0x02);
        part[0x540] ^= 0xf0;              // 0x540 as in the dump (not a pure loss)
        expect("partial rot (2 pure losses, 0x540 as in the dump)", set_of(my0, part, dy0),
               {"3 of the known dump's 7", "0x440", "0x6ef", "0x540 (as in the known dump)"}, true);
        Img extra = chip;
        extra[0x123] ^= 0x10;   // page 2
        expect("the chip with a bit rotted elsewhere", set_of(my0, extra, dy0), {"not a known dump", "1 of 16 pages", "groups 2)"}, false);
        Img dx = dump;
        dx[0x123] ^= 0x10;
        expect("the dump with a bit rotted elsewhere", set_of(my0, dx, dy0), {"6 of 16 pages", "groups 2, 8, 9, 10, 13 and 15"}, false);
        expect("MY and DY in each other's slots", set_of(dy0, chip, my0), {"MY U426 image was in the DY U633 slot", "taken by its contents"}, true);
        {
            const auto want = out_of(set_of(my0, extra, dy0));
            std::string err;
            const auto s = set_of(extra, dy0, my0);   // all three moved, the SIP unknown: placed as the one left over
            const std::string note = boot_note(s, err);
            check(err.empty() && note.find("SIP U508 image was in the MY U426 slot") != std::string::npos && out_of(s) == want,
                  "all three moved, the SIP not a known dump: \"" + note + "\"");
        }
        {
            std::string err;
            boot_note(set_of(dy0, chip, dy0), err);
            check(err.find("both look like the DY U633") != std::string::npos, "two DY images: refused: \"" + err + "\"");
            Img shortmy(my0.begin(), my0.end() - 1);
            err.clear();
            boot_note(set_of(shortmy, chip, dy0), err);
            check(err.find("2047 bytes") != std::string::npos, "a 2047-byte image: refused: \"" + err + "\"");
        }
        Img myx = my0;
        myx[2 * 128 + 5] ^= 0x01;
        expect("MY with a byte changed in room 2's table", set_of(myx, chip, dy0), {"MY U426 is not the known chip (room tables 2 differ)"}, false);
        Img dyx = dy0;
        dyx[0x100] ^= 0x01;
        expect("DY with a byte changed at 0x100", set_of(my0, chip, dyx), {"DY U633 is not the known chip (pages 2 of 16"}, false);
    }

    // ---- 9. behaviour ----
    std::printf("9. behaviour\n");
    {
        // a reverb time change while the tail rings: the ring is kept, the coefficients switch at once
        kit_test::Params pa(t.info), pb(t.info);
        pa.v[Time].store(60.0f), pb.v[Time].store(60.0f);
        auto a = impulse(t, roms, pa, 2.5);
        auto b = impulse(t, roms, pb, 2.5, 100, [](kit::Host& h, kit_test::Params& p, int at) {
            if (at >= 48000 && p.v[Time].load() > 50) p.v[Time].store(30.0f), h.sync();
        });
        const double near = 10 * std::log10(energy(b.l, 1.0, 1.03) / energy(a.l, 1.0, 1.03));
        const double late = 10 * std::log10(energy(b.l, 2.0, 2.5) / energy(a.l, 2.0, 2.5));
        check(near > -10, "reverb time 10 s -> 0.32 s at 1.0 s: the tail is kept (first 30 ms " + fmt1(near) +
                              " dB, the new decay already acting)");
        check(late < -30, "  and then dies faster (2.0-2.5 s " + fmt1(late) + " dB against no change)");
        // a room change: the output mutes for 0.5 s (manual p 15), the memory is kept, nothing blows up
        kit_test::Params pr(t.info);
        pr.v[Time].store(60.0f);
        auto r = impulse(t, roms, pr, 2.5, 100, [](kit::Host& h, kit_test::Params& p, int at) {
            if (at >= 48000 && p.v[Room].load() < 3.5f) p.v[Room].store(4.0f), h.sync();
        });
        float pk = 0;
        bool finite = true;
        for (float x : r.l) pk = std::max(pk, std::fabs(x)), finite = finite && std::isfinite(x);
        check(energy(r.l, 1.02, 1.5) == 0, "room 10^3 -> 10^4 m3 at 1.0 s: the output mutes for 0.5 s");
        check(energy(r.l, 1.6, 2.0) > 0 && finite && pk < 1.0f, "  then the kept memory rings on in the new room (peak " + fmt1(20 * std::log10(pk)) + " dBFS)");
        // a memory recall from the panel: a real unit silences the output and clears the reverb (two
        // recordings, quantec notes 2026-11-28-0314), also when the room stays the same. 7.6 (church,
        // 10^5 m3, 4.5 s) rings, then * 7 6 recalls it again at 1.0 s.
        auto church = [](kit::Host& h, kit_test::Params& p, int at, bool recall) {
            if (at == 0) {
                kit_test::press(h, {qrsplug::KeyStar, qrsplug::KeyMem1 + 6, qrsplug::KeyMem1 + 5}, 0.0, 0.0);
                kit_test::adopt(h, p);
                h.sync();
            }
            if (recall && at >= 48000 && at < 48512) kit_test::press(h, {qrsplug::KeyStar, qrsplug::KeyMem1 + 6, qrsplug::KeyMem1 + 5}, 0.0, 0.0);
        };
        kit_test::Params pk1(t.info), pk2(t.info);
        auto keep = impulse(t, roms, pk1, 2.5, 100, [&](kit::Host& h, kit_test::Params& p, int at) { church(h, p, at, false); });
        auto rec = impulse(t, roms, pk2, 2.5, 100, [&](kit::Host& h, kit_test::Params& p, int at) { church(h, p, at, true); });
        check(std::lround(pk2.v[Room].load()) == 5 && energy(keep.l, 1.0, 1.5) > 0, "memory * 7 6 loads the church (10^5 m3), which rings past 1.0 s");
        check(energy(rec.l, 1.02, 1.5) == 0, "recalling it again at 1.0 s mutes the output for 0.5 s");
        const double after = 10 * std::log10((energy(rec.l, 1.6, 2.5) + 1e-30) / energy(keep.l, 1.6, 2.5));
        check(after < -60, "  and the reverb is cleared: 1.6-2.5 s " + fmt1(after) + " dB against no recall");
    }
    {
        // extreme settings through the core at 20 kHz: 1 s of noise, then 14 s of silence
        qrs::Roms rr, raw;
        std::copy(my0.begin(), my0.end(), rr.my.begin()), std::copy(chip.begin(), chip.end(), rr.sip.begin()), std::copy(dy0.begin(), dy0.end(), rr.dy.begin());
        raw = rr;
        std::copy(dump.begin(), dump.end(), raw.sip.begin());
        auto run = [](const qrs::Roms& R, const qrs::Settings& st, double& early, double& late, double& clip) {
            auto core = std::make_unique<qrs::Core>();
            core->init(R);
            core->set(st);
            uint32_t seed = 1;
            early = late = 0;
            long nclip = 0, nlate = 0;
            for (int i = 0; i < 15 * 20000; i++) {
                seed = seed * 1664525u + 1013904223u;
                const int x = i < 20000 ? int(seed >> 16) % 8000 - 4000 : 0;
                int dac[4];
                core->frame(x, -x, dac);
                const int d1 = -dac[1] - 1;   // output 2 leaves one's-complement inverted: 0 is -1
                const double e = double(dac[0]) * dac[0] + double(d1) * d1;
                if (i >= 20000 && i < 3 * 20000) early += e;
                if (i >= 13 * 20000) late += e, nlate++, nclip += std::abs(dac[0]) >= 32000;
            }
            clip = double(nclip) / double(nlate);
        };
        auto settings = [](int room, int r, int low, int high, int revdb, int refldb, int revms = 1, int reflms = 1) {
            int rec[8];
            qrs::panel_record(rec, room, r, low, high, revms, reflms, revdb, refldb);
            return qrs::record_settings(rec);
        };
        for (int rawsip = 0; rawsip < 2; rawsip++)
            for (auto [lo, hi] : {std::pair{5, 2}, std::pair{-5, -5}, std::pair{0, 0}}) {
                double e, l, c;
                run(rawsip ? raw : rr, settings(6, 80, lo, hi, 0, -99), e, l, c);
                const double ratio = 10 * std::log10((l + 1) / (e + 1));
                check(e > 0 && ratio < 0 && c < 0.001, std::string("10^6 m3, 100 s, LOW ") + std::to_string(lo) + " HIGH " + std::to_string(hi) +
                                                           (rawsip ? ", SIP as dumped" : ", SIP repaired") + ": 13-15 s is " + fmt1(ratio) +
                                                           " dB against 1-3 s (no growth)");
            }
        {
            double e, l, c;
            run(rr, settings(0, 20, -5, -5, 0, -99), e, l, c);
            const double rms = std::sqrt(l / (2 * 2 * 20000.0));
            check(e > 0 && rms < 1.0, "1 m3, 0.1 s, LOW and HIGH 0.1: dies away to " + fmt1(rms) +
                                          " LSB rms (the floor rounding's idle pattern)");
        }
        // the SIP's HIGH / LOW registers at their 4-bit limits (the panel reaches -5..+2 / -5..+5): reported
        for (auto [l4, h4] : {std::pair{7, 7}, std::pair{8, 8}, std::pair{7, 8}, std::pair{8, 7}}) {
            qrs::Settings st = settings(6, 80, 0, 0, 0, -99);
            st.l = l4, st.h = h4;
            double e, l, c;
            run(rr, st, e, l, c);
            std::printf("    registers L %+d H %+d (beyond the panel): 13-15 s %s dB against 1-3 s, %.1f %% at full scale\n",
                        l4 > 7 ? l4 - 16 : l4, h4 > 7 ? h4 - 16 : h4, fmt1(10 * std::log10((l + 1) / (e + 1))).c_str(), 100 * c);
        }
        {
            double e, l, c;
            run(rr, settings(3, 60, 0, 0, -99, -99), e, l, c);
            check(e == 0 && l == 0, "reverb and 1st reflection both off: outputs 1 and 2 silent (output 2 at its inverted zero)");
        }
    }
    {
        kit_test::Params p1(t.info), p200(t.info);
        for (auto* p : {&p1, &p200}) p->v[RevLevel].store(float(-31)), p->v[ReflLevel].store(0.0f);
        p1.v[ReflDelay].store(1.0f), p200.v[ReflDelay].store(200.0f);
        auto a = impulse(t, roms, p1, 0.5), b = impulse(t, roms, p200, 0.5);
        const double ta = (onset(a.r) - 100 - a.latency) / 48.0, tb = (onset(b.r) - 100 - b.latency) / 48.0;
        check(std::fabs(tb - ta - 199.0) < 0.1, "1st reflection 1 ms -> 200 ms: onsets " + fmt1(ta) + " and " + fmt1(tb) + " ms after the latency");
    }

    // ---- 10. host rates and block sizes ----
    std::printf("10. host rates and block sizes\n");
    {
        std::vector<double> at;
        for (int rate : {44100, 48000, 88200, 96000, 192000}) {
            kit_test::Params p(t.info);
            p.v[RevLevel].store(float(-31)), p.v[ReflLevel].store(0.0f), p.v[ReflDelay].store(10.0f);
            auto o = impulse(t, roms, p, 0.2, 100, nullptr, rate);
            const double ms = (onset(o.r) - 100 - o.latency) * 1000.0 / rate;
            at.push_back(ms);
            std::printf("    %6d Hz: latency %ld samples, 1st reflection (10 ms) %.3f ms after it\n", rate, o.latency, ms);
        }
        const double spread = *std::max_element(at.begin(), at.end()) - *std::min_element(at.begin(), at.end());
        check(spread < 1000.0 / 44100 + 0.02, "the reported latency is the bridge's at every rate: onsets agree within " + fmt1(spread * 1000) + " us");
    }
    {
        kit_test::Params pa(t.info), pb(t.info), pc(t.info), pd(t.info);
        auto a = kit_test::render(t, roms, pa, 48000, 0.5, [](int) { return 512; }, nullptr, 4096);
        auto b = kit_test::render(t, roms, pb, 48000, 0.5, [](int) { return 1; }, nullptr, 4096);
        auto c = kit_test::render(t, roms, pc, 48000, 0.5, [](int) { return 4096; }, nullptr, 4096);
        std::mt19937 bs(3);
        auto d = kit_test::render(t, roms, pd, 48000, 0.5, [&](int) { return int(bs() % 4096) + 1; }, nullptr, 4096);
        check(a == b && a == c && a == d, "blocks of 1, 4096 and random 1-4096 = 512-sample blocks, bit for bit");
    }

    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
