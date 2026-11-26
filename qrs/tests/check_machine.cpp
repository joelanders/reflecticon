// check_machine ROM_DIR: the QRS machine, JUCE-free.
//   1. readouts for the defaults and a few settings;
//   2. the factory programs: an impulse through the core (20 kHz, no converters), the 1 kHz-octave
//      reverb time, against the QRS project's measurement of its Python model;
//   3. an impulse through the host at 48 kHz: reverb delay, the 1st reflection (swapped to the right),
//      freeze holds, mute clears;
//   4. the hardware panel: layout, a [+] key, the knob, a memory recall;
//   5. the kit's host checks (block sizes, no allocation, a change reaches the machine, instances);
//   6. a live change while audio runs; 7. what it costs.
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

enum { Room, Time, High, Low, RevDelay, RevLevel, ReflDelay, ReflLevel, Freeze, Enter, Mute, Input, Output, Outputs, Polarity };

// An impulse (0.5 full scale) into the left input through a fresh host at 48 kHz: both outputs.
struct Run {
    std::vector<float> l, r;
    long latency = 0;
};
static Run impulse(const kit::MachineType& t, const kit::RomSet& roms, kit_test::Params& p, double seconds, int at = 100,
                   std::function<void(kit::Host&, kit_test::Params&, int)> during = nullptr) {
    auto host = std::make_unique<kit::Host>(t, p.ptrs());
    std::string err;
    auto m = t.boot(roms, "", &err);
    if (!m) {
        std::fprintf(stderr, "boot failed: %s\n", err.c_str());
        std::exit(1);
    }
    host->prepare(48000, 512);
    host->install(std::move(m));
    Run o;
    o.latency = host->latency();
    int total = int(seconds * 48000);
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

    std::printf(fails ? "\n%d FAILED\n" : "\nall passed\n", fails);
    return fails ? 1 : 0;
}
