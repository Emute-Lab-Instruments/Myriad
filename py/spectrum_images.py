#!/usr/bin/env python3
"""
Generate frequency spectrum images for Myriad oscillator models.

Each image shows magnitude spectra at 5 ctrl() values, 50 Hz fundamental.
Approach: simulate one period of the amp(t) signal inside each model's
delta-sigma loop (4096 samples = one period at virtual sample rate
4096 × 50 Hz = 204.8 kHz).  The filtered bitstream output converges to
amp(t), so its spectrum is the audio spectrum.

Noise models are averaged over 16 independent instances.
"""

import numpy as np
import matplotlib.pyplot as plt
import math
from pathlib import Path

OUT_DIR       = Path("/home/ck84/src/www.eli/assets/images/myriad/waveforms")
OUT_DIR.mkdir(parents=True, exist_ok=True)

FUNDAMENTAL   = 50.0          # Hz
WAVELEN       = 4096          # samples per period
F_MAX         = 20_000        # Hz shown on x-axis
N_NOISE_AVG   = 16            # instances averaged for noise models
CTRL_VALUES   = [0.0, 0.25, 0.5, 0.75, 1.0]

# ── Wavetable builders ────────────────────────────────────────────────────────

def _psin_bell(ph):
    ph = int(ph) % 2048
    p  = ph if ph < 1024 else 2047 - ph
    v  = (p * (1024 - p)) >> 8
    return v if ph < 1024 else -v

def _build_bell_tables():
    raw_a, raw_b = np.zeros(512), np.zeros(512)
    for i in range(512):
        carrier   = i * 4
        mod_phase = (i * 14) % 2048
        mod_val   = _psin_bell(mod_phase)
        raw_a[i]  = _psin_bell(carrier + ((mod_val * 3) >> 1))
        raw_b[i]  = _psin_bell(carrier + (mod_val << 4))
    def _n(t):
        mn, mx = t.min(), t.max(); r = (mx - mn) or 1.0
        return (t - mn) * 2048.0 / r
    return _n(raw_a), _n(raw_b)

def _build_formant_tables():
    aw = [0, 0.39, 1.0, 0.75, 0.2, 0.1, 0.5, 0.2, 0.06]
    ew = [0, 1.0,  0.2, 0.1, 0.05, 0.2, 0.75, 0.5, 0.38]
    raw_a, raw_b = np.zeros(512), np.zeros(512)
    for i in range(512):
        sa = sb = 0.0
        for h in range(1, 9):
            s  = math.sin(h * i * 2 * math.pi / 512)
            sa += s * aw[h]; sb += s * ew[h]
        raw_a[i] = sa; raw_b[i] = sb
    def _n(t):
        mn, mx = t.min(), t.max(); r = (mx - mn) or 1.0
        return (t - mn) * 2048.0 / r
    return _n(raw_a), _n(raw_b)

def _build_parasine_table():
    t = np.arange(256)
    return 1024.0 + 1024.0 * np.sin(2 * np.pi * t / 256)

BELL_A,    BELL_B    = _build_bell_tables()
FORMANT_A, FORMANT_B = _build_formant_tables()
PARASINE   = _build_parasine_table()

# ── Amplitude-signal functions ────────────────────────────────────────────────
# Each returns a float64 array of length `wavelen` representing one period.
# Noise models accept an optional `seed` kwarg for averaging.

def amp_saw(wavelen, ctrl_v, **_):
    pm    = 5.0 + 40.0 * ctrl_v
    phase = np.arange(wavelen, dtype=np.float64)
    amp   = phase * pm
    amp[amp >= wavelen] = 0.0
    return amp / wavelen

def amp_expPulse1(wavelen, ctrl_v, **_):
    vinv      = 1.0 - ctrl_v
    tInc0     = vinv * (wavelen / 32.0) + 10.0
    targMul   = vinv * 0.4 + 1.0
    amps      = np.zeros(wavelen)
    counter   = 0.0; target = tInc0; tInc = tInc0; b1 = 0
    for i in range(wavelen):
        if counter >= target:
            b1 = 1 - b1; counter = 0.0; target += tInc; tInc *= targMul
        amps[i] = float(b1); counter += 1.0
    return amps

def amp_sharkTeeth(wavelen, ctrl_v, **_):
    bite = max(1, int(1.0 + ctrl_v * 19.0))
    half = wavelen // 2
    amps = np.zeros(wavelen); tp = 0.0
    for i in range(wavelen):
        if tp >= wavelen: tp -= wavelen
        amps[i] = tp if i < half else 0.0
        tp += bite
    return amps / wavelen

def amp_pulsePW(wavelen, ctrl_v, **_):
    pw    = 0.5 - 0.38 * ctrl_v
    phase = np.arange(wavelen, dtype=np.float64)
    return np.where(phase < wavelen * pw, 1.0, 0.0)

def amp_expPulse2(wavelen, ctrl_v, **_):
    vinv    = 1.0 - ctrl_v
    tInc0   = vinv * (wavelen / 16.0) + 3.0
    targMul = 1.0 - vinv * 0.8
    amps    = np.zeros(wavelen)
    counter = 0.0; target = tInc0; tInc = tInc0; b1 = 0
    for i in range(wavelen):
        if counter >= target:
            b1 = 1 - b1; counter = 0.0
            target += tInc; tInc *= targMul; tInc *= targMul
        amps[i] = float(b1); counter += 1.0
    return amps

def amp_tri(wavelen, ctrl_v, **_):
    v_tab      = int(ctrl_v * 0.9 * 1000) / 1000.0
    gradR      = 2.0 + v_tab * 7.0
    peak       = wavelen / gradR
    peakAmp    = float(wavelen)
    gradF      = peakAmp / (wavelen - peak) if wavelen > peak else 1.0
    amps       = np.zeros(wavelen)
    for i in range(wavelen):
        if i <= peak:
            amps[i] = i * gradR
        else:
            amps[i] = max(0.0, peakAmp - (i - peak) * gradF)
    return amps / wavelen

def amp_triTeeth(wavelen, ctrl_v, **_):
    bite      = max(2, int(2.0 + ctrl_v * 24.0) * 2)
    amps      = np.zeros(wavelen)
    tri       = 0; triInc = 2
    tooth     = 0.0; toothInc = float(bite)
    for i in range(wavelen):
        if tri > wavelen:   tri   = 2 * wavelen - tri;    triInc   = -triInc
        elif tri < 0:       tri   = -tri;                  triInc   = -triInc
        if tooth >= wavelen: tooth = 2.0*wavelen - tooth;  toothInc = -toothInc
        elif tooth < 0.0:   tooth = -tooth;                toothInc = -toothInc
        amps[i] = (tri if tooth < tri else int(tooth)) / wavelen
        tri   += triInc; tooth += toothInc
    return amps

def amp_parasine(wavelen, ctrl_v, **_):
    sp  = 1.0 - 0.1 * ctrl_v
    sn  = 1.0 - 0.9 * ctrl_v
    inc = 256.0 / wavelen
    amps = np.zeros(wavelen); ph = 0.0
    for i in range(wavelen):
        raw = PARASINE[int(ph) & 0xFF]; dev = raw - 1024.0
        amps[i] = (1024.0 + dev * (sp if dev >= 0 else sn)) / 2048.0
        ph += inc
    return amps

def amp_formant(wavelen, ctrl_v, **_):
    inc = 512.0 / wavelen
    amps = np.zeros(wavelen); ph = 0.0
    for i in range(wavelen):
        idx = int(ph) & 0x1FF
        amps[i] = (FORMANT_A[idx] + (FORMANT_B[idx] - FORMANT_A[idx]) * ctrl_v) / 2048.0
        ph += inc
    return amps

def amp_bell(wavelen, ctrl_v, **_):
    inc = 512.0 / wavelen
    amps = np.zeros(wavelen); ph = 0.0
    for i in range(wavelen):
        idx = int(ph) & 0x1FF
        amps[i] = (BELL_A[idx] + (BELL_B[idx] - BELL_A[idx]) * ctrl_v) / 2048.0
        ph += inc
    return amps

def amp_noiseSD(wavelen, ctrl_v, seed=42, **_):
    rng      = np.random.default_rng(seed)
    rm       = max(1, int(ctrl_v ** 2 * 500))
    amps     = np.zeros(wavelen)
    on       = False; counter = 0
    for i in range(wavelen):
        if counter == 0:
            on      = not on
            counter = 1 + int(wavelen * 0.01 * rng.integers(0, max(2, rm)))
        amps[i] = 1.0 if on else 0.0
        counter -= 1
    return amps

def amp_whiteNoise(wavelen, ctrl_v, seed=42, **_):
    rng    = np.random.default_rng(seed)
    bits   = rng.integers(0, 2, size=wavelen).astype(float)
    window = max(1, int(wavelen * 0.005 * (1.0 - ctrl_v * 0.8)))
    return np.convolve(bits, np.ones(window) / window, mode='same')

# ── Model registry ────────────────────────────────────────────────────────────

MODELS = [
    dict(idx=0,  id='saw',     name='Saw',        fn=amp_saw,
         ctrl_desc='ctrl → pulse width  (0 = wide ramp,  1 = narrow ramp)'),
    dict(idx=1,  id='exp1',    name='Chirp',       fn=amp_expPulse1,
         ctrl_desc='ctrl → pulse density  (0 = sparse,  1 = dense)'),
    dict(idx=2,  id='sdt10',   name='Sharkteeth',  fn=amp_sharkTeeth,
         ctrl_desc='ctrl → number of teeth  (0 = 1,  1 = 20)'),
    dict(idx=3,  id='pulsesd', name='Pulse',        fn=amp_pulsePW,
         ctrl_desc='ctrl → duty cycle  (0 = 50%,  1 = 12%)'),
    dict(idx=4,  id='slide',   name='Slide',        fn=amp_expPulse2,
         ctrl_desc='ctrl → slide speed  (0 = slow,  1 = fast)'),
    dict(idx=5,  id='tri',     name='Triangle',     fn=amp_tri,
         ctrl_desc='ctrl → asymmetry  (0 = symmetric,  1 = skewed)'),
    dict(idx=6,  id='trv10',   name='TriTeeth',     fn=amp_triTeeth,
         ctrl_desc='ctrl → tooth rate  (0 = slow,  1 = fast)'),
    dict(idx=7,  id='sinesd',  name='Sine',         fn=amp_parasine,
         ctrl_desc='ctrl → waveshape  (0 = symmetric,  1 = asymmetric)'),
    dict(idx=8,  id='formant', name='Formant',      fn=amp_formant,
         ctrl_desc='ctrl → morph  (0 = table A,  1 = table B)'),
    dict(idx=9,  id='metalic', name='Metallic',     fn=amp_bell,
         ctrl_desc='ctrl → morph  (0 = mellow FM,  1 = harsh FM)',
         legend_loc='lower left'),
    dict(idx=10, id='n2',      name='Noise',        fn=amp_noiseSD,       noise=True,
         ctrl_desc='ctrl → density  (0 = sparse,  1 = dense)'),
    dict(idx=11, id='wn',      name='White Noise',  fn=amp_whiteNoise,    noise=True,
         ctrl_desc='ctrl → character  (0 = smoother,  1 = rawer)'),
    dict(idx=12, id='sil',     name='Silent',       fn=None,
         ctrl_desc=''),
]

# ── Spectrum computation ───────────────────────────────────────────────────────

def _spectrum(amp):
    """One-period FFT → (freqs_hz, mag_db), normalised to 0 dB peak."""
    a   = np.asarray(amp, dtype=float)
    a  -= a.mean()                                 # remove DC
    mag = np.abs(np.fft.rfft(a))
    db  = 20.0 * np.log10(mag + 1e-12)
    db -= db[1:].max()                             # 0 dB = loudest harmonic
    freqs = np.arange(len(mag)) * FUNDAMENTAL      # bin k → k × 50 Hz
    return freqs, db

def get_spectrum(model, ctrl_v):
    fn    = model['fn']
    noise = model.get('noise', False)
    if noise:
        power = np.zeros(WAVELEN // 2 + 1)
        for seed in range(N_NOISE_AVG):
            a   = fn(WAVELEN, ctrl_v, seed=seed)
            a  -= np.mean(a)
            power += np.abs(np.fft.rfft(a)) ** 2
        power /= N_NOISE_AVG
        db    = 10.0 * np.log10(power + 1e-12)
        db   -= db[1:].max()
        freqs = np.arange(len(power)) * FUNDAMENTAL
        return freqs, db
    else:
        return _spectrum(fn(WAVELEN, ctrl_v))

# ── Plotting ──────────────────────────────────────────────────────────────────

BG      = '#0d1117'
GRID_C  = '#1c2128'
AXIS_C  = '#2d3748'
LABEL_C = '#8b949e'
CTRL_COLORS = ['#1a4a1a', '#2d7a2d', '#3aaa3a', '#57d557', '#7eff7e']

DB_MIN  = -80
DB_MAX  =   5

def plot_spectrum(m):
    if m['id'] == 'sil':
        _plot_silent(m); return

    fig, ax = plt.subplots(figsize=(9, 3.2), facecolor=BG)
    ax.set_facecolor(BG)
    ax.grid(True, which='both', color=GRID_C, linewidth=0.5, alpha=0.8)

    for cv, color in zip(CTRL_VALUES, CTRL_COLORS):
        freqs, db = get_spectrum(m, cv)
        mask = (freqs >= FUNDAMENTAL) & (freqs <= F_MAX)
        ax.plot(freqs[mask], np.clip(db[mask], DB_MIN, DB_MAX),
                color=color, linewidth=0.9, label=f'{cv:.2f}', alpha=0.92, zorder=2)

    ax.set_xscale('log')
    ax.set_xlim(FUNDAMENTAL, F_MAX)
    ax.set_ylim(DB_MIN, DB_MAX)

    ax.set_xticks([50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000])
    ax.set_xticklabels(['50', '100', '200', '500', '1k', '2k', '5k', '10k', '20k'],
                       color=LABEL_C, fontsize=8)
    ax.set_yticks([-80, -60, -40, -20, 0])
    ax.set_yticklabels(['-80', '-60', '-40', '-20', '0'], color=LABEL_C, fontsize=8)
    ax.set_xlabel('frequency (Hz)', color=LABEL_C, fontsize=8, labelpad=3)
    ax.set_ylabel('dB', color=LABEL_C, fontsize=8, labelpad=3)
    ax.tick_params(colors=LABEL_C, length=3, which='both')
    for sp in ax.spines.values():
        sp.set_edgecolor(GRID_C)

    # Harmonic reference lines at octaves of 50 Hz
    for f in [50, 100, 200, 400, 800, 1600, 3200, 6400, 12800]:
        if FUNDAMENTAL <= f <= F_MAX:
            ax.axvline(f, color='#1e2d1e', linewidth=0.5, zorder=0)

    leg = ax.legend(
        title='ctrl', title_fontsize=7,
        loc=m.get('legend_loc', 'upper right'), fontsize=7,
        framealpha=0.5, facecolor='#161b22', edgecolor=GRID_C,
        labelcolor=LABEL_C, handlelength=1.5,
    )
    leg.get_title().set_color(LABEL_C)

    ax.set_title(
        f'{m["name"]}  ·  50 Hz fundamental  ·  {m["ctrl_desc"]}',
        color='#c9d1d9', fontsize=9, pad=6, loc='left'
    )

    fig.tight_layout(pad=0.6)
    path = OUT_DIR / f"oscmodel{m['idx']}.png"
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f"  saved {path.name}")

def _plot_silent(m):
    fig, ax = plt.subplots(figsize=(9, 3.2), facecolor=BG)
    ax.set_facecolor(BG)
    ax.grid(True, which='both', color=GRID_C, linewidth=0.5)
    ax.axhline(-80, color='#3a5a3a', linewidth=1.2, label='output')
    ax.text(500, -40, '— silence —', color='#3a5a3a', fontsize=13,
            ha='center', va='center', style='italic')
    ax.set_xscale('log')
    ax.set_xlim(FUNDAMENTAL, F_MAX); ax.set_ylim(DB_MIN, DB_MAX)
    ax.set_xlabel('frequency (Hz)', color=LABEL_C, fontsize=8)
    ax.set_ylabel('dB', color=LABEL_C, fontsize=8)
    for sp in ax.spines.values(): sp.set_edgecolor(GRID_C)
    ax.tick_params(colors=LABEL_C, length=3, which='both')
    ax.set_title(m['name'], color='#c9d1d9', fontsize=9, pad=6, loc='left')
    fig.tight_layout(pad=0.6)
    path = OUT_DIR / f"oscmodel{m['idx']}.png"
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f"  saved {path.name}")

# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print(f"Generating spectrum images → {OUT_DIR}\n")
    for m in MODELS:
        print(f"[{m['idx']:2d}] {m['name']}")
        plot_spectrum(m)
    print("\nDone.")
