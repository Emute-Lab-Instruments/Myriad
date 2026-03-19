#!/usr/bin/env python3
"""
Generate waveform visualization images for Myriad oscillator models.

Approach: simulate the 'amp' signal inside each model's delta-sigma loop.
The lowpass-filtered bitstream output converges to this signal, so it
accurately represents what you'd see on an oscilloscope after the hardware filter.

Output: PNG files in OUT_DIR, one per model.
"""

import numpy as np
import matplotlib.pyplot as plt
import math
from pathlib import Path

OUT_DIR = Path("/home/ck84/src/www.eli/assets/images/myriad/waveforms")
OUT_DIR.mkdir(parents=True, exist_ok=True)

WAVELEN = 6000          # bits per simulated cycle — large enough for smooth curves
CTRL_VALUES = [0.0, 0.25, 0.5, 0.75, 1.0]

# ── Parabolic sine (bell model) ───────────────────────────────────────────────
def _psin_bell(ph):
    ph = int(ph) % 2048
    if ph < 0:
        ph += 2048
    p = ph if ph < 1024 else 2047 - ph
    val = (p * (1024 - p)) >> 8
    return val if ph < 1024 else -val

# ── Wavetable builders ────────────────────────────────────────────────────────

def build_bell_tables():
    raw_a = np.zeros(512)
    raw_b = np.zeros(512)
    for i in range(512):
        carrier   = i * 4
        mod_phase = (i * 14) % 2048
        mod_val   = _psin_bell(mod_phase)          # −512..512
        raw_a[i]  = _psin_bell(carrier + ((mod_val * 3) >> 1))
        raw_b[i]  = _psin_bell(carrier + (mod_val << 4))
    def _norm(t):
        mn, mx = t.min(), t.max()
        r = mx - mn if mx != mn else 1.0
        return ((t - mn) * 2048.0) / r
    return _norm(raw_a), _norm(raw_b)

def build_formant_tables():
    # psin here is a true sine (period = 512 samples)
    aw = [0, 0.39, 1.0, 0.75, 0.2, 0.1, 0.5, 0.2, 0.06]
    ew = [0, 1.0,  0.2, 0.1,  0.05, 0.2, 0.75, 0.5, 0.38]
    raw_a = np.zeros(512)
    raw_b = np.zeros(512)
    for i in range(512):
        sa = sb = 0.0
        for h in range(1, 9):
            s = math.sin(h * i * 2 * math.pi / 512)
            sa += s * aw[h]
            sb += s * ew[h]
        raw_a[i] = sa
        raw_b[i] = sb
    def _norm(t):
        mn, mx = t.min(), t.max()
        r = mx - mn if mx != mn else 1.0
        return ((t - mn) * 2048.0) / r
    return _norm(raw_a), _norm(raw_b)

def build_parasine_table():
    # sine shifted to 0..2048
    t = np.arange(256)
    return 1024.0 + 1024.0 * np.sin(2 * np.pi * t / 256)

BELL_A, BELL_B       = build_bell_tables()
FORMANT_A, FORMANT_B = build_formant_tables()
PARASINE_TABLE       = build_parasine_table()

# ── Amplitude-signal functions (one sample per index) ────────────────────────

def amp_saw(wavelen, ctrl_v):
    """Ramp × phaseMul, wrap to 0 when ≥ wavelen. ctrl: 0→pm=5, 1→pm=45."""
    pm = 5.0 + 40.0 * ctrl_v
    phase = np.arange(wavelen, dtype=np.float64)
    amp = phase * pm
    amp[amp >= wavelen] = 0.0
    return amp / wavelen

def amp_expPulse1(wavelen, ctrl_v):
    """Exponentially-spaced toggle pulses within one period. ctrl: 0→wide, 1→narrow."""
    vinv           = 1.0 - ctrl_v
    targmax        = wavelen / 32.0
    targetIncFP    = vinv * targmax + 10.0
    targIncMul     = vinv * 0.4 + 1.0
    amps    = np.zeros(wavelen)
    counter = 0.0
    target  = targetIncFP
    tInc    = targetIncFP
    b1      = 0
    for i in range(wavelen):
        if counter >= target:
            b1       = 1 - b1
            counter  = 0.0
            target  += tInc
            tInc    *= targIncMul
        amps[i] = float(b1)
        counter += 1.0
    return amps

def amp_sharkTeeth(wavelen, ctrl_v):
    """Multiple ramp teeth in first half-period, silence in second. ctrl: 0→1 tooth, 1→20."""
    bite = max(1, int(1.0 + ctrl_v * 19.0))
    half = wavelen // 2
    amps = np.zeros(wavelen)
    toothPhase = 0.0
    for i in range(wavelen):
        if toothPhase >= wavelen:
            toothPhase -= wavelen
        amp = toothPhase if i < half else 0.0
        amps[i] = amp
        toothPhase += bite
    return amps / wavelen

def amp_pulsePW(wavelen, ctrl_v):
    """Square wave with variable duty cycle. ctrl: 0→50%, 1→12%."""
    pw = 0.5 - 0.38 * ctrl_v
    phase = np.arange(wavelen, dtype=np.float64)
    return np.where(phase < wavelen * pw, 1.0, 0.0)

def amp_expPulse2(wavelen, ctrl_v):
    """Exponentially-decaying pulse spacing (slide). ctrl: 0→slow, 1→fast."""
    vinv        = 1.0 - ctrl_v
    targmax     = wavelen / 16.0
    targetIncFP = vinv * targmax + 3.0
    targIncMul  = 1.0 - vinv * 0.8
    amps    = np.zeros(wavelen)
    counter = 0.0
    target  = targetIncFP
    tInc    = targetIncFP
    b1      = 0
    for i in range(wavelen):
        if counter >= target:
            b1       = 1 - b1
            counter  = 0.0
            target  += tInc
            tInc    *= targIncMul
            tInc    *= targIncMul   # doubled — matches C++ which multiplies twice
        amps[i] = float(b1)
        counter += 1.0
    return amps

def amp_tri(wavelen, ctrl_v):
    """Asymmetric triangle. ctrl: 0→nearly symmetric, 1→fast-rise slow-fall."""
    idx        = int(ctrl_v * 0.9 * 1000)       # matches C++ index = (v * 900).to_int()
    v_tab      = idx / 1000.0
    gradRising = 2.0 + v_tab * 7.0              # 2..9
    peakPoint  = wavelen / gradRising
    peakAmp    = float(wavelen)
    gradFalling = peakAmp / (wavelen - peakPoint) if wavelen > peakPoint else 1.0
    amps = np.zeros(wavelen)
    for i in range(wavelen):
        if i <= peakPoint:
            amps[i] = i * gradRising
        else:
            delta   = i - peakPoint
            amps[i] = max(0.0, peakAmp - delta * gradFalling)
    return amps / wavelen

def amp_triTeeth(wavelen, ctrl_v):
    """Triangle with bouncing sub-oscillator teeth. ctrl: 0→few teeth, 1→many."""
    bite_raw = 2.0 + ctrl_v * 24.0
    bite     = int(bite_raw) * 2          # floor then ×2 (matches C++)
    bite     = max(2, bite)
    amps        = np.zeros(wavelen)
    triPhase    = 0
    triInc      = 2
    toothPhase  = 0.0
    toothInc    = float(bite)
    wlen        = wavelen
    for i in range(wavelen):
        # bounce tri 0..wlen step ±2
        if triPhase > wlen:
            triPhase = 2 * wlen - triPhase
            triInc   = -triInc
        elif triPhase < 0:
            triPhase = -triPhase
            triInc   = -triInc
        # bounce tooth 0..wlen step ±bite
        if toothPhase >= wlen:
            toothPhase = 2.0 * wlen - toothPhase
            toothInc   = -toothInc
        elif toothPhase < 0.0:
            toothPhase = -toothPhase
            toothInc   = -toothInc
        # raw_amp = max(triPhase, toothPhase)
        raw_amp  = triPhase if toothPhase < triPhase else int(toothPhase)
        amps[i]  = raw_amp / wlen
        triPhase   += triInc
        toothPhase += toothInc
    return amps

def amp_parasine(wavelen, ctrl_v):
    """Asymmetrically waveshaped sine. ctrl: 0→symmetric, 1→very asymmetric."""
    shape_pos = 1.0 - 0.1 * ctrl_v    # 1.0..0.9
    shape_neg = 1.0 - 0.9 * ctrl_v    # 1.0..0.1
    table     = PARASINE_TABLE
    phase_inc = 256.0 / wavelen
    amps  = np.zeros(wavelen)
    phase = 0.0
    for i in range(wavelen):
        idx = int(phase) & 0xFF
        raw = table[idx]
        dev = raw - 1024.0
        amp = 1024.0 + dev * (shape_pos if dev >= 0 else shape_neg)
        amps[i] = amp / 2048.0
        phase  += phase_inc
    return amps

def amp_formant(wavelen, ctrl_v):
    """Morphs between two additive-synthesis wavetables. ctrl: 0→table A, 1→table B."""
    morph     = ctrl_v
    phase_inc = 512.0 / wavelen
    amps  = np.zeros(wavelen)
    phase = 0.0
    for i in range(wavelen):
        idx    = int(phase) & 0x1FF
        a      = FORMANT_A[idx]
        b      = FORMANT_B[idx]
        amps[i] = (a + (b - a) * morph) / 2048.0
        phase  += phase_inc
    return amps

def amp_bell(wavelen, ctrl_v):
    """FM wavetable morph: gentle to harsh metallic. ctrl: 0→mellow, 1→harsh."""
    morph     = ctrl_v
    phase_inc = 512.0 / wavelen
    amps  = np.zeros(wavelen)
    phase = 0.0
    for i in range(wavelen):
        idx    = int(phase) & 0x1FF
        a      = BELL_A[idx]
        b      = BELL_B[idx]
        amps[i] = (a + (b - a) * morph) / 2048.0
        phase  += phase_inc
    return amps

def amp_noiseSD(wavelen, ctrl_v):
    """Random-length toggle bursts. ctrl: 0→slow sparse, 1→dense fast."""
    rng       = np.random.default_rng(42)
    rand_mul  = max(1, int(-1 + ctrl_v * ctrl_v * 500))
    amps    = np.zeros(wavelen)
    on      = False
    counter = 0
    for i in range(wavelen):
        if counter == 0:
            on      = not on
            dur     = 1 + int(wavelen * 0.01 * rng.integers(0, max(2, rand_mul)))
            counter = dur
        amps[i] = 1.0 if on else 0.0
        counter -= 1
    return amps

def amp_whiteNoise(wavelen, ctrl_v):
    """Chaotic bit noise; ctrl has subtle effect on mutation/XOR rate."""
    # Approximate: show filtered random pattern with ctrl-dependent smoothing
    rng    = np.random.default_rng(42)
    bits   = rng.integers(0, 2, size=wavelen).astype(float)
    # ctrl modulates the apparent density/chaos — higher ctrl → more chaotic
    window = max(1, int(wavelen * 0.005 * (1.0 - ctrl_v * 0.8)))
    kernel = np.ones(window) / window
    return np.convolve(bits, kernel, mode='same')

def amp_silent(_wavelen, _ctrl_v):
    return None    # special case: flat / no useful waveform to show

# ── Model registry ────────────────────────────────────────────────────────────

MODELS = [
    dict(idx=0,  id='saw',     name='Saw',          fn=amp_saw,
         ctrl_desc='ctrl → pulse width (0 = wide, 1 = narrow)'),
    dict(idx=1,  id='exp1',    name='Chirp',         fn=amp_expPulse1,
         ctrl_desc='ctrl → pulse density (0 = sparse, 1 = dense)'),
    dict(idx=2,  id='sdt10',   name='Sharkteeth',    fn=amp_sharkTeeth,
         ctrl_desc='ctrl → number of teeth (0 = 1, 1 = 20)'),
    dict(idx=3,  id='pulsesd', name='Pulse',          fn=amp_pulsePW,
         ctrl_desc='ctrl → duty cycle (0 = 50%, 1 = 12%)'),
    dict(idx=4,  id='slide',   name='Slide',          fn=amp_expPulse2,
         ctrl_desc='ctrl → slide speed (0 = slow, 1 = fast)'),
    dict(idx=5,  id='tri',     name='Triangle',       fn=amp_tri,
         ctrl_desc='ctrl → asymmetry (0 = symmetric, 1 = skewed)'),
    dict(idx=6,  id='trv10',   name='TriTeeth',       fn=amp_triTeeth,
         ctrl_desc='ctrl → tooth rate (0 = slow, 1 = fast)'),
    dict(idx=7,  id='sinesd',  name='Sine',           fn=amp_parasine,
         ctrl_desc='ctrl → waveshape asymmetry (0 = pure, 1 = skewed)'),
    dict(idx=8,  id='formant', name='Formant',        fn=amp_formant,
         ctrl_desc='ctrl → morph (0 = table A, 1 = table B)'),
    dict(idx=9,  id='metalic', name='Metallic',       fn=amp_bell,
         ctrl_desc='ctrl → morph (0 = mellow FM, 1 = harsh FM)'),
    dict(idx=10, id='n2',      name='Noise',          fn=amp_noiseSD,
         ctrl_desc='ctrl → density (0 = sparse, 1 = dense)'),
    dict(idx=11, id='wn',      name='White Noise',    fn=amp_whiteNoise,
         ctrl_desc='ctrl → character (0 = smoother, 1 = rawer)'),
    dict(idx=12, id='sil',     name='Silent',         fn=amp_silent,
         ctrl_desc=''),
]

# ── Plotting ──────────────────────────────────────────────────────────────────

BG      = '#0d1117'
GRID_C  = '#1c2128'
AXIS_C  = '#2d3748'
LABEL_C = '#8b949e'

# Green gradient: dim → bright
CTRL_COLORS = ['#1a4a1a', '#2d7a2d', '#3aaa3a', '#57d557', '#7eff7e']

def center_normalize(amp):
    """Remove DC and scale to −1..+1."""
    a = amp - amp.mean()
    mx = np.abs(a).max()
    return a / mx if mx > 0 else a

def plot_model(m, wavelen=WAVELEN, ctrl_values=CTRL_VALUES):
    if m['fn'] is amp_silent:
        _plot_silent(m)
        return

    fig, ax = plt.subplots(figsize=(9, 2.8), facecolor=BG)
    ax.set_facecolor(BG)
    ax.grid(True, color=GRID_C, linewidth=0.6, zorder=0)
    ax.axhline(0, color=AXIS_C, linewidth=0.8, zorder=1)

    x = np.linspace(0, 2, wavelen * 2)

    for cv, color in zip(ctrl_values, CTRL_COLORS):
        raw = m['fn'](wavelen, cv)
        sig = center_normalize(raw)
        ax.plot(x, np.tile(sig, 2), color=color, linewidth=1.3,
                label=f'{cv:.2f}', zorder=2, alpha=0.95)

    ax.set_xlim(0, 2)
    ax.set_ylim(-1.55, 1.55)
    ax.set_xticks([0, 0.5, 1.0, 1.5, 2.0])
    ax.set_xticklabels(['0', '½', '1', '1½', '2'], color=LABEL_C, fontsize=8)
    ax.set_yticks([-1, 0, 1])
    ax.set_yticklabels(['-1', '0', '1'], color=LABEL_C, fontsize=8)
    ax.set_xlabel('cycles', color=LABEL_C, fontsize=8, labelpad=3)
    ax.tick_params(colors=LABEL_C, length=3)
    for sp in ax.spines.values():
        sp.set_edgecolor(GRID_C)

    leg = ax.legend(
        title='ctrl', title_fontsize=7,
        loc='upper right', fontsize=7,
        framealpha=0.5, facecolor='#161b22', edgecolor=GRID_C,
        labelcolor=LABEL_C, handlelength=1.2,
    )
    leg.get_title().set_color(LABEL_C)

    ax.set_title(f'{m["name"]}  ·  {m["ctrl_desc"]}',
                 color='#c9d1d9', fontsize=9, pad=6, loc='left')

    fig.tight_layout(pad=0.6)
    path = OUT_DIR / f"oscmodel{m['idx']}.png"
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f"  saved {path.name}")

def _plot_silent(m):
    fig, ax = plt.subplots(figsize=(9, 2.8), facecolor=BG)
    ax.set_facecolor(BG)
    ax.grid(True, color=GRID_C, linewidth=0.6)
    ax.axhline(0, color=AXIS_C, linewidth=1.2)
    ax.text(1.0, 0.3, '— silence —', color='#3a5a3a', fontsize=13,
            ha='center', va='center', style='italic')
    ax.set_xlim(0, 2); ax.set_ylim(-1.55, 1.55)
    ax.set_xticks([]); ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_edgecolor(GRID_C)
    ax.set_title(f'{m["name"]}', color='#c9d1d9', fontsize=9, pad=6, loc='left')
    fig.tight_layout(pad=0.6)
    path = OUT_DIR / f"oscmodel{m['idx']}.png"
    fig.savefig(path, dpi=150, bbox_inches='tight', facecolor=BG)
    plt.close(fig)
    print(f"  saved {path.name}")

# ── Main ──────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    print(f"Generating waveform images → {OUT_DIR}\n")
    for model in MODELS:
        print(f"[{model['idx']:2d}] {model['name']}")
        plot_model(model)
    print("\nDone.")
