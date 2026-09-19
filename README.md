# Headway Switching, Not Packet Loss, Destabilises Cooperative Platoons Under V2X Dropout

Code and results for the manuscript (Haneesh K. M., Jisha P., Suresh K., Aravind Pitchai
Venkataraman) studying how intermittent V2X communication erodes the string-stability
guarantee of Cooperative Adaptive Cruise Control (CACC); isolating the destabilising
mechanism as *discontinuous headway switching on packet loss*, not the loss of
information itself; and evaluating a dropout-aware multi-agent reinforcement-learning
controller against classical alternatives (including a canonical, provably
string-stable plant-inverting design and a dwell-time hysteresis scheme) under the same
protocol.

**Before using this repository, read [`CORRECTIONS.md`](CORRECTIONS.md).** It documents
two rounds of issues found and fixed during development and review:

1. Five implementation issues in an earlier version of the RL code (a wrong critic
   architecture, missing reward terms, a miscentred action map, broken evaluation
   wiring, and a training-resume bug that silently changed results).
2. Issues found during the Scientific Reports manuscript revision, including two real
   bugs with numeric consequences -- Gilbert-Elliott channel parameters that were not
   actually mean-matched to the claimed value, and a frequency-sweep evaluation horizon
   that silently disagreed with every other table at their shared benchmark point --
   plus several transcription errors caught by a full line-by-line check of every
   number against its simulation output.

This is not housekeeping: several of these issues changed specific numbers, and one
(the Gilbert-Elliott correction) changed a substantive finding about the learned
controller's robustness. Every number in the current manuscript reflects the corrected
code, not any earlier draft.

## Repository layout

```
cacc/                   The simulation + control + RL library (see below)
scripts/                Training and reproduction scripts (thin CLI wrappers around cacc/)
tests/                  pytest suite -- run this after touching cacc/maddpg.py or marl_env.py
results/                Pre-computed JSON output from every reproduction script
models/                 Trained actor checkpoints for training seeds 10-14, both variants
models_extended_freq/   Checkpoints for the two extended-disturbance-frequency training seeds (30, 31)
snapshots/              Periodic checkpoints of seed 14's training run (for run_checkpoint_selection.py)
paper/                  The manuscript source and figures
CORRECTIONS.md          What changed across both rounds above, and why
```

### `cacc/` library

| Module | Contents | Requires torch? |
|---|---|---|
| `dynamics.py` | Vehicle longitudinal dynamics, actuator lag, EV energy model | No |
| `idm.py` | Intelligent Driver Model (human drivers) | No |
| `v2x.py` | V2X channel: i.i.d. Bernoulli and bursty Gilbert-Elliott dropout | No |
| `controllers.py` | `PIDACC`, `MPCACC`, `CACC`, `SmoothCACC`, `PlantInvertingCACC`, `DwellHysteresisCACC`, `RLPolicy` | No (RLPolicy just wraps a callable) |
| `platoon.py` | Deterministic, scripted-controller platoon simulator | No |
| `string_stability.py` | Closed-form transfer functions, legacy amplification metrics | No |
| `metrics.py`, `scenarios.py`, `plotting.py` | Rollout metrics, disturbance profiles, publication plotting | No |
| `marl_env.py` | Gym-style multi-agent RL environment (training) | **Yes** |
| `maddpg.py` | The shared-policy multi-agent TD3 implementation | **Yes** |
| `evaluation.py` | Canonical-protocol evaluation (`CANONICAL_T=120s`, `CANONICAL_EVAL_SEEDS=5..9`), shared by every reproduction script | **Yes** |

`PlantInvertingCACC` is the canonical, Ploeg/Naus-style controller whose closed-loop
velocity transfer function is exactly `G(s)=1/(1+h*s)` for any `h>0` under this
project's actuator model -- the strongest possible feedforward design, used to isolate
whether the paper's central "switching, not information loss, destabilises the platoon"
claim survives with the best-case classical baseline rather than only the paper's own,
weaker additive-feedforward implementation. `DwellHysteresisCACC` implements a
configurable-dwell-time hysteresis band on the same binary headway switch, to test
whether simply switching less often (without abandoning discontinuous switching itself)
recovers stability.

`cacc/__init__.py` deliberately does *not* import `marl_env`/`maddpg`/`evaluation` at
the top level, so `import cacc` for pure classical-simulation work does not require
`torch`. Import those three modules directly when you need them (every script in
`scripts/` already does).

## Installation

```bash
python -m venv venv && source venv/bin/activate
pip install -r requirements.txt
pip install -e .          # makes `cacc` importable from anywhere; or just set PYTHONPATH=.
pytest tests/             # should show 22 passed
```

Tested on Python 3.12, CPU only. Training and evaluation are both single-threaded by
design (`torch.set_num_threads(1)`) -- on the single-core machine this project was
developed on, this was roughly 4x *faster* than the default multi-threaded setting due to
thread-contention overhead at this network size; if you have many cores and want to
parallelise across seeds rather than within one, run multiple training processes instead.

## Quickstart

```bash
# Evaluate an already-trained model under the canonical protocol (p=0.3, omega=0.45, 8 followers)
python scripts/evaluate.py --model models/full_gate_seed10.pt

# Same, at 60% dropout
python scripts/evaluate.py --model models/full_gate_seed10.pt --p-drop 0.6

# Force the gate to a constant (the ablation in Table "Isolating the gate")
python scripts/evaluate.py --model models/full_gate_seed10.pt --force-gate 0.0

# Train a new seed from scratch (resumable -- rerun the same command to continue;
# see CORRECTIONS.md item 5 for why resuming is safe here and wasn't originally)
python scripts/train.py --seed 20 --episodes 150 --time-budget 180
```

Training one seed to the 150-episode budget used throughout this study takes roughly
8-10 minutes total on a single CPU core (this project's own compute budget), split
across as many `--time-budget`-limited invocations as your environment's execution
limits require; each invocation checkpoints before exiting, so nothing is lost between
runs.

## Reproducing the paper's tables and figures

Every script below writes its raw output to the matching subdirectory of `results/`,
which already contains the output used to generate the current manuscript revision --
diff against it to check your environment reproduces the same numbers.

| Paper artifact | Script | Needs |
|---|---|---|
| Table "Isolating the destabilising mechanism" (plant-inverting ablation) | `scripts/run_mechanism_ablation.py` | nothing (classical controllers only) |
| Table "Learned controller versus classical baselines" | `scripts/run_baseline_comparison.py`, `scripts/run_full_metrics.py` | models/full_gate_seed{10-14}.pt |
| Table "Isolating the gate" | `scripts/run_gate_ablation.py`, `scripts/run_gate_counterfactuals.py` | same |
| SmoothCACC tuning grid (used to produce the tuned baseline above) | `scripts/run_smooth_cacc_tuning.py` | nothing |
| Table "Dwell-time hysteresis sweep" | `scripts/run_hysteresis_sweep.py` | nothing |
| Table "Gilbert-Elliott channel parameters" + bursty-loss evaluation | `scripts/run_ge_dropout_sweep.py` | models/full_gate_seed{10-14}.pt; self-verifies its own mean-loss claim on every run |
| Figure "Frequency sensitivity is seed-dependent" | `scripts/run_frequency_sweep.py` (run once per seed) | models/full_gate_seed{10,11,12,13}.pt |
| Figure "Learned controller versus classical baselines" (dropout + frequency panels) | `scripts/make_fig_rl_benchmark.py` | same, plus the frequency-sweep JSON above |
| Figure "Per-vehicle steady-state velocity amplification" | `scripts/make_fig_amplification.py` | nothing |
| Figure "switching-transient residual" (Gamma* decomposition) | `scripts/run_ptp_vs_fft.py` | models/full_gate_seed{10-14}.pt |
| Switching-duty / fallback-fraction sub-grid (mixed-autonomy mechanism discussion) | `scripts/run_switching_duty.py` | nothing |
| Platoon-length robustness discussion | `scripts/run_platoon_length_sweep.py` | models/full_gate_seed{10-13}.pt |
| Mixed-autonomy (penetration) sweep | `scripts/run_penetration_sweep.py` | models/full_gate_seed{10-14}.pt |
| Extended-disturbance-frequency training seeds | `scripts/train_extended_freq.py`, then `scripts/run_frequency_sweep.py --model models_extended_freq/full_gate_seed{30,31}.pt` | |
| Checkpoint-selection diagnostic (the seed-14 discussion) | `scripts/run_checkpoint_selection.py` | snapshots/full_gate_seed14_ep*.pt (included) |

Every script accepts `--help` for its full option list (seeds, dropout rates, output
path, etc.). None of the classical-controller-only artifacts that don't appear above
(the closed-form transfer-function plot, the phase diagram, the mixed-autonomy topology
figure) required any of the corrections in `CORRECTIONS.md` and are not re-scripted
here; they can be reproduced directly from `cacc.string_stability`, `cacc.platoon`, and
`cacc.v2x` as described in the manuscript.

### The gate-ablation variant: `--variant hide_raw_comm`

Every training and gate-ablation script accepts `--variant full_gate` (default) or
`--variant hide_raw_comm`. The latter withholds the raw `v2x_a`/`v2x_v` observation
fields from the actor after gating, so it cannot bypass the gate and read the
communicated value directly -- the "stronger ablation" for the gate-bypass concern
discussed in the manuscript. Both variants are trained for all five seeds and their
results live side-by-side in `results/gate_ablation/`.

## Known limitations (see the manuscript's Discussion for full treatment)

- **One of five training seeds does not converge to a string-stable policy** (seed 14,
  `full_gate` variant) despite reaching training reward comparable to the successful
  seeds. This is reported, not hidden: `results/` includes its numbers throughout, and
  `scripts/run_checkpoint_selection.py` demonstrates that no periodic checkpoint of that
  training run is string stable either -- it is a genuine training-outcome failure, not
  a stopping-time artifact.
- **The reward's two clip constants** (`kappa_track`, `kappa_v` in `MARLConfig`) are
  reasonable choices, not values recovered from an original training configuration.
- **Learned-controller stability is frequency-band-limited**, not universal across the
  disturbance spectrum tested (`results/frequency_sweep/`), and each successful training
  seed fails in a *different*, non-overlapping frequency sub-band.
- **Robustness to bursty (Gilbert-Elliott) channel loss is itself seed-dependent** at
  matched mean loss: severe bursts can defeat seeds that handle i.i.d. loss and milder
  bursts fine (`results/ge_dropout_sweep/`).
- The classical continuous-scheduler baseline's tuning grid search and its headline
  evaluation reuse the same five canonical seeds (no disjoint validation split) --
  disclosed in the manuscript as a mild winner's-curse risk, not fixed here.

## Citation

See `CITATION.cff`. If you use this code, please cite the manuscript; if you build on
the corrections documented in `CORRECTIONS.md`, a mention of that is appreciated but not
required.

## License

See `LICENSE`.
