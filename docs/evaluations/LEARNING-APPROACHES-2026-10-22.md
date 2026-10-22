# Learning approaches for the tiered policy (2026-10-22)

A desk survey, not a measurement: the ways the tiered policy's behaviour could
be learned or tuned instead of hand-set, what each would learn, the data it
needs against the data this project has, and the order to try them in.
Nothing here is built or decided. Each approach that goes ahead gets its own
plan section with its bars written before measuring (CLAUDE.md section 7).

It came out of the discussion of 3.91 B6 with the user, on 2026-10-22.
B6 showed two things:
* With the local VLM first, a replay of four phone walks needed 0 cloud calls,
  where the live walks had made 11 / 12 / 9 / 10.
* A local "no" changes no steering.

The user asked whether the tier rules could be learned, and which ways to
learn them exist besides a bandit.

## The decisions that could be learned

| | Decision | Where it lives today |
|---|---|---|
| D1 | **Who to ask** about a frame: no one, the local VLM, or the cloud | `brain/tiered.py`: the triggers, `tier_local_max_unable` 2, the pacing keys |
| D2 | **What an answer does**: how sure before acting, and whether a "no" is remembered | `LOCAL_YES_AT` 0.5, `tier_local_no_below` 0.1; C's proposed rules (3.91, "Proposed for C") |
| D3 | **Steering** toward a sighting | YOLOE + CLIP's candidate steer, or the cloud's direction |
| D4 | **Where to look** when the target is out of view | From the map, never from a frame (policy ARCHITECTURE, 3.80.I); explore's goal choice (3.65, 3.88) |
| D5 | **The local VLM's own yes/no** | Qwen2.5-VL-3B, engine VE (3.78), not fine-tuned |

## What there is to learn from

| Source | Size | Holds |
|---|---|---|
| 3.48's identity frames | 299 frames | The local VLMs' and the cloud's answers on the same labelled frames (the cloud 67.9% precision) |
| 3.80.I's steering frames | 60 in view, 40 out | Every tier's direction on each frame |
| 3.91's four phone walks | about 300 decided frames | The live cloud's answers, B6's local answers on the same frames, two bags (one a lookalike) |
| Rig walks and their `labels.json` | Four valid walks (2026-09-07) | Perception's labels |
| 3.88's explore choices | A few hundred taps at most | A person's pick among explore's candidates |
| Simulator sweeps | 1,000 ProcTHOR houses | Closed-loop missions, but **the vision is not real**: the renderer writes objects as text labels, so YOLOE, CLIP and Qwen cannot run on it |

**The binding limits:**
* **Little real data.** A few hundred real frames across a handful of objects.
* **No closed loop with real vision.** The simulator's frames are not photos.
  A photoreal simulator was evaluated and parked by the user on 2026-10-22, to
  return only if steering is to be learned.
* **One policy logged everything.** Today's policy is deterministic, so the
  logs never show what another choice would have led to. That matters for
  every method that learns from logs (below).

## Ground rules for anything learned

These follow from the project's existing decisions:
1. **A learner only chooses among actions the rules allow.**
   * In safe-RL terms the hard rules are a **shield**: a runtime filter that
     blocks unsafe actions whatever the learned policy prefers.
   * Here the shield is `robot/safety.py`'s veto, a person's Stop (3.84),
     arrival's lidar rule, and "only the cloud confirms a local-led arrival"
     (3.80.O).
   * 3.65 already works this way: it learns only the order of goals the rules
     would send.
2. **Judged on data it never saw:** houses or walks held out of training and
   of selection. The learned policies of 3.65 and 3.88 both won in training
   and fell short on fresh houses.
3. **Tuned for the worst scenario, not the average.** Tag frames and missions
   by scenario (lookalike, far view, close-up, out of view) so that a rare,
   costly case cannot be traded away for a better mean.
4. **The simulator filters; real data judges.** A behaviour that wins in the
   simulator is replayed on recorded real frames (as B6 did), then run on the
   Rover.

## The approaches

### 1. Value of information over calibrated confidences (no training loop)

**What:** ask the cloud only when the expected gain in being right is worth
the price of the call. If the local tier says "no" at a calibrated 99%, the
cloud's answer rarely changes the action. At 60% it often does.

This is the decision rule behind cost-aware *model cascades*:
* FrugalGPT tries models cheapest first and stops when a scorer trusts the
  answer.
* Cascades with early abstention add "decline to answer" as an outcome.

**Learns:** D1, and the thresholds of D2.

**Needs:** each tier's confidence **calibrated**, so that "90%" is right about
90% of the time.
* Temperature scaling, or a one-parameter Platt fit on the yes-probability,
  does this. The VLM calibration study below found under 100 labelled samples
  enough.
* 3.48's 299 frames would serve for the arrival question.
* B6's frames would serve for the frame question.

**Fit: strong, and first.** It needs no training loop, uses data in hand,
explains itself, and replaces the hand-set "2 unables, then the cloud".

**Risk:** calibration holds only for frames like the calibration set. Phone
frames are not the Rover's camera, so recalibrate on the Rover's frames.

### 2. Conformal prediction: ask when the answer set is ambiguous

**What:**
* Conformal prediction turns any score into a set of plausible answers, with
  a guaranteed rate at which the true answer is in the set (for example 95%).
* KnowNo ("Robots that ask for help", CoRL 2023) uses it to decide when a
  robot should ask for help. If the set holds one answer, act. If it holds
  more than one, ask.
* Here the answers are "is the target" and "is not", and "ask" means the cloud
  or a person.

**Learns:** D1 and D2, with a stated error bound instead of a tuned threshold.

**Needs:** a labelled calibration set drawn from the same conditions as use.
That means Rover frames, for the guarantee to mean anything on the car.

**Fit: strong, after the Rover's first walks.** It answers "how often may the
local tier be wrong unchecked?" with a number the user chooses.

**Risk:** the guarantee is on average over the calibration conditions, not per
scenario. Run it per scenario class (rule 3).

### 3. A learned router (learning to defer)

**What:** a small classifier predicts, from what the local tier saw, whether
its answer will be wrong. If it predicts wrong, defer to the cloud.
* RouteLLM trains such a router from preference data.
* "Learning to defer" generalises it, with model cascades as a special case.
* Gatekeeper trains the small model itself to know when to defer.

**Features available here:**
* CLIP's P, the local VLM's score, the box's share of the frame (a stand-in
  for range).
* Whether perception and the VLM agree. The taupe bag: CLIP 0.89-1.0 against
  Qwen under 0.002.
* The trigger.

**Learns:** D1.

**Needs:** frames where both tiers answered and the truth is known. 3.48 has
299 for the arrival question; the frame question has only B6's four walks.

**Fit: medium.** It goes beyond approach 1 when the signs of an error are not
in the score alone (disagreement, range). With a few hundred frames, keep it
to a handful of features and a linear model.

### 4. A contextual bandit, judged off-policy

**What:**
* Each frame is a context.
* The action is who to ask.
* The reward is a right answer minus the cost of the call.
* The bandit learns the action per context, and *off-policy evaluation*
  scores a new policy on logs the old one made. The usual estimators are
  inverse propensity scoring (IPS), its self-normalised form SNIPS, and the
  doubly robust estimator (DR).

**Learns:** D1, and parts of D2.

**Needs (the catch):** these estimators need every action to have some chance
of being taken in every context ("support"). Today's policy is deterministic,
so it never logs the cloud's answer where the local tier was confident.

Fix it before learning anything:
* Log a small, recorded share of random choices (say 5%), each with its
  probability, on live missions.
* The cloud's share costs money. Estimate it before turning it on, and ask
  the user.

**Fit: medium, after logging is in place.** Approaches 1-3 cover most of what
it would learn, with less data.

### 5. Imitation and distillation (DAgger)

**What:** a cheap student learns to copy an expensive teacher. DAgger then
runs the student, asks the teacher only on the states the student reaches,
and adds those answers to the training set. The aim is to fix compounding
errors where the student goes wrong.

**Teachers here:**
* GPT-6 Astra for steering, the best of every tier with the target in view
  (3.80.I, 56/60 frames). Astra fails the identity check (3.80.Q, 56.1%
  precision), so it must not teach identity.
* A person's goal taps for D4 (3.88).

**Learns:** D3, D4.

**On record:** 3.88 B cloned an oracle into the ten-weight goal score and
agreed 29% (bar 60%). It underfit, because the ten features cannot express
what the oracle knew. Distillation is capped by what the student's inputs can
see.

**Fit: medium for D3, on recorded frames.** For D4 it waits on richer features.

### 6. Offline reinforcement learning (IQL, CQL)

**What:** learn a policy from logged experience without new driving, held
close to what the logs show, so it never trusts actions they never tried.
IQL fits only an extra critic and is simple to run. ReViND (CoRL 2023) used
it for real-world navigation from logged data.

**On record elsewhere:** ReViND's authors found IQL's policy failed past about
20 m (80 steps) as values degraded over the long horizon. So they used its
value function as costs in a topological planner instead.

**Learns:** D1-D4 together, in principle.

**Needs:** far more logged missions than exist, with varied actions (see
approach 4's support problem).

**Fit: later.** Revisit once the Rover has logged many missions.

### 7. Learning a reward from preferences

**What:**
* A person compares two clips of behaviour and picks the better.
* A reward model is fitted to those choices, then an RL method trains on it.
* The basic framework is Christiano et al. (2017). PEBBLE (ICML 2021) makes
  each comparison go further: it re-scores past experience when the reward
  model changes.

**Learns:** what "a good search" means where a hand-written reward is hard
to state (wasted turns, approaching the wrong object).

**Needs:** a person's time, and an RL loop to train against the reward:
closed loop again.

**Fit: low now.** It is a way to *write* the reward for approaches 6 or 10,
not a policy on its own.

### 8. Reinforcement fine-tuning of the local VLM (GRPO)

**What:** fine-tune the VLM with a reward a program can check, not a judge.
For example "said yes, and the frame is labelled target", or the box's
overlap with the labelled box.
* Visual-RFT applied GRPO to Qwen2-VL 2B/7B for grounding and detection.
* It reported +24.3% accuracy in one-shot fine-grained classification from
  about 100 samples.

**Learns:** D5, which feeds every other decision.

**Needs:**
* Labelled frames of the household's objects, lookalikes included.
* GPU time to train.
* Rebuilding VE: INT4 AWQ, the reduced vocabulary and the Edge-LLM engine
  (3.78).

**Fit: only if Qwen fails identity on the Rover's frames.** Every one of
3.78's bars must then be re-measured: precision 97.2%, p90 1.32 s, and memory
with 606 MB free beside the stack. A fine-tune can trade those away.

### 9. Black-box tuning of a behaviour tree's parameters

**What:** keep the behaviour tree's structure hand-written and search only its
numbers with a black-box optimiser, in simulation.
* Mayr et al. (IROS 2021) did this for a robot arm and transferred the result
  to the real robot without more training.
* They argue a behaviour tree's parameters are few and readable, where RL
  needs thousands of trials.
* 3.65 already tunes ten weights this way (the cross-entropy method).

**Learns:** the numbers of D1-D3 once the ladder is a tree (the user's point
5: composable policies with their own settings).

**Needs:**
* The ladder as a tree.
* A simulator whose perception reports are calibrated on real frames. That
  means a perception emulator over the ProcTHOR houses, with scenarios placed
  in them (discussed 2026-10-22), not a photoreal renderer.

**Fit: strong, once point 5's tree exists.** It is the natural successor to
tuning by hand, and the scenario sweeps make rule 3 practical.

**Risk:** the optimiser exploits the emulator's gaps, so the winner is checked
on real recorded frames (rule 4).

### 10. Hierarchical RL over the tree's nodes

**What:** learn *when to switch* between behaviours (search, approach, verify,
give up on a candidate), each behaviour kept as a node with its own rules.
Learning picks the node; the node does the work.

**Learns:** the tree's control flow, beyond its numbers.

**Needs:** a closed loop in which switching changes what is seen next. With
emulated perception that is possible in the simulator, but only as honest as
the emulator.

**Fit: later, after approach 9.**

### 11. Planning under uncertainty (a POMDP) instead of learning

**What:** model the search as a partially observed problem. The robot keeps a
belief over where the target is and how far to trust each sensor, and plans
which place to look and which sensor to consult by the information it will
gain against its cost. COS-POMDP does this for object search with unreliable
detectors, small objects included.

**Decides:** D1 and D4 from first principles, with each tier as a sensor of
known reliability.

**Needs:**
* Each tier's error rates (approaches 1-2 measure them).
* A belief over the map.
* Compute: these planners grow fast with the number of sensors and places,
  so they are usually hierarchical.

**Fit: a design reference rather than a build.** "Where to look is chosen
from the map" (3.80.I) and value of information (approach 1) are its cheap
halves.

### 12. End-to-end on-policy RL (for example PPO on steering and asking)

**Not now:**
* It needs a closed loop with real vision, and the simulator has none.
* Real-world RL on one robot needs thousands of minutes-long missions.
* The reward is mostly decided at a mission's end.
* 3.65's network and 3.88's PPO both fell short on fresh houses, even on the
  simulator's honest part.

Revisit with a photoreal or scanned-home simulator (parked 2026-10-22), warm
started by approach 5.

## Comparison

| # | Approach | Decisions | Data needed | Have it now? | Closed loop? | Fit and when |
|---|---|---|---|---|---|---|
| 1 | Value of information, calibrated | D1, D2 | ~100+ labelled frames per question | Yes (3.48, B6) | No | **First** |
| 2 | Conformal "ask for help" | D1, D2 | A calibration set like use | After the Rover's walks | No | **Second** |
| 3 | Learned router | D1 | Both tiers on labelled frames | Partly (3.48; B6 small) | No | Medium |
| 4 | Bandit, off-policy evaluation | D1, D2 | Logs with randomised choices | No: needs logged exploration (paid share asked first) | No | Medium, later |
| 5 | Distillation, DAgger | D3, D4 | A teacher's labels | Yes for D3 (Astra on 3.80.I) | DAgger: yes | Medium |
| 6 | Offline RL (IQL, CQL) | D1-D4 | Many varied missions | No | No | Later |
| 7 | Preference reward | the reward | A person's comparisons | No | For training: yes | Low now |
| 8 | GRPO on the VLM | D5 | Labelled household frames, GPU | Partly | No | Only if identity fails on the Rover |
| 9 | Tree parameters, black box | D1-D3 numbers | Calibrated perception emulator | Pieces (3.80.O's stand-ins) | Simulated | **Strong, with point 5** |
| 10 | Hierarchical RL | Tree flow | Emulator closed loop | No | Simulated | After 9 |
| 11 | POMDP planning | D1, D4 | Tier error rates, belief map | Partly | No | Design reference |
| 12 | End-to-end RL | All | Real-vision closed loop | No | Yes | Not now |

## Suggested order

1. **Calibrate the tiers and decide by value of information** (approach 1) on
   3.48's and B6's frames. No training loop, $0.
2. **Turn the ladder into a behaviour tree** (the user's point 5). Build the
   perception emulator and scenario library in the simulator, then tune the
   tree's numbers by black box (approach 9), worst scenario first.
3. **On the Rover:**
   * Recalibrate, and put conformal bounds on the local tier's unchecked
     errors (approach 2).
   * Start logging a small randomised share of "who to ask" choices with
     their probabilities, the paid share estimated and asked first. That makes
     approaches 3-4 possible.
4. **Distil Astra's steering** into the local steer on recorded frames
   (approach 5).
5. **Fine-tune Qwen with GRPO** (approach 8) only if identity fails on the
   Rover's frames.
6. **Offline RL, hierarchical RL, end-to-end RL** (6, 10, 12) only after
   many Rover missions or a realistic simulator.

## Sources

* Ren et al., [Robots That Ask For Help: Uncertainty Alignment for Large Language Model Planners](https://arxiv.org/abs/2307.01928) (KnowNo, CoRL 2023)
* [Dynamic Model Routing and Cascading for Efficient LLM Inference: A Survey](https://arxiv.org/abs/2603.04445) (FrugalGPT and successors)
* Ong et al., [RouteLLM: Learning to Route LLMs with Preference Data](https://arxiv.org/abs/2406.18665)
* [Cost-Saving LLM Cascades with Early Abstention](https://arxiv.org/abs/2502.09054)
* [Top-k learning to defer, with cascades as a special case](https://arxiv.org/abs/2504.12988)
* [Gatekeeper: confidence tuning for model cascades](https://arxiv.org/abs/2502.19335)
* Wang, Agarwal, Dudík, [Optimal and Adaptive Off-policy Evaluation in Contextual Bandits](https://proceedings.mlr.press/v70/wang17a.html) (IPS, DR, SWITCH)
* [Off-policy bandits with deficient support](https://par.nsf.gov/biblio/10170899-off-policy-bandits-deficient-support)
* Kostrikov et al., [Offline Reinforcement Learning with Implicit Q-Learning](https://arxiv.org/abs/2110.06169)
* Shah et al., [Offline Reinforcement Learning for Visual Navigation](https://arxiv.org/abs/2212.08244) (ReViND)
* Lee, Smith, Abbeel, [PEBBLE: Feedback-Efficient Interactive Reinforcement Learning](https://proceedings.mlr.press/v139/lee21i.html)
* Liu et al., [Visual-RFT: Visual Reinforcement Fine-Tuning](https://arxiv.org/abs/2503.01785)
* Mayr et al., [Learning of Parameters in Behavior Trees for Movement Skills](https://arxiv.org/abs/2109.13050)
* [An Empirical Study Into What Matters for Calibrating Vision-Language Models](https://arxiv.org/abs/2402.07417)
* Zheng et al., [Towards Optimal Correlational Object Search](https://arxiv.org/abs/2110.09991) (COS-POMDP)
* [Shields for Safe Reinforcement Learning](https://tugraz.elsevierpure.com/de/publications/shields-for-safe-reinforcement-learning/) and [compositional shield synthesis under partial observability](https://arxiv.org/abs/2509.12085)
* [DAgger Diffusion Navigation](https://arxiv.org/abs/2508.09444) (DAgger for navigation)
