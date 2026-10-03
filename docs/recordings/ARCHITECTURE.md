---
kind: architecture
domain: recordings
status: current
verified: 2026-10-02
---

# Recordings -- architecture

A recorded walk is a sequence of real camera frames taken in a real house,
in order, each stored with the decision the system made on it at the time.
Recorded walks are the only real-world dataset this project has. This
domain covers how walks are written, where they are kept, how they are
reviewed and scored, how they are replayed under another model, and how
they get human ground truth. Read this for the shape and the reasons. Read
the [engineering spec](../engineering/recordings/ENGINEERING.md) for
routes, file schemas, parameters and procedures.

## Purpose

Before the car exists, there is no other evidence about real rooms. The
simulator renders flat-shaded geometry, and every "the sim works" result
is a statement about that geometry. Walks recorded from a phone on a
wheeled rig at floor height are what the rest of the project measures
against:

- **Model and wording comparisons.** Replaying a walk holds the pixels
  fixed and varies one thing. It is the only controlled comparison the
  project has. Two live walks also differ in where the operator pointed
  the phone.
- **Perception scoring** (the perception domain). It is scored against
  per-frame human labels stored beside each walk.
- **Offline missions.** A walk can be replayed through the real mission
  loop, frame by frame, as a body made of photographs (the body and
  simulator domains).
- **Diagnosis.** A scorecard flags the failure shapes the project has hit:
  one action for every frame, long stalls, oscillation, and a target that
  flickers in and out of view.

Losing the corpus would lose every finding measured on it. One corpus has
already been lost to a storage policy that was in git but never deployed,
which is why durability is a design concern here and not an operations
detail.

## Components and boundaries

```text
 twin (Robot view, "Record this walk") --frames, finish--> write routes
                                                            |
            mounted by the brain (local)  or  by the walks service (cloud)
                                                            v
                                   walk store (one interface, two backends)
                                     directory  |  object bucket (retained,
                                                |  versioned)
                                                v
   walks service: list, view, download, label, annotate, score, replay,
                  summarise, delete; serves the review console's API
        |  scorer (pure functions, plus an optional model judge)
        |  replayer --> cloud vision, the move question (deployed)
        v
   review console (static page)

 offline, by hand: label proposer --> candidate labels --> a person
                   adjudicates --> ground-truth labels
 readers: perception scorer, replay-as-a-robot, replay demo
```

- **Write routes.** Two routes store a frame and mark a walk finished.
  Both are pure storage: they touch no robot, runner or mission state.
  Whichever process holds the storage mounts them.
- **Walk store.** One interface over a local directory or an object
  bucket, with an identical layout on both. One factory picks the backend
  from configuration. It validates every walk and file name itself,
  because an object key does not collapse a parent-directory element the
  way a filesystem path does.
- **Walks service.** The review and evaluation API, and the cloud home of
  the write routes. It never talks to the robot or the brain. The only
  service it calls is the cloud vision service, and only for replay.
- **Scorer.** Deterministic metrics over a walk's per-frame log. An
  optional model judge looks at sampled frames, and an optional collision
  check looks at frames that commanded FORWARD.
- **Replayer.** Re-asks each frame of a walk through the deployed cloud
  vision service's move question, under a chosen model and wording.
- **Label proposer.** Ranks frames by a different detector's confidence,
  so a person can adjudicate the uncertain band. It never writes ground
  truth.
- **Review console.** A static page served from the same CDN as the twin.

**What a walk holds, by kind of judgement.** These four are kept in
separate files on purpose:

| Judgement | Who made it | Trust |
|---|---|---|
| The per-frame log | the model or the mission, live | Evidence ABOUT the model, never the truth |
| The scorecard | the scorer | Advisory. Its thresholds were calibrated on six walks. |
| The operator's label | a person, as a workflow state | A curation choice, not a measurement |
| Adjudicated visibility labels | a person, frame by frame | The only ground truth for perception scoring |

**Boundaries.** The perception scorer is owned by the perception domain
and reads the ground-truth labels, never the log. Tier metrics share the
bucket and the walks service but belong to the operations domain. They
are about missions, not walks: one row per mission, kept in per-day
containers beside the walks. Operations owns where metrics rows are
stored; this domain owns the walk list and its filter, which shows walks
only, and no walk route reads or deletes a metrics container (handoff 4c,
2026-10-03; before it, the console listed each day as an empty walk and
its Delete removed a day of mission rows).
The twin owns the decision to record and the walk's name. This domain owns
everything from the first stored byte onwards.

## Decisions

### One store interface, two backends, one layout

**Decision.** Walks live behind one storage interface with a directory
backend and an object-bucket backend. The layout is byte-for-byte the same
on both, and a single factory chooses between them.

**Rejected:**

- **A network file system volume.** It held the corpus until 2026-09-04,
  and it was the one component that would have kept the deployment in a
  VPC after everything else left (`PLAN-aws-cost-redesign.md` section 2).
- **Reshaping the format during the move.** It would have made every
  existing walk unreadable to the tools that score it.

**Trade-off.** Appending to a log on the bucket is read-modify-write. That
is safe only because a walk has exactly one writer, one phone posting in
order. The backends are not perfectly identical in behaviour; the
engineering spec lists where they differ.

### The write path follows the storage, not the brain

**Decision.** The write routes are a mountable unit. The brain mounts them
when it runs locally against a directory. The walks service mounts them in
the cloud, where it holds the bucket's credentials.

**Rejected.** Writing only through the brain. The brain is going back onto
the robot (`PLAN-brain-relocation.md`), and a robot on someone's floor
should not carry cloud credentials in order to store a JPEG.

**Trade-off.** Recording is a property of whichever process the twin's
brain URL names. A locally run brain writes to the laptop's disk by
default, and on 2026-09-13 that had left 18 of 22 walks with no cloud
copy. The sync discipline in the engineering spec is the current fix.

### Only storage routes may be forwarded between brains

**Decision.** A brain that cannot store a walk may forward the two write
routes to a peer that can. Mission routes are never forwarded.

**Rejected.** Proxying the whole brain connection. A brain's robot client
is bound to one robot for the life of the process, so a forwarded mission
call would tick the wrong robot (`PLAN-teleop-robot.md`, "Recording
proxy").

### Review lives in its own service with its own secret

**Decision.** Listing, scoring, replaying and deleting walks run in a
service separate from the brain, under a different shared secret.

**Rejected.** More routes on the brain. Review would go down whenever the
mission server restarts, and its availability would depend on where the
brain runs today. Deleting the only dataset is also a different privilege
from starting a mission.

### Ground truth is human, adjudicated, and stored apart from every model output

**Decision.** Perception is scored only against adjudicated visibility
labels. A walk without them is refused, not scored against its own log. A
proposer may sort frames for review, but it writes a candidate file only.
The "adjudicated" list names only the frames a person actually looked at.
The proposer must not be the detector under test.

**Rejected.** Scoring against the per-frame log. On one search walk the
cloud model claimed the target on 34 frames of a storage bin. Also
rejected: auto-filling labels from the shipped detector, which would mark
its own homework.

**One exception, stated.** A walk where the target never appears may carry
a walk-level negative assertion: all frames are labelled not-visible, and
the frames actually reviewed are listed.

### Scores are advisory and never overwrite a person

**Decision.** The scorecard is stored beside the operator's label, never
inside it. The console shows the two side by side.

**Rejected.** One merged label. "Who decided this walk was bad" would
become unanswerable, and a scorer calibrated on six walks could overwrite
a judgement.

### Replay asks the deployed service, and an incomplete replay is not evidence

**Decision.** Replay asks the deployed service the move question, prompt,
parsing and allow-list included. A replay that got back too few of its
frames is stored but left unscored, so it can never read as a low score.

**Not yet the same question.** Two inputs differ from a live call today:
replay labels every frame as JPEG whatever format it was recorded in, and
it never sends the rooms already searched, which a live mission does. A
replay therefore answers "what would this model say with no search
memory". The engineering spec records both as known gaps.

**Rejected:**

- **Calling the model provider directly.** That answers "what does a copy
  of the prompt say" and stops tracking the real route.
- **Scoring whatever came back.** Three wordings of one walk once scored
  identically on 3, 9 and 2 surviving frames, and it read as "wording
  makes no difference".

**Trade-off.** Replay needs the vision service up and reachable with the
vision service's own credentials. When it is down, the replay still
completes and is stored, but as incomplete and unscored; nothing else on
the walks service depends on it.

### The bucket outlives its stack, and deletion is recoverable

**Decision.** The bucket is retained on stack deletion and on replacement.
It is versioned, so a deleted walk or a rewritten log stays recoverable
for a window. Walks never expire. Download exports expire quickly. The
bucket's stack depends on no other stack.

**Rejected.** Default deletion behaviour. A retain policy that sat in git
undeployed is how the previous corpus volume was lost.

### Downloads redirect, decided by backend and never by size

**Decision.** On the bucket backend, a whole-walk download is written as
an export and the caller is redirected to a short-lived signed URL. The
directory backend streams the zip.

**Rejected.** Streaming through the function. Its response size cap is
below the largest walks. Also rejected: redirecting only large walks,
which would pass every test and fail on the walks that matter.

### Recording never interrupts a walk

**Decision.** Frames are saved fire-and-forget, and failures are counted
and shown. The "finished" marker is best-effort. It records what produced
the walk, and it is also what makes the console score a walk on its own:
the console scores finished, unscored walks when it loads, and leaves a
walk without the marker (still recording, or cut off) to a person's
explicit Evaluate, so a fragment is never judged as a whole walk. Every
saved frame stays scorable either way.

**Rejected.** Blocking capture on a save. A dropped connection would stop
a walk that cannot be repeated cheaply.

## Contracts

| Neighbour | Direction | Protocol | Ownership |
|---|---|---|---|
| Twin | twin calls write routes on its configured brain URL | HTTP/JSON, shared secret | Twin owns the walk name and the sequence number, allocated at dispatch. This domain owns the stored bytes. |
| Brain | brain mounts the write routes, or forwards them | in-process; HTTP for the proxy | The brain decides whether recording is allowed. Storage routes only. |
| Cloud vision | walks service calls `/navigate` and reads its allow-lists | HTTP/JSON, the vision service's secret | Vision owns the model ids. The console never carries a copy. |
| Model provider | walks service calls it for the judge and the collision check | managed API under the function's role | Off unless the deployment opts in |
| Perception scorer | reads walk directories and their labels | files | Perception owns scoring. This domain owns the labels' format and their provenance. |
| Replay-as-a-robot and the replay demo | read a walk's frames and log in order | files | Body and simulator own playback |
| Metrics (operations) | the brain posts mission rows to the walks service | HTTP/JSON | Operations owns the rows, where they are stored, and the fix to that layout. They share the store; this domain owns the walk list's filter that keeps them out of it. |
| Bucket stack | walks service role attaches its access policy | cloud IAM | The bucket stack owns the bucket and the policy, so they survive the compute stack being replaced |

## Failure modes and resilience targets

| Failure | Response | Target |
|---|---|---|
| A tab closes or the battery dies mid-walk | No finish marker. Frames already saved stay. The console does not score it on its own; a person's Evaluate does. | Every saved frame stays scorable, and an unfinished walk is never auto-scored as if complete. |
| Overlapping calls give two frames the same number | The twin allocates numbers at dispatch | No frame silently overwrites another |
| A frame lands after its walk ended | The twin drops it and counts it as orphaned | No frame is filed under the wrong walk |
| The brain has recording off | Refused, or forwarded to a peer, and the twin checks at start | A walk never silently records nothing |
| A cloud deployment that selects the bucket backend but names no bucket | The walks service refuses to start | No frame lands on ephemeral disk |
| A cloud deployment that does not select the bucket backend at all | Not guarded: it falls back to the directory backend on the function's ephemeral disk | Open: the guard keys on the backend choice, not on running in the cloud (engineering spec) |
| A mistaken delete | Previous object versions remain for the retention window | A deleted walk is recoverable within the window |
| The vision service is down, throttled or refusing during replay | Each frame that still fails after retrying is counted. The replay is stored, flagged incomplete and unscored, and the route answers success with that record. The route refuses outright only when no vision service is configured at all. | The console still lists and scores walks, and a failed replay never reads as a model result |
| A replay loses frames | Stored, flagged incomplete, unscored | A partial replay never reads as a model result |
| Walks recorded on the laptop only | Manual sync to the bucket after each rig session | Every walk carrying adjudicated labels has a cloud copy |
| A name with a separator or a parent element | Refused by both the routes and the store | No traversal on either backend |
| A day of metrics rows is deleted from the console as an "empty walk" | Guarded: metrics containers are not listed as walks, and every walk route answers 404 for one | Met (handoff 4c, 2026-10-03) |

## Open questions

- **Should the laptop brain write straight to the bucket?** That removes
  the manual sync. It needs credentials on the recording path. The user
  decides.
- **Should replay become a job?** A job would return an id to poll, where
  today one synchronous request outlives its HTTP response.
- **Is replay a deployed feature or a local-only tool?** As templated,
  the deployed walks service cannot replay. Either the deployment is given
  what replay needs, or replay is declared local-only. The analysis, the
  failure signatures and how to check the live stack are in the
  [engineering spec's Known gaps](../engineering/recordings/ENGINEERING.md#known-gaps).
- **Is the judge on in the deployed walks service?** The template does not
  opt in. Checking the deployed stack settles it.
- **Which walks to record next.** Two out-of-vocabulary searches and a
  control under the tiered policy, so corroboration can be measured live
  (`PLAN-onboard-perception.md`, "What to record next").
