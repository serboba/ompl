# MAB-SSRRT-1L Planner Workflow Documentation

## PR Review Perspective

**Question**: Is keeping the code as-is (without refactoring) acceptable?

**Answer**: **Yes, absolutely acceptable.** 

**Reasoning**:
- The refactoring saved ~300 lines but added complexity through new abstractions
- The current code is functional, tested, and working correctly
- Not all refactorings provide net benefit - sometimes the original structure is clearer
- The code is well-organized with clear method separation
- Adding abstractions can sometimes increase cognitive load rather than decrease it
- **Recommendation**: Keep as-is unless there's a specific maintainability issue

---

## 1. Overall Planner Workflow (solve() method)

```
┌─────────────────────────────────────────────────────────────────┐
│                    MAB-SSRRT-1L::solve()                         │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │  1. INITIALIZATION                  │
        │  - Add start states to tree         │
        │  - Allocate samplers                │
        │  - Initialize sampling arms         │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  2. BURN-IN PHASE                   │
        │  setupAdaptiveSphereSampling()      │
        │  ┌───────────────────────────────┐  │
        │  │ a. Test uniform sampling      │  │
        │  │    (early exit if easy)       │  │
        │  └───────────────────────────────┘  │
        │  ┌───────────────────────────────┐  │
        │  │ b. Adaptive radius search     │  │
        │  │    - Find optimal radius      │  │
        │  │    - Collect valid samples    │  │
        │  │    - Seed cylinder sampler    │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  3. MAIN PLANNING LOOP              │
        │  while (!termination_condition)     │
        │  {                                  │
        │    ┌─────────────────────────────┐  │
        │    │ 3.1 Get Sample              │  │
        │    │   getSample()               │  │
        │    │   ┌───────────────────────┐ │  │
        │    │   │ - Select arm (MAB)    │ │  │
        │    │   │ - Check goal bias     │ │  │
        │    │   │ - Generate sample     │ │  │
        │    │   │ - Validate sample     │ │  │
        │    │   │ - Update MAB rewards  │ │  │
        │    │   └───────────────────────┘ │  │
        │    └─────────────────────────────┘  │
        │              │                      │
        │              ▼                      │
        │    ┌─────────────────────────────┐  │
        │    │ 3.2 Extend Tree             │  │
        │    │   if (goal_sample)          │  │
        │    │     handleGoalSamplePath()  │  │
        │    │   else                      │  │
        │    │     handleNormalSamplePath()│  │
        │    │   ┌───────────────────────┐ │  │
        │    │   │ - Find nearest node   │ │  │
        │    │   │ - Apply distance limit│ │  │
        │    │   │ - Check motion valid  │ │  │
        │    │   │ - Add to tree         │ │  │
        │    │   │ - Check goal reached  │ │  │
        │    │   └───────────────────────┘ │  │
        │    └─────────────────────────────┘  │
        │              │                      │
        │              ▼                      │
        │    ┌─────────────────────────────┐  │
        │    │ 3.3 Check Solution         │  │
        │    │   if (goal_reached)        │  │
        │    │     break;                 │  │
        │    └─────────────────────────────┘  │
        │  }                                  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  4. BUILD SOLUTION PATH             │
        │  - Traverse tree from goal to start│
        │  - Create PathGeometric            │
        │  - Add to ProblemDefinition        │
        └─────────────────────────────────────┘
                      │
                      ▼
                 [RETURN STATUS]
```

---

## 2. Burn-In Phase Workflow (Adaptive Sphere Sampling)

```
┌─────────────────────────────────────────────────────────────────┐
│          setupAdaptiveSphereSampling()                          │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │  performInitialUniformCheck()       │
        │  ┌───────────────────────────────┐  │
        │  │ Test N uniform samples        │  │
        │  │ Compute validity rate         │  │
        │  │ if (rate > threshold)         │  │
        │  │   return true (skip burn-in)  │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
              [Skip burn-in?]
              │           │
         YES  │           │ NO
              │           │
              ▼           ▼
        [Use uniform]  performAdaptiveBurnin()
              │           │
              │           ▼
              │   ┌───────────────────────────────┐
              │   │ Initialize:                   │
              │   │ - radius = startRadius       │
              │   │ - step = 0                   │
              │   │ - consecutive_full = 0       │
              │   └───────────────────────────────┘
              │           │
              │           ▼
              │   ┌───────────────────────────────┐
              │   │ while (radius > minRadius)    │
              │   │ {                             │
              │   │   ┌─────────────────────────┐ │
              │   │   │ 1. Generate samples     │ │
              │   │   │    sphere->collectQuasi  │ │
              │   │   │    RandomSamples(radius)│ │
              │   │   └─────────────────────────┘ │
              │   │           │                   │
              │   │           ▼                   │
              │   │   ┌─────────────────────────┐ │
              │   │   │ 2. Compute validity     │ │
              │   │   │    computeValidityRate()│ │
              │   │   │    ┌─────────────────┐ │ │
              │   │   │    │ For each sample: │ │ │
              │   │   │    │ - Convert to    │ │ │
              │   │   │    │   state         │ │ │
              │   │   │    │ - Check motion  │ │ │
              │   │   │    │   from origin   │ │ │
              │   │   │    │ - Mark valid/   │ │ │
              │   │   │    │   invalid       │ │ │
              │   │   │    └─────────────────┘ │ │
              │   │   │    Return: valid/total │ │
              │   │   └─────────────────────────┘ │
              │   │           │                   │
              │   │           ▼                   │
              │   │   ┌─────────────────────────┐ │
              │   │   │ 3. Check exit conds     │ │
              │   │   │    if (full_validity)   │ │
              │   │   │      exit early         │ │
              │   │   │    if (in_target_range) │ │
              │   │   │      break              │ │
              │   │   │    if (max_steps)       │ │
              │   │   │      break              │ │
              │   │   └─────────────────────────┘ │
              │   │           │                   │
              │   │           ▼                   │
              │   │   ┌─────────────────────────┐ │
              │   │   │ 4. Adjust radius        │ │
              │   │   │    adjustRadiusBasedOn  │ │
              │   │   │    Validity()           │ │
              │   │   │    ┌─────────────────┐ │ │
              │   │   │    │ if (rate < min) │ │ │
              │   │   │    │   shrink radius │ │ │
              │   │   │    │   keep samples  │ │ │
              │   │   │    │ if (rate > max) │ │ │
              │   │   │    │   grow radius   │ │ │
              │   │   │    │   discard       │ │ │
              │   │   │    └─────────────────┘ │ │
              │   │   └─────────────────────────┘ │
              │   │           │                   │
              │   │           ▼                   │
              │   │   step++                      │
              │   │ }                             │
              │   └───────────────────────────────┘
              │           │
              │           ▼
              │   ┌───────────────────────────────┐
              │   │ finalizeBurnin()              │
              │   │ - Set sphere->bestRadius      │
              │   │ - Check early exit condition  │
              │   │ - Append valid samples        │
              │   └───────────────────────────────┘
              │
              └───────────┐
                          │
                          ▼
                  [Burn-in Complete]
                  Valid samples collected
                  Optimal radius found
```

---

## 3. Sampling Workflow (getSample() method)

```
┌─────────────────────────────────────────────────────────────────┐
│                    getSample()                                   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │  Check Prerequisites                │
        │  - samplingArms_ initialized?       │
        │  - sphere ready?                    │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  PHASE 1: Select Sampling Arm       │
        │  selectSamplingArm()                 │
        │  ┌───────────────────────────────┐  │
        │  │ 1. Check forced uniform      │  │
        │  │    (after cylinder streak)    │  │
        │  └───────────────────────────────┘  │
        │  ┌───────────────────────────────┐  │
        │  │ 2. MAB selection              │  │
        │  │    mab_sampler_->chooseArm()  │  │
        │  │    Returns: 0, 1, or 2        │  │
        │  │    0 → UNIFORM                │  │
        │  │    1 → CYLINDER_UP            │  │
        │  │    2 → CYLINDER_DOWN          │  │
        │  └───────────────────────────────┘  │
        │  ┌───────────────────────────────┐  │
        │  │ 3. Fallback check            │  │
        │  │    if (cylinder && no points) │  │
        │  │      → UNIFORM                │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  PHASE 2: Goal Bias Check           │
        │  shouldSampleGoal()                 │
        │  ┌───────────────────────────────┐  │
        │  │ if (random() < goalBias)      │  │
        │  │   sample from goal region     │  │
        │  │   return goal sample          │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  PHASE 3: Generate Sample           │
        │  generateAndValidateSample()        │
        │  ┌───────────────────────────────┐  │
        │  │ if (CYLINDER_UP/DOWN)         │  │
        │  │   sampleFromCylinder()        │  │
        │  │   ┌─────────────────────────┐ │  │
        │  │   │ - Get point from       │ │  │
        │  │   │   cylinder sampler     │ │  │
        │  │   │ - Convert to state     │ │  │
        │  │   │ - Validate motion      │ │  │
        │  │   │ - Add to valid points  │ │  │
        │  │   │   if valid              │ │  │
        │  │   └─────────────────────────┘ │  │
        │  │ else (UNIFORM)                │  │
        │  │   sampleUniformHypothesis()   │  │
        │  │   - Sample all dimensions     │  │
        │  │   - Validate motion          │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  PHASE 4: Update Statistics         │
        │  updateSamplingStatistics()        │
        │  ┌───────────────────────────────┐  │
        │  │ 1. Update MAB rewards         │  │
        │  │    updateRewards(isValid, path)│  │
        │  │    ┌─────────────────────────┐ │  │
        │  │    │ Compute reward:         │ │  │
        │  │    │ - Valid: +1.0          │ │  │
        │  │    │ - Invalid: 0.0         │ │  │
        │  │    │ Update MAB arm          │ │  │
        │  │    └─────────────────────────┘ │  │
        │  │ 2. Update cylinder streak     │  │
        │  │    if (cylinder && valid)     │  │
        │  │      streak++                 │  │
        │  │    else                       │  │
        │  │      streak = 0               │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
              [Return sample state]
```

---

## 4. MAB Selection Workflow (selectSamplingArm())

```
┌─────────────────────────────────────────────────────────────────┐
│              selectSamplingArm()                                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │  Check Forced Uniform               │
        │  ┌───────────────────────────────┐  │
        │  │ if (forcedUniformAfterStreak  │  │
        │  │     > 0 &&                    │  │
        │  │     cylinderValidStreak_ >=   │  │
        │  │     forcedUniformAfterStreak) │  │
        │  │ {                             │  │
        │  │   return UNIFORM              │  │
        │  │   reset streak                │  │
        │  │ }                             │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  MAB Selection                      │
        │  ┌───────────────────────────────┐  │
        │  │ mab_sampler_->chooseArm()     │  │
        │  │                               │  │
        │  │ Uses UCB (Upper Confidence    │  │
        │  │ Bound) algorithm:             │  │
        │  │                               │  │
        │  │ UCB_i = μ_i + c * √(ln(n)/n_i)│ │
        │  │                               │  │
        │  │ where:                        │  │
        │  │ - μ_i = average reward        │  │
        │  │ - n = total pulls             │  │
        │  │ - n_i = pulls for arm i      │  │
        │  │ - c = exploration constant    │  │
        │  │                               │  │
        │  │ Returns arm with highest UCB  │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  Map Index to Arm                   │
        │  ┌───────────────────────────────┐  │
        │  │ switch (armIndex)            │  │
        │  │   case 0: → UNIFORM          │  │
        │  │   case 1: → CYLINDER_UP      │  │
        │  │   case 2: → CYLINDER_DOWN    │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  Fallback Check                     │
        │  ┌───────────────────────────────┐  │
        │  │ if (CYLINDER &&               │  │
        │  │    sphere->getAllValidPoints()│  │
        │  │    .empty())                  │  │
        │  │ {                             │  │
        │  │   return UNIFORM              │  │
        │  │ }                             │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
              [Return selected arm]
```

---

## 5. Tree Extension Workflow

```
┌─────────────────────────────────────────────────────────────────┐
│         Tree Extension (handleNormalSamplePath)                 │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │  Determine Motion Origin            │
        │  ┌───────────────────────────────┐  │
        │  │ if (UNIFORM)                 │  │
        │  │   origin = UNIFORM            │  │
        │  │ else                          │  │
        │  │   origin = CYLINDER           │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  Handle Empty Tree                  │
        │  ┌───────────────────────────────┐  │
        │  │ if (nearestMotion == nullptr) │  │
        │  │   addMotion(sample, nullptr) │  │
        │  │   checkSolution()            │  │
        │  │   return                     │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  Apply Distance Limit               │
        │  ┌───────────────────────────────┐  │
        │  │ if (CYLINDER)                │  │
        │  │   dstate = sample (direct)   │  │
        │  │ else (UNIFORM)               │  │
        │  │   dstate = applyDistanceLimit│  │
        │  │   ┌───────────────────────┐  │  │
        │  │   │ Interpolate from     │  │  │
        │  │   │ nearest to sample    │  │  │
        │  │   │ Limit to maxDistance │  │  │
        │  │   └───────────────────────┘  │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  Add Motion to Tree                 │
        │  addMotionToTree()                   │
        │  ┌───────────────────────────────┐  │
        │  │ 1. Create new Motion node    │  │
        │  │ 2. Copy state                │  │
        │  │ 3. Set parent pointer        │  │
        │  │ 4. Set bornFrom (origin)     │  │
        │  │ 5. Add to nearest neighbors │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  Check Solution                      │
        │  checkAndUpdateSolution()            │
        │  ┌───────────────────────────────┐  │
        │  │ if (goal->isSatisfied(state)) │  │
        │  │   solution = motion           │  │
        │  │   return true                 │  │
        │  │ else                          │  │
        │  │   update approximate solution │  │
        │  │   return false                │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
```

---

## 6. Goal Sample Path Workflow

```
┌─────────────────────────────────────────────────────────────────┐
│         handleGoalSamplePath()                                  │
└─────────────────────────────────────────────────────────────────┘
                              │
                              ▼
        ┌─────────────────────────────────────┐
        │  Find Best Parent for Goal           │
        │  findBestMotionForGoal()             │
        │  ┌───────────────────────────────┐  │
        │  │ Priority order:              │  │
        │  │ 1. Non-exhausted UNIFORM     │  │
        │  │    nodes (preferred)         │  │
        │  │ 2. Non-exhausted CYLINDER   │  │
        │  │    nodes                     │  │
        │  │ 3. Nearest neighbor         │  │
        │  │    (fallback)               │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  Apply Distance Limit               │
        │  applyDistanceLimit()                │
        │  ┌───────────────────────────────┐  │
        │  │ Interpolate from parent to    │  │
        │  │ goal, limited by maxDistance  │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  Validate Motion                     │
        │  ┌───────────────────────────────┐  │
        │  │ if (!checkMotion(parent, goal))│  │
        │  │   mark parent as exhausted     │  │
        │  │   return false                │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
                      │
                      ▼
        ┌─────────────────────────────────────┐
        │  Add Motion & Check Solution         │
        │  ┌───────────────────────────────┐  │
        │  │ addMotion(goal, parent, GOAL) │  │
        │  │ checkAndUpdateSolution()      │  │
        │  └───────────────────────────────┘  │
        └─────────────────────────────────────┘
```

---

## 7. Component Interaction Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                    MAB-SSRRT-1L Planner                         │
└─────────────────────────────────────────────────────────────────┘
                              │
        ┌─────────────────────┼─────────────────────┐
        │                     │                     │
        ▼                     ▼                     ▼
┌───────────────┐    ┌───────────────┐    ┌───────────────┐
│   MAB         │    │  Adaptive     │    │   RRT Tree    │
│   Manager     │    │  Sphere       │    │   Manager     │
│               │    │  Sampler      │    │               │
│ - chooseArm() │    │               │    │ - addMotion() │
│ - update()    │    │ - collectQuasi│    │ - findNearest │
│               │    │   Random()    │    │ - checkMotion │
│ 3 Arms:       │    │ - getSample()│    │               │
│ 0: UNIFORM    │    │ - PCA filter  │    │ Motion nodes: │
│ 1: CYL_UP     │    │               │    │ - state      │
│ 2: CYL_DOWN   │    │ Valid points: │    │ - parent      │
│               │    │ - stored      │    │ - bornFrom    │
│ UCB Algorithm │    │ - used for    │    │               │
│ (exploration/ │    │   cylinder     │    │ Nearest NN:   │
│  exploitation)│    │   sampling     │    │ - KD-tree     │
└───────────────┘    └───────────────┘    └───────────────┘
        │                     │                     │
        │                     │                     │
        └─────────────────────┼─────────────────────┘
                              │
                              ▼
                    ┌──────────────────┐
                    │  State Space      │
                    │  - 2D: XY        │
                    │  - 6D: RPY+XYZ   │
                    └──────────────────┘
```

---

## 8. Data Flow Diagram

```
┌──────────────┐
│   YAML Config │
└──────┬───────┘
       │
       ▼
┌──────────────────────┐
│  loadYAMLConfig()    │
│  - Load parameters   │
│  - Set member vars   │
└──────┬───────────────┘
       │
       ▼
┌──────────────────────┐
│  Constructor         │
│  - Create MAB        │
│  - Initialize arms   │
└──────┬───────────────┘
       │
       ▼
┌──────────────────────┐
│  solve()             │
│  ┌────────────────┐  │
│  │ 1. Burn-in     │  │
│  │    ↓           │  │
│  │    Valid       │  │
│  │    samples     │  │
│  │    ↓           │  │
│  │    Optimal     │  │
│  │    radius      │  │
│  └────────────────┘  │
│  ┌────────────────┐  │
│  │ 2. Main loop   │  │
│  │    ↓           │  │
│  │    MAB selects │  │
│  │    arm         │  │
│  │    ↓           │  │
│  │    Generate    │  │
│  │    sample      │  │
│  │    ↓           │  │
│  │    Validate    │  │
│  │    ↓           │  │
│  │    Update MAB  │  │
│  │    rewards     │  │
│  │    ↓           │  │
│  │    Extend tree │  │
│  │    ↓           │  │
│  │    Check goal  │  │
│  └────────────────┘  │
└──────┬───────────────┘
       │
       ▼
┌──────────────────────┐
│  Solution Path        │
│  - Traverse tree      │
│  - Build path         │
└───────────────────────┘
```

---

## 9. Key Algorithms Summary

### 9.1 Adaptive Burn-In Algorithm

```
Algorithm: Adaptive Radius Search
─────────────────────────────────
Input:  startRadius, minRadius, maxSteps
Output: bestRadius, validSamples

1. currentRadius ← startRadius
2. step ← 0
3. consecutiveFull ← 0
4. 
5. WHILE currentRadius > minRadius AND step < maxSteps:
6.     Generate samples at currentRadius
7.     validityRate ← computeValidityRate()
8.     
9.     IF validityRate == 1.0:
10.        consecutiveFull++
11.        IF consecutiveFull >= threshold:
12.            BREAK (early exit)
13.    
14.    IF validityRate in [minRate, maxRate]:
15.        BREAK (target range found)
16.    
17.    IF validityRate < minRate:
18.        currentRadius ← currentRadius * exp(-shrinkStep)
19.        KEEP valid samples
20.    ELSE IF validityRate > maxRate:
21.        currentRadius ← currentRadius * exp(growStep)
22.        DISCARD samples
23.    
24.    step++
25. 
26. RETURN currentRadius, validSamples
```

### 9.2 MAB Selection Algorithm

```
Algorithm: UCB-based Arm Selection
──────────────────────────────────
Input:  MAB with 3 arms, sliding window
Output: selectedArm (0, 1, or 2)

1. IF forcedUniformAfterStreak > 0:
2.     IF cylinderValidStreak >= forcedUniformAfterStreak:
3.         RETURN UNIFORM (arm 0)
4. 
5. FOR each arm i in [0, 1, 2]:
6.     μ_i ← average reward for arm i
7.     n_i ← number of pulls for arm i
8.     n ← total number of pulls
9.     
10.    IF n_i == 0:
11.        UCB_i ← ∞ (explore unexplored arm)
12.    ELSE:
13.        UCB_i ← μ_i + c * √(ln(n) / n_i)
14. 
15. selectedArm ← argmax(UCB_i)
16. 
17. IF selectedArm is CYLINDER AND no valid points:
18.     RETURN UNIFORM (fallback)
19. 
20. RETURN selectedArm
```

### 9.3 Tree Extension Algorithm

```
Algorithm: RRT Tree Extension
──────────────────────────────
Input:  sampleState, nearestMotion, selectedArm
Output: newMotion (if valid), solutionFound

1. IF nearestMotion == nullptr:
2.     newMotion ← addMotion(sampleState, nullptr)
3.     RETURN checkSolution(newMotion)
4. 
5. IF selectedArm == CYLINDER:
6.     extendedState ← sampleState (direct connection)
7. ELSE: // UNIFORM
8.     distance ← distance(nearestMotion, sampleState)
9.     extendedState ← interpolate(nearestMotion, sampleState, 
10.                                min(distance, maxDistance))
11. 
12. IF checkMotion(nearestMotion, extendedState):
13.     newMotion ← addMotion(extendedState, nearestMotion)
14.     RETURN checkSolution(newMotion)
15. ELSE:
16.     RETURN false
```

---

## 10. Configuration Parameters

### 10.1 Goal Bias
- `uniform_goal_bias`: Probability of sampling goal when UNIFORM arm selected
- `sphere_goal_bias`: Probability of sampling goal when CYLINDER arm selected

### 10.2 MAB Settings
- `mab_uniform_sphere_window_size`: Sliding window size for UCB algorithm

### 10.3 Burn-In Parameters
- `adaptive_quasirandom_sample_size`: Number of samples per radius test
- `adaptive_start_radius`: Initial sampling radius
- `adaptive_min_radius`: Minimum radius threshold
- `adaptive_shrink_step`: Radius reduction factor
- `adaptive_grow_step`: Radius increase factor
- `adaptive_min_expected_validity_rate`: Lower bound for target range
- `adaptive_max_expected_validity_rate`: Upper bound for target range
- `adaptive_burnin_max_steps`: Maximum iterations

### 10.4 MAB Rewards
- `uniform_sampler_fixed_valid_reward`: Reward for valid UNIFORM sample
- `uniform_sampler_invalid_reward`: Reward for invalid UNIFORM sample
- `sphere_sampler_fixed_valid_reward`: Reward for valid CYLINDER sample
- `sphere_sampler_invalid_reward`: Reward for invalid CYLINDER sample

### 10.5 Cylinder Configuration
- `cylinder_radius_offset_multiplier`: Cylinder radius offset
- `fibonacci_jitter_radius`: Jitter for Fibonacci sampling
- `cylinder_sampling_radius_multiplier`: Radius multiplier for sampling
- `pca_filter_top_percent`: PCA filter threshold
- `enableDynamicCylinderPCA`: Recompute PCA axis dynamically

---

## Summary

The MAB-SSRRT-1L planner follows a clear workflow:

1. **Initialization**: Load config, create MAB, initialize samplers
2. **Burn-in**: Discover constraint directions, find optimal radius
3. **Planning Loop**: MAB-guided sampling → validation → tree extension → goal check
4. **Solution**: Build path from goal to start

The code is well-structured with clear method separation. Keeping it as-is is a reasonable decision if the refactoring didn't provide sufficient benefit.
