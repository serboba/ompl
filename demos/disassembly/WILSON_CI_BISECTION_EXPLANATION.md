# Wilson CI-Guarded Bisection Search: Easy Explanation

## What Problem Are We Solving?

When we sample points on a sphere to test if they're valid (not in obstacles), we get **noisy results**. With only 32 samples, a validity rate of 0.10 (10%) could easily be 0.15 (15%) or 0.05 (5%) just by chance. 

**Old Problem:** The original bisection search used raw validity rates directly:
- If validity < 0.10 → reduce radius
- If validity > 0.50 → increase radius  
- If validity in [0.10, 0.50] → accept

But with noisy data, this could make **wrong decisions** and converge to the wrong radius!

---

## What is Wilson Confidence Interval (CI)?

Think of it like a **margin of error** for opinion polls. If a poll says "60% support candidate A", they usually add "±3% margin of error", meaning the true support is likely between 57% and 63%.

**Wilson CI does the same for our validity rate:**
- If we observe 8 valid out of 32 samples (25% validity)
- Wilson CI might say: "The true validity is likely between 12% and 42%"
- This accounts for the uncertainty from having only 32 samples

**Formula (simplified):**
```
CI = observed_rate ± margin_of_error
margin_of_error = z_score × sqrt(rate × (1-rate) / sample_count)
```

For 95% confidence, z_score = 1.96 (this is a standard statistical value).

---

## How Does Bisection with Wilson CI Work?

### Step-by-Step Process:

#### 1. **Initial Setup**
- Start with a **lower bound** (e.g., radius = 0.000001) and **upper bound** (e.g., radius = 10.0)
- Target validity range: [0.10, 0.50] (10% to 50%)
- Sample size: 32 samples per test

#### 2. **Test Initial Bounds**
- **Step 0:** Sample at lower bound → get validity rate
- **Step 1:** Sample at upper bound → get validity rate
- This tells us the search space

#### 3. **Bisection Loop** (for each step):

**a) Test Midpoint:**
- Calculate `mid = (lower + upper) / 2`
- Sample 32 points at radius = `mid`
- Count valid samples (e.g., 8 valid out of 32 = 25% validity)

**b) Compute Wilson CI:**
- Calculate 95% confidence interval: `[CI_low, CI_high]`
- Example: If 8/32 valid, CI might be [12%, 42%]

**c) Make Decision Based on CI (NOT raw validity!):**

```
IF CI_low > 0.50:
    → CI is entirely ABOVE target range
    → Radius is TOO SMALL (need larger radius)
    → Increase lower bound: lower = mid
    
ELSE IF CI_high < 0.10:
    → CI is entirely BELOW target range  
    → Radius is TOO LARGE (need smaller radius)
    → Decrease upper bound: upper = mid
    
ELSE IF CI_low >= 0.10 AND CI_high <= 0.50:
    → CI is entirely WITHIN target range
    → PERFECT! Accept this radius and stop
    
ELSE:
    → CI OVERLAPS target range (uncertain!)
    → INCONCLUSIVE - need more samples
    → Resample with 2× samples (64 instead of 32)
    → Recompute CI and try again
```

#### 4. **Adaptive Resampling (When Inconclusive):**
- If CI overlaps target → we're uncertain
- **Solution:** Double the sample size (32 → 64 → 128 → ...)
- More samples = narrower CI = clearer decision
- Continue until:
  - Clear decision is made, OR
  - Max sample size reached (512), then use best radius found

#### 5. **Convergence:**
- Repeat until `(upper - lower) < tolerance` OR max steps reached
- Return the best radius found

---

## Example Scenario:

**Target:** Validity rate between 10% and 50%

**Step 2:** Test radius = 5.0
- Sample 32 points → 8 valid (25% validity)
- Wilson CI: [12%, 42%]
- **Decision:** CI overlaps target [10%, 50%] → **INCONCLUSIVE**
- **Action:** Resample with 64 samples

**After Resampling:** 64 samples → 15 valid (23.4% validity)
- Wilson CI: [14%, 35%]  
- **Decision:** CI is within target [10%, 50%] → **ACCEPT!**
- **Result:** Use radius = 5.0

---

## Why Is This Better?

### Old Approach (Raw Validity):
- ❌ Makes decisions on noisy data
- ❌ Can converge to wrong radius
- ❌ No way to handle uncertainty

### New Approach (Wilson CI):
- ✅ Only makes decisions when statistically confident
- ✅ Handles uncertainty by resampling
- ✅ More robust to noise
- ✅ Mathematically sound (standard statistical method)

---

## Key Insight:

**We never update bisection bounds if the CI overlaps the target range!**

This prevents wrong decisions when we're uncertain. Instead, we:
1. Get more samples (reduce uncertainty)
2. Recompute CI (narrower interval)
3. Make decision when confident

This is like saying: *"I'm not sure, let me check more carefully before deciding."*

---

## Visualization Note:

The current convergence plot shows:
- ✅ Radius evolution
- ✅ Validity rate evolution  
- ✅ Decision directions (↑/↓)
- ❌ **Does NOT show Wilson CI bounds**
- ❌ **Does NOT show when resampling occurred**
- ❌ **Does NOT show inconclusive results**

**Future Enhancement:** Could add:
- CI bounds as shaded regions
- Resampling steps marked differently
- Inconclusive results highlighted


