# Task 3 target color change

On 2026-09-06 the physical target set changed to blue, replacement green, and
red. Blue is unchanged. The former pink and green cylinders are abandoned, so
their HSV samples and observed reliability do not qualify the replacement
targets.

Fresh replacement-green, red, blue-validation, and background samples were
collected on 2026-09-06. The first red/background set contained 25 background
clicks on a red-looking vertical region and was rejected rather than relabelled
by assumption. A separate red/background retry was then collected.

The adopted ranges are:

```yaml
blue:  {lower: [88, 180, 60], upper: [100, 255, 255]}
green: {lower: [63, 75, 95], upper: [78, 140, 170]}
red_1: {lower: [0, 205, 100], upper: [6, 255, 190]}
red_2: {lower: [168, 205, 100], upper: [179, 255, 190]}
```

The blue and replacement-green ranges each covered 100% of their accepted
samples with 0% sampled-background coverage. The red dual range covered 99.99%
of the retry red pixels with 0% retry-background coverage. These statistics do
not replace live detection, multi-frame confirmation, or different-lighting
tests.

## Live acceptance

After deployment, a 40-frame automated observation detected blue and red in
all 40 frames. Replacement green was not present in that automated view and
therefore produced no detections. The operator subsequently completed a visual
review of the replacement blue, green, and red targets and reported no obvious
misclassification. This accepts the current setup under the tested scene and
lighting only; it does not remove the need for multi-frame confirmation or
future near/far, shadow, overlap, and changed-lighting tests.

Sampler keys for the new target set are `b` (blue), `g` (replacement green),
`r` (red), and `n` (background). Yellow remains an optional distractor label.
