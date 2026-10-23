# Advanced validation and repair — optional reading

For everyday lessons: understand the change, build if requested, run it on the real board when available, keep the original result, and commit separately. A compiled or prepared lesson is not physically accepted.

The gate tool stores hash-chained observations with source/configuration/evidence fingerprints and dependency receipts. Physical acceptance additionally requires an actual bench identity and an operator's explicit attestation. It cannot independently prove that an operator took a measurement. The first programme deliberately has no embedded fingerprint or protocol; its purpose is learning serial output.

`qcopter.ps1 status` shows the active small-step catalogue. Historical events for the retired, larger catalogue remain preserved; they do not accept any new lesson. Source changes conservatively invalidate previous source-bound acceptance, so repeat the simple earlier smoke checks with a new build.

If06 fails after07–10 were prepared, use the implementation commit of06 **before** those later lessons were added:

```powershell
.\qcopter.ps1 repair --step ACTUAL-STEP-ID --checkpoint ACTUAL-COMMIT-SHA --branch codex/repair-example --destination C:\Users\Mohamed\Desktop\Work\QCopter-Repair
```

Replace all placeholders; this example is not a current physical test. The tool verifies the commit's Step-ID and rejects checkpoints containing implemented descendants. It creates a new branch/worktree at06, leaving07–10 out of the active code. Fix and test06, then implement a fresh07. Old failed code and evidence remain in their original branch/Git history. No reset or force-push is needed.

The new tiny first-lesson implementation has its own `Step-ID: 01-serial` checkpoint. Earlier diagnostic checkpoints belong to the older plan, kept as reference.
