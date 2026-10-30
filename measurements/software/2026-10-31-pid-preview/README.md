# First PID calculation, zero roll-rate target

This small next step demonstrates one inner roll-rate PID only. The zero target means stopping rotation, not returning to a level angle. Xbox targets remain visible but do NOT drive this demonstration. No angle cascade, mixer, PWM or ESC adapter is connected. No motor output or tuning acceptance.

At every nominal1ms FIFO sample, e=0−ωx (rad/s). P=0.1e. I candidate=clip(I+0.05eΔt,±0.2). D raw=−0.001(ωx−previousωx)/Δt; a first-order filter withτ=0.02s smooths D. Its coefficient is Δt/(τ+Δt). First sample has zero derivative. Sum=P+I+D, output clipped±0.2. Outward integral growth during saturation is rejected, while integration that reduces saturation remains allowed. These illustrative coefficients produce dimensionless demo commands; they are not gains calculated from the aircraft dynamics and cannot drive flight motors. No Ki/Kd tuning claim.

C records contain sequence,P,I,D,bounded_sum. The graph shows them. Hold still, gently twist about drone X, then stop: P should oppose rate and D respond to rate change. I remembers accumulated rate error; a stopped tilted drone may retain I. No need to chase it back to zero or recalibrate. This demo is purely for understanding PID terms and safeguards. It is not a roll-angle controller or physical acceptance of feedback signs.

Commands: upload-attitude-preview --upload-port COM3, then view-attitude --port COM3. New combined firmware requires its own hardware timing/input check. Previous logs verify only previous source; newest Xbox-target log has953Trecords and both validity states with full configured endpoints, not an independently attested disconnect/axis-sign pass.

Teacher's previous quaternion illustration was already pushed as7e77c87. Do not add claims that this new PID has been flight-tested to that report.

Build and native actual-helper tests passed; no new hardware evidence. New source not uploaded. Code reading: pid is a pointer to PidPreview; pid->p accesses its P field. dt is seconds per FIFO sample. += adds a change into stored state. fminf/fmaxf implement clipping. && and || are logical AND/OR; ! negates the saturation-growth condition. The I candidate is committed only if it does not push farther into saturation. The stored previous_rate makes a discrete measurement derivative; initialized suppresses the artificial first-sample derivative. Full source is lessons/attitude-preview/main.cpp.

