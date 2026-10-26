# RF edge copper clearance

Current project retains its folder. Native RF keepout excludes full 11mm
edge band and antenna projection on F/B copper. Clearing the former ground
bridge required splitting physical RF return and adding G2.3 (124,60.5), while
keeping three actual wires through G2's dedicated middle tap. 86 drill holes.
Native DRC/parity all 0, independent ground graph all14 common after wire fit.
No reliance on internal module ground. Masks and native3D regenerated, mask
visually reviewed. Failed unconnected/shorting route trials preserved.
GPIO logical interfaces and firmware unchanged; RF performance unmeasured.
User authorizes fresh zero-jumper candidate exploration by three subagents in
isolated scopes and useful stationary software tests afterwards. Motor outputs
remain disabled; physical rotations remain pending. Do not claim 0 physical risk.
