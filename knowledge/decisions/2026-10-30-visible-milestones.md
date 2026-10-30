# Decision: visible milestones and bounded calibration work

Date:2026-10-30. Source: user asks to replan because testing feels endless and its purpose is unclear.

Supersedes the work order and blanket preview-blocking dependencies in the earlier PLAN and D-0001 summary. Does not supersede disabled motor outputs, preservation of evidence, calibration provenance, or physical feedback-direction acceptance before actuators.

Existing calibration is frozen for development. Next deliverable is a motor-disabled attitude preview. Physical direction validation blocks motor feedback acceptance, not preview implementation or software/model work. Precision scale characterization is optional unless reference evidence identifies a relevant discrepancy. One combined physical check per milestone is the default; a targeted repeat must name the missing evidence. Extra campaigns require an explained decision rather than automatic repetition. No hardware passes are invented to clear a dependency.

See docs/PLAN.md for seven milestones, stopping conditions and deferred work. Current production source's user coefficient edit remains unresolved; review it before integration and do not silently overwrite it.
