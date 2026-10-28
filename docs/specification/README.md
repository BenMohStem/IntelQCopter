# Cahier des charges syst?me

CDC-IQC-001, revision 1.0, 28 October 2026. French working specification, not flight acceptance or MIT affiliation.

- [PDF](cahier-des-charges.pdf)
- [Standalone HTML](cahier-des-charges.html)
- [Editable Markdown](cahier-des-charges.md)
- [Document QA and fingerprints](validation.json)

15 pages,45 requirement identifiers,12 verification cases. D means user decision; P means proposed target pending review; TBD means unresolved. Existing firmware, hardware and simulation are unchanged.

Build from repository root with Python and ReportLab installed:

```powershell
python tools/docs/build_specification.py .
```

The renderer uses Windows Arial fonts. Build dependencies are not vendored. Original physical logs remain in their existing evidence folders; this document adds no new hardware result.
