# English university project report

Professional and Personal Project, IQC-PPP-002 revision2.0,28October2026.

- [Technical report PDF](technical-report.pdf)
- [Standalone HTML](technical-report.html)
- [Editable Markdown](technical-report.md)
- [31-entry bill of materials and quotation inventory](bill-of-materials.csv)
- [Historical evidence sources and hashes](evidence-manifest.json)
- [Document quality checks](validation.json)

26 pages,45 identified requirements,12 verification cases,rendered equations,two architecture/geometry diagrams and two original derived evidence figures. The CSV separates aircraft,test equipment,optional companion and future Pico/radio hardware. Prices are deliberately blank pending verified quotes.

This is an English university submission draft replacing the French document for that purpose; the earlier French specification remains preserved. Complete the cover fields,authentic personal reflection,contribution statement and university-rubric mapping before submission. Proposed targets are not verified aircraft performance. Motor restrictions and all physical acceptance gates remain unchanged.

Rebuild from the repository root with Python,ReportLab,Matplotlib,Pillow and Windows Arial fonts available:

```powershell
python tools/docs/build_university_report.py .
```

No environments or downloaded dependencies are vendored. The self-contained HTML embeds its figures. PDF mathematical expressions are rendered; editable equation source is in the Markdown. Existing source figures are copied without alteration and identified by the manifest. No new hardware,upload,simulation or flight test was conducted for this report.
