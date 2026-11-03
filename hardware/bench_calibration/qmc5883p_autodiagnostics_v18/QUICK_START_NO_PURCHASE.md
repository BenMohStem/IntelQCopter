# IntelQCopter v18 — 1-command no-purchase diagnostics

Extract into a **new** folder, open PowerShell there, with COM3 available and all other COM-port monitors closed. Props removed, ESC signals/power disconnected.

```powershell
.\tools\run_auto_tests.ps1 -Port COM3 -Suite triage -PowerMode warm
```

Then upload `results\SESSION_..._SEND_THIS.zip` to the diagnostic conversation. No USB unplugging, YKUSH, new tools, GUI, or manual intervention during individual tests. A warm reset is not a genuine sensor power cycle. If COM3 remains busy after bounded retries, the suite stops and saves the results archive.
