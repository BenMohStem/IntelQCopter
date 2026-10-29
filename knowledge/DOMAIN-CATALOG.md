# Project knowledge catalog

Small domain records store facts, assumptions, user preferences, evidence and open gates. They do not store private reasoning or chat transcripts.

1. Read `index.json`, then only the relevant domain.
2. Follow provenance to the actual artifact or source; status is explicit.
3. After a prompt changes durable information, update its domain and append a compact event. Acknowledgements with no new information need no empty commit.
4. If a domain exceeds 25 records / 15 KB, or mixes unrelated topics, split it deliberately; keep stable IDs and update the catalog. Preserve originals and superseded decisions.
5. Run the domain checker. It checks paths, hashes, IDs and provenance shape; it does not certify scientific truth, remote content or flight readiness.

Domains: presentation preferences; current report editions; illustrative control assumptions/checks; safety and evidence rules. The canonical runtime state/evidence/task graph and transactional claims remain separate in the development knowledge base; this catalog does not refresh hardware fingerprints.
