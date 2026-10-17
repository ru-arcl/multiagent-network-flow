"""The original data files (D:/temp/data/dmrpp/...) are not available; they came from an
unseeded Math.random() stream (Problem.main), so the published instances cannot be
recovered. mppflow.experiments.generate(seed, folder) regenerates the same families, file
names and formats with a seeded stream; the same seed gives byte-identical files. The
families share the one stream in the order they run, so a family's files also depend on the
families generated before it.
"""
import hashlib
import os

import _common

from mppflow.experiments import generate


def digest(seed, families=("crowded",), only=None):
    d = _common.scratch_dir()
    files = sorted(f for f in generate(seed, d, families=list(families), write_text=only is None)
                   if only is None or os.path.relpath(f, d).startswith(only))
    h = hashlib.sha256()
    for f in files:
        h.update(os.path.relpath(f, d).encode())
        with open(f, "rb") as fh:
            h.update(fh.read())
    return len(files), sorted(os.path.relpath(f, d) for f in files)[:2], h.hexdigest()[:16]


for seed in (42, 42, 7):
    print("seed", seed, "->", digest(seed))

# the n2puzzle files of seed 1 depend on whether "crowded" ran before them
for fams in (["n2puzzle"], ["n2puzzle", "crowded"], ["crowded", "n2puzzle"]):
    n, _, h = digest(1, fams, only="n2puzzle")
    print("seed 1, families", ",".join(fams), "-> n2puzzle .dat files:", n, h)
