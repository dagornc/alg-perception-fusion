#!/usr/bin/env python3
"""Mesure la parité bit-à-bit Rust/Python de ALG_PERCEPTION_FUSION."""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
REFERENCE = ROOT / "reference" / "sim_perception_fusion.py"
SCENARIOS = ("nominal", "outliers", "edge", "dropout")


def data_line(command: list[str]) -> str:
    output = subprocess.run(command, check=True, capture_output=True, text=True).stdout
    lines = output.strip().splitlines()
    if len(lines) != 2:
        raise RuntimeError(f"sortie CSV inattendue ({len(lines)} lignes): {output!r}")
    return lines[1]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binaire", default=str(ROOT / "target" / "release" / "perception_fusion_rs"))
    parser.add_argument("--graines", nargs=2, type=int, default=[1000, 1050], metavar=("DEBUT", "FIN_EXCLUE"))
    parser.add_argument("--scenarios", nargs="*", choices=SCENARIOS, default=list(SCENARIOS))
    parser.add_argument("--steps", type=int, default=80)
    args = parser.parse_args()

    binary = Path(args.binaire)
    if not binary.is_file():
        print(f"ERREUR : binaire introuvable : {binary}", file=sys.stderr)
        return 2
    if args.graines[0] < 0 or args.graines[1] <= args.graines[0] or args.steps <= 0:
        print("ERREUR : plage de graines ou steps invalide", file=sys.stderr)
        return 2

    total = 0
    differences = 0
    for scenario in args.scenarios:
        identical = 0
        for seed in range(args.graines[0], args.graines[1]):
            common = [str(seed), scenario, str(args.steps)]
            python_line = data_line([sys.executable, str(REFERENCE), *common])
            rust_line = data_line([str(binary), *common])
            total += 1
            if python_line == rust_line:
                identical += 1
            else:
                differences += 1
                if differences <= 10:
                    print(f"DIVERGENCE [{scenario}] seed={seed}")
                    print(f"  python: {python_line}")
                    print(f"  rust  : {rust_line}")
        print(f"scénario {scenario:8s} : {identical}/{args.graines[1] - args.graines[0]} identiques")

    print(f"TOTAL : {total - differences}/{total} identiques")
    print(f"COMPARAISONS : {total}")
    print(f"ECARTS : {differences}")
    if differences == 0:
        print("PARITÉ BIT-À-BIT VÉRIFIÉE ✓")
        return 0
    print("PARITÉ NON VÉRIFIÉE")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
