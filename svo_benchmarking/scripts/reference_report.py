#!/usr/bin/env python3
import argparse
import json
from svo_ceres_benchmarking.reference_report import report

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='No-scale SE(3) ATE and 1 s RPE against a VIO reference')
    parser.add_argument('estimate')
    parser.add_argument('reference')
    parser.add_argument('output')
    args = parser.parse_args()
    print(json.dumps(report(args.estimate, args.reference, args.output), indent=2))
