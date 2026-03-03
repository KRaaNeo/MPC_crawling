"""
VISPA Crawling Controller — Two-stage hierarchical locomotion control.

Stage 1: CentroidalNMPC (20 Hz) — Centroidal dynamics + momentum envelope
Stage 2: WholeBodyQP   (125 Hz) — Full multibody dynamics + contact constraints
"""
