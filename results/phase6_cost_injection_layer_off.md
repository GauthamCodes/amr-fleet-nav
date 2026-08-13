# Phase 6 - trajectory cost injection (layer_off)

Does `amr1`'s predicted trajectory become cost in
`amr2`'s **local** costmap? The probe reads
`/amr2/local_costmap/costmap_raw` at the cell `amr1`
is predicted to occupy 2.0 s ahead.

- `fleet_trajectory_layer.enabled`: **False**
- costmaps received: 87
- peer trajectories received: 126
- samples with the predicted cell in the window: **59**

| quantity | value |
|---|---|
| cells sampled | 59 |
| **samples with cost > 0** | **0** (0.0 %) |
| median cost at the predicted cell | **0** |
| max cost at the predicted cell | 0 |
| min cost at the predicted cell | 0 |
| cost the decay model predicts (mean) | 127.3 |
| **min lead, peer to probed cell** | **0.239 m** |
| inflation radius | 0.70 m |
| closest true separation | 1.529 m |

**Caveat, stated rather than buried:** the probed cell came
within 0.239 m of the peer at least once, inside the
0.70 m inflation radius. Those samples
cannot distinguish this layer from the obstacle layer's
inflation. The layer-off arm is the discriminator, not this
table.

Read this against `phase6_cost_injection_layer_off.md`. A positive
cost here only means something if the same cell, in the same
scenario, reads 0 with the layer disabled.

