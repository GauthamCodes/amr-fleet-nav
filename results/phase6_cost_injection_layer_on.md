# Phase 6 - trajectory cost injection (layer_on)

Does `amr1`'s predicted trajectory become cost in
`amr2`'s **local** costmap? The probe reads
`/amr2/local_costmap/costmap_raw` at the cell `amr1`
is predicted to occupy 2.0 s ahead.

- `fleet_trajectory_layer.enabled`: **True**
- costmaps received: 77
- peer trajectories received: 111
- samples with the predicted cell in the window: **50**

| quantity | value |
|---|---|
| cells sampled | 50 |
| **samples with cost > 0** | **50** (100.0 %) |
| median cost at the predicted cell | **145** |
| max cost at the predicted cell | 240 |
| min cost at the predicted cell | 137 |
| cost the decay model predicts (mean) | 125.9 |
| **min lead, peer to probed cell** | **0.208 m** |
| inflation radius | 0.70 m |
| closest true separation | 1.612 m |

**Caveat, stated rather than buried:** the probed cell came
within 0.208 m of the peer at least once, inside the
0.70 m inflation radius. Those samples
cannot distinguish this layer from the obstacle layer's
inflation. The layer-off arm is the discriminator, not this
table.

Read this against `phase6_cost_injection_layer_off.md`. A positive
cost here only means something if the same cell, in the same
scenario, reads 0 with the layer disabled.

