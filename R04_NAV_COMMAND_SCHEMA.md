# CAS-R04-NAV-CMD-1 — compact offline route-plan draft

The phone dashboard's **Send small plan to main ESP32** panel accepts a JSON document with:

- `schema`: exactly `CAS-R04-NAV-CMD-1`
- `mode`: exactly `route_pending`
- `waypoints`: 1–12 records, each with an `id` string

The main firmware stores this JSON in RAM (maximum body size 1,199 bytes). It returns `execution_enabled: false`. The plan is a draft only and is lost when the main ESP32 reboots.

## Example with unlocalized coordinates

Use null coordinates where the log does not provide a validated metric map frame. Do not invent x/y values from GPS drift or IMU yaw alone.

```json
{
  "schema": "CAS-R04-NAV-CMD-1",
  "mode": "route_pending",
  "route_id": "BED_TO_WASHROOM_DRAFT",
  "frame": "unlocalized_survey",
  "execution": "draft_only",
  "max_speed_m_s": 0.10,
  "obstacle_stop_cm": 35,
  "waypoints": [
    {
      "id": "A_BED",
      "x_m": null,
      "y_m": null,
      "heading_deg": null,
      "source": "operator_marked",
      "confidence": 0.0
    },
    {
      "id": "B_CORRIDOR",
      "x_m": null,
      "y_m": null,
      "heading_deg": null,
      "source": "operator_marked",
      "confidence": 0.0
    },
    {
      "id": "C_WASHROOM",
      "x_m": null,
      "y_m": null,
      "heading_deg": null,
      "source": "operator_marked",
      "confidence": 0.0
    }
  ]
}
```

This example validates as a draft, but does not authorize movement. The current R04 Navigation controller holds the rover stationary while a validated indoor localization and route executor are absent. The numbers `max_speed_m_s` and `obstacle_stop_cm` are metadata only in this version; they are not applied to the motor controller.

The offline analysis should return a plan only when its frame, coordinate provenance, uncertainty, waypoint geometry and clearance assumptions are documented. Any coordinate that cannot be supported by measurements should remain null or be explicitly labelled as an operator-provided estimate.
