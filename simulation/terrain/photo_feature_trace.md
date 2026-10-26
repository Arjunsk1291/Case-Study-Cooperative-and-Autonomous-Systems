# Photo-derived terrain feature trace

This trace records how the supplied stills/video were interpreted. Feature presence and relative relationships are grounded in the media; metric coordinates, slopes, heights, and widths are not surveyed.

| Photo/video evidence | Visible feature | Simulation treatment | Confidence |
|---|---|---|---|
| IMG_9705.jpeg; video 0–12 s | Small irregular cobbles, long lane between pink/glazed façades, raised crosswise curb/threshold, concrete door landings | Staggered cobble units with small height variation; separate raised side curb and explicitly modeled hypothesized centre crossing | High that features exist; dimensions/crossing unknown |
| Video ~12–32 s; cb00e451…jpeg | Timber-clad Campus Store/pavilion beside cobbled route; cobble-to-slab change; columns along edge | Pavilion landmark, separate paving zones and column row | Medium-high; exact local coordinates/identity of adjacent pink façade uncertain |
| Video ~32–52 s | Broad plaza paved in large-format slabs, bands between paving types | Slab grid and transition checkpoint | High feature; sizes estimated |
| c429a667…jpeg; video ~76 s | Covered walkway, repeating round columns, four parallel metal tactile strips, raised-dot warning pad, circular flush cover/light | Column/canopy geometry, guidance rails, warning-dot array and flush covers | High feature; exact positions/dimensions estimated |
| 20471da7…jpeg; video ~44–52 s | Wet large slabs, puddles, long linear drain and a transverse drain at a surface change | Low-profile drains, wet patch masks and configurable slow/traction scenario | High feature; drain profile and wet friction are unknown |
| Video ~52–64 s | Tactile guide path through plaza; D/E wayfinding signage | Tactile strip and signboard landmark cues | Text/features visible; coordinates estimated |
| All supplied media | Pedestrians occupy parts of the walkway and plaza | Optional synthetic crossing agents for stop/yield testing | People visible; trajectories are artificial simulation cases |

## Route and safety rules

- Follow the ordered Block B → Campus Store/P → courtyard/passage → ramp transition → Block H checkpoints; do not choose arbitrary open areas.
- Treat ordinary stairs and unbridged water/drainage features as no-go. Prefer a verified accessible ramp/level path.
- Reduce speed on cobblestones, ramps and marked wet areas. Wet friction factors are tunable scenarios, not measured coefficients.
- Treat tactile rails/dots, flush covers and thin drain slots as low-level features; the 17 cm-high LiDAR may not detect them reliably.
- The curb is visibly raised, but the proposed lower central crossing is a modeling hypothesis. Measure the real profile and rover wheel/ground-clearance before assuming it is traversable.
- Unknown space is blocked until surveyed.
- The final Block H approach triggers a latched FINISHED state with both simulated motor commands zero.

## External campus references

- [THD campus map and access context](https://th-deg.de/campus-deggendorf)
- [THD campus/building guide](https://www.th-deg.de/Studierende/Antraege-und-Organisatorisches/infos-fuer-erstsemester/Quietschieguide_SoSe25.pdf)
- [THD landscape project](https://www.landschaftsarchitekten.net/fh-deggendorf-planung)
- [Campus extension / terrace context](https://www.bez-kock.de/de/projekte/119/erweiterung-der-technischen-hochschule-deggendorf-2015/)
- [Campus paving reference](https://www.godelmann.de/en/references/technical-university-of-applied-sciences)
- [Campus photographs / outdoor details](https://www.hl-lichttechnik.eu/projekte/fh-deggendorf)

These public references establish feature types and broad campus context. They do not provide metric survey coordinates or an authoritative route trace.
