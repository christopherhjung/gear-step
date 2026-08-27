# gear-step

Parametric involute gear generator with STEP AP214 export. No dependencies,
`cargo build --release`.

```
gear-step -z 24 --module 2 --width 12 --bore 20 -o pinion.step
```

The same generator also compiles to WebAssembly and runs in a browser with a
3D viewer and a settings panel - see [In the browser](#in-the-browser).

## What it generates

External spur and helical gears, generated the way a rack cutter (hob) makes
them: involute flank plus a **true trochoidal root fillet** (not a tangent arc
fudge), so the root shape and the form diameter match a real hobbed gear.
Undercut appears by itself when `z` is too small, and is reported.

The solid is a closed manifold `MANIFOLD_SOLID_BREP` with **analytic
surfaces**, one face per profile segment:

| geometry | surface | edges |
|---|---|---|
| tip land, root, bore, holes | `CYLINDRICAL_SURFACE` | `CIRCLE` |
| keyway flanks, D flat, end faces | `PLANE` | `LINE` |
| involute flank, root fillet | `B_SPLINE_SURFACE_WITH_KNOTS` | `B_SPLINE_CURVE_WITH_KNOTS` |

Flanks are cubic B-splines that interpolate points on the exact involute and
the exact trochoid, so the error is the interpolation error between those
points — about 0.01 um at the default 16 points per flank, printed on every
run as *flank spline deviation*. Cylinders and planes are exact.

For helical gears the flank surface is a bicubic loft of the rotated control
net (the tip and root stay exact cylinders, their side edges become B-spline
helices); `--layers` sets the number of sections, default one per 30 deg of
twist, which keeps the same 0.01 um order.

Verified with OpenCASCADE: one solid, `BRepCheck_Analyzer` valid, volume
matching the analytic cross-section, and a point-to-solid distance check of
the exported flank against the closed-form involute (10 nm worst case, spur
and both helix hands).

## Parameters

Required: `-z/--teeth`, `-w/--width`, and a size (`--module`, `--pitch` or
`--dp`).

| group | options |
|---|---|
| size | `--module <m_n>` \| `--pitch <p>` \| `--dp <1/in>` |
| main | `-z`, `-w/--width`, `-a/--pressure-angle` (20), `--helix`, `--hand`, `--shift`, `--backlash` |
| basic rack | `--addendum` (1.0), `--dedendum` (1.25), `--fillet` (0.38) |
| bore | `--bore`, `--keyway none\|auto\|<b>x<t2>`, `--flat`, `--flat-depth`, `--double-flat`, `--bore-angle` |
| holes | `--holes`, `--hole-dia`, `--hole-circle` |
| output | `-o`, `--name`, `--svg`, `--phase`, `--flank-seg`, `--fillet-seg`, `--layers`, `--pin`, `-q` |

`--keyway auto` picks width `b` and hub depth `t2` from DIN 6885-1 for the given
bore; `t2` is measured from the bore wall, i.e. the groove roof sits at
`d/2 + t2` and the standard's toleranced dimension `d + t2` is printed in the
data sheet. Nominal sizes only — add your own fit allowances.

For D shafts use `--flat <across>`, where `across` is the usual motor-shaft
dimension from the flat to the opposite bore wall (a 6 mm shaft milled to
5.5 mm is `--bore 6 --flat 5.5`). `--flat-depth` takes the cut depth instead,
and `--double-flat <between>` gives two symmetric flats. `--bore-angle` rotates
the keyway or the flat; 0 points it at +Y.

Helical gears twist the outer surface only; the bore stays straight.

Everything is nominal geometry: no tip/root relief, no crowning, no lead or
profile modification, no tolerances or allowances beyond `--backlash`, which is
applied as a circumferential tooth thinning at the pitch circle (half of it per
flank). For a mating pair, put the full backlash on one gear or split it.

## Data sheet

Printed on every run: pitch/base/tip/root and **form** diameter, transverse and
normal tooth thickness, base tangent length `W` over `k` teeth, measurement over
pins (pin diameter defaults to 1.68·m), tooth height, centre distance with an
equal gear, and for helical gears the axial pitch and overlap ratio. Plus the
topology check of the exported shell.


## In the browser

The whole generator (geometry, STEP writer, SVG dump) compiles to
`wasm32-unknown-unknown` with no toolchain beyond stable Rust - no bindgen, no
npm, no bundler. `src/wasm.rs` exports a handful of `extern "C"` functions; the
page writes a block of `f64` parameters into linear memory, calls
`gear_generate`, and reads the mesh, the STEP text and the data sheet straight
back out.

```
rustup target add wasm32-unknown-unknown
./build.sh serve            # builds, then serves web/ on localhost:8080
```

`build.sh` produces two things:

| | |
|---|---|
| `web/` | `index.html` + `app.js` + `gear.wasm`, needs any static file server |
| `dist/gear-step.html` | the same app as **one** file, wasm inlined, opens from disk |

The page has the full parameter set of the CLI, a WebGL viewer for the display
mesh, the transverse profile with pitch/base/form/root/tip circles, and
downloads for STEP, STL and SVG. Every change regenerates the real solid: the
STEP the browser hands you is byte for byte what the CLI writes for the same
parameters (bar the timestamp in the header).

A few extras that only make sense interactively:

* **mesh pair** - draws the mate as the mirror image of the gear about the
  plane halfway to the second axis, turned by half a pitch so its teeth land
  in these gaps rather than on these teeth. That is an exact conjugate mesh at
  the printed centre distance, it counter-rotates on its own, and for a helical
  gear it comes out the opposite hand, which is what a parallel axis pair
  needs.
* **spin** - rotates both.
* **copy link** - the whole parameter set lives in the URL fragment.

The display mesh is a separate, purely triangular tessellation
(`src/mesh.rs`, with an ear clipping triangulator in `src/earcut.rs`); the
exported solid stays analytic. `cargo test` checks that the mesh is a closed
manifold whose volume matches the analytic cross-section for a spread of
parameter sets.

## Not implemented

Internal (ring) gears, racks, tip chamfers, hubs/bosses, non-involute profiles
and asymmetric flanks.

## Examples

```
# spur pinion, 20 mm bore with DIN 6885-1 keyway
gear-step -z 24 --module 2 --width 12 --bore 20

# small pinion, undercut corrected by profile shift
gear-step -z 10 --module 2 --width 8 --shift 0.45 --bore 8

# right hand helical, backlash allowance, SVG check plot
gear-step -z 31 --module 1.5 --width 20 --helix 20 --bore 12 \
          --backlash 0.06 --svg wheel.svg

# D profile bore for a 8 mm motor shaft with a 0.8 mm flat
gear-step -z 20 --module 1.5 --width 8 --bore 8 --flat 7.2

# 12 DP, 60 teeth, lightening holes
gear-step -z 60 --dp 12 --width 6 --bore 25 \
          --holes 5 --hole-dia 8 --hole-circle 60
```
