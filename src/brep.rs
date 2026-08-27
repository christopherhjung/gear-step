//! Builds the solid directly from analytic profile segments and writes it as
//! ISO 10303-21 (AP214).
//!
//! One face per profile segment:
//!   Arc   -> CYLINDRICAL_SURFACE bounded by CIRCLEs
//!   Line  -> PLANE (spur) / B_SPLINE_SURFACE (helical)
//!   Curve -> B_SPLINE_SURFACE_WITH_KNOTS bounded by B_SPLINE_CURVE_WITH_KNOTS
//! Plus one planar end face top and bottom. Edges are shared between
//! neighbouring faces, so the shell is a genuine closed manifold.

use crate::nurbs::{self, Curve};
use crate::profile::Seg;
use std::collections::HashMap;
use std::sync::atomic::{AtomicI64, Ordering};

pub struct Model {
    /// outer contour, rotates linearly from z0 to z1 by `twist`
    pub outer: Vec<Seg>,
    /// straight through holes (never twisted)
    pub holes: Vec<Vec<Seg>>,
    pub z0: f64,
    pub z1: f64,
    pub twist: f64,
    /// axial sections used for the helical loft
    pub layers: usize,
    pub deg: usize,
}

pub struct Stats {
    pub faces: usize,
    pub edges: usize,
    pub verts: usize,
    pub rings: usize,
    pub planar: usize,
    pub cylindrical: usize,
    pub spline: usize,
    pub entities: usize,
}

struct Doc {
    lines: Vec<String>,
    cache: HashMap<String, usize>,
}

impl Doc {
    fn add(&mut self, s: String) -> usize {
        self.lines.push(s);
        self.lines.len()
    }
    fn shared(&mut self, s: String) -> usize {
        if let Some(&i) = self.cache.get(&s) {
            return i;
        }
        let i = self.add(s.clone());
        self.cache.insert(s, i);
        i
    }
    fn point(&mut self, p: [f64; 3]) -> usize {
        self.shared(format!(
            "CARTESIAN_POINT('',({},{},{}))",
            r(p[0]),
            r(p[1]),
            r(p[2])
        ))
    }
    fn dir(&mut self, d: [f64; 3]) -> usize {
        self.shared(format!("DIRECTION('',({},{},{}))", r(d[0]), r(d[1]), r(d[2])))
    }
    fn axis(&mut self, o: [f64; 3], z: [f64; 3], x: [f64; 3]) -> usize {
        let (po, dz, dx) = (self.point(o), self.dir(z), self.dir(x));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{},#{},#{})", po, dz, dx))
    }
    fn vertex(&mut self, p: [f64; 3]) -> usize {
        let q = self.point(p);
        self.shared(format!("VERTEX_POINT('',#{})", q))
    }
    fn bspline(&mut self, c: &Curve) -> usize {
        let pts: Vec<String> = c
            .ctrl
            .iter()
            .map(|p| format!("#{}", self.point(*p)))
            .collect();
        let m: Vec<String> = c.mult.iter().map(|x| x.to_string()).collect();
        let k: Vec<String> = c.knots.iter().map(|x| r(*x)).collect();
        self.add(format!(
            "B_SPLINE_CURVE_WITH_KNOTS('',{},({}),.UNSPECIFIED.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
            c.deg,
            pts.join(","),
            m.join(","),
            k.join(",")
        ))
    }
}

fn r(v: f64) -> String {
    let v = if v == 0.0 || !v.is_finite() { 0.0 } else { v };
    let mut s = format!("{:.9}", v);
    if s.contains('.') {
        while s.ends_with('0') {
            s.pop();
        }
    }
    if s.ends_with('.') {
        s.push('0');
    }
    s
}

fn rot(p: [f64; 2], a: f64) -> [f64; 2] {
    let (s, c) = a.sin_cos();
    [c * p[0] - s * p[1], s * p[0] + c * p[1]]
}

/// Per contour bookkeeping: one entry per segment junction / segment.
struct Ring {
    v_bot: Vec<usize>,
    v_top: Vec<usize>,
    e_bot: Vec<usize>,
    e_top: Vec<usize>,
    e_side: Vec<usize>,
    faces: Vec<usize>,
}

pub fn write(model: &Model, name: &str) -> (String, Stats) {
    let mut d = Doc { lines: Vec::new(), cache: HashMap::new() };
    let mut st = Stats {
        faces: 0,
        edges: 0,
        verts: 0,
        rings: 0,
        planar: 0,
        cylindrical: 0,
        spline: 0,
        entities: 0,
    };
    let mut oriented: HashMap<(usize, bool), usize> = HashMap::new();
    let mut rings: Vec<Ring> = Vec::new();

    let contours: Vec<(&Vec<Seg>, bool)> = std::iter::once((&model.outer, true))
        .chain(model.holes.iter().map(|h| (h, false)))
        .collect();

    for (ci, (segs, is_outer)) in contours.iter().enumerate() {
        let twist = if *is_outer { model.twist } else { 0.0 };
        let helical = twist.abs() > 1e-12;
        let layers = if helical { model.layers.max(4) } else { 2 };
        let vparams = nurbs::uniform_params(layers);
        let zs: Vec<f64> = (0..layers)
            .map(|i| model.z0 + (model.z1 - model.z0) * vparams[i])
            .collect();
        let angs: Vec<f64> = vparams.iter().map(|v| twist * v).collect();

        let n = segs.len();
        let mut ring = Ring {
            v_bot: Vec::new(),
            v_top: Vec::new(),
            e_bot: Vec::new(),
            e_top: Vec::new(),
            e_side: Vec::new(),
            faces: Vec::new(),
        };

        // ---- vertices and the vertical edges ---------------------------
        for s in segs.iter() {
            let p = s.start();
            let b = rot(p, angs[0]);
            let t = rot(p, *angs.last().unwrap());
            let vb = d.vertex([b[0], b[1], zs[0]]);
            let vt = d.vertex([t[0], t[1], *zs.last().unwrap()]);
            ring.v_bot.push(vb);
            ring.v_top.push(vt);
            let e = if helical {
                let pts: Vec<[f64; 3]> = (0..layers)
                    .map(|i| {
                        let q = rot(p, angs[i]);
                        [q[0], q[1], zs[i]]
                    })
                    .collect();
                let c = nurbs::interpolate(&pts, &vparams, model.deg);
                let g = d.bspline(&c);
                d.add(format!("EDGE_CURVE('',#{},#{},#{},.T.)", vb, vt, g))
            } else {
                let po = d.point([b[0], b[1], zs[0]]);
                let dv = d.dir([0.0, 0.0, 1.0]);
                let vec = d.add(format!("VECTOR('',#{},{})", dv, r(model.z1 - model.z0)));
                let line = d.add(format!("LINE('',#{},#{})", po, vec));
                d.add(format!("EDGE_CURVE('',#{},#{},#{},.T.)", vb, vt, line))
            };
            ring.e_side.push(e);
        }

        // ---- profile edges at the bottom and the top -------------------
        let base_curves: Vec<Option<Curve>> = segs
            .iter()
            .map(|s| match s {
                Seg::Curve(pts) => {
                    let p3: Vec<[f64; 3]> = pts.iter().map(|p| [p[0], p[1], 0.0]).collect();
                    let up = nurbs::chord_params(&p3);
                    Some(nurbs::interpolate(&p3, &up, model.deg))
                }
                _ => None,
            })
            .collect();

        for (i, s) in segs.iter().enumerate() {
            let j = (i + 1) % n;
            for lay in 0..2usize {
                let (z, ang) = if lay == 0 {
                    (zs[0], angs[0])
                } else {
                    (*zs.last().unwrap(), *angs.last().unwrap())
                };
                let (v0, v1) = if lay == 0 {
                    (ring.v_bot[i], ring.v_bot[j])
                } else {
                    (ring.v_top[i], ring.v_top[j])
                };
                let geom = match s {
                    Seg::Line(a, b) => {
                        let (pa, pb) = (rot(*a, ang), rot(*b, ang));
                        let dx = [pb[0] - pa[0], pb[1] - pa[1]];
                        let l = dx[0].hypot(dx[1]).max(1e-15);
                        let po = d.point([pa[0], pa[1], z]);
                        let dv = d.dir([dx[0] / l, dx[1] / l, 0.0]);
                        let vec = d.add(format!("VECTOR('',#{},{})", dv, r(l)));
                        d.add(format!("LINE('',#{},#{})", po, vec))
                    }
                    Seg::Arc { c, r: rr, a0, .. } => {
                        let cc = rot(*c, ang);
                        let a0r = *a0 + ang;
                        let ax = d.axis(
                            [cc[0], cc[1], z],
                            [0.0, 0.0, 1.0],
                            [a0r.cos(), a0r.sin(), 0.0],
                        );
                        d.add(format!("CIRCLE('',#{},{})", ax, r(*rr)))
                    }
                    Seg::Curve(_) => {
                        let g = base_curves[i].as_ref().unwrap().transformed(ang, z);
                        d.bspline(&g)
                    }
                };
                let same = match s {
                    Seg::Arc { a0, a1, .. } => a1 > a0,
                    _ => true,
                };
                let e = d.add(format!(
                    "EDGE_CURVE('',#{},#{},#{},{})",
                    v0,
                    v1,
                    geom,
                    if same { ".T." } else { ".F." }
                ));
                if lay == 0 {
                    ring.e_bot.push(e)
                } else {
                    ring.e_top.push(e)
                }
            }
        }

        // ---- lateral faces --------------------------------------------
        for (i, s) in segs.iter().enumerate() {
            let j = (i + 1) % n;
            let mut oe: Vec<(usize, bool)> = Vec::new();
            if *is_outer {
                oe.push((ring.e_bot[i], true));
                oe.push((ring.e_side[j], true));
                oe.push((ring.e_top[i], false));
                oe.push((ring.e_side[i], false));
            } else {
                oe.push((ring.e_bot[i], false));
                oe.push((ring.e_side[i], true));
                oe.push((ring.e_top[i], true));
                oe.push((ring.e_side[j], false));
            }
            // outward normal = travel direction x Z
            let (mp, tg) = s.mid_tangent();
            let dirn = if *is_outer { 1.0 } else { -1.0 };
            let out = [dirn * tg[1], -dirn * tg[0]];

            let (surf, same) = match s {
                Seg::Arc { c, r: rr, .. } => {
                    st.cylindrical += 1;
                    let ax = d.axis([c[0], c[1], model.z0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
                    let s_id = d.add(format!("CYLINDRICAL_SURFACE('',#{},{})", ax, r(*rr)));
                    // surface normal is radially outward from the arc centre
                    let rad = [mp[0] - c[0], mp[1] - c[1]];
                    (s_id, rad[0] * out[0] + rad[1] * out[1] > 0.0)
                }
                Seg::Line(a, b) if !helical => {
                    st.planar += 1;
                    let pa = rot(*a, angs[0]);
                    let pb = rot(*b, angs[0]);
                    let l = (pb[0] - pa[0]).hypot(pb[1] - pa[1]).max(1e-15);
                    let ex = [(pb[0] - pa[0]) / l, (pb[1] - pa[1]) / l, 0.0];
                    let ax = d.axis([pa[0], pa[1], model.z0], [out[0], out[1], 0.0], ex);
                    (d.add(format!("PLANE('',#{})", ax)), true)
                }
                other => {
                    st.spline += 1;
                    // control net: profile curve control points, swept
                    let (base, udeg, uknots, umult): (Vec<[f64; 3]>, usize, Vec<f64>, Vec<usize>) =
                        match other {
                            Seg::Line(a, b) => (
                                vec![[a[0], a[1], 0.0], [b[0], b[1], 0.0]],
                                1,
                                vec![0.0, 1.0],
                                vec![2, 2],
                            ),
                            Seg::Curve(_) => {
                                let cv = base_curves[i].as_ref().unwrap();
                                (cv.ctrl.clone(), cv.deg, cv.knots.clone(), cv.mult.clone())
                            }
                            _ => unreachable!(),
                        };
                    // sweep every control point and interpolate across v
                    let mut rows: Vec<Vec<usize>> = Vec::new();
                    let mut vkn = (vec![0.0, 1.0], vec![2usize, 2], 1usize);
                    for cp in &base {
                        let traj: Vec<[f64; 3]> = (0..layers)
                            .map(|i| {
                                let q = rot([cp[0], cp[1]], angs[i]);
                                [q[0], q[1], zs[i]]
                            })
                            .collect();
                        let cv = if helical {
                            nurbs::interpolate(&traj, &vparams, model.deg)
                        } else {
                            nurbs::interpolate(&traj, &vparams, 1)
                        };
                        vkn = (cv.knots.clone(), cv.mult.clone(), cv.deg);
                        rows.push(cv.ctrl.iter().map(|p| d.point(*p)).collect());
                    }
                    let grid: Vec<String> = rows
                        .iter()
                        .map(|row| {
                            let v: Vec<String> = row.iter().map(|i| format!("#{}", i)).collect();
                            format!("({})", v.join(","))
                        })
                        .collect();
                    let um: Vec<String> = umult.iter().map(|x| x.to_string()).collect();
                    let uk: Vec<String> = uknots.iter().map(|x| r(*x)).collect();
                    let vm: Vec<String> = vkn.1.iter().map(|x| x.to_string()).collect();
                    let vk: Vec<String> = vkn.0.iter().map(|x| r(*x)).collect();
                    let s_id = d.add(format!(
                        "B_SPLINE_SURFACE_WITH_KNOTS('',{},{},({}),.UNSPECIFIED.,.F.,.F.,.F.,({}),({}),({}),({}),.UNSPECIFIED.)",
                        udeg,
                        vkn.2,
                        grid.join(","),
                        um.join(","),
                        vm.join(","),
                        uk.join(","),
                        vk.join(",")
                    ));
                    // surface normal = d/du x d/dv = travel x Z = outward
                    (s_id, true)
                }
            };
            let f = face(&mut d, &mut oriented, &[oe], surf, same);
            ring.faces.push(f);
            st.faces += 1;
        }
        rings.push(ring);
        let _ = ci;
    }

    // ---- end caps ------------------------------------------------------
    let mut caps = Vec::new();
    for (lay, (z, nz)) in [(model.z0, -1.0f64), (model.z1, 1.0f64)].iter().enumerate() {
        let mut bounds: Vec<Vec<(usize, bool)>> = Vec::new();
        for (ci, ring) in rings.iter().enumerate() {
            let e = if lay == 0 { &ring.e_bot } else { &ring.e_top };
            let outer = ci == 0;
            // bottom cap: outer loop reversed, hole loops forward; top: swapped
            let fwd = if lay == 0 { !outer } else { outer };
            let mut lp: Vec<(usize, bool)> = e.iter().map(|&x| (x, fwd)).collect();
            if !fwd {
                lp.reverse();
            }
            bounds.push(lp);
        }
        let ax = d.axis([0.0, 0.0, *z], [0.0, 0.0, *nz], [1.0, 0.0, 0.0]);
        let pl = d.add(format!("PLANE('',#{})", ax));
        st.planar += 1;
        st.faces += 1;
        caps.push(face(&mut d, &mut oriented, &bounds, pl, true));
    }

    let mut all: Vec<usize> = rings.iter().flat_map(|r| r.faces.clone()).collect();
    all.extend(caps);
    st.verts = rings.iter().map(|r| r.v_bot.len() * 2).sum();
    st.edges = rings.iter().map(|r| r.e_bot.len() * 3).sum();
    st.rings = 2 * (rings.len() - 1);

    let refs: Vec<String> = all.iter().map(|i| format!("#{}", i)).collect();
    let shell = d.add(format!("CLOSED_SHELL('',({}))", refs.join(",")));
    let brep = d.add(format!("MANIFOLD_SOLID_BREP('{}',#{})", esc(name), shell));
    let world = d.axis([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);
    let text = finish(&mut d, name, brep, world);
    st.entities = d.lines.len();
    (text, st)
}

fn face(
    d: &mut Doc,
    oriented: &mut HashMap<(usize, bool), usize>,
    bounds: &[Vec<(usize, bool)>],
    surf: usize,
    same: bool,
) -> usize {
    let mut refs = Vec::new();
    for (bi, lp) in bounds.iter().enumerate() {
        let oe: Vec<String> = lp
            .iter()
            .map(|&(e, f)| {
                let id = *oriented.entry((e, f)).or_insert_with(|| {
                    d.lines.push(format!(
                        "ORIENTED_EDGE('',*,*,#{},{})",
                        e,
                        if f { ".T." } else { ".F." }
                    ));
                    d.lines.len()
                });
                format!("#{}", id)
            })
            .collect();
        let el = d.add(format!("EDGE_LOOP('',({}))", oe.join(",")));
        refs.push(format!(
            "#{}",
            if bi == 0 {
                d.add(format!("FACE_OUTER_BOUND('',#{},.T.)", el))
            } else {
                d.add(format!("FACE_BOUND('',#{},.T.)", el))
            }
        ));
    }
    d.add(format!(
        "ADVANCED_FACE('',({}),#{},{})",
        refs.join(","),
        surf,
        if same { ".T." } else { ".F." }
    ))
}

fn finish(d: &mut Doc, name: &str, brep: usize, world: usize) -> String {
    let len_unit = d.add("(LENGTH_UNIT()NAMED_UNIT(*)SI_UNIT(.MILLI.,.METRE.))".into());
    let ang_unit = d.add("(NAMED_UNIT(*)PLANE_ANGLE_UNIT()SI_UNIT($,.RADIAN.))".into());
    let sol_unit = d.add("(NAMED_UNIT(*)SOLID_ANGLE_UNIT()SI_UNIT($,.STERADIAN.))".into());
    let unc = d.add(format!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.E-07),#{},'distance_accuracy_value','confusion accuracy')",
        len_unit
    ));
    let ctx = d.add(format!(
        "(GEOMETRIC_REPRESENTATION_CONTEXT(3)GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{}))GLOBAL_UNIT_ASSIGNED_CONTEXT((#{},#{},#{}))REPRESENTATION_CONTEXT('',''))",
        unc, len_unit, ang_unit, sol_unit
    ));
    let rep = d.add(format!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION('{}',(#{},#{}),#{})",
        esc(name),
        world,
        brep,
        ctx
    ));
    let app =
        d.add("APPLICATION_CONTEXT('core data for automotive mechanical design processes')".into());
    d.add(format!(
        "APPLICATION_PROTOCOL_DEFINITION('international standard','automotive_design',2000,#{})",
        app
    ));
    let pctx = d.add(format!("PRODUCT_CONTEXT('',#{},'mechanical')", app));
    let prod = d.add(format!(
        "PRODUCT('{}','{}','',(#{}))",
        esc(name),
        esc(name),
        pctx
    ));
    d.add(format!(
        "PRODUCT_RELATED_PRODUCT_CATEGORY('part','',(#{}))",
        prod
    ));
    let pdf = d.add(format!("PRODUCT_DEFINITION_FORMATION('','',#{})", prod));
    let pdc = d.add(format!(
        "PRODUCT_DEFINITION_CONTEXT('part definition',#{},'design')",
        app
    ));
    let pd = d.add(format!("PRODUCT_DEFINITION('design','',#{},#{})", pdf, pdc));
    let pds = d.add(format!("PRODUCT_DEFINITION_SHAPE('','',#{})", pd));
    d.add(format!("SHAPE_DEFINITION_REPRESENTATION(#{},#{})", pds, rep));

    let mut s = String::with_capacity(d.lines.len() * 64);
    s.push_str("ISO-10303-21;\nHEADER;\n");
    s.push_str(&format!("FILE_DESCRIPTION(('{}'),'2;1');\n", esc(name)));
    s.push_str(&format!(
        "FILE_NAME('{}.step','{}',('gear-step'),(''),'gear-step','gear-step','');\n",
        esc(name),
        timestamp()
    ));
    s.push_str("FILE_SCHEMA(('AUTOMOTIVE_DESIGN { 1 0 10303 214 1 1 1 1 }'));\nENDSEC;\nDATA;\n");
    for (i, l) in d.lines.iter().enumerate() {
        s.push_str(&format!("#{}={};\n", i + 1, l));
    }
    s.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    s
}

fn esc(s: &str) -> String {
    s.replace('\\', "").replace('\'', "''")
}

/// Epoch seconds used in the STEP header. Hosts without a clock (wasm) set
/// this from the outside; -1 means "ask the platform".
static EPOCH: AtomicI64 = AtomicI64::new(-1);

/// Override the timestamp written into the STEP header.
pub fn set_epoch(secs: i64) {
    EPOCH.store(secs, Ordering::Relaxed);
}

fn now_secs() -> i64 {
    let o = EPOCH.load(Ordering::Relaxed);
    if o >= 0 {
        return o;
    }
    #[cfg(not(target_arch = "wasm32"))]
    {
        use std::time::{SystemTime, UNIX_EPOCH};
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs() as i64)
            .unwrap_or(0)
    }
    #[cfg(target_arch = "wasm32")]
    {
        0
    }
}

fn timestamp() -> String {
    let secs = now_secs();
    let days = secs / 86400;
    let rem = secs % 86400;
    let (mut y, mut dd) = (1970i64, days);
    loop {
        let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
        let dy = if leap { 366 } else { 365 };
        if dd < dy {
            break;
        }
        dd -= dy;
        y += 1;
    }
    let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    let ml = [31, if leap { 29 } else { 28 }, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    let mut m = 0;
    while dd >= ml[m] {
        dd -= ml[m];
        m += 1;
    }
    format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
        y,
        m + 1,
        dd + 1,
        rem / 3600,
        (rem % 3600) / 60,
        rem % 60
    )
}

impl Stats {
    /// Euler-Poincare check: V - E + F - R = 2*(S - G)
    pub fn check(&self, genus: i64) -> Result<String, String> {
        let chi = self.verts as i64 - self.edges as i64 + self.faces as i64 - self.rings as i64;
        let want = 2 - 2 * genus;
        if chi != want {
            return Err(format!("Euler characteristic {} != {}", chi, want));
        }
        Ok(format!(
            "closed shell: V={} E={} F={} (plane {}, cylinder {}, spline {}) chi={}",
            self.verts, self.edges, self.faces, self.planar, self.cylindrical, self.spline, chi
        ))
    }
}
