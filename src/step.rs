//! Minimal ISO 10303-21 (STEP) part 21 writer.
//! Emits an ADVANCED_BREP_SHAPE_REPRESENTATION / MANIFOLD_SOLID_BREP made of
//! planar ADVANCED_FACEs, with EDGE_CURVEs shared between adjacent faces.

use crate::solid::{newell, Brep};
use std::collections::HashMap;

struct Doc {
    lines: Vec<String>,
    cache: HashMap<String, usize>,
}

impl Doc {
    fn new() -> Self {
        Doc { lines: Vec::new(), cache: HashMap::new() }
    }
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
        let s = format!("CARTESIAN_POINT('',({},{},{}))", r(p[0]), r(p[1]), r(p[2]));
        self.shared(s)
    }
    fn dir(&mut self, d: [f64; 3]) -> usize {
        let s = format!("DIRECTION('',({},{},{}))", r(d[0]), r(d[1]), r(d[2]));
        self.shared(s)
    }
    fn axis(&mut self, o: [f64; 3], z: [f64; 3], x: [f64; 3]) -> usize {
        let (po, dz, dx) = (self.point(o), self.dir(z), self.dir(x));
        self.add(format!("AXIS2_PLACEMENT_3D('',#{},#{},#{})", po, dz, dx))
    }
}

/// STEP real literal: always carries a decimal point.
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

fn norm(v: [f64; 3]) -> [f64; 3] {
    let l = (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
    if l < 1e-15 {
        [1.0, 0.0, 0.0]
    } else {
        [v[0] / l, v[1] / l, v[2] / l]
    }
}

pub fn write(brep: &Brep, name: &str, author: &str) -> String {
    let mut d = Doc::new();

    // vertices
    let mut vp = Vec::with_capacity(brep.verts.len());
    for v in &brep.verts {
        let p = d.point(*v);
        vp.push(d.add(format!("VERTEX_POINT('',#{})", p)));
    }

    // shared edge curves, keyed by the sorted vertex pair
    let mut edges: HashMap<(usize, usize), usize> = HashMap::new();
    let mut oriented: HashMap<(usize, usize), usize> = HashMap::new();

    let mut faces: Vec<usize> = Vec::new();
    for f in &brep.faces {
        let n = newell(&brep.verts, &f[0]);
        let o = brep.verts[f[0][0]];
        // reference direction: first loop edge, orthogonalised against n
        let p1 = brep.verts[f[0][1]];
        let mut ex = [p1[0] - o[0], p1[1] - o[1], p1[2] - o[2]];
        let dot = ex[0] * n[0] + ex[1] * n[1] + ex[2] * n[2];
        for k in 0..3 {
            ex[k] -= dot * n[k];
        }
        let ex = norm(ex);
        let ax = d.axis(o, n, ex);
        let plane = d.add(format!("PLANE('',#{})", ax));

        let mut bounds: Vec<String> = Vec::new();
        for (li, lp) in f.iter().enumerate() {
            let mut oe: Vec<usize> = Vec::new();
            for i in 0..lp.len() {
                let (a, b) = (lp[i], lp[(i + 1) % lp.len()]);
                let key = (a.min(b), a.max(b));
                let ec = match edges.get(&key) {
                    Some(&e) => e,
                    None => {
                        let (pa, pb) = (brep.verts[key.0], brep.verts[key.1]);
                        let v = norm([pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]]);
                        let len = ((pb[0] - pa[0]).powi(2)
                            + (pb[1] - pa[1]).powi(2)
                            + (pb[2] - pa[2]).powi(2))
                        .sqrt();
                        let (po, dv) = (d.point(pa), d.dir(v));
                        let vec = d.add(format!("VECTOR('',#{},{})", dv, r(len)));
                        let line = d.add(format!("LINE('',#{},#{})", po, vec));
                        let e = d.add(format!(
                            "EDGE_CURVE('',#{},#{},#{},.T.)",
                            vp[key.0], vp[key.1], line
                        ));
                        edges.insert(key, e);
                        e
                    }
                };
                let fwd = a < b;
                let okey = (ec, fwd as usize);
                let oid = match oriented.get(&okey) {
                    Some(&x) => x,
                    None => {
                        let x = d.add(format!(
                            "ORIENTED_EDGE('',*,*,#{},{})",
                            ec,
                            if fwd { ".T." } else { ".F." }
                        ));
                        oriented.insert(okey, x);
                        x
                    }
                };
                oe.push(oid);
            }
            let refs: Vec<String> = oe.iter().map(|i| format!("#{}", i)).collect();
            let el = d.add(format!("EDGE_LOOP('',({}))", refs.join(",")));
            bounds.push(format!(
                "#{}",
                if li == 0 {
                    d.add(format!("FACE_OUTER_BOUND('',#{},.T.)", el))
                } else {
                    d.add(format!("FACE_BOUND('',#{},.T.)", el))
                }
            ));
        }
        faces.push(d.add(format!(
            "ADVANCED_FACE('',({}),#{},.T.)",
            bounds.join(","),
            plane
        )));
    }

    let shell_refs: Vec<String> = faces.iter().map(|i| format!("#{}", i)).collect();
    let shell = d.add(format!("CLOSED_SHELL('',({}))", shell_refs.join(",")));
    let brep_id = d.add(format!("MANIFOLD_SOLID_BREP('{}',#{})", esc(name), shell));
    let world = d.axis([0.0, 0.0, 0.0], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]);

    // ---- context / product boilerplate --------------------------------
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
        brep_id,
        ctx
    ));
    let app = d.add(
        "APPLICATION_CONTEXT('core data for automotive mechanical design processes')".into(),
    );
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
    d.add(format!(
        "SHAPE_DEFINITION_REPRESENTATION(#{},#{})",
        pds, rep
    ));

    // ---- serialise ----------------------------------------------------
    let mut s = String::with_capacity(d.lines.len() * 48);
    s.push_str("ISO-10303-21;\nHEADER;\n");
    s.push_str(&format!(
        "FILE_DESCRIPTION(('{}'),'2;1');\n",
        esc(name)
    ));
    s.push_str(&format!(
        "FILE_NAME('{}.step','{}',('{}'),(''),'gear-step','gear-step','');\n",
        esc(name),
        timestamp(),
        esc(author)
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

/// Coarse UTC timestamp from the system clock (no chrono dependency).
fn timestamp() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let days = secs / 86400;
    let rem = secs % 86400;
    let (mut y, mut d) = (1970i64, days);
    loop {
        let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
        let dy = if leap { 366 } else { 365 };
        if d < dy {
            break;
        }
        d -= dy;
        y += 1;
    }
    let leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
    let ml = [
        31,
        if leap { 29 } else { 28 },
        31,
        30,
        31,
        30,
        31,
        31,
        30,
        31,
        30,
        31,
    ];
    let mut m = 0;
    while d >= ml[m] {
        d -= ml[m];
        m += 1;
    }
    format!(
        "{:04}-{:02}-{:02}T{:02}:{:02}:{:02}",
        y,
        m + 1,
        d + 1,
        rem / 3600,
        (rem % 3600) / 60,
        rem % 60
    )
}
