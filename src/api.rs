//! Everything the CLI and the WebAssembly build have in common: one parameter
//! struct in, geometry + STEP + SVG + data sheet out. No I/O.

use crate::brep;
use crate::gear::{self, GearParams};
use crate::keyway;
use crate::mesh;
use crate::profile::{self, Seg};
use crate::svg;
use std::f64::consts::PI;

pub const D2R: f64 = PI / 180.0;

#[derive(Clone, Debug)]
pub enum Key {
    None,
    Auto,
    Custom(f64, f64),
    /// one flat, value = dimension across (flat to opposite wall)
    DFlat(f64),
    /// two flats, value = dimension between them
    DoubleD(f64),
}

#[derive(Clone, Debug)]
pub struct Spec {
    pub z: u32,
    pub m_n: f64,
    pub alpha_n_deg: f64,
    pub beta_deg: f64,
    pub width: f64,
    pub x: f64,
    pub ha: f64,
    pub hf: f64,
    pub rho: f64,
    pub backlash: f64,
    pub bore: f64,
    pub key: Key,
    pub key_angle_deg: f64,
    pub holes: u32,
    pub hole_dia: f64,
    pub hole_circle: f64,
    pub phase_deg: f64,
    pub flank_seg: usize,
    pub fillet_seg: usize,
    pub layers: usize,
    pub pin_dia: f64,
    pub name: String,
}

impl Default for Spec {
    fn default() -> Self {
        Spec {
            z: 24,
            m_n: 2.0,
            alpha_n_deg: 20.0,
            beta_deg: 0.0,
            width: 12.0,
            x: 0.0,
            ha: 1.0,
            hf: 1.25,
            rho: 0.38,
            backlash: 0.0,
            bore: 0.0,
            key: Key::Auto,
            key_angle_deg: 0.0,
            holes: 0,
            hole_dia: 0.0,
            hole_circle: 0.0,
            phase_deg: 0.0,
            flank_seg: 16,
            fillet_seg: 10,
            layers: 0,
            pin_dia: 0.0,
            name: String::new(),
        }
    }
}

impl Spec {
    /// Fill in the derived defaults (name, pin diameter) and reject nonsense.
    pub fn normalise(&mut self) -> Result<(), String> {
        if self.z < 3 {
            return Err("teeth must be >= 3".into());
        }
        if self.z > 400 {
            return Err("teeth must be <= 400".into());
        }
        if self.width <= 0.0 {
            return Err("width must be > 0".into());
        }
        if self.m_n <= 0.0 {
            return Err("module must be > 0".into());
        }
        if self.alpha_n_deg <= 0.0 || self.alpha_n_deg >= 45.0 {
            return Err("pressure angle must be between 0 and 45 deg".into());
        }
        if self.beta_deg.abs() >= 80.0 {
            return Err("helix angle must be below 80 deg".into());
        }
        if self.holes > 0 && (self.hole_dia <= 0.0 || self.hole_circle <= 0.0) {
            return Err("lightening holes need a hole diameter and a bolt circle".into());
        }
        if self.name.is_empty() {
            self.name = format!("gear_z{}_m{}", self.z, self.m_n);
        }
        if self.pin_dia <= 0.0 {
            self.pin_dia = (1.68 * self.m_n * 100.0).round() / 100.0;
        }
        self.flank_seg = self.flank_seg.clamp(4, 200);
        self.fillet_seg = self.fillet_seg.clamp(4, 200);
        Ok(())
    }

    pub fn params(&self) -> GearParams {
        GearParams {
            z: self.z,
            m_n: self.m_n,
            alpha_n: self.alpha_n_deg * D2R,
            beta: self.beta_deg * D2R,
            width: self.width,
            x: self.x,
            ha_c: self.ha,
            hf_c: self.hf,
            rho_c: self.rho,
            backlash: self.backlash,
            flank_seg: self.flank_seg,
            fillet_seg: self.fillet_seg,
        }
    }
}

/// A row of the data sheet.
pub enum Row {
    Section(String),
    Kv(String, String),
}

pub struct Built {
    pub spec: Spec,
    pub p: GearParams,
    pub g: gear::Geom,
    pub shape: keyway::Bore,
    pub outer: Vec<Seg>,
    pub holes: Vec<Vec<Seg>>,
    pub model: brep::Model,
    /// non fatal remarks made while resolving the bore
    pub notes: Vec<String>,
}

/// Resolve the bore shape, cut the contours and set up the sweep.
pub fn build(spec: &Spec) -> Result<Built, String> {
    let mut spec = spec.clone();
    spec.normalise()?;
    let p = spec.params();
    let g = p.build();
    let mut notes: Vec<String> = Vec::new();

    // ---- bore shape ---------------------------------------------------
    let shape = if spec.bore > 0.0 {
        match spec.key {
            Key::None => keyway::Bore::Round,
            Key::Custom(b, t) => keyway::Bore::Key(b, t),
            Key::DFlat(x) => keyway::Bore::DFlat(x),
            Key::DoubleD(x) => keyway::Bore::DoubleD(x),
            Key::Auto => match keyway::din6885(spec.bore) {
                Some((b, t)) => keyway::Bore::Key(b, t),
                None => {
                    notes.push(format!(
                        "no DIN 6885-1 entry for a {:.3} mm bore - plain round bore",
                        spec.bore
                    ));
                    keyway::Bore::Round
                }
            },
        }
    } else {
        keyway::Bore::Round
    };
    if let Some((h, depth)) = shape.flat(spec.bore) {
        if depth <= 0.0 {
            return Err("the flat does not cut into the bore - check the flat dimension".into());
        }
        if h <= -spec.bore / 2.0 + 1e-9 {
            return Err("the flat is deeper than the bore".into());
        }
        if h < 0.0 {
            notes.push("the flat cuts past the bore axis".into());
        }
    }

    // ---- contours -----------------------------------------------------
    let outer = g.outline(&p, spec.phase_deg * D2R);
    let mut holes: Vec<Vec<Seg>> = Vec::new();
    if spec.bore > 0.0 {
        if spec.bore / 2.0 >= g.r_f - 0.05 * p.m_n {
            return Err(format!(
                "bore ({:.3} mm) leaves no rim: root diameter is {:.3} mm",
                spec.bore,
                2.0 * g.r_f
            ));
        }
        if let keyway::Bore::Key(_, t2) = shape {
            if spec.bore / 2.0 + t2 >= g.r_f {
                return Err("keyway breaks through the tooth root".into());
            }
        }
        holes.push(keyway::bore_contour(spec.bore, shape, spec.key_angle_deg * D2R));
    }
    for i in 0..spec.holes {
        let th = 2.0 * PI * i as f64 / spec.holes as f64 + spec.phase_deg * D2R;
        let rc = spec.hole_circle / 2.0;
        let c = [rc * th.cos(), rc * th.sin()];
        let rh = spec.hole_dia / 2.0;
        if rc + rh >= g.r_f - 1e-9 {
            return Err("lightening holes break through the tooth root".into());
        }
        if spec.bore > 0.0 && rc - rh <= spec.bore / 2.0 + 1e-9 {
            return Err("lightening holes break into the bore".into());
        }
        if spec.holes > 1 {
            let pitch_gap = 2.0 * rc * (PI / spec.holes as f64).sin();
            if pitch_gap <= 2.0 * rh + 1e-9 {
                return Err("lightening holes overlap each other".into());
            }
        }
        holes.push(profile::circle(c, rh));
    }

    // ---- sweep ---------------------------------------------------------
    profile::check_closed(&outer)?;
    for h in &holes {
        profile::check_closed(h)?;
    }
    let twist = p.width * p.beta.tan() / g.r_p; // total rotation over the face width
    let layers = if spec.layers > 0 {
        spec.layers.clamp(4, 400)
    } else {
        (4.0 + (twist.abs() / (30.0 * D2R)).ceil()) as usize
    };
    let model = brep::Model {
        outer: outer.clone(),
        holes: holes.clone(),
        z0: 0.0,
        z1: p.width,
        twist,
        layers,
        deg: 3,
    };
    Ok(Built { spec, p, g, shape, outer, holes, model, notes })
}

impl Built {
    pub fn genus(&self) -> i64 {
        self.holes.len() as i64
    }

    pub fn step(&self) -> (String, brep::Stats) {
        brep::write(&self.model, &self.spec.name)
    }

    pub fn svg(&self) -> String {
        let poly = |c: &Vec<Seg>| -> Vec<[f64; 2]> {
            let mut v: Vec<[f64; 2]> = Vec::new();
            for s in c {
                for q in s.sample(160) {
                    if v
                        .last()
                        .map(|l: &[f64; 2]| (l[0] - q[0]).hypot(l[1] - q[1]) > 1e-9)
                        .unwrap_or(true)
                    {
                        v.push(q);
                    }
                }
            }
            v
        };
        let mut cs = vec![poly(&self.outer)];
        cs.extend(self.holes.iter().map(poly));
        svg::render(
            &cs,
            &[
                (self.g.r_p, "pitch"),
                (self.g.r_b, "base"),
                (self.g.r_f, "root"),
                (self.g.r_a, "tip"),
            ],
        )
    }

    /// Triangle mesh of the same solid, for display.
    pub fn mesh(&self) -> mesh::Mesh {
        mesh::tessellate(&self.model)
    }

    /// The transverse section as closed polylines: outer contour first, then
    /// the bore and the lightening holes.
    pub fn section(&self) -> Vec<Vec<[f64; 2]>> {
        let mut v = vec![mesh::sample_contour(&self.outer)];
        v.extend(self.holes.iter().map(|h| mesh::sample_contour(h)));
        v
    }

    pub fn warnings(&self) -> Vec<String> {
        let mut w = self.notes.clone();
        w.extend(self.g.warnings.iter().cloned());
        w
    }

    /// The data sheet, as rows.
    pub fn sheet(&self, stats: &brep::Stats, check: &Result<String, String>) -> Vec<Row> {
        let (p, g, a) = (&self.p, &self.g, &self.spec);
        let mut rows: Vec<Row> = Vec::new();
        let sec = |s: &str, rows: &mut Vec<Row>| rows.push(Row::Section(s.into()));
        let kv = |k: &str, v: String| Row::Kv(k.into(), v);
        let (k, w) = g.span(p);
        let m = g.over_pins(p, a.pin_dia);

        sec("macro geometry", &mut rows);
        rows.push(kv("teeth z", format!("{}", p.z)));
        rows.push(kv("normal module m_n", format!("{:.4} mm", p.m_n)));
        rows.push(kv("transverse module m_t", format!("{:.4} mm", g.m_t)));
        rows.push(kv("circular pitch p_t", format!("{:.4} mm", PI * g.m_t)));
        rows.push(kv(
            "pressure angle alpha_n / alpha_t",
            format!("{:.4} deg / {:.4} deg", p.alpha_n / D2R, g.alpha_t / D2R),
        ));
        rows.push(kv(
            "helix angle beta",
            format!(
                "{:.4} deg {}",
                p.beta.abs() / D2R,
                if p.beta.abs() < 1e-12 {
                    "(spur)"
                } else if p.beta > 0.0 {
                    "right hand"
                } else {
                    "left hand"
                }
            ),
        ));
        rows.push(kv("profile shift x", format!("{:.4}", p.x)));
        rows.push(kv("face width b", format!("{:.4} mm", p.width)));

        sec("diameters", &mut rows);
        rows.push(kv("pitch d", format!("{:.4} mm", 2.0 * g.r_p)));
        rows.push(kv("base d_b", format!("{:.4} mm", 2.0 * g.r_b)));
        rows.push(kv("tip d_a", format!("{:.4} mm", 2.0 * g.r_a)));
        rows.push(kv("root d_f", format!("{:.4} mm", 2.0 * g.r_f)));
        rows.push(kv(
            "form (usable involute) d_Ff",
            format!("{:.4} mm", 2.0 * g.r_form),
        ));
        rows.push(kv("root fillet radius", format!("{:.4} mm", g.rho)));

        sec("tooth / inspection", &mut rows);
        rows.push(kv("tooth thickness s_t at d", format!("{:.4} mm", g.s_t)));
        rows.push(kv(
            "normal tooth thickness s_n",
            format!("{:.4} mm", g.s_t * p.beta.cos()),
        ));
        rows.push(kv(
            "base tangent W over k teeth",
            format!("{:.4} mm (k = {})", w, k),
        ));
        rows.push(kv(
            "measurement over pins",
            format!("{:.4} mm (d_M = {:.2} mm)", m, a.pin_dia),
        ));
        rows.push(kv("tooth height h", format!("{:.4} mm", g.r_a - g.r_f)));
        let (a_w, alpha_w) = g.centre_distance(p);
        rows.push(kv(
            "centre distance with equal gear",
            format!("{:.4} mm", a_w),
        ));
        rows.push(kv(
            "operating pressure angle alpha_w",
            format!("{:.4} deg", alpha_w / D2R),
        ));
        rows.push(kv(
            "tip clearance with equal gear",
            format!("{:.4} mm", g.pair_clearance(p)),
        ));
        if p.beta.abs() > 1e-12 {
            rows.push(kv(
                "axial pitch p_x",
                format!("{:.4} mm", PI * p.m_n / p.beta.sin().abs()),
            ));
            rows.push(kv(
                "overlap ratio eps_beta",
                format!("{:.3}", p.width * p.beta.sin().abs() / (PI * p.m_n)),
            ));
        }

        if a.bore > 0.0 {
            sec("bore", &mut rows);
            rows.push(kv("bore d", format!("{:.4} mm (H7 suggested)", a.bore)));
            match self.shape {
                keyway::Bore::Key(b, t2) => {
                    rows.push(kv("keyway b x t2", format!("{:.2} x {:.2} mm", b, t2)));
                    rows.push(kv("dimension d + t2", format!("{:.3} mm", a.bore + t2)));
                    rows.push(kv(
                        "rim under the groove",
                        format!("{:.3} mm", g.r_f - a.bore / 2.0 - t2),
                    ));
                }
                keyway::Bore::DFlat(across) => {
                    let (h, depth) = self.shape.flat(a.bore).unwrap();
                    rows.push(kv("D profile across flat", format!("{:.3} mm", across)));
                    rows.push(kv(
                        "flat depth / chord",
                        format!(
                            "{:.3} mm / {:.3} mm",
                            depth,
                            2.0 * ((a.bore / 2.0).powi(2) - h * h).max(0.0).sqrt()
                        ),
                    ));
                }
                keyway::Bore::DoubleD(across) => {
                    let (h, depth) = self.shape.flat(a.bore).unwrap();
                    rows.push(kv("double D across flats", format!("{:.3} mm", across)));
                    rows.push(kv(
                        "flat depth each / chord",
                        format!(
                            "{:.3} mm / {:.3} mm",
                            depth,
                            2.0 * ((a.bore / 2.0).powi(2) - h * h).max(0.0).sqrt()
                        ),
                    ));
                }
                keyway::Bore::Round => rows.push(kv("bore profile", "plain round".into())),
            }
            rows.push(kv(
                "rim width (root to bore)",
                format!("{:.3} mm", g.r_f - a.bore / 2.0),
            ));
        }

        sec("output", &mut rows);
        rows.push(kv(
            "faces / edges / vertices",
            format!("{} / {} / {}", stats.faces, stats.edges, stats.verts),
        ));
        rows.push(kv(
            "surfaces",
            format!(
                "{} planar, {} cylindrical, {} B-spline",
                stats.planar, stats.cylindrical, stats.spline
            ),
        ));
        rows.push(kv("STEP entities", format!("{}", stats.entities)));
        rows.push(kv(
            "flank spline deviation",
            format!("{:.4} um max vs exact involute", g.flank_deviation(3) * 1000.0),
        ));
        rows.push(kv(
            "topology",
            match check {
                Ok(s) => s.clone(),
                Err(e) => format!("FAILED: {}", e),
            },
        ));
        rows
    }
}
