//! Involute gear geometry (external spur / helical), generated the way a rack
//! cutter (hob) would generate it: involute flank + true trochoidal root fillet.

use crate::profile::Seg;
use std::f64::consts::PI;

#[derive(Clone, Debug)]
pub struct GearParams {
    pub z: u32,
    /// normal module [mm]
    pub m_n: f64,
    /// normal pressure angle [rad]
    pub alpha_n: f64,
    /// helix angle [rad], 0 = spur. Sign carries the hand (+ = right hand).
    pub beta: f64,
    /// face width [mm]
    pub width: f64,
    /// profile shift coefficient x
    pub x: f64,
    /// addendum coefficient h_a* (1.0)
    pub ha_c: f64,
    /// dedendum coefficient h_f* (1.25)
    pub hf_c: f64,
    /// cutter tip radius coefficient rho* (0.38)
    pub rho_c: f64,
    /// circumferential tooth thinning at the pitch circle [mm] (backlash allowance)
    pub backlash: f64,
    pub flank_seg: usize,
    pub fillet_seg: usize,
}

pub struct Geom {
    pub m_t: f64,
    pub alpha_t: f64,
    pub r_p: f64,
    pub r_b: f64,
    pub r_a: f64,
    pub r_f: f64,
    /// transverse tooth thickness at the pitch circle
    pub s_t: f64,
    /// effective cutter tip radius after clamping
    pub rho: f64,
    /// half tooth angle at the root circle
    pub psi_root: f64,
    /// half tooth angle at the tip circle
    pub psi_tip: f64,
    /// form (true involute start) radius
    pub r_form: f64,
    /// half flank, root -> tip, as (radius, half angle from tooth centre line)
    pub half: Vec<(f64, f64)>,
    /// number of leading points of `half` that belong to the root fillet
    /// (the last of them is shared with the involute)
    pub split: usize,
    pub warnings: Vec<String>,
}

pub fn inv(a: f64) -> f64 {
    a.tan() - a
}

/// Newton inversion of the involute function.
pub fn inv_inverse(iv: f64) -> f64 {
    let mut a = (3.0 * iv.max(1e-12)).powf(1.0 / 3.0);
    for _ in 0..80 {
        let t = a.tan();
        let f = t - a - iv;
        let d = t * t;
        if d.abs() < 1e-14 {
            break;
        }
        let step = f / d;
        a -= step;
        if step.abs() < 1e-14 {
            break;
        }
    }
    a
}

impl GearParams {
    pub fn build(&self) -> Geom {
        let mut warnings = Vec::new();
        let z = self.z as f64;
        let m_n = self.m_n;
        let m_t = m_n / self.beta.cos();
        let alpha_t = (self.alpha_n.tan() / self.beta.cos()).atan();
        let r_p = z * m_t / 2.0;
        let r_b = r_p * alpha_t.cos();
        let mut r_a = r_p + m_n * (self.ha_c + self.x);
        let r_f = r_p - m_n * (self.hf_c - self.x);
        let s_t = PI * m_t / 2.0 + 2.0 * self.x * m_n * alpha_t.tan() - self.backlash;

        if r_f <= 0.0 {
            warnings.push("root circle <= 0 - parameters are not manufacturable".into());
        }
        let z_min = 2.0 * (self.ha_c - self.x) / (alpha_t.sin() * alpha_t.sin());
        if z < z_min - 1e-9 {
            warnings.push(format!(
                "undercut expected: z = {} < z_min = {:.1} (use a profile shift of {:.3} or more)",
                self.z,
                z_min,
                (self.ha_c - z * alpha_t.sin().powi(2) / 2.0).max(0.0)
            ));
        }

        // half tooth angle from the involute
        let psi_inv = |r: f64| -> f64 {
            let ar = (r_b / r).min(1.0).acos();
            s_t / (2.0 * r_p) + inv(alpha_t) - inv(ar)
        };

        // pointed tooth -> clamp tip circle
        let psi_a_min = 0.02 * PI / z;
        if psi_inv(r_a) < psi_a_min {
            let target = s_t / (2.0 * r_p) + inv(alpha_t) - psi_a_min;
            let r_new = r_b / inv_inverse(target.max(1e-9)).cos();
            warnings.push(format!(
                "pointed tooth: tip diameter reduced from {:.4} to {:.4} mm",
                2.0 * r_a,
                2.0 * r_new
            ));
            r_a = r_new;
        }

        // ---- rack cutter in the transverse plane -------------------------
        // pitch line at py = 0, +py away from the gear centre.
        let ta = alpha_t.tan();
        let a_off = PI * m_t / 4.0 + self.backlash / 2.0 - self.x * m_n * ta;
        let py_tip = self.x * m_n - self.hf_c * m_n; // cutter tip level
        let h_tip = a_off + py_tip * ta; // half width of the cutter tip
        let rho_max = h_tip * alpha_t.cos() / (1.0 - alpha_t.sin());
        let mut rho = self.rho_c * m_n;
        if rho > 0.98 * rho_max {
            rho = 0.98 * rho_max.max(0.0);
            warnings.push(format!(
                "root fillet radius limited to {:.4} mm by the cutter tip width",
                rho
            ));
        }
        let cy = py_tip + rho; // corner centre, rack frame
        let cx = a_off + cy * ta - rho / alpha_t.cos();

        // Envelope of the rounded cutter corner while the rack rolls on the
        // pitch circle. Parametrised by t = cx + r_p*phi, the rack abscissa of
        // the corner centre:
        //   P(t) = R(phi) * ( t*(1+k), r_p + cy*(1+k) ),  k = -sgn(cy)*rho/N,
        //   N = hypot(t, cy).
        // t = 0 is the tangency with the root circle (the corner touches the
        // cutter tip land); |t| = |cy|/tan(alpha_t) is the tangency with the
        // cutter flank, i.e. exactly the form point where the involute starts.
        let trochoid = |t: f64| -> (f64, f64) {
            let n = (t * t + cy * cy).sqrt().max(1e-12);
            let k = -cy.signum() * rho / n;
            let vx = t * (1.0 + k);
            let vy = r_p + cy * (1.0 + k);
            let phi = (t - cx) / r_p;
            let (s, c) = phi.sin_cos();
            let px = c * vx - s * vy;
            let py = s * vx + c * vy;
            // delta = angle off the tooth space centre line (+y)
            (px.hypot(py), PI / z - px.atan2(py))
        };

        let t_end = -cy.signum() * cy.abs() / alpha_t.tan();
        if cy.abs() < 1e-9 {
            warnings.push("degenerate root fillet (cutter corner sits on the pitch line)".into());
        }
        let n_fil = self.fillet_seg.max(4);
        let fillet: Vec<(f64, f64)> = (0..=n_fil)
            .map(|i| trochoid(t_end * i as f64 / n_fil as f64))
            .collect();
        let psi_root = fillet[0].1;
        if psi_root > PI / z {
            warnings.push("root fillets of neighbouring teeth overlap".into());
        }

        // involute samples, base circle -> tip
        let alpha_a = (r_b / r_a).min(1.0).acos();
        let n_inv = self.flank_seg.max(4);
        let involute: Vec<(f64, f64)> = (0..=n_inv)
            .map(|i| {
                let al = alpha_a * i as f64 / n_inv as f64;
                let r = r_b / al.cos();
                (r, psi_inv(r))
            })
            .collect();

        // ---- splice ---------------------------------------------------------
        // The rack flank only cuts from L >= 0 on, L measured along the line of
        // action from the base circle tangency. L_min > 0: the corner stops
        // cutting exactly where the flank starts, so the last trochoid point is
        // the form point and sits on the involute. L_min < 0: the flank cannot
        // reach that far down, the trochoid eats into the involute (undercut)
        // and the profile is the inner envelope of both curves.
        let py_t = cy - rho * alpha_t.sin();
        let l_min = r_p * alpha_t.sin() + py_t / alpha_t.sin();
        let sig = |t: f64| -> f64 {
            let (r, ps) = trochoid(t);
            if r <= r_b {
                -1.0
            } else {
                ps - psi_inv(r)
            }
        };
        let mut half: Vec<(f64, f64)> = fillet.clone();
        let r_form;
        if l_min >= 0.0 {
            r_form = fillet[n_fil].0;
            let gap = (fillet[n_fil].1 - psi_inv(r_form.max(r_b))) * r_form;
            if gap.abs() > 1e-6 {
                warnings.push(format!(
                    "fillet/involute mismatch of {:.2} um at the form circle",
                    gap * 1000.0
                ));
            }
        } else {
            let step = |i: usize| t_end * i as f64 / n_fil as f64;
            let last = (0..=n_fil).filter(|&i| sig(step(i)) < 0.0).last();
            match last {
                Some(i) if i < n_fil => {
                    let (mut lo, mut hi) = (step(i), step(i + 1));
                    for _ in 0..80 {
                        let mid = 0.5 * (lo + hi);
                        if sig(mid) < 0.0 {
                            lo = mid
                        } else {
                            hi = mid
                        }
                    }
                    let p = trochoid(hi);
                    r_form = p.0;
                    half.truncate(i + 1);
                    half.push(p);
                }
                _ => {
                    r_form = fillet[n_fil].0;
                    warnings.push(
                        "severe undercut: the flank is cut away, profile is approximate".into(),
                    );
                }
            }
            warnings.push(format!(
                "flank is undercut, usable involute starts at d = {:.4} mm",
                2.0 * r_form
            ));
        }
        let split = half.len();
        half.extend(involute.iter().filter(|q| q.0 > r_form).cloned());
        // drop anything past the tip, force an exact tip point, dedupe
        half.retain(|p| p.0 <= r_a + 1e-9);
        let psi_tip = psi_inv(r_a);
        half.push((r_a, psi_tip));
        half.dedup_by(|a, b| (a.0 - b.0).abs() < 1e-9 && (a.1 - b.1).abs() < 1e-12);

        Geom {
            m_t,
            alpha_t,
            r_p,
            r_b,
            r_a,
            r_f,
            s_t,
            rho,
            psi_root,
            psi_tip,
            r_form,
            half,
            split,
            warnings,
        }
    }
}

impl Geom {
    /// Closed CCW profile of the gear body as analytic segments:
    /// spline fillet, spline involute, cylindrical tip land and root.
    pub fn outline(&self, p: &GearParams, phase: f64) -> Vec<Seg> {
        let z = p.z as f64;
        let pitch = 2.0 * PI / z;
        let pol = |r: f64, th: f64| [r * th.cos(), r * th.sin()];
        let mut segs: Vec<Seg> = Vec::new();
        for k in 0..p.z {
            let th = phase + k as f64 * pitch;
            let fil: Vec<[f64; 2]> = self.half[..self.split]
                .iter()
                .map(|&(r, ps)| pol(r, th - ps))
                .collect();
            let inv: Vec<[f64; 2]> = self.half[self.split - 1..]
                .iter()
                .map(|&(r, ps)| pol(r, th - ps))
                .collect();
            let mirror = |v: &Vec<[f64; 2]>| -> Vec<[f64; 2]> {
                let (s, c) = (2.0 * th).sin_cos();
                v.iter()
                    .rev()
                    .map(|q| [c * q[0] + s * q[1], s * q[0] - c * q[1]])
                    .collect()
            };
            segs.push(Seg::Curve(fil.clone()));
            segs.push(Seg::Curve(inv.clone()));
            segs.push(Seg::Arc {
                c: [0.0, 0.0],
                r: self.r_a,
                a0: th - self.psi_tip,
                a1: th + self.psi_tip,
            });
            segs.push(Seg::Curve(mirror(&inv)));
            segs.push(Seg::Curve(mirror(&fil)));
            segs.push(Seg::Arc {
                c: [0.0, 0.0],
                r: self.r_f,
                a0: th + self.psi_root,
                a1: th + pitch - self.psi_root,
            });
        }
        crate::profile::tidy(segs)
    }

    /// Half tooth angle of the exact involute at radius r.
    pub fn psi_involute(&self, r: f64) -> f64 {
        let ar = (self.r_b / r).min(1.0).acos();
        self.s_t / (2.0 * self.r_p) + inv(self.alpha_t) - inv(ar)
    }

    /// Largest deviation of the fitted flank spline from the exact involute,
    /// measured between the interpolation points. This is the real geometric
    /// error of the exported flank surface.
    pub fn flank_deviation(&self, deg: usize) -> f64 {
        let pts: Vec<[f64; 3]> = self.half[self.split - 1..]
            .iter()
            .map(|&(r, ps)| [r * (-ps).cos(), r * (-ps).sin(), 0.0])
            .collect();
        if pts.len() < 3 {
            return 0.0;
        }
        let up = crate::nurbs::chord_params(&pts);
        let cv = crate::nurbs::interpolate(&pts, &up, deg);
        let mut worst: f64 = 0.0;
        for i in 0..=800 {
            let q = cv.eval(i as f64 / 800.0);
            let r = q[0].hypot(q[1]);
            if r < self.r_b + 1e-9 {
                continue;
            }
            worst = worst.max((q[1].atan2(q[0]) + self.psi_involute(r)).abs() * r);
        }
        worst
    }

    /// Base tangent (span) measurement over k teeth, DIN 3960.
    pub fn span(&self, p: &GearParams) -> (u32, f64) {
        let z = p.z as f64;
        let z_n = z / p.beta.cos().powi(3); // virtual (normal section) teeth
        let k = (z_n * p.alpha_n / PI + 0.5 + 2.0 * p.x * p.alpha_n.tan() / PI)
            .round()
            .max(2.0)
            .min(z - 1.0);
        let w = p.m_n * p.alpha_n.cos() * (PI * (k - 0.5) + z * inv(self.alpha_t))
            + 2.0 * p.x * p.m_n * p.alpha_n.sin();
        (k as u32, w)
    }

    /// Measurement over pins/balls of diameter d_m.
    pub fn over_pins(&self, p: &GearParams, d_m: f64) -> f64 {
        let z = p.z as f64;
        let invp = inv(self.alpha_t) + d_m / (z * p.m_n * p.alpha_n.cos())
            - PI / (2.0 * z)
            + 2.0 * p.x * p.alpha_n.tan() / z;
        let ap = inv_inverse(invp.max(1e-9));
        let base = z * self.m_t * self.alpha_t.cos() / ap.cos();
        if p.z % 2 == 0 {
            base + d_m
        } else {
            base * (PI / (2.0 * z)).cos() + d_m
        }
    }
}
