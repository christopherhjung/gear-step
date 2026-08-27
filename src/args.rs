//! Tiny dependency-free argument parser. Fills in the shared `Spec` plus the
//! handful of options that only make sense on the command line.

use gear_step::api::{Key, Spec};
use std::f64::consts::PI;

/// Command line only options.
pub struct Cli {
    pub out: String,
    pub svg: Option<String>,
    pub quiet: bool,
}

pub const HELP: &str = r#"gear-step - parametric involute gear generator with STEP (AP214) export

USAGE:
    gear-step -z <teeth> --module <mm> --width <mm> [options]

SIZE (pick exactly one)
    --module <mm>        normal module m_n                      [default 1.0]
    --pitch <mm>         circular pitch p = pi*m
    --dp <1/in>          diametral pitch (imperial)

MAIN
    -z, --teeth <n>      number of teeth                        [required]
    -w, --width <mm>     face width / thickness                 [required]
    -a, --pressure-angle <deg>   angle of attack                [default 20]
    --helix <deg>        helix angle (0 = spur gear)            [default 0]
    --hand <right|left>  helix hand                             [default right]
    --shift <x>          profile shift coefficient x            [default 0]
    --backlash <mm>      tooth thinning at the pitch circle     [default 0]

BASIC RACK (ISO 53 / DIN 867 profile A by default)
    --addendum <ha*>     addendum coefficient                   [default 1.0]
    --dedendum <hf*>     dedendum coefficient                   [default 1.25]
    --fillet <rho*>      cutter tip radius coefficient          [default 0.38]

BORE AND KEYWAY
    --bore <mm>          bore diameter, 0 = solid               [default 0]
    --keyway <spec>      none | auto | <b>x<t2>                 [default auto]
                         auto = DIN 6885-1 for the given bore;
                         t2 is the hub groove depth (roof at d/2 + t2)
    --flat <mm>          D profile bore: dimension across, i.e. from the
                         flat to the opposite bore wall (D shaft fit)
    --flat-depth <mm>    D profile given as cut depth instead
    --double-flat <mm>   double D: dimension between the two flats
    --bore-angle <deg>   keyway / flat position, 0 = +Y          [default 0]
                         (--keyway-angle is an alias)

LIGHTENING HOLES
    --holes <n>          number of holes on a bolt circle       [default 0]
    --hole-dia <mm>      hole diameter
    --hole-circle <mm>   bolt circle diameter

OUTPUT / QUALITY
    (tip land, root and bore are exact cylinders; flanks are cubic B-splines
     interpolating the points below, so a handful of points is plenty)
    -o, --out <file>     STEP output file            [default <name>.step]
    --name <str>         product name in the STEP file
    --svg <file>         also dump the transverse profile as SVG
    --phase <deg>        rotate the gear about its axis (for meshing pairs)
    --flank-seg <n>      involute interpolation points          [default 16]
    --fillet-seg <n>     root fillet interpolation points       [default 10]
    --layers <n>         axial sections of the helical loft     [auto]
    --pin <mm>           pin diameter for measurement over pins [auto 1.68*m]
    -q, --quiet          suppress the data sheet
    -h, --help           this text
"#;

fn num(v: &str, k: &str) -> Result<f64, String> {
    v.parse::<f64>().map_err(|_| format!("{}: '{}' is not a number", k, v))
}

pub fn parse(argv: &[String]) -> Result<Option<(Spec, Cli)>, String> {
    let mut a = Spec { z: 0, m_n: 1.0, width: 0.0, ..Spec::default() };
    let mut cli = Cli { out: String::new(), svg: None, quiet: false };
    let mut hand_left = false;
    let mut flat_depth: Option<f64> = None;
    let mut size_given = 0;
    let mut i = 0;
    while i < argv.len() {
        let raw = argv[i].clone();
        let (k, inline) = match raw.split_once('=') {
            Some((k, v)) => (k.to_string(), Some(v.to_string())),
            None => (raw.clone(), None),
        };
        let mut val = || -> Result<String, String> {
            if let Some(v) = inline.clone() {
                return Ok(v);
            }
            i += 1;
            argv.get(i).cloned().ok_or(format!("{} needs a value", k))
        };
        match k.as_str() {
            "-h" | "--help" => {
                print!("{}", HELP);
                return Ok(None);
            }
            "-q" | "--quiet" => cli.quiet = true,
            "-z" | "--teeth" => a.z = num(&val()?, "--teeth")? as u32,
            "--module" | "--mn" | "-m" => {
                a.m_n = num(&val()?, "--module")?;
                size_given += 1;
            }
            "--pitch" => {
                a.m_n = num(&val()?, "--pitch")? / PI;
                size_given += 1;
            }
            "--dp" => {
                a.m_n = 25.4 / num(&val()?, "--dp")?;
                size_given += 1;
            }
            "-w" | "--width" | "--thickness" => a.width = num(&val()?, "--width")?,
            "-a" | "--pressure-angle" | "--alpha" => {
                a.alpha_n_deg = num(&val()?, "--pressure-angle")?
            }
            "--helix" | "--beta" => a.beta_deg = num(&val()?, "--helix")?,
            "--hand" => hand_left = val()?.to_lowercase().starts_with('l'),
            "--shift" | "-x" => a.x = num(&val()?, "--shift")?,
            "--backlash" => a.backlash = num(&val()?, "--backlash")?,
            "--addendum" => a.ha = num(&val()?, "--addendum")?,
            "--dedendum" => a.hf = num(&val()?, "--dedendum")?,
            "--fillet" => a.rho = num(&val()?, "--fillet")?,
            "--bore" => a.bore = num(&val()?, "--bore")?,
            "--keyway" => {
                let v = val()?;
                a.key = match v.to_lowercase().as_str() {
                    "none" | "no" | "0" => Key::None,
                    "auto" | "din" | "din6885" => Key::Auto,
                    s => {
                        let (b, t) = s
                            .split_once('x')
                            .ok_or("--keyway expects none | auto | <b>x<t2>".to_string())?;
                        Key::Custom(num(b, "--keyway b")?, num(t, "--keyway t2")?)
                    }
                }
            }
            "--flat" => a.key = Key::DFlat(num(&val()?, "--flat")?),
            "--flat-depth" => flat_depth = Some(num(&val()?, "--flat-depth")?),
            "--double-flat" | "--dd" => a.key = Key::DoubleD(num(&val()?, "--double-flat")?),
            "--bore-angle" | "--keyway-angle" => {
                a.key_angle_deg = num(&val()?, "--bore-angle")?
            }
            "--holes" => a.holes = num(&val()?, "--holes")? as u32,
            "--hole-dia" => a.hole_dia = num(&val()?, "--hole-dia")?,
            "--hole-circle" => a.hole_circle = num(&val()?, "--hole-circle")?,
            "--phase" => a.phase_deg = num(&val()?, "--phase")?,
            "--flank-seg" => a.flank_seg = num(&val()?, "--flank-seg")? as usize,
            "--fillet-seg" => a.fillet_seg = num(&val()?, "--fillet-seg")? as usize,
            "--layers" => a.layers = num(&val()?, "--layers")? as usize,
            "--pin" => a.pin_dia = num(&val()?, "--pin")?,
            "--name" => a.name = val()?,
            "-o" | "--out" => cli.out = val()?,
            "--svg" => cli.svg = Some(val()?),
            other => return Err(format!("unknown option '{}' (try --help)", other)),
        }
        i += 1;
    }
    if size_given > 1 {
        return Err("use only one of --module / --pitch / --dp".into());
    }
    if let Some(t) = flat_depth {
        if t <= 0.0 || t >= a.bore {
            return Err("--flat-depth must be > 0 and < the bore diameter".into());
        }
        a.key = Key::DFlat(a.bore - t);
    }
    if hand_left {
        a.beta_deg = -a.beta_deg.abs();
    }
    a.normalise()?;
    if cli.out.is_empty() {
        cli.out = format!("{}.step", a.name);
    }
    Ok(Some((a, cli)))
}
