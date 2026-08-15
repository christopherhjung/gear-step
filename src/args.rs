//! Tiny dependency-free argument parser.

use std::f64::consts::PI;

pub struct Args {
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
    pub out: String,
    pub svg: Option<String>,
    pub quiet: bool,
}

pub enum Key {
    None,
    Auto,
    Custom(f64, f64),
    /// one flat, value = dimension across (flat to opposite wall)
    DFlat(f64),
    /// two flats, value = dimension between them
    DoubleD(f64),
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

pub fn parse(argv: &[String]) -> Result<Option<Args>, String> {
    let mut a = Args {
        z: 0,
        m_n: 1.0,
        alpha_n_deg: 20.0,
        beta_deg: 0.0,
        width: 0.0,
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
        out: String::new(),
        svg: None,
        quiet: false,
    };
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
            "-q" | "--quiet" => a.quiet = true,
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
            "-o" | "--out" => a.out = val()?,
            "--svg" => a.svg = Some(val()?),
            other => return Err(format!("unknown option '{}' (try --help)", other)),
        }
        i += 1;
    }
    if a.z < 3 {
        return Err("--teeth must be >= 3".into());
    }
    if a.width <= 0.0 {
        return Err("--width must be > 0".into());
    }
    if a.m_n <= 0.0 {
        return Err("module must be > 0".into());
    }
    if size_given > 1 {
        return Err("use only one of --module / --pitch / --dp".into());
    }
    if a.holes > 0 && (a.hole_dia <= 0.0 || a.hole_circle <= 0.0) {
        return Err("--holes needs --hole-dia and --hole-circle".into());
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
    if a.name.is_empty() {
        a.name = format!("gear_z{}_m{}", a.z, a.m_n);
    }
    if a.out.is_empty() {
        a.out = format!("{}.step", a.name);
    }
    if a.pin_dia <= 0.0 {
        a.pin_dia = (1.68 * a.m_n * 100.0).round() / 100.0;
    }
    Ok(Some(a))
}
