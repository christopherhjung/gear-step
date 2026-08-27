mod args;

use gear_step::api::{self, Row};

fn main() {
    let argv: Vec<String> = std::env::args().skip(1).collect();
    if argv.is_empty() {
        print!("{}", args::HELP);
        std::process::exit(1);
    }
    match run(&argv) {
        Ok(()) => {}
        Err(e) => {
            eprintln!("error: {}", e);
            std::process::exit(2);
        }
    }
}

fn run(argv: &[String]) -> Result<(), String> {
    let (spec, cli) = match args::parse(argv)? {
        Some(a) => a,
        None => return Ok(()),
    };

    let built = api::build(&spec)?;
    let (text, stats) = built.step();
    let check = stats.check(built.genus());
    std::fs::write(&cli.out, &text).map_err(|e| format!("cannot write {}: {}", cli.out, e))?;

    if let Some(path) = &cli.svg {
        std::fs::write(path, built.svg()).map_err(|e| format!("cannot write {}: {}", path, e))?;
    }

    if !cli.quiet {
        println!("\n{}", built.spec.name);
        for row in built.sheet(&stats, &check) {
            match row {
                Row::Section(s) => println!("  --- {:-<48}", format!("{} ", s)),
                Row::Kv(k, v) => println!("  {:<34}{}", k, v),
            }
        }
        println!("  {:<34}{}", "file", cli.out);
        println!();
    }
    for w in built.warnings() {
        eprintln!("warning: {}", w);
    }
    if let Err(e) = &check {
        eprintln!("warning: topology check failed: {}", e);
    }
    Ok(())
}
