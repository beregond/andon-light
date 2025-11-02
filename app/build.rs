use vergen_gix::{BuildBuilder, Emitter, GixBuilder};

fn main() {
    linker_be_nice();
    // make sure linkall.x is the last linker script (otherwise might cause problems with flip-link)
    println!("cargo:rustc-link-arg=-Tlinkall.x");

    println!("cargo:rerun-if-changed=$DEFAULT_LEDS_AMOUNT");
    println!("cargo:rerun-if-changed=$MAX_SUPPORTED_LEDS");

    // This will automatically run `git describe` and set several env vars
    Emitter::default().emit().unwrap();
    let build = BuildBuilder::all_build().unwrap();
    let gitcl = GixBuilder::all_git().unwrap();

    Emitter::default()
        .add_instructions(&build)
        .unwrap()
        .add_instructions(&gitcl)
        .unwrap()
        .emit()
        .unwrap();
}

fn linker_be_nice() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() > 1 {
        let kind = &args[1];
        let what = &args[2];

        match kind.as_str() {
            "undefined-symbol" => match what.as_str() {
                "_defmt_timestamp" => {
                    eprintln!();
                    eprintln!("💡 `defmt` not found - make sure `defmt.x` is added as a linker script and you have included `use defmt_rtt as _;`");
                    eprintln!();
                }
                "_stack_start" => {
                    eprintln!();
                    eprintln!("💡 Is the linker script `linkall.x` missing?");
                    eprintln!();
                }
                _ => (),
            },
            // we don't have anything helpful for "missing-lib" yet
            _ => {
                std::process::exit(1);
            }
        }

        std::process::exit(0);
    }

    println!(
        "cargo:rustc-link-arg=--error-handling-script={}",
        std::env::current_exe().unwrap().display()
    );
}
