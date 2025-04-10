fn main() {
    println!("cargo:rustc-link-arg-bins=-Tlinkall.x");
    // println!("cargo:rustc-link-arg-bins=-Trom_functions.x");

    // TODO: Scan source to find ENV vars
    println!("cargo:rerun-if-env-changed=DEFAULT_LED_AMOUNT");
}
