use std::fs::{remove_file, File};

fn main() {
    // Touch a build marker to ensure Cargo sees a change every time
    // This is because there are macros dependent on UNKNOWN env vars,
    // so to avoid caching of those values this crate must be fully recompiled every time
    touch_file("src/build_marker.txt").unwrap();
    println!("cargo:rerun-if-changed=build_marker.txt");
}

fn touch_file(file_path: &str) -> std::io::Result<()> {
    // Simple way to touch a file that is cross-platfom
    // Step 1: Remove the file if it exists
    if let Err(e) = remove_file(file_path) {
        panic!("File not found or failed to remove: {}", e);
    }

    // Step 2: Create an empty file (this will recreate the file if it was deleted)
    File::create(file_path)?;

    println!("File has been recreated: {}", file_path);
    Ok(())
}
