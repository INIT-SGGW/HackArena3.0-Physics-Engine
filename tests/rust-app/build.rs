use std::env;
use std::path::PathBuf;

fn main()
{
    let manifest_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap());
    let lib_path = manifest_dir.join("lib");

    println!("cargo:rustc-link-search=native={}", lib_path.display());
    println!("cargo:rustc-link-lib=dylib=boink");
}
