fn main() {
    println!("cargo:rerun-if-changed=../../protobufs/sim/world.proto");
    // Use prost's DEFAULT output directory (OUT_DIR under target/), not
    // a checked-in src/proto/ path. This matches the st3215 driver's
    // build.rs pattern and avoids git-tracked generated code.
    prost_build::Config::new()
        .compile_protos(
            &["../../protobufs/sim/world.proto"],
            &["../../protobufs"],
        )
        .expect("failed to compile world.proto");
}
