fn main() -> anyhow::Result<()> {
    femtopb_build::compile_protos(
        &["protos/message.proto"],
        &["protos/"],
    )
}
