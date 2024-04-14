// Include the `items` module, which is generated from items.proto.
pub mod message {
    include!(concat!(env!("OUT_DIR"), "/_.rs"));
}