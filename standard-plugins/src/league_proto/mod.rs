pub mod firmware;
pub mod helpers;
pub mod vision;

pub mod league_protos {
    include!(concat!(env!("OUT_DIR"), "/league_protos.rs"));
}
