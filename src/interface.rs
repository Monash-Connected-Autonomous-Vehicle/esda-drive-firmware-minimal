#[repr(u32)]
pub enum ESDAMessageID {
    SetTargetVelLeft = 1,
    SetTargetVelRight = 2,
    CurrentVelLeft = 3,
    CurrentVelRight = 4,
    CurrentDSPLeft = 5,
    CurrentDSPRight = 6,
    SteerAmount = 7,
    MCUState = 16,
    MCUErrorState = 17,
    ESTOP = 8,
    SetAutonomousMode = 9,
}

pub struct ESDAMessage {
    id: ESDAMessageID,
    data: f32,
}
