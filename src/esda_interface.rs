#[repr(u32)]
/// Convenient wrapper around the u32 used to denote the 'topic' being described by a control message
#[derive(Clone, Copy, Debug)]
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

/// The size of a single message in bytes
pub const MESSAGE_SIZE: usize = 8;

/// Struct wrapper around the u32 used to denote the 'topic' being described by a control message
#[derive(Clone, Copy, Debug)]
pub struct ESDAMessage {
    pub id: ESDAMessageID,
    pub data: f32,
}

impl ESDAMessage {
    /// Converts a slice of little endian bytes to an ESDAMessage
    pub fn from_le_bytes(bytes: &[u8]) -> Result<Self, &[u8]> {
        // Return an error immediately if the slice is the wrong length
        if bytes.len() != 8 {
            return Err(bytes);
        }

        // Convert the first four bytes of the slice into a u32
        let mut id: [u8; 4] = [0; 4];
        id.copy_from_slice(&bytes[0..=3]);
        let id: u32 = u32::from_le_bytes(id);

        let id: ESDAMessageID = match id {
            1 => ESDAMessageID::SetTargetVelLeft,
            2 => ESDAMessageID::SetTargetVelRight,
            3 => ESDAMessageID::CurrentVelLeft,
            4 => ESDAMessageID::CurrentVelRight,
            5 => ESDAMessageID::CurrentDSPLeft,
            6 => ESDAMessageID::CurrentDSPRight,
            7 => ESDAMessageID::SteerAmount,
            8 => ESDAMessageID::ESTOP,
            9 => ESDAMessageID::SetAutonomousMode,
            16 => ESDAMessageID::MCUState,
            17 => ESDAMessageID::MCUErrorState,
            // Error if the value is invalid
            _ => return Err(bytes),
        };

        // Store the last four bytes of the message in an array
        let mut value: [u8; 4] = [0; 4];
        value.copy_from_slice(&bytes[4..=7]);
        // Convert the last four bytes
        let value: f32 = f32::from_le_bytes(value);

        // Return the id and value packaged into an ESDAMessage
        Ok(ESDAMessage { id, data: value })
    }

    /// Converts a slice of big endian bytes to an ESDAMessage
    pub fn from_be_bytes(bytes: &[u8]) -> Result<Self, &[u8]> {
        // Return an error immediately if the slice is the wrong length
        if bytes.len() != 8 {
            return Err(bytes);
        }

        // Convert the first four bytes of the slice into a u32
        let mut id: [u8; 4] = [0; 4];
        id.copy_from_slice(&bytes[0..=3]);
        let id: u32 = u32::from_be_bytes(id);

        let id: ESDAMessageID = match id {
            1 => ESDAMessageID::SetTargetVelLeft,
            2 => ESDAMessageID::SetTargetVelRight,
            3 => ESDAMessageID::CurrentVelLeft,
            4 => ESDAMessageID::CurrentVelRight,
            5 => ESDAMessageID::CurrentDSPLeft,
            6 => ESDAMessageID::CurrentDSPRight,
            7 => ESDAMessageID::SteerAmount,
            8 => ESDAMessageID::ESTOP,
            9 => ESDAMessageID::SetAutonomousMode,
            16 => ESDAMessageID::MCUState,
            17 => ESDAMessageID::MCUErrorState,
            // Error if the value is invalid
            _ => return Err(bytes),
        };

        // Store the last four bytes of the message in an array
        let mut value: [u8; 4] = [0; 4];
        value.copy_from_slice(&bytes[4..=7]);
        // Convert the last four bytes
        let value: f32 = f32::from_be_bytes(value);

        // Return the id and value packaged into an ESDAMessage
        Ok(ESDAMessage { id, data: value })
    }

    /// Converts this ESDAMessage into byte form
    pub fn to_le_bytes(&self) -> [u8; 8] {
        let mut byte_form: [u8; 8] = [0; 8];

        // Convert the id to bytes
        // NOTE: DOUBLE CHECK CORRECT ENDIANNESS
        let id_quartet: [u8; 4] = (self.id as u32).to_le_bytes();
        // Copy the bytes to the output buffer
        byte_form[0..=3].copy_from_slice(&id_quartet);

        // Copy the bytes to the output buffer
        let value_quartet: [u8; 4] = self.data.to_le_bytes();
        byte_form[4..=7].copy_from_slice(&value_quartet);

        byte_form
    }

    /// Converts this ESDAMessage into big endian byte form
    pub fn to_be_bytes(&self) -> [u8; 8] {
        let mut byte_form: [u8; 8] = [0; 8];

        // Convert the id to bytes
        // NOTE: DOUBLE CHECK CORRECT ENDIANNESS
        let id_quartet: [u8; 4] = (self.id as u32).to_be_bytes();
        // Copy the bytes to the output buffer
        byte_form[0..=3].copy_from_slice(&id_quartet);

        // Copy the bytes to the output buffer
        let value_quartet: [u8; 4] = self.data.to_be_bytes();
        byte_form[4..=7].copy_from_slice(&value_quartet);

        byte_form
    }
}
