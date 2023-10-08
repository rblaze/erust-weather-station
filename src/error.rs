#[deny(unsafe_code)]
#[derive(Debug, Clone, Copy)]
pub enum Error {
    AlreadyTaken,
    Environment(async_scheduler::executor::EnvError),
}

impl From<async_scheduler::executor::EnvError> for Error {
    fn from(error: async_scheduler::executor::EnvError) -> Self {
        Error::Environment(error)
    }
}
