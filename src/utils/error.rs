use std::error::Error;

#[derive(Debug)]
struct TestError {
    source: Option<&'static dyn Error>,
}

impl std::fmt::Display for TestError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "This is a TestError!")
    }
}

impl Error for TestError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        self.source
    }
}