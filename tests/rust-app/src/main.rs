use std::ffi::c_void;

// Define the opaque handle type Rust will use
type BoinkHandle = *mut c_void;

// 1. The FFI Block
// Tells Rust to link to a dynamic library named "physics_engine"
// NOTE: You must compile your C++ code to a DLL named e.g., 'physics_engine.dll'
// and place it where the Rust executable can find it.
#[link(name = "boink", kind = "dylib")]
unsafe extern "C" {
    // Declare the functions exactly as they appear in physics_capi.h
    fn create_engine() -> BoinkHandle;
    fn destroy_engine(handle: BoinkHandle);
}

// 2. Safe Rust Wrapper (Highly Recommended!)
// This struct makes the FFI safer and manages memory automatically.
struct BoinkEngineWrapper {
    handle: BoinkHandle,
}

impl BoinkEngineWrapper {
    // Safe constructor
    pub fn new() -> Self {
        unsafe {
            let handle = create_engine();
            Self { handle }
        }
    }
}

// 3. Implement the Drop trait for automatic cleanup
// This is critical for preventing memory leaks!
impl Drop for BoinkEngineWrapper {
    fn drop(&mut self) {
        unsafe {
            println!("Rust: Automatically destroying C++ engine instance...");
            destroy_engine(self.handle);
        }
    }
}

// 4. Main application logic
fn main() {
    // Create the safe Rust wrapper
    let _engine = BoinkEngineWrapper::new();

    // When 'engine' goes out of scope here, the Drop trait is called, 
    // which calls destroy_engine() in the C++ DLL.
    println!("Rust: Engine wrapper going out of scope.");
}
