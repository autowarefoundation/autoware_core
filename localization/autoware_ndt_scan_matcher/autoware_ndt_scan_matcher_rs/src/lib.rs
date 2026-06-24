pub fn add(left: u64, right: u64) -> u64 {
    left.saturating_add(right)
}

/// C ABI entry point for the C++ side of `autoware_ndt_scan_matcher`.
///
/// Both arguments are passed by value and the return is a plain `u64`, so there
/// are no pointers or lifetimes crossing the boundary to validate.
#[unsafe(no_mangle)]
pub extern "C" fn autoware_ndt_scan_matcher_rs_add(left: u64, right: u64) -> u64 {
    add(left, right)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
